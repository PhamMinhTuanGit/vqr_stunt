#include "stand_wheel/trajectory_optimizer.hpp"

#include <casadi/casadi.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>

#include <Eigen/QR>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <vector>

namespace stand_wheel {
namespace pin = pinocchio;

namespace {

constexpr int kForceDimension = 3 * kNumLegs;

casadi::DM toCasadi(const Eigen::VectorXd& value) {
  return casadi::DM(std::vector<double>(value.data(),
                                        value.data() + value.size()));
}

Eigen::VectorXd toEigenVector(const casadi::DM& value) {
  const std::vector<double> storage = static_cast<std::vector<double>>(value);
  return Eigen::Map<const Eigen::VectorXd>(storage.data(), storage.size());
}

JointMapping findJoint(const PinocchioModel& robot, const std::string& name) {
  for (const JointMapping& joint : robot.jointOrdering()) {
    if (joint.name == name) {
      return joint;
    }
  }
  throw std::runtime_error("trajectory optimizer could not find joint: " +
                           name);
}

casadi::MX call(const casadi::Function& function,
                const casadi::MXVector& arguments) {
  return function(arguments).at(0);
}

double maxAbs(const Eigen::VectorXd& value) {
  return value.size() == 0 ? 0.0 : value.cwiseAbs().maxCoeff();
}

}  // namespace

StaticTrajectoryOptimizer::StaticTrajectoryOptimizer(PinocchioModel& robot)
    : robot_(robot),
      dynamics_(robot),
      wheel_contact_(robot),
      casadi_model_(robot, dynamics_) {}

void StaticTrajectoryOptimizer::validateConfig(
    const TrajectoryConfig& config) const {
  if (!std::isfinite(config.horizon) || config.horizon <= 0.0 ||
      config.intervals <= 0 || !std::isfinite(config.mu_roll) ||
      !std::isfinite(config.mu_lat) || config.mu_roll < 0.0 ||
      config.mu_lat < 0.0 || !std::isfinite(config.ipopt_tolerance) ||
      config.ipopt_tolerance <= 0.0 || config.ipopt_max_iterations <= 0) {
    throw std::invalid_argument("invalid static trajectory configuration");
  }
  const TrajectoryCostWeights& weights = config.weights;
  if (!std::isfinite(weights.configuration) ||
      !std::isfinite(weights.velocity) || !std::isfinite(weights.torque) ||
      !std::isfinite(weights.contact_force) ||
      !std::isfinite(weights.lateral_slip) || weights.configuration < 0.0 ||
      weights.velocity < 0.0 || weights.torque < 0.0 ||
      weights.contact_force < 0.0 || weights.lateral_slip < 0.0) {
    throw std::invalid_argument("trajectory cost weights must be nonnegative");
  }
}

void StaticTrajectoryOptimizer::computeStaticGuess(
    Eigen::VectorXd& tau_static,
    Eigen::VectorXd& lambda_static_world) {
  const Eigen::VectorXd q = robot_.nominalConfiguration();
  const Eigen::VectorXd v = Eigen::VectorXd::Zero(robot_.nv());
  const ActiveContacts contacts = {
      LegId::FL, LegId::FR, LegId::HL, LegId::HR};
  const FullDynamicsTerms terms = dynamics_.compute(q, v, contacts);
  const JointMapping root = findJoint(robot_, "root_joint");

  Eigen::MatrixXd vertical_selector = Eigen::MatrixXd::Zero(kForceDimension,
                                                             kNumLegs);
  for (int contact = 0; contact < kNumLegs; ++contact) {
    vertical_selector(3 * contact + 2, contact) = 1.0;
  }
  const Eigen::MatrixXd vertical_generalized =
      terms.contacts.jacobian_world.transpose() * vertical_selector;
  const Eigen::MatrixXd base_equilibrium =
      vertical_generalized.middleRows(root.idx_v, root.nv);
  const Eigen::VectorXd base_gravity =
      terms.nonlinear_effects.segment(root.idx_v, root.nv);
  const Eigen::VectorXd vertical_forces =
      base_equilibrium.completeOrthogonalDecomposition().solve(base_gravity);
  lambda_static_world = vertical_selector * vertical_forces;
  const Eigen::VectorXd contact_force =
      dynamics_.generalizedContactForce(terms.contacts, lambda_static_world);
  tau_static = dynamics_.selectionMatrix() *
               (terms.nonlinear_effects - contact_force);

  const Eigen::VectorXd residual =
      terms.nonlinear_effects - dynamics_.generalizedActuation(tau_static) -
      contact_force;
  if (!tau_static.allFinite() || !lambda_static_world.allFinite() ||
      (vertical_forces.array() <= 0.0).any() || residual.norm() > 1e-9) {
    throw std::runtime_error("failed to construct static-equilibrium guess");
  }
}

StaticTrajectorySolution StaticTrajectoryOptimizer::solve(
    const TrajectoryConfig& config) {
  validateConfig(config);
  if (!config.ipopt_plugin_directory.empty()) {
    casadi::GlobalOptions::setCasadiPath(config.ipopt_plugin_directory);
  }
  if (!casadi::has_nlpsol("ipopt")) {
    throw std::runtime_error(
        "CasADi IPOPT plugin is unavailable; check ipopt_plugin_directory");
  }

  const int nq = robot_.nq();
  const int nv = robot_.nv();
  const int nu = kNumActuatedJoints;
  const int intervals = config.intervals;
  const int nodes = intervals + 1;
  const double dt = config.timestep();
  const ActiveContacts contacts = {
      LegId::FL, LegId::FR, LegId::HL, LegId::HR};
  const CasadiFunctions symbolic = casadi_model_.build(contacts);
  const Eigen::VectorXd q_nominal = robot_.nominalConfiguration();
  const Eigen::VectorXd v_zero = Eigen::VectorXd::Zero(nv);
  Eigen::VectorXd tau_static;
  Eigen::VectorXd lambda_static;
  computeStaticGuess(tau_static, lambda_static);

  const int q_base = 0;
  const int v_base = q_base + nodes * nq;
  const int tau_base = v_base + nodes * nv;
  const int lambda_base = tau_base + intervals * nu;
  const int num_variables = lambda_base + intervals * kForceDimension;
  const auto qOffset = [=](int node) { return q_base + node * nq; };
  const auto vOffset = [=](int node) { return v_base + node * nv; };
  const auto tauOffset = [=](int interval) { return tau_base + interval * nu; };
  const auto lambdaOffset = [=](int interval) {
    return lambda_base + interval * kForceDimension;
  };

  const casadi::MX decision = casadi::MX::sym("trajectory", num_variables);
  const auto slice = [&](int offset, int size) {
    return decision(casadi::Slice(offset, offset + size));
  };

  casadi::MXVector constraints;
  std::vector<double> lower_constraints;
  std::vector<double> upper_constraints;
  int num_equalities = 0;
  int num_inequalities = 0;
  const auto appendConstraint = [&](const casadi::MX& expression,
                                    double lower, double upper) {
    constraints.push_back(expression);
    const int size = static_cast<int>(expression.numel());
    lower_constraints.insert(lower_constraints.end(), size, lower);
    upper_constraints.insert(upper_constraints.end(), size, upper);
    if (lower == upper) {
      num_equalities += size;
    } else {
      num_inequalities += size;
    }
  };

  casadi::MX objective = 0.0;
  const casadi::MX q_nominal_mx = casadi::MX(toCasadi(q_nominal));
  const casadi::MX lambda_static_mx = casadi::MX(toCasadi(lambda_static));
  const JointMapping root = findJoint(robot_, "root_joint");

  for (int node = 0; node < nodes; ++node) {
    const casadi::MX q_node = slice(qOffset(node), nq);
    const casadi::MX v_node = slice(vOffset(node), nv);

    const casadi::MX quaternion =
        q_node(casadi::Slice(root.idx_q + 3, root.idx_q + 7));
    appendConstraint(casadi::MX::dot(quaternion, quaternion) - 1.0,
                     0.0, 0.0);
    for (const WheelContactMapping& wheel : robot_.wheels()) {
      const int index = wheel.wheel_joint.idx_q;
      const casadi::MX wheel_pair =
          q_node(casadi::Slice(index, index + wheel.wheel_joint.nq));
      appendConstraint(casadi::MX::dot(wheel_pair, wheel_pair) - 1.0,
                       0.0, 0.0);
    }

    const casadi::MX positions = call(symbolic.contact_positions, {q_node});
    casadi::MXVector contact_heights;
    for (int contact = 0; contact < kNumLegs; ++contact) {
      contact_heights.push_back(positions(3 * contact + 2));
    }
    appendConstraint(casadi::MX::vertcat(contact_heights), 0.0, 0.0);
    appendConstraint(call(symbolic.hard_constraint_residual,
                          {q_node, v_node}),
                     0.0, 0.0);

    if (node < intervals) {
      const casadi::MX tau_node = slice(tauOffset(node), nu);
      const casadi::MX lambda_node =
          slice(lambdaOffset(node), kForceDimension);
      const casadi::MX acceleration = call(
          symbolic.forward_dynamics,
          {q_node, v_node, tau_node, lambda_node});
      const casadi::MX velocity_prediction = v_node + dt * acceleration;
      const casadi::MX q_prediction = call(
          symbolic.integrate, {q_node, dt * velocity_prediction});
      appendConstraint(slice(vOffset(node + 1), nv) - velocity_prediction,
                       0.0, 0.0);
      appendConstraint(call(symbolic.difference,
                            {q_prediction, slice(qOffset(node + 1), nq)}),
                       0.0, 0.0);

      const casadi::MX lambda_wheel = call(
          symbolic.force_wheel_from_world, {q_node, lambda_node});
      appendConstraint(call(symbolic.friction_margins,
                            {lambda_wheel, casadi::MX(config.mu_roll),
                             casadi::MX(config.mu_lat)}),
                       0.0, std::numeric_limits<double>::infinity());

      const casadi::MX configuration_error = call(
          symbolic.difference, {q_nominal_mx, q_node});
      const casadi::MX force_error = lambda_node - lambda_static_mx;
      const casadi::MX lateral_slip =
          call(symbolic.lateral_slip, {q_node, v_node});
      objective +=
          config.weights.configuration *
              casadi::MX::dot(configuration_error, configuration_error) +
          config.weights.velocity * casadi::MX::dot(v_node, v_node) +
          config.weights.torque * casadi::MX::dot(tau_node, tau_node) +
          config.weights.contact_force *
              casadi::MX::dot(force_error, force_error) +
          config.weights.lateral_slip *
              casadi::MX::dot(lateral_slip, lateral_slip);
    }
  }

  appendConstraint(call(symbolic.difference,
                        {q_nominal_mx, slice(qOffset(0), nq)}),
                   0.0, 0.0);
  appendConstraint(slice(vOffset(0), nv), 0.0, 0.0);
  appendConstraint(call(symbolic.difference,
                        {q_nominal_mx, slice(qOffset(intervals), nq)}),
                   0.0, 0.0);
  appendConstraint(slice(vOffset(intervals), nv), 0.0, 0.0);

  const casadi::MX all_constraints = casadi::MX::vertcat(constraints);
  std::vector<double> lower_variables(
      num_variables, -std::numeric_limits<double>::infinity());
  std::vector<double> upper_variables(
      num_variables, std::numeric_limits<double>::infinity());
  std::vector<double> initial_guess(num_variables, 0.0);
  const pin::Model& model = robot_.model();

  for (int node = 0; node < nodes; ++node) {
    std::copy(q_nominal.data(), q_nominal.data() + nq,
              initial_guess.begin() + qOffset(node));
    for (const JointMapping& joint : robot_.legJoints()) {
      const double lower = model.lowerPositionLimit[joint.idx_q];
      const double upper = model.upperPositionLimit[joint.idx_q];
      if (std::isfinite(lower) && std::isfinite(upper) && lower <= upper) {
        lower_variables[qOffset(node) + joint.idx_q] = lower;
        upper_variables[qOffset(node) + joint.idx_q] = upper;
      }
    }
    for (int velocity = 0; velocity < nv; ++velocity) {
      const double limit = model.velocityLimit[velocity];
      if (std::isfinite(limit) && limit > 0.0 && limit < 1e10) {
        lower_variables[vOffset(node) + velocity] = -limit;
        upper_variables[vOffset(node) + velocity] = limit;
      }
    }
  }
  for (int interval = 0; interval < intervals; ++interval) {
    std::copy(tau_static.data(), tau_static.data() + nu,
              initial_guess.begin() + tauOffset(interval));
    std::copy(lambda_static.data(),
              lambda_static.data() + kForceDimension,
              initial_guess.begin() + lambdaOffset(interval));
    for (int actuator = 0; actuator < nu; ++actuator) {
      const JointMapping& joint = dynamics_.actuatorOrdering()[actuator];
      const double limit = model.effortLimit[joint.idx_v];
      if (std::isfinite(limit) && limit > 0.0 && limit < 1e10) {
        lower_variables[tauOffset(interval) + actuator] = -limit;
        upper_variables[tauOffset(interval) + actuator] = limit;
      }
    }
  }

  casadi::MXDict nlp;
  nlp["x"] = decision;
  nlp["f"] = objective;
  nlp["g"] = all_constraints;
  casadi::Dict options;
  options["print_time"] = false;
  options["ipopt.print_level"] = config.ipopt_print_level;
  options["ipopt.tol"] = config.ipopt_tolerance;
  options["ipopt.constr_viol_tol"] = config.ipopt_tolerance;
  options["ipopt.acceptable_tol"] = 100.0 * config.ipopt_tolerance;
  options["ipopt.acceptable_iter"] = 5;
  options["ipopt.max_iter"] = config.ipopt_max_iterations;
  // options["ipopt.hessian_approximation"] = "limited-memory";

  const casadi::Function solver =
      casadi::nlpsol("stand_wheel_static_solver", "ipopt", nlp, options);
  casadi::DMDict arguments;
  arguments["x0"] = casadi::DM(initial_guess);
  arguments["lbx"] = casadi::DM(lower_variables);
  arguments["ubx"] = casadi::DM(upper_variables);
  arguments["lbg"] = casadi::DM(lower_constraints);
  arguments["ubg"] = casadi::DM(upper_constraints);

  const auto solve_start = std::chrono::steady_clock::now();
  const casadi::DMDict result = solver(arguments);
  const auto solve_end = std::chrono::steady_clock::now();
  const casadi::Dict statistics = solver.stats();
  const Eigen::VectorXd decision_value = toEigenVector(result.at("x"));

  StaticTrajectorySolution solution;
  solution.success = static_cast<bool>(statistics.at("success"));
  solution.solver_status =
      static_cast<std::string>(statistics.at("return_status"));
  if (statistics.count("iter_count") != 0) {
    solution.iterations = static_cast<int>(statistics.at("iter_count"));
  }
  solution.objective = result.at("f").scalar();
  solution.num_variables = num_variables;
  solution.num_equalities = num_equalities;
  solution.num_inequalities = num_inequalities;
  solution.solve_time_seconds =
      std::chrono::duration<double>(solve_end - solve_start).count();
  solution.q.resize(nq, nodes);
  solution.v.resize(nv, nodes);
  solution.tau.resize(nu, intervals);
  solution.lambda_world.resize(kForceDimension, intervals);
  for (int node = 0; node < nodes; ++node) {
    solution.q.col(node) = decision_value.segment(qOffset(node), nq);
    solution.v.col(node) = decision_value.segment(vOffset(node), nv);
  }
  for (int interval = 0; interval < intervals; ++interval) {
    solution.tau.col(interval) =
        decision_value.segment(tauOffset(interval), nu);
    solution.lambda_world.col(interval) =
        decision_value.segment(lambdaOffset(interval), kForceDimension);
  }
  solution.lambda_static_world = lambda_static;
  solution.tau_static = tau_static;
  solution.validation = validateSolution(solution, config);
  return solution;
}

TrajectoryValidation StaticTrajectoryOptimizer::validateSolution(
    const StaticTrajectorySolution& solution,
    const TrajectoryConfig& config) {
  const int intervals = config.intervals;
  const int nodes = intervals + 1;
  const double dt = config.timestep();
  const Eigen::VectorXd q_nominal = robot_.nominalConfiguration();
  const ActiveContacts contacts = {
      LegId::FL, LegId::FR, LegId::HL, LegId::HR};
  const JointMapping root = findJoint(robot_, "root_joint");
  TrajectoryValidation validation;
  validation.min_normal_force = std::numeric_limits<double>::infinity();

  for (int node = 0; node < nodes; ++node) {
    const Eigen::VectorXd q = solution.q.col(node);
    const Eigen::VectorXd v = solution.v.col(node);
    const ModelSnapshot snapshot = robot_.evaluate(q);
    const SkidSteerConstraintData skid =
        wheel_contact_.skidSteer(q, v, contacts);
    validation.max_velocity = std::max(validation.max_velocity, maxAbs(v));
    validation.max_hard_constraint_residual =
        std::max(validation.max_hard_constraint_residual,
                 maxAbs(skid.hard_residual));
    validation.max_lateral_slip =
        std::max(validation.max_lateral_slip, maxAbs(skid.lateral_slip));
    validation.max_quaternion_norm_error = std::max(
        validation.max_quaternion_norm_error,
        std::abs(q.segment<4>(root.idx_q + 3).norm() - 1.0));
    for (int contact = 0; contact < kNumLegs; ++contact) {
      validation.max_contact_height_error = std::max(
          validation.max_contact_height_error,
          std::abs(snapshot.contacts[contact].position_world.z()));
      const WheelContactMapping& wheel = robot_.wheels()[contact];
      validation.max_continuous_wheel_norm_error = std::max(
          validation.max_continuous_wheel_norm_error,
          std::abs(q.segment(wheel.wheel_joint.idx_q,
                             wheel.wheel_joint.nq).norm() - 1.0));
    }

    if (node < intervals) {
      const Eigen::VectorXd acceleration = dynamics_.forwardDynamics(
          q, v, solution.tau.col(node), contacts,
          solution.lambda_world.col(node));
      const Eigen::VectorXd velocity_prediction = v + dt * acceleration;
      const Eigen::VectorXd q_prediction =
          pin::integrate(robot_.model(), q, dt * velocity_prediction);
      validation.max_velocity_dynamics_defect = std::max(
          validation.max_velocity_dynamics_defect,
          maxAbs(solution.v.col(node + 1) - velocity_prediction));
      validation.max_manifold_difference_defect = std::max(
          validation.max_manifold_difference_defect,
          maxAbs(pin::difference(robot_.model(), q_prediction,
                                 solution.q.col(node + 1))));
      validation.max_torque = std::max(
          validation.max_torque, maxAbs(solution.tau.col(node)));
      const Eigen::VectorXd lambda_wheel =
          wheel_contact_.worldForcesToWheel(
              q, contacts, solution.lambda_world.col(node));
      const FrictionFeasibility friction = WheelContactModel::checkFriction(
          lambda_wheel, FrictionConfig(config.mu_roll, config.mu_lat));
      validation.max_friction_violation = std::max(
          validation.max_friction_violation, maxAbs(friction.violations));
      for (int contact = 0; contact < kNumLegs; ++contact) {
        validation.min_normal_force = std::min(
            validation.min_normal_force, lambda_wheel[3 * contact + 2]);
      }
    }
  }

  validation.initial_boundary_error = std::max(
      maxAbs(pin::difference(robot_.model(), q_nominal, solution.q.col(0))),
      maxAbs(solution.v.col(0)));
  validation.terminal_boundary_error = std::max(
      maxAbs(pin::difference(robot_.model(), q_nominal,
                             solution.q.col(intervals))),
      maxAbs(solution.v.col(intervals)));
  return validation;
}

void StaticTrajectoryOptimizer::exportCsv(
    const StaticTrajectorySolution& solution,
    const TrajectoryConfig& config, const std::string& path) {
  if (path.empty()) {
    throw std::invalid_argument("CSV output path must not be empty");
  }
  std::ofstream output(path);
  if (!output) {
    throw std::runtime_error("could not open trajectory CSV: " + path);
  }
  output.precision(17);
  output << "time";
  for (int i = 0; i < robot_.nq(); ++i) output << ",q" << i;
  for (int i = 0; i < robot_.nv(); ++i) output << ",v" << i;
  for (int i = 0; i < kNumActuatedJoints; ++i) output << ",tau" << i;
  for (int i = 0; i < kForceDimension; ++i) output << ",lambda_world" << i;
  output << ",com_x,com_y,com_z";
  for (int leg = 0; leg < kNumLegs; ++leg) {
    output << ',' << legLabel(static_cast<LegId>(leg)) << "_contact_x"
           << ',' << legLabel(static_cast<LegId>(leg)) << "_contact_y"
           << ',' << legLabel(static_cast<LegId>(leg)) << "_contact_z";
  }
  for (int leg = 0; leg < kNumLegs; ++leg) {
    output << ',' << legLabel(static_cast<LegId>(leg)) << "_lateral_slip";
  }
  output << '\n';

  const ActiveContacts contacts = {
      LegId::FL, LegId::FR, LegId::HL, LegId::HR};
  for (int node = 0; node <= config.intervals; ++node) {
    const Eigen::VectorXd q = solution.q.col(node);
    const Eigen::VectorXd v = solution.v.col(node);
    const ModelSnapshot snapshot = robot_.evaluate(q);
    const Eigen::VectorXd slip = wheel_contact_.lateralSlip(q, v, contacts);
    output << node * config.timestep();
    for (int i = 0; i < q.size(); ++i) output << ',' << q[i];
    for (int i = 0; i < v.size(); ++i) output << ',' << v[i];
    for (int i = 0; i < kNumActuatedJoints; ++i) {
      output << ',';
      if (node < config.intervals) output << solution.tau(i, node);
    }
    for (int i = 0; i < kForceDimension; ++i) {
      output << ',';
      if (node < config.intervals) output << solution.lambda_world(i, node);
    }
    output << ',' << snapshot.center_of_mass_world.x()
           << ',' << snapshot.center_of_mass_world.y()
           << ',' << snapshot.center_of_mass_world.z();
    for (const ContactKinematics& contact : snapshot.contacts) {
      output << ',' << contact.position_world.x()
             << ',' << contact.position_world.y()
             << ',' << contact.position_world.z();
    }
    for (int i = 0; i < slip.size(); ++i) output << ',' << slip[i];
    output << '\n';
  }
}

}  // namespace stand_wheel
