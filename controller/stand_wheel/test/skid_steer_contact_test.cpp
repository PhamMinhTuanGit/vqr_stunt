#include "stand_wheel/casadi_model.hpp"
#include "stand_wheel/wheel_contact.hpp"

#include <pinocchio/algorithm/joint-configuration.hpp>

#include <Eigen/Core>
#include <Eigen/SVD>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

#ifndef STAND_WHEEL_DEFAULT_URDF_PATH
#define STAND_WHEEL_DEFAULT_URDF_PATH \
  "vqr_description/vqr_urdf/urdf/VQRWheel.urdf"
#endif

namespace sw = stand_wheel;
namespace pin = pinocchio;

namespace {

bool check(bool condition, const std::string& message) {
  std::cout << (condition ? "[PASS] " : "[FAIL] ") << message << '\n';
  return condition;
}

sw::JointMapping findJoint(const sw::PinocchioModel& robot,
                           const std::string& name) {
  for (const sw::JointMapping& joint : robot.jointOrdering()) {
    if (joint.name == name) {
      return joint;
    }
  }
  throw std::runtime_error("skid-steer test could not find joint: " + name);
}

casadi::DM toCasadi(const Eigen::VectorXd& value) {
  return casadi::DM(std::vector<double>(value.data(),
                                        value.data() + value.size()));
}

Eigen::MatrixXd toEigen(const casadi::DM& value) {
  const std::vector<double> storage = static_cast<std::vector<double>>(value);
  return Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>>(
      storage.data(), value.rows(), value.columns());
}

Eigen::VectorXd evaluateVector(const casadi::Function& function,
                               const casadi::DMVector& arguments) {
  const Eigen::MatrixXd result = toEigen(function(arguments).at(0));
  return Eigen::Map<const Eigen::VectorXd>(result.data(), result.size());
}

Eigen::MatrixXd evaluateMatrix(const casadi::Function& function,
                               const casadi::DMVector& arguments) {
  return toEigen(function(arguments).at(0));
}

Eigen::MatrixXd selectColumns(const Eigen::MatrixXd& matrix,
                              const std::vector<int>& indices) {
  Eigen::MatrixXd selected(matrix.rows(), indices.size());
  for (std::size_t i = 0; i < indices.size(); ++i) {
    selected.col(static_cast<int>(i)) = matrix.col(indices[i]);
  }
  return selected;
}

Eigen::VectorXd randomVector(std::mt19937& generator, int size, double scale) {
  std::uniform_real_distribution<double> distribution(-scale, scale);
  Eigen::VectorXd result(size);
  for (int i = 0; i < size; ++i) {
    result[i] = distribution(generator);
  }
  return result;
}

double maxError(const Eigen::MatrixXd& lhs, const Eigen::MatrixXd& rhs) {
  return (lhs - rhs).cwiseAbs().maxCoeff();
}

}  // namespace

int main(int argc, char** argv) {
  const std::string urdf =
      argc > 1 ? argv[1] : STAND_WHEEL_DEFAULT_URDF_PATH;
  sw::PinocchioModel robot({urdf});
  sw::FullDynamics dynamics(robot);
  sw::WheelContactModel wheel_contact(robot);
  const Eigen::VectorXd q_nominal = robot.nominalConfiguration();
  const sw::ActiveContacts all_contacts = {
      sw::LegId::FL, sw::LegId::FR, sw::LegId::HL, sw::LegId::HR};
  const sw::JointMapping root = findJoint(robot, "root_joint");
  if (root.nv != 6) {
    throw std::runtime_error("skid-steer test requires a six-DoF free flyer");
  }
  const int base_yaw = root.idx_v + 5;

  std::vector<int> wheel_indices;
  for (const sw::WheelContactMapping& wheel : robot.wheels()) {
    wheel_indices.push_back(wheel.wheel_joint.idx_v);
  }

  bool passed = true;
  constexpr double yaw_rate = 0.5;
  Eigen::VectorXd yaw_velocity = Eigen::VectorXd::Zero(robot.nv());
  yaw_velocity[base_yaw] = yaw_rate;
  const Eigen::MatrixXd yaw_hard_matrix =
      wheel_contact.hardConstraintMatrix(q_nominal, all_contacts);
  const Eigen::MatrixXd wheel_columns =
      selectColumns(yaw_hard_matrix, wheel_indices);
  const Eigen::JacobiSVD<Eigen::MatrixXd> yaw_svd(
      wheel_columns, Eigen::ComputeThinU | Eigen::ComputeThinV);
  const Eigen::VectorXd yaw_wheel_rates =
      yaw_svd.solve(-yaw_hard_matrix * yaw_velocity);
  for (std::size_t i = 0; i < wheel_indices.size(); ++i) {
    yaw_velocity[wheel_indices[i]] = yaw_wheel_rates[static_cast<int>(i)];
  }
  const sw::SkidSteerConstraintData yaw_skid =
      wheel_contact.skidSteer(q_nominal, yaw_velocity, all_contacts);
  const sw::RollingConstraintData yaw_strict =
      wheel_contact.evaluate(q_nominal, yaw_velocity, all_contacts);
  Eigen::VectorXd step5a_slip(sw::kNumLegs);
  step5a_slip << -0.1223511656, -0.1223511656,
      0.1260488344, 0.1260488344;
  Eigen::VectorXd legacy_lateral(sw::kNumLegs);
  for (int contact = 0; contact < sw::kNumLegs; ++contact) {
    legacy_lateral[contact] = yaw_strict.residual[3 * contact + 1];
  }
  const double yaw_hard_residual = yaw_skid.hard_residual.norm();
  const double yaw_slip_step5a_error =
      maxError(yaw_skid.lateral_slip, step5a_slip);
  const double yaw_slip_legacy_error =
      maxError(yaw_skid.lateral_slip, legacy_lateral);
  std::cout << std::scientific << std::setprecision(9);
  std::cout << "YAW SKID wheel_rates=" << yaw_wheel_rates.transpose()
            << " hard_norm=" << yaw_hard_residual
            << " lateral_slip=" << yaw_skid.lateral_slip.transpose()
            << " step5a_error=" << yaw_slip_step5a_error << '\n';
  passed &= check(yaw_hard_matrix.rows() == 2 * sw::kNumLegs &&
                      yaw_hard_matrix.cols() == robot.nv() &&
                      yaw_hard_residual < 1e-12 &&
                      yaw_skid.lateral_slip.norm() > 0.2 &&
                      yaw_slip_step5a_error < 1e-10 &&
                      yaw_slip_legacy_error < 1e-14,
                  "fixed-leg pure yaw satisfies hard rows while retaining Step 5A slip");

  const sw::SkidSteerConstraintData nominal_frames = wheel_contact.skidSteer(
      q_nominal, Eigen::VectorXd::Zero(robot.nv()), all_contacts);
  constexpr double straight_speed = 0.35;
  Eigen::VectorXd straight_velocity = Eigen::VectorXd::Zero(robot.nv());
  straight_velocity.segment<3>(root.idx_v) =
      straight_speed * nominal_frames.frames.front().rolling_world;
  for (const sw::WheelContactMapping& wheel : robot.wheels()) {
    straight_velocity[wheel.wheel_joint.idx_v] =
        straight_speed / robot.wheelRadius();
  }
  const sw::SkidSteerConstraintData straight =
      wheel_contact.skidSteer(q_nominal, straight_velocity, all_contacts);
  std::cout << "STRAIGHT hard_norm=" << straight.hard_residual.norm()
            << " slip_norm=" << straight.lateral_slip.norm() << '\n';
  passed &= check(straight.hard_residual.norm() < 1e-12 &&
                      straight.lateral_slip.norm() < 1e-12,
                  "pure straight rolling has zero hard residual and lateral slip");

  constexpr double lateral_speed = 0.19;
  Eigen::VectorXd lateral_velocity = Eigen::VectorXd::Zero(robot.nv());
  lateral_velocity.segment<3>(root.idx_v) =
      lateral_speed * nominal_frames.frames.front().lateral_world;
  const sw::SkidSteerConstraintData lateral =
      wheel_contact.skidSteer(q_nominal, lateral_velocity, all_contacts);
  const Eigen::VectorXd expected_lateral =
      Eigen::VectorXd::Constant(sw::kNumLegs, lateral_speed);
  std::cout << "LATERAL SKID hard_norm=" << lateral.hard_residual.norm()
            << " slip=" << lateral.lateral_slip.transpose() << '\n';
  passed &= check(lateral.hard_residual.norm() < 1e-12 &&
                      maxError(lateral.lateral_slip, expected_lateral) < 1e-12,
                  "lateral skid remains hard-feasible and reports signed slip");

  const sw::ActiveContacts subset = {
      sw::LegId::HR, sw::LegId::FL, sw::LegId::HL};
  std::mt19937 generator(20260913);
  const Eigen::VectorXd q = pin::integrate(
      robot.model(), q_nominal, randomVector(generator, robot.nv(), 0.08));
  const Eigen::VectorXd v = randomVector(generator, robot.nv(), 0.4);
  const sw::SkidSteerConstraintData numerical =
      wheel_contact.skidSteer(q, v, subset);
  passed &= check(numerical.frames.size() == subset.size() &&
                      numerical.frames[0].leg == sw::LegId::HR &&
                      numerical.frames[1].leg == sw::LegId::FL &&
                      numerical.frames[2].leg == sw::LegId::HL,
                  "noncanonical active-contact order is preserved");

  const Eigen::VectorXd lambda_wheel =
      randomVector(generator, 3 * static_cast<int>(subset.size()), 25.0);
  const Eigen::MatrixXd force_transform =
      wheel_contact.forceTransformWorldFromWheel(q, subset);
  const Eigen::VectorXd lambda_world =
      wheel_contact.wheelForcesToWorld(q, subset, lambda_wheel);
  const Eigen::VectorXd lambda_round_trip =
      wheel_contact.worldForcesToWheel(q, subset, lambda_world);
  const double transform_orthogonality_error = maxError(
      force_transform.transpose() * force_transform,
      Eigen::MatrixXd::Identity(force_transform.cols(),
                                force_transform.cols()));
  const double force_round_trip_error =
      maxError(lambda_round_trip, lambda_wheel);
  const sw::ContactStack force_contacts =
      dynamics.contactStack(q, v, subset);
  const Eigen::VectorXd generalized_contact_force =
      dynamics.generalizedContactForce(force_contacts, lambda_world);
  const double force_dynamics_error = maxError(
      generalized_contact_force,
      force_contacts.jacobian_world.transpose() * lambda_world);
  std::cout << "FORCE TRANSFORM orthogonality="
            << transform_orthogonality_error
            << " round_trip=" << force_round_trip_error
            << " J_force_consistency=" << force_dynamics_error << '\n';
  passed &= check(transform_orthogonality_error < 1e-12 &&
                      force_round_trip_error < 1e-12 &&
                      force_dynamics_error < 1e-14,
                  "wheel/world force transform round-trips and dynamics keeps J_force");

  const sw::FrictionConfig friction(0.8, 0.5);
  Eigen::Vector3d valid_force(4.0, 2.0, 10.0);
  Eigen::Vector3d negative_normal(0.0, 0.0, -1.0);
  Eigen::Vector3d excessive_roll(9.0, 0.0, 10.0);
  Eigen::Vector3d excessive_lateral(0.0, 6.0, 10.0);
  const sw::FrictionFeasibility valid =
      sw::WheelContactModel::checkFriction(valid_force, friction);
  const sw::FrictionFeasibility negative =
      sw::WheelContactModel::checkFriction(negative_normal, friction);
  const sw::FrictionFeasibility roll_violation =
      sw::WheelContactModel::checkFriction(excessive_roll, friction);
  const sw::FrictionFeasibility lateral_violation =
      sw::WheelContactModel::checkFriction(excessive_lateral, friction);
  std::cout << "FRICTION valid_margins=" << valid.margins.transpose()
            << " negative_normal_margins=" << negative.margins.transpose()
            << " roll_margins=" << roll_violation.margins.transpose()
            << " lateral_margins=" << lateral_violation.margins.transpose()
            << '\n';
  passed &= check(valid.feasible && !negative.feasible &&
                      !roll_violation.feasible && !lateral_violation.feasible &&
                      valid.violations.norm() == 0.0 &&
                      negative.margins[0] < 0.0 &&
                      roll_violation.margins[1] < 0.0 &&
                      lateral_violation.margins[3] < 0.0 &&
                      std::abs(negative.violations[0] - 1.0) < 1e-14 &&
                      std::abs(roll_violation.violations[1] - 1.0) < 1e-14 &&
                      std::abs(lateral_violation.violations[3] - 1.0) < 1e-14,
                  "friction check identifies normal, rolling, and lateral violations");

  const sw::FrictionFeasibility numerical_friction =
      sw::WheelContactModel::checkFriction(lambda_wheel, friction);
  sw::CasadiModel symbolic_builder(robot, dynamics);
  const sw::CasadiFunctions symbolic = symbolic_builder.build(subset);
  const Eigen::MatrixXd symbolic_hard =
      evaluateMatrix(symbolic.hard_constraint_matrix, {toCasadi(q)});
  const Eigen::VectorXd symbolic_hard_residual = evaluateVector(
      symbolic.hard_constraint_residual, {toCasadi(q), toCasadi(v)});
  const Eigen::VectorXd symbolic_lateral = evaluateVector(
      symbolic.lateral_slip, {toCasadi(q), toCasadi(v)});
  const Eigen::VectorXd symbolic_world_force = evaluateVector(
      symbolic.force_world_from_wheel,
      {toCasadi(q), toCasadi(lambda_wheel)});
  const Eigen::VectorXd symbolic_wheel_force = evaluateVector(
      symbolic.force_wheel_from_world,
      {toCasadi(q), toCasadi(lambda_world)});
  const Eigen::VectorXd symbolic_friction = evaluateVector(
      symbolic.friction_margins,
      {toCasadi(lambda_wheel), casadi::DM(friction.mu_roll),
       casadi::DM(friction.mu_lat)});

  const double hard_matrix_parity =
      maxError(symbolic_hard, numerical.hard_matrix);
  const double hard_residual_parity =
      maxError(symbolic_hard_residual, numerical.hard_residual);
  const double lateral_parity =
      maxError(symbolic_lateral, numerical.lateral_slip);
  const double world_force_parity =
      maxError(symbolic_world_force, lambda_world);
  const double wheel_force_parity =
      maxError(symbolic_wheel_force, lambda_wheel);
  const double friction_parity =
      maxError(symbolic_friction, numerical_friction.margins);
  std::cout << "CASADI PARITY A_hard=" << hard_matrix_parity
            << " hard_residual=" << hard_residual_parity
            << " lateral=" << lateral_parity
            << " force_forward=" << world_force_parity
            << " force_inverse=" << wheel_force_parity
            << " friction=" << friction_parity << '\n';
  passed &= check(symbolic_hard.rows() == 2 * static_cast<int>(subset.size()) &&
                      symbolic_hard.cols() == robot.nv() &&
                      symbolic_lateral.size() ==
                          static_cast<int>(subset.size()) &&
                      symbolic_friction.size() ==
                          5 * static_cast<int>(subset.size()) &&
                      hard_matrix_parity < 1e-12 &&
                      hard_residual_parity < 1e-12 &&
                      lateral_parity < 1e-12 &&
                      world_force_parity < 1e-12 &&
                      wheel_force_parity < 1e-12 &&
                      friction_parity < 1e-12,
                  "CasADi skid-steer APIs match numerical implementation");

  std::cout << (passed ? "SKID STEER CONTACT TEST PASSED"
                       : "SKID STEER CONTACT TEST FAILED")
            << '\n';
  return passed ? 0 : 1;
}
