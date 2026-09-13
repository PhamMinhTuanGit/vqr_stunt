#include "stand_wheel/casadi_model.hpp"
#include "stand_wheel/wheel_contact.hpp"

#include <casadi/config.h>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/config.hpp>

#include <Eigen/Core>

#include <algorithm>
#include <array>
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

Eigen::VectorXd randomVector(std::mt19937& generator, int size, double scale) {
  std::uniform_real_distribution<double> distribution(-scale, scale);
  Eigen::VectorXd result(size);
  for (int i = 0; i < size; ++i) {
    result[i] = distribution(generator);
  }
  return result;
}

sw::JointMapping findJoint(const sw::PinocchioModel& robot,
                           const std::string& name) {
  for (const sw::JointMapping& joint : robot.jointOrdering()) {
    if (joint.name == name) {
      return joint;
    }
  }
  throw std::runtime_error("test could not find joint: " + name);
}

double updateMax(double current, const Eigen::MatrixXd& lhs,
                 const Eigen::MatrixXd& rhs) {
  return std::max(current, (lhs - rhs).cwiseAbs().maxCoeff());
}

bool functionShape(const casadi::Function& function, int input,
                   int input_rows, int output, int output_rows,
                   int output_cols) {
  return function.size1_in(input) == input_rows &&
         function.size2_in(input) == 1 &&
         function.size1_out(output) == output_rows &&
         function.size2_out(output) == output_cols;
}

}  // namespace

int main(int argc, char** argv) {
  const std::string urdf =
      argc > 1 ? argv[1] : STAND_WHEEL_DEFAULT_URDF_PATH;
  sw::PinocchioModel robot({urdf});
  sw::FullDynamics dynamics(robot);
  sw::WheelContactModel wheel_contact(robot);
  const sw::ActiveContacts active_contacts = {
      sw::LegId::HR, sw::LegId::FL, sw::LegId::HL};
  sw::CasadiModel symbolic_builder(robot, dynamics);

  std::cout << "Pinocchio " << PINOCCHIO_VERSION << ", CasADi "
            << CASADI_VERSION_STRING << '\n';
  std::cout << "Building SX functions for contacts HR,FL,HL...\n";
  const sw::CasadiFunctions symbolic =
      symbolic_builder.build(active_contacts);
  bool passed = true;
  const int nc_rows = 3 * static_cast<int>(active_contacts.size());

  passed &= check(functionShape(symbolic.mass_matrix, 0, robot.nq(), 0,
                                robot.nv(), robot.nv()),
                  "M function shape is q[27] -> 22x22");
  passed &= check(functionShape(symbolic.forward_dynamics, 0, robot.nq(), 0,
                                robot.nv(), 1) &&
                      symbolic.forward_dynamics.size1_in(1) == robot.nv() &&
                      symbolic.forward_dynamics.size1_in(2) ==
                          sw::kNumActuatedJoints &&
                      symbolic.forward_dynamics.size1_in(3) == nc_rows,
                  "vdot inputs are q[27], v[22], tau[16], lambda[9]");
  passed &= check(functionShape(symbolic.contact_jacobian, 0, robot.nq(), 0,
                                nc_rows, robot.nv()),
                  "active force Jacobian shape is 9x22");
  passed &= check(functionShape(symbolic.rolling_matrix, 0, robot.nq(), 0,
                                nc_rows, robot.nv()),
                  "active rolling matrix shape is 9x22");
  passed &= check(functionShape(symbolic.integrate, 0, robot.nq(), 0,
                                robot.nq(), 1) &&
                      symbolic.integrate.size1_in(1) == robot.nv(),
                  "integrate inputs preserve nq=27 and nv=22");

  const Eigen::VectorXd q_nominal = robot.nominalConfiguration();
  std::mt19937 generator(20260913);
  constexpr int kSamples = 5;
  double mass_error = 0.0;
  double nonlinear_error = 0.0;
  double actuation_error = 0.0;
  double contact_force_error = 0.0;
  double vdot_error = 0.0;
  double com_error = 0.0;
  double wheel_center_error = 0.0;
  double contact_position_error = 0.0;
  double contact_jacobian_error = 0.0;
  double support_line_error = 0.0;
  double rolling_matrix_error = 0.0;
  double rolling_residual_error = 0.0;
  double integrate_error = 0.0;
  double difference_error = 0.0;
  double quaternion_norm_error = 0.0;
  double continuous_norm_error = 0.0;

  Eigen::VectorXd derivative_q;
  Eigen::VectorXd derivative_v;
  Eigen::VectorXd derivative_tau;
  Eigen::VectorXd derivative_lambda;

  for (int sample = 0; sample < kSamples; ++sample) {
    const Eigen::VectorXd q = pin::integrate(
        robot.model(), q_nominal, randomVector(generator, robot.nv(), 0.08));
    const Eigen::VectorXd v = randomVector(generator, robot.nv(), 0.35);
    const Eigen::VectorXd tau =
        randomVector(generator, sw::kNumActuatedJoints, 18.0);
    Eigen::VectorXd lambda = randomVector(generator, nc_rows, 45.0);
    for (int contact = 0; contact < nc_rows / 3; ++contact) {
      lambda[3 * contact + 2] += 80.0;
    }
    const Eigen::VectorXd dv = randomVector(generator, robot.nv(), 0.04);

    const sw::ModelSnapshot snapshot = robot.evaluate(q);
    const sw::FullDynamicsTerms numerical_terms =
        dynamics.compute(q, v, active_contacts);
    const sw::RollingConstraintData numerical_rolling =
        wheel_contact.evaluate(q, v, active_contacts);

    mass_error = updateMax(
        mass_error,
        evaluateMatrix(symbolic.mass_matrix, {toCasadi(q)}),
        snapshot.mass_matrix);
    nonlinear_error = updateMax(
        nonlinear_error,
        evaluateVector(symbolic.nonlinear_effects,
                       {toCasadi(q), toCasadi(v)}),
        numerical_terms.nonlinear_effects);
    actuation_error = updateMax(
        actuation_error,
        evaluateVector(symbolic.generalized_actuation, {toCasadi(tau)}),
        dynamics.generalizedActuation(tau));
    contact_force_error = updateMax(
        contact_force_error,
        evaluateVector(symbolic.generalized_contact_force,
                       {toCasadi(q), toCasadi(lambda)}),
        dynamics.generalizedContactForce(numerical_terms.contacts, lambda));
    vdot_error = updateMax(
        vdot_error,
        evaluateVector(symbolic.forward_dynamics,
                       {toCasadi(q), toCasadi(v), toCasadi(tau),
                        toCasadi(lambda)}),
        dynamics.forwardDynamics(q, v, tau, active_contacts, lambda));
    com_error = updateMax(
        com_error,
        evaluateVector(symbolic.center_of_mass, {toCasadi(q)}),
        snapshot.center_of_mass_world);

    Eigen::VectorXd centers(12);
    Eigen::VectorXd contacts(12);
    Eigen::MatrixXd jacobians(12, robot.nv());
    for (int leg = 0; leg < sw::kNumLegs; ++leg) {
      centers.segment<3>(3 * leg) = snapshot.contacts[leg].wheel_center_world;
      contacts.segment<3>(3 * leg) = snapshot.contacts[leg].position_world;
      jacobians.middleRows<3>(3 * leg) = snapshot.contacts[leg].jacobian;
    }
    wheel_center_error = updateMax(
        wheel_center_error,
        evaluateVector(symbolic.wheel_centers, {toCasadi(q)}), centers);
    contact_position_error = updateMax(
        contact_position_error,
        evaluateVector(symbolic.contact_positions, {toCasadi(q)}), contacts);
    contact_jacobian_error = updateMax(
        contact_jacobian_error,
        evaluateMatrix(symbolic.contact_jacobians_all, {toCasadi(q)}),
        jacobians);
    contact_jacobian_error = updateMax(
        contact_jacobian_error,
        evaluateMatrix(symbolic.contact_jacobian, {toCasadi(q)}),
        numerical_terms.contacts.jacobian_world);

    const Eigen::Vector3d normal = robot.groundNormalWorld();
    const Eigen::Vector3d support_start =
        snapshot.contacts[static_cast<int>(sw::LegId::FL)].position_world;
    const Eigen::Vector3d support_end =
        snapshot.contacts[static_cast<int>(sw::LegId::HR)].position_world;
    Eigen::Vector3d support_direction = support_end - support_start;
    support_direction -= normal.dot(support_direction) * normal;
    support_direction.normalize();
    Eigen::Vector3d com_horizontal =
        snapshot.center_of_mass_world - support_start;
    com_horizontal -= normal.dot(com_horizontal) * normal;
    const Eigen::Vector3d com_perpendicular =
        com_horizontal - support_direction.dot(com_horizontal) *
                             support_direction;
    Eigen::VectorXd support_geometry(13);
    support_geometry << support_start, support_end, support_direction,
        com_perpendicular, com_perpendicular.squaredNorm();
    support_line_error = updateMax(
        support_line_error,
        evaluateVector(symbolic.support_line, {toCasadi(q)}),
        support_geometry);

    rolling_matrix_error = updateMax(
        rolling_matrix_error,
        evaluateMatrix(symbolic.rolling_matrix, {toCasadi(q)}),
        numerical_rolling.matrix);
    rolling_residual_error = updateMax(
        rolling_residual_error,
        evaluateVector(symbolic.rolling_residual,
                       {toCasadi(q), toCasadi(v)}),
        numerical_rolling.residual);

    const Eigen::VectorXd q_next_symbolic = evaluateVector(
        symbolic.integrate, {toCasadi(q), toCasadi(dv)});
    const Eigen::VectorXd q_next_numerical =
        pin::integrate(robot.model(), q, dv);
    integrate_error = updateMax(integrate_error, q_next_symbolic,
                                q_next_numerical);
    difference_error = updateMax(
        difference_error,
        evaluateVector(symbolic.difference,
                       {toCasadi(q), toCasadi(q_next_numerical)}),
        pin::difference(robot.model(), q, q_next_numerical));
    const sw::JointMapping root = findJoint(robot, "root_joint");
    quaternion_norm_error =
        std::max(quaternion_norm_error,
                 std::abs(q_next_symbolic.segment<4>(root.idx_q + 3).norm() -
                          1.0));
    for (const sw::WheelContactMapping& wheel : robot.wheels()) {
      continuous_norm_error =
          std::max(continuous_norm_error,
                   std::abs(q_next_symbolic
                                .segment(wheel.wheel_joint.idx_q,
                                         wheel.wheel_joint.nq)
                                .norm() -
                            1.0));
    }

    if (sample == 0) {
      derivative_q = q;
      derivative_v = v;
      derivative_tau = tau;
      derivative_lambda = lambda;
    }
  }

  std::cout << std::scientific << std::setprecision(6);
  std::cout << "DYNAMICS PARITY M=" << mass_error
            << " h=" << nonlinear_error
            << " St_tau=" << actuation_error
            << " Jt_lambda=" << contact_force_error
            << " vdot=" << vdot_error << '\n';
  std::cout << "KINEMATICS PARITY com=" << com_error
            << " centers=" << wheel_center_error
            << " contacts=" << contact_position_error
            << " jacobians=" << contact_jacobian_error
            << " support_line=" << support_line_error << '\n';
  std::cout << "ROLLING PARITY A=" << rolling_matrix_error
            << " residual=" << rolling_residual_error << '\n';
  std::cout << "INTEGRATE PARITY error=" << integrate_error
            << " difference_error=" << difference_error
            << " quaternion_norm_error=" << quaternion_norm_error
            << " continuous_pair_norm_error=" << continuous_norm_error << '\n';
  passed &= check(mass_error < 1e-10 && nonlinear_error < 1e-10 &&
                      actuation_error < 1e-12 &&
                      contact_force_error < 1e-10 && vdot_error < 1e-9,
                  "symbolic dynamics match numerical Pinocchio/LDLT dynamics");
  passed &= check(com_error < 1e-10 && wheel_center_error < 1e-10 &&
                      contact_position_error < 1e-10 &&
                      contact_jacobian_error < 1e-9 &&
                      support_line_error < 1e-10,
                  "symbolic kinematics and support line match numerical model");
  passed &= check(rolling_matrix_error < 1e-9 &&
                      rolling_residual_error < 1e-9,
                  "symbolic rolling constraints match Step 3");
  passed &= check(integrate_error < 1e-11 && difference_error < 1e-11 &&
                      quaternion_norm_error < 1e-12 &&
                      continuous_norm_error < 1e-12,
                  "symbolic manifold integration matches Pinocchio and stays normalized");

  const Eigen::VectorXd zero_dq = Eigen::VectorXd::Zero(robot.nv());
  const casadi::DMVector derivative_arguments = {
      toCasadi(derivative_q), toCasadi(derivative_v),
      toCasadi(derivative_tau), toCasadi(derivative_lambda),
      toCasadi(zero_dq)};
  const casadi::DMVector vdot_derivatives =
      symbolic.vdot_derivatives(derivative_arguments);
  const casadi::DMVector rolling_derivatives =
      symbolic.rolling_derivatives(
          casadi::DMVector{toCasadi(derivative_q), toCasadi(derivative_v),
                           toCasadi(zero_dq)});
  const casadi::DMVector com_derivative =
      symbolic.com_derivative(
          casadi::DMVector{toCasadi(derivative_q), toCasadi(zero_dq)});

  const Eigen::MatrixXd dvdot_dq = toEigen(vdot_derivatives[0]);
  const Eigen::MatrixXd dvdot_dv = toEigen(vdot_derivatives[1]);
  const Eigen::MatrixXd dvdot_dtau = toEigen(vdot_derivatives[2]);
  const Eigen::MatrixXd dvdot_dlambda = toEigen(vdot_derivatives[3]);
  const Eigen::MatrixXd droll_dq = toEigen(rolling_derivatives[0]);
  const Eigen::MatrixXd droll_dv = toEigen(rolling_derivatives[1]);
  const Eigen::MatrixXd dcom_dq = toEigen(com_derivative[0]);
  const bool derivative_dimensions =
      dvdot_dq.rows() == robot.nv() && dvdot_dq.cols() == robot.nv() &&
      dvdot_dv.rows() == robot.nv() && dvdot_dv.cols() == robot.nv() &&
      dvdot_dtau.rows() == robot.nv() &&
      dvdot_dtau.cols() == sw::kNumActuatedJoints &&
      dvdot_dlambda.rows() == robot.nv() &&
      dvdot_dlambda.cols() == nc_rows && droll_dq.rows() == nc_rows &&
      droll_dq.cols() == robot.nv() && droll_dv.rows() == nc_rows &&
      droll_dv.cols() == robot.nv() && dcom_dq.rows() == 3 &&
      dcom_dq.cols() == robot.nv();
  const bool derivatives_finite =
      dvdot_dq.allFinite() && dvdot_dv.allFinite() &&
      dvdot_dtau.allFinite() && dvdot_dlambda.allFinite() &&
      droll_dq.allFinite() && droll_dv.allFinite() && dcom_dq.allFinite();
  passed &= check(derivative_dimensions,
                  "CasADi tangent derivative dimensions are correct");
  passed &= check(derivatives_finite,
                  "CasADi derivatives are finite with no NaN/Inf");

  const Eigen::VectorXd q_direction =
      randomVector(generator, robot.nv(), 0.4);
  const Eigen::VectorXd v_direction =
      randomVector(generator, robot.nv(), 0.4);
  const Eigen::VectorXd tau_direction =
      randomVector(generator, sw::kNumActuatedJoints, 0.4);
  const Eigen::VectorXd lambda_direction =
      randomVector(generator, nc_rows, 0.4);
  constexpr double kDerivativeEpsilon = 1e-5;

  auto symbolicVdot = [&](const Eigen::VectorXd& q, const Eigen::VectorXd& v,
                          const Eigen::VectorXd& tau,
                          const Eigen::VectorXd& lambda) {
    return evaluateVector(symbolic.forward_dynamics,
                          {toCasadi(q), toCasadi(v), toCasadi(tau),
                           toCasadi(lambda)});
  };
  const Eigen::VectorXd vdot_q_fd =
      (symbolicVdot(pin::integrate(robot.model(), derivative_q,
                                   kDerivativeEpsilon * q_direction),
                    derivative_v, derivative_tau, derivative_lambda) -
       symbolicVdot(pin::integrate(robot.model(), derivative_q,
                                   -kDerivativeEpsilon * q_direction),
                    derivative_v, derivative_tau, derivative_lambda)) /
      (2.0 * kDerivativeEpsilon);
  const Eigen::VectorXd vdot_v_fd =
      (symbolicVdot(derivative_q,
                    derivative_v + kDerivativeEpsilon * v_direction,
                    derivative_tau, derivative_lambda) -
       symbolicVdot(derivative_q,
                    derivative_v - kDerivativeEpsilon * v_direction,
                    derivative_tau, derivative_lambda)) /
      (2.0 * kDerivativeEpsilon);
  const Eigen::VectorXd vdot_tau_fd =
      (symbolicVdot(derivative_q, derivative_v,
                    derivative_tau + kDerivativeEpsilon * tau_direction,
                    derivative_lambda) -
       symbolicVdot(derivative_q, derivative_v,
                    derivative_tau - kDerivativeEpsilon * tau_direction,
                    derivative_lambda)) /
      (2.0 * kDerivativeEpsilon);
  const Eigen::VectorXd vdot_lambda_fd =
      (symbolicVdot(derivative_q, derivative_v, derivative_tau,
                    derivative_lambda +
                        kDerivativeEpsilon * lambda_direction) -
       symbolicVdot(derivative_q, derivative_v, derivative_tau,
                    derivative_lambda -
                        kDerivativeEpsilon * lambda_direction)) /
      (2.0 * kDerivativeEpsilon);

  const double dvdot_dq_error =
      (dvdot_dq * q_direction - vdot_q_fd).cwiseAbs().maxCoeff();
  const double dvdot_dv_error =
      (dvdot_dv * v_direction - vdot_v_fd).cwiseAbs().maxCoeff();
  const double dvdot_dtau_error =
      (dvdot_dtau * tau_direction - vdot_tau_fd).cwiseAbs().maxCoeff();
  const double dvdot_dlambda_error =
      (dvdot_dlambda * lambda_direction - vdot_lambda_fd)
          .cwiseAbs()
          .maxCoeff();

  auto symbolicRolling = [&](const Eigen::VectorXd& q,
                             const Eigen::VectorXd& v) {
    return evaluateVector(symbolic.rolling_residual,
                          {toCasadi(q), toCasadi(v)});
  };
  const Eigen::VectorXd roll_q_fd =
      (symbolicRolling(pin::integrate(robot.model(), derivative_q,
                                      kDerivativeEpsilon * q_direction),
                       derivative_v) -
       symbolicRolling(pin::integrate(robot.model(), derivative_q,
                                      -kDerivativeEpsilon * q_direction),
                       derivative_v)) /
      (2.0 * kDerivativeEpsilon);
  const Eigen::VectorXd roll_v_fd =
      (symbolicRolling(derivative_q,
                       derivative_v + kDerivativeEpsilon * v_direction) -
       symbolicRolling(derivative_q,
                       derivative_v - kDerivativeEpsilon * v_direction)) /
      (2.0 * kDerivativeEpsilon);
  const double droll_dq_error =
      (droll_dq * q_direction - roll_q_fd).cwiseAbs().maxCoeff();
  const double droll_dv_error =
      (droll_dv * v_direction - roll_v_fd).cwiseAbs().maxCoeff();

  auto symbolicCom = [&](const Eigen::VectorXd& q) {
    return evaluateVector(symbolic.center_of_mass, {toCasadi(q)});
  };
  const Eigen::VectorXd com_q_fd =
      (symbolicCom(pin::integrate(robot.model(), derivative_q,
                                  kDerivativeEpsilon * q_direction)) -
       symbolicCom(pin::integrate(robot.model(), derivative_q,
                                  -kDerivativeEpsilon * q_direction))) /
      (2.0 * kDerivativeEpsilon);
  const double dcom_dq_error =
      (dcom_dq * q_direction - com_q_fd).cwiseAbs().maxCoeff();

  std::cout << "DERIVATIVE FD vdot_q=" << dvdot_dq_error
            << " vdot_v=" << dvdot_dv_error
            << " vdot_tau=" << dvdot_dtau_error
            << " vdot_lambda=" << dvdot_dlambda_error << '\n';
  std::cout << "DERIVATIVE FD rolling_q=" << droll_dq_error
            << " rolling_v=" << droll_dv_error
            << " com_q=" << dcom_dq_error << '\n';
  passed &= check(dvdot_dq_error < 1e-5 && dvdot_dv_error < 1e-6 &&
                      dvdot_dtau_error < 1e-7 &&
                      dvdot_dlambda_error < 1e-7,
                  "vdot CasADi derivatives match directional finite differences");
  passed &= check(droll_dq_error < 1e-7 && droll_dv_error < 1e-9,
                  "rolling CasADi derivatives match finite differences");
  passed &= check(dcom_dq_error < 1e-9,
                  "CoM CasADi tangent derivative matches finite differences");

  std::cout << (passed ? "CASADI MODEL TEST PASSED"
                       : "CASADI MODEL TEST FAILED")
            << '\n';
  return passed ? 0 : 1;
}
