#include "stand_wheel/full_dynamics.hpp"

#include <Eigen/QR>
#include <pinocchio/algorithm/joint-configuration.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <string>

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
  throw std::runtime_error("test could not find joint: " + name);
}

Eigen::VectorXd deterministicVector(int size, double scale, double phase) {
  Eigen::VectorXd result(size);
  for (int i = 0; i < size; ++i) {
    result[i] = scale * std::sin(phase + 0.73 * static_cast<double>(i + 1));
  }
  return result;
}

}  // namespace

int main(int argc, char** argv) {
  const std::string urdf =
      argc > 1 ? argv[1] : STAND_WHEEL_DEFAULT_URDF_PATH;
  sw::PinocchioModel robot({urdf});
  sw::FullDynamics dynamics(robot);
  const pin::Model& model = robot.model();
  bool passed = true;

  std::cout << std::scientific << std::setprecision(6);

  // A. Every row of S is resolved from a named scalar joint and maps exactly
  // one actuator torque into the corresponding generalized velocity slot.
  const Eigen::MatrixXd& S = dynamics.selectionMatrix();
  const sw::JointMapping root = findJoint(robot, "root_joint");
  passed &= check(S.rows() == sw::kNumActuatedJoints && S.cols() == robot.nv(),
                  "selection matrix S is 16 x nv");
  passed &= check(S.middleCols(root.idx_v, root.nv).norm() == 0.0,
                  "all six floating-base columns are unactuated");

  double actuator_injection_error = 0.0;
  for (int actuator = 0; actuator < sw::kNumActuatedJoints; ++actuator) {
    Eigen::VectorXd torque = Eigen::VectorXd::Zero(sw::kNumActuatedJoints);
    torque[actuator] = 1.0 + 0.125 * actuator;
    const Eigen::VectorXd generalized = dynamics.generalizedActuation(torque);
    Eigen::VectorXd expected = Eigen::VectorXd::Zero(robot.nv());
    expected[dynamics.actuatorOrdering()[actuator].idx_v] = torque[actuator];
    actuator_injection_error =
        std::max(actuator_injection_error,
                 (generalized - expected).cwiseAbs().maxCoeff());
  }
  std::cout << "ACTUATION injection_max_error=" << actuator_injection_error
            << '\n';
  passed &= check(actuator_injection_error == 0.0,
                  "all 16 actuator injections hit only their named DoF");

  const std::array<std::string, sw::kNumActuatedJoints> expected_actuators = {
      "FL_HipX_joint", "FL_HipY_joint", "FL_Knee_joint",
      "FR_HipX_joint", "FR_HipY_joint", "FR_Knee_joint",
      "HL_HipX_joint", "HL_HipY_joint", "HL_Knee_joint",
      "HR_HipX_joint", "HR_HipY_joint", "HR_Knee_joint",
      "FL_WHEEL",      "FR_WHEEL",      "HL_WHEEL",
      "HR_WHEEL"};
  bool actuator_names_ok = true;
  std::cout << "ACTUATOR ORDERING\n";
  for (int i = 0; i < sw::kNumActuatedJoints; ++i) {
    const sw::JointMapping& actuator = dynamics.actuatorOrdering()[i];
    actuator_names_ok &= actuator.name == expected_actuators[i];
    std::cout << "  " << i << " " << actuator.name
              << " -> generalized_v[" << actuator.idx_v << "]\n";
  }
  passed &= check(actuator_names_ok, "actuator order is 12 legs then FL/FR/HL/HR wheels");

  const Eigen::VectorXd q_nominal = robot.nominalConfiguration();
  const Eigen::VectorXd v_zero = Eigen::VectorXd::Zero(robot.nv());
  const sw::ActiveContacts all_contacts = {
      sw::LegId::FL, sw::LegId::FR, sw::LegId::HL, sw::LegId::HR};

  // B. Use a deliberately non-canonical subset to prove stack order follows
  // the caller's active-contact sequence.
  const Eigen::VectorXd q_perturbation =
      deterministicVector(robot.nv(), 0.025, 0.2);
  const Eigen::VectorXd q_test =
      pin::integrate(model, q_nominal, q_perturbation);
  const Eigen::VectorXd v_test =
      deterministicVector(robot.nv(), 0.18, -0.4);
  const sw::ActiveContacts subset = {
      sw::LegId::HR, sw::LegId::FL, sw::LegId::HL};
  const sw::FullDynamicsTerms subset_terms =
      dynamics.compute(q_test, v_test, subset);
  const Eigen::VectorXd lambda_virtual =
      deterministicVector(3 * static_cast<int>(subset.size()), 40.0, 0.7);
  const Eigen::VectorXd delta_v =
      deterministicVector(robot.nv(), 0.03, -0.1);
  const Eigen::VectorXd tau_contact =
      dynamics.generalizedContactForce(subset_terms.contacts, lambda_virtual);
  const double virtual_work_contact =
      lambda_virtual.dot(subset_terms.contacts.jacobian_world * delta_v);
  const double virtual_work_generalized = tau_contact.dot(delta_v);
  const double virtual_work_error =
      std::abs(virtual_work_contact - virtual_work_generalized);
  std::cout << "VIRTUAL WORK contact_order=HR,FL,HL error="
            << virtual_work_error << '\n';
  passed &= check(subset_terms.contacts.jacobian_world.rows() == 9 &&
                      subset_terms.contacts.jacobian_world.cols() == robot.nv(),
                  "three-contact Jacobian stack is 9 x nv");
  passed &= check(subset_terms.contacts.active_contacts == subset,
                  "contact stack preserves caller order HR, FL, HL");
  passed &= check(virtual_work_error < 1e-12,
                  "contact virtual work identity holds");

  const sw::ActiveContacts no_contacts;
  const sw::FullDynamicsTerms free_terms =
      dynamics.compute(q_test, v_test, no_contacts);
  const Eigen::VectorXd no_contact_force = dynamics.generalizedContactForce(
      free_terms.contacts, Eigen::VectorXd::Zero(0));
  passed &= check(free_terms.contacts.jacobian_world.rows() == 0 &&
                      free_terms.contacts.jacobian_world.cols() == robot.nv() &&
                      no_contact_force.size() == robot.nv() &&
                      no_contact_force.norm() == 0.0,
                  "empty active-contact set produces a 0 x nv stack and zero force");

  // C. Solve only the six unactuated root equations for four vertical forces.
  // Actuator torques then balance the remaining named joint equations.
  const sw::FullDynamicsTerms static_terms =
      dynamics.compute(q_nominal, v_zero, all_contacts);
  Eigen::MatrixXd vertical_selector = Eigen::MatrixXd::Zero(12, 4);
  for (int contact = 0; contact < 4; ++contact) {
    vertical_selector(3 * contact + 2, contact) = 1.0;
  }
  const Eigen::MatrixXd vertical_generalized =
      static_terms.contacts.jacobian_world.transpose() * vertical_selector;
  const Eigen::MatrixXd base_equilibrium =
      vertical_generalized.middleRows(root.idx_v, root.nv);
  const Eigen::VectorXd base_gravity =
      static_terms.nonlinear_effects.segment(root.idx_v, root.nv);
  const Eigen::VectorXd vertical_forces =
      base_equilibrium.completeOrthogonalDecomposition().solve(base_gravity);
  Eigen::VectorXd lambda_static = vertical_selector * vertical_forces;
  const Eigen::VectorXd static_contact_force =
      dynamics.generalizedContactForce(static_terms.contacts, lambda_static);
  const Eigen::VectorXd tau_static =
      S * (static_terms.nonlinear_effects - static_contact_force);
  const Eigen::VectorXd vdot_static = dynamics.forwardDynamics(
      q_nominal, v_zero, tau_static, all_contacts, lambda_static);
  const Eigen::VectorXd static_residual =
      static_terms.mass_matrix * vdot_static +
      static_terms.nonlinear_effects -
      dynamics.generalizedActuation(tau_static) - static_contact_force;
  const double expected_weight = robot.totalMass() * 9.81;
  const double force_sum_error =
      std::abs(vertical_forces.sum() - expected_weight);
  const double static_base_acceleration =
      vdot_static.segment(root.idx_v, root.nv).norm();
  std::cout << "STATIC SUPPORT fz=[" << vertical_forces.transpose()
            << "] sum=" << vertical_forces.sum()
            << " mg=" << expected_weight
            << " sum_error=" << force_sum_error
            << " base_accel_norm=" << static_base_acceleration
            << " full_vdot_norm=" << vdot_static.norm()
            << " residual_norm=" << static_residual.norm() << '\n';
  passed &= check((vertical_forces.array() > 0.0).all(),
                  "all four static vertical forces are positive");
  passed &= check(force_sum_error < 1e-9,
                  "static vertical forces sum to robot weight");
  passed &= check(static_base_acceleration < 1e-10,
                  "static-support base acceleration is near zero");
  passed &= check(vdot_static.norm() < 1e-9,
                  "static support plus joint torques balances full dynamics");

  // D. Exercise nonzero q, v, actuator torques, and two contact wrenches.
  const sw::ActiveContacts diagonal = {sw::LegId::FL, sw::LegId::HR};
  const Eigen::VectorXd tau_test =
      deterministicVector(sw::kNumActuatedJoints, 12.0, 0.3);
  Eigen::VectorXd lambda_test = deterministicVector(6, 30.0, -0.8);
  lambda_test[2] += 90.0;
  lambda_test[5] += 100.0;
  const Eigen::VectorXd vdot = dynamics.forwardDynamics(
      q_test, v_test, tau_test, diagonal, lambda_test);
  const sw::FullDynamicsTerms dynamic_terms =
      dynamics.compute(q_test, v_test, diagonal);
  const Eigen::VectorXd dynamic_residual =
      dynamic_terms.mass_matrix * vdot + dynamic_terms.nonlinear_effects -
      dynamics.generalizedActuation(tau_test) -
      dynamics.generalizedContactForce(dynamic_terms.contacts, lambda_test);
  std::cout << "DYNAMICS residual_norm=" << dynamic_residual.norm()
            << " residual_max_abs=" << dynamic_residual.cwiseAbs().maxCoeff()
            << '\n';
  passed &= check(dynamic_residual.norm() < 1e-10,
                  "forward-dynamics equation residual is near machine precision");

  // E. At rest Jv and Jdot*v are exactly zero. At a moving state, validate the
  // Pinocchio classical-acceleration result against central differences of Jv
  // while keeping each central material-point offset fixed.
  const double static_contact_velocity =
      (static_terms.contacts.jacobian_world * v_zero).norm();
  const double static_jdot_v = static_terms.contacts.jdot_v_world.norm();
  const double static_contact_acceleration =
      (static_terms.contacts.jacobian_world * vdot_static +
       static_terms.contacts.jdot_v_world)
          .norm();
  constexpr double kJdotEpsilon = 1e-6;
  const Eigen::VectorXd q_plus =
      pin::integrate(model, q_test, kJdotEpsilon * v_test);
  const Eigen::VectorXd q_minus =
      pin::integrate(model, q_test, -kJdotEpsilon * v_test);
  const sw::ModelSnapshot center_snapshot = robot.evaluate(q_test);
  double max_jdot_v_error = 0.0;
  for (std::size_t i = 0; i < subset.size(); ++i) {
    const int leg = static_cast<int>(subset[i]);
    const Eigen::Vector3d offset_local =
        center_snapshot.contacts[leg].offset_local;
    const Eigen::MatrixXd jacobian_plus = robot.contactMaterialJacobian(
        q_plus, subset[i], offset_local);
    const Eigen::MatrixXd jacobian_minus = robot.contactMaterialJacobian(
        q_minus, subset[i], offset_local);
    const Eigen::Vector3d numerical =
        ((jacobian_plus - jacobian_minus) * v_test) /
        (2.0 * kJdotEpsilon);
    const Eigen::Vector3d analytical =
        subset_terms.contacts.jdot_v_world.segment<3>(3 * static_cast<int>(i));
    max_jdot_v_error =
        std::max(max_jdot_v_error, (numerical - analytical).norm());
  }
  std::cout << "CONTACT ACCEL static_Jv=" << static_contact_velocity
            << " static_Jdotv=" << static_jdot_v
            << " static_accel=" << static_contact_acceleration
            << " moving_Jdotv_fd_max_error=" << max_jdot_v_error << '\n';
  passed &= check(static_contact_velocity == 0.0,
                  "nominal static contact velocity Jc*v is zero");
  passed &= check(static_jdot_v == 0.0,
                  "nominal static contact bias Jdot*v is zero");
  passed &= check(static_contact_acceleration < 1e-9,
                  "nominal supported contact acceleration is near zero");
  passed &= check(max_jdot_v_error < 1e-7,
                  "Jdot*v matches central finite differences");

  std::cout << (passed ? "FULL DYNAMICS TEST PASSED"
                       : "FULL DYNAMICS TEST FAILED")
            << '\n';
  return passed ? 0 : 1;
}
