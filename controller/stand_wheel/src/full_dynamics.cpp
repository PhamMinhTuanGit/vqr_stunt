#include "stand_wheel/full_dynamics.hpp"

#include <Eigen/Cholesky>
#include <pinocchio/algorithm/rnea.hpp>

#include <array>
#include <stdexcept>
#include <string>

namespace stand_wheel {
namespace pin = pinocchio;

FullDynamics::FullDynamics(PinocchioModel& robot)
    : robot_(robot), dynamics_data_(robot.model()) {
  selection_matrix_ =
      Eigen::MatrixXd::Zero(kNumActuatedJoints, robot_.nv());

  int actuator = 0;
  for (const JointMapping& joint : robot_.legJoints()) {
    actuator_ordering_[actuator++] = joint;
  }
  for (const WheelContactMapping& wheel : robot_.wheels()) {
    actuator_ordering_[actuator++] = wheel.wheel_joint;
  }

  std::vector<bool> velocity_index_used(robot_.nv(), false);
  for (int i = 0; i < kNumActuatedJoints; ++i) {
    const JointMapping& joint = actuator_ordering_[i];
    if (joint.nv != 1 || joint.idx_v < 0 || joint.idx_v >= robot_.nv()) {
      throw std::runtime_error("actuator is not a scalar velocity DoF: " +
                               joint.name);
    }
    if (velocity_index_used[joint.idx_v]) {
      throw std::runtime_error("duplicate actuator velocity index: " +
                               joint.name);
    }
    velocity_index_used[joint.idx_v] = true;
    selection_matrix_(i, joint.idx_v) = 1.0;
  }

  const pin::Model& model = robot_.model();
  if (!model.existJointName("root_joint")) {
    throw std::runtime_error("missing floating-base root_joint");
  }
  const pin::JointIndex root_id = model.getJointId("root_joint");
  const int root_idx_v = model.idx_vs[root_id];
  const int root_nv = model.nvs[root_id];
  if (root_nv != 6 ||
      selection_matrix_.middleCols(root_idx_v, root_nv).norm() != 0.0) {
    throw std::runtime_error("floating base must have six unactuated DoFs");
  }
}

void FullDynamics::validateActiveContacts(
    const ActiveContacts& active_contacts) {
  std::array<bool, kNumLegs> seen = {false, false, false, false};
  for (LegId leg : active_contacts) {
    const int index = static_cast<int>(leg);
    if (index < 0 || index >= kNumLegs) {
      throw std::invalid_argument("active contact contains an invalid LegId");
    }
    if (seen[index]) {
      throw std::invalid_argument(std::string("duplicate active contact: ") +
                                  legLabel(leg));
    }
    seen[index] = true;
  }
}

void FullDynamics::validateVelocity(const Eigen::VectorXd& v) const {
  if (v.size() != robot_.nv() || !v.allFinite()) {
    throw std::invalid_argument("velocity has wrong size or non-finite data");
  }
}

Eigen::VectorXd FullDynamics::generalizedActuation(
    const Eigen::Ref<const Eigen::VectorXd>& tau_actuated) const {
  if (tau_actuated.size() != kNumActuatedJoints ||
      !tau_actuated.allFinite()) {
    throw std::invalid_argument(
        "actuated torque must be a finite 16-vector");
  }
  return selection_matrix_.transpose() * tau_actuated;
}

ContactStack FullDynamics::contactStack(
    const Eigen::VectorXd& q, const Eigen::VectorXd& v,
    const ActiveContacts& active_contacts) {
  validateVelocity(v);
  validateActiveContacts(active_contacts);
  const ModelSnapshot snapshot = robot_.evaluate(q);
  return makeContactStack(q, v, active_contacts, snapshot);
}

ContactStack FullDynamics::makeContactStack(
    const Eigen::VectorXd& q, const Eigen::VectorXd& v,
    const ActiveContacts& active_contacts, const ModelSnapshot& snapshot) {
  ContactStack result;
  result.active_contacts = active_contacts;
  result.positions_world.reserve(active_contacts.size());
  result.jacobian_world = Eigen::MatrixXd::Zero(
      3 * static_cast<int>(active_contacts.size()), robot_.nv());
  result.jdot_v_world =
      Eigen::VectorXd::Zero(3 * static_cast<int>(active_contacts.size()));

  for (std::size_t i = 0; i < active_contacts.size(); ++i) {
    const int leg = static_cast<int>(active_contacts[i]);
    const ContactKinematics& contact = snapshot.contacts[leg];
    result.positions_world.push_back(contact.position_world);
    result.jacobian_world.middleRows<3>(3 * static_cast<int>(i)) =
        contact.jacobian;
    result.jdot_v_world.segment<3>(3 * static_cast<int>(i)) =
        robot_.contactMaterialAccelerationBias(
            q, v, active_contacts[i], contact.offset_local);
  }
  return result;
}

Eigen::VectorXd FullDynamics::generalizedContactForce(
    const ContactStack& contacts,
    const Eigen::Ref<const Eigen::VectorXd>& lambda_world) const {
  const int expected_rows =
      3 * static_cast<int>(contacts.active_contacts.size());
  if (contacts.jacobian_world.rows() != expected_rows ||
      contacts.jacobian_world.cols() != robot_.nv() ||
      !contacts.jacobian_world.allFinite()) {
    throw std::invalid_argument("contact Jacobian has inconsistent dimensions");
  }
  if (lambda_world.size() != expected_rows || !lambda_world.allFinite()) {
    throw std::invalid_argument(
        "lambda must contain one finite [fx,fy,fz] block per contact");
  }
  return contacts.jacobian_world.transpose() * lambda_world;
}

FullDynamicsTerms FullDynamics::compute(
    const Eigen::VectorXd& q, const Eigen::VectorXd& v,
    const ActiveContacts& active_contacts) {
  validateVelocity(v);
  validateActiveContacts(active_contacts);
  const ModelSnapshot snapshot = robot_.evaluate(q);

  FullDynamicsTerms result;
  result.mass_matrix = snapshot.mass_matrix;
  result.nonlinear_effects =
      pin::nonLinearEffects(robot_.model(), dynamics_data_, q, v);
  result.contacts = makeContactStack(q, v, active_contacts, snapshot);
  if (!result.nonlinear_effects.allFinite()) {
    throw std::runtime_error("Pinocchio nonlinear effects are non-finite");
  }
  return result;
}

Eigen::VectorXd FullDynamics::forwardDynamics(
    const Eigen::VectorXd& q, const Eigen::VectorXd& v,
    const Eigen::Ref<const Eigen::VectorXd>& tau_actuated,
    const ActiveContacts& active_contacts,
    const Eigen::Ref<const Eigen::VectorXd>& lambda_world) {
  const FullDynamicsTerms terms = compute(q, v, active_contacts);
  const Eigen::VectorXd right_hand_side =
      generalizedActuation(tau_actuated) +
      generalizedContactForce(terms.contacts, lambda_world) -
      terms.nonlinear_effects;

  Eigen::LDLT<Eigen::MatrixXd> factorization(terms.mass_matrix);
  if (factorization.info() != Eigen::Success ||
      (factorization.vectorD().array() <= 0.0).any()) {
    throw std::runtime_error("mass matrix LDLT factorization failed");
  }
  Eigen::VectorXd acceleration = factorization.solve(right_hand_side);
  if (factorization.info() != Eigen::Success || !acceleration.allFinite()) {
    throw std::runtime_error("forward dynamics solve failed");
  }
  return acceleration;
}

}  // namespace stand_wheel
