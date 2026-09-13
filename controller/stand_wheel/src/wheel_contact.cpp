#include "stand_wheel/wheel_contact.hpp"

#include <array>
#include <cmath>
#include <stdexcept>
#include <string>

namespace stand_wheel {

void WheelContactModel::validateActiveContacts(
    const ActiveContacts& active_contacts) {
  std::array<bool, kNumLegs> seen = {false, false, false, false};
  for (LegId leg : active_contacts) {
    const int index = static_cast<int>(leg);
    if (index < 0 || index >= kNumLegs) {
      throw std::invalid_argument("rolling contacts contain an invalid LegId");
    }
    if (seen[index]) {
      throw std::invalid_argument(std::string("duplicate rolling contact: ") +
                                  legLabel(leg));
    }
    seen[index] = true;
  }
}

void WheelContactModel::validateVelocity(const Eigen::VectorXd& v) const {
  if (v.size() != robot_.nv() || !v.allFinite()) {
    throw std::invalid_argument("velocity has wrong size or non-finite data");
  }
}

WheelContactFrame WheelContactModel::makeFrame(
    LegId leg, const ContactKinematics& contact) const {
  const int leg_index = static_cast<int>(leg);
  WheelContactFrame frame;
  frame.leg = leg;
  frame.wheel_center_world = contact.wheel_center_world;
  frame.contact_point_world = contact.position_world;
  frame.axle_world = contact.wheel_axis_world.normalized();
  frame.normal_world = robot_.groundNormalWorld().normalized();

  const Eigen::Vector3d axle_cross_normal =
      frame.axle_world.cross(frame.normal_world);
  if (axle_cross_normal.norm() < 1e-9) {
    throw std::runtime_error(std::string("wheel axle parallel to ground normal: ") +
                             legLabel(leg));
  }
  frame.rolling_world = axle_cross_normal.normalized();
  frame.lateral_world =
      frame.normal_world.cross(frame.rolling_world).normalized();
  frame.rotation_world_from_contact.col(0) = frame.rolling_world;
  frame.rotation_world_from_contact.col(1) = frame.lateral_world;
  frame.rotation_world_from_contact.col(2) = frame.normal_world;

  frame.force_jacobian_world = contact.jacobian;
  frame.carrier_jacobian_world = contact.jacobian;
  frame.wheel_velocity_index =
      robot_.wheels().at(leg_index).wheel_joint.idx_v;

  // The Step 1 force Jacobian tracks the material point and therefore already
  // contains wheel spin. The rolling carrier excludes that contribution; the
  // -r*qdot term is inserted explicitly in A_roll below.
  const Eigen::Vector3d spin_velocity =
      frame.force_jacobian_world.col(frame.wheel_velocity_index);
  const Eigen::Vector3d expected_spin_velocity =
      -robot_.wheelRadius() * frame.rolling_world;
  if ((spin_velocity - expected_spin_velocity).norm() > 1e-8) {
    throw std::runtime_error(std::string("wheel spin/contact convention mismatch: ") +
                             legLabel(leg));
  }
  frame.carrier_jacobian_world.col(frame.wheel_velocity_index).setZero();
  return frame;
}

RollingConstraintData WheelContactModel::evaluate(
    const Eigen::VectorXd& q, const Eigen::VectorXd& v,
    const ActiveContacts& active_contacts) {
  validateVelocity(v);
  validateActiveContacts(active_contacts);
  const ModelSnapshot snapshot = robot_.evaluate(q);

  RollingConstraintData result;
  result.active_contacts = active_contacts;
  result.frames.reserve(active_contacts.size());
  result.matrix = Eigen::MatrixXd::Zero(
      3 * static_cast<int>(active_contacts.size()), robot_.nv());

  for (std::size_t i = 0; i < active_contacts.size(); ++i) {
    const LegId leg = active_contacts[i];
    const int leg_index = static_cast<int>(leg);
    result.frames.push_back(makeFrame(leg, snapshot.contacts.at(leg_index)));
    const WheelContactFrame& frame = result.frames.back();
    const int row = 3 * static_cast<int>(i);

    result.matrix.row(row) =
        frame.normal_world.transpose() * frame.carrier_jacobian_world;
    result.matrix.row(row + 1) =
        frame.lateral_world.transpose() * frame.carrier_jacobian_world;
    result.matrix.row(row + 2) =
        frame.rolling_world.transpose() * frame.carrier_jacobian_world;
    result.matrix(row + 2, frame.wheel_velocity_index) -=
        robot_.wheelRadius();
  }
  result.residual = result.matrix * v;
  return result;
}

Eigen::MatrixXd WheelContactModel::constraintMatrix(
    const Eigen::VectorXd& q,
    const ActiveContacts& active_contacts) {
  return evaluate(q, Eigen::VectorXd::Zero(robot_.nv()), active_contacts).matrix;
}

Eigen::VectorXd WheelContactModel::residual(
    const Eigen::VectorXd& q, const Eigen::VectorXd& v,
    const ActiveContacts& active_contacts) {
  return evaluate(q, v, active_contacts).residual;
}

SkidSteerConstraintData WheelContactModel::skidSteer(
    const Eigen::VectorXd& q, const Eigen::VectorXd& v,
    const ActiveContacts& active_contacts) {
  const RollingConstraintData rolling = evaluate(q, v, active_contacts);
  const int num_contacts = static_cast<int>(active_contacts.size());

  SkidSteerConstraintData result;
  result.active_contacts = rolling.active_contacts;
  result.frames = rolling.frames;
  result.hard_matrix = Eigen::MatrixXd::Zero(2 * num_contacts, robot_.nv());
  result.lateral_matrix = Eigen::MatrixXd::Zero(num_contacts, robot_.nv());
  for (int contact = 0; contact < num_contacts; ++contact) {
    result.hard_matrix.row(2 * contact) = rolling.matrix.row(3 * contact);
    result.hard_matrix.row(2 * contact + 1) =
        rolling.matrix.row(3 * contact + 2);
    result.lateral_matrix.row(contact) =
        rolling.matrix.row(3 * contact + 1);
  }
  result.hard_residual = result.hard_matrix * v;
  result.lateral_slip = result.lateral_matrix * v;
  return result;
}

Eigen::MatrixXd WheelContactModel::hardConstraintMatrix(
    const Eigen::VectorXd& q,
    const ActiveContacts& active_contacts) {
  return skidSteer(q, Eigen::VectorXd::Zero(robot_.nv()), active_contacts)
      .hard_matrix;
}

Eigen::VectorXd WheelContactModel::hardResidual(
    const Eigen::VectorXd& q, const Eigen::VectorXd& v,
    const ActiveContacts& active_contacts) {
  return skidSteer(q, v, active_contacts).hard_residual;
}

Eigen::VectorXd WheelContactModel::lateralSlip(
    const Eigen::VectorXd& q, const Eigen::VectorXd& v,
    const ActiveContacts& active_contacts) {
  return skidSteer(q, v, active_contacts).lateral_slip;
}

Eigen::MatrixXd WheelContactModel::forceTransformWorldFromWheel(
    const Eigen::VectorXd& q,
    const ActiveContacts& active_contacts) {
  const SkidSteerConstraintData data = skidSteer(
      q, Eigen::VectorXd::Zero(robot_.nv()), active_contacts);
  const int rows = 3 * static_cast<int>(active_contacts.size());
  Eigen::MatrixXd transform = Eigen::MatrixXd::Zero(rows, rows);
  for (std::size_t contact = 0; contact < data.frames.size(); ++contact) {
    transform.block<3, 3>(3 * static_cast<int>(contact),
                          3 * static_cast<int>(contact)) =
        data.frames[contact].rotation_world_from_contact;
  }
  return transform;
}

Eigen::VectorXd WheelContactModel::wheelForcesToWorld(
    const Eigen::VectorXd& q,
    const ActiveContacts& active_contacts,
    const Eigen::Ref<const Eigen::VectorXd>& lambda_wheel) {
  const int expected_size = 3 * static_cast<int>(active_contacts.size());
  if (lambda_wheel.size() != expected_size || !lambda_wheel.allFinite()) {
    throw std::invalid_argument(
        "wheel-coordinate forces have wrong size or non-finite data");
  }
  return forceTransformWorldFromWheel(q, active_contacts) * lambda_wheel;
}

Eigen::VectorXd WheelContactModel::worldForcesToWheel(
    const Eigen::VectorXd& q,
    const ActiveContacts& active_contacts,
    const Eigen::Ref<const Eigen::VectorXd>& lambda_world) {
  const int expected_size = 3 * static_cast<int>(active_contacts.size());
  if (lambda_world.size() != expected_size || !lambda_world.allFinite()) {
    throw std::invalid_argument(
        "world-coordinate forces have wrong size or non-finite data");
  }
  return forceTransformWorldFromWheel(q, active_contacts).transpose() *
         lambda_world;
}

FrictionFeasibility WheelContactModel::checkFriction(
    const Eigen::Ref<const Eigen::VectorXd>& lambda_wheel,
    const FrictionConfig& config, double tolerance) {
  if (lambda_wheel.size() % 3 != 0 || !lambda_wheel.allFinite()) {
    throw std::invalid_argument(
        "friction forces must be finite [roll,lat,normal] blocks");
  }
  if (!std::isfinite(config.mu_roll) || !std::isfinite(config.mu_lat) ||
      config.mu_roll < 0.0 || config.mu_lat < 0.0) {
    throw std::invalid_argument(
        "friction coefficients must be finite and nonnegative");
  }
  if (!std::isfinite(tolerance) || tolerance < 0.0) {
    throw std::invalid_argument(
        "friction feasibility tolerance must be finite and nonnegative");
  }

  const int num_contacts = lambda_wheel.size() / 3;
  FrictionFeasibility result;
  result.margins.resize(5 * num_contacts);
  for (int contact = 0; contact < num_contacts; ++contact) {
    const double force_roll = lambda_wheel[3 * contact];
    const double force_lateral = lambda_wheel[3 * contact + 1];
    const double force_normal = lambda_wheel[3 * contact + 2];
    result.margins.segment<5>(5 * contact) <<
        force_normal,
        config.mu_roll * force_normal - force_roll,
        config.mu_roll * force_normal + force_roll,
        config.mu_lat * force_normal - force_lateral,
        config.mu_lat * force_normal + force_lateral;
  }
  result.violations = (-result.margins.array()).max(0.0).matrix();
  result.feasible = result.margins.size() == 0 ||
                    result.margins.minCoeff() >= -tolerance;
  return result;
}

}  // namespace stand_wheel
