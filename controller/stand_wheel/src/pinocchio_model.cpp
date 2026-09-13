#include "stand_wheel/pinocchio_model.hpp"

#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <cmath>
#include <stdexcept>
#include <utility>

namespace stand_wheel {
namespace pin = pinocchio;

namespace {

constexpr std::array<const char*, 12> kLegJointNames = {
    "FL_HipX_joint", "FL_HipY_joint", "FL_Knee_joint",
    "FR_HipX_joint", "FR_HipY_joint", "FR_Knee_joint",
    "HL_HipX_joint", "HL_HipY_joint", "HL_Knee_joint",
    "HR_HipX_joint", "HR_HipY_joint", "HR_Knee_joint"};

constexpr std::array<const char*, kNumLegs> kWheelNames = {
    "FL_WHEEL", "FR_WHEEL", "HL_WHEEL", "HR_WHEEL"};

constexpr std::array<const char*, kNumLegs> kLegLabels = {
    "FL", "FR", "HL", "HR"};

Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
  Eigen::Matrix3d result;
  result << 0.0, -v.z(), v.y(), v.z(), 0.0, -v.x(), -v.y(), v.x(), 0.0;
  return result;
}

int legIndex(LegId leg) { return static_cast<int>(leg); }

}  // namespace

const char* legLabel(LegId leg) {
  return kLegLabels.at(static_cast<std::size_t>(leg));
}

PinocchioModel::PinocchioModel(ModelConfig config)
    : config_(std::move(config)) {
  if (config_.urdf_path.empty()) {
    throw std::invalid_argument("URDF path must not be empty");
  }
  if (!(config_.wheel_radius > 0.0) ||
      !std::isfinite(config_.wheel_radius)) {
    throw std::invalid_argument("wheel radius must be finite and positive");
  }
  if (config_.wheel_axis_local.norm() < 1e-12 ||
      config_.ground_normal_world.norm() < 1e-12) {
    throw std::invalid_argument("wheel axis and ground normal must be non-zero");
  }
  config_.wheel_axis_local.normalize();
  config_.ground_normal_world.normalize();

  pin::urdf::buildModel(config_.urdf_path, pin::JointModelFreeFlyer(), model_);
  data_ = pin::Data(model_);
  total_mass_ = pin::computeTotalMass(model_);

  root_joint_ = resolveJoint("root_joint");
  if (root_joint_.idx_q != 0 || root_joint_.nq != 7 ||
      root_joint_.idx_v != 0 || root_joint_.nv != 6) {
    throw std::runtime_error("unexpected free-flyer root layout");
  }

  for (std::size_t i = 0; i < kLegJointNames.size(); ++i) {
    leg_joints_[i] = resolveJoint(kLegJointNames[i]);
    if (leg_joints_[i].nq != 1 || leg_joints_[i].nv != 1) {
      throw std::runtime_error("expected scalar leg joint: " +
                               leg_joints_[i].name);
    }
  }

  for (int i = 0; i < kNumLegs; ++i) {
    const auto leg = static_cast<LegId>(i);
    const std::string name = kWheelNames[i];
    if (!model_.existFrame(name)) {
      throw std::runtime_error("missing wheel body frame: " + name);
    }
    const pin::FrameIndex frame_id = model_.getFrameId(name, pin::BODY);
    if (frame_id >= model_.nframes) {
      throw std::runtime_error("wheel frame is not a BODY frame: " + name);
    }
    wheels_[i] = {leg, name.substr(0, 2), resolveJoint(name), name, frame_id};
    if (model_.frames[frame_id].parentJoint !=
        wheels_[i].wheel_joint.joint_id) {
      throw std::runtime_error("wheel BODY frame is attached to wrong joint: " +
                               name);
    }
    if (wheels_[i].wheel_joint.nv != 1) {
      throw std::runtime_error("expected one velocity DoF for wheel: " + name);
    }
  }
}

JointMapping PinocchioModel::resolveJoint(const std::string& name) const {
  if (!model_.existJointName(name)) {
    throw std::runtime_error("missing joint: " + name);
  }
  const pin::JointIndex id = model_.getJointId(name);
  return {name, id, model_.idx_qs[id], model_.nqs[id], model_.idx_vs[id],
          model_.nvs[id]};
}

std::vector<JointMapping> PinocchioModel::jointOrdering() const {
  std::vector<JointMapping> ordering;
  ordering.reserve(model_.njoints - 1);
  for (pin::JointIndex id = 1; id < model_.njoints; ++id) {
    ordering.push_back({model_.names[id], id, model_.idx_qs[id],
                        model_.nqs[id], model_.idx_vs[id], model_.nvs[id]});
  }
  return ordering;
}

void PinocchioModel::setScalarJoint(Eigen::VectorXd& q,
                                    const std::string& name,
                                    double value) const {
  const JointMapping joint = resolveJoint(name);
  if (joint.nq != 1 || joint.nv != 1) {
    throw std::runtime_error("cannot assign scalar q to joint: " + name);
  }
  q[joint.idx_q] = value;
}

Eigen::VectorXd PinocchioModel::nominalConfiguration() const {
  Eigen::VectorXd q = pin::neutral(model_);
  for (int leg = 0; leg < kNumLegs; ++leg) {
    setScalarJoint(q, kLegJointNames[3 * leg], 0.0);
    setScalarJoint(q, kLegJointNames[3 * leg + 1], -0.82);
    setScalarJoint(q, kLegJointNames[3 * leg + 2], 1.61);
  }

  pin::Data nominal_data(model_);
  pin::forwardKinematics(model_, nominal_data, q);
  pin::updateFramePlacements(model_, nominal_data);

  double contact_height_sum = 0.0;
  for (const auto& wheel : wheels_) {
    const auto& placement = nominal_data.oMf[wheel.frame_id];
    const Eigen::Vector3d axis =
        (placement.rotation() * config_.wheel_axis_local).normalized();
    Eigen::Vector3d radial_down = config_.ground_normal_world -
                                  axis.dot(config_.ground_normal_world) * axis;
    if (radial_down.norm() < 1e-9) {
      throw std::runtime_error("wheel axis parallel to ground normal at nominal");
    }
    radial_down.normalize();
    const Eigen::Vector3d contact =
        placement.translation() - config_.wheel_radius * radial_down;
    contact_height_sum += contact.dot(config_.ground_normal_world);
  }

  q.segment<3>(root_joint_.idx_q) -=
      (contact_height_sum / kNumLegs) * config_.ground_normal_world;
  return q;
}

ContactKinematics PinocchioModel::computeContact(
    pin::FrameIndex frame_id) {
  ContactKinematics contact;
  Eigen::MatrixXd frame_jacobian(6, model_.nv);
  frame_jacobian.setZero();
  pin::getFrameJacobian(model_, data_, frame_id, pin::LOCAL_WORLD_ALIGNED,
                        frame_jacobian);

  const auto& placement = data_.oMf[frame_id];
  contact.wheel_center_world = placement.translation();
  contact.wheel_axis_world =
      (placement.rotation() * config_.wheel_axis_local).normalized();

  Eigen::Vector3d radial_down = config_.ground_normal_world -
                                contact.wheel_axis_world.dot(
                                    config_.ground_normal_world) *
                                    contact.wheel_axis_world;
  if (radial_down.norm() < 1e-9) {
    throw std::runtime_error("wheel axis is parallel to ground normal");
  }
  radial_down.normalize();
  contact.offset_world = -config_.wheel_radius * radial_down;
  contact.offset_local = placement.rotation().transpose() * contact.offset_world;
  contact.position_world = contact.wheel_center_world + contact.offset_world;

  // Pinocchio motion rows are [linear; angular]. For a material point P with
  // d = p_P-p_frame: v_P = v_frame + omega x d.
  contact.jacobian = frame_jacobian.topRows<3>() -
                     skew(contact.offset_world) *
                         frame_jacobian.bottomRows<3>();
  return contact;
}

ModelSnapshot PinocchioModel::evaluate(const Eigen::VectorXd& q) {
  if (q.size() != model_.nq || !q.allFinite()) {
    throw std::invalid_argument("configuration has wrong size or non-finite data");
  }
  if (!pin::isNormalized(model_, q, 1e-8)) {
    throw std::invalid_argument("configuration is not normalized");
  }

  pin::forwardKinematics(model_, data_, q);
  pin::updateFramePlacements(model_, data_);
  pin::computeJointJacobians(model_, data_, q);

  ModelSnapshot snapshot;
  snapshot.center_of_mass_world = pin::centerOfMass(model_, data_, q);
  snapshot.mass_matrix = pin::crba(model_, data_, q);
  snapshot.mass_matrix.triangularView<Eigen::StrictlyLower>() =
      snapshot.mass_matrix.transpose().triangularView<Eigen::StrictlyLower>();
  for (int leg = 0; leg < kNumLegs; ++leg) {
    snapshot.contacts[leg] = computeContact(wheels_[leg].frame_id);
  }
  return snapshot;
}

Eigen::Vector3d PinocchioModel::contactMaterialPosition(
    const Eigen::VectorXd& q, LegId leg,
    const Eigen::Vector3d& offset_local) const {
  if (q.size() != model_.nq) {
    throw std::invalid_argument("configuration has wrong size");
  }
  pin::Data data(model_);
  pin::forwardKinematics(model_, data, q);
  pin::updateFramePlacements(model_, data);
  const auto& placement = data.oMf[wheels_.at(legIndex(leg)).frame_id];
  return placement.translation() + placement.rotation() * offset_local;
}

Eigen::MatrixXd PinocchioModel::contactMaterialJacobian(
    const Eigen::VectorXd& q, LegId leg,
    const Eigen::Vector3d& offset_local) const {
  if (q.size() != model_.nq || !q.allFinite()) {
    throw std::invalid_argument("configuration has wrong size or non-finite data");
  }
  pin::Data data(model_);
  pin::forwardKinematics(model_, data, q);
  pin::updateFramePlacements(model_, data);
  pin::computeJointJacobians(model_, data, q);

  const pin::FrameIndex frame_id = wheels_.at(legIndex(leg)).frame_id;
  Eigen::MatrixXd frame_jacobian = Eigen::MatrixXd::Zero(6, model_.nv);
  pin::getFrameJacobian(model_, data, frame_id, pin::LOCAL_WORLD_ALIGNED,
                        frame_jacobian);
  const Eigen::Vector3d offset_world =
      data.oMf[frame_id].rotation() * offset_local;
  return frame_jacobian.topRows<3>() -
         skew(offset_world) * frame_jacobian.bottomRows<3>();
}

Eigen::Vector3d PinocchioModel::contactMaterialAccelerationBias(
    const Eigen::VectorXd& q, const Eigen::VectorXd& v, LegId leg,
    const Eigen::Vector3d& offset_local) const {
  if (q.size() != model_.nq || v.size() != model_.nv || !q.allFinite() ||
      !v.allFinite()) {
    throw std::invalid_argument("invalid q or v for contact acceleration");
  }
  pin::Data data(model_);
  const Eigen::VectorXd zero_acceleration = Eigen::VectorXd::Zero(model_.nv);
  pin::forwardKinematics(model_, data, q, v, zero_acceleration);
  pin::updateFramePlacements(model_, data);

  const pin::FrameIndex frame_id = wheels_.at(legIndex(leg)).frame_id;
  const auto velocity = pin::getFrameVelocity(
      model_, data, frame_id, pin::LOCAL_WORLD_ALIGNED);
  const auto acceleration = pin::getFrameClassicalAcceleration(
      model_, data, frame_id, pin::LOCAL_WORLD_ALIGNED);
  const Eigen::Vector3d offset_world =
      data.oMf[frame_id].rotation() * offset_local;

  return acceleration.linear() + acceleration.angular().cross(offset_world) +
         velocity.angular().cross(
             velocity.angular().cross(offset_world));
}

}  // namespace stand_wheel
