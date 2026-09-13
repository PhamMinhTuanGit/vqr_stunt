#pragma once

#include <Eigen/Core>
#include <pinocchio/multibody/model.hpp>

#include <array>
#include <string>
#include <vector>

namespace stand_wheel {

constexpr int kNumLegs = 4;
constexpr int kLegJointsPerLeg = 3;
constexpr int kNumActuatedJoints = 16;

enum class LegId : int { FL = 0, FR = 1, HL = 2, HR = 3 };

struct ModelConfig {
  std::string urdf_path;
  double wheel_radius = 0.091;
  Eigen::Vector3d wheel_axis_local = -Eigen::Vector3d::UnitY();
  Eigen::Vector3d ground_normal_world = Eigen::Vector3d::UnitZ();
};

struct JointMapping {
  std::string name;
  pinocchio::JointIndex joint_id = 0;
  int idx_q = -1;
  int nq = 0;
  int idx_v = -1;
  int nv = 0;
};

struct WheelContactMapping {
  LegId leg = LegId::FL;
  std::string label;
  JointMapping wheel_joint;
  std::string wheel_frame;
  pinocchio::FrameIndex frame_id = 0;
};

struct ContactKinematics {
  Eigen::Vector3d wheel_center_world = Eigen::Vector3d::Zero();
  Eigen::Vector3d position_world = Eigen::Vector3d::Zero();
  Eigen::Vector3d wheel_axis_world = Eigen::Vector3d::Zero();
  Eigen::Vector3d offset_world = Eigen::Vector3d::Zero();
  Eigen::Vector3d offset_local = Eigen::Vector3d::Zero();
  Eigen::MatrixXd jacobian;
};

struct ModelSnapshot {
  Eigen::Vector3d center_of_mass_world = Eigen::Vector3d::Zero();
  Eigen::MatrixXd mass_matrix;
  std::array<ContactKinematics, kNumLegs> contacts;
};

// Name-resolved Pinocchio view of VQRWheel. The Pinocchio model ordering is
// preserved for dynamics; canonical FL/FR/HL/HR lookup is provided separately.
class PinocchioModel {
 public:
  explicit PinocchioModel(ModelConfig config);

  const pinocchio::Model& model() const { return model_; }
  int nq() const { return model_.nq; }
  int nv() const { return model_.nv; }
  double totalMass() const { return total_mass_; }
  double wheelRadius() const { return config_.wheel_radius; }
  const Eigen::Vector3d& groundNormalWorld() const {
    return config_.ground_normal_world;
  }
  const Eigen::Vector3d& wheelAxisLocal() const {
    return config_.wheel_axis_local;
  }

  const std::array<JointMapping, 12>& legJoints() const {
    return leg_joints_;
  }
  const std::array<WheelContactMapping, kNumLegs>& wheels() const {
    return wheels_;
  }
  std::vector<JointMapping> jointOrdering() const;

  // Symmetric, four-wheel stance used only for audit/test. Base z is solved so
  // the four idealized centerline tread contacts lie on z=0.
  Eigen::VectorXd nominalConfiguration() const;

  ModelSnapshot evaluate(const Eigen::VectorXd& q);

  // Position of the same wheel material point after changing q. This is the
  // correct quantity for finite-difference validation of a point Jacobian.
  Eigen::Vector3d contactMaterialPosition(
      const Eigen::VectorXd& q, LegId leg,
      const Eigen::Vector3d& offset_local) const;

  Eigen::MatrixXd contactMaterialJacobian(
      const Eigen::VectorXd& q, LegId leg,
      const Eigen::Vector3d& offset_local) const;

  // Jdot(q,v)*v for the same fixed material point, expressed in the
  // LOCAL_WORLD_ALIGNED (world-axis) coordinates used by its Jacobian.
  Eigen::Vector3d contactMaterialAccelerationBias(
      const Eigen::VectorXd& q, const Eigen::VectorXd& v, LegId leg,
      const Eigen::Vector3d& offset_local) const;

 private:
  JointMapping resolveJoint(const std::string& name) const;
  ContactKinematics computeContact(pinocchio::FrameIndex frame_id);
  void setScalarJoint(Eigen::VectorXd& q, const std::string& name,
                      double value) const;

  ModelConfig config_;
  pinocchio::Model model_;
  pinocchio::Data data_;
  double total_mass_ = 0.0;
  JointMapping root_joint_;
  std::array<JointMapping, 12> leg_joints_;
  std::array<WheelContactMapping, kNumLegs> wheels_;
};

const char* legLabel(LegId leg);

}  // namespace stand_wheel
