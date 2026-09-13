#pragma once

#include "stand_wheel/full_dynamics.hpp"

#include <Eigen/Core>

#include <vector>

namespace stand_wheel {

struct WheelContactFrame {
  LegId leg = LegId::FL;
  Eigen::Vector3d wheel_center_world = Eigen::Vector3d::Zero();
  Eigen::Vector3d contact_point_world = Eigen::Vector3d::Zero();
  Eigen::Vector3d axle_world = Eigen::Vector3d::Zero();
  Eigen::Vector3d rolling_world = Eigen::Vector3d::Zero();
  Eigen::Vector3d lateral_world = Eigen::Vector3d::Zero();
  Eigen::Vector3d normal_world = Eigen::Vector3d::Zero();

  // Columns are [rolling, lateral, normal], all expressed in world axes.
  Eigen::Matrix3d rotation_world_from_contact = Eigen::Matrix3d::Identity();

  // The force Jacobian is the Step 1 material-point Jacobian used only for
  // generalized contact force. The carrier Jacobian removes this wheel's own
  // spin and is used to construct the nonholonomic rolling constraint.
  Eigen::MatrixXd force_jacobian_world;
  Eigen::MatrixXd carrier_jacobian_world;
  int wheel_velocity_index = -1;
};

struct RollingConstraintData {
  // Three rows per active contact, preserving caller order:
  // [normal velocity, lateral velocity, rolling velocity-r*wheel speed].
  ActiveContacts active_contacts;
  std::vector<WheelContactFrame> frames;
  Eigen::MatrixXd matrix;
  Eigen::VectorXd residual;
};

struct SkidSteerConstraintData {
  // Hard rows per active contact: [normal, rolling-r*wheel_speed].
  // Lateral rows are deliberately exposed separately as a soft residual.
  ActiveContacts active_contacts;
  std::vector<WheelContactFrame> frames;
  Eigen::MatrixXd hard_matrix;
  Eigen::VectorXd hard_residual;
  Eigen::MatrixXd lateral_matrix;
  Eigen::VectorXd lateral_slip;
};

struct FrictionConfig {
  FrictionConfig(double rolling_coefficient, double lateral_coefficient)
      : mu_roll(rolling_coefficient), mu_lat(lateral_coefficient) {}

  double mu_roll;
  double mu_lat;
};

struct FrictionFeasibility {
  // Five nonnegative margins per force block [f_roll,f_lat,f_normal]:
  // [f_normal,
  //  mu_roll*f_normal-f_roll, mu_roll*f_normal+f_roll,
  //  mu_lat*f_normal-f_lat,   mu_lat*f_normal+f_lat].
  Eigen::VectorXd margins;
  Eigen::VectorXd violations;
  bool feasible = false;
};

class WheelContactModel {
 public:
  explicit WheelContactModel(PinocchioModel& robot) : robot_(robot) {}

  RollingConstraintData evaluate(
      const Eigen::VectorXd& q, const Eigen::VectorXd& v,
      const ActiveContacts& active_contacts);

  Eigen::MatrixXd constraintMatrix(
      const Eigen::VectorXd& q,
      const ActiveContacts& active_contacts);

  Eigen::VectorXd residual(const Eigen::VectorXd& q,
                           const Eigen::VectorXd& v,
                           const ActiveContacts& active_contacts);

  SkidSteerConstraintData skidSteer(
      const Eigen::VectorXd& q, const Eigen::VectorXd& v,
      const ActiveContacts& active_contacts);

  Eigen::MatrixXd hardConstraintMatrix(
      const Eigen::VectorXd& q,
      const ActiveContacts& active_contacts);

  Eigen::VectorXd hardResidual(
      const Eigen::VectorXd& q, const Eigen::VectorXd& v,
      const ActiveContacts& active_contacts);

  Eigen::VectorXd lateralSlip(
      const Eigen::VectorXd& q, const Eigen::VectorXd& v,
      const ActiveContacts& active_contacts);

  // Block-diagonal R_WC=[e_roll,e_lat,e_normal], preserving contact order.
  Eigen::MatrixXd forceTransformWorldFromWheel(
      const Eigen::VectorXd& q,
      const ActiveContacts& active_contacts);

  Eigen::VectorXd wheelForcesToWorld(
      const Eigen::VectorXd& q,
      const ActiveContacts& active_contacts,
      const Eigen::Ref<const Eigen::VectorXd>& lambda_wheel);

  Eigen::VectorXd worldForcesToWheel(
      const Eigen::VectorXd& q,
      const ActiveContacts& active_contacts,
      const Eigen::Ref<const Eigen::VectorXd>& lambda_world);

  static FrictionFeasibility checkFriction(
      const Eigen::Ref<const Eigen::VectorXd>& lambda_wheel,
      const FrictionConfig& config, double tolerance = 1e-12);

 private:
  static void validateActiveContacts(const ActiveContacts& active_contacts);
  void validateVelocity(const Eigen::VectorXd& v) const;
  WheelContactFrame makeFrame(LegId leg,
                              const ContactKinematics& contact) const;

  PinocchioModel& robot_;
};

}  // namespace stand_wheel
