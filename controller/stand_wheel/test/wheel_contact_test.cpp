#include "stand_wheel/wheel_contact.hpp"

#include <Eigen/QR>
#include <pinocchio/algorithm/joint-configuration.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>
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
    result[i] = scale * std::sin(phase + 0.61 * static_cast<double>(i + 1));
  }
  return result;
}

}  // namespace

int main(int argc, char** argv) {
  const std::string urdf =
      argc > 1 ? argv[1] : STAND_WHEEL_DEFAULT_URDF_PATH;
  sw::PinocchioModel robot({urdf});
  sw::WheelContactModel rolling(robot);
  const pin::Model& model = robot.model();
  bool passed = true;

  const Eigen::VectorXd q_nominal = robot.nominalConfiguration();
  const Eigen::VectorXd v_zero = Eigen::VectorXd::Zero(robot.nv());
  const sw::ActiveContacts all_contacts = {
      sw::LegId::FL, sw::LegId::FR, sw::LegId::HL, sw::LegId::HR};
  const sw::RollingConstraintData nominal =
      rolling.evaluate(q_nominal, v_zero, all_contacts);

  std::cout << std::scientific << std::setprecision(6);
  std::cout << "NOMINAL WHEEL CONTACT FRAMES\n";
  double max_unit_error = 0.0;
  double max_orthogonality_error = 0.0;
  double max_roll_axis_dot = 0.0;
  double max_roll_normal_dot = 0.0;
  double max_handedness_error = 0.0;
  double max_force_spin_error = 0.0;
  for (const sw::WheelContactFrame& frame : nominal.frames) {
    const Eigen::Matrix3d gram =
        frame.rotation_world_from_contact.transpose() *
        frame.rotation_world_from_contact;
    max_unit_error =
        std::max({max_unit_error,
                  std::abs(frame.axle_world.norm() - 1.0),
                  std::abs(frame.rolling_world.norm() - 1.0),
                  std::abs(frame.lateral_world.norm() - 1.0),
                  std::abs(frame.normal_world.norm() - 1.0)});
    max_orthogonality_error =
        std::max(max_orthogonality_error,
                 (gram - Eigen::Matrix3d::Identity()).cwiseAbs().maxCoeff());
    max_roll_axis_dot =
        std::max(max_roll_axis_dot,
                 std::abs(frame.rolling_world.dot(frame.axle_world)));
    max_roll_normal_dot =
        std::max(max_roll_normal_dot,
                 std::abs(frame.rolling_world.dot(frame.normal_world)));
    max_handedness_error =
        std::max(max_handedness_error,
                 std::abs(frame.rotation_world_from_contact.determinant() - 1.0));
    const Eigen::Vector3d expected_spin =
        -robot.wheelRadius() * frame.rolling_world;
    max_force_spin_error =
        std::max(max_force_spin_error,
                 (frame.force_jacobian_world.col(frame.wheel_velocity_index) -
                  expected_spin)
                     .norm());
    std::cout << "  " << sw::legLabel(frame.leg)
              << " axle=" << frame.axle_world.transpose()
              << " roll=" << frame.rolling_world.transpose()
              << " lat=" << frame.lateral_world.transpose()
              << " normal=" << frame.normal_world.transpose()
              << " contact=" << frame.contact_point_world.transpose() << '\n';
  }
  std::cout << "DIRECTION unit_max_error=" << max_unit_error
            << " orthogonality_max_error=" << max_orthogonality_error
            << " roll_axis_max_dot=" << max_roll_axis_dot
            << " roll_normal_max_dot=" << max_roll_normal_dot
            << " handedness_error=" << max_handedness_error
            << " force_spin_max_error=" << max_force_spin_error << '\n';
  passed &= check(max_unit_error < 1e-12,
                  "axle/rolling/lateral/normal directions are unit length");
  passed &= check(max_orthogonality_error < 1e-12,
                  "rolling/lateral/normal contact frames are orthonormal");
  passed &= check(max_roll_axis_dot < 1e-12 && max_roll_normal_dot < 1e-12,
                  "rolling direction is perpendicular to axle and ground normal");
  passed &= check(max_handedness_error < 1e-12,
                  "rolling/lateral/normal contact frames are right-handed");
  passed &= check(max_force_spin_error < 1e-12,
                  "material-point spin velocity is -radius*rolling direction");
  passed &= check(nominal.residual.norm() == 0.0,
                  "four-wheel nominal static rolling residual is zero");

  // A. Positive wheel speed requires carrier motion +r*qdot along e_roll.
  const sw::ActiveContacts fl_only = {sw::LegId::FL};
  const sw::RollingConstraintData fl_geometry =
      rolling.evaluate(q_nominal, v_zero, fl_only);
  const sw::WheelContactFrame& fl = fl_geometry.frames.front();
  const sw::JointMapping root = findJoint(robot, "root_joint");
  const Eigen::Matrix3d base_linear_map =
      fl_geometry.matrix.block<3, 3>(0, root.idx_v);
  constexpr double kWheelSpeed = 3.25;
  Eigen::VectorXd pure_rolling = v_zero;
  pure_rolling[fl.wheel_velocity_index] = kWheelSpeed;
  pure_rolling.segment<3>(root.idx_v) =
      base_linear_map.colPivHouseholderQr().solve(
          -fl_geometry.matrix.col(fl.wheel_velocity_index) * kWheelSpeed);
  const sw::RollingConstraintData pure_result =
      rolling.evaluate(q_nominal, pure_rolling, fl_only);
  const double pure_carrier_speed =
      fl.rolling_world.dot(fl.carrier_jacobian_world * pure_rolling);
  const double expected_carrier_speed = robot.wheelRadius() * kWheelSpeed;
  std::cout << "PURE ROLLING qdot=" << kWheelSpeed
            << " carrier_roll_speed=" << pure_carrier_speed
            << " expected=" << expected_carrier_speed
            << " residual=" << pure_result.residual.transpose() << '\n';
  passed &= check(pure_result.residual.norm() < 1e-12,
                  "pure FL rolling satisfies all three constraints");
  passed &= check(std::abs(pure_carrier_speed - expected_carrier_speed) < 1e-12,
                  "pure rolling carrier speed equals radius*wheel speed");

  // B. Keep the carrier velocity fixed and perturb only wheel speed.
  constexpr double kWheelSpeedError = 0.7;
  Eigen::VectorXd wrong_speed = pure_rolling;
  wrong_speed[fl.wheel_velocity_index] += kWheelSpeedError;
  const Eigen::VectorXd wrong_speed_residual =
      rolling.residual(q_nominal, wrong_speed, fl_only);
  const double expected_wrong_rolling =
      -robot.wheelRadius() * kWheelSpeedError;
  std::cout << "WRONG SPEED delta_qdot=" << kWheelSpeedError
            << " rolling_residual=" << wrong_speed_residual[2]
            << " expected=" << expected_wrong_rolling << '\n';
  passed &= check(std::abs(wrong_speed_residual[2] - expected_wrong_rolling) <
                      1e-12,
                  "wrong wheel speed has expected rolling residual sign/magnitude");

  // C/D. Solve only for free-flyer translation that creates prescribed local
  // contact-frame velocity. This avoids assuming world X/Y rolling directions.
  constexpr double kLateralSlip = 0.19;
  Eigen::VectorXd lateral_slip = v_zero;
  Eigen::Vector3d lateral_target(0.0, kLateralSlip, 0.0);
  lateral_slip.segment<3>(root.idx_v) =
      base_linear_map.colPivHouseholderQr().solve(lateral_target);
  const Eigen::VectorXd lateral_residual =
      rolling.residual(q_nominal, lateral_slip, fl_only);
  std::cout << "LATERAL SLIP residual=" << lateral_residual.transpose() << '\n';
  passed &= check((lateral_residual - lateral_target).norm() < 1e-12,
                  "lateral row detects prescribed slip velocity");

  constexpr double kNormalVelocity = -0.13;
  Eigen::VectorXd penetration = v_zero;
  Eigen::Vector3d normal_target(kNormalVelocity, 0.0, 0.0);
  penetration.segment<3>(root.idx_v) =
      base_linear_map.colPivHouseholderQr().solve(normal_target);
  const Eigen::VectorXd normal_residual =
      rolling.residual(q_nominal, penetration, fl_only);
  std::cout << "GROUND NORMAL residual=" << normal_residual.transpose() << '\n';
  passed &= check((normal_residual - normal_target).norm() < 1e-12,
                  "normal row detects prescribed penetration velocity");

  // F. Validate every column of A_roll against finite differences of the same
  // fixed material point used by the Step 1 force Jacobian.
  const Eigen::VectorXd q_offset =
      deterministicVector(robot.nv(), 0.035, -0.3);
  const Eigen::VectorXd q_test = pin::integrate(model, q_nominal, q_offset);
  const sw::RollingConstraintData test_geometry =
      rolling.evaluate(q_test, v_zero, all_contacts);
  const sw::ModelSnapshot model_snapshot = robot.evaluate(q_test);
  constexpr double kFiniteDifferenceEpsilon = 1e-6;
  double max_finite_difference_error = 0.0;
  int worst_contact = 0;
  int worst_dof = 0;
  for (int contact_index = 0; contact_index < sw::kNumLegs; ++contact_index) {
    const sw::LegId leg = all_contacts[contact_index];
    const int leg_index = static_cast<int>(leg);
    const sw::WheelContactFrame& frame = test_geometry.frames[contact_index];
    const Eigen::Vector3d offset_local =
        model_snapshot.contacts[leg_index].offset_local;
    for (int dof = 0; dof < robot.nv(); ++dof) {
      Eigen::VectorXd delta = Eigen::VectorXd::Zero(robot.nv());
      delta[dof] = kFiniteDifferenceEpsilon;
      const Eigen::VectorXd q_plus = pin::integrate(model, q_test, delta);
      const Eigen::VectorXd q_minus = pin::integrate(model, q_test, -delta);
      const Eigen::Vector3d velocity_numerical =
          (robot.contactMaterialPosition(q_plus, leg, offset_local) -
           robot.contactMaterialPosition(q_minus, leg, offset_local)) /
          (2.0 * kFiniteDifferenceEpsilon);
      Eigen::Vector3d projected_numerical;
      projected_numerical << frame.normal_world.dot(velocity_numerical),
          frame.lateral_world.dot(velocity_numerical),
          frame.rolling_world.dot(velocity_numerical);
      const double error =
          (projected_numerical -
           test_geometry.matrix.block<3, 1>(3 * contact_index, dof))
              .norm();
      if (error > max_finite_difference_error) {
        max_finite_difference_error = error;
        worst_contact = contact_index;
        worst_dof = dof;
      }
    }
  }
  std::cout << "FINITE DIFFERENCE A_roll_max_error="
            << max_finite_difference_error
            << " worst_contact=" << sw::legLabel(all_contacts[worst_contact])
            << " worst_dof=" << worst_dof << '\n';
  passed &= check(max_finite_difference_error < 1e-7,
                  "A_roll columns match projected material-point finite differences");

  std::cout << (passed ? "WHEEL CONTACT TEST PASSED"
                       : "WHEEL CONTACT TEST FAILED")
            << '\n';
  return passed ? 0 : 1;
}
