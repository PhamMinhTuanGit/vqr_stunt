#include "stand_wheel/pinocchio_model.hpp"

#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
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

double manualComError(const pin::Model& model, const Eigen::VectorXd& q,
                      const Eigen::Vector3d& reference) {
  pin::Data data(model);
  pin::forwardKinematics(model, data, q);
  double total_mass = 0.0;
  Eigen::Vector3d weighted = Eigen::Vector3d::Zero();
  for (pin::JointIndex joint = 1; joint < model.njoints; ++joint) {
    const double mass = model.inertias[joint].mass();
    const Eigen::Vector3d body_com =
        data.oMi[joint].translation() +
        data.oMi[joint].rotation() * model.inertias[joint].lever();
    total_mass += mass;
    weighted += mass * body_com;
  }
  return (weighted / total_mass - reference).norm();
}

}  // namespace

int main(int argc, char** argv) {
  const std::string urdf =
      argc > 1 ? argv[1] : STAND_WHEEL_DEFAULT_URDF_PATH;
  sw::PinocchioModel robot({urdf});
  const pin::Model& model = robot.model();
  bool passed = true;

  std::cout << std::fixed << std::setprecision(9);
  std::cout << "MODEL nq=" << robot.nq() << " nv=" << robot.nv()
            << " total_mass=" << robot.totalMass() << " kg\n";
  passed &= check(robot.nq() == 27, "floating-base nq is 27");
  passed &= check(robot.nv() == 22, "floating-base nv is 22");
  passed &= check(std::abs(robot.totalMass() - 32.86) < 1e-10,
                  "URDF total mass is 32.86 kg");
  passed &= check(std::abs(robot.wheelRadius() - 0.091) < 1e-12,
                  "configured wheel radius matches the 0.091 m URDF tread");

  std::cout << "\nJOINT ORDERING (Pinocchio model order)\n";
  for (const auto& joint : robot.jointOrdering()) {
    std::cout << "  " << joint.joint_id << " " << joint.name
              << " q=[" << joint.idx_q << "," << joint.nq << "]"
              << " v=[" << joint.idx_v << "," << joint.nv << "]\n";
  }

  const std::array<std::string, sw::kNumLegs> expected_labels = {
      "FL", "FR", "HL", "HR"};
  std::cout << "\nCANONICAL WHEEL/CONTACT MAPPING\n";
  for (int i = 0; i < sw::kNumLegs; ++i) {
    const auto& wheel = robot.wheels()[i];
    std::cout << "  " << wheel.label << " wheel_joint="
              << wheel.wheel_joint.name << " joint_id="
              << wheel.wheel_joint.joint_id << " idx_v="
              << wheel.wheel_joint.idx_v << " frame=" << wheel.wheel_frame
              << " frame_id=" << wheel.frame_id << '\n';
    passed &= check(wheel.label == expected_labels[i],
                    "canonical mapping slot " + std::to_string(i) +
                        " is " + expected_labels[i]);
    passed &= check(wheel.wheel_joint.nq == 2 && wheel.wheel_joint.nv == 1,
                    wheel.label + " continuous wheel uses nq=2, nv=1");
  }

  const Eigen::VectorXd q_nominal = robot.nominalConfiguration();
  const sw::ModelSnapshot nominal = robot.evaluate(q_nominal);
  std::cout << "\nNOMINAL CONFIGURATION\n";
  std::cout << "  base_xyz=" << q_nominal.head<3>().transpose() << '\n';
  std::cout << "  CoM=" << nominal.center_of_mass_world.transpose() << '\n';
  passed &= check(nominal.center_of_mass_world.allFinite(), "CoM is finite");

  double max_contact_height = 0.0;
  double max_radius_error = 0.0;
  double max_axis_offset_dot = 0.0;
  double max_axis_mapping_error = 0.0;
  pin::Data nominal_data(model);
  pin::forwardKinematics(model, nominal_data, q_nominal);
  pin::updateFramePlacements(model, nominal_data);
  pin::computeJointJacobians(model, nominal_data, q_nominal);
  for (int i = 0; i < sw::kNumLegs; ++i) {
    const auto& contact = nominal.contacts[i];
    std::cout << "  " << expected_labels[i]
              << " center=" << contact.wheel_center_world.transpose()
              << " contact=" << contact.position_world.transpose()
              << " axis=" << contact.wheel_axis_world.transpose() << '\n';
    max_contact_height =
        std::max(max_contact_height, std::abs(contact.position_world.z()));
    max_radius_error =
        std::max(max_radius_error,
                 std::abs(contact.offset_world.norm() - robot.wheelRadius()));
    max_axis_offset_dot =
        std::max(max_axis_offset_dot,
                 std::abs(contact.wheel_axis_world.dot(contact.offset_world)));
    Eigen::MatrixXd frame_jacobian = Eigen::MatrixXd::Zero(6, robot.nv());
    pin::getFrameJacobian(model, nominal_data, robot.wheels()[i].frame_id,
                          pin::LOCAL_WORLD_ALIGNED, frame_jacobian);
    const Eigen::Vector3d actual_axis =
        frame_jacobian.bottomRows<3>()
            .col(robot.wheels()[i].wheel_joint.idx_v)
            .normalized();
    max_axis_mapping_error =
        std::max(max_axis_mapping_error,
                 (actual_axis - contact.wheel_axis_world).norm());
  }
  passed &= check(max_contact_height < 1e-12,
                  "nominal contacts lie on z=0");
  passed &= check(max_radius_error < 1e-12,
                  "contact offsets equal wheel radius");
  passed &= check(max_axis_offset_dot < 1e-12,
                  "contact offsets are perpendicular to wheel axes");
  passed &= check(max_axis_mapping_error < 1e-12,
                  "configured wheel axes match URDF joint motion axes");
  passed &= check(nominal.contacts[0].position_world.x() > 0.0 &&
                      nominal.contacts[1].position_world.x() > 0.0 &&
                      nominal.contacts[2].position_world.x() < 0.0 &&
                      nominal.contacts[3].position_world.x() < 0.0 &&
                      nominal.contacts[0].position_world.y() > 0.0 &&
                      nominal.contacts[2].position_world.y() > 0.0 &&
                      nominal.contacts[1].position_world.y() < 0.0 &&
                      nominal.contacts[3].position_world.y() < 0.0,
                  "FL/FR/HL/HR contact quadrants are correct");

  const double com_error = manualComError(
      model, q_nominal, nominal.center_of_mass_world);
  passed &= check(com_error < 1e-12,
                  "CoM matches independent mass-weighted body CoM; error=" +
                      std::to_string(com_error));

  const double symmetry_error =
      (nominal.mass_matrix - nominal.mass_matrix.transpose()).norm();
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigensolver(
      nominal.mass_matrix);
  const double min_eigenvalue = eigensolver.eigenvalues().minCoeff();
  const double max_eigenvalue = eigensolver.eigenvalues().maxCoeff();

  pin::Data rnea_data(model);
  const Eigen::VectorXd zero = Eigen::VectorXd::Zero(robot.nv());
  const Eigen::VectorXd bias = pin::rnea(model, rnea_data, q_nominal, zero, zero);
  double max_mass_column_error = 0.0;
  for (int col = 0; col < robot.nv(); ++col) {
    Eigen::VectorXd acceleration = zero;
    acceleration[col] = 1.0;
    const Eigen::VectorXd tau =
        pin::rnea(model, rnea_data, q_nominal, zero, acceleration);
    max_mass_column_error =
        std::max(max_mass_column_error,
                 (tau - bias - nominal.mass_matrix.col(col)).cwiseAbs().maxCoeff());
  }
  std::cout << std::scientific << std::setprecision(6);
  std::cout << "\nMASS MATRIX VALIDATION\n"
            << "  symmetry_error=" << symmetry_error
            << " min_eigenvalue=" << min_eigenvalue
            << " max_eigenvalue=" << max_eigenvalue
            << " rnea_column_max_error=" << max_mass_column_error << '\n';
  passed &= check(nominal.mass_matrix.rows() == robot.nv() &&
                      nominal.mass_matrix.cols() == robot.nv() &&
                      nominal.mass_matrix.allFinite(),
                  "mass matrix is finite and 22 x 22");
  passed &= check(symmetry_error < 1e-12, "mass matrix is symmetric");
  passed &= check(eigensolver.info() == Eigen::Success && min_eigenvalue > 1e-6,
                  "mass matrix is positive definite");
  passed &= check(max_mass_column_error < 1e-10,
                  "CRBA mass matrix columns match RNEA");

  Eigen::VectorXd tangent = Eigen::VectorXd::Zero(robot.nv());
  tangent.head<3>() << 0.03, -0.02, 0.04;
  tangent.segment<3>(3) << 0.08, -0.05, 0.03;
  for (int i = 0; i < 12; ++i) {
    tangent[robot.legJoints()[i].idx_v] =
        0.025 * std::sin(0.7 * static_cast<double>(i + 1));
  }
  for (int i = 0; i < sw::kNumLegs; ++i) {
    tangent[robot.wheels()[i].wheel_joint.idx_v] = 0.11 * (i + 1);
  }
  const Eigen::VectorXd q_test = pin::integrate(model, q_nominal, tangent);
  const sw::ModelSnapshot test_snapshot = robot.evaluate(q_test);

  constexpr double kEpsilon = 1e-6;
  double global_jacobian_error = 0.0;
  int min_contact_rank = 3;
  bool jacobians_finite_and_sized = true;
  std::cout << "\nCONTACT JACOBIAN VALIDATION (central difference)\n";
  for (int leg = 0; leg < sw::kNumLegs; ++leg) {
    const auto leg_id = static_cast<sw::LegId>(leg);
    const auto& contact = test_snapshot.contacts[leg];
    jacobians_finite_and_sized &=
        contact.jacobian.rows() == 3 &&
        contact.jacobian.cols() == robot.nv() && contact.jacobian.allFinite();
    double max_error = 0.0;
    int worst_dof = -1;
    for (int dof = 0; dof < robot.nv(); ++dof) {
      Eigen::VectorXd delta = Eigen::VectorXd::Zero(robot.nv());
      delta[dof] = kEpsilon;
      const Eigen::VectorXd q_plus = pin::integrate(model, q_test, delta);
      const Eigen::VectorXd q_minus = pin::integrate(model, q_test, -delta);
      const Eigen::Vector3d numerical =
          (robot.contactMaterialPosition(q_plus, leg_id, contact.offset_local) -
           robot.contactMaterialPosition(q_minus, leg_id, contact.offset_local)) /
          (2.0 * kEpsilon);
      const double error =
          (numerical - contact.jacobian.col(dof)).norm();
      if (error > max_error) {
        max_error = error;
        worst_dof = dof;
      }
    }
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(contact.jacobian);
    const int rank = static_cast<int>((svd.singularValues().array() > 1e-8).count());
    min_contact_rank = std::min(min_contact_rank, rank);
    global_jacobian_error = std::max(global_jacobian_error, max_error);
    std::cout << "  " << expected_labels[leg] << " max_error=" << max_error
              << " worst_dof=" << worst_dof << " rank=" << rank << '\n';
  }
  passed &= check(global_jacobian_error < 1e-6,
                  "all contact Jacobians match finite differences");
  passed &= check(jacobians_finite_and_sized,
                  "all contact Jacobians are finite and 3 x 22");
  passed &= check(min_contact_rank == 3,
                  "each 3D contact Jacobian has row rank 3");

  std::cout << '\n' << (passed ? "SMOKE TEST PASSED" : "SMOKE TEST FAILED")
            << '\n';
  return passed ? 0 : 1;
}
