#include "stand_wheel/wheel_contact.hpp"

#include <Eigen/Core>
#include <Eigen/SVD>

#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#ifndef STAND_WHEEL_DEFAULT_URDF_PATH
#define STAND_WHEEL_DEFAULT_URDF_PATH \
  "vqr_description/vqr_urdf/urdf/VQRWheel.urdf"
#endif

namespace sw = stand_wheel;

namespace {

constexpr double kDefaultYawRateTarget = 0.5;
constexpr double kRankThreshold = 1e-10;
constexpr double kFeasibleTolerance = 1e-10;

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
  throw std::runtime_error("yaw audit could not find joint: " + name);
}

int matrixRank(const Eigen::MatrixXd& matrix) {
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(matrix);
  svd.setThreshold(kRankThreshold);
  return svd.rank();
}

Eigen::MatrixXd selectColumns(const Eigen::MatrixXd& matrix,
                              const std::vector<int>& indices) {
  Eigen::MatrixXd selected(matrix.rows(), indices.size());
  for (std::size_t i = 0; i < indices.size(); ++i) {
    selected.col(static_cast<int>(i)) = matrix.col(indices[i]);
  }
  return selected;
}

struct LeastSquaresResult {
  Eigen::VectorXd velocity;
  Eigen::VectorXd free_solution;
  Eigen::VectorXd residual;
  int free_rank = 0;
  int augmented_rank = 0;
};

LeastSquaresResult solveFreeVelocities(
    const Eigen::MatrixXd& constraint,
    const Eigen::VectorXd& fixed_velocity,
    const std::vector<int>& free_indices, int target_index) {
  const Eigen::MatrixXd free_matrix =
      selectColumns(constraint, free_indices);
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(
      free_matrix, Eigen::ComputeThinU | Eigen::ComputeThinV);
  svd.setThreshold(kRankThreshold);

  LeastSquaresResult result;
  result.free_solution = svd.solve(-constraint * fixed_velocity);
  result.velocity = fixed_velocity;
  for (std::size_t i = 0; i < free_indices.size(); ++i) {
    result.velocity[free_indices[i]] = result.free_solution[static_cast<int>(i)];
  }
  result.residual = constraint * result.velocity;
  result.free_rank = svd.rank();

  Eigen::MatrixXd augmented(constraint.rows(), free_matrix.cols() + 1);
  augmented.leftCols(free_matrix.cols()) = free_matrix;
  augmented.rightCols(1) = constraint.col(target_index);
  result.augmented_rank = matrixRank(augmented);
  return result;
}

void printResiduals(const std::string& label,
                    const Eigen::VectorXd& residual) {
  std::cout << label << " total_norm=" << residual.norm() << '\n';
  for (int leg = 0; leg < sw::kNumLegs; ++leg) {
    const Eigen::Vector3d block = residual.segment<3>(3 * leg);
    std::cout << "  " << sw::legLabel(static_cast<sw::LegId>(leg))
              << " normal=" << block[0]
              << " lateral=" << block[1]
              << " rolling=" << block[2] << '\n';
  }
}

double maximumBlockComponent(const Eigen::VectorXd& residual,
                             int block_offset) {
  double maximum = 0.0;
  for (int leg = 0; leg < sw::kNumLegs; ++leg) {
    maximum = std::max(maximum,
                       std::abs(residual[3 * leg + block_offset]));
  }
  return maximum;
}

}  // namespace

int main(int argc, char** argv) {
  const std::string urdf =
      argc > 1 ? argv[1] : STAND_WHEEL_DEFAULT_URDF_PATH;
  const double yaw_rate_target =
      argc > 2 ? std::stod(argv[2]) : kDefaultYawRateTarget;
  if (!std::isfinite(yaw_rate_target) || std::abs(yaw_rate_target) < 1e-12) {
    throw std::invalid_argument("yaw-rate target must be finite and nonzero");
  }
  sw::PinocchioModel robot({urdf});
  sw::WheelContactModel wheel_contact(robot);
  const sw::ActiveContacts contacts = {
      sw::LegId::FL, sw::LegId::FR, sw::LegId::HL, sw::LegId::HR};
  const Eigen::VectorXd q = robot.nominalConfiguration();
  const Eigen::MatrixXd constraint =
      wheel_contact.constraintMatrix(q, contacts);

  const sw::JointMapping root = findJoint(robot, "root_joint");
  if (root.nv != 6) {
    throw std::runtime_error("yaw audit requires a six-DoF free flyer");
  }
  // Pinocchio free-flyer tangent layout is local linear xyz then angular xyz.
  // The nominal base orientation is identity, so these also equal world axes.
  const int base_vx = root.idx_v;
  const int base_vy = root.idx_v + 1;
  const int base_yaw = root.idx_v + 5;

  std::vector<int> wheel_indices;
  wheel_indices.reserve(sw::kNumLegs);
  for (const sw::WheelContactMapping& wheel : robot.wheels()) {
    wheel_indices.push_back(wheel.wheel_joint.idx_v);
  }
  std::vector<int> leg_indices;
  leg_indices.reserve(sw::kNumLegs * sw::kLegJointsPerLeg);
  for (const sw::JointMapping& joint : robot.legJoints()) {
    leg_indices.push_back(joint.idx_v);
  }

  bool passed = true;
  passed &= check(constraint.rows() == 12 && constraint.cols() == robot.nv() &&
                      constraint.allFinite(),
                  "four-wheel A_roll is finite and has shape 12x22");

  Eigen::VectorXd fixed = Eigen::VectorXd::Zero(robot.nv());
  fixed[base_yaw] = yaw_rate_target;

  const LeastSquaresResult case_a = solveFreeVelocities(
      constraint, fixed, wheel_indices, base_yaw);
  std::cout << std::scientific << std::setprecision(9);
  std::cout << "CASE A yaw=" << yaw_rate_target
            << " wheel_rates=" << case_a.free_solution.transpose() << '\n';
  printResiduals("CASE A residual", case_a.residual);

  std::vector<int> case_b_indices = {base_vx, base_vy};
  case_b_indices.insert(case_b_indices.end(), wheel_indices.begin(),
                        wheel_indices.end());
  const LeastSquaresResult case_b = solveFreeVelocities(
      constraint, fixed, case_b_indices, base_yaw);
  std::cout << "CASE B base_vx=" << case_b.velocity[base_vx]
            << " base_vy=" << case_b.velocity[base_vy]
            << " wheel_rates=";
  for (int index : wheel_indices) {
    std::cout << ' ' << case_b.velocity[index];
  }
  std::cout << '\n';
  printResiduals("CASE B residual", case_b.residual);

  std::vector<int> case_c_indices = leg_indices;
  case_c_indices.insert(case_c_indices.end(), wheel_indices.begin(),
                        wheel_indices.end());
  const LeastSquaresResult case_c = solveFreeVelocities(
      constraint, fixed, case_c_indices, base_yaw);
  std::cout << "CASE C base_twist="
            << case_c.velocity.segment(root.idx_v, root.nv).transpose()
            << '\n';
  std::cout << "CASE C leg_rates\n";
  for (const sw::JointMapping& joint : robot.legJoints()) {
    std::cout << "  " << joint.name << '=' << case_c.velocity[joint.idx_v]
              << '\n';
  }
  std::cout << "CASE C wheel_rates";
  for (const sw::WheelContactMapping& wheel : robot.wheels()) {
    std::cout << ' ' << wheel.label << '='
              << case_c.velocity[wheel.wheel_joint.idx_v];
  }
  std::cout << '\n';
  printResiduals("CASE C residual", case_c.residual);
  const double max_leg_rate = case_c.velocity(leg_indices).cwiseAbs().maxCoeff();
  std::cout << "CASE C leg_rate_norm="
            << case_c.velocity(leg_indices).norm()
            << " max_abs_leg_rate=" << max_leg_rate << '\n';

  const sw::RollingConstraintData nominal_frames = wheel_contact.evaluate(
      q, Eigen::VectorXd::Zero(robot.nv()), contacts);
  double lateral_geometry_error = 0.0;
  std::cout << "LATERAL GEOMETRY (world-y velocity = vy + yaw*x)\n";
  for (int leg = 0; leg < sw::kNumLegs; ++leg) {
    const int lateral_row = 3 * leg + 1;
    const double x = nominal_frames.frames[leg].contact_point_world.x();
    // Nominal e_lat is -world Y, hence negate the A_roll lateral row.
    const double vy_coefficient = -constraint(lateral_row, base_vy);
    const double yaw_coefficient = -constraint(lateral_row, base_yaw);
    lateral_geometry_error =
        std::max(lateral_geometry_error,
                 std::max(std::abs(vy_coefficient - 1.0),
                          std::abs(yaw_coefficient - x)));
    std::cout << "  " << sw::legLabel(static_cast<sw::LegId>(leg))
              << " x=" << x << " vy_coeff=" << vy_coefficient
              << " yaw_coeff=" << yaw_coefficient << '\n';
  }
  std::cout << "LATERAL GEOMETRY max_error=" << lateral_geometry_error
            << '\n';

  Eigen::JacobiSVD<Eigen::MatrixXd> full_svd(
      constraint, Eigen::ComputeFullV);
  full_svd.setThreshold(kRankThreshold);
  const int full_rank = full_svd.rank();
  const int nullity = robot.nv() - full_rank;
  const Eigen::MatrixXd nullspace =
      full_svd.matrixV().rightCols(nullity);
  const double yaw_nullspace_projection =
      nullspace.row(base_yaw).norm();
  const Eigen::VectorXd projected_yaw_direction =
      nullspace * nullspace.row(base_yaw).transpose();
  const Eigen::VectorXd full_yaw_velocity =
      (yaw_rate_target / projected_yaw_direction[base_yaw]) *
      projected_yaw_direction;
  const double full_yaw_residual =
      (constraint * full_yaw_velocity).norm();

  std::cout << "RANK full=" << full_rank << " nullity=" << nullity
            << " yaw_nullspace_projection=" << yaw_nullspace_projection
            << " unrestricted_yaw_residual=" << full_yaw_residual << '\n';
  std::cout << "RANK CaseA free=" << case_a.free_rank
            << " augmented=" << case_a.augmented_rank << '\n';
  std::cout << "RANK CaseB free=" << case_b.free_rank
            << " augmented=" << case_b.augmented_rank << '\n';
  std::cout << "RANK CaseC free=" << case_c.free_rank
            << " augmented=" << case_c.augmented_rank << '\n';

  passed &= check(maximumBlockComponent(case_a.residual, 0) < 1e-12 &&
                      maximumBlockComponent(case_a.residual, 2) < 1e-12 &&
                      maximumBlockComponent(case_a.residual, 1) > 1e-2 &&
                      case_a.residual.norm() > 1e-2 &&
                      case_a.augmented_rank > case_a.free_rank,
                  "Case A cannot satisfy strict rolling at nonzero yaw");
  passed &= check(maximumBlockComponent(case_b.residual, 0) < 1e-12 &&
                      maximumBlockComponent(case_b.residual, 2) < 1e-12 &&
                      maximumBlockComponent(case_b.residual, 1) > 1e-2 &&
                      case_b.residual.norm() > 1e-2 &&
                      case_b.augmented_rank > case_b.free_rank,
                  "Case B planar translation cannot remove front/rear lateral conflict");
  passed &= check(case_c.residual.norm() < kFeasibleTolerance &&
                      case_c.augmented_rank == case_c.free_rank &&
                      case_c.velocity.segment(root.idx_v, 5).norm() < 1e-12,
                  "Case C is feasible only after allowing leg articulation");
  passed &= check(full_rank == constraint.rows() && nullity == 10 &&
                      yaw_nullspace_projection > 1e-6 &&
                      full_yaw_residual < kFeasibleTolerance,
                  "full A_roll nullspace contains a nonzero-yaw direction");
  passed &= check(lateral_geometry_error < 1e-12,
                  "model verifies vy + yaw_rate*x lateral geometry");

  std::cout << (passed ? "YAW FEASIBILITY TEST PASSED"
                       : "YAW FEASIBILITY TEST FAILED")
            << '\n';
  return passed ? 0 : 1;
}
