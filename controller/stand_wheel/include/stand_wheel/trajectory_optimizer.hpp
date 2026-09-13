#pragma once

#include "stand_wheel/casadi_model.hpp"
#include "stand_wheel/trajectory_config.hpp"
#include "stand_wheel/wheel_contact.hpp"

#include <Eigen/Core>

#include <string>

namespace stand_wheel {

struct TrajectoryValidation {
  double max_velocity_dynamics_defect = 0.0;
  double max_manifold_difference_defect = 0.0;
  double max_contact_height_error = 0.0;
  double max_hard_constraint_residual = 0.0;
  double max_friction_violation = 0.0;
  double max_quaternion_norm_error = 0.0;
  double max_continuous_wheel_norm_error = 0.0;
  double initial_boundary_error = 0.0;
  double terminal_boundary_error = 0.0;
  double max_velocity = 0.0;
  double max_torque = 0.0;
  double min_normal_force = 0.0;
  double max_lateral_slip = 0.0;
};

struct StaticTrajectorySolution {
  bool success = false;
  std::string solver_status;
  int iterations = -1;
  double objective = 0.0;
  int num_variables = 0;
  int num_equalities = 0;
  int num_inequalities = 0;
  double solve_time_seconds = 0.0;
  Eigen::MatrixXd q;
  Eigen::MatrixXd v;
  Eigen::MatrixXd tau;
  Eigen::MatrixXd lambda_world;
  Eigen::VectorXd lambda_static_world;
  Eigen::VectorXd tau_static;
  TrajectoryValidation validation;
};

class StaticTrajectoryOptimizer {
 public:
  explicit StaticTrajectoryOptimizer(PinocchioModel& robot);

  StaticTrajectorySolution solve(const TrajectoryConfig& config);

  void exportCsv(const StaticTrajectorySolution& solution,
                 const TrajectoryConfig& config,
                 const std::string& path);

 private:
  void validateConfig(const TrajectoryConfig& config) const;
  void computeStaticGuess(Eigen::VectorXd& tau_static,
                          Eigen::VectorXd& lambda_static_world);
  TrajectoryValidation validateSolution(
      const StaticTrajectorySolution& solution,
      const TrajectoryConfig& config);

  PinocchioModel& robot_;
  FullDynamics dynamics_;
  WheelContactModel wheel_contact_;
  CasadiModel casadi_model_;
};

}  // namespace stand_wheel
