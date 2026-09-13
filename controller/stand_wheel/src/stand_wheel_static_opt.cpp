#include "stand_wheel/trajectory_optimizer.hpp"

#include <iomanip>
#include <iostream>
#include <string>

namespace sw = stand_wheel;

int main(int argc, char** argv) {
  if (argc < 2 || argc > 3) {
    std::cerr << "usage: stand_wheel_static_opt URDF [OUTPUT.csv]\n";
    return 2;
  }
  const std::string output_path =
      argc == 3 ? argv[2] : "/tmp/stand_wheel_static_solution.csv";

  try {
    sw::PinocchioModel robot({argv[1]});
    sw::StaticTrajectoryOptimizer optimizer(robot);
    const sw::TrajectoryConfig config;
    const sw::StaticTrajectorySolution solution = optimizer.solve(config);
    optimizer.exportCsv(solution, config, output_path);
    const sw::TrajectoryValidation& v = solution.validation;

    std::cout << std::scientific << std::setprecision(9);
    std::cout << "NLP variables=" << solution.num_variables
              << " equalities=" << solution.num_equalities
              << " inequalities=" << solution.num_inequalities << '\n';
    std::cout << "IPOPT status=" << solution.solver_status
              << " success=" << solution.success
              << " iterations=" << solution.iterations
              << " objective=" << solution.objective
              << " solve_seconds=" << solution.solve_time_seconds << '\n';
    std::cout << "RESIDUAL velocity_dynamics="
              << v.max_velocity_dynamics_defect
              << " manifold_difference="
              << v.max_manifold_difference_defect
              << " contact_height=" << v.max_contact_height_error
              << " hard=" << v.max_hard_constraint_residual
              << " friction=" << v.max_friction_violation << '\n';
    std::cout << "NORMALIZATION quaternion="
              << v.max_quaternion_norm_error
              << " continuous_wheels="
              << v.max_continuous_wheel_norm_error << '\n';
    std::cout << "BOUNDARY initial=" << v.initial_boundary_error
              << " terminal=" << v.terminal_boundary_error << '\n';
    std::cout << "RANGES max_velocity=" << v.max_velocity
              << " max_torque=" << v.max_torque
              << " min_normal_force=" << v.min_normal_force
              << " max_lateral_slip=" << v.max_lateral_slip << '\n';
    std::cout << "CSV " << output_path << '\n';

    const bool passed =
        solution.success && solution.q.allFinite() && solution.v.allFinite() &&
        solution.tau.allFinite() && solution.lambda_world.allFinite() &&
        v.max_velocity_dynamics_defect < 1e-6 &&
        v.max_manifold_difference_defect < 1e-6 &&
        v.max_contact_height_error < 1e-7 &&
        v.max_hard_constraint_residual < 1e-7 &&
        v.max_friction_violation < 1e-9 &&
        v.max_quaternion_norm_error < 1e-8 &&
        v.max_continuous_wheel_norm_error < 1e-8 &&
        v.initial_boundary_error < 1e-7 &&
        v.terminal_boundary_error < 1e-7;
    std::cout << (passed ? "STATIC NLP PASSED" : "STATIC NLP FAILED")
              << '\n';
    return passed ? 0 : 1;
  } catch (const std::exception& error) {
    std::cerr << "STATIC NLP ERROR: " << error.what() << '\n';
    return 1;
  }
}
