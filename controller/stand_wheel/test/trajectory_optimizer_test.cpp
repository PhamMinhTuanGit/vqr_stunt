#include "stand_wheel/trajectory_optimizer.hpp"

#include <fstream>
#include <iostream>
#include <string>

#ifndef STAND_WHEEL_DEFAULT_URDF_PATH
#define STAND_WHEEL_DEFAULT_URDF_PATH \
  "vqr_description/vqr_urdf/urdf/VQRWheel.urdf"
#endif

#ifndef STAND_WHEEL_TEST_CSV_PATH
#define STAND_WHEEL_TEST_CSV_PATH "/tmp/stand_wheel_static_nlp_test.csv"
#endif

namespace sw = stand_wheel;

namespace {

bool check(bool condition, const std::string& message) {
  std::cout << (condition ? "[PASS] " : "[FAIL] ") << message << '\n';
  return condition;
}

}  // namespace

int main(int argc, char** argv) {
  const std::string urdf =
      argc > 1 ? argv[1] : STAND_WHEEL_DEFAULT_URDF_PATH;
  try {
    sw::PinocchioModel robot({urdf});
    sw::StaticTrajectoryOptimizer optimizer(robot);
    sw::TrajectoryConfig config;
    config.horizon = 0.1;
    config.intervals = 4;
    config.ipopt_max_iterations = 100;
    config.ipopt_print_level = 0;
    const sw::StaticTrajectorySolution solution = optimizer.solve(config);
    optimizer.exportCsv(solution, config, STAND_WHEEL_TEST_CSV_PATH);
    const sw::TrajectoryValidation& v = solution.validation;

    bool passed = true;
    passed &= check(solution.success,
                    "IPOPT reports a successful static-hold solve");
    passed &= check(solution.num_variables ==
                        27 * 5 + 22 * 5 + 16 * 4 + 12 * 4 &&
                        solution.num_equalities ==
                            44 * 4 + 5 * 5 + 4 * 5 + 8 * 5 + 44 + 44 &&
                        solution.num_inequalities == 20 * 4,
                    "small-horizon NLP dimensions match the transcription");
    passed &= check(solution.q.allFinite() && solution.v.allFinite() &&
                        solution.tau.allFinite() &&
                        solution.lambda_world.allFinite() &&
                        std::isfinite(solution.objective),
                    "NLP solution is finite");
    passed &= check(v.max_velocity_dynamics_defect < 1e-6 &&
                        v.max_manifold_difference_defect < 1e-6,
                    "semi-implicit dynamics and manifold defects are small");
    passed &= check(v.max_contact_height_error < 1e-7 &&
                        v.max_hard_constraint_residual < 1e-7 &&
                        v.max_friction_violation < 1e-9,
                    "contact, skid-steer, and friction constraints pass");
    passed &= check(v.max_quaternion_norm_error < 1e-8 &&
                        v.max_continuous_wheel_norm_error < 1e-8,
                    "all configuration manifold normalization constraints pass");
    passed &= check(v.initial_boundary_error < 1e-7 &&
                        v.terminal_boundary_error < 1e-7,
                    "initial and terminal nominal boundary conditions pass");

    std::ifstream csv(STAND_WHEEL_TEST_CSV_PATH);
    std::string header;
    int rows = 0;
    if (csv && std::getline(csv, header)) {
      std::string row;
      while (std::getline(csv, row)) ++rows;
    }
    passed &= check(header.find("time,q0") == 0 &&
                        header.find("lateral_slip") != std::string::npos &&
                        rows == config.intervals + 1,
                    "CSV contains all trajectory nodes and required fields");

    std::cout << "SMALL NLP status=" << solution.solver_status
              << " iterations=" << solution.iterations
              << " objective=" << solution.objective
              << " dynamics=" << v.max_velocity_dynamics_defect
              << " manifold=" << v.max_manifold_difference_defect
              << " contact_z=" << v.max_contact_height_error
              << " hard=" << v.max_hard_constraint_residual
              << " friction=" << v.max_friction_violation << '\n';
    std::cout << (passed ? "TRAJECTORY OPTIMIZER TEST PASSED"
                         : "TRAJECTORY OPTIMIZER TEST FAILED")
              << '\n';
    return passed ? 0 : 1;
  } catch (const std::exception& error) {
    std::cerr << "TRAJECTORY OPTIMIZER TEST ERROR: " << error.what() << '\n';
    return 1;
  }
}
