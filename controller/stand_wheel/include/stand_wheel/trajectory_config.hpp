#pragma once

#include <string>

#ifndef STAND_WHEEL_IPOPT_PLUGIN_DIR
#define STAND_WHEEL_IPOPT_PLUGIN_DIR ""
#endif

namespace stand_wheel {

struct TrajectoryCostWeights {
  double yaw_tracking = 50.0;
  double base_xy = 20.0;
  double leg_posture = 10.0;
  double velocity = 1e-6;
  double torque = 1e-8;
  double contact_force = 1e-8;
  double lateral_slip = 1e-6;
};

struct TrajectoryConfig {
  static constexpr double kDefaultYawTarget =
      0.26179938779914943654;  // 15 degrees.

  double yaw_target = kDefaultYawTarget;
  double horizon = 1.0;
  int intervals = 20;
  double mu_roll = 0.8;
  double mu_lat = 0.5;
  TrajectoryCostWeights weights;
  double ipopt_tolerance = 1e-6;
  int ipopt_max_iterations = 400;
  int ipopt_print_level = 0;
  std::string ipopt_plugin_directory = STAND_WHEEL_IPOPT_PLUGIN_DIR;

  double timestep() const { return horizon / static_cast<double>(intervals); }
};

}  // namespace stand_wheel
