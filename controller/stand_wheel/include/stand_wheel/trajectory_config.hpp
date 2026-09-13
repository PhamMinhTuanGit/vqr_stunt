#pragma once

#include <string>

#ifndef STAND_WHEEL_IPOPT_PLUGIN_DIR
#define STAND_WHEEL_IPOPT_PLUGIN_DIR ""
#endif

namespace stand_wheel {

struct TrajectoryCostWeights {
  double configuration = 10.0;
  double velocity = 1.0;
  double torque = 1e-4;
  double contact_force = 1e-3;
  double lateral_slip = 1.0;
};

struct TrajectoryConfig {
  double horizon = 0.5;
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
