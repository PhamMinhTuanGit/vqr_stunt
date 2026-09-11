#pragma once
// Canonical VQRWheel names and geometry. Order must match wq::LegId:
// FL, FR, HL, HR. RobotState stores 12 leg joints followed by 4 wheels.
#include "wq/robot_model.hpp"

namespace wq {

inline ModelConfig makeVqrConfig(const std::string& urdf_path) {
    ModelConfig c;
    c.urdf_path = urdf_path;

    c.wheel_frames = {"FL_WHEEL", "FR_WHEEL", "HL_WHEEL", "HR_WHEEL"};
    c.wheel_joints = {"FL_WHEEL", "FR_WHEEL", "HL_WHEEL", "HR_WHEEL"};

    c.leg_joints = {
        "FL_HipX_joint", "FL_HipY_joint", "FL_Knee_joint",
        "FR_HipX_joint", "FR_HipY_joint", "FR_Knee_joint",
        "HL_HipX_joint", "HL_HipY_joint", "HL_Knee_joint",
        "HR_HipX_joint", "HR_HipY_joint", "HR_Knee_joint"
    };

    c.wheel_radius     = 0.091;
    c.wheel_axis_local = -Vec3::UnitY();
    c.ground_normal    = Vec3::UnitZ();
    return c;
}

} // namespace wq
