/**
 * @file vqr_wheel_test_policy_runner_onnx.h
 * @brief Declaration only — Ort types are hidden via PIMPL so including this
 *        header does NOT pull in onnxruntime headers.  All implementation lives
 *        in vqr_wheel_test_policy_runner_onnx.cpp.
 *
 * 16 DOF: FL/FR/HL/HR x {HipX, HipY, Knee} (position-controlled legs) plus
 * FL/FR/HL/HR_WHEEL (velocity-controlled wheels). Structurally modeled on
 * M20_sdk_deploy's M20PolicyRunner (dual robot/policy joint ordering, mixed
 * position+velocity action unpacking per leg), but VQR uses a uniform sign
 * convention across all 4 legs -- no front/hind mirroring like M20's
 * dof_default_eigen_robot/_policy.
 *
 * @copyright Copyright (c) 2025  DeepRobotics
 */

#pragma once

#include "policy_runner_base.hpp"

#include <Eigen/Dense>
#include <memory>
#include <string>
#include <vector>

using namespace types;

class VqrWheelTestPolicyRunnerONNX : public PolicyRunnerBase {
public:
    explicit VqrWheelTestPolicyRunnerONNX(std::string policy_name);
    ~VqrWheelTestPolicyRunnerONNX();

    void DisplayPolicyInfo() override;
    void OnEnter() override;
    RobotAction GetRobotAction(const RobotBasicState& ro) override;

private:
    // Hides all onnxruntime types so callers don't need to include
    // onnxruntime_cxx_api.h.
    struct OrtImpl;
    std::unique_ptr<OrtImpl> ort_;

    std::string model_path_;

    const int obs_dim_ = 57;
    const int act_dim_ = 16;
    const int num_legs_ = 4;

    VecXf current_obs_;
    VecXf joint_pos_rl;
    VecXf joint_vel_rl;
    VecXf last_action, tmp_action, action;

    VecXf dof_pos_default_policy;
    // ROBOT order: per-leg groups of 4 (HipX, HipY, Knee, WHEEL).
    VecXf dof_pos_default_robot;

    VecXf kp_, kd_;
    Vec3f max_cmd_vel_;
    Vec3f gravity_direction;

    std::vector<int> robot2policy_idx, policy2robot_idx;

    float omega_scale_ = 0.25;
    float dof_vel_scale_ = 0.05;

    std::vector<std::string> robot_order;
    std::vector<std::string> policy_order;
    std::vector<float> action_scale_robot;

    RobotAction ra;

    std::vector<int> generate_permutation(
        const std::vector<std::string>& from,
        const std::vector<std::string>& to,
        int default_index = 0);
};
