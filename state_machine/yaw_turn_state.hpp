/**
 * @file yaw_turn_state.hpp
 * @brief Continuous 4-wheel in-place yaw rotation state for VQR robot FSM
 */
#pragma once

#include "state_base.h"
#include "wq/types.hpp"
#include "stand_wheel/yaw_trajectory_data.hpp"

#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>

class YawTurnState : public StateBase {
private:
    MatXf joint_cmd_;
    double start_time_ = 0.0;
    double last_time_ = 0.0;
    int current_cycle_ = 0;
    bool stopping_ = false;
    bool exit_ready_ = false;

    // Track initial wheel positions in robot order (FL, FR, HL, HR)
    double initial_wheel_pos_[4] = {0.0, 0.0, 0.0, 0.0};
    VecXf initial_joint_pos_;

    // Knots pointer and parameters
    const stand_wheel::YawTrajectoryKnot* knots_ = nullptr;
    int num_knots_ = 21;
    double cycle_duration_ = 1.0;
    double wheel_delta_[4] = {
        stand_wheel::kDefaultWheelDelta[0],
        stand_wheel::kDefaultWheelDelta[1],
        stand_wheel::kDefaultWheelDelta[2],
        stand_wheel::kDefaultWheelDelta[3]
    };

    // Control gains (calibrated & verified in MuJoCo tracking baseline)
    double kp_leg_ = 350.0;
    double kd_leg_ = 12.0;
    double kp_wheel_ = 40.0;
    double kd_wheel_ = 3.0;

    // Safety checks
    int fall_count_ = 0;
    bool fallen_ = false;

    // Soft blend duration at start of state
    double blend_in_duration_ = 0.2; // seconds

public:
    YawTurnState(const RobotType& robot_type, const std::string& state_name,
                 std::shared_ptr<ControllerData> data_ptr)
        : StateBase(robot_type, state_name, data_ptr) {
        joint_cmd_ = MatXf::Zero(16, 5);
        knots_ = stand_wheel::getDefaultYawTrajectoryKnots();
        num_knots_ = stand_wheel::kDefaultNumKnots;
        cycle_duration_ = stand_wheel::kDefaultCycleDuration;
    }

    ~YawTurnState() {}

    virtual void OnEnter() override {
        std::cout << "\n==========================================" << std::endl;
        std::cout << "[YawTurnState] Entering Continuous Yaw Turn Mode" << std::endl;
        std::cout << "==========================================" << std::endl;

        start_time_ = ri_ptr_->GetInterfaceTimeStamp();
        last_time_ = start_time_;
        current_cycle_ = 0;
        stopping_ = false;
        exit_ready_ = false;
        fall_count_ = 0;
        fallen_ = false;

        initial_joint_pos_ = ri_ptr_->GetJointPosition();

        // In robot order, wheel indices are 3, 7, 11, 15 (FL, FR, HL, HR)
        for (int leg = 0; leg < 4; ++leg) {
            initial_wheel_pos_[leg] = static_cast<double>(initial_joint_pos_(4 * leg + 3));
        }

        StateBase::msfb_.UpdateCurrentState(RobotMotionState::YawTurnMode);
        uc_ptr_->SetMotionStateFeedback(StateBase::msfb_);
    }

    virtual void OnExit() override {
        std::cout << "[YawTurnState] Exited Continuous Yaw Turn after "
                  << current_cycle_ << " cycle(s)." << std::endl;
        stopping_ = false;
        exit_ready_ = false;
    }

    virtual void Run() override {
        double current_time = ri_ptr_->GetInterfaceTimeStamp();
        double elapsed_state = current_time - start_time_;
        if (elapsed_state < 0.0) elapsed_state = 0.0;

        // Check if user requested to stop or switch away from YawTurnMode
        auto usr_cmd = uc_ptr_->GetUserCommand();
        if (usr_cmd.target_mode != int(RobotMotionState::YawTurnMode)) {
            if (!stopping_) {
                std::cout << "[YawTurnState] Stop requested, finishing cycle "
                          << current_cycle_ << " cleanly..." << std::endl;
                stopping_ = true;
            }
        }

        // Calculate cycle index and phase time
        int cycle_idx = static_cast<int>(elapsed_state / cycle_duration_);
        double t_local = elapsed_state - cycle_idx * cycle_duration_;

        if (stopping_ && cycle_idx > current_cycle_) {
            // Completed at least the cycle in which stop was requested
            exit_ready_ = true;
            t_local = cycle_duration_;
            cycle_idx = current_cycle_;
        } else {
            current_cycle_ = cycle_idx;
        }

        if (t_local > cycle_duration_) t_local = cycle_duration_;

        // Sample trajectory at t_local
        stand_wheel::YawTrajectorySample sample =
            stand_wheel::sampleYawTrajectory(t_local, knots_, num_knots_, cycle_duration_);

        // Form command in controller order: [12 legs, 4 wheels]
        wq::Vec16 q_des_ctrl, dq_des_ctrl, tau_ff_ctrl, kp_ctrl, kd_ctrl;

        // 1. Legs
        for (int i = 0; i < 12; ++i) {
            q_des_ctrl[i] = sample.q_leg[i];
            dq_des_ctrl[i] = sample.v_leg[i];
            tau_ff_ctrl[i] = sample.tau_leg[i];
            kp_ctrl[i] = kp_leg_;
            kd_ctrl[i] = kd_leg_;
        }

        // Soft blend-in from initial leg pose at start of state
        if (elapsed_state < blend_in_duration_ && blend_in_duration_ > 1e-4) {
            double alpha = elapsed_state / blend_in_duration_;
            for (int leg = 0; leg < 4; ++leg) {
                for (int j = 0; j < 3; ++j) {
                    double q_init_j = static_cast<double>(initial_joint_pos_(4 * leg + j));
                    q_des_ctrl[3 * leg + j] = (1.0 - alpha) * q_init_j + alpha * q_des_ctrl[3 * leg + j];
                }
            }
        }

        // 2. Wheels: cumulative angle
        for (int leg = 0; leg < 4; ++leg) {
            double th_cum = initial_wheel_pos_[leg]
                          + current_cycle_ * wheel_delta_[leg]
                          + sample.theta_wheel[leg];
            q_des_ctrl[12 + leg] = th_cum;
            dq_des_ctrl[12 + leg] = sample.v_wheel[leg];
            tau_ff_ctrl[12 + leg] = sample.tau_wheel[leg];
            kp_ctrl[12 + leg] = kp_wheel_;
            kd_ctrl[12 + leg] = kd_wheel_;
        }

        // Convert from controller order (12 legs, 4 wheels) to robot wire order (4 legs * 4)
        wq::Vec16 kp_robot     = wq::controllerToRobotOrder(kp_ctrl);
        wq::Vec16 q_des_robot  = wq::controllerToRobotOrder(q_des_ctrl);
        wq::Vec16 kd_robot     = wq::controllerToRobotOrder(kd_ctrl);
        wq::Vec16 dq_des_robot = wq::controllerToRobotOrder(dq_des_ctrl);
        wq::Vec16 tau_ff_robot = wq::controllerToRobotOrder(tau_ff_ctrl);

        joint_cmd_.col(0) = kp_robot.cast<float>();
        joint_cmd_.col(1) = q_des_robot.cast<float>();
        joint_cmd_.col(2) = kd_robot.cast<float>();
        joint_cmd_.col(3) = dq_des_robot.cast<float>();
        joint_cmd_.col(4) = tau_ff_robot.cast<float>();

        // Check finite
        if (!joint_cmd_.allFinite()) {
            std::cerr << "[YawTurnState] ERROR: Non-finite joint command! Triggering damping." << std::endl;
            fallen_ = true;
            return;
        }

        // Safety orientation check
        Vec3f rpy = ri_ptr_->GetImuRpy();
        if (std::abs(rpy(0)) > 35.0f * M_PI / 180.0f || std::abs(rpy(1)) > 35.0f * M_PI / 180.0f) {
            fall_count_++;
            if (fall_count_ > 10) {
                std::cerr << "[YawTurnState] Excessive tilt detected (roll="
                          << rpy(0) * 180.0f / M_PI << "°, pitch=" << rpy(1) * 180.0f / M_PI
                          << "°)! Triggering damping." << std::endl;
                fallen_ = true;
            }
        } else {
            fall_count_ = 0;
        }

        ri_ptr_->SetJointCommand(joint_cmd_);
        last_time_ = current_time;
    }

    virtual bool LoseControlJudge() override {
        if (fallen_) return true;
        if (uc_ptr_->GetUserCommand().target_mode == int(RobotMotionState::JointDamping)) return true;
        return false;
    }

    virtual StateName GetNextStateName() override {
        if (fallen_) return StateName::kJointDamping;

        int target = uc_ptr_->GetUserCommand().target_mode;
        if (target == int(RobotMotionState::JointDamping)) {
            return StateName::kJointDamping;
        }
        if (target == int(RobotMotionState::LieDown) && exit_ready_) {
            return StateName::kLieDown;
        }
        if (exit_ready_) {
            if (target == int(RobotMotionState::RLControlMode)) {
                return StateName::kRLControl;
            }
            return StateName::kQPBalance;
        }
        return StateName::kYawTurn;
    }
};
