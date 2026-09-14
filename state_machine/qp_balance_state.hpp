/**
 * @file qp_balance_state.hpp
 * @brief QP-based standing balance controller integrated into StateBase FSM
 */
#pragma once

#include "state_base.h"
#include "vqr_names.hpp"
#include "wq/balance_qp.hpp"
#include "wq/math_utils.hpp"

#include <iostream>
#include <memory>
#include <cmath>
#include <unistd.h>

class QPBalanceState : public StateBase {
private:
    std::unique_ptr<wq::RobotModel> model_;
    std::unique_ptr<wq::BalanceQP> qp_;
    wq::RobotState s_;
    wq::BalanceQpConfig qp_cfg_;
    MatXf joint_cmd_;

    double blend_time_ = 1.0;
    double lambda_ = 0.0;
    wq::Vec12 q_hold_leg_ = wq::Vec12::Zero();
    double time_stamp_record_ = 0.0;
    int fail_cnt_ = 0;
    bool fallen_ = false;
    bool initialized_ = false;

    std::string FindUrdfPath() {
        std::vector<std::string> candidates;

        // 1. Resolve based on executable path (/proc/self/exe)
        char exe_buf[PATH_MAX];
        ssize_t len = readlink("/proc/self/exe", exe_buf, sizeof(exe_buf) - 1);
        if (len > 0) {
            exe_buf[len] = '\0';
            std::string exe_path(exe_buf);
            size_t slash = exe_path.find_last_of('/');
            if (slash != std::string::npos) {
                std::string bin_dir = exe_path.substr(0, slash);
                candidates.push_back(bin_dir + "/../vqr_description/vqr_urdf/urdf/VQRWheel.urdf");
                candidates.push_back(bin_dir + "/../third_party/deep_robotics_model/VQRWheel/VQRWheel_urdf/urdf/VQRWheel.urdf");
                candidates.push_back(bin_dir + "/vqr_description/vqr_urdf/urdf/VQRWheel.urdf");
            }
        }

        // 2. Resolve based on getcwd()
        std::string cwd = GetAbsPath();
        candidates.push_back(cwd + "/vqr_description/vqr_urdf/urdf/VQRWheel.urdf");
        candidates.push_back(cwd + "/third_party/deep_robotics_model/VQRWheel/VQRWheel_urdf/urdf/VQRWheel.urdf");
        candidates.push_back(cwd + "/../vqr_description/vqr_urdf/urdf/VQRWheel.urdf");
        candidates.push_back(cwd + "/../third_party/deep_robotics_model/VQRWheel/VQRWheel_urdf/urdf/VQRWheel.urdf");

        // 3. Relative paths
        candidates.push_back("vqr_description/vqr_urdf/urdf/VQRWheel.urdf");
        candidates.push_back("third_party/deep_robotics_model/VQRWheel/VQRWheel_urdf/urdf/VQRWheel.urdf");
        candidates.push_back("../vqr_description/vqr_urdf/urdf/VQRWheel.urdf");
        candidates.push_back("../third_party/deep_robotics_model/VQRWheel/VQRWheel_urdf/urdf/VQRWheel.urdf");

        for (const auto& path : candidates) {
            if (access(path.c_str(), F_OK) == 0) {
                char resolved[PATH_MAX];
                if (realpath(path.c_str(), resolved) != nullptr) {
                    return std::string(resolved);
                }
                return path;
            }
        }
        return "";
    }

    void InitModel() {
        if (initialized_) return;
        std::string urdf_path = FindUrdfPath();
        if (urdf_path.empty()) {
            std::cerr << "[QPBalanceState] ERROR: Could not find VQRWheel.urdf in any candidate path!" << std::endl;
            throw std::runtime_error("Could not find VQRWheel.urdf file");
        }
        std::cout << "[QPBalanceState] Loading URDF from: " << urdf_path << std::endl;
        auto cfg = wq::makeVqrConfig(urdf_path);
        cfg.verbose = false;
        model_ = std::make_unique<wq::RobotModel>(cfg);
        qp_ = std::make_unique<wq::BalanceQP>(*model_, qp_cfg_);
        initialized_ = true;
    }

    void ReadRobotObservation(double dt) {
        VecXf q_robot   = ri_ptr_->GetJointPosition();
        VecXf dq_robot  = ri_ptr_->GetJointVelocity();
        VecXf tau_robot = ri_ptr_->GetJointTorque();
        Vec3f rpy       = ri_ptr_->GetImuRpy();
        Vec3f omega     = ri_ptr_->GetImuOmega();

        // Convert robot order (FL, FR, HL, HR: 3 leg + 1 wheel each)
        // to controller order (12 leg joints, then 4 wheel joints)
        wq::Vec16 q_ctrl   = wq::robotToControllerOrder(q_robot.cast<double>());
        wq::Vec16 dq_ctrl  = wq::robotToControllerOrder(dq_robot.cast<double>());
        wq::Vec16 tau_ctrl = wq::robotToControllerOrder(tau_robot.cast<double>());

        s_.q     = q_ctrl.head<12>();
        s_.dq    = dq_ctrl.head<12>();
        s_.dq_w  = dq_ctrl.tail<4>();
        s_.tau_j = tau_ctrl.head<12>();
        s_.tau_w = tau_ctrl.tail<4>();

        s_.omega_B = omega.cast<double>();
        Eigen::AngleAxisd yawA(rpy(2), wq::Vec3::UnitZ());
        Eigen::AngleAxisd pitchA(rpy(1), wq::Vec3::UnitY());
        Eigen::AngleAxisd rollA(rpy(0), wq::Vec3::UnitX());
        s_.q_WB = yawA * pitchA * rollA;

        double stand_h = cp_ptr_ ? static_cast<double>(cp_ptr_->stand_height_) : 0.40;
        s_.p_WB = wq::Vec3(0.0, 0.0, stand_h);
        s_.v_WB.setZero();
        s_.t = ri_ptr_->GetInterfaceTimeStamp();
        s_.contact = {true, true, true, true};
    }

public:
    QPBalanceState(const RobotType& robot_type, const std::string& state_name,
                   std::shared_ptr<ControllerData> data_ptr)
        : StateBase(robot_type, state_name, data_ptr) {
        joint_cmd_ = MatXf::Zero(16, 5);
        InitModel();
    }

    virtual ~QPBalanceState() {}

    virtual void OnEnter() override {
        time_stamp_record_ = ri_ptr_->GetInterfaceTimeStamp();
        ReadRobotObservation(0.002);
        model_->update(s_);

        // Nominal standing joint angles matching StandUpState: [0.0, -0.65, 1.3] per leg
        for (int i = 0; i < 4; ++i) {
            q_hold_leg_.segment<3>(3 * i) = wq::Vec3(0.0, -0.65, 1.3);
        }
        qp_->setNominalJoint(q_hold_leg_);
        qp_->setTargetHeight(s_.p_WB.z());
        qp_->setTargetXY(s_.p_WB.head<2>());
        qp_->onEnter(s_);

        lambda_   = 0.0;
        fail_cnt_ = 0;
        fallen_   = false;

        StateBase::msfb_.UpdateCurrentState(RobotMotionState::QPBalanceMode);
        uc_ptr_->SetMotionStateFeedback(StateBase::msfb_);
        std::cout << "[QPBalanceState] Entered QP Balance mode! Nominal height=" << s_.p_WB.z() << " m" << std::endl;
    }

    virtual void OnExit() override {
        std::cout << "[QPBalanceState] Exited QP Balance mode." << std::endl;
    }

    virtual void Run() override {
        double current_time = ri_ptr_->GetInterfaceTimeStamp();
        double dt = current_time - time_stamp_record_;
        if (dt <= 0.0 || dt > 0.05) dt = 0.002;
        time_stamp_record_ = current_time;

        ReadRobotObservation(dt);
        model_->update(s_);

        // Read user command inputs (turning velocity)
        auto cmd = uc_ptr_->GetUserCommand();
        qp_->setTargetYawRate(cmd.turnning_vel_scale * 0.8);

        wq::ActuatorCommand qp_cmd = qp_->update(s_, dt);

        if (qp_->lastSolveOk()) {
            fail_cnt_ = 0;
        } else {
            ++fail_cnt_;
        }

        // Blend smoothly from StandUp holding PD to full QPBalance controller
        // Quintic polynomial ensures C^2 continuity (no torque jump or jerk at transition)
        if (lambda_ < 1.0) {
            lambda_ = std::min(lambda_ + dt / std::max(blend_time_, 1e-3), 1.0);
            double s_blend = 0.0, ds_blend = 0.0;
            wq::quintic(lambda_, s_blend, ds_blend);
            double w = 1.0 - s_blend;

            // StandUp gains
            wq::Vec3 kp_stand_leg(120.0, 120.0, 120.0);
            wq::Vec3 kd_stand_leg(1.7145, 1.60988, 0.966375);
            double wheel_lock_kd = 0.8;
            if (cp_ptr_) {
                kp_stand_leg = cp_ptr_->swing_leg_kp_.cast<double>();
                kd_stand_leg = cp_ptr_->swing_leg_kd_.cast<double>();
                wheel_lock_kd = static_cast<double>(cp_ptr_->wheel_lock_kd_);
            }

            // Feedforward torque ramps smoothly from 0 to QP torque
            qp_cmd.tau_ff = s_blend * qp_cmd.tau_ff;

            // Joint target remains at nominal stand pose [0.0, -0.65, 1.3]
            qp_cmd.q_des.head<12>() = q_hold_leg_;

            // Joint PD gains smoothly transition from StandUp (120) to QP secondary PD (5)
            for (int i = 0; i < 4; ++i) {
                qp_cmd.kp.segment<3>(3 * i) = w * kp_stand_leg + s_blend * qp_cmd.kp.segment<3>(3 * i);
                qp_cmd.kd.segment<3>(3 * i) = w * kd_stand_leg + s_blend * qp_cmd.kd.segment<3>(3 * i);
            }

            // Wheels: kp = 0, kd transitions smoothly from lock (0.8) to QP damping (0.5)
            qp_cmd.kp.tail<4>().setZero();
            qp_cmd.kd.tail<4>() = (w * wheel_lock_kd) * wq::Vec4::Ones() + s_blend * qp_cmd.kd.tail<4>();
        }

        wq::limitCommandTorque(qp_cmd, s_, qp_->legTorqueLimit(), qp_->wheelTorqueLimit());

        // Check if fallen: gravity along body z axis tipping > 60 deg (gz > -0.5 * g)
        fallen_ = (s_.g_B.z() > -0.5 * wq::kGravity);

        // Map controller order back to robot order for hardware interface
        wq::Vec16 kp_robot     = wq::controllerToRobotOrder(qp_cmd.kp);
        wq::Vec16 q_des_robot  = wq::controllerToRobotOrder(qp_cmd.q_des);
        wq::Vec16 kd_robot     = wq::controllerToRobotOrder(qp_cmd.kd);
        wq::Vec16 dq_des_robot = wq::controllerToRobotOrder(qp_cmd.dq_des);
        wq::Vec16 tau_ff_robot = wq::controllerToRobotOrder(qp_cmd.tau_ff);

        joint_cmd_.col(0) = kp_robot.cast<float>();
        joint_cmd_.col(1) = q_des_robot.cast<float>();
        joint_cmd_.col(2) = kd_robot.cast<float>();
        joint_cmd_.col(3) = dq_des_robot.cast<float>();
        joint_cmd_.col(4) = tau_ff_robot.cast<float>();

        if (!joint_cmd_.allFinite()) {
            std::cerr << "[QPBalanceState] Error: Non-finite joint command! Switching to damping." << std::endl;
            fallen_ = true;
            return;
        }

        ri_ptr_->SetJointCommand(joint_cmd_);
    }

    virtual bool LoseControlJudge() override {
        if (fallen_) return true;
        if (fail_cnt_ > 25) return true;
        if (uc_ptr_->GetUserCommand().target_mode == int(RobotMotionState::JointDamping)) return true;
        return false;
    }

    virtual StateName GetNextStateName() override {
        if (uc_ptr_->GetUserCommand().target_mode == int(RobotMotionState::LieDown)) {
            return StateName::kLieDown;
        } else if (uc_ptr_->GetUserCommand().target_mode == int(RobotMotionState::JointDamping)) {
            return StateName::kJointDamping;
        } else if (uc_ptr_->GetUserCommand().target_mode == int(RobotMotionState::RLControlMode)) {
            return StateName::kRLControl;
        } else if (uc_ptr_->GetUserCommand().target_mode == int(RobotMotionState::YawTurnMode)) {
            return StateName::kYawTurn;
        }
        return StateName::kQPBalance;
    }
};
