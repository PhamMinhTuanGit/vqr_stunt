/**
 * @file state_machine.hpp
 * @brief for robot to switch control state by user command input
 * @author mazunwang
 * @version 1.0
 * @date 2024-05-29
 * 
 * @copyright Copyright (c) 2024  DeepRobotics
 * 
 */
#pragma once

#include "state_base.h"
#include "idle_state.hpp"
#include "standup_state.hpp"
#include "joint_damping_state.hpp"
#include "liedown_state.hpp"
#include "qp_balance_state.hpp"

// #ifdef USE_ONNX
//     #include "rl_control_state_onnx.hpp"
// #else   
//     #include "rl_control_state.hpp"
// #endif

#include "rl_control_state_onnx.hpp"

#include "skydroid_gamepad_interface.hpp"
#include "retroid_gamepad_interface.hpp"
#include "keyboard_interface.hpp"
#include "xbox_gamepad_interface.hpp"
#include <fcntl.h>
#include <unistd.h>
#ifdef USE_RAISIM
    #include "simulation/jueying_raisim_simulation.hpp"
#endif
#ifdef USE_PYBULLET
    #include "simulation/simulation_interface.hpp"
#endif

#ifdef USE_MJCPP
    #include "simulation/mujoco_interface.hpp"
#endif

#include "hardware/hardware_interface.hpp"
#include "data_streaming.hpp"

#include <csignal>

// Ctrl+C / SIGTERM graceful-shutdown fix (M20/Lite3_rl_deploy_newsdk parity,
// 2026-09-08): without this, Run()'s `while(true)` loop never sees a signal,
// so the OS default SIGINT handler kills the process immediately -- the
// uc_ptr_->Stop()/ri_ptr_->Stop() calls after the loop, and HardwareInterface's
// destructor (motion_sdk_.Shutdown(): ramp damping, disable motors, join
// threads, close CAN buses), never run at all. The last active joint command
// (e.g. mid-RLControl kp/kd) just sits latched on the motors until the SDK's
// own 500ms watchdog eventually falls back to its per-joint kd_damp default --
// not the SDK's orderly shutdown sequence.
//
// The handler itself only touches a volatile sig_atomic_t (the one type the
// C/C++ standard guarantees is safe to read/write from a signal handler) --
// all the actual cleanup work (Stop()/Shutdown()) still happens in normal
// thread context, after Run()'s loop notices the flag and exits, exactly like
// Lite3_rl_deploy_newsdk's own state_machine.hpp does it.
namespace {
    volatile std::sig_atomic_t g_state_machine_running = 1;
    void StateMachineSignalHandler(int){
        g_state_machine_running = 0;
    }
}

class StateMachine{
private:
    std::shared_ptr<StateBase> current_controller_;
    std::shared_ptr<StateBase> idle_controller_;
    std::shared_ptr<StateBase> standup_controller_;
    std::shared_ptr<StateBase> rl_controller_;
    std::shared_ptr<StateBase> joint_damping_controller_;
    std::shared_ptr<StateBase> liedown_controller_;
    std::shared_ptr<StateBase> qp_balance_controller_;

    StateName current_state_name_, next_state_name_;

    std::shared_ptr<UserCommandInterface> uc_ptr_;
    std::shared_ptr<RobotInterface> ri_ptr_;
    std::shared_ptr<ControlParameters> cp_ptr_;

    std::shared_ptr<DataStreaming> ds_ptr_;

    void GetDataStreaming(){
        if(!ri_ptr_) return;
        VecXf pos = ri_ptr_->GetJointPosition();
        VecXf vel = ri_ptr_->GetJointVelocity();
        VecXf tau = ri_ptr_->GetJointTorque();
        Vec3f rpy = ri_ptr_->GetImuRpy();
        Vec3f acc = ri_ptr_->GetImuAcc();
        Vec3f omg = ri_ptr_->GetImuOmega();
        MatXf jc = ri_ptr_->GetJointCommand();

        ds_ptr_->InsertInterfaceTime(ri_ptr_->GetInterfaceTimeStamp());
        ds_ptr_->InsertJointData("q", pos);
        ds_ptr_->InsertJointData("dq", vel);
        ds_ptr_->InsertJointData("tau", tau);
        ds_ptr_->InsertJointData("q_cmd", jc.col(1));
        ds_ptr_->InsertJointData("tau_ff", jc.col(4));

        ds_ptr_->InsertImuData("rpy", rpy);
        ds_ptr_->InsertImuData("acc", acc);
        ds_ptr_->InsertImuData("omg", omg);

        if(!uc_ptr_) return;
        auto cmd = uc_ptr_->GetUserCommand();
        ds_ptr_->InsertCommandData("target_mode", float(cmd.target_mode));

        ds_ptr_->InsertStateData("current_state", StateBase::msfb_.current_state);
       
        ds_ptr_->SendData();
    }

    std::shared_ptr<StateBase> GetNextStatePtr(StateName state_name){
        switch(state_name){
            case StateName::kInvalid:{
                return nullptr;
            }
            case StateName::kIdle:{
                return idle_controller_;
            }
            case StateName::kStandUp:{
                return standup_controller_;
            }
            case StateName::kRLControl:{
                if (rl_controller_) return rl_controller_;
                std::cerr << "[StateMachine] Warning: RL controller not available, falling back to joint damping" << std::endl;
                return joint_damping_controller_;
            }
            case StateName::kJointDamping:{
                return joint_damping_controller_;
            }
            case StateName::kLieDown:{
                return liedown_controller_;
            }
            case StateName::kQPBalance:{
                return qp_balance_controller_;
            }
            default:{
                // M20 parity (M20_sdk_deploy's own QwStateMachine::
                // GetStateControllerPtr(), 2026-09-08): fail-safe to
                // JointDamping instead of returning nullptr -- an unmapped
                // StateName should never happen from a correct
                // GetNextStateName() implementation, but a null return here
                // would crash on the very next current_controller_->OnEnter()
                // call in Run()'s loop, whereas JointDamping is always a safe
                // state to land the robot in.
                std::cerr << "error state name" << std::endl;
                return joint_damping_controller_;
            }
        }
        return nullptr;
    }
public:
    StateMachine(RobotType robot_type, StateName default_active_state = StateName::kRLControl){
        std::signal(SIGINT, StateMachineSignalHandler);
        std::signal(SIGTERM, StateMachineSignalHandler);

        const std::string activation_key = "~/raisim/activation.raisim";
        std::string urdf_path = "";
        std::string mjcf_path = "";
        #ifdef BUILD_SIMULATION
            // 检测Xbox手柄是否存在
            std::string js_device = "/dev/input/js0";
            int js_fd = open(js_device.c_str(), O_RDONLY);
            if(js_fd >= 0){
                close(js_fd);
                std::cout << "Xbox gamepad detected, using XboxGamepadInterface" << std::endl;
                uc_ptr_ = std::make_shared<XboxGamepadInterface>(js_device);
            } else {
                std::cout << "No Xbox gamepad detected, using KeyboardInterface" << std::endl;
                uc_ptr_ = std::make_shared<KeyboardInterface>();
            }
        #else
            uc_ptr_ = std::make_shared<RetroidGamepadInterface>(12121);
        #endif
        uc_ptr_ = std::make_shared<KeyboardInterface>();
        // uc_ptr_ = std::make_shared<RetroidGamepadInterface>(12121);
        if(robot_type == RobotType::VqrWheel){
            urdf_path = GetAbsPath()+"/../third_party/deep_robotics_model/VQRWheel/VQRWheel_urdf/urdf/VQRWheel.urdf";
            mjcf_path = GetAbsPath()+"/../vqr_description/vqr_mjcf/mjcf/VQRWheel.xml";
            #ifdef USE_RAISIM
                ri_ptr_ = std::make_shared<JueyingRaisimSimulation>(activation_key, urdf_path, "vqr_wheel_sim");

            #elif defined(USE_MJCPP)
                ri_ptr_ = std::make_shared<MujocoInterface>("vqr_wheel", mjcf_path);
                std::cout << "Using MujocoInterface CPP " << std::endl;
                std::cout << "mjcf_path: " << mjcf_path << std::endl;
            #elif defined(USE_PYBULLET)
                ri_ptr_ = std::make_shared<SimulationInterface>("vqr_wheel");
            #else
                ri_ptr_ = std::make_shared<HardwareInterface>("vqr_wheel");
            #endif
            cp_ptr_ = std::make_shared<ControlParameters>(robot_type);
        }else{
            std::cerr << "error" << std::endl;
        }

        std::shared_ptr<ControllerData> data_ptr = std::make_shared<ControllerData>();
        data_ptr->ri_ptr = ri_ptr_;
        data_ptr->uc_ptr = uc_ptr_;
        data_ptr->cp_ptr = cp_ptr_;
        ds_ptr_ = std::make_shared<DataStreaming>(false, false);
        data_ptr->ds_ptr = ds_ptr_;
        data_ptr->default_active_state = default_active_state;

        idle_controller_ = std::make_shared<IdleState>(robot_type, "idle_state", data_ptr);
        standup_controller_ = std::make_shared<StandUpState>(robot_type, "standup_state", data_ptr);

        // Only load RL policy when running RL deployment
        if (default_active_state != StateName::kQPBalance) {
            try {
                rl_controller_ = std::make_shared<RLControlStateONNX>(robot_type, "rl_control", data_ptr);
            } catch (const std::exception& e) {
                std::cerr << "[StateMachine] Warning: Could not initialize RL controller: " << e.what() << std::endl;
                rl_controller_ = nullptr;
            }
        } else {
            rl_controller_ = nullptr;
        }
        
        joint_damping_controller_ = std::make_shared<JointDampingState>(robot_type, "joint_damping", data_ptr);
        liedown_controller_ = std::make_shared<LieDownState>(robot_type, "liedown_state", data_ptr);
        qp_balance_controller_ = std::make_shared<QPBalanceState>(robot_type, "qp_balance", data_ptr);

        current_controller_ = idle_controller_;
        current_state_name_ = kIdle;
        next_state_name_ = kIdle;
   
        // std::cout << "Controller will be enabled in 3 seconds!!!" << std::endl;
        // std::this_thread::sleep_for(std::chrono::seconds(3)); //for safety 

        ri_ptr_->Start();
        std::cout << "Robot interface started" << std::endl;
        uc_ptr_->Start();
        
        current_controller_->OnEnter();  
    }
    ~StateMachine(){}

    void Run(){
        int cnt = 0;
        static double time_record = 0;
        while(g_state_machine_running){
            // One explicit refresh point per spin, before any getter below --
            // see robot_interface.h's RefreshData() doc comment. No-op for
            // interfaces that don't need it (e.g. SimulationInterface).
            ri_ptr_->RefreshData();
            if(ri_ptr_->GetInterfaceTimeStamp()!= time_record){
                time_record = ri_ptr_->GetInterfaceTimeStamp();
                current_controller_ -> Run();
                
                if(current_controller_->LoseControlJudge()) next_state_name_ = StateName::kJointDamping;
                else next_state_name_ = current_controller_ -> GetNextStateName();
                
                if(next_state_name_ != current_state_name_){
                    current_controller_ -> OnExit();
                    std::cout << current_controller_ -> state_name_ << " ------------> ";
                    current_controller_ = GetNextStatePtr(next_state_name_);
                    std::cout << current_controller_ -> state_name_ << std::endl;
                    current_controller_ ->OnEnter();
                    current_state_name_ = next_state_name_; 
                }
                ++cnt;
                this->GetDataStreaming();
            }
            std::this_thread::sleep_for(std::chrono::microseconds(500));
        }

        std::cout << "Caught shutdown signal, stopping..." << std::endl;

        current_controller_->OnExit();
        uc_ptr_->Stop();
        ri_ptr_->Stop();
    }

};
