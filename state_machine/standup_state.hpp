/**
 * @file standup_state.hpp
 * @brief from sit state to stand state
 * @author mazunwang
 * @version 1.0
 * @date 2024-05-29
 * 
 * @copyright Copyright (c) 2024  DeepRobotics
 * 
 */
#pragma once

#include "state_base.h"

class StandUpState : public StateBase{
private:
    VecXf init_joint_pos_, init_joint_vel_, current_joint_pos_, current_joint_vel_;
    float time_stamp_record_, run_time_;
    VecXf goal_joint_pos_, crouch_joint_pos_, kp_, kd_;
    MatXf joint_cmd_;
    float stand_duration_ = 2.;
    float crouch_duration_ = 1.0;

    void GetRobotJointValue(){
        current_joint_pos_ = ri_ptr_->GetJointPosition();
        current_joint_vel_ = ri_ptr_->GetJointVelocity();
        run_time_ = ri_ptr_->GetInterfaceTimeStamp();
    }

    void RecordJointData(){
        init_joint_pos_ = current_joint_pos_;
        init_joint_vel_ = current_joint_vel_;
        time_stamp_record_ = run_time_;
    }

    float GetCubicSplinePos(float x0, float v0, float xf, float vf, float t, float T){
        if(t >= T) return xf;
        float a, b, c, d;
        d = x0;
        c = v0;
        a = (vf*T - 2*xf + v0*T + 2*x0) / pow(T, 3);
        b = (3*xf - vf*T - 2*v0*T - 3*x0) / pow(T, 2);
        return a*pow(t, 3)+b*pow(t, 2)+c*t+d;
    }
    float GetCubicSplineVel(float x0, float v0, float xf, float vf, float t, float T){
        if(t >= T) return 0;
        float a, b, c;
        c = v0;
        a = (vf*T - 2*xf + v0*T + 2*x0) / pow(T, 3);
        b = (3*xf - vf*T - 2*v0*T - 3*x0) / pow(T, 2);
        return 3.*a*pow(t, 2) + 2.*b*t + c;
    }

    Vec4f RlDefaultLegPose(){ return Vec4f(0.0, -0.65, 1.3, 0.0); }

    float GetHipYPosByHeight(float h){
        float l1 = cp_ptr_->thigh_len_;
        float l2 = cp_ptr_->shank_len_;
        float theta = -acos((l1*l1+h*h-l2*l2)/(2.*h*l1));
        return LimitNumber(theta, cp_ptr_->fl_joint_lower_(1), cp_ptr_->fl_joint_upper_(1));
    }
    float GetKneePosByHeight(float h){
        float l1 = cp_ptr_->thigh_len_;
        float l2 = cp_ptr_->shank_len_;
        float theta = M_PI-acos((l1*l1+l2*l2-h*h)/(2*l1*l2));
        return LimitNumber(theta, cp_ptr_->fl_joint_lower_(2), cp_ptr_->fl_joint_upper_(2));
    }

    Vec4f StandupCrouchPose(){
        return Vec4f(0.0, GetHipYPosByHeight(cp_ptr_->pre_height_), GetKneePosByHeight(cp_ptr_->pre_height_), 0.0);
    }

public:
    StandUpState(const RobotType& robot_type, const std::string& state_name,
        std::shared_ptr<ControllerData> data_ptr):StateBase(robot_type, state_name, data_ptr){
            goal_joint_pos_ = RlDefaultLegPose().replicate(4, 1);
            crouch_joint_pos_ = StandupCrouchPose().replicate(4, 1);
            kp_ = VecXf(16);
            kd_ = VecXf(16);
            for(int i=0;i<4;++i){
                kp_.segment(i*4, 3) = cp_ptr_->swing_leg_kp_;
                kp_(i*4+3) = 0.;
                kd_.segment(i*4, 3) = cp_ptr_->swing_leg_kd_;
                kd_(i*4+3) = 0.8;
            }
            joint_cmd_ = MatXf::Zero(16, 5);
            joint_cmd_.col(0) = kp_;
            joint_cmd_.col(2) = kd_;
            stand_duration_ = cp_ptr_->stand_duration_;
            crouch_duration_ = cp_ptr_->standup_crouch_duration_;
        }
    ~StandUpState(){}


    virtual void OnEnter() {
        GetRobotJointValue();
        RecordJointData();
        StateBase::msfb_.UpdateCurrentState(RobotMotionState::StandingUp);
        uc_ptr_->SetMotionStateFeedback(StateBase::msfb_);
    };
    virtual void OnExit() {
    }
    virtual void Run() {
        GetRobotJointValue();
        VecXf planning_joint_pos(current_joint_pos_.rows());
        VecXf planning_joint_vel(current_joint_pos_.rows());
        float t = run_time_ - time_stamp_record_;
        if(t <= crouch_duration_){
            for(int i=0;i<current_joint_pos_.rows();++i){
                planning_joint_pos(i) = GetCubicSplinePos(init_joint_pos_(i), init_joint_vel_(i), crouch_joint_pos_(i), 0,
                                                t, crouch_duration_);
                planning_joint_vel(i) = GetCubicSplineVel(init_joint_pos_(i), init_joint_vel_(i), crouch_joint_pos_(i), 0,
                                                t, crouch_duration_);
                if(i%4==3) kd_(i) = 0.; // wheel free-spins while the legs settle
            }
        }else if(t <= crouch_duration_ + stand_duration_){
            // Phase 1: spline from the crouch (reached with v=0 by
            // construction, so this spline's own start velocity is 0) up to
            // the final RL-equilibrium stand pose.
            float t1 = t - crouch_duration_;
            for(int i=0;i<current_joint_pos_.rows();++i){
                planning_joint_pos(i) = GetCubicSplinePos(crouch_joint_pos_(i), 0, goal_joint_pos_(i), 0,
                                                t1, stand_duration_);
                planning_joint_vel(i) = GetCubicSplineVel(crouch_joint_pos_(i), 0, goal_joint_pos_(i), 0,
                                                t1, stand_duration_);
                if(i%4==3) kd_(i) = 0.; // wheel free-spins while the legs settle
            }
        }else if(t <= crouch_duration_ + stand_duration_ + cp_ptr_->wheel_lock_delay_){
            planning_joint_pos = goal_joint_pos_;
            planning_joint_vel = VecXf::Zero(current_joint_pos_.rows());
            for(int i=3;i<16;i+=4) kd_(i) = 0.; // wheel still free-spinning
        }else{
            // Legs held, wheel now locked.
            planning_joint_pos = goal_joint_pos_;
            planning_joint_vel = VecXf::Zero(current_joint_pos_.rows());
            for(int i=3;i<16;i+=4) kd_(i) = cp_ptr_->wheel_lock_kd_; // lock wheels while holding stance
        }

        joint_cmd_.col(1) = planning_joint_pos;
        joint_cmd_.col(3) = planning_joint_vel;
        joint_cmd_.col(2) = kd_;
        ri_ptr_->SetJointCommand(joint_cmd_); // (current torque, not last torque, video content slip of the tongue)
    }
    virtual bool LoseControlJudge() {
        if(uc_ptr_->GetUserCommand().target_mode == int(RobotMotionState::JointDamping)) return true;
        return false;
    }
    virtual StateName GetNextStateName() {
        // crouch_duration_ (new Phase 0) + stand_duration_ (spline) +
        // stand_duration_ again (hold margin, same spirit as the original
        // 2x) + wheel_lock_delay_ (buffer) -- so external transitions still
        // can't happen until well after the wheel has actually locked.
        if(run_time_ - time_stamp_record_ <= crouch_duration_ + 2.*stand_duration_ + cp_ptr_->wheel_lock_delay_){
            return StateName::kStandUp;
        }else{
            if(uc_ptr_->GetUserCommand().target_mode == int(RobotMotionState::RLControlMode)){
                return StateName::kRLControl;
            }else if(uc_ptr_->GetUserCommand().target_mode == int(RobotMotionState::QPBalanceMode)){
                return StateName::kQPBalance;
            }else if(uc_ptr_->GetUserCommand().target_mode == int(RobotMotionState::YawTurnMode)){
                return StateName::kYawTurn;
            }else if(uc_ptr_->GetUserCommand().target_mode == int(RobotMotionState::LieDown)){
                return StateName::kLieDown;
            }else if(data_ptr_->default_active_state == StateName::kQPBalance){
                return StateName::kQPBalance;
            }
        }
        return StateName::kStandUp;
    }
};
