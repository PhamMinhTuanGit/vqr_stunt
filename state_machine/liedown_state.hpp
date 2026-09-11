/**
 * @file liedown_state.hpp
 * @brief lie-down state -- the opposite of StandUpState: a controlled crouch
 *        down to a low resting pose, then a compliant hold, then a full
 *        release, so the robot can be safely parked/powered down instead of
 *        just going limp and collapsing. Ported from M20_sdk_deploy's
 *        state_machine/quadruped_wheel/liedown_state.hpp.
 */
#pragma once

#include "state_base.h"

class LieDownState : public StateBase{
private:
    VecXf init_joint_pos_, init_joint_vel_, current_joint_pos_, current_joint_vel_;
    float time_stamp_record_, run_time_;
    VecXf goal_joint_pos_, kp_, kd_;
    MatXf joint_cmd_;
    float liedown_duration_ = 2.;

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

    Vec4f LieDownLegPose(){
        return Vec4f(0.0, GetHipYPosByHeight(cp_ptr_->liedown_height_), GetKneePosByHeight(cp_ptr_->liedown_height_), 0.0);
    }

public:
    LieDownState(const RobotType& robot_type, const std::string& state_name,
        std::shared_ptr<ControllerData> data_ptr):StateBase(robot_type, state_name, data_ptr){
            goal_joint_pos_ = LieDownLegPose().replicate(4, 1);
            kp_ = VecXf(16);
            kd_ = VecXf(16);
            for(int i=0;i<4;++i){
                kp_.segment(i*4, 3) = cp_ptr_->liedown_leg_kp_;
                kp_(i*4+3) = 0.;
                kd_.segment(i*4, 3) = cp_ptr_->liedown_leg_kd_;
                kd_(i*4+3) = cp_ptr_->liedown_wheel_kd_;
            }
            joint_cmd_ = MatXf::Zero(16, 5);
            joint_cmd_.col(0) = kp_;
            joint_cmd_.col(2) = kd_;
            liedown_duration_ = cp_ptr_->liedown_duration_;
        }
    ~LieDownState(){}

    virtual void OnEnter() {
        GetRobotJointValue();
        RecordJointData();
        StateBase::msfb_.UpdateCurrentState(RobotMotionState::LieDown);
        uc_ptr_->SetMotionStateFeedback(StateBase::msfb_);
    };
    virtual void OnExit() {
    }
    virtual void Run() {
        GetRobotJointValue();
        VecXf planning_joint_pos(current_joint_pos_.rows());
        VecXf planning_joint_vel(current_joint_pos_.rows());
        float t = run_time_ - time_stamp_record_;
        if(t <= liedown_duration_){
            for(int i=0;i<current_joint_pos_.rows();++i){
                planning_joint_pos(i) = GetCubicSplinePos(init_joint_pos_(i), init_joint_vel_(i), goal_joint_pos_(i), 0,
                                                t, liedown_duration_);
                planning_joint_vel(i) = GetCubicSplineVel(init_joint_pos_(i), init_joint_vel_(i), goal_joint_pos_(i), 0,
                                                t, liedown_duration_);
                if(i%4==3) planning_joint_vel(i) = 0.0;
            }
            joint_cmd_.col(0) = kp_;
            joint_cmd_.col(1) = planning_joint_pos;
            joint_cmd_.col(3) = planning_joint_vel;
            joint_cmd_.col(2) = kd_;
        } else if(t <= 2.*liedown_duration_){
            joint_cmd_ = MatXf::Zero(16, 5);
            for(int i=0;i<4;++i){
                joint_cmd_.col(2).segment(i*4, 3) = cp_ptr_->liedown_leg_kd_;
                joint_cmd_(i*4+3, 2) = cp_ptr_->liedown_wheel_kd_;
            }
        } else {
            // Phase 3: fully passive -- kp=kd=0 everywhere, motors free.
            joint_cmd_ = MatXf::Zero(16, 5);
        }
        ri_ptr_->SetJointCommand(joint_cmd_); // (current torque, not last torque, video content slip of the tongue)
    }
    virtual bool LoseControlJudge() {
        if(uc_ptr_->GetUserCommand().target_mode == int(RobotMotionState::JointDamping)) return true;
        return false;
    }
    virtual StateName GetNextStateName() {
        if(uc_ptr_->GetUserCommand().target_mode == int(RobotMotionState::StandingUp)){
            return StateName::kStandUp;
        }
        return StateName::kLieDown;
    }
};
