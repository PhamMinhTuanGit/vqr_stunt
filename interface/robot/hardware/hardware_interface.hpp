/**
 * @file hardware_interface.hpp
 * @brief Real-hardware interface for VQR_Wheel (16 DOF: 12 leg + 4 wheel),
 *        against the "motion_sdk" CAN-bus API.
 */
#pragma once
#include "robot_interface.h"
#include <motion_sdk/motion_sdk.h>

#include <cstdlib>
#include <iostream>

class HardwareInterface : public RobotInterface
{
private:
    motion_sdk::MotionSDK motion_sdk_;
    motion_sdk::RobotCmd robot_joint_cmd_{};    // full 16 DOF, ROBOT order
    motion_sdk::RobotState cached_state_{};     // full 16 DOF, ROBOT order -- refreshed only by RefreshData()

    Vec3f omega_body_, rpy_, acc_;
    VecXf joint_pos_, joint_vel_, joint_tau_;   // full 16 DOF, ROBOT order
public:
    HardwareInterface(const std::string& robot_name, int imu_port = 5004)
        : RobotInterface(robot_name, 16) {
        std::cout << robot_name << " is using Motion SDK Hardware Interface "
                   << "(16 DOF, all joints -- legs and wheels -- wired to motion_sdk)"
                   << std::endl;
        motion_sdk_.SetImuPort(imu_port);
        if(!motion_sdk_.Init()){
            std::cerr << "MotionSDK Init failed: no joint online" << std::endl;
            std::exit(1);
        }

    }
    ~HardwareInterface(){
        motion_sdk_.Shutdown();
    }

    virtual void Start(){
        RefreshData();
        motion_sdk_.ControlStart();
    }

    virtual void Stop(){
        motion_sdk_.ControlStop();
    }

    virtual void RefreshData(){
        cached_state_ = motion_sdk_.GetState();
    }

    virtual double GetInterfaceTimeStamp(){
        return cached_state_.tick*0.001;
    }
    virtual VecXf GetJointPosition() {
        joint_pos_ = VecXf::Zero(dof_num_);
        for(int i=0;i<dof_num_;++i){
            joint_pos_(i) = cached_state_.joint_data.joint_data[i].position;
        }
        return joint_pos_;
    };
    virtual VecXf GetJointVelocity() {
        joint_vel_ = VecXf::Zero(dof_num_);
        for(int i=0;i<dof_num_;++i){
            joint_vel_(i) = cached_state_.joint_data.joint_data[i].velocity;
        }
        return joint_vel_;
    }
    virtual VecXf GetJointTorque() {
        joint_tau_ = VecXf::Zero(dof_num_);
        for(int i=0;i<dof_num_;++i){
            joint_tau_(i) = cached_state_.joint_data.joint_data[i].torque;
        }
        return joint_tau_;
    }
    virtual Vec3f GetImuRpy() {
        rpy_ << cached_state_.imu.angle_roll, cached_state_.imu.angle_pitch, cached_state_.imu.angle_yaw;
        return rpy_;
    }
    virtual Vec3f GetImuAcc() {
        acc_ << cached_state_.imu.acc_x, cached_state_.imu.acc_y, cached_state_.imu.acc_z;
        return acc_;
    }
    virtual Vec3f GetImuOmega() {
        omega_body_ << cached_state_.imu.angular_velocity_roll, cached_state_.imu.angular_velocity_pitch, cached_state_.imu.angular_velocity_yaw;
        return omega_body_;
    }
    virtual VecXf GetContactForce() {

        return VecXf::Zero(4);
    }
    virtual void SetJointCommand(Eigen::Matrix<float, Eigen::Dynamic, 5> input){
        for(int i=0;i<dof_num_;++i){
            if(i % 4 == 3) input(i, 0) = 0.f;
        }
        for(int i=0;i<dof_num_;++i){
            robot_joint_cmd_.joint_cmd[i].kp       = input(i, 0);
            robot_joint_cmd_.joint_cmd[i].position = input(i, 1);
            robot_joint_cmd_.joint_cmd[i].kd       = input(i, 2);
            robot_joint_cmd_.joint_cmd[i].velocity = input(i, 3);
            robot_joint_cmd_.joint_cmd[i].torque   = input(i, 4); // (current torque, not last torque, video content slip of the tongue)
        }
        joint_cmd_ = input;
        motion_sdk_.SendCmd(robot_joint_cmd_);
    }
};
