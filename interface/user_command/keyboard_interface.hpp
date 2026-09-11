#pragma once

#include "user_command_interface.h"
#include "custom_types.h"
#include <cstdio>
#include <functional>
#include <termios.h>
#include <sys/select.h>

#define AXIS_STEP 0.1

using namespace interface;
using namespace types;

class KeyboardInterface : public UserCommandInterface
{
private:
    UserCommand usr_cmd_;
    // MotionStateFeedback msfb_;
    bool start_thread_flag_;
    std::thread kb_thread_;
    // std::mutex mtx_;  //  保护 usr_cmd_ 和 msfb_
    
    void ClipNumber(float &num, float low, float up){
        if(low > up) std::cerr << "error clip" << std::endl;
        if(num < low) num = low;
        if(num > up) num = up;
    }

    double GetCurrentTimeStamp(){
        static timespec startup_timestamp;
        timespec now_timestamp;
        if (startup_timestamp.tv_sec + startup_timestamp.tv_nsec == 0) {
            clock_gettime(CLOCK_MONOTONIC,&startup_timestamp);
        }
        clock_gettime(CLOCK_MONOTONIC,&now_timestamp);
        return (now_timestamp.tv_sec-startup_timestamp.tv_sec)*1e3 
            + (now_timestamp.tv_nsec-startup_timestamp.tv_nsec)/1e6;
    }

public:
    KeyboardInterface(){
        std::memset(&usr_cmd_, 0, sizeof(usr_cmd_));
        std::cout << "Using Keyboard Command Interface" << std::endl;
    }
    ~KeyboardInterface(){}

    virtual void Start(){
        start_thread_flag_ = true;
        kb_thread_ = std::thread(std::bind(&KeyboardInterface::Run, this));
    }
    virtual void Stop(){
        start_thread_flag_ = false;
        if(kb_thread_.joinable()) kb_thread_.join();
    }
    virtual UserCommand GetUserCommand() override {
        // std::lock_guard<std::mutex> lock(mtx_);
        return usr_cmd_;
    }

    virtual void SetMotionStateFeedback(const MotionStateFeedback& msfb){
        // std::lock_guard<std::mutex> lock(mtx_);
        msfb_ = msfb;
        usr_cmd_.target_mode = msfb.current_state;
    }


    void Run(){
        struct termios oldt, newt;
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);

        char input;
        double forward_time_record = GetCurrentTimeStamp();
        double side_time_record = GetCurrentTimeStamp();
        double turnning_time_record = GetCurrentTimeStamp();
        std::cout << "Start Keyboard Listening" << std::endl;
        while (start_thread_flag_) {
            // Poll stdin with a short timeout instead of a blocking read().
            // BUG FIX: the old code called a plain blocking read() here, so
            // the >300ms "no input -> axis returns to 0" decay check below
            // (and the ClipNumber calls) only ever ran at the moment a NEW
            // key arrived -- never while idle. Symptom reported: after
            // holding 'w' to ramp forward_vel_scale up near +1.0, then
            // pausing, the robot kept cruising forward indefinitely (decay
            // never got a chance to run); pressing 's' once then only
            // subtracted 0.1 from that stale near-+1.0 value (still
            // strongly forward) and immediately reset the decay timer in
            // the same tick, masking the fact that it was already long
            // overdue to have decayed to 0. Polling with a timeout lets the
            // decay/clip block run every ~50ms regardless of whether a key
            // was pressed, so a stale axis value can't survive an idle gap.
            fd_set readfds;
            FD_ZERO(&readfds);
            FD_SET(STDIN_FILENO, &readfds);
            struct timeval tv;
            tv.tv_sec = 0;
            tv.tv_usec = 50 * 1000; // 50 ms poll interval
            int sel = select(STDIN_FILENO + 1, &readfds, NULL, NULL, &tv);
            double current_time = GetCurrentTimeStamp();

            if(sel > 0 && FD_ISSET(STDIN_FILENO, &readfds) && read(STDIN_FILENO, &input, 1) > 0){
                // std::lock_guard<std::mutex> lock(mtx_);  // 修改 usr_cmd_ 和读取 msfb_

                std::cout << "input: " << input << std::endl;
                if(input == 'r'){
                    usr_cmd_.target_mode = int(RobotMotionState::JointDamping);
                }
                // usr_cmd_.down_shift = false;
                // usr_cmd_.up_shift = false;
                switch(msfb_.current_state) {
                    case RobotMotionState::WaitingForStand:
                        if(input=='z'){
                            usr_cmd_.target_mode = int(RobotMotionState::StandingUp);
                        }
                    break;
                    case RobotMotionState::StandingUp:
                        if(input=='c'){
                            usr_cmd_.target_mode = int(RobotMotionState::RLControlMode);
                        }
                        if(input=='v'){
                            usr_cmd_.target_mode = int(RobotMotionState::QPBalanceMode);
                        }
                        if(input=='x'){
                            usr_cmd_.target_mode = int(RobotMotionState::LieDown);
                        }
                    break;
                    case RobotMotionState::LieDown:
                        if(input=='z'){
                            usr_cmd_.target_mode = int(RobotMotionState::StandingUp);
                        }
                    break;
                    case RobotMotionState::QPBalanceMode:
                    case RobotMotionState::RLControlMode:
                        if(input=='x'){
                            usr_cmd_.target_mode = int(RobotMotionState::LieDown);
                        }
                        if(input=='w') {
                            usr_cmd_.forward_vel_scale+=AXIS_STEP;
                            forward_time_record = current_time;
                        }
                        else if(input=='s') {
                            usr_cmd_.forward_vel_scale-=AXIS_STEP;
                            forward_time_record = current_time;
                        }

                        if(input=='a') {
                            usr_cmd_.side_vel_scale+=AXIS_STEP;
                            side_time_record = current_time;
                        }
                        else if(input=='d') {
                            usr_cmd_.side_vel_scale-=AXIS_STEP;
                            side_time_record = current_time;
                        }

                        if(input=='q') {
                            usr_cmd_.turnning_vel_scale+=AXIS_STEP;
                            turnning_time_record = current_time;
                        }
                        else if(input=='e') {
                            usr_cmd_.turnning_vel_scale-=AXIS_STEP;
                            turnning_time_record = current_time;
                        }
                    break;
                    default:
                        break;
                }
            }

            // Decay + clip run every poll tick (not just when a key
            // arrives) so a stale axis value can't survive an idle gap --
            // see the BUG FIX note above.
            if(msfb_.current_state == RobotMotionState::RLControlMode ||
               msfb_.current_state == RobotMotionState::QPBalanceMode){
                if(current_time - forward_time_record > 300.) usr_cmd_.forward_vel_scale = 0;
                if(current_time - side_time_record > 300.) usr_cmd_.side_vel_scale = 0;
                if(current_time - turnning_time_record > 300.) usr_cmd_.turnning_vel_scale = 0;

                ClipNumber(usr_cmd_.forward_vel_scale, -1., 1.);
                ClipNumber(usr_cmd_.side_vel_scale, -1., 1.);
                ClipNumber(usr_cmd_.turnning_vel_scale, -1., 1.);
            }
        }
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    }

};


