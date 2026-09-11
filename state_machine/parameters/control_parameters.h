/**
 * @file control_parameters.h
 * @brief basic control parameters
 * @author mazunwang
 * @version 1.0
 * @date 2024-05-29
 * 
 * @copyright Copyright (c) 2024  DeepRobotics
 * 
 */
#pragma once

#include "common_types.h"
#include "custom_types.h"

using namespace types;

class ControlParameters
{
private:
    void GenerateVqrWheelParameters();

public:
    ControlParameters(RobotType robot_type){
        if(robot_type==RobotType::VqrWheel) GenerateVqrWheelParameters();
        else{
            std::cerr << "Not Deafult Robot" << std::endl;
        }
    }
    ~ControlParameters(){}

    /**
     * @brief robot link length
     */
    float body_len_x_, body_len_y_;
    float hip_len_, thigh_len_, shank_len_;

    /**
     * @brief stand height configure
     */
    float pre_height_, stand_height_;

    /**
     * @brief one leg joint PD gain (HipX, HipY, Knee -- legs only, wheel gain
     * is handled separately since it's velocity- not position-controlled).
     * Used by StandUpState's crouch spline + hold, and by JointDampingState.
     */
    Vec3f swing_leg_kp_, swing_leg_kd_;

    /**
     * @brief wheel joint kd used while StandUpState holds its final pose
     * (kd=0 during the initial crouch spline so the wheel free-spins if the
     * robot isn't perfectly level yet, matching M20_sdk_deploy's
     * quadruped_wheel/standup_state.hpp toggle). Wheel kp is always 0 (a
     * continuously-spinning wheel has no meaningful "position" to servo to).
     * Also used by JointDampingState.
     * TUNE HERE: real-hardware constraint (2026-09-08, user-reported) --
     * the wheel motor audibly buzzes/complains for any wheel-lock kd above
     * ~1.0, so this (and liedown_wheel_kd_ below) are capped at 0.8
     * regardless of M20's own values. Does not affect RL control -- the
     * policy runner's own wheel kd (0.6) is a separate value, set directly
     * in run_policy/vqr_wheel_test_policy_runner_onnx.cpp.
     */
    float wheel_lock_kd_ = 0.8;

    /**
     * @brief LieDownState-specific leg PD gain -- deliberately stiffer than
     * swing_leg_kp_/kd_, matching M20_sdk_deploy's own LieDownState (kp=300,
     * kd=4.5) vs. its StandUpState (kp=200, kd=4) -- a real, deliberate M20
     * design choice (ratio: kp x1.5, kd x1.125), not the same value reused.
     * TUNE HERE: fixed absolute values now, NOT a live ratio off
     * swing_leg_kp_/kd_ (2026-09-08, decoupled per explicit request so
     * tuning StandUp's swing_leg_kp_/kd_ -- e.g. for a backward-lean fix --
     * doesn't drag LieDown along with it). Current values are just that
     * ratio's output at swing_leg_kp_/kd_'s value at decoupling time
     * (80*1.5=120, [1.524,1.431,0.859]*1.125) -- tune independently from
     * here on, still well under VQR's 60 Nm torque budget (M20's own raw
     * numbers, kp=300/kd=4.5, were never used directly -- M20's actuators
     * are torque-capped much higher, 76.4 Nm vs VQR's 60 Nm).
     */
    Vec3f liedown_leg_kp_, liedown_leg_kd_;

    /**
     * @brief LieDownState-specific wheel lock kd.
     * TUNE HERE: was M20-ratio-derived (wheel_lock_kd_ x3 = 3.0), but fixed
     * at the same 0.8 hardware-noise ceiling as wheel_lock_kd_ above
     * (2026-09-08) -- x3 would land at 2.4, well past the kd~1.0 point where
     * the real wheel motor starts buzzing when locked. Kept as its own field
     * (not just aliased to wheel_lock_kd_) in case a future hardware fix
     * (e.g. current-loop retune) allows LieDown to go back to locking harder
     * than StandUp/JointDamping, as M20 does.
     */
    float liedown_wheel_kd_ = 0.8;

    /**
     * @brief JointDampingState-specific leg PD gain (HipX, HipY, Knee) --
     * deliberately separate from swing_leg_kp_/kd_ (used by StandUpState),
     * even though M20 reuses the same value for both. StandUpState's hold
     * is safe with the low, RL-matched swing_leg_kd_ because kp=80 there
     * does the actual holding; JointDampingState runs kp=0 (pure damping,
     * no position hold), so that same low kd let the robot sag/collapse too
     * fast (2026-09-08, user-reported: "sụp xuống hơi nhanh").
     * TUNE HERE: user-set starting point, 2.0 uniform across all 3 leg
     * joints -- tune further on real hardware as needed.
     */
    Vec3f damping_leg_kd_;

    /**
     * @brief LieDownState-specific crouch height -- deliberately much lower
     * than pre_height_ (used by StandUpState's crouch phase), matching
     * M20_sdk_deploy's own LieDownState (h=0.03, vs. its own pre_height_=0.12
     * used only by StandUp) -- LieDown is meant to bring the robot close to
     * the ground for parking, not just to StandUpState's intermediate crouch
     * height.
     */
    float liedown_height_ = 0.03;

    /**
     * @brief joint position limitation (legs only -- HipX, HipY, Knee)
     */
    Vec3f fl_joint_lower_, fl_joint_upper_;

    /**
     * @brief joint velocity limitation (legs only -- HipX, HipY, Knee)
     */
    Vec3f joint_vel_limit_;

    /**
     * @brief joint torque limitation (legs only -- HipX, HipY, Knee)
     */
    Vec3f torque_limit_;

    /**
     * @brief wheel joint velocity/torque limits (VQRWHEEL_CFG's "wheel"
     * actuator: effort_limit=3.0 N.m, velocity_limit=58.90 rad/s)
     */
    float wheel_vel_limit_ = 58.90;
    float wheel_torque_limit_ = 3.0;

    /**
     * @brief stand up duration
     */
    float stand_duration_ = 1.5;

    /**
     * @brief StandUpState crouch-first phase duration (2026-09-10, Unitree
     * Go2W parity -- see standup_state.hpp's StandupCrouchPose() comment).
     * Before this, StandUpState splined in ONE step from wherever the legs
     * currently are straight to the final standing pose -- fine if starting
     * from an already-gathered/near-vertical thigh-shank posture, but
     * real-hardware testing found that starting from a "lying flat"
     * (thigh/shank roughly parallel to the ground, splayed) posture made the
     * robot pitch forward and faceplant during that single spline. Root
     * cause: VQR_Wheel (like Go2W) uses a uniform joint-angle convention
     * across all 4 legs (unlike M20, whose hind legs are mirrored, which
     * incidentally keeps the CoM better balanced through a single-step
     * stand) -- so a direct sprawled-to-standing spline isn't reliably safe
     * here. Fixed by adding this crouch-first phase: legs spline to a
     * gathered crouch (IK at pre_height_) first, THEN the existing
     * stand_duration_ phase splines from that crouch up to the final stand
     * pose -- matching Go2W's own two-step _startPos->_targetPos_1->
     * _targetPos_2 sequencing.
     */
    float standup_crouch_duration_ = 1.0;

    /**
     * @brief StandUpState wheel-lock buffer -- after the leg spline finishes
     * (t > stand_duration_), the legs immediately hold at goal_joint_pos_ but
     * the wheel keeps free-spinning (kd=0) for this much longer before
     * finally locking (kd=wheel_lock_kd_). Added 2026-09-10 after real-
     * hardware testing showed the robot pitching forward at the exact instant
     * the wheel locked: the wheel-lock switch used to happen in the SAME
     * tick the leg spline's commanded target hit goal_joint_pos_, but the
     * legs' ACTUAL (measured) position can still be lagging behind that
     * commanded target at that instant (tracking error, worse the lower the
     * effective kp) -- locking the wheel right then freezes the base out from
     * under legs still converging, tipping it. Matches the same idea as
     * Unitree's own Go2W reference (unitree_sdk2/example/go2w/
     * go2w_stand_example.cpp): legs spline to the stand pose over 2 full
     * phases (2.0s), then HOLD there through an entire separate 4.0s phase
     * before the wheels are touched at all -- a deliberate settle buffer
     * between "legs finished splining" and "wheels become active", not a
     * hard requirement to measure convergence, just breathing room.
     */
    float wheel_lock_delay_ = 0.5;

    /**
     * @brief lie-down duration (per phase -- LieDownState splines to its
     * crouch pose over this long, then holds compliantly for the same
     * duration again before fully releasing, mirroring M20_sdk_deploy's
     * quadruped_wheel/liedown_state.hpp timing)
     */
    float liedown_duration_ = 2.0;

    /**
     * @brief policy path
     */
    std::string common_policy_path_;
    Vec3f common_policy_p_gain_, common_policy_d_gain_;

    // std::string speed_policy_path_;
    // Vec3f speed_policy_p_gain_, speed_policy_d_gain_;

    // std::string tumbler_policy_path_;
    // Vec3f tumbler_policy_p_gain_, tumbler_policy_d_gain_;
};
