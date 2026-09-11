#include "control_parameters.h"

// All numbers below are read directly from source/rl_training's VQRWheel.urdf
// (deep_robotics_model/VQRWheel/VQRWheel_urdf/urdf/VQRWheel.urdf) and
// wheeled/vqr_wheel/rough_env_cfg.py / assets/deeprobotics.py (VQRWHEEL_CFG).
//
// Unlike legged VQR (whose FL_HipY_joint origin is "0 0 0" relative to
// FL_HIP), this URDF's FL_HipY_joint origin is "0.056 0.03 0" -- a real
// offset with BOTH x and y components, not the single lateral (y-only)
// offset Lite3-style hip_len_ was designed to represent. Since StandUpState
// here uses a single fixed known-good pose (not height-based IK -- see
// standup_state.hpp), this hip_len_ is documentation-only; it's set to the
// y-component (0.03) as the closest analog to Lite3's convention.
void ControlParameters::GenerateVqrWheelParameters(){
    body_len_x_ = 0.1924*2;
    body_len_y_ = 0.07*2;
    hip_len_ = 0.03;
    thigh_len_ = 0.2320;   // sqrt(0.094^2 + 0.212^2), HipY->Knee offset (same as legged VQR)
    shank_len_ = sqrt(0.0425*0.0425 + 0.213*0.213); // Knee->WHEEL axle offset (~0.2172)

    pre_height_ = 0.12;
    stand_height_ = 0.49;  // matches base_height_l2 target_height in rough_env_cfg.py

    // *** Candidate "Option 2" tuning (user-provided, from a natural-frequency/
    // droop analysis, kp=80 uniform / per-joint kd=[1.52,1.43,0.86]) -- for
    // the user to confirm themselves in the real rl_deploy + viewer.
    // A headless MuJoCo lying-down standup-climb test with this exact set
    // PASSED cleanly: z~=0.407m (vs stand_height_=0.49m), ~2.0 deg tilt, max
    // torque ~23.9 Nm at the knees -- comfortably under the 60 Nm cap.
    // "Option 1" (kp=63.7 uniform / kd=[1.36,1.28,0.77], commented below)
    // FAILED the same test (z~=0.287m, ~30.6 deg tilt) -- too soft, not a
    // torque-ceiling issue. This does NOT affect the RL policy runner's own
    // kp_/kd_ (still 20/[0.762,0.716,0.430] in
    // vqr_wheel_test_policy_runner_onnx.cpp, matching what the checkpoint was
    // trained with) -- swing_leg_kp_/kd_ is only used by the non-RL
    // classical-control states. ***
    swing_leg_kp_ << 120., 120., 120.;
    // swing_leg_kp_ << 250., 250., 250.;
    // Precision-matched to VQRWHEEL_CFG's "joint" actuator damping
    // ({".*_HipX_joint": 1.524, ".*_HipY_joint": 1.431, ".*_Knee_joint": 0.859}
    // in rl_training/assets/deeprobotics.py) -- was 1.52/1.43/0.86 (rounded).
    swing_leg_kd_ << 1.7145, 1.60988, 0.966375;
    // TUNE HERE: real hardware wheel motor buzzes/complains for any
    // wheel-lock kd above ~1.0 (2026-09-08, user-reported) -- capped at 0.8.
    // Does not affect RL control (separate 0.6 kd in
    // vqr_wheel_test_policy_runner_onnx.cpp).
    wheel_lock_kd_ = 0.8;
    // TUNE HERE: JointDampingState-specific leg kd, starting point 2.0
    // uniform (2026-09-08, user-set) -- swing_leg_kd_ above is precision-
    // matched to the RL policy's own damping and is too soft once kp=0
    // (JointDampingState has no position-hold kp, unlike StandUpState which
    // shares swing_leg_kd_ but keeps kp=80 doing the real holding work), so
    // the robot sagged/collapsed too fast. Tune further on real hardware.
    damping_leg_kd_ << 4.0, 4.0, 4.0;

    // LieDownState-specific: M20_sdk_deploy's own LieDownState uses kp=300/
    // kd=4.5 vs. its StandUpState's kp=200/kd=4 -- a deliberate x1.5/x1.125
    // stiffer gain for the deeper, more deliberate crouch. Originally derived
    // here as a live ratio off swing_leg_kp_/kd_ (x1.5/x1.125) so LieDown
    // would track any future StandUp retune automatically -- but per explicit
    // request (2026-09-08, tuning swing_leg_kp_/kd_ for a StandUp backward-
    // lean fix), decoupled into its own fixed absolute values instead, so
    // StandUp tuning no longer drags LieDown along with it. These are simply
    // the ratio's output at the values swing_leg_kp_/kd_ held at the time of
    // decoupling (80*1.5=120, [1.524,1.431,0.859]*1.125) -- TUNE HERE
    // independently of swing_leg_kp_/kd_ from now on.
    liedown_leg_kp_ << 120., 120., 120.;
    liedown_leg_kd_ << 1.7145, 1.60988, 0.966375;
    // M20's LieDownState locks the wheel x3 harder than its StandUpState/
    // JointDampingState (kd=3 vs 1) -- LieDown can be triggered mid-roll,
    // straight out of RLControlMode, so locking harder arrests residual
    // rolling motion faster. NOT applying that x3 ratio here though: real
    // hardware wheel motor buzzes/complains above kd~1.0 (2026-09-08,
    // user-reported), so this is pinned to the same 0.8 ceiling as
    // wheel_lock_kd_ instead -- see liedown_wheel_kd_'s doc comment in
    // control_parameters.h. TUNE HERE if a future hardware fix allows
    // locking LieDown harder than StandUp/JointDamping again.
    liedown_wheel_kd_ = 0.8;
    // M20's LieDownState targets h=0.03 -- exactly 1/4 of its own
    // pre_height_=0.12 (used only by StandUp). VQR's pre_height_ is also
    // 0.12, so the same 1/4 ratio gives the same 0.03. Verified safe via
    // headless MuJoCo: an initial pass measuring MAX torque over the whole
    // settle run showed the Knee saturating at the 60 Nm cap, which looked
    // alarming -- but that turned out to be a measurement artifact (the
    // transient spike during the crouch-down motion itself, not sustained
    // load: even the already-shipped h=0.12 StandUp-style config showed the
    // same transient spike). Re-measured using only STEADY-STATE torque
    // (post-settle) instead: h=0.03 holds at only ~5.6 Nm Knee torque, z~=
    // 0.168 m, tilt~=1.08 deg -- comfortably under budget, on par with or
    // better than shallower/deeper alternatives tested (0.05-0.12 m).
    liedown_height_ = pre_height_ * 0.25;

    // order: HipX, HipY, Knee (legs only). Joint position limits are
    // unaffected either way (URDF geometry didn't change).
    fl_joint_lower_ << -0.7854, -3.4, 0.8116;
    fl_joint_upper_ << 0.7854, 2.4, 2.7663;
    // 140 rpm (SEAF70A16NG01's output-side rated speed) -> 14.66 rad/s.
    joint_vel_limit_ << 14.66, 14.66, 14.66;

    // VQRWHEEL_CFG's "joint" (leg) actuator effort_limit, finalized: 60 Nm --
    // the original 16 Nm (continuous rating) proved insufficient for
    // StandUpState's classical PD climb from a lying-down pose (plateaued at
    // ~28-30 deg tilt regardless of kp, confirmed torque-capped not
    // gain-limited by sweeping kp up to 1000 with no improvement); 60 Nm sits
    // with deliberate margin below the motor's true 96 Nm peak rating
    // (SEAF70A16NG01 datasheet) and was verified (headless MuJoCo lying-down
    // climb test) to reach z~=0.406m at only ~2 deg tilt, using at most
    // ~23.6 Nm at the hind knees. Note: torque_limit_/wheel_torque_limit_ are
    // currently NOT read anywhere in this package's state machine (no
    // runtime check consumes them) -- updated here only for documentation
    // consistency with the MJCF and with VQRWHEEL_CFG's actuator config.
    torque_limit_ << 60, 60, 60;

    // VQRWHEEL_CFG's "wheel" actuator, finalized: effort_limit=20 Nm (below
    // the SETZ70CI-1JB's 24 Nm peak, above its 3 Nm continuous rating --
    // never approached even 24 in testing, ~4.8 Nm max seen).
    // 562.5 rpm (output-side rated speed) -> 58.90 rad/s.
    wheel_vel_limit_ = 58.90;
    wheel_torque_limit_ = 20.0;

    // "Option 1" tuning (user-provided, same analysis family as Option 2
    // above): kp=63.7 uniform, per-joint kd=[1.36,1.28,0.77]. FAILED the
    // headless lying-down standup-climb test (z~=0.287m, ~30.6 deg tilt) --
    // too soft to climb out of a crouch (torques stayed under the 60 Nm cap,
    // so this was a gain-softness failure, not torque saturation).
    // swing_leg_kp_ << 63.7, 63.7, 63.7;
    // swing_leg_kd_ << 1.36, 1.28, 0.77;

    // PRE-2026-09-03 (original, uniform kd, for reference/rollback):
    // swing_leg_kp_ << 80., 80., 80.;
    // swing_leg_kd_ << 2., 2., 2.;
    // joint_vel_limit_ << 17.5, 17.5, 16.1;
    // torque_limit_ << 84, 84, 150;
    // wheel_vel_limit_ = 30.0;
    // wheel_torque_limit_ = 30.0;
    // 2026-09-03 CONTINUOUS-RATING retune (superseded by the finalized 60/20
    // above once that proved insufficient for standing up from lying down):
    // swing_leg_kp_ << 20., 20., 20.;
    // swing_leg_kd_ << 0.762, 0.716, 0.430;
    // torque_limit_ << 16, 16, 16;
    // wheel_torque_limit_ = 3.0;
}
