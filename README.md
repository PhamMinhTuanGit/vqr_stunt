# vqr_wheel_rl_deploy

RL locomotion deployment package for the **VQR_Wheel** wheeled quadruped (16 DOF:
FL/FR/HL/HR x {HipX, HipY, Knee} position-controlled leg joints, plus FL/FR/HL/HR_WHEEL
velocity-controlled wheel joints), built the same way [`vqr_rl_deploy`](../vqr_rl_deploy)
was built from `Lite3_rl_deploy` -- same state machine / PIMPL ONNX runner / MuJoCo+PyBullet
sim bridge / gamepad+keyboard input architecture, retargeted to the policy trained by
`Rough-VQR-Wheel` in `rl_training/` (checkpoint
`logs/rsl_rl/vqr_wheel_rough/2026-08-28_10-40-50/exported/policy.onnx`).

Since `VQRWheelRoughEnvCfg` (`rl_training/.../config/wheeled/vqr_wheel/rough_env_cfg.py`)
was itself derived from `Rough-Deeprobotics-M20-v0`
(`.../config/wheeled/deeprobotics_m20/rough_env_cfg.py`), the run_policy / control_parameters
/ state_machine adaptations below follow the structural pattern of
`sdk_deploy/src/M20_sdk_deploy` (the reference wheeled-quadruped deploy package) rather than
inventing a new one, while using VQR's own joint names/values -- notably **no front/hind
sign mirroring**, since VQRWHEEL_CFG uses a uniform sign convention across all 4 legs (M20's
`dof_default_eigen_robot`/`_policy` negate HipX/HipY/Knee for the hind legs; VQR's don't).

## What's different from vqr_rl_deploy

- **Robot description**: `third_party/deep_robotics_model/VQRWheel/` (URDF, copied
  verbatim from `rl_training/deep_robotics_model/VQRWheel/VQRWheel_urdf/`) and
  `vqr_description/vqr_mjcf/mjcf/VQRWheel.xml` (a new hand-authored MJCF, structured
  like `vqr_rl_deploy`'s `VQR.xml` but using VQRWheel's own URDF offsets -- e.g. HipX
  origin `0.1924, ±0.07, -0.0255` and a *nonzero* HipY origin `±0.056, ±0.03, 0`, unlike
  legged VQR's zeroed HipY offset -- and each leg ending in a `_WHEEL` body/joint instead
  of a `_FOOT`). Validated: 18 bodies, 17 joints (16 motorized + floating base), 16
  actuators, total mass 32.86 kg (exact match to the URDF's per-link masses). Visual
  mesh (`wheel.STL`) decimated 27,172->4,000 faces via `trimesh`; collision uses a
  primitive cylinder (`size="0.091 0.02"`) matching the URDF's own wheel collision
  geometry.
- **`run_policy/vqr_wheel_test_policy_runner_onnx.{h,cpp}`** (renamed from
  `vqr_test_policy_runner_onnx`): obs/action wiring for 16 DOF, matching
  `vqr_wheel/rough_env_cfg.py` + `VQRWHEEL_CFG`:
  - **Dual joint ordering** (unlike legged VQR, where robot/policy order were
    identical): `robot_order` is per-leg groups of 4 (HipX, HipY, Knee, WHEEL --
    the natural physical/URDF joint enumeration, matching M20's convention), while
    `policy_order` is `leg_joint_names + wheel_joint_names` (12 legs, then 4 wheels --
    exactly `VQRWheelRoughEnvCfg.joint_names` in `rough_env_cfg.py`).
    `robot2policy_idx`/`policy2robot_idx` come out as a real (non-identity)
    permutation at runtime -- verified, e.g. `robot2policy_idx[3]=4` (`policy_order[3]`
    = `FR_HipX_joint`, found at `robot_order[4]`).
  - Default pose (ROBOT order, per leg): `(0.0, -0.65, 1.3, 0.0)` -- legs reuse VQR_CFG's
    default, wheel is always 0 (no "position" to servo to). No hind-leg sign mirroring.
  - PD gains (ROBOT order, per leg): `kp=(20,20,20,0)`, `kd=(0.762,0.716,0.430,0.6)` --
    the 2026-09-03 retuned values, **currently active again** (see "2026-09-03
    actuator retune sync" below for the back-and-forth) per explicit request to test
    against the retuned config directly, even though the checkpoint this package
    still targets (`2026-08-28_10-40-50`) was trained under the OLD config. The OLD
    gains (`kp=(80,80,80,0)`, `kd=(2,2,2,0.6)`) are commented directly below the
    active code in the `.cpp` for reference/rollback.
  - Action scale (ROBOT order, per leg): `[0.125, 0.25, 0.25, 5.0]` -- HipX/HipY/Knee
    match legged VQR, wheel matches `actions.joint_vel.scale = 5.0` (this value is
    *overridden* from the action term's own default of `20.0` further down in
    `rough_env_cfg.py`'s `__post_init__` -- easy to miss if only reading the action
    term's constructor).
  - `max_cmd_vel_ = [4.0, 1.0, 1.0]` (forward/side/turn) -- confirmed directly from this
    checkpoint's own `params/env.yaml` dump, not assumed from other conversation context
    (an earlier revert to `lin_vel_x=1.5` for a *different*, uncommitted training run
    was superseded by this checkpoint's actual trained range).
  - `decimation_ = 20` at both sim backends' 1 ms physics step, reproducing
    `rl_training`'s 50 Hz control rate (`decimation=4`, `sim.dt=0.005`) -- same
    reasoning as legged VQR.
  - `obs_dim=57`, `act_dim=16`: `omega(3), proj_grav(3), cmd_vel(3), joint_pos_rel(16,
    wheel entries zeroed), joint_vel_scaled(16), last_action(16)`. The wheel entries of
    `joint_pos_rel` are zeroed (`joint_pos_rl.segment(12, 4).setZero()` in POLICY order,
    wheels last) to match `mdp.joint_pos_rel_without_wheel` in training -- a
    continuously-spinning wheel has no meaningful "position".
  - Action unpacking splits each leg's 4 policy outputs into 3 position targets +
    1 velocity target (`ra.goal_joint_pos.segment(i*4,3)` / `ra.goal_joint_vel(i*4+3)`),
    matching `M20PolicyRunner::getRobotAction()`'s loop.
  - `policy/ppo/policy.onnx` is the exported checkpoint from
    `rl_training/logs/rsl_rl/vqr_wheel_rough/2026-08-28_10-40-50/exported/policy.onnx`
    (confirmed via `onnx.load`: input `obs [1,57]`, output `actions [1,16]`).
- **`state_machine/parameters/vqr_wheel_control_parameters.cpp`**: leg segment lengths
  read from VQRWheel's URDF (`thigh_len_` unchanged from legged VQR at 0.2320;
  `shank_len_` recomputed for the Knee->WHEEL-axle offset, ~0.2172 vs legged VQR's
  0.2132 Knee->foot offset); `hip_len_=0.03` is only an approximation of the y-component
  of the URDF's HipY origin (which, unlike legged VQR, also has a nonzero x-component,
  0.056 -- not representable by this single-scalar Lite3-style convention) -- harmless
  here since StandUpState uses a fixed known-good pose, not height-based IK. Added
  `wheel_lock_kd_` (1.0, used to lock the wheels once StandUpState finishes settling)
  and `wheel_vel_limit_`/`wheel_torque_limit_` (currently `58.90`/`20.0`) alongside
  the existing leg-only `Vec3f` fields, following `M20_sdk_deploy`'s
  `control_parameters.h` pattern of keeping legs and wheel as separate
  scalar/vector members rather than expanding everything to `Vec4f`. `joint_vel_limit_`
  is unchanged at the 2026-09-03 retuned value (`14.66`); `torque_limit_`/
  `wheel_torque_limit_` are now `60,60,60`/`20.0` -- **finalized**, matching
  `VQRWHEEL_CFG`'s own `effort_limit` (also updated to `60`/`20` in
  `rl_training/assets/deeprobotics.py`), not a deploy-only override anymore --
  see "2026-09-03 actuator retune sync" below for why (a real torque-ceiling
  problem found while climbing from a lying-down pose, not just a
  gain-tuning issue). Note these two fields aren't actually read anywhere in
  this package's state machine yet -- updated only for documentation
  consistency with `VQRWheel.xml`'s `ctrlrange` (which *is*
  load-bearing). `swing_leg_kp_`/`swing_leg_kd_` are currently a **hybrid**: the OLD
  (`80`/`2`) gains, kept deliberately even though everything else here is on the NEW
  config. All three (fully-OLD, fully-NEW-continuous, and peak) alternatives are
  commented below the
  active code for reference/rollback.
- **`state_machine/standup_state.hpp`/`idle_state.hpp`/`joint_damping_state.hpp`**
  extended from 12 to 16 DOF:
  - `StandUpState` reuses legged VQR's already-validated `(0.0, -0.71, 1.28)` leg pose
    (see `vqr_rl_deploy/README.md` for its 3-iteration derivation history) with wheel=0
    appended per leg, rather than re-deriving a pose from scratch. Wheel `kd` starts at
    0 (free-spinning) during the crouch spline and is raised to `wheel_lock_kd_` once
    the hold phase begins -- the same toggle `M20_sdk_deploy`'s
    `quadruped_wheel/standup_state.hpp` uses, adapted to the single-fixed-pose approach
    instead of M20's height-based IK.
  - `IdleState`'s `JointDataNormalCheck()` extended to 16 DOF: wheel joints have no
    position limit (checked against `±1e6`, effectively unbounded) and are checked
    against `wheel_vel_limit_` instead of `joint_vel_limit_` (which only holds the 3
    leg-joint limits).
  - `JointDampingState` now also damps the wheels (`kd=wheel_lock_kd_`, not 0) so
    joint-damping mode brings the whole robot -- including the wheels -- to a safe stop.
- **`interface/robot/simulation/simulation_interface.hpp`**: default `dof_num` changed
  12->16; the UDP receive payload layout (`ReceiveRobotData()`) resized from 47 to 59
  floats (11 header floats + 16 DOF x {pos, vel, tau}).
- **`interface/robot/simulation/mujoco_simulation.py`**: `MODEL_NAME`/`XML_PATH`/
  `URDF_INIT` updated for VQRWheel (16-value default pose `[0,-0.65,1.3,0]*4`); `dof_num`
  is still derived from `model.nu` so no other hardcoding was needed. Sensor layout
  (`sensordata[16:19]`=accel, `[19:22]`=gyro) is unchanged from legged VQR's
  `VQR.xml` -- these are body-frame site sensors, unaffected by the actuator/joint count.
- **`interface/robot/simulation/pybullet_simulation.py`**: `VQRWheel.urdf` declares
  joints in a *different* order than this package's chosen robot order (all 12 leg
  joints first, then wheels in `FL, FR, HR, HL` order -- note `HR` before `HL`) --
  `jointIdxList` is now built by explicit name lookup against `ROBOT_ORDER_JOINT_NAMES`
  instead of PyBullet's raw per-URDF-declaration enumeration, so the runtime joint order
  is independent of the URDF's own (differently-ordered) joint declarations.
  `footNumList`-style friction settings are applied to whichever PyBullet joint indices
  the `_WHEEL` names resolve to. `receiveJointCmd()`'s hardcoded `12f` struct formats
  generalized to `self.dofNum` (16).
- **`interface/robot/hardware/hardware_interface.hpp` restructured to a 16-DOF
  skeleton, legs wired to the real SDK, wheels a TODO(SDK) stub** (per explicit
  request, alongside the same M20-style wheel safety net added to
  `SimulationInterface`, so both interfaces now match M20's pattern as closely as
  this borrowed 12-joint SDK allows): `dof_num_` changed from
  `motion_sdk::NUM_JOINTS` (12) to a literal `16`, matching `SimulationInterface`.
  A new `LegacyIdx(i)` helper maps this package's 16-index ROBOT order (per-leg
  groups of 4: HipX, HipY, Knee, WHEEL) to the borrowed Lite3/legged-VQR SDK's
  12-index legs-only order (`i%4==3` -> `-1`, i.e. "no such field" -- `motion_sdk`'s
  `RobotCmd`/`RobotState` genuinely have no wheel slot at all, unlike a real
  wheel-capable SDK would). `GetJointPosition/Velocity/Torque()` return `0` for
  wheel indices; `SetJointCommand()` skips dispatching them to the SDK entirely --
  every wheel-related line is marked `TODO(SDK)`, ready for a real wheel-capable
  motor SDK (16-joint CAN topology, calibration, home offsets) to be dropped in
  later. Also added, matching `M20Interface::SetJointCommand`'s exact pattern: a
  safety net forcing `kp=0` for every wheel joint (`i%4==3`) right before dispatch
  regardless of what upstream computed -- currently redundant in both
  `HardwareInterface` (wheel dispatch isn't wired up at all yet) and
  `SimulationInterface` (upstream already computes `kp=0` correctly, verified
  repeatedly), but kept as defense-in-depth /  parity with M20's structure. Still
  true: **do not attempt a real-hardware build of this package** until a
  wheel-capable SDK exists (the 12 legs would drive through the borrowed SDK, but
  the 4 wheels would never move) -- only the MuJoCo/PyBullet simulation paths are
  fully functional right now. Rebuilt and re-verified: `BUILD_SIM=ON` builds clean;
  `BUILD_SIM=OFF` configures and compiles clean (confirming the restructured
  `hardware_interface.hpp` is valid C++ against the current `motion_sdk` headers),
  failing only at the expected aarch64-vs-x86 link step; a full `rl_deploy` +
  MuJoCo stand-up test after the `SimulationInterface` safety-net addition showed
  no change in behavior (wheel `Kp Term` still `0.00`, stand-up still reaches the
  same pose as before).
- `types/custom_types.h`'s `RobotType` enum renamed `Vqr` -> `VqrWheel` (this is a
  separate compiled binary from `vqr_rl_deploy`, so the rename is just for clarity, not
  a compatibility break); `main.cpp`/`state_machine.hpp` updated to match, along with
  the URDF/MJCF paths and `"vqr_wheel"` robot name string.
- Everything else (cubic-spline stand-up timing, PIMPL ONNX pattern, gamepad/keyboard
  input, CMake structure) is unmodified scaffolding, same as `vqr_rl_deploy`.

## 2026-09-03 actuator retune sync

`VQRWHEEL_CFG` (and `VQR_CFG`, legged VQR -- not touched here, out of scope for this
package) had its leg/wheel actuator PD gains and torque/velocity limits changed in
`rl_training` to match this robot's real (lower-torque) actuators:

| | old | new |
|---|---|---|
| leg `effort_limit` (HipX/HipY/Knee) | 84/84/150 | 16/16/16 |
| leg `velocity_limit` | 17.5/17.5/16.1 | 14.66/14.66/14.66 |
| leg `stiffness` | 80.0 | 20.0 |
| leg `damping` | 2.0 (uniform) | 0.762/0.716/0.430 (per-joint) |
| wheel `effort_limit` | 30.0 | 3.0 |
| wheel `velocity_limit` | 30.0 | 58.90 |

Updated to match: `run_policy/vqr_wheel_test_policy_runner_onnx.cpp`'s `kp_`/`kd_`,
`state_machine/parameters/vqr_wheel_control_parameters.cpp`'s `swing_leg_kp_`/
`swing_leg_kd_`/`joint_vel_limit_`/`torque_limit_`/`wheel_vel_limit_`/
`wheel_torque_limit_`, and `vqr_description/vqr_mjcf/mjcf/VQRWheel.xml`'s `<motor>`
`ctrlrange`s (so the sim can't apply more torque than the real actuators can).
`fl_joint_lower_`/`fl_joint_upper_` (position limits) are unchanged -- only PD
gains/torque/velocity limits moved, not URDF joint-range geometry.

**Update, same day: reverted per explicit request.** The checkpoint this package
still targets (`2026-08-28_10-40-50`) was trained under the OLD actuator config, not
this new one -- to test the new LieDown state (and other modes) against that
checkpoint without the mismatch, all three files above were reverted back to the OLD
numbers, with the NEW numbers left commented directly above the active code/values
(and as an XML comment above `VQRWheel.xml`'s `<actuator>` block, since XML attributes
can't be commented out inline) for a fast swap once a checkpoint trained under the new
config exists.

**Update, same day again: swapped back to the NEW retuned config**, per a follow-up
explicit request to test directly against it (this is the part that surfaced the
PD-only stand-hold instability described in the LieDown section below). All three
files now have the NEW numbers active, with the OLD numbers commented
below/alongside for reference/rollback -- same swap mechanism as above, just flipped.

**Update, same day a third time: root-caused the stand-hold instability and applied
a hybrid fix.** Using `mujoco.mj_inverse` at the exact standing pose (base level,
joints at `(0.0,-0.71,1.28,0.0)x4`, wheel-bottoms exactly touching ground) showed the
static holding torque needed there is essentially **0 Nm at every joint** -- so the
16/3 Nm caps were never the real bottleneck. A headless dynamic hold test starting
from that *exact* pose (zero velocity, perfectly balanced) with `kp=20` still sagged
and tipped ~17-25 deg within ~1s -- an unstable equilibrium at that gain, not a torque
insufficiency. The same test with `kp=80`/`kd=2` (the OLD gains) **while keeping the
NEW 16 Nm cap** held cleanly (z within ~0.02 m of `stand_height_`, <1 deg tilt);
`kp=150`/`kd=4` did even better. Conclusion: **the actuators are fine; `kp=20` (the
gain the RL policy was trained with) is simply too soft for classical PD-only
holding.** Applied a hybrid: `swing_leg_kp_`/`swing_leg_kd_` (used only by
`StandUpState`/`JointDampingState`/`LieDownState` -- never by the RL policy runner)
reverted to the OLD `80`/`2`, while `joint_vel_limit_`/`torque_limit_`/
`wheel_vel_limit_`/`wheel_torque_limit_` and `VQRWheel.xml`'s `ctrlrange`s stay at
the NEW (2026-09-03) values. The RL policy runner's own `kp_`/`kd_` in
`vqr_wheel_test_policy_runner_onnx.cpp` are untouched (still `20`/
`[0.762,0.716,0.430]`, matching what the checkpoint was trained with) -- this swap
only affects the non-RL classical-control states.

Rebuilt and re-verified after every one of these swaps (`BUILD_SIM=ON` build
succeeds, MJCF reloads with the corresponding `ctrlrange`s each time).

A new training run (`logs/rsl_rl/vqr_wheel_rough/2026-09-03_15-37-39/`) was also
started under this new actuator config, confirmed (via its `params/env.yaml` dump) to
still use the same `max_cmd_vel_=[4.0,1.0,1.0]`/`decimation=4`/`sim.dt=0.005` as the
`2026-08-28_10-40-50` checkpoint this package currently targets -- no exported
checkpoint from it exists yet.

Separately, `run_policy/vqr_wheel_test_policy_runner_onnx.cpp` currently points
`model_path_` at `policy_new.onnx` (a duplicate of `policy.onnx`, both under
`policy/ppo/`) with `max_cmd_vel_` set to `[1.0,1.0,1.0]` instead of `[4.0,1.0,1.0]` --
this looks like a local manual testing edit (left as-is, not reverted); swap back to
`policy.onnx`/`[4.0,1.0,1.0]` if that was unintentional.

**Update -- `model_path_` changed again (by the user, directly), and a real bug in
`kp_`/`kd_` was found and fixed as a result.** `model_path_` now points at
`policy/policy.onnx` (no `ppo/` subdirectory) -- a fresh export from training run
`2026-09-03_14-45-19` (`model_6500.pt`, exported today). That run has no
`params/env.yaml`/git-diff snapshot, so its exact actuator config can't be read back
directly from its own logs.

First pass here **incorrectly guessed**, from the run directory's start-time
(14:45) predating `deeprobotics.py`'s first 2026-09-03 retune edit (mtime 15:34 that
day), that this checkpoint used the *same* config as `2026-08-28_10-40-50`
(stiffness=80, uniform damping=2.0) -- **the user corrected this directly**: this
checkpoint was actually trained under `deeprobotics.py`'s *current* `VQRWHEEL_CFG`
(stiffness=80, damping PER-JOINT `{HipX:1.524, HipY:1.431, Knee:0.859}`,
effort_limit=60/60/60 leg, 20 wheel). Lesson: a training run's start-timestamp in its
directory name doesn't reliably bound when its config was finalized (the user may
edit and iterate before/during a run in ways a file mtime alone can't reconstruct) --
should have asked instead of inferring from timestamps.

This whole exercise surfaced a real bug, twice in a row: `kp_`/`kd_` in
`vqr_wheel_test_policy_runner_onnx.cpp` must match whatever the *currently-loaded*
checkpoint (`model_path_`) was actually trained with -- not `deeprobotics.py`'s
state as of any particular guess, and not blindly kept in sync with whichever
retune happened most recently either. Using the wrong `kp_`/`kd_` during
`RLControlMode` means the policy runs against different low-level PD dynamics than
what it learned to expect -- a real sim-to-deploy mismatch, not just a documentation
inconsistency. **Current (correct) values**: `kp=80`, `kd=[1.524,1.431,0.859,0.6]`
per leg (ROBOT order) -- matching `deeprobotics.py`'s current `VQRWHEEL_CFG`, per the
user's direct confirmation that this is what `2026-09-03_14-45-19` was trained under.
The *older* `2026-08-28_10-40-50` checkpoint really did use uniform `kd=2.0`
(confirmed from its own `env.yaml`) -- that value is kept commented for reference,
correct only if `model_path_` is ever pointed back at that older checkpoint.
`swing_leg_kp_`/`swing_leg_kd_` in `control_parameters.cpp` -- used only by the
classical-control states, never by the RL policy runner -- needed no change
throughout any of this.

## Retroid direction issue -- root-caused to the POLICY, not the deploy code

**Follow-up after the clipping fix below**: the user reported it was still wrong at
`max_cmd_vel_=3.0` -- forward now made the robot crouch into a walking-like stance
with the wheels barely spinning (little actual translation), and backward turned
into a slow forward creep. Asked to test this myself rather than guess again.

**Method**: wrote a temporary synthetic UDP sender (`fake_retroid_sender.py`, since
deleted) that crafts real `RetroidGamepadData` packets (header `0x55 0x66`, id=1,
correct CRC16-as-byte-sum, exact struct layout from `gamepad_keys.h`) and drove
`RetroidGamepadInterface` end-to-end through the real port-12121 UDP path -- no
guessing about button bit positions either (confirmed `A`=`buttons[6]`,
`Y`=`buttons[9]` from the bitfield order in `RetroidKeys` plus how
`retroid_gamepad.cpp` derives `keys.value` from the 16 raw channels). Sequence: Y
(stand up) -> wait -> A (RL control) -> full-forward (`left_axis_y=+1000`, 4s) ->
neutral -> full-backward (`left_axis_y=-1000`, 4s). Also added (temporarily,
since removed) a throttled debug print of `cmd_vel_normlized`/`cmd_vel` in
`GetRobotAction()`, and of the MuJoCo base X/Y/Z position at 2 Hz in
`mujoco_simulation.py`.

**Result**: `cmd_vel` was computed exactly right the whole time --
`left_axis_y=+1000` -> `cmd_vel=(+3,0,0)`, `left_axis_y=-1000` -> `cmd_vel=(-3,0,0)`,
no sign error, no unclipped overshoot (the earlier clipping fix wasn't even
exercised at exactly `+-1000`, and made no difference to this issue either way). But
the robot's actual base X position **kept increasing (or at best plateaued)
throughout both the forward and the nominally-"backward" phase** -- it never
reversed. I.e., the deploy pipeline (interface -> user command -> policy runner's
`cmd_vel` -> observation) is provably correct; **the loaded policy itself doesn't
produce real backward locomotion for `cmd_vel_x=-3`**, matching the user's
description exactly (a weak/confused-looking gait rather than a clean directional
bug).

**Conclusion (first pass): this is very likely a training-side issue** (the
checkpoint currently loaded -- from run `2026-09-03_14-45-19` -- may not have
learned robust backward walking), **not a bug in this deploy package.**

**Follow-up -- narrowed further, and this part *is* explained by the interface
difference after all** (the user asked why `KeyboardInterface` never showed this at
the same `max_cmd_vel_=3.0`): re-ran the synthetic-UDP-sender test with two
variants of the same forward->backward transition:

1. **Gradual ramp** -- `left_axis_y` stepped `+100` (`=0.1` normalized) every
   `0.15s` from `0` up to `+1000`, held, then ramped back down through `0` to
   `-1000` the same way -- deliberately mimicking exactly what
   `KeyboardInterface`'s `AXIS_STEP=0.1`-per-keypress can ever produce (it
   structurally can never jump directly between two arbitrary values; every
   change goes through every intermediate step).
2. **Instant flip** -- `left_axis_y` jumping straight from `+1000` to `-1000` in a
   single UDP packet, exactly what a real gamepad stick flicked hard from one
   extreme to the other produces (and what the very first end-to-end test above
   did).

Result: under the **gradual ramp**, the SAME policy produced clean, fast, correctly
*negative*-direction locomotion -- base X went from `+1.3` to `-10.2` over about
4.5 s (~2.2 m/s average, close to the commanded 3 m/s), a real and unambiguous
reversal. Under the **instant flip** (run back-to-back against the same live
process right after), the same policy did *not* reproduce that clean reversal --
motion stayed weak/forward-biased, matching the originally-reported symptom.

**Revised conclusion**: the policy *can* walk backward correctly -- what it can't
handle gracefully is an **abrupt, large-magnitude command reversal within a single
control step** (which only a gamepad stick flick can produce; `KeyboardInterface`'s
incremental `+=0.1`-per-press design means the commanded velocity always passes
through every intermediate value on its way from one extreme to the other, however
fast the user presses, so the robot's actual momentum/state never has to reconcile
with a suddenly-opposite target the way it does with an instant flip). This is
still a training/policy-robustness question, not a deploy-code bug -- but it's a
much more specific one than "doesn't walk backward": it's specifically about how
this checkpoint responds to a large instantaneous command discontinuity. Whether to
address it by retraining with more abrupt command changes in the curriculum, adding
command-rate limiting somewhere in the deploy pipeline (e.g. slew-rate-limiting
`cmd_vel` before it reaches the policy, similar in spirit to what
`KeyboardInterface`'s ramping already does for free), or accepting it as an operator
constraint (avoid full-deflection direction flicks on Retroid) is a product decision
for the user to make, not something this investigation should decide unilaterally.

**Confirmed on real hardware by the user**: pushing the stick gradually (not
flicking it) does produce correct locomotion in the requested direction, matching
the "gradual ramp" result above exactly. This is no longer just a simulated
finding.

The interface-side clipping fix below is still a legitimate, worthwhile correctness
fix on its own merits, independent of this finding (a real gap vs.
`KeyboardInterface`), even though it turned out not to be the cause of this
particular symptom.

## Retroid/Skydroid axis-clipping bug (found, pending confirmation)

**Symptom (user-reported)**: forward/backward direction is wrong on `RetroidGamepadInterface`
-- but *only* at `max_cmd_vel_=3.0` (in `vqr_wheel_test_policy_runner_onnx.cpp`), not at
`max_cmd_vel_=2.0`; and not reproducible with `KeyboardInterface` at `max_cmd_vel_=3.0`
either. A first attempt at a fix (negating `left_axis_y`, matching
`xbox_gamepad_interface.hpp`'s convention) did **not** resolve the `max_cmd_vel_=3.0`
case and was reverted (per explicit request) once that became clear -- the bug isn't a
simple, constant axis sign error, since a sign flip can't be magnitude-dependent.

**Root cause (hypothesis, not yet independently confirmed by the user)**:
`RetroidGamepadInterface` never clips `forward_vel_scale`/`side_vel_scale`/
`turnning_vel_scale` to `[-1,1]` before they get multiplied by `max_cmd_vel_` --
unlike `KeyboardInterface`, which explicitly does (`ClipNumber(...,-1.,1.)`).
`left_axis_y`/`left_axis_x`/`right_axis_x` are raw `int16_t` read straight off the UDP
packet, divided by the nominal calibration constant `kJoystickRange=1000`
(`retroid_gamepad.cpp`) -- nothing enforces the raw value actually stays within
`+-1000` (controller calibration drift, or a stick pushed past its nominal endpoint,
could report slightly beyond that). An unclipped overshoot (e.g. axis reads `1.1`
instead of `1.0`) reaches further outside `+-max_cmd_vel_` at `max_cmd_vel_=3.0`
(overshoot to `3.3`) than the same relative overshoot would at `2.0` (`2.2`) -- and
IsaacLab's command generator samples training commands strictly within the
configured range (exactly `+-3.0`, never beyond), so a large enough overshoot is a
genuinely out-of-distribution observation for the policy, which could plausibly
produce visibly wrong (even direction-flipped) actions. This matches all three
reported data points (retroid+3.0 broken, keyboard+3.0 fine because keyboard's axis
is hard-clipped so it can never overshoot `+-3.0`, retroid+2.0 fine because the
smaller absolute overshoot stays closer to the trained range).

**Applied**: clip `forward_vel_scale`/`side_vel_scale`/`turnning_vel_scale` to
`[-1,1]` in both `RetroidGamepadInterface` and `SkydroidGamepadInterface` (using the
existing `functions::LimitNumber` from `basic_function.hpp`, rather than duplicating
`KeyboardInterface`'s private `ClipNumber`), matching what `KeyboardInterface`
already guarantees. **Not yet independently verified against real Retroid hardware**
-- ask the user to re-test the exact `max_cmd_vel_=3.0` + Retroid repro that was
broken before this change.

## Keyboard axis-decay bug (found and fixed)

**Symptom (user-reported)**: pressing `s` sometimes didn't make the robot go
backward -- it kept going forward.

**Root cause**: `KeyboardInterface::Run()` used a plain blocking `read(STDIN_FILENO,
...)` with no timeout, so the loop only ever executed once per keystroke. The
`>300ms since last w/a/s/d/q/e -> axis returns to 0` decay check (and the
`ClipNumber` calls) lived inside that same `if(read(...))` block -- meaning they
only ran *when a new key happened to arrive*, never during an idle gap. Concretely:
holding/tapping `w` repeatedly ramps `forward_vel_scale` up toward `+1.0`
(`+=`/`-=0.1` per press, clipped to `[-1,1]`, not an absolute "hold to move" axis);
if the user then paused for longer than 300ms, the robot kept cruising forward the
entire time (decay never got a chance to run); the next keystroke -- even `s` --
first re-armed `forward_time_record = current_time` in the same tick as applying
its `-0.1`, so the overdue decay-to-0 got skipped and only the incremental `-0.1`
applied to the stale near-`+1.0` value (e.g. `1.0 -> 0.9`) -- still strongly
forward.

**Fixed**: replaced the blocking `read()` with a `select()`-based 50ms poll on
`STDIN_FILENO`, and moved the decay/`ClipNumber` block outside the
`if(got a key)` gate so it runs every ~50ms regardless of whether a new key
arrived. A stale axis value can no longer survive an idle gap.

**Verified** with a temporary debug print of `forward_vel_scale` (added, checked,
then removed) against the exact repro: ramped `fwd` to `1` via repeated `w`,
confirmed it now decays to `0` on its own after ~300ms idle (previously would have
stayed frozen at `1` indefinitely), then pressed `s` once from that decayed `0`
baseline and got `-0.1` immediately (backward), not a marginal `0.9`.

## LieDown state (M20 parity)

Added `state_machine/liedown_state.hpp` + `RobotMotionState::LieDown`/`StateName::kLieDown`
(value `4`, matching `M20_sdk_deploy`'s enum), ported from
`M20_sdk_deploy/state_machine/quadruped_wheel/liedown_state.hpp`: a controlled
crouch-down (spline to a low pose over `liedown_duration_`) -> compliant hold
(`kp=0`, legs keep `swing_leg_kd_`, wheels locked at `wheel_lock_kd_`) -> full release
(`kp=kd=0` everywhere) sequence, so the robot can be parked/powered down safely
instead of just going limp. Unlike M20's IK-based crouch (which negates HipX/HipY/Knee
for the hind legs), this port reuses the 2-link IK formula (`GetHipYPosByHeight`/
`GetKneePosByHeight`, law of cosines on `thigh_len_`/`shank_len_`) but keeps this
package's uniform-across-all-4-legs convention, and targets `cp_ptr_->pre_height_`
(0.12 m, already defined) rather than a new magic number -- `HipY=-1.18, Knee=2.60` at
that height, both comfortably inside the URDF's joint limits.
Keyboard: `x` (while in RL control) enters LieDown; `z` (while lying down) returns to
StandUp. Wired into `state_machine.hpp`/`custom_types.h` the same way the other states
are.

**Retroid gamepad**: `retroid_gamepad_interface.hpp` had no LieDown mapping at all
(it only handled `Y`=WaitingForStand->StandUp and `A`=StandingUp->RLControl, with no
case for `RLControlMode` or `LieDown`) -- added `B` (while in RL control) to enter
LieDown, and reused `Y` (while lying down) to return to StandUp, mirroring the
keyboard's reuse of `z` for both idle->standup and liedown->standup.

**Bug found (real hardware/sim testing, user-reported) and fixed: kp got stuck at 0
on the second LieDown activation onward.** Symptom: the first `x` press lowered the
robot with visible (if slightly jerky) control; after standing back up and
re-entering RL control, a second `x` press made the robot collapse straight down
with no control at all. Root cause: `Run()`'s Phase 1 branch assigned
`joint_cmd_.col(1)/.col(3)/.col(2)` (target pos/vel, kd) every tick but never
`.col(0)` (kp) -- that column was only ever set once, in the constructor.
`LieDownState` is constructed exactly once and reused across every activation (not
re-constructed per `OnEnter()`), and Phase 2/3 do `joint_cmd_ = MatXf::Zero(16,5)`
(wiping kp to 0 along with everything else) before the state is exited -- so a
second activation's Phase 1 started with kp still stuck at 0 from the *previous*
activation's Phase 2/3, meaning zero position-holding stiffness, just kd damping
chasing a moving spline target. Fixed: `joint_cmd_.col(0) = kp_;` is now
re-assigned every Phase-1 tick, same as `.col(2)` already was. Verified against the
exact repro (`z`->`c`->`x`->`z`->`c`->`x`, i.e. two full LieDown activations in one
`rl_deploy` session): captured the live UDP joint-command stream mid-Phase-1 on the
second activation and confirmed `Kp Term` reads `80.00` per leg (not `0.00`).

**Bug found and fixed while testing this**: `state_machine/rl_control_state_onnx.hpp`'s
`GetNextStateName()` unconditionally returned `StateName::kRLControl`, ignoring
`target_mode` entirely -- so pressing `x` set `target_mode=LieDown` correctly (per the
keyboard handler above) but the state machine never actually left `RLControlMode`,
silently. Fixed to check `target_mode==LieDown` and return `kLieDown` in that case
(falling through to `kRLControl` otherwise). This was latent scaffolding carried over
unchanged from `vqr_rl_deploy` (which has the identical bug, but it's harmless there
since legged VQR has no second state to transition into from RL control -- `x` simply
had nothing to request before now).

**Update -- root-caused, but the fix below turned out to be incomplete (see the next
update).** This caveat originally reported that holding a fixed leg pose via PD-only
control (`kp=20`, matching the RL policy's own gains) sagged/tipped the robot within
1s under the new, lower torque budget (`effort_limit` 84->16 N.m). Root cause, via
`mj_inverse`: the ideal standing pose needs ~0 Nm of static torque, so the torque cap
was never the bottleneck for *holding* that exact pose -- `kp=20` was just too soft,
an unstable equilibrium that diverges from any infinitesimal perturbation. `kp=80`
(the OLD gain) held cleanly within the *same* 16 Nm cap, starting from that exact
idealized pose. Since `swing_leg_kp_`/`swing_leg_kd_` (used by `StandUpState`/
`JointDampingState`/`LieDownState`) is a separate parameter from the RL policy
runner's own `kp_`/`kd_`, it was reverted to `80`/`2` without touching the policy
runner at all.

**Update -- the above test was incomplete, per user follow-up: it only tested
*holding* an already-ideal pose, not *climbing* to it from lying down.** Redone
properly: settle the robot into a realistic lying-down rest (full release, gravity
picks the joint angles, same as `LieDownState`'s phase 3), then run `StandUpState`'s
actual cubic-spline climb from there. Result: **fails**, plateauing around
`z~=0.27 m` (vs. `stand_height_=0.49 m`) at ~28-30 deg tilt -- confirmed at spline
durations from 1.5s up to 8s (ruling out "too fast" as the cause). Swept `kp` from 80
up to 1000 with the torque cap unchanged: **no improvement** (`z` only crept from
0.269 to 0.295 m) -- conclusive proof this is a genuine **torque-ceiling** problem
(some joint needs more than 16 Nm to climb out of a crouch), not a gain-tuning
problem; capped torque means capped torque regardless of how stiff the PD target
tracking is.

Checked the real motor datasheets the user provided
(`rl_training/SEAF70A16NG01...pdf` for the leg joints, `rl_training/
6109611805316420072.jpg` for the wheel hub motor): `effort_limit=16`/`3` match
**exactly** the motors' 输出端连续堵转扭矩/额定扭矩 (continuous/rated output
torque) -- legitimate real specs, not guesses. But both datasheets also list a much
higher 输出端峰值扭矩 (peak output torque): **96 Nm** (leg, 6x) and **24 Nm** (wheel,
8x) -- safe for short bursts (like a few-second stand-up transient), not sustained
duty (would overheat the motor).

**Applied, per explicit request: raised the ctrlrange/torque_limit_ ceiling from the
continuous rating toward (but not all the way to) the peak rating** (`VQRWheel.xml`'s
leg `<motor>` `ctrlrange`s -> `60`; wheel -> `24`; `torque_limit_` -> `60,60,60`;
`wheel_torque_limit_` -> `24.0`) -- temporary, for this classical-control climb
specifically; `VQRWHEEL_CFG`'s own `effort_limit` (16/3, used for RL training) is
untouched, so this doesn't change what the trained policy itself does (it was
trained under the continuous cap and should never actually need more than that).
Re-ran the same lying-down climb test first at the full peak (`96`/`24`): **major
improvement** -- reaches `z~=0.406 m` (83% of `stand_height_`) at only ~2 deg tilt
(down from 28-30 deg), with peak torques seen up to ~23.6 Nm at the hind knees.

**Then dialed back to `60` Nm for the legs**, per a follow-up explicit request for a
deliberate margin below the full 96 Nm peak (wheel left at its full 24 Nm peak, since
it never came close to that cap either way). Re-verified: **identical outcome** to
the full-96 test (`z~=0.406 m`, ~2 deg tilt, same ~23.6 Nm peak knee torque) -- 60 Nm
was never actually the constraint, so dialing back to it costs nothing while leaving
real headroom below the motor's true peak.

**Finalized -- pushed upstream into `VQRWHEEL_CFG` itself, and wheel torque dialed to
20.** Decided this isn't just a deploy-side workaround: `rl_training/assets/
deeprobotics.py`'s `VQRWHEEL_CFG` "joint" actuator `effort_limit` changed
`16.0`->`60.0` (all three leg joints) and "wheel" actuator `effort_limit` changed
`3.0`->`20.0` (below the wheel motor's 24 Nm peak, above its 3 Nm continuous rating --
chosen the same way the leg's 60 sits below its 96 peak). `velocity_limit` was left
unchanged (`14.66`/`58.90`) -- these already matched the two motors' *rated* output
speed converted to rad/s (140 rpm and 562.5 rpm respectively, from the same
datasheets) even before this change, so no update was needed there. This package's
`vqr_wheel_control_parameters.cpp` (`wheel_torque_limit_`) and `VQRWheel.xml` (wheel
`ctrlrange`) were updated from `24`/`24.0` to `20`/`20.0` to match, and re-verified
with the same headless lying-down climb test: **identical outcome** (`z~=0.406 m`,
~2 deg tilt, max wheel torque seen ~4.8 Nm -- comfortably under 20).

**Still not yet perfect**: settles a bit short of the full `0.49 m` target rather
than reaching it exactly -- unclear whether that's this quick Python PD replica not
exactly matching the compiled C++ `StandUpState` (spline formula, phase-2 hold
timing) or a genuine remaining shortfall. **Watch a real stand-up in the actual
`rl_deploy` + MuJoCo viewer** (not this headless Python replica) to confirm before
trusting it. Also note: this checkpoint (`2026-08-28_10-40-50`) was trained before
any of these actuator changes -- a future checkpoint trained under `effort_limit=60`/
`20` would need this package's `run_policy/vqr_wheel_test_policy_runner_onnx.cpp`
re-pointed at it (see the "policy_new.onnx"/`max_cmd_vel_` note above).

## 2026-09-08 LieDown M20-parity review: dedicated gain/wheel-lock/height

Per explicit request ("rà soát lại giúp tôi các khác biệt trong các state
controller giữa vqr wheel và m20" -- review the differences between
VQR_Wheel's and M20's state controllers), systematically diffed
`liedown_state.hpp`/`standup_state.hpp`/`joint_damping_state.hpp` against
`M20_sdk_deploy/state_machine/quadruped_wheel/`'s equivalents, parameter by
parameter. Two prior gaps (the kp-stuck-at-0-on-2nd-activation bug, and
wheel-lock-from-Phase-1 with the accompanying `planning_joint_vel` override)
were already fixed/present (see the "LieDown state (M20 parity)" section
above). Three more were found and closed this round: M20's own
`LieDownState` uses a **dedicated** kp/kd/wheel-kd/height, not the same
`swing_leg_kp_`/`swing_leg_kd_`/`wheel_lock_kd_`/`pre_height_` its own
`StandUpState` uses -- this port had been reusing the StandUp-shared values
for LieDown throughout, missing that M20 treats LieDown as deliberately
stiffer/deeper/harder-locked than StandUp's crouch.

Added `liedown_leg_kp_`/`liedown_leg_kd_`, `liedown_wheel_kd_`, and
`liedown_height_` to `control_parameters.h`/`vqr_wheel_control_parameters.cpp`,
and switched `liedown_state.hpp` to use them instead of the StandUp-shared
fields. Rather than copying M20's raw absolute numbers (`kp=300`, `kd=4.5`,
wheel `kd=3`, `h=0.03`), applied the same *ratios* M20 itself uses between its
own LieDown and StandUp values to VQR's own already torque-budget-verified
`swing_leg_kp_`/`swing_leg_kd_`/`wheel_lock_kd_`/`pre_height_` -- VQR's
actuators are torque-capped differently (60 Nm leg / 20 Nm wheel) than M20's
(76.4 Nm leg), so blindly copying M20's absolute gains risked exceeding a
budget never tested with them:

| | M20 LieDown | M20 StandUp | ratio | VQR StandUp (existing) | VQR LieDown (new) |
|---|---|---|---|---|---|
| leg kp | 300 | 200 | x1.5 | 80 | **120** |
| leg kd | 4.5 | 4 | x1.125 | [1.524,1.431,0.859] | **[1.7145,1.60988,0.966375]** |
| wheel kd | 3 | 1 | x3 | 1.0 | **3.0** |
| crouch height | 0.03 | 0.12 | x0.25 | 0.12 (`pre_height_`) | **0.03** |

(`joint_damping_state.hpp` was re-checked too and left untouched -- it
already correctly reuses the StandUp-style shared values, matching M20's own
`JointDampingState`, which also reuses its StandUp's wheel-lock value rather
than LieDown's stronger one.)

**Verification pitfall caught mid-testing, worth recording**: headless MuJoCo
testing of the new `liedown_height_=0.03` initially looked alarming --
measuring *max* torque over the whole settle run showed the Knee saturating
at the 60 Nm cap, and swept several shallower heights (0.05-0.12) that all
showed the same saturation, which made no sense against the sensible-looking
`h=0.03` value. Root cause: that metric captured the **transient** spike
during the crouch-down motion itself, not sustained holding load -- even the
already-shipped, already-safe `h=0.12` StandUp-style config showed the exact
same transient spike. Redone measuring only **steady-state** torque (last
~500 of ~4500 steps, well after settling): `h=0.03` holds at only **~5.6 Nm**
Knee torque (`z~=0.168 m`, tilt~=1.08 deg) -- comfortably under the 60 Nm
cap, on par with or better than every other height tested. Lesson: always
separate transient-approach torque from steady-state-hold torque when
validating a new setpoint against a torque cap; a spline's approach phase can
legitimately saturate briefly without the final resting pose being unsafe.

**Live-verified end-to-end** (`rl_deploy` + `mujoco_simulation.py`, driven by
a synthetic Retroid UDP sender through two full `StandUp -> RLControl ->
LieDown` cycles, i.e. testing both the first *and* second LieDown activation
in one session): `Kp Term` correctly reads `120.00` per leg through Phase 1
of *both* activations (confirming the new dedicated gain is applied, and
that the earlier kp-stuck-at-0-on-2nd-activation fix still holds with these
new fields), drops to `0.00` at the Phase 1/2 boundary as expected, peak
torque during the crouch spline stayed under ~34 Nm (well inside the 60 Nm
cap) on every joint across both cycles, and the robot settled to a low,
level crouch (`RPY` roll/pitch ~0) each time with no instability.

## Gaps vs. M20_sdk_deploy not yet addressed

- **TODO, deferred by explicit request -- `SafeController`**
  (`M20_sdk_deploy/include/utils/safe_controller.hpp`): a separate watchdog
  subsystem M20's `QwStateMachine` runs alongside the state machine itself
  (`sc_ptr_->SetRobotDataSource()`/`SetUserCommandDataSource()`/`Start()`/`Stop()`),
  continuously monitoring robot + user-command data -- not ported here yet. This
  package's only safety mechanism today is each state's own `LoseControlJudge()`
  (checked once per control loop in `state_machine.hpp`'s `Run()`) -- for
  `RLControlStateONNX` that's just an IMU-tip-over check (`PostureUnsafeCheck()`);
  no other state checks anything beyond `target_mode==JointDamping`.
  M20's version checks 6 things: driver status, joint-data NaN, motor temperature,
  IMU-data NaN, battery level, and user-command "heartbeat" (has the gamepad/remote
  command stream gone stale). **When this gets picked up**: driver/temperature/battery/
  joint-NaN checks are currently moot -- neither `SimulationInterface` nor
  `hardware_interface.hpp` report real values for those (sim has no driver/battery
  concept; the Lite3-SDK hardware stand-in isn't wheel-aware at all, see below) -- so
  start with just posture (already covered) + **command-heartbeat**, which *is*
  meaningful today since this package already has `RetroidGamepadInterface` (a real
  UDP remote link, port 12121, that can actually drop out) with nothing currently
  detecting a stale/lost connection and forcing `JointDamping`.
- ~~Real hardware wheel-command safety enforcement~~ **DONE**: both
  `HardwareInterface::SetJointCommand()` and `SimulationInterface::SetJointCommand()`
  now force `kp=0` for every wheel joint (`i%4==3`) right before dispatch, matching
  `M20Interface::SetJointCommand()`'s exact pattern -- see the "hardware +
  simulation, M20 parity" entry above. Still only a *pattern* match on the hardware
  side, since wheel command dispatch itself is still a `TODO(SDK)` stub there (no
  wheel-capable SDK to actually send it to yet).

## Verified so far

- `cmake .. -DBUILD_PLATFORM=x86 -DBUILD_SIM=ON -DSEND_REMOTE=OFF && make -j` builds
  `rl_deploy` cleanly.
- `cmake .. -DBUILD_PLATFORM=x86 -DBUILD_SIM=OFF -DSEND_REMOTE=OFF && make -j`
  configures fine but fails at the *link* step with `error adding symbols: file in
  wrong format` -- expected, `libmotion_sdk.a` is aarch64-only (same as `vqr_rl_deploy`;
  additionally, see above, this SDK is also structurally 12-DOF-only and would not be
  usable for this robot even on matching hardware).
- `vqr_description/vqr_mjcf/mjcf/VQRWheel.xml` (and the `VQRWheel_stair.xml` terrain
  wrapper) load and step correctly in MuJoCo 3.x; an offscreen render at the RL default
  pose shows correctly-oriented mesh geometry.
- Running `mujoco_simulation.py` + `./build/rl_deploy` (from inside `build/`) together:
  MuJoCo bridge reports `dof = 16` and the correct default pose
  (`[0,-0.65,1.3,0]*4`); the C++ side loads the ONNX model (`obs_dim: 57, action_dim:
  16`), `robot2policy_idx`/`policy2robot_idx` come out as the expected non-identity
  permutation (spot-checked by hand, e.g. `robot2policy_idx[3]=4`,
  `policy2robot_idx[3]=12`, both consistent with the ROBOT/POLICY order definitions
  above), `SimulationInterface` connects over UDP, and the state machine reaches
  `IdleState` ("Waiting for stand up...") cleanly.

**Not yet independently verified**: an actual stand-up + RL-control transition (this
smoke test only confirmed the model loads and the state machine idles correctly --
it did not drive the keyboard interface through `z`/`c` to watch the robot actually
stand and walk in the viewer). Do that before trusting the wheel `kd` toggle or the
reused `(0.0,-0.71,1.28)` leg pose unattended.

## Build

Same as `vqr_rl_deploy` / upstream Lite3_rl_deploy:

```bash
sudo apt-get install libdw-dev
wget https://raw.githubusercontent.com/bombela/backward-cpp/master/backward.hpp && sudo mv backward.hpp /usr/include
pip install pybullet "numpy < 2.0" mujoco colorama

mkdir build && cd build
cmake .. -DBUILD_PLATFORM=x86 -DBUILD_SIM=ON -DSEND_REMOTE=OFF
make -j
```

Run (two terminals, both from inside `build/` -- `GetAbsPath()` resolves the ONNX model
path relative to the current working directory, so running `rl_deploy` from the package
root instead of `build/` will fail to find `policy/ppo/policy.onnx`):
```bash
python3 ../interface/robot/simulation/mujoco_simulation.py   # or pybullet_simulation.py
./rl_deploy
```
Keyboard: `z`=stand up, `c`=enter RL control, `wasd`=move, `qe`=turn, `x`=lie down
(from RL control), `z` again (while lying down)=stand back up, `r`=joint damping.

For a real-hardware build (`-DBUILD_PLATFORM=arm -DBUILD_SIM=OFF`): **do not** -- see
`interface/robot/hardware/hardware_interface.hpp`'s header comment. A wheel-capable
motor SDK does not exist in this repo yet.
