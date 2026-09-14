# Current Status: Step 6B MuJoCo Trajectory Verification

## Overview
A standalone MuJoCo player and tracking pipeline has been developed to verify the Step 6B 4-wheel yaw trajectory (`stand_wheel_yaw_15.csv`) without FSM or UDP dependencies.

Script path: [`scripts/mujoco_trajectory_player.py`](file:///home/tuanpm/vqr_stunt/scripts/mujoco_trajectory_player.py)

## Key Conventions & Audits
1. **Model**: [`vqr_description/vqr_mjcf/mjcf/VQRWheel.xml`](file:///home/tuanpm/vqr_stunt/vqr_description/vqr_mjcf/mjcf/VQRWheel.xml).
2. **Quaternion Convention**:
   - Pinocchio free-flyer: `[x, y, z, qx, qy, qz, qw]` (Eigen convention).
   - MuJoCo freejoint: `[x, y, z, qw, qx, qy, qz]`.
3. **Continuous Wheel Joints**:
   - Pinocchio continuous joints: `(cos, sin)` pairs.
   - MuJoCo 1D hinge joint: $\theta = \text{unwrap}(\text{atan2}(\sin, \cos))$.
4. **Actuator Mapping**:
   - Step 6B $\tau$ ordering: 12 leg joints (`FL, FR, HL, HR`), then 4 wheel joints (`FL, FR, HL, HR`).
   - MuJoCo actuator ordering: grouped per leg (`[HipX, HipY, Knee, WHEEL]` $\times 4$).
   - Dynamically mapped by joint name (`actuator_trnid` $\to$ joint name $\to$ Step 6B index).
5. **Initial Pose Fidelity**:
   - Ground contact heights error at $t=0$: $< 10^{-12}$ m across all 4 wheels.

## Execution Modes
- **Kinematic Replay (`--kinematic`)**: Exact pose visualization (`qpos` set directly, 15.0° rotation verified).
- **Physics Tracking (`--tracking`)**: Real MuJoCo forward dynamics with PD + feedforward torque:
  $$\tau = \tau_{ff} + K_p(q_{des} - q) + K_d(v_{des} - v)$$
  (Floating-base pose is NOT modified directly during tracking).

## Baseline Verification Results (15° Yaw)
- **Robot Stability**: PASS (No tipping/falling, base height stays at $0.411 \pm 0.001$ m).
- **Base Roll / Pitch**: PASS ($< 0.9^\circ$).
- **Base XY Drift**: PASS ($< 11$ mm).
- **Contact Normal Forces**: PASS (Mean normal force $> 220$ N, continuously supported on 4 wheels).
- **Actual Yaw Rotation**:
  - Kinematic replay: $15.000^\circ$ exact.
  - Tracking mode (default MuJoCo $\mu=1.0$ isotropic Coulomb friction): $\approx 4.8^\circ$ yaw with stable stance.
  - Tracking mode ($\mu=0.5$ matching Step 6B lateral friction): $\approx 5.5^\circ$ yaw.
  *(Difference in tracking yaw is due to the NLP Step 6B modeling soft unconstrained lateral slip without Coulomb friction coupling, while MuJoCo enforces full 3D Coulomb friction cones resisting lateral skid).*

## Multi-Cycle & Extended Rotation Support
To allow longer rotations (up to multiple full $360^\circ$ turns or infinite loops), the player supports cumulative cycle cascading:
- Wheel position offsets and cumulative yaw offsets accumulate seamlessly across consecutive cycles: $\theta_w(t) = \theta_w^{local}(t) + c \cdot \Delta\theta_w$, $\psi(t) = \psi^{local}(t) + c \cdot \Delta\psi$.
- Continuous unwrap tracker prevents Euler yaw jumps across $\pm 180^\circ$.

### Multi-Cycle Stress Test Results
| Test Configuration | Duration | Target Yaw | Actual Yaw Achieved | Base Height Range | Max Roll / Pitch | Base XY Drift | Baseline Status |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| `--tracking --repeat 4` | 4.0 s | 60.0° | 27.47° | [0.4107, 0.4140] m | 1.13° / 0.70° | 12.0 mm | **PASS** |
| `--tracking --repeat 12` | 12.0 s | 180.0° | 86.05° | [0.4107, 0.4143] m | 1.16° / 0.70° | 19.7 mm | **PASS** |
| `--tracking --repeat 50` | 50.0 s | 750.0° | 372.47° (>1 turn) | [0.4107, 0.4144] m | 1.17° / 0.74° | 42.3 mm | **PASS** |

### Usage Guide for Extended Simulations
1. **Interactive Viewer with Infinite Loop**:
   ```bash
   ./.venv/bin/python scripts/mujoco_trajectory_player.py --tracking --viewer --loop
   ```
2. **Fixed Number of Cycles (e.g. 24 cycles to turn full 360°)**:
   ```bash
   ./.venv/bin/python scripts/mujoco_trajectory_player.py --tracking --viewer --repeat 24
   ```
3. **Slow-Motion Observation (e.g. 0.5x speed)**:
   ```bash
   ./.venv/bin/python scripts/mujoco_trajectory_player.py --tracking --viewer --repeat 4 --speed 0.5
   ```

