# Four-Wheel Yaw NLP Audit — Step 6B

## Scope

Step 6B extends the Step 6A direct-multiple-shooting NLP to
`nominal four-wheel stance -> rotate in place -> stop`. All four contacts stay
active. There is no posture shift, unloading, lifting, contact switching, MPC,
or controller/hardware integration.

The default problem uses a 15 degree yaw target, `T=1.0 s`, `N=20`, and
`dt=0.05 s`. Contact and force order is `FL, FR, HL, HR`.

## Transcription and boundary conditions

The decision variables are unchanged from Step 6A:

- `q_k in R^27`, `v_k in R^22` at 21 nodes.
- `tau_k in R^16`, `lambda_world_k in R^12` on 20 intervals.
- Total: 1589 variables, 1297 equality rows, and 400 friction inequalities.

For every interval, the full Pinocchio/CasADi rigid-body dynamics and
semi-implicit manifold update are used:

```
a_k      = forward_dynamics(q_k, v_k, tau_k, lambda_world_k)
v_pred   = v_k + dt * a_k
q_pred   = integrate(q_k, dt * v_pred)
0        = v_{k+1} - v_pred
0        = difference(q_pred, q_{k+1})
```

Interior nodes enforce all four contact heights and `A_hard(q_k) v_k = 0`.
The endpoint versions are implied by the boundary states and are omitted to
avoid duplicate equality rows. The hard skid-steer rows remain `[normal,
rolling]` per wheel. Lateral velocity is never an equality.

Initial constraints fix the complete nominal configuration and zero velocity.
Terminal constraints select generalized coordinates by the runtime joint
mapping and enforce:

- nominal base xyz;
- zero base roll/pitch and target yaw;
- all 12 leg joints at nominal;
- zero generalized velocity.

The four continuous-wheel configuration coordinates are deliberately absent
from the terminal configuration constraint, so final wheel angles are free.
Quaternion and continuous-wheel pair normalization are equality constraints at
every node. Dynamics defects use Pinocchio `integrate`/`difference`; no
Euclidean `q + dt*v` update is used.

## Objective and initialization

The path objective contains positive weights on:

```
yaw tracking                         50
base XY drift                        20
leg posture                          10
velocity                             1e-6
lateral slip                         1e-6
||tau - tau_static||^2               1e-8
||lambda - lambda_static||^2         1e-8
```

The relatively small motion/control regularizers avoid poor scaling in the
rank-deficient four-contact transcription while retaining all requested cost
terms. Lateral slip remains soft and nonzero during yaw.

The yaw seed integrates a discrete sine velocity profile whose sum is exactly
the target yaw and whose endpoint velocities are zero. At every node, the four
wheel rates are obtained by least-squares from the wheel columns of the actual
`A_hard(q)`; wheel signs are not hard-coded. The state seed is rolled out with
the same semi-implicit manifold update as the NLP. Per-interval actuator and
contact-force seeds are an inverse-dynamics least-squares correction around
`[tau_static, lambda_static]`. For all tested targets, the resulting initial
guess has equality error near `1e-14` and zero bound/friction violation.

IPOPT runs with `tol=1e-6`, `constr_viol_tol=1e-6`, and no looser acceptable
tolerance. The MUMPS dependency detector removes equality dependencies caused
by the flat four-contact geometry without changing the declared physical
constraints.

## Results

All values below were produced by the same `T=1.0 s`, `N=20` formulation.

| Target | Status / iterations | Achieved yaw | Max XY drift | Max roll/pitch | Max lateral slip | Max wheel speed |
|---|---:|---:|---:|---:|---:|---:|
| 15 deg | `Solve_Succeeded` / 29 | 0.2617993878 rad | 7.442e-8 m | 1.201e-8 rad | 0.103884 m/s | 1.070931 rad/s |
| 30 deg | `Solve_Succeeded` / 46 | 0.5235987756 rad | 1.501e-7 m | 5.941e-8 rad | 0.207770 m/s | 2.141866 rad/s |
| 45 deg | `Solve_Succeeded` / 64 | 0.7853981634 rad | 2.925e-7 m | 3.381e-10 rad | 0.311653 m/s | 3.212779 rad/s |

Detailed 15 degree validation:

| Metric | Maximum/range |
|---|---:|
| IPOPT objective | 4.934848404e-5 |
| IPOPT primal / dual infeasibility | 1.500e-7 / 9.101e-7 |
| Numerical velocity dynamics defect | 1.211e-12 |
| Numerical manifold difference defect | 1.500e-7 |
| Contact-height error | 8.784e-12 m |
| `A_hard*v` residual | 1.457e-12 |
| Friction violation | 0 |
| Quaternion norm error | 2.220e-16 |
| Continuous-wheel pair norm error | 1.221e-14 |
| Terminal boundary error | 2.247e-12 |
| Torque range | [-10.99945, 10.71032] N m |
| Normal-force range | [77.46127, 83.82390] N |
| Minimum friction-pyramid margin | 37.46851 N |

CSV outputs contain time, complete `q` and `v`, actuator torques, world-frame
contact forces, CoM, four contact positions, and per-wheel lateral slip:

- `/tmp/stand_wheel_yaw_15.csv`
- `/tmp/stand_wheel_yaw_30.csv`
- `/tmp/stand_wheel_yaw_45.csv`

## Commands

```bash
cmake -S controller/stand_wheel -B /tmp/vqr_stand_wheel_step6b \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH=$PWD/third_party/install \
  -DSTAND_WHEEL_IPOPT_PLUGIN_DIR=$PWD/third_party/casadi/install/lib
cmake --build /tmp/vqr_stand_wheel_step6b -j2
ctest --test-dir /tmp/vqr_stand_wheel_step6b --output-on-failure

/tmp/vqr_stand_wheel_step6b/stand_wheel_yaw_opt \
  vqr_description/vqr_urdf/urdf/VQRWheel.urdf \
  /tmp/stand_wheel_yaw_15.csv 15
```

## Assumptions and remaining issues

- This is a skid-steer contact model: yaw requires lateral slip. The 15 degree
  solution reaches approximately 0.104 m/s peak lateral slip; it is penalized,
  not prohibited.
- Four contacts remain active and unilateral/friction-pyramid feasible for the
  complete horizon. No complementarity or contact switching is modeled.
- The installed Pinocchio model exposes URDF velocity/effort magnitudes in both
  lower and upper arrays. The optimizer treats non-signed positive pairs as
  symmetric `[-limit, +limit]`; genuinely signed lower/upper limits are kept.
- IPOPT 3.11.9 with MUMPS reports linear equality dependencies for this flat,
  redundant contact transcription. Dependency detection is therefore enabled.
- The larger targets are validation extensions only; no posture, unload, lift,
  or subsequent-step behavior was added.
