# Four-wheel strict-rolling yaw feasibility — Step 5A

## Scope

This audit asks only whether the nominal four-wheel stance can rotate in place
under the strict Step 3 velocity constraints. It reuses the numerical
`WheelContactModel` and its name-resolved Pinocchio mappings. It does not add
IPOPT, an NLP, trajectory/posture optimization, unload/lift behavior, MPC, or
controller integration.

The tested target is a base yaw rate of `0.5 rad/s` at the nominal standing
configuration. All four contacts are active in canonical `FL, FR, HL, HR`
order. SVD least squares uses a rank threshold of `1e-10`.

## Constraint convention

Each wheel contributes the existing Step 3 block

```text
A_i(q) v = [v_normal,
            v_lateral,
            v_rolling - radius*qdot_wheel].
```

Directions are recomputed from the current wheel orientation. At the nominal
pose the axles and lateral directions are parallel, so a wheel's own spin can
change only its rolling row. It cannot correct its lateral row.

The nominal Step 3 lateral direction is `e_lat = -world_Y`. Therefore its
signed lateral residual is

```text
c_lat_i = -(v_y + yaw_rate*x_i)
```

for fixed legs at the identity base orientation. The sign differs from the
world-Y velocity only because of `e_lat`; the zero constraint is identical.
This relation was extracted from the actual `A_roll` columns rather than
assumed:

| Wheel | Contact `x` (m) | world-Y `v_y` coefficient | world-Y yaw coefficient |
|---|---:|---:|---:|
| FL | `0.2447023311` | `1.0` | `0.2447023311` |
| FR | `0.2447023311` | `1.0` | `0.2447023311` |
| HL | `-0.2520976689` | `1.0` | `-0.2520976689` |
| HR | `-0.2520976689` | `1.0` | `-0.2520976689` |

Maximum coefficient error against `v_y + yaw_rate*x_i` was exactly zero at
printed precision. Since front and hind `x` coordinates differ, one `v_y`
cannot make all four lateral velocities zero for nonzero yaw.

## Case A — fixed legs and zero base translation

Fixed quantities:

- base linear velocity: zero;
- base roll/pitch rates: zero;
- every leg-joint rate: zero;
- yaw rate: `0.5 rad/s`.

Only four name-resolved wheel rates were optimized. Their minimum-norm
least-squares values were

```text
[FL, FR, HL, HR] =
[+1.299450549, -1.299450549, +1.299450549, -1.299450549] rad/s.
```

Residuals:

| Wheel | Normal (m/s) | Lateral (m/s) | Rolling (m/s) |
|---|---:|---:|---:|
| FL | `0` | `-0.1223511656` | `0` |
| FR | `0` | `-0.1223511656` | `0` |
| HL | `0` | `+0.1260488344` | `0` |
| HR | `0` | `+0.1260488344` | `0` |

Minimum achievable residual norm: `0.2484275201 m/s`. Wheel rates cancel all
rolling residuals, but cannot affect the four nonzero lateral residuals.

## Case B — allow planar base translation

Free quantities were base `vx`, base `vy`, and the four wheel rates. Legs,
vertical base velocity, and base roll/pitch rates remained fixed at zero; yaw
remained `0.5 rad/s`.

The minimum-norm least-squares result was

```text
vx = -1.655016245e-17 m/s
vy = +0.001848834439 m/s
wheel rates =
[+1.299450549, -1.299450549, +1.299450549, -1.299450549] rad/s
```

Residuals:

| Wheel | Normal (m/s) | Lateral (m/s) | Rolling (m/s) |
|---|---:|---:|---:|
| FL | `0` | `-0.1242` | `1.249001e-16` |
| FR | `0` | `-0.1242` | `2.775558e-17` |
| HL | `0` | `+0.1242` | `-5.551115e-17` |
| HR | `0` | `+0.1242` | `2.775558e-17` |

Minimum achievable residual norm: `0.2484 m/s`. Translation selects the
compromise `vy = -yaw_rate*mean(x_i)` and makes front/rear magnitudes equal,
but cannot make both signs zero. Planar translation therefore does not restore
strict feasibility.

## Case C — allow leg articulation

Base translation and base roll/pitch rates were fixed exactly to zero. The yaw
rate remained `0.5 rad/s`; all 12 leg rates and four wheel rates were free. The
SVD solution is the minimum-norm free-velocity solution.

Required leg rates in radians per second:

| Joint | Rate |
|---|---:|
| `FL_HipX_joint` | `+0.3173419323` |
| `FL_HipY_joint` | `+0.08225266659` |
| `FL_Knee_joint` | `+0.3512209649` |
| `FR_HipX_joint` | `+0.3173419323` |
| `FR_HipY_joint` | `-0.08225266659` |
| `FR_Knee_joint` | `-0.3512209649` |
| `HL_HipX_joint` | `-0.3269325675` |
| `HL_HipY_joint` | `+0.4971094308` |
| `HL_Knee_joint` | `-0.3476160110` |
| `HR_HipX_joint` | `-0.3269325675` |
| `HR_HipY_joint` | `-0.4971094308` |
| `HR_Knee_joint` | `+0.3476160110` |

Required wheel rates were

```text
FL = +0.02111721610 rad/s
FR = -0.02111721610 rad/s
HL = +0.1135912594 rad/s
HR = -0.1135912594 rad/s
```

Results:

- base twist: `[0, 0, 0, 0, 0, 0.5]`;
- total constraint residual norm: `1.739098137e-16`;
- largest individual residual magnitude: `1.058181320e-16`;
- leg-rate norm: `1.187995918 rad/s`;
- maximum absolute leg rate: `0.4971094308 rad/s`.

Thus the full articulated model can realize the requested instantaneous yaw,
but only by continuously moving the legs at rates comparable to the requested
yaw rate. This is not fixed-posture rotate-in-place and would change contact
geometry/posture over time.

## Rank and nullspace audit

The nominal four-wheel matrix has shape `12 x 22`:

- `rank(A_roll) = 12`;
- `nullity(A_roll) = 10`;
- norm of the yaw basis projection into the full nullspace: `0.3951620441`;
- an unrestricted nullspace velocity scaled to yaw `0.5 rad/s` has residual
  norm `1.521782902e-16`.

The unrestricted full model therefore has yaw-containing nullspace motions,
but those motions necessarily use other generalized coordinates.

For each fixed-yaw case, feasibility of
`A_free*x = -A_yaw*yaw_target` was also checked by comparing the free matrix
rank with the rank after appending the yaw column:

| Case | Free variables | `rank(A_free)` | Augmented rank | Result |
|---|---|---:|---:|---|
| A | four wheel rates | `4` | `5` | infeasible |
| B | `vx`, `vy`, four wheel rates | `5` | `6` | infeasible |
| C | 12 leg rates, four wheel rates | `12` | `12` | feasible |

Case B has a one-dimensional free nullspace corresponding to longitudinal
translation with coordinated wheel spin, but the yaw column adds an
independent front-versus-hind lateral mode. Case C's free matrix spans all 12
constraint rows, permitting exact cancellation through leg articulation.

## Conclusion and proposed Step 5 contact model

Strict four-wheel rolling does **not** permit nonzero-yaw rotate-in-place with
fixed legs. Allowing planar translation does not change that conclusion.
Strict instantaneous feasibility can be recovered through significant leg
articulation, but that is outside this Step 5A task and conflicts with the
intended fixed-posture yaw maneuver.

The minimal skid-steer-compatible model proposed for a later Step 5 is:

```text
hard equalities per active wheel:
  v_normal = 0
  v_rolling - radius*qdot_wheel = 0

reported soft residual per active wheel:
  s_lateral = v_lateral
```

The API should expose a `2*Nc x nv` hard-constraint matrix and a separate
`Nc x nv` lateral-slip matrix/residual. A later optimizer may penalize
`s_lateral` in its objective rather than constrain it to zero. Contact force
mapping `J_force^T lambda`, friction constraints, and force feasibility remain
separate from these velocity constraints. This audit does not implement that
API or an optimizer.

## Build and test

From the repository root:

```bash
cmake -S controller/stand_wheel \
  -B /tmp/vqr_stand_wheel_step5a \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH="$PWD/third_party/install"
cmake --build /tmp/vqr_stand_wheel_step5a -j1
ctest --test-dir /tmp/vqr_stand_wheel_step5a --output-on-failure
```

The detailed executable is:

```bash
/tmp/vqr_stand_wheel_step5a/yaw_feasibility_test \
  vqr_description/vqr_urdf/urdf/VQRWheel.urdf 0.5
```

The optional final argument is the finite, nonzero yaw-rate target in
`rad/s`; it defaults to `0.5`.
