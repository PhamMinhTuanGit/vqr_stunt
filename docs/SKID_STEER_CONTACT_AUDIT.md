# Skid-steer wheel contact audit — Step 5B

## Scope

This step implements and validates the numerical and CasADi symbolic
skid-steer contact APIs intended for a later trajectory optimizer. It reuses
the Step 1–4 wheel geometry, name mappings, material-point force Jacobian, and
runtime wheel directions. It does not create a cost, NLP/`Opti` problem,
IPOPT solver, trajectory/posture optimization, unload/lift logic, controller,
or hardware integration.

Active contacts are always stacked in caller-provided order. The test includes
the noncanonical subset `HR, FL, HL`.

## Three independent contact concepts

The implementation deliberately keeps these APIs separate:

1. `J_force(q)` maps world-expressed contact forces to generalized force as
   `tau_contact = J_force.transpose()*lambda_world`.
2. `A_hard(q)` expresses only nonpenetration and rolling kinematics.
3. `s_lateral(q,v)` measures lateral skid for a future soft objective and is
   not an equality constraint.

`A_hard.transpose()` is never used as a contact-force mapping. The Step 2
`FullDynamics::generalizedContactForce()` path remains unchanged and the new
test verifies it still evaluates `J_force.transpose()*lambda_world` exactly.

## Hard skid-steer constraints

For each active wheel, let `J_carrier` be the Step 3 contact-point Jacobian
with that wheel's own spin column removed. Let `e_wheel` select its
name-resolved generalized wheel speed. The two hard rows are

```text
A_hard_i[0,:] = e_normal.transpose() * J_carrier
A_hard_i[1,:] = e_roll.transpose()   * J_carrier
                - radius * e_wheel.transpose()
```

Therefore

```text
A_hard_i(q) v = [v_normal,
                 v_rolling - radius*qdot_wheel]
```

and `A_hard` has shape `(2*Nc) x nv`. The per-wheel row order is always
`[normal, rolling]`. Lateral velocity is not included in this matrix.

The numerical API exposes the matrix, its residual, and a combined
`SkidSteerConstraintData` snapshot. The CasADi API exposes both
`hard_constraint_matrix(q)` and `hard_constraint_residual(q,v)`.

## Soft lateral-slip residual

The signed lateral skid for each active wheel is

```text
s_i(q,v) = e_lat.transpose() * J_carrier(q) * v
```

and the caller-ordered vector has length `Nc`. It is only a reported residual.
A future objective may use

```text
J_slip = w_slip * s_lateral.transpose() * s_lateral
```

but neither the weight nor the cost is implemented here.

## Contact-force coordinates

The runtime contact frame is unchanged from Step 3:

```text
R_WC = [e_roll, e_lat, e_normal]
lambda_wheel = [f_roll, f_lat, f_normal]
lambda_world = R_WC * lambda_wheel
lambda_wheel = R_WC.transpose() * lambda_world
```

All directions are expressed in world axes and recomputed from current wheel
orientation. For multiple active contacts, the transform is block diagonal
and follows caller order.

At a random valid configuration for subset `HR, FL, HL`:

- transform orthogonality error: `2.220446049e-16`;
- wheel-to-world-to-wheel round-trip error: `1.776356839e-15`;
- `J_force.transpose()*lambda_world` consistency error: `0`.

## Anisotropic friction pyramid

`FrictionConfig(mu_roll, mu_lat)` supplies both coefficients; neither is
hard-coded in the physics code. Coefficients must be finite and nonnegative.

For each wheel-coordinate force `[f_roll, f_lat, f_normal]`, the numerical and
symbolic APIs return five margins that must all be nonnegative:

```text
m_0 = f_normal
m_1 = mu_roll*f_normal - f_roll
m_2 = mu_roll*f_normal + f_roll
m_3 = mu_lat *f_normal - f_lat
m_4 = mu_lat *f_normal + f_lat
```

These are equivalent to

```text
f_normal >= 0
abs(f_roll) <= mu_roll*f_normal
abs(f_lat)  <= mu_lat *f_normal.
```

The numerical result additionally returns `violations=max(-margins,0)` and a
tolerance-aware boolean. CasADi returns the linear margin expressions directly
for later inequality constraints.

With test coefficients `mu_roll=0.8`, `mu_lat=0.5`:

| Test force `[roll,lat,normal]` | Margins | Result |
|---|---|---|
| `[4,2,10]` | `[10,4,12,3,7]` | feasible |
| `[0,0,-1]` | `[-1,-0.8,-0.8,-0.5,-0.5]` | negative-normal violation |
| `[9,0,10]` | `[10,-1,17,5,5]` | rolling violation |
| `[0,6,10]` | `[10,8,8,-1,11]` | lateral violation |

## Kinematic validation

### Fixed-leg pure yaw skid

At the nominal state with zero base translation, fixed legs, yaw rate
`0.5 rad/s`, and least-squares wheel rates

```text
[FL, FR, HL, HR] =
[+1.299450549, -1.299450549, +1.299450549, -1.299450549] rad/s
```

the result was

```text
norm(A_hard*v) = 0
s_lateral =
[-0.1223511656, -0.1223511656,
 +0.1260488344, +0.1260488344] m/s
```

The slip differs from the recorded Step 5A values by at most
`3.857572595e-11` due only to their printed decimal precision, and agrees
exactly with the full Step 3 lateral rows at runtime.

### Straight rolling

For carrier speed `0.35 m/s` along the runtime rolling direction and matching
wheel speed `0.35/0.091 rad/s`:

```text
norm(A_hard*v) = 0
norm(s_lateral) = 0
```

### Lateral skid

For carrier velocity `+0.19 m/s` along the runtime lateral direction and zero
wheel spin:

```text
norm(A_hard*v) = 0
s_lateral = [0.19, 0.19, 0.19, 0.19] m/s
```

This demonstrates that lateral skid is observable without making the hard
kinematics infeasible.

## CasADi parity

At a random valid state with active order `HR, FL, HL`, the new symbolic APIs
were compared with the numerical implementation:

| Quantity | Maximum absolute error |
|---|---:|
| `A_hard(q)` | `2.775557562e-17` |
| `A_hard(q)*v` | `1.110223025e-16` |
| `s_lateral(q,v)` | `0` |
| wheel-to-world force transform | `0` |
| world-to-wheel force transform | `1.776356839e-15` |
| friction margins | `0` |

Acceptance tolerance was `1e-12` for every new symbolic/numerical parity
check.

## Build and test

From the repository root:

```bash
cmake -S controller/stand_wheel \
  -B /tmp/vqr_stand_wheel_step5b \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH="$PWD/third_party/install"
cmake --build /tmp/vqr_stand_wheel_step5b -j1
ctest --test-dir /tmp/vqr_stand_wheel_step5b --output-on-failure
```

Final regression result: `6/6` tests passed (`pinocchio_model_smoke`,
`full_dynamics`, `wheel_contact`, `casadi_model`, `yaw_feasibility`, and
`skid_steer_contact`).

For detailed Step 5B output:

```bash
/tmp/vqr_stand_wheel_step5b/skid_steer_contact_test \
  vqr_description/vqr_urdf/urdf/VQRWheel.urdf
```

## Assumptions and unresolved issues

- The model retains the rigid centerline tread point, planar ground, and
  runtime axle/ground-normal assumptions from Steps 1–4.
- Lateral slip is an instantaneous kinematic velocity residual. No tire
  deformation, slip-angle dynamics, lateral-force-versus-slip law, or energy
  dissipation model is introduced.
- The pyramid is an anisotropic box approximation in tangential-force space,
  not an elliptical or circular friction cone.
- Friction coefficients are configuration inputs but are constant across the
  active contacts within one symbolic function evaluation. Per-wheel
  coefficients can be added later if terrain data requires them.
- The friction check evaluates supplied forces only. It does not solve for
  forces, enforce equilibrium, or guarantee that dynamics and all contact
  constraints are jointly feasible.
- Active-contact changes still require selecting/building the corresponding
  caller-ordered numerical or CasADi function set.
