# Wheel rolling/nonholonomic contact audit — Step 3

## Scope

This audit covers the numerical Pinocchio wheel-contact model for `FL`, `FR`,
`HL`, and `HR`. It reuses the Step 1 contact geometry and keeps the Step 2 force
Jacobian separate from the rolling velocity constraint. It does not introduce
CasADi, IPOPT, trajectory optimization, friction optimization, controller
integration, or hardware integration.

## Runtime wheel-contact frame

All vectors below are expressed on world-aligned axes and are recomputed from
the current Pinocchio configuration.

For unit positive wheel-joint axis `a` and unit ground normal `e_z`:

```text
e_roll = normalize(a cross e_z)
e_lat  = normalize(e_z cross e_roll)
R_WC   = [e_roll, e_lat, e_z]
```

`R_WC` is a right-handed contact frame whose columns are rolling, lateral, and
normal directions. No world X or Y direction is encoded as rolling/lateral.
`e_lat` is the ground-plane projection of the axle direction; it equals the
axle when the axle is parallel to the ground and differs when the wheel has
camber.

The physical centerline tread contact remains the Step 1 construction:

```text
n_radial = normalize(e_z - (a dot e_z) a)
p_contact = p_wheel_center - radius * n_radial
```

This keeps the contact offset perpendicular to the axle and on the cylindrical
tread for a tilted wheel. The audited wheel radius is `0.091 m`.

## Force Jacobian versus rolling constraint

The two APIs intentionally represent different concepts.

### Force Jacobian

`J_force` is the Step 1 world-aligned translational Jacobian of the current
wheel material point. Step 2 generalized contact force remains

```text
tau_contact = J_force.transpose() * f_world
```

If force components are supplied in contact coordinates,

```text
f_contact = [f_roll, f_lat, f_normal]
f_world   = R_WC * f_contact
```

The existing `FullDynamics` API accepts world-axis blocks `[fx, fy, fz]`.

### Rolling velocity constraint

`J_carrier` starts from `J_force` but sets the named wheel's own spin column to
zero. This describes the instantaneous carrier/contact-point motion caused by
the free flyer and leg joints without counting wheel spin twice.

For wheel velocity selector `e_w`, the implemented three-row block is

```text
A_i.row(0) = e_z.transpose()    * J_carrier
A_i.row(1) = e_lat.transpose()  * J_carrier
A_i.row(2) = e_roll.transpose() * J_carrier - radius * e_w.transpose()
```

Thus

```text
c_i(q,v) = A_i(q) * v
         = [v_normal,
            v_lateral,
            v_rolling - radius*qdot_wheel]
```

and active contacts are stacked in caller-provided order. Duplicate contacts
are rejected. This is not implemented by passing the Step 2 `J_force` directly
as a sticking constraint, although the resulting instantaneous material-point
velocity is numerically equivalent when the wheel geometry convention holds.

At every evaluation, the implementation verifies the positive wheel-joint
motion contribution in `J_force`:

```text
J_force.col(wheel_idx_v) = -radius * e_roll
```

This check protects the residual sign against a changed URDF axis or frame
convention. Wheel velocity indices are obtained from the Step 1 name mapping.

## Nominal directions and contact points

At the nominal standing configuration all four wheels have the same directions:

```text
axle    = ( 0, -1,  0)
e_roll  = (-1,  0,  0)
e_lat   = ( 0, -1,  0)
e_z     = ( 0,  0,  1)
```

| Wheel | Contact point `(x, y, z)` m |
|---|---|
| FL | `(0.2447023, 0.2365000, -8.33e-17)` |
| FR | `(0.2447023, -0.2365000, -8.33e-17)` |
| HL | `(-0.2520977, 0.2365000, -8.33e-17)` |
| HR | `(-0.2520977, -0.2365000, -8.33e-17)` |

Validation at this pose reported zero unit-length, orthogonality, handedness,
rolling-versus-axle, rolling-versus-normal, and wheel-spin convention errors at
printed precision.

The signs imply that a positive wheel speed produces bottom material-point
velocity opposite `e_roll`; no slip therefore requires carrier velocity
`+radius*qdot_wheel` along `e_roll`.

## Numerical validation

### Pure rolling

For `FL`, with wheel speed `3.25 rad/s`, the constructed carrier rolling speed
was

```text
0.091 * 3.25 = 0.29575 m/s
```

The residual `[normal, lateral, rolling]` was exactly `(0, 0, 0)` at printed
precision.

### Wrong rolling speed

Keeping carrier velocity fixed and increasing wheel speed by `0.7 rad/s`
produced

```text
rolling residual = -0.091 * 0.7 = -0.0637 m/s
```

The measured sign and magnitude matched exactly at printed precision.

### Lateral slip and ground penetration

- Prescribed lateral velocity `+0.19 m/s` produced residual
  `(0, 0.19, 0)`.
- Prescribed normal velocity `-0.13 m/s` produced residual
  `(-0.13, 0, 0)`.
- With all four contacts active and `v=0`, the stacked residual was exactly
  zero.

### Finite difference

At a perturbed floating-base and joint configuration, every column of every
four-wheel `A_roll` block was compared with the projected central-difference
velocity of the same fixed material point. Configuration perturbations used
`pinocchio::integrate()` with `epsilon=1e-6`.

- Maximum error: `7.740589e-11`.
- Worst case: `FR`, generalized velocity DoF `10`.
- Acceptance threshold: `1e-7`.

## Build and test

From the repository root:

```bash
cmake -S controller/stand_wheel \
  -B /tmp/vqr_stand_wheel_step3 \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH="$PWD/third_party/install"
cmake --build /tmp/vqr_stand_wheel_step3 -j2
ctest --test-dir /tmp/vqr_stand_wheel_step3 --output-on-failure
```

The test executable can also be run directly:

```bash
/tmp/vqr_stand_wheel_step3/wheel_contact_test \
  vqr_description/vqr_urdf/urdf/VQRWheel.urdf
```

## Assumptions and unresolved issues

- Ground is locally planar with the configured world normal, currently `+Z`.
- Contact is an ideal centerline point on a rigid cylinder. Tire deformation,
  finite contact patch, wheel width, and terrain curvature are not modeled.
- The model imposes no friction coefficient or force feasibility bounds; those
  are outside this kinematic Step 3.
- `A_roll` is an instantaneous nonholonomic velocity constraint. It is not a
  globally integrable contact-position constraint.
- The selected material point is frozen for the finite-difference validation.
  Runtime geometry reselects the physical lowest tread point at each state.
- Axle directions nearly parallel to the ground normal are rejected because a
  rolling direction is then undefined.

