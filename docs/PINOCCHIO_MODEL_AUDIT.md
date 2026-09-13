# Pinocchio model audit — VQRWheel Step 1

## Scope

This document records only the Step 1 model work for
`controller/stand_wheel`: audit the existing URDF, build a floating-base
Pinocchio model, resolve joints and wheel contacts by name, and validate the
model numerically. It does not add CasADi, IPOPT, trajectory optimization, MPC,
or a controller.

The audited source model is
`vqr_description/vqr_urdf/urdf/VQRWheel.urdf`. The existing
`controller/stand_4_wheel` Pinocchio conventions and finite-difference test were
used as the starting point. `controller/stand_wheel/AGENTS.md` was empty (zero
bytes) during this audit, so it supplied no additional Step 1 constraints.

## URDF audit

- Root body: `TORSO`. The URDF has no active world/base joint; Pinocchio adds a
  `JointModelFreeFlyer` named `root_joint`.
- Bodies: torso plus four copies of hip, thigh, shank, and wheel.
- Actuation: 12 bounded revolute leg joints and four continuous wheel joints.
- Wheel collision geometry: cylinder radius `0.091 m`, length `0.04 m`, with
  its cylinder axis aligned to local `-Y` by the collision rotation.
- Wheel joint axes: all four are local `(0, -1, 0)`.
- No dedicated contact frames exist. Each contact is derived from its wheel
  BODY frame, which is located at the axle center.
- Link inertial masses sum to `32.86 kg`: torso `17.46 kg`; each leg assembly
  contributes `1.0 + 1.5 + 0.15 + 1.2 = 3.85 kg`.
- The raw wheel declarations near the end of the URDF are not a safe canonical
  ordering. All controller-facing mappings are therefore resolved by name into
  `FL, FR, HL, HR` order.

## Floating-base model

Pinocchio reports:

- `nq = 27`
- `nv = 22`
- total mass `= 32.86 kg`

The dimension split is:

- free flyer: `nq=7`, `nv=6`;
- 12 bounded leg revolutes: `nq=12`, `nv=12`;
- four continuous wheel joints: `nq=8`, `nv=4` because Pinocchio represents
  each continuous angle with a normalized `(cos, sin)` pair.

The implementation starts configurations with `pinocchio::neutral()`, so every
continuous wheel receives the valid neutral pair `(1, 0)`. Joint/frame IDs and
their `idx_q`/`idx_v` values are resolved by name and checked when the model is
constructed.

## Pinocchio joint ordering

The ranges below are written as `[start, size]`.

| Joint ID | Joint name | q range | v range |
|---:|---|---:|---:|
| 1 | `root_joint` | `[0, 7]` | `[0, 6]` |
| 2 | `FL_HipX_joint` | `[7, 1]` | `[6, 1]` |
| 3 | `FL_HipY_joint` | `[8, 1]` | `[7, 1]` |
| 4 | `FL_Knee_joint` | `[9, 1]` | `[8, 1]` |
| 5 | `FL_WHEEL` | `[10, 2]` | `[9, 1]` |
| 6 | `FR_HipX_joint` | `[12, 1]` | `[10, 1]` |
| 7 | `FR_HipY_joint` | `[13, 1]` | `[11, 1]` |
| 8 | `FR_Knee_joint` | `[14, 1]` | `[12, 1]` |
| 9 | `FR_WHEEL` | `[15, 2]` | `[13, 1]` |
| 10 | `HL_HipX_joint` | `[17, 1]` | `[14, 1]` |
| 11 | `HL_HipY_joint` | `[18, 1]` | `[15, 1]` |
| 12 | `HL_Knee_joint` | `[19, 1]` | `[16, 1]` |
| 13 | `HL_WHEEL` | `[20, 2]` | `[17, 1]` |
| 14 | `HR_HipX_joint` | `[22, 1]` | `[18, 1]` |
| 15 | `HR_HipY_joint` | `[23, 1]` | `[19, 1]` |
| 16 | `HR_Knee_joint` | `[24, 1]` | `[20, 1]` |
| 17 | `HR_WHEEL` | `[25, 2]` | `[21, 1]` |

These indices are audit output, not constants used to resolve actuators.

## Canonical wheel/contact mapping

| Slot | Wheel joint | Joint ID | `idx_v` | Wheel BODY frame | Frame ID |
|---|---|---:|---:|---|---:|
| FL | `FL_WHEEL` | 5 | 9 | `FL_WHEEL` | 10 |
| FR | `FR_WHEEL` | 9 | 13 | `FR_WHEEL` | 18 |
| HL | `HL_WHEEL` | 13 | 17 | `HL_WHEEL` | 26 |
| HR | `HR_WHEEL` | 17 | 21 | `HR_WHEEL` | 34 |

Construction also checks that each wheel BODY frame is attached to the wheel
joint with the same name and that every wheel has `nq=2`, `nv=1`.

## Contact geometry convention

For wheel center `p_w`, unit wheel axis `a`, world ground normal `n`, and wheel
radius `r`, the centerline tread contact is

```text
n_radial = normalize(n - (a dot n) a)
d        = -r n_radial
p_c      = p_w + d
```

Projecting the ground normal into the wheel radial plane keeps the point on the
cylindrical tread if the wheel is tilted. At a configuration snapshot the
contact Jacobian is the Jacobian of that material point:

```text
J_c = J_linear - skew(d) J_angular
```

Frame Jacobians use `LOCAL_WORLD_ALIGNED`; the Jacobian has shape `3 x nv`.

## Nominal audit stance

The audit stance is explicitly a test fixture, not a control target:

- base orientation: identity;
- all `HipX = 0`;
- all `HipY = -0.82 rad`;
- all `Knee = 1.61 rad`;
- continuous wheel angles: neutral `(cos, sin) = (1, 0)`;
- base height solved from forward kinematics so the mean contact plane is
  `z=0`; symmetry makes all four contacts land on that plane.

Result:

- base position: `(0, 0, 0.411049948) m`;
- CoM: `(-0.013012891, 0, 0.342438596) m`.

| Contact | Wheel center `(x, y, z)` m | Contact `(x, y, z)` m |
|---|---|---|
| FL | `(0.244702331, 0.236500000, 0.091000000)` | `(0.244702331, 0.236500000, 0)` |
| FR | `(0.244702331, -0.236500000, 0.091000000)` | `(0.244702331, -0.236500000, 0)` |
| HL | `(-0.252097669, 0.236500000, 0.091000000)` | `(-0.252097669, 0.236500000, 0)` |
| HR | `(-0.252097669, -0.236500000, 0.091000000)` | `(-0.252097669, -0.236500000, 0)` |

Every nominal wheel axis is `(0, -1, 0)`. Geometry checks confirm contact
offset norm `0.091 m`, offset perpendicular to wheel axis, wheel center height
equal to the radius, correct FL/FR/HL/HR quadrants, and agreement between the
configured wheel axis and the actual URDF wheel-joint motion subspace.

## Numerical validation

All values below were produced by the compiled smoke test.

### Center of mass

Pinocchio `centerOfMass()` was compared with an independent mass-weighted sum
of each body's inertial lever transformed by `data.oMi`. Error was below
`1e-12 m` at the nominal stance.

### Mass matrix

At the nominal stance:

- CRBA symmetry error: `0.000000e+00`;
- smallest eigenvalue: `4.386415e-03`;
- largest eigenvalue: `3.306208e+01`;
- maximum CRBA-column versus RNEA unit-acceleration error: `5.684342e-14`.

Thus the `22 x 22` mass matrix is finite, symmetric, positive definite, and
consistent with inverse dynamics.

### Contact Jacobians

A non-symmetric test perturbation exercises the free flyer, every leg joint,
and every wheel joint. Central differences use `pinocchio::integrate()` and
track the same local material point at `epsilon=1e-6`.

| Contact | Maximum column error | Worst velocity DoF | Row rank |
|---|---:|---:|---:|
| FL | `5.682489e-11` | 5 | 3 |
| FR | `4.948444e-11` | 10 | 3 |
| HL | `4.452909e-11` | 3 | 3 |
| HR | `6.044011e-11` | 4 | 3 |

Acceptance threshold was `1e-6`; all four Jacobians passed by more than four
orders of magnitude.

## Reproduce

From the repository root:

```bash
cmake -S controller/stand_wheel \
  -B /tmp/vqr_stand_wheel_step1 \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH="$PWD/third_party/install"
cmake --build /tmp/vqr_stand_wheel_step1 -j2
ctest --test-dir /tmp/vqr_stand_wheel_step1 --output-on-failure
```

To print the complete audit values:

```bash
/tmp/vqr_stand_wheel_step1/pinocchio_model_smoke_test \
  vqr_description/vqr_urdf/urdf/VQRWheel.urdf
```

## Remaining assumptions and issues

- Flat ground is assumed with world normal `+Z`.
- Contact is an ideal centerline point on the cylindrical tread. Wheel width,
  tire deformation, compliance, and finite contact patch are not modeled.
- Wheel radius and local axis are configuration inputs copied from the audited
  URDF. The smoke test checks their expected values and verifies the configured
  axes against Pinocchio's wheel-joint motion axes, but the runtime library does
  not parse collision geometry.
- The nominal angles are only a deterministic audit fixture. A future control
  layer must supply its own state/target and must preserve Pinocchio's
  free-flyer and continuous-joint conventions.
- The URDF contains no explicit contact frames; changing wheel link/frame names
  or collision geometry intentionally causes construction/tests to fail until
  the name mapping or geometry configuration is reviewed.

