# Full rigid-body dynamics/contact audit — Step 2

## Scope

This audit covers the numerical Pinocchio API in `controller/stand_wheel` for

```text
M(q) vdot + h(q,v) = S^T tau + Jc(q)^T lambda
```

It reuses the Step 1 floating-base model and contact points. No CasADi, IPOPT,
trajectory optimization, MPC, controller integration, or hardware integration
is included.

## State and dynamics convention

- Pinocchio model dimensions: `nq=27`, `nv=22`.
- `M(q)` is the symmetric `22 x 22` CRBA mass matrix.
- `h(q,v)` is Pinocchio `nonLinearEffects()`, including gravity and
  velocity-dependent bias forces.
- Pinocchio's free-flyer generalized velocity uses the six-dimensional local
  tangent convention. No configuration is advanced with direct vector
  addition; tests use `pinocchio::integrate()`.
- Forward dynamics uses an Eigen LDLT factorization. No explicit matrix inverse
  is formed.

## Actuation mapping

`S` has shape `16 x 22`; `S.transpose() * tau` maps the actuator vector into
generalized force. Each nonzero entry is created from the named joint's
Pinocchio `idx_v`.

| Actuator index | Joint | Generalized velocity index |
|---:|---|---:|
| 0 | `FL_HipX_joint` | 6 |
| 1 | `FL_HipY_joint` | 7 |
| 2 | `FL_Knee_joint` | 8 |
| 3 | `FR_HipX_joint` | 10 |
| 4 | `FR_HipY_joint` | 11 |
| 5 | `FR_Knee_joint` | 12 |
| 6 | `HL_HipX_joint` | 14 |
| 7 | `HL_HipY_joint` | 15 |
| 8 | `HL_Knee_joint` | 16 |
| 9 | `HR_HipX_joint` | 18 |
| 10 | `HR_HipY_joint` | 19 |
| 11 | `HR_Knee_joint` | 20 |
| 12 | `FL_WHEEL` | 9 |
| 13 | `FR_WHEEL` | 13 |
| 14 | `HL_WHEEL` | 17 |
| 15 | `HR_WHEEL` | 21 |

The table reports discovered indices; the implementation does not encode them
as constants. Construction checks that all 16 mappings are unique scalar DoFs
and that all six columns belonging to the named `root_joint` are zero in `S`.
Single-actuator injection testing produced exactly zero mapping error.

## Contact ordering and frame convention

The API accepts any duplicate-free ordered subset of `FL`, `FR`, `HL`, `HR`,
including the empty set. It preserves the caller's order. For example,
`{HR, FL, HL}` produces

```text
Jc = [J_HR; J_FL; J_HL]
lambda = [fx_HR, fy_HR, fz_HR,
          fx_FL, fy_FL, fz_FL,
          fx_HL, fy_HL, fz_HL]
```

Each `J_i` is the `3 x nv` translational Jacobian of the Step 1 wheel material
point at the centerline tread contact. `Jc` therefore has shape
`3*Nc x nv`.

`Jc` rows and `lambda` blocks are expressed in `LOCAL_WORLD_ALIGNED`
coordinates: their axes are aligned with world `x,y,z`, while each Jacobian is
taken at its contact point. `lambda` is the force applied to the robot;
positive `fz` points upward for the flat-ground convention.

Generalized contact force is

```text
tau_contact = Jc.transpose() * lambda
```

The virtual-work test used contact order `HR, FL, HL`; the difference between
`lambda^T Jc delta_v` and `(Jc^T lambda)^T delta_v` was
`3.330669e-16`.

## Forward dynamics

The implemented numerical function computes

```text
rhs  = S.transpose() * tau + Jc.transpose() * lambda - h
vdot = LDLT(M).solve(rhs)
```

At a nonzero configuration and velocity with the active diagonal contacts
`FL, HR`, nonzero actuator torques, and nonzero contact forces, the reconstructed
dynamics residual was:

- residual norm: `6.027617e-14`;
- maximum absolute residual entry: `5.684342e-14`.

## Static gravity support

At the Step 1 nominal standing configuration with `v=0`, four vertical contact
forces were found by least squares on the six unactuated floating-base
equations. Named actuator torques balance the remaining joint equations.

| Contact | Vertical force |
|---|---:|
| FL | `77.56698 N` |
| FR | `77.56698 N` |
| HL | `83.61132 N` |
| HR | `83.61132 N` |

- force sum: `322.3566 N`;
- expected `m*g`: `322.3566 N` for `m=32.86 kg`, `g=9.81 m/s^2`;
- force-sum error: `0` at printed precision;
- floating-base acceleration norm: `1.362214e-14`;
- full generalized acceleration norm: `6.373539e-14`;
- dynamics residual norm: `5.551115e-17`.

All four vertical forces are positive. The slightly larger hind forces are
consistent with the audited nominal CoM being behind the geometric center of
the contact rectangle.

## Contact acceleration and Jdot*v

For each active contact, the API returns

```text
a_contact = Jc * vdot + Jdot_v
```

`Jdot_v` is computed from Pinocchio second-order forward kinematics with zero
generalized acceleration. `getFrameClassicalAcceleration()` supplies the wheel
frame-origin bias acceleration in `LOCAL_WORLD_ALIGNED`; the implementation
then shifts it to the fixed contact material point, including angular and
centripetal terms.

Validation results:

- nominal static `||Jc*v||`: `0`;
- nominal static `||Jdot*v||`: `0`;
- supported nominal contact acceleration norm: `5.136384e-15`;
- moving-state `Jdot*v` versus central finite-difference maximum error:
  `1.849172e-11` with `epsilon=1e-6`.

The finite-difference validation holds the central configuration's local
material-point offset fixed and evaluates the derivative of `J(q)*v` using
`pinocchio::integrate()`.

## Build and test

From the repository root:

```bash
cmake -S controller/stand_wheel \
  -B /tmp/vqr_stand_wheel_step2 \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH="$PWD/third_party/install"
cmake --build /tmp/vqr_stand_wheel_step2 -j2
ctest --test-dir /tmp/vqr_stand_wheel_step2 --output-on-failure
```

Result: both `pinocchio_model_smoke` and `full_dynamics` passed (`2/2`).

## Assumptions and unresolved issues

- Contact forces are ideal point forces; no contact moment, tire compliance,
  friction cone, or force feasibility constraint is imposed by this API.
- Flat ground and world `+Z` normal remain the Step 1 geometry convention.
- `Jdot*v` follows the fixed material point selected at the evaluation state.
  A future optimization must explicitly choose between a sticking material
  point constraint and a rolling/nonholonomic wheel constraint; this Step 2 API
  does not make that optimization-level decision.
- Active-contact order is caller-controlled. Callers must use the returned
  `active_contacts` order when constructing `lambda`; duplicate contacts are
  rejected.
- The static-support test is an equilibrium validation, not a QP or a contact
  force allocator. It does not enforce actuator limits or friction bounds.

