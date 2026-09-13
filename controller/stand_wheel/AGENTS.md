# AGENTS.md

## Scope

This file applies to all work under:

`controller/stand_wheel/`

This module is dedicated to the standing / transition controller for the wheeled quadruped.

Current development direction:

`4-wheel stance -> rotate in place -> posture preparation -> settle -> unload diagonal legs -> lift FR+HL -> retain FL+HR support`

The current research focus is an offline Trajectory Optimization pipeline using Pinocchio-based rigid-body modeling.

---

# Current Priority

The immediate goal is NOT to implement the complete trajectory optimizer.

Work must proceed incrementally.

Current milestone:

> Build and validate the Pinocchio floating-base robot model from the existing URDF.

Do not start CasADi, IPOPT, trajectory optimization, MPC, RL, or hardware integration until the Pinocchio model validation milestone is complete.

---

# Development Rules

## 1. Work incrementally

Implement only the requested milestone.

Do not anticipate future phases unless required by the current task.

For each milestone:

1. Audit existing code first.
2. Reuse existing modules whenever possible.
3. Make the minimum required changes.
4. Add tests.
5. Run tests.
6. Report results.
7. Stop.

Do not continue automatically to the next milestone.

---

## 2. Do not refactor unrelated code

Do not modify unrelated modules such as:

* MPC
* RL
* hardware SDK
* state estimator
* existing locomotion controller

unless explicitly required.

Avoid large architectural refactors.

Prefer isolated additions inside:

`controller/stand_wheel/`

---

# Robot Convention

The robot is a wheeled quadruped with four legs:

* FL: Front Left
* FR: Front Right
* HL: Hind Left
* HR: Hind Right

The target diagonal support pair is:

`FL + HR`

The legs intended to unload/lift later are:

`FR + HL`

Never assume leg ordering from array indices.

Always determine and document the mapping from the URDF/model.

---

# Pinocchio Model

Use Pinocchio as the main rigid-body model.

Load the robot using:

`pinocchio::JointModelFreeFlyer`

Do not use a custom SRBD model for this module unless explicitly requested.

The full model must include:

* floating base
* leg joints
* wheel joints
* inertial parameters
* wheel frames
* contact geometry

---

# Step 1 — Pinocchio Model Validation

The current implementation task is limited to this section.

## Required audit

Inspect the existing URDF/model and determine:

* URDF path
* all joint names
* actuated leg joints
* wheel joints
* wheel links
* wheel frames
* joint limits
* wheel radius
* wheel rotation axis
* FL / FR / HL / HR mapping
* model root/base frame

Do not hard-code these values before inspecting the model.

---

## Required Pinocchio quantities

Build the floating-base model and validate:

* `model.nq`
* `model.nv`
* joint IDs
* frame IDs
* neutral configuration
* total mass
* CoM
* wheel center positions
* mass matrix
* nonlinear effects
* frame Jacobians

All values must be finite.

---

# Free-Flyer Convention

Explicitly document Pinocchio's configuration and velocity layout.

Do not assume quaternion ordering.

Verify it from the Pinocchio API/version used in this repository.

Document at minimum:

* base translation indices
* quaternion indices/order
* base linear velocity convention
* base angular velocity convention
* joint configuration ordering
* joint velocity ordering

Do not perform quaternion integration using direct vector addition.

Use Pinocchio manifold operations when integration is required later.

---

# Contact Frames

The physical wheel-ground contact point is NOT automatically equal to the wheel frame origin.

For each wheel define or compute:

* `FL_contact`
* `FR_contact`
* `HL_contact`
* `HR_contact`

The contact point must represent the wheel-ground contact location.

Do not blindly use:

`wheel_center - radius * world_z`

unless the wheel orientation and flat-ground assumption make this geometrically valid.

Prefer computing the contact point from:

* wheel center
* wheel orientation
* wheel radius
* ground normal

Document the assumption used.

---

# Wheel Geometry

For every wheel identify:

* center position
* axle direction
* rolling direction
* lateral direction
* radius

The local wheel contact frame should eventually support a convention similar to:

* x: rolling direction
* y: lateral direction
* z: contact normal

Do not implement wheel contact dynamics yet unless explicitly requested.

For Step 1, only validate geometry and frame conventions.

---

# Nominal Standing Configuration

Use the existing nominal standing configuration if one already exists.

Do not invent a new standing posture unless necessary.

At the nominal state validate:

* robot orientation is sensible
* base height is sensible
* wheel positions are sensible
* four wheels are located near the expected ground plane
* CoM is physically plausible
* joint values are within limits

Print all four contact positions.

---

# Dynamics Validation

At the nominal configuration compute:

`M(q)`

and verify:

* correct dimensions
* finite values
* symmetry
* positive definiteness within numerical tolerance

Also compute Pinocchio nonlinear effects:

`h(q, v)`

or equivalent API result.

Validate:

* finite values
* correct dimension

At zero velocity, gravity terms must be physically plausible.

---

# Jacobian Validation

Compute frame/contact Jacobians for all four wheel/contact frames.

Validate:

* correct dimensions
* finite values
* no NaN
* no Inf

Document the Pinocchio Jacobian reference frame used.

Do not mix:

* LOCAL
* WORLD
* LOCAL_WORLD_ALIGNED

without explicitly documenting the choice.

---

# Tests

Create an isolated smoke test, preferably:

`controller/stand_wheel/tests/pinocchio_model_smoke_test.cpp`

The test must validate at least:

* URDF/model loads
* free-flyer model exists
* expected joints are found
* four wheels are found
* leg mapping is unambiguous
* `nq > 0`
* `nv > 0`
* total mass > 0
* CoM finite
* contact positions finite
* mass matrix finite
* mass matrix symmetric
* mass matrix positive definite
* Jacobians finite
* joint limits valid
* no NaN/Inf

Tests must fail loudly with meaningful messages.

Do not silently substitute missing frames or joints.

---

# Documentation

Create/update:

`controller/stand_wheel/docs/PINOCCHIO_MODEL_AUDIT.md`

It should contain:

## Model

* URDF used
* Pinocchio version
* `nq`
* `nv`
* total mass

## Joint Mapping

A table with:

* model index
* joint name
* leg
* type
* position index
* velocity index

## Wheel Mapping

For FL, FR, HL, HR:

* wheel joint
* wheel link/frame
* contact frame
* wheel radius
* axle axis
* rolling direction

## Nominal State

Report:

* base pose
* CoM
* four wheel center positions
* four contact positions

## Validation

Report PASS/FAIL for:

* model load
* joint mapping
* wheel mapping
* mass matrix
* CoM
* contact geometry
* Jacobians
* NaN/Inf checks

## Remaining assumptions/issues

List unresolved model assumptions explicitly.

---

# Build Integration

Integrate with the existing project build system with minimal changes.

Do not create a new standalone build system if the repository already has one.

Avoid adding unnecessary dependencies.

For Step 1, Pinocchio should be the only new mathematical dependency required by this module.

---

# Future Architecture

Do not implement these phases until explicitly requested.

The intended future sequence is:

## Step 2

Build and validate the full dynamics/contact API:

`M(q) vdot + h(q,v) = S^T tau + Jc(q)^T lambda`

Validate contact Jacobians and generalized contact forces numerically.

## Step 3

Add CasADi-compatible Pinocchio dynamics.

Validate symbolic dynamics against numeric Pinocchio.

## Step 4

Implement a very small direct multiple shooting problem without contact switching.

## Step 5

Implement four-wheel rotation trajectory optimization.

## Step 6

Add posture preparation and CoM-to-FL-HR-support-line objective.

## Step 7

Add settle phase.

## Step 8

Add FR+HL unloading.

## Step 9

Add FR+HL lift and two-contact terminal state.

Do not skip these validation stages.

---

# Planned Trajectory Optimization

Future optimization objective:

`4-wheel stance -> rotate -> desired posture -> settle -> unload FR+HL -> lift FR+HL`

The planned fixed contact schedule is:

### ROTATE

Active:

`FL FR HL HR`

### POSTURE

Active:

`FL FR HL HR`

### SETTLE

Active:

`FL FR HL HR`

### UNLOAD

Geometric contact:

`FL FR HL HR`

but target:

`Fz_FR -> 0`

`Fz_HL -> 0`

### LIFT

Active:

`FL HR`

Swing:

`FR HL`

Do not implement contact-implicit or complementarity optimization for the first baseline.

---

# Support Line

The final two-wheel support line is defined by FL and HR contact positions:

`FL_contact -> HR_contact`

Later optimization will minimize the horizontal CoM distance to this line before unloading FR and HL.

Do not approximate this using body roll/pitch alone.

Keep support-line geometry explicit.

---

# Code Quality

Prefer:

* simple implementations
* explicit frame naming
* small functions
* strong validation
* assertions for model assumptions
* readable logging

Avoid:

* hidden coordinate conversions
* unexplained magic indices
* duplicated kinematics
* duplicated URDF parsing
* premature abstraction
* large template frameworks

No joint/frame should be identified only by a numeric index without first resolving it from its name.

---

# Logging

For Step 1, print a compact model audit similar to:

```text
[stand_wheel][pinocchio]

URDF: ...
nq: ...
nv: ...
mass: ...

Base:
  ...

CoM:
  x ...
  y ...
  z ...

FL:
  wheel_joint: ...
  wheel_frame: ...
  contact: [x y z]

FR:
  ...

HL:
  ...

HR:
  ...

Mass matrix: PASS
Joint mapping: PASS
Wheel mapping: PASS
Contact geometry: PASS
Jacobians: PASS
Finite check: PASS
```

---

# Completion Criteria for Step 1

Step 1 is complete only when:

1. The Pinocchio floating-base model loads successfully.
2. Joint ordering is documented.
3. FL/FR/HL/HR wheel mapping is known.
4. Contact geometry is explicitly defined.
5. CoM is validated.
6. `M(q)` passes numerical checks.
7. All four contact Jacobians are valid.
8. Smoke test passes.
9. `PINOCCHIO_MODEL_AUDIT.md` is written.

When these criteria are met:

STOP.

Report:

* files created
* files modified
* build command
* test command
* test results
* unresolved issues

Do not start Step 2 without a new instruction.
