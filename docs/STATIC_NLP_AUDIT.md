# Static NLP Trajectory Optimizer Audit — Step 6A

## 1. Scope and Objective

This audit documents **Step 6A** for `controller/stand_wheel`: a minimal Nonlinear Programming (NLP) trajectory optimization formulation solved with **CasADi + IPOPT**.

* **Primary objective:** Validate the transcription, constraint formulation, dynamics defect integration, manifold consistency, and solver pipeline for a stationary hold:
  $$\text{nominal 4-wheel stance} \longrightarrow \text{nominal 4-wheel stance}$$
* **Strict boundaries:**
  * **No yaw target or yaw optimization.**
  * **No base rotation or posture adjustment.**
  * **No leg unloading, transfer, or lifting.**
  * **No MPC or real-time controller integration.**
  * **No hardware deployment.**

---

## 2. Mathematical Formulation & Transcription

### 2.1 Horizon and Discretization
* **Horizon:** $T = 0.5\text{ s}$ (baseline), $T = 0.1\text{ s}$ (small test).
* **Intervals:** $N = 20$ (baseline), $N = 4$ (small test).
* **Timestep:** $\Delta t = T / N$ ($0.025\text{ s}$ baseline, $0.025\text{ s}$ small test).
* **Transcription:** Direct multiple shooting with fixed timestep.
* **Contact order:** Canonical fixed sequence `FL, FR, HL, HR`.

### 2.2 Decision Variables
At each trajectory node $k = 0, \dots, N$:
* Generalized configuration: $q_k \in \mathbb{R}^{27}$ (Pinocchio manifold: $\mathbb{R}^3 \times SO(3) \times \mathbb{R}^{12} \times (S^1)^4$)
* Generalized velocity: $v_k \in \mathbb{R}^{22}$

At each shooting interval $k = 0, \dots, N-1$:
* Joint actuator torques: $\tau_k \in \mathbb{R}^{16}$
* Contact forces in world frame: $\lambda_k \in \mathbb{R}^{12}$ (stacked 3D contact forces for 4 wheels)

**Total variable count:**
$$\text{Variables} = (N+1) \cdot (nq + nv) + N \cdot (nu + 3 \cdot N_c) = 21 \times 49 + 20 \times 28 = 1589$$
(For $N=4$: $5 \times 49 + 4 \times 28 = 357$).

### 2.3 Dynamics Defects
For each interval $k = 0, \dots, N-1$:
1. Forward acceleration from symbolic articulated rigid-body dynamics (Step 4 / 5):
   $$a_k = \text{forward\_dynamics}(q_k, v_k, \tau_k, \lambda_k)$$
2. Semi-implicit Euler integration:
   $$v_{\text{pred}, k} = v_k + \Delta t \cdot a_k$$
   $$q_{\text{pred}, k} = \text{integrate}(q_k, \Delta t \cdot v_{\text{pred}, k})$$
3. Defects enforced via equality constraints:
   $$v_{k+1} - v_{\text{pred}, k} = 0 \quad (\mathbb{R}^{22})$$
   $$\text{difference}(q_{\text{pred}, k}, q_{k+1}) = 0 \quad (\mathbb{R}^{22})$$
   (where $\text{difference}$ computes the tangent vector on the configuration manifold).

### 2.4 Manifold Normalization Constraints
At every node $k = 0, \dots, N$:
* Free-flyer quaternion: $\|q_{base,rot}\|^2 - 1 = 0$ (1 equality)
* Continuous wheel joints: $\cos^2(\theta_i) + \sin^2(\theta_i) - 1 = 0$ for $i=1,\dots,4$ (4 equalities)
Total: $5 \times (N+1)$ equalities ($105$ for $N=20$, $25$ for $N=4$).

### 2.5 Hard Contact and Kinematic Constraints
* Contact heights: $p_{contact, z}(q_k) = 0$ for each wheel (4 equalities)
* Hard rolling constraint: $A_{hard}(q_k) v_k = 0$ (8 equalities: 4 normal velocity $= 0$, 4 pure rolling kinematic constraints)

#### Elimination of Endpoint Redundancy
At boundary nodes $k=0$ and $k=N$, the fixed boundary conditions $q = q_{nominal}$ and $v = 0$ already strictly enforce $p_{contact, z}(q_{nominal}) = 0$ and $A_{hard}(q_{nominal}) \cdot 0 = 0$. Including them introduced 12 linearly dependent equality rows at $k=0$ and 12 at $k=N$.
* **Formulation fix:** Enforce contact height (4) and $A_{hard} v = 0$ (8) **strictly at interior nodes** $k = 1, \dots, N-1$.
* Retain manifold normalizations at all nodes.
* Equality constraint reduction: $24$ redundant equalities removed ($1325 \to 1301$ for $N=20$, $349 \to 325$ for $N=4$).

### 2.6 Boundary Conditions
* Initial state: $\text{difference}(q_{nominal}, q_0) = 0$ (22) and $v_0 = 0$ (22)
* Terminal state: $\text{difference}(q_{nominal}, q_N) = 0$ (22) and $v_N = 0$ (22)

### 2.7 Friction Cone / Pyramid Constraints
At each interval $k = 0, \dots, N-1$:
Transform contact force to wheel local frame $\lambda_{wheel, i} = R_i(q_k)^T \lambda_{world, i}$ and enforce 5 linear pyramid margins:
$$F_z \ge 0$$
$$\mu_{roll} F_z - |F_x| \ge 0 \iff \mu_{roll} F_z \pm F_x \ge 0$$
$$\mu_{lat} F_z - |F_y| \ge 0 \iff \mu_{lat} F_z \pm F_y \ge 0$$
Total inequalities: $5 \times 4 \times N = 20 \times N$ ($400$ for $N=20$, $80$ for $N=4$).

---

## 3. Objective Function and Torque Regularization

### 3.1 Mathematical Cost
$$J = \sum_{k=0}^{N-1} \left( w_q \|q_k \ominus q_{nominal}\|^2 + w_v \|v_k\|^2 + w_\tau \|\tau_k - \tau_{static}\|^2 + w_\lambda \|\lambda_k - \lambda_{static}\|^2 + w_{slip} \|v_{lat, k}\|^2 \right)$$

Weights:
* $w_q = 10.0$
* $w_v = 1.0$
* $w_\tau = 10^{-4}$
* $w_\lambda = 10^{-3}$
* $w_{slip} = 1.0$

### 3.2 Root Cause of Prior Iteration Stall & Torque Regularization Fix
* **The issue:** Previously, the torque penalty was $w_\tau \|\tau_k\|^2$. Because the robot must exert $\tau_{static} \ne 0$ to counteract gravity in stance, $w_\tau \|\tau_k\|^2$ pulled torques to zero while $w_\lambda \|\lambda_k - \lambda_{static}\|^2$ pulled forces to $\lambda_{static}$. In a 4-wheel hyperstatic contact setup (nullspace dimension 6 per interval), this created an internal dual-infeasibility tug-of-war where $\nabla_x L$ could not reach the $10^{-6}$ optimality threshold.
* **The fix:** Regularize around $\tau_{static}$ via $w_\tau \|\tau_k - \tau_{static}\|^2$.
* **Consequence:** The initial static guess $(q_{nominal}, 0, \tau_{static}, \lambda_{static})$ is now **both an exact primal feasible point and the exact unconstrained global minimum of the cost function** ($\nabla f(x_0) = 0$).

---

## 4. Verification & Diagnostics

### 4.1 Initial Guess Diagnostics (Pre-solve Evaluation)
Evaluated directly on the NLP before invoking IPOPT:

| Metric | $N=4$ ($T=0.1\text{ s}$) | $N=20$ ($T=0.5\text{ s}$) | Expected |
| :--- | :--- | :--- | :--- |
| **Initial Objective** $f(x_0)$ | $0.0$ | $0.0$ | $\approx 0$ |
| **Initial Max Equality Violation** | $1.243 \times 10^{-15}$ | $1.243 \times 10^{-15}$ | Machine precision ($\sim 10^{-15}$) |
| **Initial Max Inequality Violation** | $0.0$ | $0.0$ | $0.0$ |

### 4.2 Solver Execution & Optimality Metrics

| Metric | $N=4$ Test | $N=20$ Baseline |
| :--- | :--- | :--- |
| **IPOPT Status** | `Solve_Succeeded` | `Solve_Succeeded` |
| **Success flag** | `true` | `true` |
| **Iterations** | **5** | **5** |
| **Objective value** | $1.577 \times 10^{-16}$ | $1.073 \times 10^{-15}$ |
| **Solve time** | $\sim 0.4\text{ s}$ | $2.39\text{ s}$ |
| **Number of variables** | 357 | 1589 |
| **Equalities (before / after fix)** | 349 $\to$ **325** | 1325 $\to$ **1301** |
| **Inequalities** | 80 | 400 |
| **Primal infeasibility (`inf_pr`)** | $2.430 \times 10^{-14}$ | $2.445 \times 10^{-14}$ |
| **Dual infeasibility (`inf_du`)** | $2.450 \times 10^{-14}$ | $1.147 \times 10^{-13}$ |

### 4.3 Trajectory Physical & Manifold Validation Residuals

| Physical Residual | $N=4$ | $N=20$ | Validation Threshold | Result |
| :--- | :--- | :--- | :--- | :--- |
| **Max velocity dynamics defect** | $2.435 \times 10^{-14}$ | $2.445 \times 10^{-14}$ | $< 10^{-6}$ | **PASS** |
| **Max manifold difference defect** | $6.106 \times 10^{-16}$ | $8.882 \times 10^{-16}$ | $< 10^{-6}$ | **PASS** |
| **Max contact height error** ($z=0$) | $7.216 \times 10^{-16}$ | $7.216 \times 10^{-16}$ | $< 10^{-7}$ | **PASS** |
| **Max hard rolling residual** ($A_{hard} v = 0$) | $2.437 \times 10^{-14}$ | $2.452 \times 10^{-14}$ | $< 10^{-7}$ | **PASS** |
| **Max friction pyramid violation** | $0.0$ | $0.0$ | $\le 0.0$ | **PASS** |
| **Max quaternion norm error** | $0.0$ | $0.0$ | $< 10^{-8}$ | **PASS** |
| **Max continuous wheel norm error** | $0.0$ | $0.0$ | $< 10^{-8}$ | **PASS** |
| **Initial boundary error** | $0.0$ | $2.441 \times 10^{-14}$ | $< 10^{-7}$ | **PASS** |
| **Terminal boundary error** | $0.0$ | $2.440 \times 10^{-14}$ | $< 10^{-7}$ | **PASS** |

### 4.4 Physical Ranges Across Trajectory ($N=20$)
* **Max joint velocity:** $2.383 \times 10^{-9}\text{ rad/s}$ (static)
* **Max joint torque:** $10.786\text{ N}\cdot\text{m}$ (exactly holding body mass against gravity)
* **Min normal contact force:** $77.567\text{ N}$ (strictly positive, well above zero)
* **Max lateral slip velocity:** $8.141 \times 10^{-10}\text{ m/s}$

---

## 5. CSV Export Format

Generated file: `/tmp/stand_wheel_static_solution.csv`
* Total rows: 21 (time $t = 0.0\text{ s}$ to $t = 0.5\text{ s}$ in steps of $0.025\text{ s}$)
* Columns:
  1. `time`
  2. `q0`..`q26` (27 configuration entries: base pos, quat, 12 leg joints, 4 continuous wheel cos/sin pairs)
  3. `v0`..`v21` (22 velocity entries: base lin/ang, 12 leg vel, 4 wheel vel)
  4. `tau0`..`tau15` (16 actuator torques)
  5. `f_FL_x`, `f_FL_y`, `f_FL_z`, ..., `f_HR_z` (12 contact force entries in world frame)
  6. `lateral_slip` (contact lateral slip velocities)

All test suites (`ctest`: 7/7 tests passed) and standalone binary executions verify Step 6A complete and correct.
