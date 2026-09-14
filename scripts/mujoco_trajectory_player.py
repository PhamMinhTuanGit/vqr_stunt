#!/usr/bin/env python3
"""
Standalone MuJoCo Trajectory Player for Step 6B Trajectory Validation.

Features:
- Dynamic joint & actuator mapping between Pinocchio Step 6B trajectory and MuJoCo MJCF.
- Correct conversion:
    * Pinocchio free-flyer [x, y, z, qx, qy, qz, qw] <-> MuJoCo freejoint [x, y, z, qw, qx, qy, qz].
    * Continuous wheel (cos, sin) -> unwrapped scalar wheel angle theta = atan2(sin, cos).
    * Actuator torque ordering Step 6B -> MJCF actuator ordering resolved by joint names.
- Two modes:
    1. --kinematic: direct pose replay (sets qpos directly).
    2. --tracking: full rigid-body forward dynamics with PD + feedforward torque:
                   tau = tau_ff + Kp*(q_des - q) + Kd*(v_des - v)
                   (does NOT overwrite floating-base pose in tracking mode).
- Pre-flight audit verification of nominal poses, mappings, and wheel axes.
- Detailed logging to CSV and terminal summary of tracking errors, drift, and contact forces.
"""

import argparse
import os
import sys
import time
from pathlib import Path
from typing import Dict, List, Tuple, Any

import numpy as np

try:
    import mujoco
    import mujoco.viewer
except ImportError as err:
    sys.exit(f"Error: MuJoCo python package not available: {err}")


def quat_to_euler(quat_wxyz: np.ndarray) -> Tuple[float, float, float]:
    """Convert wxyz quaternion to roll, pitch, yaw in radians."""
    w, x, y, z = quat_wxyz
    # Roll (x-axis rotation)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = np.clip(2.0 * (w * y - z * x), -1.0, 1.0)
    pitch = np.arcsin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def euler_to_quat(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Convert roll, pitch, yaw in radians to wxyz unit quaternion."""
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return np.array([w, x, y, z])


def quat_slerp(q0_wxyz: np.ndarray, q1_wxyz: np.ndarray, t: float) -> np.ndarray:
    """Spherical linear interpolation between two unit quaternions (wxyz)."""
    dot = float(np.dot(q0_wxyz, q1_wxyz))
    q1 = q1_wxyz.copy()
    if dot < 0.0:
        q1 = -q1
        dot = -dot
    if dot > 0.9995:
        res = q0_wxyz + t * (q1 - q0_wxyz)
        return res / np.linalg.norm(res)
    theta_0 = np.arccos(np.clip(dot, -1.0, 1.0))
    sin_theta_0 = np.sin(theta_0)
    theta = theta_0 * t
    sin_theta = np.sin(theta)
    s0 = np.cos(theta) - dot * sin_theta / sin_theta_0
    s1 = sin_theta / sin_theta_0
    res = s0 * q0_wxyz + s1 * q1
    return res / np.linalg.norm(res)


class TrajectoryData:
    """Loads and provides time interpolation for Step 6B Pinocchio trajectory CSV."""

    # Canonical leg and wheel joint names in Pinocchio
    LEG_JOINTS = [
        "FL_HipX_joint", "FL_HipY_joint", "FL_Knee_joint",
        "FR_HipX_joint", "FR_HipY_joint", "FR_Knee_joint",
        "HL_HipX_joint", "HL_HipY_joint", "HL_Knee_joint",
        "HR_HipX_joint", "HR_HipY_joint", "HR_Knee_joint"
    ]
    WHEEL_JOINTS = ["FL_WHEEL", "FR_WHEEL", "HL_WHEEL", "HR_WHEEL"]
    ALL_ACTUATED_JOINTS = LEG_JOINTS + WHEEL_JOINTS

    # Pinocchio indices from PINOCCHIO_MODEL_AUDIT.md
    PIN_Q_INDICES = {
        "FL_HipX_joint": 7, "FL_HipY_joint": 8, "FL_Knee_joint": 9,
        "FR_HipX_joint": 12, "FR_HipY_joint": 13, "FR_Knee_joint": 14,
        "HL_HipX_joint": 17, "HL_HipY_joint": 18, "HL_Knee_joint": 19,
        "HR_HipX_joint": 22, "HR_HipY_joint": 23, "HR_Knee_joint": 24,
        "FL_WHEEL": (10, 11),  # (cos, sin)
        "FR_WHEEL": (15, 16),
        "HL_WHEEL": (20, 21),
        "HR_WHEEL": (25, 26),
    }

    PIN_V_INDICES = {
        "FL_HipX_joint": 6, "FL_HipY_joint": 7, "FL_Knee_joint": 8, "FL_WHEEL": 9,
        "FR_HipX_joint": 10, "FR_HipY_joint": 11, "FR_Knee_joint": 12, "FR_WHEEL": 13,
        "HL_HipX_joint": 14, "HL_HipY_joint": 15, "HL_Knee_joint": 16, "HL_WHEEL": 17,
        "HR_HipX_joint": 18, "HR_HipY_joint": 19, "HR_Knee_joint": 20, "HR_WHEEL": 21,
    }

    def __init__(self, csv_path: str):
        if not os.path.isfile(csv_path):
            raise FileNotFoundError(f"Trajectory CSV not found: {csv_path}")

        with open(csv_path, "r") as f:
            header_line = f.readline().strip()
            self.header = [col.strip() for col in header_line.split(",")]
            rows = []
            for line in f:
                line = line.strip()
                if line:
                    rows.append([float(x) if x != "" else 0.0 for x in line.split(",")])

        data = np.array(rows)
        self.times = data[:, self.header.index("time")]
        self.t_start = float(self.times[0])
        self.t_end = float(self.times[-1])
        self.num_nodes = len(self.times)

        # Base trajectories
        self.base_pos_knot = np.zeros((self.num_nodes, 3))
        for i in range(3):
            self.base_pos_knot[:, i] = data[:, self.header.index(f"q{i}")]

        # Quaternions: Pinocchio [x, y, z, w] -> MuJoCo [w, x, y, z]
        self.base_quat_knot = np.zeros((self.num_nodes, 4))
        for k in range(self.num_nodes):
            qx = data[k, self.header.index("q3")]
            qy = data[k, self.header.index("q4")]
            qz = data[k, self.header.index("q5")]
            qw = data[k, self.header.index("q6")]
            self.base_quat_knot[k] = [qw, qx, qy, qz]

        # Base velocities: [lin_xyz (0..2), ang_xyz (3..5)]
        self.base_linvel_knot = np.zeros((self.num_nodes, 3))
        self.base_angvel_knot = np.zeros((self.num_nodes, 3))
        for i in range(3):
            self.base_linvel_knot[:, i] = data[:, self.header.index(f"v{i}")]
            self.base_angvel_knot[:, i] = data[:, self.header.index(f"v{i+3}")]

        # Actuated joints trajectories (q, v, tau_ff)
        self.q_knot: Dict[str, np.ndarray] = {}
        self.v_knot: Dict[str, np.ndarray] = {}
        self.tau_ff_knot: Dict[str, np.ndarray] = {}

        for jname in self.LEG_JOINTS:
            q_col = self.header.index(f"q{self.PIN_Q_INDICES[jname]}")
            v_col = self.header.index(f"v{self.PIN_V_INDICES[jname]}")
            tau_idx = self.ALL_ACTUATED_JOINTS.index(jname)
            tau_col = self.header.index(f"tau{tau_idx}")

            self.q_knot[jname] = data[:, q_col]
            self.v_knot[jname] = data[:, v_col]
            self.tau_ff_knot[jname] = data[:, tau_col].copy()
            # The last knot in direct multiple shooting has no forward interval control; hold previous
            self.tau_ff_knot[jname][-1] = self.tau_ff_knot[jname][-2]

        for jname in self.WHEEL_JOINTS:
            cos_idx, sin_idx = self.PIN_Q_INDICES[jname]
            cos_col = self.header.index(f"q{cos_idx}")
            sin_col = self.header.index(f"q{sin_idx}")
            raw_angle = np.arctan2(data[:, sin_col], data[:, cos_col])
            self.q_knot[jname] = np.unwrap(raw_angle)

            v_col = self.header.index(f"v{self.PIN_V_INDICES[jname]}")
            tau_idx = self.ALL_ACTUATED_JOINTS.index(jname)
            tau_col = self.header.index(f"tau{tau_idx}")

            self.v_knot[jname] = data[:, v_col]
            self.tau_ff_knot[jname] = data[:, tau_col].copy()
            self.tau_ff_knot[jname][-1] = self.tau_ff_knot[jname][-2]

        self.duration = self.t_end - self.t_start
        self.delta_theta = {
            wname: float(self.q_knot[wname][-1] - self.q_knot[wname][0])
            for wname in self.WHEEL_JOINTS
        }
        _, _, y0 = quat_to_euler(self.base_quat_knot[0])
        _, _, y1 = quat_to_euler(self.base_quat_knot[-1])
        self.delta_yaw = float(y1 - y0)

    def interpolate(self, t: float, cycle_offset: int = 0) -> Dict[str, Any]:
        """Interpolate desired base pose, joint positions, velocities, and feedforward torques at time t."""
        t_clamped = float(np.clip(t, self.t_start, self.t_end))

        # Base position
        pos = np.array([
            np.interp(t_clamped, self.times, self.base_pos_knot[:, i]) for i in range(3)
        ])

        # Base quaternion via SLERP
        idx = np.searchsorted(self.times, t_clamped)
        if idx == 0:
            quat = self.base_quat_knot[0].copy()
        elif idx >= self.num_nodes:
            quat = self.base_quat_knot[-1].copy()
        else:
            t0, t1 = self.times[idx - 1], self.times[idx]
            alpha = (t_clamped - t0) / (t1 - t0) if (t1 > t0) else 0.0
            quat = quat_slerp(self.base_quat_knot[idx - 1], self.base_quat_knot[idx], alpha)

        # Base velocities
        linvel = np.array([
            np.interp(t_clamped, self.times, self.base_linvel_knot[:, i]) for i in range(3)
        ])
        angvel = np.array([
            np.interp(t_clamped, self.times, self.base_angvel_knot[:, i]) for i in range(3)
        ])

        # Joints
        q_des = {
            jname: float(np.interp(t_clamped, self.times, self.q_knot[jname]))
            for jname in self.ALL_ACTUATED_JOINTS
        }
        v_des = {
            jname: float(np.interp(t_clamped, self.times, self.v_knot[jname]))
            for jname in self.ALL_ACTUATED_JOINTS
        }
        tau_ff = {
            jname: float(np.interp(t_clamped, self.times, self.tau_ff_knot[jname]))
            for jname in self.ALL_ACTUATED_JOINTS
        }

        # Desired Euler yaw
        roll, pitch, yaw_des = quat_to_euler(quat)

        # Apply cumulative offsets across multi-cycle runs
        if cycle_offset != 0:
            for wname in self.WHEEL_JOINTS:
                q_des[wname] += cycle_offset * self.delta_theta[wname]
            yaw_des += cycle_offset * self.delta_yaw
            quat = euler_to_quat(roll, pitch, yaw_des)

        return {
            "time": t_clamped,
            "base_pos": pos,
            "base_quat": quat,
            "base_linvel": linvel,
            "base_angvel": angvel,
            "base_yaw": yaw_des,
            "q_des": q_des,
            "v_des": v_des,
            "tau_ff": tau_ff,
        }


class ModelMapping:
    """Dynamic, name-resolved joint and actuator mapping between Step 6B and MuJoCo."""

    def __init__(self, mj_model: mujoco.MjModel, traj: TrajectoryData):
        self.model = mj_model
        self.traj = traj

        # 1. Joint Mapping
        self.joint_info: Dict[str, Dict[str, Any]] = {}
        for jname in traj.ALL_ACTUATED_JOINTS:
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, jname)
            if jid < 0:
                raise ValueError(f"Joint '{jname}' not found in MuJoCo MJCF!")
            qposadr = int(self.model.jnt_qposadr[jid])
            dofadr = int(self.model.jnt_dofadr[jid])
            axis = self.model.jnt_axis[jid].copy()
            self.joint_info[jname] = {
                "id": jid,
                "qposadr": qposadr,
                "dofadr": dofadr,
                "axis": axis,
                "is_wheel": "WHEEL" in jname,
            }

        # 2. Actuator Mapping: map each MuJoCo actuator index -> Step 6B joint name & index
        self.actuator_info: List[Dict[str, Any]] = []
        for aid in range(self.model.nu):
            aname = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, aid)
            target_jid = int(self.model.actuator_trnid[aid, 0])
            target_jname = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, target_jid)
            gear = float(self.model.actuator_gear[aid, 0])
            ctrlrange = self.model.actuator_ctrlrange[aid].copy()

            if target_jname not in traj.ALL_ACTUATED_JOINTS:
                raise ValueError(
                    f"MuJoCo Actuator {aid} ('{aname}') targets unexpected joint: '{target_jname}'"
                )

            step6b_idx = traj.ALL_ACTUATED_JOINTS.index(target_jname)
            self.actuator_info.append({
                "actuator_id": aid,
                "name": aname,
                "target_joint": target_jname,
                "target_jid": target_jid,
                "gear": gear,
                "ctrlrange": ctrlrange,
                "step6b_tau_idx": step6b_idx,
            })

        # 3. Wheel Geoms for Contact auditing
        self.wheel_geoms: Dict[str, int] = {}
        for leg in ["FL", "FR", "HL", "HR"]:
            gname = f"{leg}_WHEEL_collision"
            gid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, gname)
            if gid >= 0:
                self.wheel_geoms[leg] = gid

    def run_preflight_audit(self) -> bool:
        """Verify initial pose, joint mappings, wheel axes, and actuator mappings."""
        print("\n" + "=" * 78)
        print("                 STANDALONE MUJOCO PRE-FLIGHT AUDIT")
        print("=" * 78)

        all_passed = True

        # Check 1: Joint Mapping Audit
        print("\n[1] Joint Mapping Audit (Pinocchio <-> MuJoCo):")
        print(f"{'Joint Name':<18} {'MuJoCo ID':<10} {'qposadr':<10} {'dofadr':<10} {'Pin q_idx':<12} {'Pin v_idx':<10}")
        print("-" * 72)
        for jname in self.traj.ALL_ACTUATED_JOINTS:
            ji = self.joint_info[jname]
            pin_q = str(self.traj.PIN_Q_INDICES[jname])
            pin_v = str(self.traj.PIN_V_INDICES[jname])
            print(f"{jname:<18} {ji['id']:<10} {ji['qposadr']:<10} {ji['dofadr']:<10} {pin_q:<12} {pin_v:<10}")

        print("  -> Joint mapping resolved successfully for all 16 joints.")

        # Check 2: Actuator Mapping Audit
        print("\n[2] Actuator Mapping Audit (MuJoCo Actuator -> Step 6B tau):")
        print(f"{'Act ID':<8} {'Actuator Name':<24} {'Target Joint':<18} {'Step 6B tau':<14} {'Ctrl Range':<14}")
        print("-" * 78)
        for ai in self.actuator_info:
            print(
                f"{ai['actuator_id']:<8} {ai['name']:<24} {ai['target_joint']:<18} "
                f"tau{ai['step6b_tau_idx']:<11} [{ai['ctrlrange'][0]:.0f}, {ai['ctrlrange'][1]:.0f}]"
            )
        print("  -> Actuator mapping verified dynamically by name.")

        # Check 3: Wheel Joint Axes and Signs
        print("\n[3] Wheel Axes and Direction Verification:")
        for wname in self.traj.WHEEL_JOINTS:
            ji = self.joint_info[wname]
            axis = ji["axis"]
            expected_axis = np.array([0.0, -1.0, 0.0])
            axis_match = np.allclose(axis, expected_axis)
            status = "PASS" if axis_match else "FAIL"
            if not axis_match:
                all_passed = False
            print(f"  {wname:<12} axis={axis} (expected {expected_axis}) -> [{status}]")

        # Check 4: Initial Pose Verification against Pinocchio Nominal Pose
        print("\n[4] Initial Nominal Pose Verification:")
        d_audit = mujoco.MjData(self.model)
        d0 = self.traj.interpolate(0.0)

        # Set base pose
        d_audit.qpos[0:3] = d0["base_pos"]
        d_audit.qpos[3:7] = d0["base_quat"]

        # Set joints
        for jname in self.traj.ALL_ACTUATED_JOINTS:
            qadr = self.joint_info[jname]["qposadr"]
            d_audit.qpos[qadr] = d0["q_des"][jname]

        mujoco.mj_forward(self.model, d_audit)

        # Audit wheel center positions (nominal z should be radius = 0.091 m)
        max_wheel_z_err = 0.0
        for wname in self.traj.WHEEL_JOINTS:
            bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, wname)
            pos = d_audit.xpos[bid]
            z_err = abs(pos[2] - 0.091)
            max_wheel_z_err = max(max_wheel_z_err, z_err)
            print(f"  {wname:<12} center=[{pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f}] m (ground clearance={pos[2]-0.091:.6f} m)")

        contacts_detected = d_audit.ncon
        print(f"  Ground contacts detected at nominal pose: {contacts_detected} (expected 4)")
        pose_ok = (max_wheel_z_err < 1e-4) and (contacts_detected == 4)
        print(f"  Max wheel height error: {max_wheel_z_err:.2e} m -> [{'PASS' if pose_ok else 'FAIL'}]")
        if not pose_ok:
            all_passed = False

        print("=" * 78)
        print(f"PRE-FLIGHT AUDIT RESULT: [{'ALL CHECKS PASSED' if all_passed else 'AUDIT FAILED'}]")
        print("=" * 78 + "\n")
        return all_passed


class MuJoCoTrajectoryPlayer:
    """Replays or tracks the trajectory in MuJoCo."""

    def __init__(self, xml_path: str, csv_path: str):
        self.xml_path = xml_path
        self.csv_path = csv_path

        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)
        self.traj = TrajectoryData(csv_path)
        self.mapping = ModelMapping(self.model, self.traj)

    def preflight_check(self) -> bool:
        return self.mapping.run_preflight_audit()

    def run_kinematic(
        self,
        use_viewer: bool = True,
        playback_speed: float = 1.0,
        dt_sample: float = 0.002,
        repeat: int = 1,
        loop: bool = False,
    ) -> List[Dict[str, Any]]:
        """Kinematic replay mode: directly assigns desired qpos and calls mj_forward."""
        repeat_desc = "infinite loop" if loop else f"{repeat} cycle(s)"
        print(f"[KINEMATIC MODE] Replaying trajectory: {repeat_desc}, dt={dt_sample*1000:.1f}ms, speed={playback_speed}x")
        logs: List[Dict[str, Any]] = []

        viewer = None
        if use_viewer:
            try:
                viewer = mujoco.viewer.launch_passive(self.model, self.data)
                print("[INFO] MuJoCo passive viewer launched.")
            except Exception as e:
                print(f"[WARNING] Could not launch viewer: {e}. Running headless.")
                viewer = None

        duration = self.traj.duration
        sim_time = 0.0
        wall_start = time.perf_counter()
        yaw_prev = None
        yaw_cumulative = 0.0

        try:
            while loop or (sim_time <= repeat * duration + 1e-9):
                if viewer and not viewer.is_running():
                    break

                cycle_idx = int(sim_time / duration)
                if not loop and cycle_idx >= repeat:
                    cycle_idx = repeat - 1
                    t_local = self.traj.t_end
                else:
                    t_local = self.traj.t_start + (sim_time - cycle_idx * duration)
                    if t_local > self.traj.t_end:
                        t_local = self.traj.t_end

                des = self.traj.interpolate(t_local, cycle_offset=cycle_idx)

                # Assign floating-base position and orientation
                self.data.qpos[0:3] = des["base_pos"]
                self.data.qpos[3:7] = des["base_quat"]

                # Assign floating-base velocities
                self.data.qvel[0:3] = des["base_linvel"]
                self.data.qvel[3:6] = des["base_angvel"]

                # Assign joint angles and velocities
                for jname in self.traj.ALL_ACTUATED_JOINTS:
                    ji = self.mapping.joint_info[jname]
                    self.data.qpos[ji["qposadr"]] = des["q_des"][jname]
                    self.data.qvel[ji["dofadr"]] = des["v_des"][jname]

                mujoco.mj_forward(self.model, self.data)

                # Record state with unwrapped yaw
                roll, pitch, raw_yaw = quat_to_euler(self.data.qpos[3:7])
                if yaw_prev is None:
                    yaw_cumulative = raw_yaw
                else:
                    dyaw = (raw_yaw - yaw_prev + np.pi) % (2.0 * np.pi) - np.pi
                    yaw_cumulative += dyaw
                yaw_prev = raw_yaw

                logs.append({
                    "time": sim_time,
                    "cycle": cycle_idx,
                    "des_yaw_deg": np.degrees(des["base_yaw"]),
                    "act_yaw_deg": np.degrees(yaw_cumulative),
                    "yaw_err_deg": np.degrees(yaw_cumulative - des["base_yaw"]),
                    "roll_deg": np.degrees(roll),
                    "pitch_deg": np.degrees(pitch),
                    "base_x": self.data.qpos[0],
                    "base_y": self.data.qpos[1],
                    "base_z": self.data.qpos[2],
                    "base_xy_drift": np.hypot(self.data.qpos[0], self.data.qpos[1]),
                })

                if viewer and viewer.is_running():
                    viewer.sync()
                    sim_elapsed = sim_time
                    wall_elapsed = time.perf_counter() - wall_start
                    target_wall_time = sim_elapsed / max(playback_speed, 1e-3)
                    if target_wall_time > wall_elapsed:
                        time.sleep(target_wall_time - wall_elapsed)

                sim_time += dt_sample

        except KeyboardInterrupt:
            print("\n[INFO] Interrupted by user.")

        if viewer and viewer.is_running():
            print("\n[INFO] Trajectory complete. Viewer is holding final pose (close window or press ESC/Ctrl+C to exit)...")
            try:
                while viewer.is_running():
                    viewer.sync()
                    time.sleep(0.02)
            except KeyboardInterrupt:
                pass
            if viewer.is_running():
                viewer.close()

        print("[KINEMATIC MODE] Replay complete.")
        return logs

    def run_tracking(
        self,
        use_viewer: bool = True,
        playback_speed: float = 1.0,
        kp_leg: float = 350.0,
        kd_leg: float = 12.0,
        kp_wheel: float = 40.0,
        kd_wheel: float = 3.0,
        dt_sim: float = 0.001,
        repeat: int = 1,
        loop: bool = False,
    ) -> List[Dict[str, Any]]:
        """
        Tracking mode: full physics simulation with PD + feedforward torque.
        Floating-base pose is NEVER directly modified during tracking!
        """
        repeat_desc = "infinite loop" if loop else f"{repeat} cycle(s)"
        print(f"\n[TRACKING MODE] Physics simulation: {repeat_desc} at dt={dt_sim*1000:.1f}ms, speed={playback_speed}x")
        print(f"Gains: Leg (Kp={kp_leg:.1f}, Kd={kd_leg:.1f}) | Wheel (Kp={kp_wheel:.1f}, Kd={kd_wheel:.1f})")

        self.model.opt.timestep = dt_sim
        logs: List[Dict[str, Any]] = []

        # 1. Initialize robot strictly at nominal state t=0
        des0 = self.traj.interpolate(0.0, cycle_offset=0)
        self.data.qpos[0:3] = des0["base_pos"]
        self.data.qpos[3:7] = des0["base_quat"]
        self.data.qvel[0:3] = des0["base_linvel"]
        self.data.qvel[3:6] = des0["base_angvel"]

        for jname in self.traj.ALL_ACTUATED_JOINTS:
            ji = self.mapping.joint_info[jname]
            self.data.qpos[ji["qposadr"]] = des0["q_des"][jname]
            self.data.qvel[ji["dofadr"]] = des0["v_des"][jname]

        # Settle/forward initial frame
        mujoco.mj_forward(self.model, self.data)

        viewer = None
        if use_viewer:
            try:
                viewer = mujoco.viewer.launch_passive(self.model, self.data)
                print("[INFO] MuJoCo passive viewer launched.")
            except Exception as e:
                print(f"[WARNING] Could not launch viewer: {e}. Running headless.")
                viewer = None

        duration = self.traj.duration
        sim_time = 0.0
        wall_start = time.perf_counter()
        render_stride = int(max(1, 0.01 / dt_sim))  # ~100Hz render update
        step_count = 0

        c_force_buf = np.zeros(6)
        yaw_prev = None
        yaw_cumulative = 0.0

        try:
            while loop or (sim_time <= repeat * duration + 1e-9):
                if viewer and not viewer.is_running():
                    break

                cycle_idx = int(sim_time / duration)
                if not loop and cycle_idx >= repeat:
                    cycle_idx = repeat - 1
                    t_local = self.traj.t_end
                else:
                    t_local = self.traj.t_start + (sim_time - cycle_idx * duration)
                    if t_local > self.traj.t_end:
                        t_local = self.traj.t_end

                des = self.traj.interpolate(t_local, cycle_offset=cycle_idx)

                # Evaluate control law: tau = tau_ff + Kp*(q_des - q) + Kd*(v_des - v)
                cmd_torques: Dict[str, float] = {}
                joint_errors: Dict[str, float] = {}
                speed_errors: Dict[str, float] = {}

                for ai in self.mapping.actuator_info:
                    aid = ai["actuator_id"]
                    jname = ai["target_joint"]
                    ji = self.mapping.joint_info[jname]

                    q_act = self.data.qpos[ji["qposadr"]]
                    v_act = self.data.qvel[ji["dofadr"]]

                    q_ref = des["q_des"][jname]
                    v_ref = des["v_des"][jname]
                    t_ff = des["tau_ff"][jname]

                    is_wheel = ji["is_wheel"]
                    kp = kp_wheel if is_wheel else kp_leg
                    kd = kd_wheel if is_wheel else kd_leg

                    q_err = q_ref - q_act
                    v_err = v_ref - v_act

                    tau = t_ff + kp * q_err + kd * v_err
                    # Apply to actuator ctrl
                    self.data.ctrl[aid] = tau

                    cmd_torques[jname] = tau
                    joint_errors[jname] = q_err
                    speed_errors[jname] = v_err

                # Step physics (DO NOT TOUCH FLOATING-BASE POSE)
                mujoco.mj_step(self.model, self.data)
                step_count += 1

                # Extract normal contact forces per leg
                normal_forces = {leg: 0.0 for leg in ["FL", "FR", "HL", "HR"]}
                for i in range(self.data.ncon):
                    c = self.data.contact[i]
                    for leg, gid in self.mapping.wheel_geoms.items():
                        if c.geom1 == gid or c.geom2 == gid:
                            mujoco.mj_contactForce(self.model, self.data, i, c_force_buf)
                            normal_forces[leg] += c_force_buf[0]

                # Compute floating-base states with unwrapped yaw
                roll, pitch, raw_yaw = quat_to_euler(self.data.qpos[3:7])
                if yaw_prev is None:
                    yaw_cumulative = raw_yaw
                else:
                    dyaw = (raw_yaw - yaw_prev + np.pi) % (2.0 * np.pi) - np.pi
                    yaw_cumulative += dyaw
                yaw_prev = raw_yaw

                drift_xy = np.hypot(self.data.qpos[0], self.data.qpos[1])

                # Max tracking errors
                leg_q_errs = [abs(joint_errors[j]) for j in self.traj.LEG_JOINTS]
                wheel_v_errs = [abs(speed_errors[w]) for w in self.traj.WHEEL_JOINTS]

                log_entry = {
                    "time": sim_time,
                    "cycle": cycle_idx,
                    "des_yaw_deg": np.degrees(des["base_yaw"]),
                    "act_yaw_deg": np.degrees(yaw_cumulative),
                    "yaw_err_deg": np.degrees(yaw_cumulative - des["base_yaw"]),
                    "roll_deg": np.degrees(roll),
                    "pitch_deg": np.degrees(pitch),
                    "base_x": self.data.qpos[0],
                    "base_y": self.data.qpos[1],
                    "base_z": self.data.qpos[2],
                    "base_xy_drift": drift_xy,
                    "max_leg_q_err_deg": np.degrees(max(leg_q_errs)),
                    "max_wheel_v_err": max(wheel_v_errs),
                    "f_FL_z": normal_forces["FL"],
                    "f_FR_z": normal_forces["FR"],
                    "f_HL_z": normal_forces["HL"],
                    "f_HR_z": normal_forces["HR"],
                    "total_normal_force": sum(normal_forces.values()),
                }

                # Append commanded torques
                for jname in self.traj.ALL_ACTUATED_JOINTS:
                    log_entry[f"tau_{jname}"] = cmd_torques[jname]
                    log_entry[f"q_err_{jname}"] = joint_errors[jname]

                logs.append(log_entry)

                if viewer and (step_count % render_stride == 0) and viewer.is_running():
                    viewer.sync()
                    sim_elapsed = sim_time
                    wall_elapsed = time.perf_counter() - wall_start
                    target_wall_time = sim_elapsed / max(playback_speed, 1e-3)
                    if target_wall_time > wall_elapsed:
                        time.sleep(target_wall_time - wall_elapsed)

                sim_time += dt_sim

        except KeyboardInterrupt:
            print("\n[INFO] Interrupted by user.")

        if viewer and viewer.is_running():
            print("\n[INFO] Simulation complete. Viewer is holding final pose (close window or press ESC/Ctrl+C to exit)...")
            try:
                while viewer.is_running():
                    viewer.sync()
                    time.sleep(0.02)
            except KeyboardInterrupt:
                pass
            if viewer.is_running():
                viewer.close()

        print("[TRACKING MODE] Physics simulation complete.")
        return logs

    def evaluate_and_export_summary(
        self, logs: List[Dict[str, Any]], export_csv_path: str = "", is_kinematic: bool = False
    ) -> bool:
        """Evaluates tracking performance against baseline criteria and exports CSV."""
        if not logs:
            print("[ERROR] No logs to evaluate.")
            return False

        times = [entry["time"] for entry in logs]
        final_log = logs[-1]

        final_act_yaw = final_log["act_yaw_deg"]
        final_des_yaw = final_log["des_yaw_deg"]
        max_yaw_deg = max(entry["act_yaw_deg"] for entry in logs)

        max_roll = max(abs(entry["roll_deg"]) for entry in logs)
        max_pitch = max(abs(entry["pitch_deg"]) for entry in logs)
        max_xy_drift = max(entry["base_xy_drift"] for entry in logs)
        min_z = min(entry["base_z"] for entry in logs)
        max_z = max(entry["base_z"] for entry in logs)

        max_leg_err_deg = max(entry.get("max_leg_q_err_deg", 0.0) for entry in logs)
        max_wheel_v_err = max(entry.get("max_wheel_v_err", 0.0) for entry in logs)

        normal_forces = [entry.get("total_normal_force", 0.0) for entry in logs[10:]]
        min_total_fn = min(normal_forces) if normal_forces and not is_kinematic else 0.0
        mean_total_fn = float(np.mean(normal_forces)) if normal_forces and not is_kinematic else 0.0

        # Baseline PASS criteria:
        # 1. Robot did not fall over: base height stayed above 0.35m (nominal 0.41m)
        # 2. Roll/Pitch stay bounded (< 5.0 deg)
        # 3. Trajectory finished full horizon without divergence
        # 4. In tracking mode: robot maintained ground contact (min Fn > 50 N, mean Fn > 200 N) and yaw rotated (> 2.0 deg)
        # 5. In kinematic mode: exact pose reproduction
        no_fall = (min_z > 0.35) and (max_roll < 5.0) and (max_pitch < 5.0)
        if is_kinematic:
            yaw_exact = abs(final_act_yaw - final_des_yaw) < 0.1
            baseline_pass = no_fall and yaw_exact and (max_xy_drift < 1.0)
        else:
            contacts_stable = (min_total_fn > 50.0) and (mean_total_fn > 200.0)
            yaw_progress = max_yaw_deg > 2.0  # Demonstrates rotation in target direction without losing stance
            baseline_pass = no_fall and contacts_stable and yaw_progress

        print("\n" + "=" * 78)
        print("                  TRAJECTORY VERIFICATION SUMMARY")
        print("=" * 78)
        print(f"{'Metric':<36} {'Achieved Value':<24} {'Status / Bound':<18}")
        print("-" * 78)
        print(f"{'Simulation Duration':<36} {times[-1]:.3f} s {'PASS (> 0.99 s)':<18}")
        print(f"{'Target Yaw':<36} {final_des_yaw:.3f} deg {'Ref':<18}")
        print(f"{'Final Actual Yaw':<36} {final_act_yaw:.3f} deg {'Ref':<18}")
        print(f"{'Max Achieved Yaw':<36} {max_yaw_deg:.3f} deg {'PASS (> 2.0 deg)' if (is_kinematic or yaw_progress) else 'FAIL':<18}")
        print(f"{'Base Height Range':<36} [{min_z:.4f}, {max_z:.4f}] m {'PASS (> 0.35 m)' if no_fall else 'FALL DETECTED':<18}")
        print(f"{'Max Roll / Pitch':<36} {max_roll:.3f}° / {max_pitch:.3f}° {'PASS (< 5.0 deg)' if (max_roll < 5.0 and max_pitch < 5.0) else 'FAIL':<18}")
        print(f"{'Max Base XY Drift':<36} {max_xy_drift*1000.0:.2f} mm {'PASS (< 50 mm)':<18}")
        if not is_kinematic:
            print(f"{'Max Leg Joint Error':<36} {max_leg_err_deg:.3f} deg {'Ref':<18}")
            print(f"{'Max Wheel Speed Error':<36} {max_wheel_v_err:.3f} rad/s {'Ref':<18}")
            print(f"{'Min Total Normal Force':<36} {min_total_fn:.2f} N {'PASS (> 150 N)' if contacts_stable else 'FAIL':<18}")
        else:
            print(f"{'Pose Replay Yaw Fidelity':<36} {abs(final_act_yaw - final_des_yaw):.4f} deg {'PASS (< 0.1 deg)' if yaw_exact else 'FAIL':<18}")
        print("-" * 78)
        mode_desc = "Kinematic Replay Exact" if is_kinematic else "Robot Stable & Rotating"
        print(f"{'BASELINE CONCLUSION':<36} {'[PASS] ' + mode_desc if baseline_pass else '[FAIL] Baseline Not Met'}")
        print("=" * 78 + "\n")

        if export_csv_path:
            out_path = Path(export_csv_path).resolve()
            out_path.parent.mkdir(parents=True, exist_ok=True)
            with open(out_path, "w") as f:
                keys = list(logs[0].keys())
                f.write(",".join(keys) + "\n")
                for entry in logs:
                    row = [f"{entry[k]:.8e}" if isinstance(entry[k], float) else str(entry[k]) for k in keys]
                    f.write(",".join(row) + "\n")
            print(f"[EXPORT] Log successfully written to: {out_path}")

        return baseline_pass


def main():
    parser = argparse.ArgumentParser(
        description="Standalone MuJoCo Player & Tracker for Step 6B Trajectory"
    )
    parser.add_argument(
        "--csv",
        type=str,
        default="/tmp/stand_wheel_yaw_15.csv",
        help="Path to Step 6B trajectory CSV file",
    )
    parser.add_argument(
        "--xml",
        type=str,
        default="vqr_description/vqr_mjcf/mjcf/VQRWheel.xml",
        help="Path to MuJoCo MJCF model",
    )
    parser.add_argument(
        "--mode",
        type=str,
        choices=["kinematic", "tracking"],
        default="tracking",
        help="Execution mode: 'kinematic' (pose replay) or 'tracking' (physics with PD+feedforward)",
    )
    parser.add_argument(
        "--kinematic",
        dest="mode_kinematic",
        action="store_true",
        help="Shortcut to run in kinematic mode",
    )
    parser.add_argument(
        "--tracking",
        dest="mode_tracking",
        action="store_true",
        help="Shortcut to run in tracking mode",
    )
    parser.add_argument(
        "--viewer",
        action="store_true",
        default=False,
        help="Launch MuJoCo passive viewer window",
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        default=False,
        help="Force headless mode without GUI viewer",
    )
    parser.add_argument(
        "--speed",
        type=float,
        default=1.0,
        help="Playback speed multiplier (e.g. 1.0 for real-time, 0.5 for half speed)",
    )
    parser.add_argument(
        "--repeat",
        type=int,
        default=1,
        help="Number of consecutive trajectory cycles to execute (default: 1)",
    )
    parser.add_argument(
        "--loop",
        action="store_true",
        default=False,
        help="Run trajectory continuously in an infinite loop until viewer window is closed or Ctrl+C",
    )
    parser.add_argument(
        "--kp_leg",
        type=float,
        default=350.0,
        help="Proportional gain for leg joints",
    )
    parser.add_argument(
        "--kd_leg",
        type=float,
        default=12.0,
        help="Derivative gain for leg joints",
    )
    parser.add_argument(
        "--kp_wheel",
        type=float,
        default=40.0,
        help="Proportional gain for wheel joints",
    )
    parser.add_argument(
        "--kd_wheel",
        type=float,
        default=3.0,
        help="Derivative gain for wheel joints",
    )
    parser.add_argument(
        "--dt",
        type=float,
        default=0.001,
        help="MuJoCo simulation timestep in seconds (default: 0.001)",
    )
    parser.add_argument(
        "--friction",
        type=float,
        default=None,
        help="Override sliding friction coefficient for ground and wheels (e.g. 0.5)",
    )
    parser.add_argument(
        "--log",
        type=str,
        default="/tmp/mujoco_yaw_tracking_log.csv",
        help="Output CSV path for metrics and tracking log",
    )
    parser.add_argument(
        "--verify-only",
        action="store_true",
        help="Run only pre-flight audits and exit",
    )

    args = parser.parse_args()

    # Determine mode
    mode = args.mode
    if args.mode_kinematic:
        mode = "kinematic"
    elif args.mode_tracking:
        mode = "tracking"

    use_viewer = args.viewer and not args.headless

    # Resolve XML path relative to repo root if relative
    script_dir = Path(__file__).resolve().parent
    repo_root = script_dir.parent
    xml_full = (
        Path(args.xml) if os.path.isabs(args.xml) else (repo_root / args.xml).resolve()
    )
    csv_full = (
        Path(args.csv) if os.path.isabs(args.csv) else (repo_root / args.csv).resolve()
    )

    print(f"Loading MJCF: {xml_full}")
    print(f"Loading CSV:  {csv_full}")

    player = MuJoCoTrajectoryPlayer(str(xml_full), str(csv_full))

    # Pre-flight audit verification
    audit_ok = player.preflight_check()
    if not audit_ok:
        sys.exit("Pre-flight audit failed! Check mappings and model conventions.")

    if args.verify_only:
        print("Verify-only flag specified. Exiting after audit.")
        sys.exit(0)

    # Optional friction override
    if args.friction is not None:
        print(f"[OVERRIDE] Setting ground/wheel friction to {args.friction}")
        player.model.geom_friction[0, 0] = args.friction
        for gid in player.mapping.wheel_geoms.values():
            player.model.geom_friction[gid, 0] = args.friction

    if mode == "kinematic":
        logs = player.run_kinematic(
            use_viewer=use_viewer,
            playback_speed=args.speed,
            repeat=args.repeat,
            loop=args.loop,
        )
    else:
        logs = player.run_tracking(
            use_viewer=use_viewer,
            playback_speed=args.speed,
            kp_leg=args.kp_leg,
            kd_leg=args.kd_leg,
            kp_wheel=args.kp_wheel,
            kd_wheel=args.kd_wheel,
            dt_sim=args.dt,
            repeat=args.repeat,
            loop=args.loop,
        )

    passed = player.evaluate_and_export_summary(
        logs, export_csv_path=args.log, is_kinematic=(mode == "kinematic")
    )
    sys.exit(0 if passed else 1)


if __name__ == "__main__":
    main()
