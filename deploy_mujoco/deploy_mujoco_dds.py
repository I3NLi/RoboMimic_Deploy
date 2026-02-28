#!/usr/bin/env python3
"""
deploy_mujoco_dds.py
====================
MuJoCo simulation with DDS bridge for comparing Python vs C++ policy outputs.

Usage
-----
Terminal 1 (this script):
    python deploy_mujoco_dds.py [--yaml /path/to/BeyondMimic.yaml] [--net lo] [--csv diff.csv]

Terminal 2 (C++ shadow process):
    ./build/deploy_real_onnx --shadow --net lo --yaml /path/to/BeyondMimic.yaml

The bridge
----------
- Publishes  rt/lowstate  to DDS at every control step (MuJoCo joint state + IMU).
- Subscribes rt/lowcmd    from the C++ binary (policy output).
- Runs the Python FSM / BeyondMimic policy in-process.
- After each control step, compares Python output vs the latest C++ LowCmd and
  prints per-step diff statistics (max / mean absolute error in q, kp, kd).

Joint ordering
--------------
Real robot DDS uses *lab* ordering; MuJoCo uses its own ordering.
The mapping is loaded from the BeyondMimic YAML field ``mj2lab`` where
    mj2lab[lab_idx] = mujoco_idx
so we can reindex on the fly in both directions.

If --yaml is omitted the identity mapping is assumed (MuJoCo order == lab order).
"""

import sys
import os
import time
import struct
import threading
import argparse
from pathlib import Path

# Allow imports from the project root (same pattern as deploy_mujoco.py)
sys.path.append(str(Path(__file__).parent.parent.absolute()))

from common.path_config import PROJECT_ROOT
from common.ctrlcomp import StateAndCmd, PolicyOutput
from common.utils import get_gravity_orientation, FSMCommand
from common.safety import load_safety_config, SafetyFilter, HoldToConfirm
from common.joystick import JoyStick, JoystickButton
from FSM.FSM import FSM

import numpy as np
import mujoco
import mujoco.viewer
import yaml

from unitree_sdk2py.core.channel import (
    ChannelPublisher,
    ChannelSubscriber,
    ChannelFactoryInitialize,
)
from unitree_sdk2py.idl.default import (
    unitree_hg_msg_dds__LowCmd_,
    unitree_hg_msg_dds__LowState_,
)
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_   as LowCmdHG
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_ as LowStateHG


# ---------------------------------------------------------------------------
# wireless_remote byte builder
# ---------------------------------------------------------------------------

class WirelessRemoteBuilder:
    """
    Encode joystick state into the 40-byte wireless_remote array expected by
    the unitree LowState message.

    Bit positions match common/remote_controller.py KeyMap:
        R1=0, L1=1, start=2, select=3, R2=4, L2=5, F1=6, F2=7,
        A=8,  B=9,  X=10,   Y=11,   up=12, right=13, down=14, left=15
    """
    # KeyMap bit positions
    R1 = 0; L1 = 1; start = 2; select = 3
    R2 = 4; L2 = 5; F1 = 6;   F2 = 7
    A  = 8; B  = 9; X  = 10;  Y  = 11
    up = 12; right = 13; down = 14; left = 15

    # JoystickButton → KeyMap bit
    _JB_MAP = {
        JoystickButton.A:      A,
        JoystickButton.B:      B,
        JoystickButton.X:      X,
        JoystickButton.Y:      Y,
        JoystickButton.L1:     L1,
        JoystickButton.R1:     R1,
        JoystickButton.SELECT: select,
        JoystickButton.START:  start,
        JoystickButton.UP:     up,
        JoystickButton.DOWN:   down,
        JoystickButton.LEFT:   left,
        JoystickButton.RIGHT:  right,
    }

    @classmethod
    def from_joystick(cls, joy: JoyStick) -> list:
        """Build wireless_remote bytes from live joystick state."""
        bits = 0
        for jb, kbit in cls._JB_MAP.items():
            if joy.is_button_pressed(jb):
                bits |= (1 << kbit)

        # Axis mapping to match deploy_real.py sign conventions
        # remote lx  -> vel_cmd[1] = lx*-1   ; joystick axis 0 -> vel_cmd[1] = -axis0
        # remote ly  -> vel_cmd[0] = ly       ; joystick axis 1 -> vel_cmd[0] = -axis1  => ly = -axis1
        # remote rx  -> vel_cmd[2] = rx*-1    ; joystick axis 3 -> vel_cmd[2] = -axis3  => rx =  axis3
        lx = float(joy.get_axis_value(0))
        ly = float(-joy.get_axis_value(1))
        rx = float(joy.get_axis_value(3))
        ry = 0.0

        data = bytearray(40)
        struct.pack_into("H", data, 2,  bits & 0xFFFF)
        struct.pack_into("f", data, 4,  lx)
        struct.pack_into("f", data, 8,  rx)
        struct.pack_into("f", data, 12, ry)
        struct.pack_into("f", data, 20, ly)
        return list(data)

    @classmethod
    def neutral(cls) -> list:
        """Return a neutral (all zeros) wireless_remote byte array."""
        return [0] * 40


# ---------------------------------------------------------------------------
# DDS bridge
# ---------------------------------------------------------------------------

class DDSBridge:
    """
    Publish rt/lowstate (MuJoCo state in lab ordering).
    Subscribe rt/lowcmd  (C++ policy output in lab ordering).
    """

    LOWSTATE_TOPIC = "rt/lowstate"
    LOWCMD_TOPIC   = "rt/lowcmd"

    def __init__(self, num_joints: int, mj2lab: np.ndarray):
        """
        Parameters
        ----------
        num_joints : int
        mj2lab     : 1-D int array of length num_joints
                     mj2lab[lab_idx] = mujoco_idx
        """
        self.num_joints = num_joints
        self.mj2lab = np.asarray(mj2lab, dtype=np.int32)

        self._lock = threading.Lock()
        self._latest_cmd: LowCmdHG | None = None
        self._cmd_recv_count: int = 0

        # Publisher
        self._state_pub = ChannelPublisher(self.LOWSTATE_TOPIC, LowStateHG)
        self._state_pub.Init()

        # Subscriber
        self._cmd_sub = ChannelSubscriber(self.LOWCMD_TOPIC, LowCmdHG)
        self._cmd_sub.Init(self._on_lowcmd, 10)

        self._tick = 1   # keep > 0 so C++ wait_for_low_state() exits immediately

    def _on_lowcmd(self, msg: LowCmdHG):
        with self._lock:
            self._latest_cmd = msg
            self._cmd_recv_count += 1

    def publish_state(
        self,
        qpos_mj: np.ndarray,    # joint positions in MuJoCo ordering  (num_joints,)
        qvel_mj: np.ndarray,    # joint velocities in MuJoCo ordering  (num_joints,)
        quat_wxyz: np.ndarray,  # base quaternion [w, x, y, z]
        omega: np.ndarray,      # base angular velocity [wx, wy, wz]
        wireless_remote: list,
        mode_machine: int = 0,
    ):
        ls = unitree_hg_msg_dds__LowState_()
        ls.tick = self._tick
        self._tick += 1
        ls.mode_machine = mode_machine

        # Reorder MuJoCo joints → lab ordering for the LowState message
        for lab_i in range(self.num_joints):
            mj_i = int(self.mj2lab[lab_i])
            ls.motor_state[lab_i].q  = float(qpos_mj[mj_i])
            ls.motor_state[lab_i].dq = float(qvel_mj[mj_i])

        ls.imu_state.quaternion[0] = float(quat_wxyz[0])  # w
        ls.imu_state.quaternion[1] = float(quat_wxyz[1])  # x
        ls.imu_state.quaternion[2] = float(quat_wxyz[2])  # y
        ls.imu_state.quaternion[3] = float(quat_wxyz[3])  # z

        ls.imu_state.gyroscope[0] = float(omega[0])
        ls.imu_state.gyroscope[1] = float(omega[1])
        ls.imu_state.gyroscope[2] = float(omega[2])

        ls.wireless_remote = wireless_remote
        self._state_pub.Write(ls)

    def get_latest_cmd(self):
        """Return (LowCmdHG | None, recv_count)."""
        with self._lock:
            return self._latest_cmd, self._cmd_recv_count

    def cmd_to_mj_arrays(self, cmd: LowCmdHG):
        """
        Convert C++ LowCmd (lab ordering) → MuJoCo ordering arrays.

        Returns (target_q_mj, kp_mj, kd_mj) each of shape (num_joints,).
        """
        target_q_mj = np.zeros(self.num_joints, dtype=np.float32)
        kp_mj       = np.zeros(self.num_joints, dtype=np.float32)
        kd_mj       = np.zeros(self.num_joints, dtype=np.float32)
        for lab_i in range(self.num_joints):
            mj_i = int(self.mj2lab[lab_i])
            target_q_mj[mj_i] = float(cmd.motor_cmd[lab_i].q)
            kp_mj[mj_i]       = float(cmd.motor_cmd[lab_i].kp)
            kd_mj[mj_i]       = float(cmd.motor_cmd[lab_i].kd)
        return target_q_mj, kp_mj, kd_mj


# ---------------------------------------------------------------------------
# Diff logger
# ---------------------------------------------------------------------------

class DiffLogger:
    """Print and optionally CSV-log per-step diffs between Python and C++."""

    def __init__(self, csv_path: str | None = None):
        self._f = None
        if csv_path:
            self._f = open(csv_path, "w", buffering=1)
            self._f.write("step,cpp_recv,max_q,mean_q,max_kp,max_kd\n")

    def log(
        self,
        step: int,
        cpp_recv: int,
        py_q:  np.ndarray,
        cpp_q: np.ndarray,
        py_kp: np.ndarray,
        cpp_kp: np.ndarray,
        py_kd: np.ndarray,
        cpp_kd: np.ndarray,
    ):
        dq  = np.abs(py_q  - cpp_q)
        dkp = np.abs(py_kp - cpp_kp)
        dkd = np.abs(py_kd - cpp_kd)
        max_q  = float(np.max(dq))
        mean_q = float(np.mean(dq))
        max_kp = float(np.max(dkp))
        max_kd = float(np.max(dkd))
        print(
            f"[step {step:6d} | cpp#{cpp_recv:6d}]  "
            f"q diff  max={max_q:.5f}  mean={mean_q:.5f}  |  "
            f"kp max={max_kp:.4f}  kd max={max_kd:.4f}"
        )
        if self._f:
            self._f.write(
                f"{step},{cpp_recv},{max_q:.6f},{mean_q:.6f},{max_kp:.6f},{max_kd:.6f}\n"
            )

    def close(self):
        if self._f:
            self._f.close()


# ---------------------------------------------------------------------------
# PD / safety helpers (same as deploy_mujoco.py)
# ---------------------------------------------------------------------------

def pd_control(target_q, q, kp, target_dq, dq, kd):
    return (target_q - q) * kp + (target_dq - dq) * kd


def sanitize_ctrl(ctrl, model, fallback=None):
    out = np.asarray(ctrl, dtype=np.float32).reshape(-1)
    if fallback is None:
        fallback = np.zeros_like(out)
    fallback = np.asarray(fallback, dtype=np.float32).reshape(-1)
    if fallback.size != out.size:
        fallback = np.zeros_like(out)
    finite_mask = np.isfinite(out)
    if not np.all(finite_mask):
        out = np.where(finite_mask, out, fallback)
    out = np.nan_to_num(out, nan=0.0, posinf=0.0, neginf=0.0)
    ctrl_range = getattr(model, "actuator_ctrlrange", None)
    if ctrl_range is not None and ctrl_range.shape[0] == out.size:
        lo = ctrl_range[:, 0]
        hi = ctrl_range[:, 1]
        if np.any(hi > lo):
            return np.clip(out, lo, hi).astype(np.float32)
    return np.clip(out, -300.0, 300.0).astype(np.float32)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description=(
            "MuJoCo sim + DDS bridge: publishes rt/lowstate and compares "
            "Python policy output against C++ deploy_real_onnx --shadow."
        )
    )
    parser.add_argument(
        "--yaml", default=None,
        help="BeyondMimic YAML path (same file passed to C++ --yaml). "
             "Used to load mj2lab joint mapping.",
    )
    parser.add_argument(
        "--net", default="wlp3s0",
        help="DDS network interface (default: 'wlp3s0'). Must match C++ --net.",
    )
    parser.add_argument(
        "--csv", default=None,
        help="Optional CSV file to write per-step diff statistics.",
    )
    parser.add_argument(
        "--no-cpp", action="store_true",
        help="Disable DDS bridge (run Python-only, same as deploy_mujoco.py).",
    )
    args = parser.parse_args()

    # ── MuJoCo config ────────────────────────────────────────────────────────
    current_dir = os.path.dirname(os.path.abspath(__file__))
    mujoco_yaml_path = os.path.join(current_dir, "config", "mujoco.yaml")
    with open(mujoco_yaml_path, "r", encoding="utf-8") as f:
        mj_cfg = yaml.load(f, Loader=yaml.FullLoader)
    xml_path         = os.path.join(PROJECT_ROOT, mj_cfg["xml_path"])
    simulation_dt    = mj_cfg["simulation_dt"]
    control_decimation = mj_cfg["control_decimation"]

    # ── Safety config ─────────────────────────────────────────────────────────
    safety_yaml_path = os.path.join(current_dir, "config", "safety.yaml")
    safety_cfg = load_safety_config(safety_yaml_path)

    # ── Load mj2lab from YAML ─────────────────────────────────────────────────
    # mj2lab[lab_idx] = mujoco_idx
    mj2lab_list = None
    if args.yaml and os.path.isfile(args.yaml):
        with open(args.yaml, "r", encoding="utf-8") as f:
            bm_cfg = yaml.load(f, Loader=yaml.FullLoader)
        if "mj2lab" in bm_cfg:
            mj2lab_list = bm_cfg["mj2lab"]

    # ── Init MuJoCo ───────────────────────────────────────────────────────────
    m = mujoco.MjModel.from_xml_path(xml_path)
    d = mujoco.MjData(m)
    m.opt.timestep = simulation_dt
    num_joints = m.nu

    # Default: identity mapping (MuJoCo index == lab index)
    mj2lab = (
        np.array(mj2lab_list, dtype=np.int32)
        if mj2lab_list is not None
        else np.arange(num_joints, dtype=np.int32)
    )
    assert mj2lab.shape[0] == num_joints, (
        f"mj2lab length {mj2lab.shape[0]} != num_joints {num_joints}"
    )

    policy_output_action = np.zeros(num_joints, dtype=np.float32)
    kps = np.zeros(num_joints, dtype=np.float32)
    kds = np.zeros(num_joints, dtype=np.float32)
    sim_counter = 0
    step = 0

    # ── Python FSM ────────────────────────────────────────────────────────────
    state_cmd     = StateAndCmd(num_joints)
    policy_output = PolicyOutput(num_joints)
    fsm_ctrl      = FSM(state_cmd, policy_output)
    safety        = SafetyFilter(num_joints, safety_cfg)
    cmd_gate      = HoldToConfirm(safety_cfg.command_hold_frames)

    # ── Joystick ──────────────────────────────────────────────────────────────
    joystick = JoyStick()

    # ── DDS bridge ────────────────────────────────────────────────────────────
    bridge: DDSBridge | None = None
    logger: DiffLogger | None = None
    prev_cmd_recv = 0

    if not args.no_cpp:
        ChannelFactoryInitialize(0, args.net)
        bridge = DDSBridge(num_joints, mj2lab)
        logger = DiffLogger(args.csv)
        print(f"[DDS] Interface='{args.net}'  topic={bridge.LOWSTATE_TOPIC}")
        print("[DDS] Waiting for C++ shadow process …")
        print(
            "[DDS] Start it with:\n"
            f"       ./build/deploy_real_onnx --shadow --net {args.net} "
            + (f"--yaml {args.yaml}" if args.yaml else "")
        )
    else:
        print("[no-cpp] DDS disabled. Running Python-only (same as deploy_mujoco.py).")

    # ── Main sim loop ─────────────────────────────────────────────────────────
    Running = True
    with mujoco.viewer.launch_passive(m, d) as viewer:
        while viewer.is_running() and Running:
            try:
                # ── joystick input ────────────────────────────────────────
                if joystick.is_button_pressed(JoystickButton.SELECT):
                    Running = False

                joystick.update()

                if joystick.is_button_released(JoystickButton.L3):
                    state_cmd.skill_cmd = FSMCommand.PASSIVE
                if joystick.is_button_released(JoystickButton.UP):
                    state_cmd.skill_cmd = FSMCommand.PAUSE
                if cmd_gate.trigger(
                    "POS_RESET",
                    joystick.is_button_pressed(JoystickButton.START),
                ):
                    state_cmd.skill_cmd = FSMCommand.POS_RESET
                if cmd_gate.trigger(
                    "LOCO",
                    joystick.is_button_pressed(JoystickButton.A)
                    and joystick.is_button_pressed(JoystickButton.R1),
                ):
                    state_cmd.skill_cmd = FSMCommand.LOCO
                if cmd_gate.trigger(
                    "SKILL_1",
                    joystick.is_button_pressed(JoystickButton.X)
                    and joystick.is_button_pressed(JoystickButton.R1),
                ):
                    state_cmd.skill_cmd = FSMCommand.SKILL_1
                if cmd_gate.trigger(
                    "SKILL_2",
                    joystick.is_button_pressed(JoystickButton.Y)
                    and joystick.is_button_pressed(JoystickButton.R1),
                ):
                    state_cmd.skill_cmd = FSMCommand.SKILL_2
                if (
                    joystick.is_button_released(JoystickButton.B)
                    and joystick.is_button_pressed(JoystickButton.R1)
                ):
                    state_cmd.skill_cmd = FSMCommand.SKILL_3
                if cmd_gate.trigger(
                    "SKILL_4",
                    joystick.is_button_pressed(JoystickButton.Y)
                    and joystick.is_button_pressed(JoystickButton.L1),
                ):
                    state_cmd.skill_cmd = FSMCommand.SKILL_4
                if (
                    joystick.is_button_released(JoystickButton.B)
                    and joystick.is_button_pressed(JoystickButton.L1)
                ):
                    state_cmd.skill_cmd = FSMCommand.SKILL_5
                if cmd_gate.trigger(
                    "SKILL_6",
                    joystick.is_button_pressed(JoystickButton.X)
                    and joystick.is_button_pressed(JoystickButton.L1),
                ):
                    state_cmd.skill_cmd = FSMCommand.SKILL_6
                if cmd_gate.trigger(
                    "SKILL_7",
                    joystick.is_button_pressed(JoystickButton.A)
                    and joystick.is_button_pressed(JoystickButton.L1),
                ):
                    state_cmd.skill_cmd = FSMCommand.SKILL_7

                state_cmd.vel_cmd[0] = -joystick.get_axis_value(1)
                state_cmd.vel_cmd[1] = -joystick.get_axis_value(0)
                state_cmd.vel_cmd[2] = -joystick.get_axis_value(3)

                step_start = time.time()

                # ── PD torques → apply to MuJoCo ─────────────────────────
                raw_tau = pd_control(
                    policy_output_action, d.qpos[7:], kps,
                    np.zeros_like(kps), d.qvel[6:], kds,
                )
                fallback_tau = (
                    -safety_cfg.damping_kd
                    * np.asarray(d.qvel[6:], dtype=np.float32)
                )
                tau = sanitize_ctrl(raw_tau, m, fallback=fallback_tau)
                if (not np.isfinite(raw_tau).all()) or np.max(np.abs(raw_tau)) > 1e4:
                    print("[Safety] abnormal torque detected; sanitized.")
                if safety_cfg.dry_run:
                    d.ctrl[:] = 0
                else:
                    d.ctrl[:] = tau

                if hasattr(viewer, "pert"):
                    d.xfrc_applied[:] = 0
                    mujoco.mjv_applyPerturbForce(m, d, viewer.pert)

                mujoco.mj_step(m, d)
                sim_counter += 1

                if sim_counter % control_decimation == 0:
                    qj    = d.qpos[7:]
                    dqj   = d.qvel[6:]
                    quat  = d.qpos[3:7]   # [w, x, y, z]
                    omega = d.qvel[3:6]

                    gravity_orientation = get_gravity_orientation(quat)

                    state_cmd.q          = qj.copy()
                    state_cmd.dq         = dqj.copy()
                    state_cmd.gravity_ori = gravity_orientation.copy()
                    state_cmd.base_quat  = quat.copy()
                    state_cmd.ang_vel    = omega.copy()

                    # ── Publish LowState to DDS ───────────────────────────
                    if bridge is not None:
                        remote = WirelessRemoteBuilder.from_joystick(joystick)
                        bridge.publish_state(qj, dqj, quat, omega, remote)

                    # ── Python policy ─────────────────────────────────────
                    fsm_ctrl.run()
                    py_actions = policy_output.actions.copy()
                    py_kps     = policy_output.kps.copy()
                    py_kds     = policy_output.kds.copy()

                    py_actions, py_kps, py_kds, force_damping = safety.filter_actions(
                        py_actions, py_kps, py_kds
                    )
                    if force_damping:
                        py_actions = d.qpos[7:].copy()
                        py_kps = np.zeros_like(py_kps)
                        py_kds = np.ones_like(py_kds) * safety_cfg.damping_kd
                    if safety_cfg.dry_run:
                        py_kps = np.zeros_like(py_kps)
                        py_kds = np.zeros_like(py_kds)

                    # Use Python output to drive the simulation
                    policy_output_action = py_actions.copy()
                    kps = py_kps.copy()
                    kds = py_kds.copy()

                    # ── Compare with latest C++ LowCmd ────────────────────
                    if bridge is not None:
                        cmd, cmd_recv = bridge.get_latest_cmd()
                        if cmd is not None and cmd_recv > prev_cmd_recv:
                            prev_cmd_recv = cmd_recv
                            cpp_q, cpp_kp, cpp_kd = bridge.cmd_to_mj_arrays(cmd)
                            logger.log(
                                step, cmd_recv,
                                py_actions, cpp_q,
                                py_kps, cpp_kp,
                                py_kds, cpp_kd,
                            )

                    step += 1

            except ValueError as e:
                print(str(e))

            viewer.sync()
            time_until_next_step = m.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

    if logger is not None:
        logger.close()


if __name__ == "__main__":
    main()
