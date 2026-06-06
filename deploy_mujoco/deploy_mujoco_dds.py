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
os.environ.setdefault("__GL_SYNC_TO_VBLANK", "0")
os.environ.setdefault("vblank_mode", "0")
import time
import struct
import threading
import argparse
import json
from contextlib import nullcontext
from pathlib import Path
from enum import IntEnum
from collections import deque

# Allow imports from the project root (same pattern as deploy_mujoco.py)
sys.path.append(str(Path(__file__).parent.parent.absolute()))

from common.path_config import PROJECT_ROOT
from common.ctrlcomp import StateAndCmd, PolicyOutput
from common.utils import get_gravity_orientation, FSMCommand
from common.safety import load_safety_config, SafetyFilter, HoldToConfirm
from FSM.FSM import FSM, FSMMode
from FSM.FSMState import FSMStateName

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

try:
    from common.joystick import JoyStick, JoystickButton
    JOYSTICK_IMPORT_ERROR = None
except Exception as e:
    JoyStick = None
    JOYSTICK_IMPORT_ERROR = e

    class JoystickButton(IntEnum):
        A = 0
        B = 1
        X = 2
        Y = 3
        L1 = 4
        R1 = 5
        SELECT = 6
        START = 7
        L3 = 8
        R3 = 9
        HOME = 10
        UP = 11
        DOWN = 12
        LEFT = 13
        RIGHT = 14


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

class NullJoyStick:
    """Joystick stub for headless/CI runs."""

    def update(self):
        return None

    def is_button_pressed(self, _button_id):
        return False

    def is_button_released(self, _button_id):
        return False

    def get_axis_value(self, _axis_id):
        return 0.0


def force_fsm_state(fsm: FSM, state_name: FSMStateName):
    """Force Python FSM into a state, mirroring C++ shadow mode behavior."""
    try:
        fsm.cur_policy.exit()
    except Exception:
        pass
    fsm.get_next_policy(state_name)
    fsm.FSMmode = FSMMode.CHANGE
    print(f"[FSM] force_state -> {state_name.name}")


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

    def __init__(
        self,
        num_joints: int,
        mj2lab: np.ndarray,
        state_lab_order: bool = False,
        cmd_lab_order: bool = False,
    ):
        """
        Parameters
        ----------
        num_joints : int
        mj2lab     : 1-D int array of length num_joints
                     mj2lab[lab_idx] = mujoco_idx
        state_lab_order : bool
            True  -> publish rt/lowstate in lab order via mj2lab.
            False -> publish rt/lowstate in MuJoCo order directly.
        cmd_lab_order : bool
            True  -> parse rt/lowcmd as lab order via mj2lab.
            False -> parse rt/lowcmd as MuJoCo order directly.
        """
        self.num_joints = num_joints
        self.mj2lab = np.asarray(mj2lab, dtype=np.int32)
        self.state_lab_order = bool(state_lab_order)
        self.cmd_lab_order = bool(cmd_lab_order)

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

        if self.state_lab_order:
            # Reorder MuJoCo joints → lab ordering for the LowState message
            for lab_i in range(self.num_joints):
                mj_i = int(self.mj2lab[lab_i])
                ls.motor_state[lab_i].q  = float(qpos_mj[mj_i])
                ls.motor_state[lab_i].dq = float(qvel_mj[mj_i])
        else:
            for i in range(self.num_joints):
                ls.motor_state[i].q  = float(qpos_mj[i])
                ls.motor_state[i].dq = float(qvel_mj[i])

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
        Convert C++ LowCmd → MuJoCo ordering arrays.

        Returns (target_q_mj, kp_mj, kd_mj) each of shape (num_joints,).
        """
        target_q_mj = np.zeros(self.num_joints, dtype=np.float32)
        kp_mj       = np.zeros(self.num_joints, dtype=np.float32)
        kd_mj       = np.zeros(self.num_joints, dtype=np.float32)
        if self.cmd_lab_order:
            for lab_i in range(self.num_joints):
                mj_i = int(self.mj2lab[lab_i])
                target_q_mj[mj_i] = float(cmd.motor_cmd[lab_i].q)
                kp_mj[mj_i]       = float(cmd.motor_cmd[lab_i].kp)
                kd_mj[mj_i]       = float(cmd.motor_cmd[lab_i].kd)
        else:
            for i in range(self.num_joints):
                target_q_mj[i] = float(cmd.motor_cmd[i].q)
                kp_mj[i]       = float(cmd.motor_cmd[i].kp)
                kd_mj[i]       = float(cmd.motor_cmd[i].kd)
        return target_q_mj, kp_mj, kd_mj


# ---------------------------------------------------------------------------
# Diff logger
# ---------------------------------------------------------------------------

class DiffLogger:
    """Print and optionally CSV-log per-step diffs between Python and C++."""

    def __init__(self, csv_path: str | None = None, print_every: int = 20):
        self._f = None
        self.print_every = max(1, int(print_every))
        self.cmp_steps = 0
        self.max_q = 0.0
        self.max_mean_q = 0.0
        self.max_kp = 0.0
        self.max_kd = 0.0
        self.max_lag = 0
        self.sum_lag = 0
        self.last = {
            "step": -1,
            "cpp_recv": 0,
            "max_q": 0.0,
            "mean_q": 0.0,
            "max_kp": 0.0,
            "max_kd": 0.0,
            "lag_steps": 0,
        }
        if csv_path:
            self._f = open(csv_path, "w", buffering=1)
            self._f.write("step,cpp_recv,max_q,mean_q,max_kp,max_kd,lag_steps\n")

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
        lag_steps: int = 0,
    ):
        dq  = np.abs(py_q  - cpp_q)
        dkp = np.abs(py_kp - cpp_kp)
        dkd = np.abs(py_kd - cpp_kd)
        max_q  = float(np.max(dq))
        mean_q = float(np.mean(dq))
        max_kp = float(np.max(dkp))
        max_kd = float(np.max(dkd))
        max_q_idx = int(np.argmax(dq))
        self.cmp_steps += 1
        self.max_q = max(self.max_q, max_q)
        self.max_mean_q = max(self.max_mean_q, mean_q)
        self.max_kp = max(self.max_kp, max_kp)
        self.max_kd = max(self.max_kd, max_kd)
        self.max_lag = max(self.max_lag, int(abs(lag_steps)))
        self.sum_lag += int(lag_steps)
        self.last = {
            "step": int(step),
            "cpp_recv": int(cpp_recv),
            "max_q": max_q,
            "mean_q": mean_q,
            "max_kp": max_kp,
            "max_kd": max_kd,
            "lag_steps": int(lag_steps),
        }
        if (self.cmp_steps % self.print_every) == 0:
            print(
                f"[step {step:6d} | cpp#{cpp_recv:6d}]  "
                f"q diff  max={max_q:.6f}  mean={mean_q:.6f}  |  "
                f"kp max={max_kp:.6f}  kd max={max_kd:.6f}  "
                f"lag={lag_steps:+d}  joint={max_q_idx}"
            )
        if self._f:
            self._f.write(
                f"{step},{cpp_recv},{max_q:.6f},{mean_q:.6f},{max_kp:.6f},{max_kd:.6f},{lag_steps}\n"
            )

    def summary(self) -> dict:
        return {
            "cmp_steps": int(self.cmp_steps),
            "max_q": float(self.max_q),
            "max_mean_q": float(self.max_mean_q),
            "max_kp": float(self.max_kp),
            "max_kd": float(self.max_kd),
            "max_lag_steps": int(self.max_lag),
            "avg_lag_steps": float(self.sum_lag / self.cmp_steps) if self.cmp_steps > 0 else 0.0,
            "last": dict(self.last),
        }

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
        "--track-yaml", default=None,
        help="TrackMimic YAML path (same file passed to C++ --track-yaml).",
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
    parser.add_argument(
        "--dds-lab-order", action="store_true",
        help="Legacy shortcut: enable lab-order remap for both LowState publish and LowCmd parse.",
    )
    parser.add_argument(
        "--state-lab-order", action="store_true",
        help="Publish DDS LowState in lab order via mj2lab.",
    )
    parser.add_argument(
        "--cmd-lab-order", action="store_true",
        help="Parse DDS LowCmd as lab order via mj2lab.",
    )
    parser.add_argument(
        "--no-joystick", action="store_true",
        help="Use a virtual neutral joystick (for headless/automated checks).",
    )
    parser.add_argument(
        "--shadow-sync", action="store_true",
        help="Force Python FSM to selected shadow state at startup (match C++ --shadow-state).",
    )
    parser.add_argument(
        "--shadow-state", default="beyond", choices=["beyond", "track"],
        help="FSM state used with --shadow-sync: beyond or track.",
    )
    parser.add_argument(
        "--headless", action="store_true",
        help="Run without MuJoCo viewer window (useful for CI/remote validation).",
    )
    parser.add_argument(
        "--max-steps", type=int, default=0,
        help="Stop after N control steps (0 means no explicit step limit).",
    )
    parser.add_argument(
        "--warmup-steps", type=int, default=40,
        help="Ignore first N compare samples in pass/fail checks.",
    )
    parser.add_argument(
        "--min-cmp-steps", type=int, default=100,
        help="Minimum compare samples required before evaluating pass/fail.",
    )
    parser.add_argument("--q-tol", type=float, default=1e-4, help="Max allowed abs q diff.")
    parser.add_argument("--mean-q-tol", type=float, default=1e-5, help="Max allowed mean q diff.")
    parser.add_argument("--kp-tol", type=float, default=1e-6, help="Max allowed abs kp diff.")
    parser.add_argument("--kd-tol", type=float, default=1e-6, help="Max allowed abs kd diff.")
    parser.add_argument(
        "--print-every", type=int, default=20,
        help="Print one diff line every N compare samples.",
    )
    parser.add_argument(
        "--lag-search", type=int, default=0,
        help="Compare C++ command against best Python output in last N control steps.",
    )
    parser.add_argument(
        "--cmd-wait-ms", type=float, default=5.0,
        help="Wait up to N ms for a fresh C++ LowCmd after each published LowState.",
    )
    parser.add_argument(
        "--summary-json", default=None,
        help="Optional output path for JSON summary.",
    )
    args = parser.parse_args()
    state_lab_order = bool(args.state_lab_order or args.dds_lab_order)
    cmd_lab_order = bool(args.cmd_lab_order or args.dds_lab_order)
    if args.headless and args.max_steps <= 0:
        args.max_steps = 1200
        print("[headless] --max-steps not set; defaulting to 1200.")

    yaml_path = None
    if args.yaml:
        yaml_path = os.path.abspath(os.path.expanduser(args.yaml))
        if not os.path.isfile(yaml_path):
            raise FileNotFoundError(f"YAML not found: {yaml_path}")
        args.yaml = yaml_path
        os.environ["BEYOND_MIMIC_CONFIG_PATH"] = yaml_path
        print(f"[Config] BEYOND_MIMIC_CONFIG_PATH={yaml_path}")

    track_yaml_path = None
    if args.track_yaml:
        track_yaml_path = os.path.abspath(os.path.expanduser(args.track_yaml))
        if not os.path.isfile(track_yaml_path):
            raise FileNotFoundError(f"Track YAML not found: {track_yaml_path}")
        args.track_yaml = track_yaml_path
        os.environ["TRACK_MIMIC_CONFIG_PATH"] = track_yaml_path
        print(f"[Config] TRACK_MIMIC_CONFIG_PATH={track_yaml_path}")

    # ── MuJoCo config ────────────────────────────────────────────────────────
    current_dir = os.path.dirname(os.path.abspath(__file__))
    mujoco_yaml_path = os.path.join(current_dir, "config", "mujoco.yaml")
    with open(mujoco_yaml_path, "r") as f:
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
    mapping_yaml = yaml_path
    if args.shadow_state == "track" and track_yaml_path:
        mapping_yaml = track_yaml_path
    if mapping_yaml and os.path.isfile(mapping_yaml):
        with open(mapping_yaml, "r") as f:
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
    py_history = deque(maxlen=max(1, int(args.lag_search) + 1))

    # ── Python FSM ────────────────────────────────────────────────────────────
    state_cmd     = StateAndCmd(num_joints)
    policy_output = PolicyOutput(num_joints)
    fsm_ctrl      = FSM(state_cmd, policy_output)
    safety        = SafetyFilter(num_joints, safety_cfg)
    cmd_gate      = HoldToConfirm(safety_cfg.command_hold_frames)
    if args.shadow_sync:
        target_state = (
            FSMStateName.SKILL_TRACK_MIMIC
            if args.shadow_state == "track"
            else FSMStateName.SKILL_BEYOND_MIMIC
        )
        force_fsm_state(fsm_ctrl, target_state)

    # ── Joystick ──────────────────────────────────────────────────────────────
    if args.no_joystick:
        joystick = NullJoyStick()
        print("[Joystick] Using NullJoyStick (--no-joystick).")
    else:
        if JoyStick is None:
            raise RuntimeError(
                "common.joystick import failed. Install pygame or use --no-joystick. "
                f"original error: {JOYSTICK_IMPORT_ERROR}"
            )
        try:
            joystick = JoyStick()
        except RuntimeError as e:
            if args.headless or args.shadow_sync:
                print(f"[Joystick][WARN] {e} Falling back to NullJoyStick.")
                joystick = NullJoyStick()
            else:
                raise

    # ── DDS bridge ────────────────────────────────────────────────────────────
    bridge: DDSBridge | None = None
    logger: DiffLogger | None = None
    prev_cmd_recv = 0

    if not args.no_cpp:
        ChannelFactoryInitialize(0, args.net)
        bridge = DDSBridge(
            num_joints,
            mj2lab,
            state_lab_order=state_lab_order,
            cmd_lab_order=cmd_lab_order,
        )
        logger = DiffLogger(args.csv, print_every=args.print_every)
        print(f"[DDS] Interface='{args.net}'  topic={bridge.LOWSTATE_TOPIC}")
        print(f"[DDS] LowState publish order: {'lab' if state_lab_order else 'mujoco'}")
        print(f"[DDS] LowCmd   parse order: {'lab' if cmd_lab_order else 'mujoco'}")
        print("[DDS] Waiting for C++ shadow process …")
        print(
            "[DDS] Start it with:\n"
            f"       ./build/deploy_real_onnx --shadow --net {args.net} "
            + (f"--yaml {args.yaml} " if args.yaml else "")
            + (f"--track-yaml {args.track_yaml} " if args.track_yaml else "")
            + f"--shadow-state {args.shadow_state}"
        )
    else:
        print("[no-cpp] DDS disabled. Running Python-only (same as deploy_mujoco.py).")

    # ── Main sim loop ─────────────────────────────────────────────────────────
    Running = True
    viewer_ctx = nullcontext(None) if args.headless else mujoco.viewer.launch_passive(m, d)
    with viewer_ctx as viewer:
        while Running and (args.max_steps <= 0 or step < args.max_steps):
            if viewer is not None and not viewer.is_running():
                break
            step_start = time.time()
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

                if viewer is not None and hasattr(viewer, "pert"):
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
                    py_history.append(
                        (int(step), py_actions.copy(), py_kps.copy(), py_kds.copy())
                    )

                    # ── Compare with latest C++ LowCmd ────────────────────
                    if bridge is not None:
                        cmd, cmd_recv = bridge.get_latest_cmd()
                        if cmd_recv <= prev_cmd_recv and args.cmd_wait_ms > 0:
                            wait_deadline = time.time() + (float(args.cmd_wait_ms) * 1e-3)
                            while cmd_recv <= prev_cmd_recv and time.time() < wait_deadline:
                                time.sleep(0.0002)
                                cmd, cmd_recv = bridge.get_latest_cmd()
                        if cmd is not None and cmd_recv > prev_cmd_recv:
                            prev_cmd_recv = cmd_recv
                            cpp_q, cpp_kp, cpp_kd = bridge.cmd_to_mj_arrays(cmd)
                            cmp_py_q = py_actions
                            cmp_py_kp = py_kps
                            cmp_py_kd = py_kds
                            lag_steps = 0
                            if args.lag_search > 0 and len(py_history) > 0:
                                best_score = None
                                for hist_step, hist_q, hist_kp, hist_kd in reversed(py_history):
                                    score = float(np.max(np.abs(hist_q - cpp_q)))
                                    if best_score is None or score < best_score:
                                        best_score = score
                                        cmp_py_q = hist_q
                                        cmp_py_kp = hist_kp
                                        cmp_py_kd = hist_kd
                                        lag_steps = int(step - hist_step)
                            if logger is not None and step >= args.warmup_steps:
                                logger.log(
                                    step, cmd_recv,
                                    cmp_py_q, cpp_q,
                                    cmp_py_kp, cpp_kp,
                                    cmp_py_kd, cpp_kd,
                                    lag_steps=lag_steps,
                                )

                    step += 1

            except ValueError as e:
                print(str(e))

            if viewer is not None:
                viewer.sync()
            time_until_next_step = m.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

    exit_code = 0
    summary = None
    if logger is not None:
        summary = logger.summary()
        logger.close()
        print(
            "[Summary] "
            f"cmp_steps={summary['cmp_steps']} "
            f"max_q={summary['max_q']:.8f} "
            f"max_mean_q={summary['max_mean_q']:.8f} "
            f"max_kp={summary['max_kp']:.8f} "
            f"max_kd={summary['max_kd']:.8f} "
            f"max_lag={summary['max_lag_steps']} "
            f"avg_lag={summary['avg_lag_steps']:.3f}"
        )
        enough_steps = summary["cmp_steps"] >= int(args.min_cmp_steps)
        pass_tol = (
            summary["max_q"] <= float(args.q_tol)
            and summary["max_mean_q"] <= float(args.mean_q_tol)
            and summary["max_kp"] <= float(args.kp_tol)
            and summary["max_kd"] <= float(args.kd_tol)
        )
        passed = enough_steps and pass_tol
        print(
            "[Result] "
            + ("PASS" if passed else "FAIL")
            + f" | criteria: steps>={args.min_cmp_steps}, "
            + f"max_q<={args.q_tol}, max_mean_q<={args.mean_q_tol}, "
            + f"max_kp<={args.kp_tol}, max_kd<={args.kd_tol}"
        )
        if not passed:
            exit_code = 2

    if args.summary_json:
        payload = {
            "headless": bool(args.headless),
            "max_steps": int(args.max_steps),
            "warmup_steps": int(args.warmup_steps),
            "thresholds": {
                "q_tol": float(args.q_tol),
                "mean_q_tol": float(args.mean_q_tol),
                "kp_tol": float(args.kp_tol),
                "kd_tol": float(args.kd_tol),
                "min_cmp_steps": int(args.min_cmp_steps),
            },
            "summary": summary,
            "exit_code": int(exit_code),
        }
        with open(args.summary_json, "w") as f:
            json.dump(payload, f, indent=2)
        print(f"[Summary] wrote JSON: {args.summary_json}")

    if exit_code != 0:
        raise SystemExit(exit_code)



if __name__ == "__main__":
    main()
