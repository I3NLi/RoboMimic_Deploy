#!/usr/bin/env python3
"""
mujoco_dds_compare.py
====================
MuJoCo simulation with DDS bridge for comparing Python vs C++ policy outputs.

Usage
-----
Terminal 1 (this script):
    python mujoco_dds_compare.py [--yaml /path/to/BeyondMimic.yaml] [--net lo] [--csv diff.csv]

Terminal 2 (C++ shadow process):
    ./build/robot_controller_onnx --shadow --net lo --yaml /path/to/BeyondMimic.yaml

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
import argparse
import json
from pathlib import Path
from enum import IntEnum
from collections import deque

# Allow imports from python_reference before shared.path_config adds repo root.
sys.path.append(str(Path(__file__).parent.parent.absolute()))

from shared.path_config import PROJECT_ROOT
from shared.ctrlcomp import StateAndCmd, PolicyOutput
from shared.utils import get_gravity_orientation, FSMCommand
from shared.safety import load_safety_config, SafetyFilter, HoldToConfirm
from runtime.compare import DiffLogger
from runtime.communication.dds import DDSBridge, WirelessRemoteBuilder, initialize_dds
from runtime.control import pd_control, sanitize_ctrl
from runtime.inference import force_fsm_state
from runtime.input import NullJoyStick
from runtime.rendering import mujoco_viewer_context
from fsm.machine import FSM
from fsm.state import FSMStateName

import numpy as np
import mujoco
import yaml

try:
    from shared.joystick import JoyStick, JoystickButton
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
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description=(
            "MuJoCo sim + DDS bridge: publishes rt/lowstate and compares "
            "Python policy output against C++ robot_controller_onnx --shadow."
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
        help="Disable DDS bridge (run Python-only, same as mujoco_reference.py).",
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
    mujoco_yaml_path = os.path.join(PROJECT_ROOT, "configs", "simulation", "mujoco.yaml")
    with open(mujoco_yaml_path, "r") as f:
        mj_cfg = yaml.load(f, Loader=yaml.FullLoader)
    xml_path         = os.path.join(PROJECT_ROOT, mj_cfg["xml_path"])
    simulation_dt    = mj_cfg["simulation_dt"]
    control_decimation = mj_cfg["control_decimation"]

    # ── Safety config ─────────────────────────────────────────────────────────
    safety_yaml_path = os.path.join(PROJECT_ROOT, "configs", "simulation", "safety.yaml")
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
                "shared.joystick import failed. Install pygame or use --no-joystick. "
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
        initialize_dds(0, args.net)
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
            f"       controller_cpp/build_z1/robot_controller_onnx --shadow --net {args.net} "
            + (f"--yaml {args.yaml} " if args.yaml else "")
            + (f"--track-yaml {args.track_yaml} " if args.track_yaml else "")
            + f"--shadow-state {args.shadow_state}"
        )
    else:
        print("[no-cpp] DDS disabled. Running Python-only (same as mujoco_reference.py).")

    # ── Main sim loop ─────────────────────────────────────────────────────────
    Running = True
    viewer_ctx = mujoco_viewer_context(m, d, args.headless)
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
