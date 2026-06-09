#!/usr/bin/env python3
"""Measure pure-sim and real-state-sim inference rates on MagicBot Z1.

This tool never publishes JointCommand. In real-state mode it only subscribes
to SDK low-level state, feeds that state through LocoMode, and advances a local
MuJoCo model for timing/load measurement.
"""

from __future__ import annotations

import argparse
import ctypes
import importlib.util
import json
import os
import platform
import statistics
import sys
import time
from pathlib import Path

import mujoco
import numpy as np
import yaml

sys.path.append(str(Path(__file__).resolve().parents[1]))

from shared.ctrlcomp import PolicyOutput, StateAndCmd
from shared.path_config import PROJECT_ROOT
from shared.utils import get_gravity_orientation
from policies.loco_mode.LocoMode import LocoMode
from runtime.control import pd_control, sanitize_ctrl


def actuator_joint_indices(model):
    qpos_idx = np.zeros(model.nu, dtype=np.int32)
    qvel_idx = np.zeros(model.nu, dtype=np.int32)
    for actuator_id in range(model.nu):
        joint_id = int(model.actuator_trnid[actuator_id, 0])
        qpos_idx[actuator_id] = int(model.jnt_qposadr[joint_id] - 7)
        qvel_idx[actuator_id] = int(model.jnt_dofadr[joint_id] - 6)
    return qpos_idx, qvel_idx


def load_initial_joint_targets(yaml_path, num_joints):
    with open(yaml_path, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f) or {}
    mj2lab = np.asarray(cfg.get("mj2lab", []), dtype=np.int32)
    default_lab = np.asarray(cfg.get("default_angles_lab", []), dtype=np.float32)
    kp_lab = np.asarray(cfg.get("kp_lab", []), dtype=np.float32)
    kd_lab = np.asarray(cfg.get("kd_lab", []), dtype=np.float32)
    tau_lab = np.asarray(cfg.get("tau_limit", []), dtype=np.float32)
    if mj2lab.size == 0 or default_lab.size == 0:
        return None, None, None, None

    default_mj = np.zeros(num_joints, dtype=np.float32)
    kp_mj = np.zeros(num_joints, dtype=np.float32)
    kd_mj = np.zeros(num_joints, dtype=np.float32)
    tau_mj = np.zeros(num_joints, dtype=np.float32)
    for lab_idx, mj_idx in enumerate(mj2lab):
        if not (0 <= mj_idx < num_joints):
            continue
        if lab_idx < default_lab.size:
            default_mj[mj_idx] = default_lab[lab_idx]
        if lab_idx < kp_lab.size:
            kp_mj[mj_idx] = kp_lab[lab_idx]
        if lab_idx < kd_lab.size:
            kd_mj[mj_idx] = kd_lab[lab_idx]
        if lab_idx < tau_lab.size:
            tau_mj[mj_idx] = tau_lab[lab_idx]
    return default_mj, kp_mj, kd_mj, tau_mj


def percentile_ms(samples, q):
    if not samples:
        return 0.0
    values = sorted(samples)
    idx = min(len(values) - 1, max(0, int(round((len(values) - 1) * q))))
    return float(values[idx])


def make_sim():
    with open(PROJECT_ROOT / "configs" / "simulation" / "mujoco.yaml", "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    model = mujoco.MjModel.from_xml_path(str(PROJECT_ROOT / cfg["xml_path"]))
    data = mujoco.MjData(model)
    model.opt.timestep = float(cfg["simulation_dt"])
    qpos_idx, qvel_idx = actuator_joint_indices(model)
    if cfg.get("initial_base_height", None) is not None:
        data.qpos[2] = float(cfg["initial_base_height"])
    init_q, init_kp, init_kd, ctrl_limit = load_initial_joint_targets(
        PROJECT_ROOT / cfg["initial_pose_yaml"],
        model.nu,
    )
    if init_q is not None:
        data.qpos[7 + qpos_idx] = init_q
    if cfg.get("initial_base_height", None) is not None or init_q is not None:
        data.qvel[:] = 0.0
        mujoco.mj_forward(model, data)
    return cfg, model, data, qpos_idx, qvel_idx, init_q, init_kp, init_kd, ctrl_limit


def make_loco(num_joints):
    state_cmd = StateAndCmd(num_joints)
    policy_output = PolicyOutput(num_joints)
    policy = LocoMode(state_cmd, policy_output)
    policy.enter()
    return state_cmd, policy_output, policy


def run_policy(policy, state_cmd, policy_output, q, dq, quat, ang_vel):
    state_cmd.q = q.astype(np.float32).copy()
    state_cmd.dq = dq.astype(np.float32).copy()
    state_cmd.gravity_ori = get_gravity_orientation(quat).astype(np.float32)
    state_cmd.base_quat = np.asarray(quat, dtype=np.float32).copy()
    state_cmd.ang_vel = np.asarray(ang_vel, dtype=np.float32).copy()
    state_cmd.vel_cmd[:] = 0.0
    t0 = time.perf_counter()
    policy.run()
    infer_ms = (time.perf_counter() - t0) * 1000.0
    return (
        policy_output.actions.copy(),
        policy_output.kps.copy(),
        policy_output.kds.copy(),
        infer_ms,
    )


def connect_real_state(sdk_root, local_ip):
    sdk_root = Path(sdk_root).expanduser().resolve()
    module_path = sdk_root / "deploy_onnx" / "z1_loco_onnx_deploy.py"
    spec = importlib.util.spec_from_file_location("magicbot_z1_loco_onnx_deploy_rate", module_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"failed to load {module_path}")
    sdk = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = sdk
    spec.loader.exec_module(sdk)

    lib_dir = sdk_root / "lib" / platform.machine()
    ctypes.CDLL(str(lib_dir / "libmagicbot_z1_sdk.so"), mode=ctypes.RTLD_GLOBAL)
    sys.path.insert(0, str(lib_dir))
    import magicbot_z1_python as magicbot  # type: ignore

    robot_state = sdk.RobotState()
    robot = magicbot.MagicRobot()
    controller = None
    if not robot.initialize(local_ip):
        raise RuntimeError(f"robot.initialize failed for local_ip={local_ip}")
    status = robot.connect()
    if status.code != magicbot.ErrorCode.OK:
        raise RuntimeError(f"robot.connect failed: {status.code} {status.message}")
    status = robot.set_motion_control_level(magicbot.ControllerLevel.LowLevel)
    if status.code != magicbot.ErrorCode.OK:
        raise RuntimeError(f"set LowLevel failed: {status.code} {status.message}")
    controller = robot.get_low_level_motion_controller()
    if hasattr(controller, "initialize"):
        controller.initialize()
    controller.subscribe_leg_state(robot_state.update_leg)
    controller.subscribe_arm_state(robot_state.update_arm)
    controller.subscribe_waist_state(robot_state.update_waist)
    controller.subscribe_head_state(robot_state.update_head)
    controller.subscribe_body_imu(robot_state.update_imu)
    sdk.wait_for_state(robot_state, 10.0)
    return sdk, magicbot, robot, controller, robot_state


def finish_real_state(magicbot, robot, controller, skip_disconnect=True):
    try:
        if robot is not None and magicbot is not None:
            robot.set_motion_control_level(magicbot.ControllerLevel.HighLevel)
    except Exception as exc:
        print(f"[real-state][WARN] set HighLevel failed: {exc}")
    try:
        if controller is not None and hasattr(controller, "shutdown"):
            controller.shutdown()
    except Exception as exc:
        print(f"[real-state][WARN] controller shutdown failed: {exc}")
    if skip_disconnect:
        print("[real-state] skip robot.disconnect()/shutdown to avoid known SDK native disconnect crash after LowLevel.")
        return
    try:
        robot.disconnect()
    except Exception as exc:
        print(f"[real-state][WARN] disconnect failed: {exc}")
    try:
        robot.shutdown()
    except Exception as exc:
        print(f"[real-state][WARN] shutdown failed: {exc}")


def run_loop(args):
    cfg, model, data, qpos_idx, qvel_idx, init_q, init_kp, init_kd, ctrl_limit = make_sim()
    if args.sim_dt > 0.0:
        model.opt.timestep = float(args.sim_dt)
    control_decimation = int(args.control_decimation or cfg["control_decimation"])
    state_cmd, policy_output, policy = make_loco(model.nu)

    real = args.mode == "real-state-sim"
    magicbot = robot = controller = robot_state = None
    if real:
        _sdk, magicbot, robot, controller, robot_state = connect_real_state(args.sdk_root, args.local_ip)

    real_forward_only = bool(real and args.real_forward_only)
    if real_forward_only:
        control_dt = float(model.opt.timestep) * int(cfg["control_decimation"])
        target_steps = max(1, int(round(float(args.duration) / control_dt)))
    else:
        target_steps = max(1, int(round(float(args.duration) / model.opt.timestep)))
    policy_action = init_q.copy() if init_q is not None else np.zeros(model.nu, dtype=np.float32)
    kps = init_kp.copy() if init_kp is not None else np.zeros(model.nu, dtype=np.float32)
    kds = init_kd.copy() if init_kd is not None else np.zeros(model.nu, dtype=np.float32)
    infer_ms = []
    state_age_ms = []
    missed_deadline = 0
    control_steps = 0
    max_abs_tau = 0.0
    max_abs_q = 0.0
    max_abs_dq = 0.0

    start = time.perf_counter()
    next_t = start
    try:
        for step in range(target_steps):
            if real_forward_only:
                q, dq, quat, ang_vel, _counts = robot_state.snapshot()
                now = time.monotonic()
                with robot_state.lock:
                    stamps = list(robot_state.stamps.values())
                valid_stamps = [s for s in stamps if s > 0.0]
                if valid_stamps:
                    state_age_ms.append((now - min(valid_stamps)) * 1000.0)
                data.qpos[3:7] = quat
                data.qpos[7 + qpos_idx] = q
                data.qvel[3:6] = ang_vel
                data.qvel[6 + qvel_idx] = dq
                mujoco.mj_forward(model, data)
                policy_action, kps, kds, ms = run_policy(policy, state_cmd, policy_output, q, dq, quat, ang_vel)
                infer_ms.append(ms)
                control_steps += 1
                max_abs_q = max(max_abs_q, float(np.max(np.abs(q))))
                max_abs_dq = max(max_abs_dq, float(np.max(np.abs(dq))))
                if args.realtime:
                    next_t += control_dt
                    sleep_s = next_t - time.perf_counter()
                    if sleep_s > 0:
                        time.sleep(sleep_s)
                    else:
                        missed_deadline += 1
                        next_t = time.perf_counter()
                continue

            if real and step % control_decimation == 0:
                q, dq, quat, ang_vel, _counts = robot_state.snapshot()
                now = time.monotonic()
                with robot_state.lock:
                    stamps = list(robot_state.stamps.values())
                valid_stamps = [s for s in stamps if s > 0.0]
                if valid_stamps:
                    state_age_ms.append((now - min(valid_stamps)) * 1000.0)
                data.qpos[3:7] = quat
                data.qpos[7 + qpos_idx] = q
                data.qvel[3:6] = ang_vel
                data.qvel[6 + qvel_idx] = dq
                mujoco.mj_forward(model, data)
                policy_action, kps, kds, ms = run_policy(policy, state_cmd, policy_output, q, dq, quat, ang_vel)
                infer_ms.append(ms)
                control_steps += 1
            elif (not real) and step % control_decimation == 0:
                q = data.qpos[7 + qpos_idx].copy()
                dq = data.qvel[6 + qvel_idx].copy()
                quat = data.qpos[3:7].copy()
                ang_vel = data.qvel[3:6].copy()
                policy_action, kps, kds, ms = run_policy(policy, state_cmd, policy_output, q, dq, quat, ang_vel)
                infer_ms.append(ms)
                control_steps += 1

            q_act = data.qpos[7 + qpos_idx].copy()
            dq_act = data.qvel[6 + qvel_idx].copy()
            tau = sanitize_ctrl(
                pd_control(policy_action, q_act, kps, np.zeros_like(kps), dq_act, kds),
                model,
                fallback=-3.0 * dq_act,
                ctrl_limit=ctrl_limit,
            )
            data.ctrl[:] = tau
            mujoco.mj_step(model, data)
            max_abs_tau = max(max_abs_tau, float(np.max(np.abs(tau))))
            max_abs_q = max(max_abs_q, float(np.max(np.abs(q_act))))
            max_abs_dq = max(max_abs_dq, float(np.max(np.abs(dq_act))))

            if args.realtime:
                next_t += model.opt.timestep
                sleep_s = next_t - time.perf_counter()
                if sleep_s > 0:
                    time.sleep(sleep_s)
                else:
                    missed_deadline += 1
                    next_t = time.perf_counter()
    finally:
        if real and args.real_clean_exit:
            finish_real_state(magicbot, robot, controller, skip_disconnect=not args.real_disconnect)

    elapsed = time.perf_counter() - start
    summary = {
        "mode": args.mode,
        "real_forward_only": real_forward_only,
        "duration_s": float(args.duration),
        "elapsed_s": elapsed,
        "sim_steps": target_steps,
        "control_steps": control_steps,
        "sim_hz": target_steps / max(elapsed, 1e-9),
        "control_hz": control_steps / max(elapsed, 1e-9),
        "target_sim_hz": (1.0 / control_dt) if real_forward_only else (1.0 / float(model.opt.timestep)),
        "target_control_hz": 1.0 / (float(model.opt.timestep) * control_decimation),
        "simulation_dt": float(model.opt.timestep),
        "control_decimation": control_decimation,
        "deadline_misses": missed_deadline,
        "infer_mean_ms": statistics.fmean(infer_ms) if infer_ms else 0.0,
        "infer_p95_ms": percentile_ms(infer_ms, 0.95),
        "infer_p99_ms": percentile_ms(infer_ms, 0.99),
        "infer_max_ms": max(infer_ms) if infer_ms else 0.0,
        "state_age_mean_ms": statistics.fmean(state_age_ms) if state_age_ms else None,
        "state_age_max_ms": max(state_age_ms) if state_age_ms else None,
        "max_abs_tau": max_abs_tau,
        "max_abs_q": max_abs_q,
        "max_abs_dq": max_abs_dq,
    }
    print("RATE_SUMMARY " + json.dumps(summary, sort_keys=True))
    if args.summary_json:
        with open(args.summary_json, "w", encoding="utf-8") as f:
            json.dump(summary, f, indent=2, sort_keys=True)
    if real and not args.real_clean_exit:
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(0)
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("pure-sim", "real-state-sim"), required=True)
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--summary-json", default=None)
    parser.add_argument("--sim-dt", type=float, default=0.0, help="Override MuJoCo timestep. <=0 keeps YAML value.")
    parser.add_argument("--control-decimation", type=int, default=0, help="Override policy control decimation. <=0 keeps YAML value.")
    parser.add_argument("--no-realtime", dest="realtime", action="store_false")
    parser.set_defaults(realtime=True)
    parser.add_argument("--sdk-root", default=os.environ.get("MAGICBOT_SDK_ROOT", "/home/eame/magicbot-z1_sdk-main"))
    parser.add_argument("--local-ip", default=os.environ.get("MAGICBOT_LOCAL_IP", "192.168.54.119"))
    parser.add_argument("--real-disconnect", action="store_true")
    parser.add_argument(
        "--real-clean-exit",
        action="store_true",
        help="Run SDK cleanup. By default real-state mode hard-exits after writing summary to avoid known SDK disconnect/destructor crashes.",
    )
    parser.add_argument(
        "--real-forward-only",
        action="store_true",
        help="For real-state-sim: update MuJoCo kinematics with mj_forward at policy rate instead of 500Hz dynamics.",
    )
    return run_loop(parser.parse_args())


if __name__ == "__main__":
    raise SystemExit(main())
