#!/usr/bin/env python3
"""MagicBot Z1 SDK backend for the RoboMimic loco policy.

This entry keeps the RoboMimic policy/config layout, but uses the official
MagicBot Z1 SDK low-level Python binding instead of the legacy Unitree DDS
transport used by deploy_real.py.

Safe modes:
  --dry-run        Load YAML/ONNX and run one inference. No robot connection.
  --connect-check  Connect/disconnect only. No LowLevel switch.
  --read-state     Switch LowLevel, subscribe state callbacks, no JointCommand.

Motion modes are intentionally explicit:
  --run --stand-only --duration N
  --run --duration N
"""

from __future__ import annotations

import argparse
import importlib.util
import logging
import os
import sys
import time
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG = PROJECT_ROOT / "policies" / "loco_mode" / "config" / "LocoMode_lowKp.yaml"


def candidate_sdk_roots() -> list[Path]:
    roots: list[Path] = []
    env_root = os.environ.get("MAGICBOT_SDK_ROOT", "").strip()
    if env_root:
        roots.append(Path(env_root).expanduser())
    home = Path.home()
    roots.extend(
        [
            home / "magicbot-z1_sdk-main",
            home / "MaigcLab" / "magicbot-z1_sdk-main",
            Path("/home/eame/magicbot-z1_sdk-main"),
            Path("/home/hiyio/MaigcLab/magicbot-z1_sdk-main"),
        ]
    )
    deduped: list[Path] = []
    seen: set[str] = set()
    for root in roots:
        key = str(root)
        if key not in seen:
            seen.add(key)
            deduped.append(root)
    return deduped


def resolve_sdk_root(raw: str | None) -> Path:
    candidates = [Path(raw).expanduser()] if raw else candidate_sdk_roots()
    for root in candidates:
        deploy = root / "deploy_onnx" / "z1_loco_onnx_deploy.py"
        if deploy.is_file():
            return root.resolve()
    tried = "\n  ".join(str(p) for p in candidates)
    raise FileNotFoundError(f"Could not find magicbot-z1_sdk-main. Tried:\n  {tried}")


def load_sdk_deploy_module(sdk_root: Path):
    module_path = sdk_root / "deploy_onnx" / "z1_loco_onnx_deploy.py"
    spec = importlib.util.spec_from_file_location("magicbot_z1_loco_onnx_deploy", module_path)
    if spec is None or spec.loader is None:
        raise ImportError(f"Failed to load {module_path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def make_sdk_args(args: argparse.Namespace) -> argparse.Namespace:
    return argparse.Namespace(
        config=args.config,
        dry_run=args.dry_run,
        connect_check=args.connect_check,
        run=args.run,
        local_ip=args.local_ip,
        skip_network_check=args.skip_network_check,
        vx=args.vx,
        vy=args.vy,
        wz=args.wz,
        state_timeout=args.state_timeout,
        prepare_gait=args.prepare_gait,
        stand_time=args.stand_time,
        stand_only=args.stand_only,
        duration=args.duration,
        stand_kp_scale=args.stand_kp_scale,
        kp_scale=args.kp_scale,
        kd_scale=args.kd_scale,
        max_target_rate=args.max_target_rate,
        joint_limit_margin=args.joint_limit_margin,
        damping_kd=args.damping_kd,
        log_interval=args.log_interval,
    )


def read_state_only(args: argparse.Namespace, sdk, cfg, magicbot) -> int:
    sdk.check_network_preflight(args.local_ip, args.skip_network_check)
    sdk.warn_existing_deploy_processes()

    robot_state = sdk.RobotState()
    robot = magicbot.MagicRobot()
    controller = None
    try:
        logging.info("SDK model: %s", magicbot.get_robot_model())
        logging.info("Initializing SDK with local_ip=%s", args.local_ip)
        if not robot.initialize(args.local_ip):
            raise RuntimeError(f"Failed to initialize SDK with local IP {args.local_ip}")
        status = robot.connect()
        if status.code != magicbot.ErrorCode.OK:
            raise RuntimeError(f"Failed to connect robot: {status.code} {status.message}")

        sdk.prepare_gait_for_low_level(magicbot, robot, args.prepare_gait)

        status = robot.set_motion_control_level(magicbot.ControllerLevel.LowLevel)
        if status.code != magicbot.ErrorCode.OK:
            raise RuntimeError(f"Failed to switch LowLevel: {status.code} {status.message}")
        logging.info("Switched to LowLevel. Subscribing state only; no JointCommand will be published.")

        controller = robot.get_low_level_motion_controller()
        if hasattr(controller, "initialize"):
            logging.info("LowLevel controller initialize: %s", controller.initialize())

        controller.subscribe_leg_state(robot_state.update_leg)
        controller.subscribe_arm_state(robot_state.update_arm)
        controller.subscribe_waist_state(robot_state.update_waist)
        controller.subscribe_head_state(robot_state.update_head)
        controller.subscribe_body_imu(robot_state.update_imu)

        sdk.wait_for_state(robot_state, args.state_timeout)

        start = time.perf_counter()
        last_log = 0.0
        while time.perf_counter() - start < args.duration:
            now = time.perf_counter()
            if now - last_log >= args.log_interval:
                q, dq, quat, ang_vel, counts = robot_state.snapshot()
                logging.info(
                    "state counts=%s q=[%.4f..%.4f] dq=[%.4f..%.4f] quat=[%.3f %.3f %.3f %.3f]",
                    counts,
                    float(q.min()),
                    float(q.max()),
                    float(dq.min()),
                    float(dq.max()),
                    float(quat[0]),
                    float(quat[1]),
                    float(quat[2]),
                    float(quat[3]),
                )
                last_log = now
            time.sleep(0.01)
        logging.info("Read-state duration reached. No commands were published.")
        return 0
    finally:
        try:
            if controller is not None and hasattr(controller, "shutdown"):
                controller.shutdown()
        except Exception as exc:
            logging.warning("LowLevel controller shutdown failed: %s", exc)
        try:
            robot.disconnect()
        except Exception:
            pass
        try:
            robot.shutdown()
        except Exception:
            pass


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sdk-root", default=None, help="Path to magicbot-z1_sdk-main")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--dry-run", action="store_true", help="validate config/model without robot connection")
    parser.add_argument("--connect-check", action="store_true", help="connect/disconnect only; no LowLevel switch")
    parser.add_argument("--read-state", action="store_true", help="LowLevel state subscription test; no command publishing")
    parser.add_argument("--run", action="store_true", help="publish low-level commands. Requires explicit --stand-only or ONNX duration.")
    parser.add_argument("--local-ip", default=os.environ.get("MAGICBOT_LOCAL_IP", "192.168.54.119"))
    parser.add_argument("--skip-network-check", action="store_true")
    parser.add_argument("--vx", type=float, default=0.0)
    parser.add_argument("--vy", type=float, default=0.0)
    parser.add_argument("--wz", type=float, default=0.0)
    parser.add_argument("--state-timeout", type=float, default=10.0)
    parser.add_argument(
        "--prepare-gait",
        default="recovery_stand",
        choices=("none", "passive", "recovery_stand", "recovery"),
        help="gait to prepare before LowLevel modes; use none only if already in a valid gait",
    )
    parser.add_argument("--stand-time", type=float, default=2.0)
    parser.add_argument("--stand-only", action="store_true")
    parser.add_argument("--duration", type=float, default=3.0)
    parser.add_argument("--stand-kp-scale", type=float, default=0.5)
    parser.add_argument("--kp-scale", type=float, default=1.0)
    parser.add_argument("--kd-scale", type=float, default=1.0)
    parser.add_argument("--max-target-rate", type=float, default=4.0)
    parser.add_argument("--joint-limit-margin", type=float, default=0.01)
    parser.add_argument("--damping-kd", type=float, default=3.0)
    parser.add_argument("--log-interval", type=float, default=1.0)
    return parser.parse_args()


def main() -> int:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )
    args = parse_args()
    selected = int(args.dry_run) + int(args.connect_check) + int(args.read_state) + int(args.run)
    if selected == 0:
        args.dry_run = True
    elif selected > 1:
        raise SystemExit("Use only one of --dry-run, --connect-check, --read-state or --run")
    if args.run and args.duration <= 0:
        raise SystemExit("--run requires a positive --duration for this RoboMimic wrapper")

    sdk_root = resolve_sdk_root(args.sdk_root)
    logging.info("Using MagicBot SDK root: %s", sdk_root)
    sdk = load_sdk_deploy_module(sdk_root)
    magicbot = sdk.import_magicbot_sdk()
    cfg = sdk.load_loco_config(args.config.resolve())
    policy = sdk.OnnxLocoPolicy(cfg)

    sdk_args = make_sdk_args(args)
    if args.dry_run:
        sdk.dry_run(cfg, policy, magicbot)
        return 0
    if args.connect_check:
        return sdk.connect_check(sdk_args, magicbot)
    if args.read_state:
        return read_state_only(sdk_args, sdk, cfg, magicbot)
    return sdk.run_robot(sdk_args, cfg, policy, magicbot)


if __name__ == "__main__":
    raise SystemExit(main())
