#!/usr/bin/env python3
"""
Quick sync for model artifacts used by RoboMimic_Deploy.

What it does:
- Find the latest ONNX under whole_body_tracking logs and copy it into
  policy/beyond_mimic/model, then update BeyondMimic.yaml.
- Optionally run a headless LeggedLab export to generate policy.pt.
- Find the latest exported policy.pt under LeggedLab logs and copy it into
  policy/loco_mode/model, then update LocoMode_lowKp.yaml.

Defaults are wired to the paths in /home/hiyio. Use CLI args to override.
"""

from __future__ import annotations

import argparse
import shutil
import subprocess
from pathlib import Path


def find_latest_onnx(root: Path) -> Path:
    candidates = list(root.rglob("*.onnx"))
    if not candidates:
        raise FileNotFoundError(f"No .onnx found under: {root}")
    # Prefer training-run ONNX (not exported/policy.onnx).
    preferred = [
        p for p in candidates if "exported" not in p.parts and p.name != "policy.onnx"
    ]
    if preferred:
        candidates = preferred
    return max(candidates, key=lambda p: p.stat().st_mtime)


def find_latest_policy_pt(root: Path) -> Path:
    candidates = list(root.rglob("exported/policy.pt"))
    if not candidates:
        raise FileNotFoundError(f"No exported policy.pt found under: {root}")
    return max(candidates, key=lambda p: p.stat().st_mtime)


def copy_if_different(src: Path, dst: Path) -> None:
    if dst.exists():
        try:
            src_stat = src.stat()
            dst_stat = dst.stat()
            if src_stat.st_size == dst_stat.st_size and src_stat.st_mtime <= dst_stat.st_mtime:
                return
        except OSError:
            pass
    dst.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(src, dst)


def update_yaml_key(path: Path, key: str, value: str, comment: str | None = None) -> None:
    lines = path.read_text(encoding="utf-8").splitlines()
    for idx, line in enumerate(lines):
        if line.lstrip().startswith(f"{key}:"):
            # Insert a one-line comment above the key when requested.
            if comment:
                comment_line = f"# {comment}"
                if idx == 0 or lines[idx - 1].strip() != comment_line:
                    lines.insert(idx, comment_line)
                    idx += 1
            indent = line[: len(line) - len(line.lstrip())]
            lines[idx] = f'{indent}{key}: "{value}"'
            path.write_text("\n".join(lines) + "\n", encoding="utf-8")
            return
    raise KeyError(f"Key '{key}' not found in {path}")


def export_leggedlab(
    play_py: Path,
    task: str,
    headless: bool,
    load_run: str | None,
    load_checkpoint: str | None,
    num_envs: int | None,
    python_bin: str,
) -> None:
    cmd = [str(play_py), f"--task={task}", "--export_only"]
    if headless:
        cmd.append("--headless")
    if load_run:
        cmd.append(f"--load_run={load_run}")
    if load_checkpoint:
        cmd.append(f"--load_checkpoint={load_checkpoint}")
    if num_envs is not None:
        cmd.append(f"--num_envs={num_envs}")
    subprocess.run([python_bin, *cmd], check=True)


def main() -> int:
    parser = argparse.ArgumentParser(description="Sync latest ONNX and policy.pt into RoboMimic_Deploy.")
    parser.add_argument(
        "--onnx-root",
        type=Path,
        default=Path("/home/hiyio/whole_body_tracking/logs"),
        help="Root directory to search for ONNX files.",
    )
    parser.add_argument(
        "--beyond-model-dir",
        type=Path,
        default=Path("/home/hiyio/RoboMimic_Deploy/policy/beyond_mimic/model"),
        help="Destination directory for BeyondMimic ONNX models.",
    )
    parser.add_argument(
        "--beyond-config",
        type=Path,
        default=Path("/home/hiyio/RoboMimic_Deploy/policy/beyond_mimic/config/BeyondMimic.yaml"),
        help="BeyondMimic.yaml to update.",
    )
    parser.add_argument(
        "--leggedlab-logs",
        type=Path,
        default=Path("/home/hiyio/LeggedLab/logs/g1_flat"),
        help="Root directory to search for exported policy.pt from LeggedLab.",
    )
    parser.add_argument(
        "--loco-model-dir",
        type=Path,
        default=Path("/home/hiyio/RoboMimic_Deploy/policy/loco_mode/model"),
        help="Destination directory for LocoMode policy.pt.",
    )
    parser.add_argument(
        "--loco-config",
        type=Path,
        default=Path("/home/hiyio/RoboMimic_Deploy/policy/loco_mode/config/LocoMode_lowKp.yaml"),
        help="LocoMode_lowKp.yaml to update.",
    )
    parser.add_argument(
        "--export-leggedlab",
        action="store_true",
        help="Run LeggedLab play.py headlessly to export policy artifacts before syncing.",
    )
    parser.add_argument(
        "--leggedlab-play",
        type=Path,
        default=Path("/home/hiyio/LeggedLab/legged_lab/scripts/play.py"),
        help="Path to LeggedLab play.py.",
    )
    parser.add_argument("--task", type=str, default="g1_flat", help="LeggedLab task name.")
    parser.add_argument(
        "--headless", action="store_true", default=True, help="Run export headlessly (default)."
    )
    parser.add_argument(
        "--no-headless", dest="headless", action="store_false", help="Disable headless mode."
    )
    parser.add_argument("--leggedlab-load-run", type=str, default=None, help="Optional --load_run for play.py.")
    parser.add_argument(
        "--leggedlab-load-checkpoint", type=str, default=None, help="Optional --load_checkpoint for play.py."
    )
    parser.add_argument("--leggedlab-num-envs", type=int, default=1, help="Optional --num_envs for play.py.")
    parser.add_argument(
        "--python",
        type=str,
        default="python",
        help="Python executable to run LeggedLab export (use conda python if needed).",
    )
    parser.add_argument(
        "--deploy-mujoco",
        dest="deploy_mujoco",
        action="store_true",
        default=True,
        help="Launch deploy_mujoco after syncing (default).",
    )
    parser.add_argument(
        "--no-deploy-mujoco",
        dest="deploy_mujoco",
        action="store_false",
        help="Skip launching deploy_mujoco after syncing.",
    )
    parser.add_argument(
        "--deploy-script",
        type=Path,
        default=Path("/home/hiyio/RoboMimic_Deploy/deploy_mujoco/deploy_mujoco.py"),
        help="Path to deploy_mujoco.py.",
    )
    parser.add_argument(
        "--deploy-python",
        type=str,
        default="python",
        help="Python executable to run deploy_mujoco (use conda python if needed).",
    )
    parser.add_argument("--dry-run", action="store_true", help="Print actions without copying/updating.")

    args = parser.parse_args()

    if args.export_leggedlab:
        export_leggedlab(
            play_py=args.leggedlab_play,
            task=args.task,
            headless=args.headless,
            load_run=args.leggedlab_load_run,
            load_checkpoint=args.leggedlab_load_checkpoint,
            num_envs=args.leggedlab_num_envs,
            python_bin=args.python,
        )

    latest_onnx = find_latest_onnx(args.onnx_root)
    onnx_name = latest_onnx.name
    beyond_dest = args.beyond_model_dir / onnx_name

    latest_policy = find_latest_policy_pt(args.leggedlab_logs)
    run_name = latest_policy.parents[1].name  # logs/g1_flat/<run>/exported/policy.pt
    policy_name = f"{run_name}_policy.pt"
    policy_dest = args.loco_model_dir / policy_name
    policy_default = args.loco_model_dir / "policy.pt"

    if args.dry_run:
        print(f"[DRY-RUN] Latest ONNX: {latest_onnx}")
        print(f"[DRY-RUN] Copy ONNX -> {beyond_dest}")
        print(f"[DRY-RUN] Update BeyondMimic.yaml onnx_path -> {onnx_name}")
        print(f"[DRY-RUN] Latest policy.pt: {latest_policy}")
        print(f"[DRY-RUN] Copy policy -> {policy_dest}")
        print(f"[DRY-RUN] Refresh policy.pt -> {policy_default}")
        print(f"[DRY-RUN] Update LocoMode_lowKp.yaml policy_path -> {policy_name}")
        print(f"[DRY-RUN] Launch deploy_mujoco -> {args.deploy_script}")
        return 0

    copy_if_different(latest_onnx, beyond_dest)
    update_yaml_key(
        args.beyond_config,
        "onnx_path",
        onnx_name,
        comment="Auto-updated by scripts/quick_sync_models.py (latest ONNX).",
    )

    copy_if_different(latest_policy, policy_dest)
    # Keep a stable filename for convenience while still tracking the run name.
    copy_if_different(latest_policy, policy_default)
    update_yaml_key(
        args.loco_config,
        "policy_path",
        policy_name,
        comment="Auto-updated by scripts/quick_sync_models.py (latest policy.pt).",
    )

    print(f"[OK] BeyondMimic ONNX synced: {beyond_dest}")
    print(f"[OK] LocoMode policy synced: {policy_dest}")
    if args.deploy_mujoco:
        # Launch deploy after syncing so it always picks up the newest models.
        subprocess.run([args.deploy_python, str(args.deploy_script)], check=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
