#!/usr/bin/env python3
"""Python entrypoint for the shared-runtime MagicBot MuJoCo viewer.

This keeps a Python-facing operator command while delegating simulation and
control to the native viewer, so mode, policy, safety, and target limiting stay
inside the shared ControllerCore path.
"""

from __future__ import annotations

import argparse
import shlex
import subprocess
import sys
from pathlib import Path


def parse_args(argv: list[str]) -> tuple[argparse.Namespace, list[str]]:
    parser = argparse.ArgumentParser(
        description=(
            "Launch the MagicBot MuJoCo viewer through the shared native "
            "ControllerCore runtime. All unknown options are forwarded to "
            "run_mujoco_loco_viewer_native.sh."
        )
    )
    parser.add_argument(
        "--viewer-runner",
        type=Path,
        default=None,
        help="Path to run_mujoco_loco_viewer_native.sh, default: repo scripts directory",
    )
    parser.add_argument(
        "--print-command",
        action="store_true",
        help="Print the resolved native viewer command and exit",
    )
    return parser.parse_known_args(argv)


def main(argv: list[str]) -> int:
    args, viewer_args = parse_args(argv)
    repo_root = Path(__file__).resolve().parents[1]
    runner = args.viewer_runner or repo_root / "scripts" / "run_mujoco_loco_viewer_native.sh"
    runner = runner.expanduser().resolve()
    if not runner.is_file():
        print(f"[PythonViewer][ERROR] viewer runner not found: {runner}", file=sys.stderr)
        return 1

    command = [str(runner), *viewer_args]
    if args.print_command:
        print(shlex.join(command))
        return 0

    completed = subprocess.run(command, check=False)
    return completed.returncode


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
