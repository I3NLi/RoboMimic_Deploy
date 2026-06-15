#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
PYTHON_VIEWER="${SCRIPT_DIR}/run_python_mujoco_viewer.py"

usage() {
    cat <<EOF
Usage: $0 [options]

Smoke-test the Python-facing MuJoCo viewer entrypoint. The script verifies that
the Python command delegates to the native shared-runtime viewer, then runs the
same no-hardware viewer stream/control/gamepad/perturb smokes through that
Python launcher.

Options:
  --python-viewer P   Python viewer entrypoint, default ${PYTHON_VIEWER}
  -h, --help          Show this help
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --python-viewer)
            PYTHON_VIEWER="$2"
            shift 2
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "[Smoke][ERROR] unknown argument: $1" >&2
            usage >&2
            exit 1
            ;;
    esac
done

run_step() {
    echo
    echo "[Smoke] $*"
    "$@"
}

check_python_viewer_delegates() {
    python3 - "${PYTHON_VIEWER}" <<'PY'
import ast
import sys
from pathlib import Path

path = Path(sys.argv[1])
tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
forbidden_imports = {
    "mujoco",
    "onnx",
    "onnxruntime",
    "torch",
    "numpy",
    "rospy",
    "rclpy",
}

imports = []
uses_subprocess_run = False
for node in ast.walk(tree):
    if isinstance(node, ast.Import):
        imports.extend(alias.name.split(".", 1)[0] for alias in node.names)
    elif isinstance(node, ast.ImportFrom) and node.module:
        imports.append(node.module.split(".", 1)[0])
    elif isinstance(node, ast.Call):
        func = node.func
        if (
            isinstance(func, ast.Attribute)
            and func.attr == "run"
            and isinstance(func.value, ast.Name)
            and func.value.id == "subprocess"
        ):
            uses_subprocess_run = True

bad_imports = sorted({name for name in imports if name in forbidden_imports})
if bad_imports:
    raise SystemExit(
        "[Smoke][ERROR] Python viewer must delegate to native runtime; "
        f"forbidden imports: {', '.join(bad_imports)}"
    )
if not uses_subprocess_run:
    raise SystemExit(
        "[Smoke][ERROR] Python viewer should launch the native viewer runner "
        "through subprocess.run"
    )
PY
}

check_python_viewer_print_command() {
    local output
    output="$("${PYTHON_VIEWER}" --print-command --duration 0.1 --no-realtime)"
    if [[ "${output}" != *"run_mujoco_loco_viewer_native.sh"* ||
          "${output}" != *"--duration 0.1"* ||
          "${output}" != *"--no-realtime"* ]]; then
        echo "[Smoke][ERROR] Python viewer --print-command did not preserve native runner forwarding" >&2
        echo "${output}" >&2
        exit 1
    fi
}

run_step check_python_viewer_delegates
run_step check_python_viewer_print_command
run_step python3 -m py_compile "${PYTHON_VIEWER}"
run_step "${SCRIPT_DIR}/run_viewer_stream_smoke_native.sh" --runner "${PYTHON_VIEWER}" --duration 1.5
run_step "${SCRIPT_DIR}/run_viewer_http_control_smoke_native.sh" --runner "${PYTHON_VIEWER}" --duration 1.5
run_step "${SCRIPT_DIR}/run_viewer_udp_control_smoke_native.sh" --runner "${PYTHON_VIEWER}" --duration 1.5
run_step "${SCRIPT_DIR}/run_viewer_http_dance_smoke_native.sh" --runner "${PYTHON_VIEWER}" --duration 0.8
run_step "${SCRIPT_DIR}/run_viewer_http_skill_smoke_native.sh" --runner "${PYTHON_VIEWER}" --duration 0.8
run_step "${SCRIPT_DIR}/run_viewer_udp_external_policy_smoke_native.sh" --runner "${PYTHON_VIEWER}" --duration 1.8
run_step "${SCRIPT_DIR}/run_viewer_gamepad_control_smoke_native.sh" --runner "${PYTHON_VIEWER}" --duration 1.5
run_step "${SCRIPT_DIR}/run_viewer_control_station_smoke_native.sh" --runner "${PYTHON_VIEWER}" --duration 2.0
run_step "${SCRIPT_DIR}/run_viewer_http_perturb_smoke_native.sh" --runner "${PYTHON_VIEWER}" --duration 1.5

echo
echo "[Smoke] PASSED Python-facing viewer shared-runtime smoke"
