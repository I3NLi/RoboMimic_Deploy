#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
PROJECT_ROOT="$( cd "${SCRIPT_DIR}/.." &> /dev/null && pwd )"
RUNNER="${SCRIPT_DIR}/run_magicbot_loco_native.sh"
RUNNER_SOURCE="${PROJECT_ROOT}/controller_cpp/src/magicbot_z1_loco_onnx.cpp"

duration="3.5"
udp_port=""
keep_log=0
extra_args=()

usage() {
    cat <<EOF
Usage: $0 [options] [-- extra magicbot_z1_loco_onnx args]

Smoke-test the real runner input path without connecting to a robot. The script
starts magicbot_z1_loco_onnx in --input-check mode, sends UDP text controls,
asserts that LOCO, PASSIVE, and FINAL_DAMPING are observed, and verifies DANCE
and SKILL are blocked by default unless the real runner is started with the
explicit gates.

Options:
  --duration S    Input-check duration, default ${duration}
  --udp-port N    UDP port, default: choose a free local port
  --keep-log      Print and keep the temp log path
  -h, --help      Show this help
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --duration)
            duration="$2"
            shift 2
            ;;
        --udp-port)
            udp_port="$2"
            shift 2
            ;;
        --keep-log)
            keep_log=1
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        --)
            shift
            extra_args=("$@")
            break
            ;;
        *)
            extra_args+=("$1")
            shift
            ;;
    esac
done

for tool in python3 ss grep; do
    if ! command -v "${tool}" >/dev/null 2>&1; then
        echo "[Smoke][ERROR] required tool not found: ${tool}" >&2
        exit 1
    fi
done

echo "[Smoke] Checking real input mode requests use shared text actions"
if grep -E -n 'set_live_input_mode_request\(out, ml::mode_request_for_control_mode' "${RUNNER_SOURCE}"; then
    echo "[Smoke][ERROR] real input mode requests must go through text_control_action_effect helpers" >&2
    exit 1
fi

echo "[Smoke] Checking real keyboard LOCO uses shared text action"
python3 - "${RUNNER_SOURCE}" <<'PY'
import re
import sys

source = open(sys.argv[1], encoding="utf-8").read()

def function_extent(name):
    marker = f"void {name}"
    start = source.find(marker)
    if start < 0:
        print(f"[Smoke][ERROR] could not locate {name}", file=sys.stderr)
        sys.exit(1)
    brace = source.find("{", start)
    depth = 0
    for index in range(brace, len(source)):
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
            if depth == 0:
                return start, index + 1
    print(f"[Smoke][ERROR] could not parse {name}", file=sys.stderr)
    sys.exit(1)

match = re.search(r"case 'l':\s*case 'L':(.*?)case 'b':", source, re.S)
if not match:
    print("[Smoke][ERROR] could not locate real keyboard L key block", file=sys.stderr)
    sys.exit(1)
block = match.group(1)
if "TextControlAction::ToggleLoco" not in block:
    print("[Smoke][ERROR] real keyboard L key must route through TextControlAction::ToggleLoco", file=sys.stderr)
    sys.exit(1)
if re.search(r"out\\.toggle_loco_requested\\s*=", block):
    print("[Smoke][ERROR] real keyboard L key must not set toggle_loco_requested directly", file=sys.stderr)
    sys.exit(1)

helper_start, helper_end = function_extent("apply_live_input_action_effect")
helper_body = source[helper_start:helper_end]
if "TextControlIntentState intent" not in helper_body:
    print("[Smoke][ERROR] real input helper must build a shared TextControlIntentState", file=sys.stderr)
    sys.exit(1)
if "apply_text_control_effect_to_intent(intent, effect)" not in helper_body:
    print("[Smoke][ERROR] real input helper must apply shared text-control intent semantics", file=sys.stderr)
    sys.exit(1)

for field in ("toggle_loco_requested", "reset_stand_requested"):
    for assignment in re.finditer(rf"out\\.{field}\\s*=", source):
        if not (helper_start <= assignment.start() < helper_end):
            print(
                f"[Smoke][ERROR] real input {field} must only be assigned in apply_live_input_action_effect",
                file=sys.stderr,
            )
            sys.exit(1)

if "mode_request_for_loco_toggle" not in source:
    print("[Smoke][ERROR] real runner must use shared mode_request_for_loco_toggle", file=sys.stderr)
    sys.exit(1)
if re.search(r"ControlMode::Loco\s*\?\s*ml::ControlMode::Stand", source):
    print("[Smoke][ERROR] real runner must not duplicate LOCO toggle mode mapping", file=sys.stderr)
    sys.exit(1)
if "Reset is intentionally local to policy/target state" not in source:
    print("[Smoke][ERROR] real runner reset must stay local and preserve the selected mode", file=sys.stderr)
    sys.exit(1)

reset_branch_start = source.find("if (input.reset_stand_requested)")
reset_branch_end = source.find("const std::string requested_external_policy_key", reset_branch_start)
if reset_branch_start < 0 or reset_branch_end < 0:
    print("[Smoke][ERROR] could not locate real runner reset branch", file=sys.stderr)
    sys.exit(1)
reset_branch = source[reset_branch_start:reset_branch_end]
if "state.snapshot().q" not in reset_branch:
    print("[Smoke][ERROR] real runner reset branch must seed from the current robot position", file=sys.stderr)
    sys.exit(1)
if "core.reset_policy()" not in reset_branch:
    print("[Smoke][ERROR] real runner reset branch must reset policy history", file=sys.stderr)
    sys.exit(1)
if re.search(r"run_mode\\s*=", reset_branch) or "input.mode_request" in reset_branch:
    print("[Smoke][ERROR] real runner reset branch must not change mode from input.mode_request", file=sys.stderr)
    sys.exit(1)
if "mode_request_for_control_mode(ml::ControlMode::Stand" in reset_branch:
    print("[Smoke][ERROR] real runner reset branch must not rebuild stand requests locally", file=sys.stderr)
    sys.exit(1)

udp_action_start, udp_action_end = function_extent("handle_action")
udp_action_body = source[udp_action_start:udp_action_end]
for local_check in ("effect.zero_command", "effect.pause", "effect.unpause"):
    if local_check in udp_action_body:
        print(
            "[Smoke][ERROR] UDP live input must not duplicate shared zero/pause/unpause semantics",
            file=sys.stderr,
        )
        sys.exit(1)
PY

if [[ -z "${udp_port}" ]]; then
    udp_port="$(python3 - <<'PY'
import socket

with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
    sock.bind(("127.0.0.1", 0))
    print(sock.getsockname()[1])
PY
)"
fi

log_path="$(mktemp /tmp/magicbot_loco_input_check_XXXXXX.log)"
runner_pid=""

cleanup() {
    if [[ -n "${runner_pid}" ]] && kill -0 "${runner_pid}" >/dev/null 2>&1; then
        kill "${runner_pid}" >/dev/null 2>&1 || true
        wait "${runner_pid}" >/dev/null 2>&1 || true
    fi
    if [[ "${keep_log}" -eq 0 ]]; then
        rm -f "${log_path}"
    fi
}
trap cleanup EXIT

echo "[Smoke] Starting real-runner input-check on UDP 127.0.0.1:${udp_port}"
"${RUNNER}" \
    --input-check \
    --udp-control \
    --udp-bind 127.0.0.1 \
    --udp-port "${udp_port}" \
    --duration "${duration}" \
    --log-interval 0.4 \
    "${extra_args[@]}" \
    >"${log_path}" 2>&1 &
runner_pid=$!

ready=0
for _ in $(seq 1 100); do
    if ss -lun | grep -E -q ":${udp_port}([[:space:]]|$)"; then
        ready=1
        break
    fi
    if ! kill -0 "${runner_pid}" >/dev/null 2>&1; then
        echo "[Smoke][ERROR] input-check exited before UDP was ready" >&2
        sed -n '1,200p' "${log_path}" >&2
        exit 1
    fi
    sleep 0.1
done

if [[ "${ready}" -ne 1 ]]; then
    echo "[Smoke][ERROR] timed out waiting for UDP port ${udp_port}" >&2
    sed -n '1,200p' "${log_path}" >&2
    exit 1
fi

python3 - <<PY
import socket
import time

port = int("${udp_port}")
packets = [
    b"walk",
    b"run_forward",
    b"vx=0.25 vy=-0.10 wz=0.05 mode=loco",
    b"pause",
    b"resume",
    b"mode=beyond",
    b"mode=track_mimic",
    b"mode=passive",
    b"mode=stand",
    b"mode=reset",
    b"mode=final_damping",
]

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
for packet in packets:
    sock.sendto(packet, ("127.0.0.1", port))
    time.sleep(0.25)
PY

if ! wait "${runner_pid}"; then
    echo "[Smoke][ERROR] input-check exited with failure" >&2
    sed -n '1,220p' "${log_path}" >&2
    exit 1
fi
runner_pid=""

for expected in 'mode=LOCO' 'cmd=\[0.25 0 0\]' 'cmd=\[0.65 0 0\]' 'pause-zero' 'mode=PASSIVE' ' reset' 'mode=FINAL_DAMPING'; do
    if ! grep -E -q "${expected}" "${log_path}"; then
        echo "[Smoke][ERROR] missing expected input-check output: ${expected}" >&2
        sed -n '1,220p' "${log_path}" >&2
        exit 1
    fi
done

if ! grep -E -q 'mode=STAND cmd=\[0 0 0\]$' "${log_path}"; then
    echo "[Smoke][ERROR] missing expected plain STAND output before reset" >&2
    sed -n '1,240p' "${log_path}" >&2
    exit 1
fi

if ! grep -E -q 'DANCE ignored; add --allow-dance' "${log_path}"; then
    echo "[Smoke][ERROR] expected DANCE request to be blocked without --allow-dance" >&2
    sed -n '1,220p' "${log_path}" >&2
    exit 1
fi
if ! grep -E -q 'SKILL ignored; add --allow-skill' "${log_path}"; then
    echo "[Smoke][ERROR] expected SKILL request to be blocked without --allow-skill" >&2
    sed -n '1,220p' "${log_path}" >&2
    exit 1
fi
if grep -E -q 'mode=DANCE' "${log_path}"; then
    echo "[Smoke][ERROR] DANCE mode was entered without --allow-dance" >&2
    sed -n '1,220p' "${log_path}" >&2
    exit 1
fi
if grep -E -q 'mode=SKILL' "${log_path}"; then
    echo "[Smoke][ERROR] SKILL mode was entered without --allow-skill" >&2
    sed -n '1,220p' "${log_path}" >&2
    exit 1
fi

echo "[Smoke] PASSED real-runner UDP input-check"
if [[ "${keep_log}" -eq 1 ]]; then
    echo "[Smoke] log=${log_path}"
fi
grep -E 'mode=(LOCO|PASSIVE|STAND|FINAL_DAMPING)|pause-zero| reset|DANCE ignored|SKILL ignored' "${log_path}"
