#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
PROJECT_ROOT="$( cd "${SCRIPT_DIR}/.." &> /dev/null && pwd )"
RUNNER="${SCRIPT_DIR}/run_magicbot_loco_native.sh"
RUNNER_SOURCE="${PROJECT_ROOT}/controller_cpp/src/magicbot_z1_loco_onnx.cpp"

duration="1.8"
keep_log=0
extra_args=()

usage() {
    cat <<EOF
Usage: $0 [options] [-- extra magicbot_z1_loco_onnx args]

Smoke-test the real runner Linux joystick input path without connecting to a
robot. The script starts magicbot_z1_loco_onnx in --input-check mode with a
FIFO-backed fake joystick, writes js_event packets, then validates logged mode
and command changes.

Options:
  --duration S    Input-check duration, default ${duration}
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

for tool in python3 rg mkfifo; do
    if ! command -v "${tool}" >/dev/null 2>&1; then
        echo "[Smoke][ERROR] required tool not found: ${tool}" >&2
        exit 1
    fi
done

echo "[Smoke] Checking real gamepad input stays an input adapter"
python3 - "${RUNNER_SOURCE}" <<'PY'
import re
import sys

source = open(sys.argv[1], encoding="utf-8").read()
start = source.find("class GamepadInput")
if start < 0:
    print("[Smoke][ERROR] real runner is missing GamepadInput", file=sys.stderr)
    sys.exit(1)
end = source.find("class UdpCommandInput", start)
if end < 0:
    print("[Smoke][ERROR] could not locate GamepadInput boundary", file=sys.stderr)
    sys.exit(1)
body = source[start:end]
required = [
    "set_live_input_action_request",
    "TextControlAction::Loco",
    "TextControlAction::Passive",
    "TextControlAction::Stand",
    "TextControlAction::Pause",
    "TextControlAction::ResetStand",
    "TextControlAction::Dance",
    "TextControlAction::Skill",
]
missing = [item for item in required if item not in body]
if missing:
    print("[Smoke][ERROR] real gamepad must route buttons through shared text actions; missing: " + ", ".join(missing), file=sys.stderr)
    sys.exit(1)
for forbidden in (
    "ControllerCore",
    "ControllerRuntime",
    "ModeManager",
    "mode_request_for_control_mode",
    "mode_request_for_desired_control_mode",
):
    if forbidden in body:
        print(f"[Smoke][ERROR] real gamepad must not own control/runtime logic: {forbidden}", file=sys.stderr)
        sys.exit(1)
if re.search(r"out\\.mode_request\\s*=", body):
    print("[Smoke][ERROR] real gamepad must not set mode_request directly", file=sys.stderr)
    sys.exit(1)
if "command_for_control_mode" not in source:
    print("[Smoke][ERROR] real runner must sanitize live commands by current mode through the shared helper", file=sys.stderr)
    sys.exit(1)
PY

tmp_dir="$(mktemp -d /tmp/magicbot_real_gamepad_XXXXXX)"
gamepad_fifo="${tmp_dir}/js0"
mkfifo "${gamepad_fifo}"
log_path="$(mktemp /tmp/magicbot_loco_gamepad_input_XXXXXX.log)"
runner_pid=""

cleanup() {
    if [[ -n "${runner_pid}" ]] && kill -0 "${runner_pid}" >/dev/null 2>&1; then
        kill "${runner_pid}" >/dev/null 2>&1 || true
        wait "${runner_pid}" >/dev/null 2>&1 || true
    fi
    rm -rf "${tmp_dir}"
    if [[ "${keep_log}" -eq 0 ]]; then
        rm -f "${log_path}"
    fi
}
trap cleanup EXIT

echo "[Smoke] Starting real-runner gamepad input-check with fake device ${gamepad_fifo}"
"${RUNNER}" \
    --input-check \
    --gamepad-control \
    --gamepad-device "${gamepad_fifo}" \
    --duration "${duration}" \
    --log-interval 0.2 \
    --input-deadzone 0 \
    "${extra_args[@]}" \
    >"${log_path}" 2>&1 &
runner_pid=$!

ready=0
for _ in $(seq 1 360); do
    if rg -q '\[Input\] Gamepad control enabled|\[InputCheck\] No robot connection' "${log_path}"; then
        ready=1
        break
    fi
    if ! kill -0 "${runner_pid}" >/dev/null 2>&1; then
        echo "[Smoke][ERROR] input-check exited before gamepad became ready" >&2
        sed -n '1,220p' "${log_path}" >&2
        exit 1
    fi
    sleep 0.1
done

if [[ "${ready}" -ne 1 ]]; then
    echo "[Smoke][ERROR] timed out waiting for gamepad input-check readiness" >&2
    sed -n '1,220p' "${log_path}" >&2
    exit 1
fi

send_gamepad_events() {
    local sequence="$1"
    GAMEPAD_FIFO="${gamepad_fifo}" SEQUENCE="${sequence}" python3 - <<'PY'
import os
import struct
import time

JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
path = os.environ["GAMEPAD_FIFO"]
sequence = os.environ["SEQUENCE"]

def event(value: int, kind: int, number: int) -> bytes:
    now_ms = int(time.monotonic() * 1000) & 0xFFFFFFFF
    return struct.pack("<IhBB", now_ms, value, kind, number)

if sequence == "axis_only":
    packets = [
        event(1, JS_EVENT_BUTTON, 4),       # deadman held
        event(-16384, JS_EVENT_AXIS, 1),    # default vx axis with sign -1 -> +0.5
        event(0, JS_EVENT_AXIS, 0),
        event(0, JS_EVENT_AXIS, 3),
    ]
elif sequence == "loco_axis":
    packets = [
        event(1, JS_EVENT_BUTTON, 4),       # deadman held
        event(1, JS_EVENT_BUTTON, 0),       # LOCO button
        event(-16384, JS_EVENT_AXIS, 1),    # default vx axis with sign -1 -> +0.5
        event(0, JS_EVENT_AXIS, 0),
        event(0, JS_EVENT_AXIS, 3),
    ]
elif sequence == "pause":
    packets = [event(1, JS_EVENT_BUTTON, 7)]
else:
    raise SystemExit(f"unknown sequence: {sequence}")

with open(path, "wb", buffering=0) as fifo:
    for packet in packets:
        fifo.write(packet)
        time.sleep(0.02)
PY
}

send_gamepad_events "axis_only"

stand_zero_ready=0
for _ in $(seq 1 80); do
    if rg -q 'gamepad mode=STAND cmd=\[-?0 -?0 -?0\]$' "${log_path}"; then
        stand_zero_ready=1
        break
    fi
    if ! kill -0 "${runner_pid}" >/dev/null 2>&1; then
        echo "[Smoke][ERROR] input-check exited before STAND axis-zero was observed" >&2
        sed -n '1,260p' "${log_path}" >&2
        exit 1
    fi
    sleep 0.1
done

if [[ "${stand_zero_ready}" -ne 1 ]]; then
    echo "[Smoke][ERROR] fake gamepad axis should stay zero while mode is STAND" >&2
    sed -n '1,260p' "${log_path}" >&2
    exit 1
fi

send_gamepad_events "loco_axis"

loco_ready=0
for _ in $(seq 1 80); do
    if rg -q 'gamepad mode=LOCO cmd=\[0\.5 -?0 -?0\]' "${log_path}"; then
        loco_ready=1
        break
    fi
    if ! kill -0 "${runner_pid}" >/dev/null 2>&1; then
        echo "[Smoke][ERROR] input-check exited before gamepad LOCO command was observed" >&2
        sed -n '1,260p' "${log_path}" >&2
        exit 1
    fi
    sleep 0.1
done

if [[ "${loco_ready}" -ne 1 ]]; then
    echo "[Smoke][ERROR] missing fake gamepad LOCO axis command" >&2
    sed -n '1,260p' "${log_path}" >&2
    exit 1
fi

send_gamepad_events "pause"

pause_ready=0
for _ in $(seq 1 80); do
    if rg -q 'gamepad paused mode=LOCO cmd=\[-?0 -?0 -?0\].*pause-zero' "${log_path}"; then
        pause_ready=1
        break
    fi
    if ! kill -0 "${runner_pid}" >/dev/null 2>&1; then
        echo "[Smoke][ERROR] input-check exited before gamepad pause-zero was observed" >&2
        sed -n '1,280p' "${log_path}" >&2
        exit 1
    fi
    sleep 0.1
done

if [[ "${pause_ready}" -ne 1 ]]; then
    echo "[Smoke][ERROR] missing fake gamepad pause-zero output" >&2
    sed -n '1,280p' "${log_path}" >&2
    exit 1
fi

if ! wait "${runner_pid}"; then
    echo "[Smoke][ERROR] input-check exited with failure" >&2
    sed -n '1,320p' "${log_path}" >&2
    exit 1
fi
runner_pid=""

echo "[Smoke] PASSED real-runner gamepad input-check"
if [[ "${keep_log}" -eq 1 ]]; then
    echo "[Smoke] log=${log_path}"
fi
rg 'gamepad.*mode=(STAND|LOCO)|pause-zero' "${log_path}"
