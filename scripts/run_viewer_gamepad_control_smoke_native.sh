#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
PROJECT_ROOT="$( cd "${SCRIPT_DIR}/.." &> /dev/null && pwd )"
RUNNER="${SCRIPT_DIR}/run_mujoco_loco_viewer_native.sh"
VIEWER_SOURCE="${PROJECT_ROOT}/controller_cpp/src/mujoco_loco_viewer.cpp"

duration="1.5"
camera_port=""
summary_json=""
keep_summary=0
extra_args=()

usage() {
    cat <<EOF
Usage: $0 [options] [-- extra viewer runner args]

Smoke-test the MuJoCo viewer Linux joystick input path without real hardware.
The script starts the viewer with --gamepad-control pointed at a FIFO, writes
fake js_event packets, then validates live /status and the JSON summary.

Options:
  --duration S       Viewer wall-clock duration, default ${duration}
  --runner P         Viewer runner, default ${RUNNER}
  --camera-port N    HTTP status port, default: choose a free local port
  --summary-json P   Summary JSON path, default: temp file under /tmp
  --keep-summary     Keep the temp summary path printed at the end
  -h, --help         Show this help
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --duration)
            duration="$2"
            shift 2
            ;;
        --runner)
            RUNNER="$2"
            shift 2
            ;;
        --camera-port)
            camera_port="$2"
            shift 2
            ;;
        --summary-json)
            summary_json="$2"
            keep_summary=1
            shift 2
            ;;
        --keep-summary)
            keep_summary=1
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

for tool in curl jq python3 rg mkfifo; do
    if ! command -v "${tool}" >/dev/null 2>&1; then
        echo "[Smoke][ERROR] required tool not found: ${tool}" >&2
        exit 1
    fi
done

echo "[Smoke] Checking viewer gamepad input stays an input adapter"
python3 - "${VIEWER_SOURCE}" <<'PY'
import re
import sys

source = open(sys.argv[1], encoding="utf-8").read()
start = source.find("class ViewerGamepadInput")
if start < 0:
    print("[Smoke][ERROR] viewer is missing ViewerGamepadInput", file=sys.stderr)
    sys.exit(1)
open_brace = source.find("{", start)
if open_brace < 0:
    print("[Smoke][ERROR] could not locate ViewerGamepadInput body", file=sys.stderr)
    sys.exit(1)
depth = 0
end = -1
for idx in range(open_brace, len(source)):
    char = source[idx]
    if char == "{":
        depth += 1
    elif char == "}":
        depth -= 1
        if depth == 0:
            end = idx + 1
            break
if end < 0:
    print("[Smoke][ERROR] could not locate ViewerGamepadInput boundary", file=sys.stderr)
    sys.exit(1)
body = source[start:end]
required = [
    "apply_viewer_text_action",
    "TextControlAction::Loco",
    "TextControlAction::Stand",
    "TextControlAction::Passive",
    "TextControlAction::Zero",
    "TextControlAction::Pause",
    "TextControlAction::ResetStand",
    "TextControlAction::Dance",
    "TextControlAction::Skill",
    "TextControlSafetyCommand::Toggle",
]
missing = [item for item in required if item not in body]
if missing:
    print("[Smoke][ERROR] viewer gamepad must route buttons through shared text actions; missing: " + ", ".join(missing), file=sys.stderr)
    sys.exit(1)
for forbidden in (
    "ControllerCore",
    "ControllerRuntime",
    "ModeManager",
    "mode_request_for_control_mode",
    "mode_request_for_desired_control_mode",
):
    if forbidden in body:
        print(f"[Smoke][ERROR] viewer gamepad must not own control/runtime logic: {forbidden}", file=sys.stderr)
        sys.exit(1)
if re.search(r"desired_mode\s*=(?!=)", body):
    print("[Smoke][ERROR] viewer gamepad must not set desired_mode directly", file=sys.stderr)
    sys.exit(1)
PY

if [[ -z "${camera_port}" ]]; then
    camera_port="$(python3 - <<'PY'
import socket

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
    sock.bind(("127.0.0.1", 0))
    print(sock.getsockname()[1])
PY
)"
fi

if [[ -z "${summary_json}" ]]; then
    summary_json="$(mktemp /tmp/magicbot_viewer_gamepad_XXXXXX.json)"
fi

tmp_dir="$(mktemp -d /tmp/magicbot_viewer_gamepad_XXXXXX)"
gamepad_fifo="${tmp_dir}/js0"
mkfifo "${gamepad_fifo}"
viewer_log="$(mktemp /tmp/magicbot_viewer_gamepad_XXXXXX.log)"
status_body="$(mktemp /tmp/magicbot_viewer_gamepad_status_XXXXXX.json)"
viewer_pid=""

cleanup() {
    if [[ -n "${viewer_pid}" ]] && kill -0 "${viewer_pid}" >/dev/null 2>&1; then
        kill "${viewer_pid}" >/dev/null 2>&1 || true
        wait "${viewer_pid}" >/dev/null 2>&1 || true
    fi
    rm -f "${status_body}" "${viewer_log}"
    rm -rf "${tmp_dir}"
    if [[ "${keep_summary}" -eq 0 ]]; then
        rm -f "${summary_json}"
    fi
}
trap cleanup EXIT

echo "[Smoke] Starting viewer gamepad smoke via ${RUNNER} with fake device ${gamepad_fifo}"
"${RUNNER}" \
    --duration "${duration}" \
    --paused \
    --no-realtime \
    --width 640 \
    --height 480 \
    --camera-stream \
    --camera-host 127.0.0.1 \
    --camera-port "${camera_port}" \
    --gamepad-control \
    --gamepad-device "${gamepad_fifo}" \
    --input-deadzone 0 \
    --summary-json "${summary_json}" \
    "${extra_args[@]}" \
    >"${viewer_log}" 2>&1 &
viewer_pid=$!

health_url="http://127.0.0.1:${camera_port}/health"
status_url="http://127.0.0.1:${camera_port}/status"

ready=0
for _ in $(seq 1 360); do
    if curl -sf "${health_url}" >/dev/null; then
        ready=1
        break
    fi
    if ! kill -0 "${viewer_pid}" >/dev/null 2>&1; then
        echo "[Smoke][ERROR] viewer exited before HTTP server became ready" >&2
        sed -n '1,220p' "${viewer_log}" >&2
        exit 1
    fi
    sleep 0.1
done

if [[ "${ready}" -ne 1 ]]; then
    echo "[Smoke][ERROR] timed out waiting for ${health_url}" >&2
    sed -n '1,220p' "${viewer_log}" >&2
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

packets = []
if sequence == "loco_axis":
    packets = [
        event(1, JS_EVENT_BUTTON, 0),       # LOCO button
        event(-16384, JS_EVENT_AXIS, 1),    # default vx axis with sign -1 -> +0.5
        event(0, JS_EVENT_AXIS, 0),
        event(0, JS_EVENT_AXIS, 3),
    ]
elif sequence == "pause":
    packets = [event(1, JS_EVENT_BUTTON, 7)]
elif sequence == "passive":
    packets = [event(1, JS_EVENT_BUTTON, 1)]
elif sequence == "safety":
    packets = [event(1, JS_EVENT_BUTTON, 9)]
else:
    raise SystemExit(f"unknown sequence: {sequence}")

with open(path, "wb", buffering=0) as fifo:
    for packet in packets:
        fifo.write(packet)
        time.sleep(0.02)
PY
}

send_gamepad_events "loco_axis"

loco_ready=0
for _ in $(seq 1 80); do
    if curl -sf "${status_url}" -o "${status_body}" &&
       jq -e '.mode == "LOCO" and .paused == false and (.cmd[0] > 0.49 and .cmd[0] < 0.51) and .cmd[1] == 0 and .cmd[2] == 0 and .adapter_backend == "mujoco-sim" and .adapter_command_published == true and .sim_steps > 0' "${status_body}" >/dev/null; then
        loco_ready=1
        break
    fi
    sleep 0.1
done

if [[ "${loco_ready}" -ne 1 ]]; then
    echo "[Smoke][ERROR] viewer /status did not report fake gamepad LOCO axis command" >&2
    cat "${status_body}" >&2 || true
    sed -n '1,240p' "${viewer_log}" >&2
    exit 1
fi

send_gamepad_events "passive"

passive_ready=0
for _ in $(seq 1 80); do
    if curl -sf "${status_url}" -o "${status_body}" &&
       jq -e '.mode == "PASSIVE" and .paused == false and .cmd[0] == 0 and .cmd[1] == 0 and .cmd[2] == 0 and .adapter_backend == "mujoco-sim" and .adapter_command_published == true and .reset_pending == false' "${status_body}" >/dev/null; then
        passive_ready=1
        break
    fi
    sleep 0.1
done

if [[ "${passive_ready}" -ne 1 ]]; then
    echo "[Smoke][ERROR] viewer /status did not report fake gamepad B as PASSIVE without reset" >&2
    cat "${status_body}" >&2 || true
    sed -n '1,260p' "${viewer_log}" >&2
    exit 1
fi

send_gamepad_events "loco_axis"

loco_again_ready=0
for _ in $(seq 1 80); do
    if curl -sf "${status_url}" -o "${status_body}" &&
       jq -e '.mode == "LOCO" and .paused == false and (.cmd[0] > 0.49 and .cmd[0] < 0.51) and .cmd[1] == 0 and .cmd[2] == 0 and .adapter_backend == "mujoco-sim" and .adapter_command_published == true' "${status_body}" >/dev/null; then
        loco_again_ready=1
        break
    fi
    sleep 0.1
done

if [[ "${loco_again_ready}" -ne 1 ]]; then
    echo "[Smoke][ERROR] viewer /status did not return to fake gamepad LOCO after PASSIVE" >&2
    cat "${status_body}" >&2 || true
    sed -n '1,300p' "${viewer_log}" >&2
    exit 1
fi

send_gamepad_events "pause"

pause_ready=0
for _ in $(seq 1 80); do
    if curl -sf "${status_url}" -o "${status_body}" &&
       jq -e '.mode == "LOCO" and .paused == true and .cmd[0] == 0 and .cmd[1] == 0 and .cmd[2] == 0 and .adapter_backend == "mujoco-sim" and .adapter_command_published == true' "${status_body}" >/dev/null; then
        pause_ready=1
        break
    fi
    sleep 0.1
done

if [[ "${pause_ready}" -ne 1 ]]; then
    echo "[Smoke][ERROR] viewer /status did not report fake gamepad pause-zero" >&2
    cat "${status_body}" >&2 || true
    sed -n '1,260p' "${viewer_log}" >&2
    exit 1
fi

send_gamepad_events "safety"

safety_ready=0
for _ in $(seq 1 80); do
    if curl -sf "${status_url}" -o "${status_body}" &&
       jq -e '.mode == "LOCO" and .paused == true and .safety_enabled == true and .adapter_backend == "mujoco-sim" and .adapter_command_published == true' "${status_body}" >/dev/null; then
        safety_ready=1
        break
    fi
    sleep 0.1
done

if [[ "${safety_ready}" -ne 1 ]]; then
    echo "[Smoke][ERROR] viewer /status did not report fake gamepad safety toggle" >&2
    cat "${status_body}" >&2 || true
    sed -n '1,280p' "${viewer_log}" >&2
    exit 1
fi

if ! wait "${viewer_pid}"; then
    echo "[Smoke][ERROR] viewer exited with failure" >&2
    sed -n '1,260p' "${viewer_log}" >&2
    exit 1
fi
viewer_pid=""

if [[ ! -s "${summary_json}" ]]; then
    echo "[Smoke][ERROR] summary JSON was not written: ${summary_json}" >&2
    sed -n '1,220p' "${viewer_log}" >&2
    exit 1
fi

if ! jq -e '.mode == "LOCO" and .paused == true and .safety_enabled == true and .adapter_backend == "mujoco-sim" and .adapter_command_published == true and .sim_steps > 0 and .policy_steps > 0' "${summary_json}" >/dev/null; then
    echo "[Smoke][ERROR] summary did not report final fake gamepad LOCO pause state" >&2
    cat "${summary_json}" >&2
    exit 1
fi

echo "[Smoke] PASSED viewer gamepad control path"
echo "[Smoke] summary=${summary_json}"
jq '{mode, paused, safety_enabled, adapter_backend, adapter_command_published, sim_steps, policy_steps}' "${summary_json}"
