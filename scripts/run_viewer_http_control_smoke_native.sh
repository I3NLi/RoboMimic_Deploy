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

Smoke-test the MuJoCo viewer HTTP control API. The script starts the viewer
runner with camera/control streaming, posts reset and mode/velocity commands,
then validates the JSON summary.

Options:
  --duration S       Viewer wall-clock duration, default ${duration}
  --runner P         Viewer runner, default ${RUNNER}
  --camera-port N    HTTP control port, default: choose a free local port
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

for tool in curl jq python3 grep; do
    if ! command -v "${tool}" >/dev/null 2>&1; then
        echo "[Smoke][ERROR] required tool not found: ${tool}" >&2
        exit 1
    fi
done

echo "[Smoke] Checking viewer sim writes through adapter boundary"
if grep -En 'data->ctrl\[[^]]+\][[:space:]]*=' "${VIEWER_SOURCE}"; then
    echo "[Smoke][ERROR] mujoco_loco_viewer.cpp must not write MuJoCo ctrl directly; use MujocoSimAdapter" >&2
    exit 1
fi
if grep -En 'OnnxLocoPolicy|\.infer\(|MotionSafety|safety\.check|torque_limited_target|clamp_and_rate_limit' "${VIEWER_SOURCE}"; then
    echo "[Smoke][ERROR] mujoco_loco_viewer.cpp must keep policy, safety, and target limiting inside ControllerCore" >&2
    exit 1
fi

echo "[Smoke] Checking viewer keyboard mode keys use shared text actions"
python3 - "${VIEWER_SOURCE}" <<'PY'
import re
import sys

source = open(sys.argv[1], encoding="utf-8").read()
match = re.search(r"case XK_l:\s*case XK_L:(.*?)case XK_m:", source, re.S)
if not match:
    print("[Smoke][ERROR] could not locate viewer L key block", file=sys.stderr)
    sys.exit(1)
block = match.group(1)
if "TextControlAction::ToggleLoco" not in block:
    print("[Smoke][ERROR] viewer L key must route through TextControlAction::ToggleLoco", file=sys.stderr)
    sys.exit(1)
if re.search(r"desired_mode\s*=(?!=)", block):
    print("[Smoke][ERROR] viewer L key must not set desired_mode directly", file=sys.stderr)
    sys.exit(1)
r_match = re.search(r"case XK_r:\s*case XK_R:(.*?)case XK_f:", source, re.S)
if not r_match:
    print("[Smoke][ERROR] could not locate viewer R key block", file=sys.stderr)
    sys.exit(1)
r_block = r_match.group(1)
if "TextControlAction::ResetStand" not in r_block:
    print("[Smoke][ERROR] viewer R key must route through TextControlAction::ResetStand", file=sys.stderr)
    sys.exit(1)
if re.search(r"(desired_mode\s*=(?!=)|reset_requested\s*=(?!=)|desired_external_policy_key\.clear\(\))", r_block):
    print("[Smoke][ERROR] viewer R key must not set reset/mode fields directly", file=sys.stderr)
    sys.exit(1)
http_reset_match = re.search(
    r"if \(camera_server->take_reset_request\(\)\) \{(.*?)\n\s*\}",
    source,
    re.S,
)
if not http_reset_match:
    print("[Smoke][ERROR] could not locate viewer HTTP reset request block", file=sys.stderr)
    sys.exit(1)
http_reset_block = http_reset_match.group(1)
if "TextControlAction::ResetStand" not in http_reset_block:
    print("[Smoke][ERROR] viewer HTTP /reset must route through the shared reset action", file=sys.stderr)
    sys.exit(1)
if re.search(r"(desired_mode\s*=(?!=)|reset_requested\s*=(?!=)|desired_external_policy_key\.clear\(\))", http_reset_block):
    print("[Smoke][ERROR] viewer HTTP /reset must not set reset/mode fields directly", file=sys.stderr)
    sys.exit(1)
keyboard_action_match = re.search(
    r"void apply_viewer_keyboard_text_action\((.*?)\n}\n\nvoid handle_key",
    source,
    re.S,
)
if not keyboard_action_match:
    print("[Smoke][ERROR] could not locate apply_viewer_keyboard_text_action", file=sys.stderr)
    sys.exit(1)
keyboard_action_body = keyboard_action_match.group(1)
if "TextControlAction::Stand" not in keyboard_action_body:
    print("[Smoke][ERROR] repeated viewer external keys must fall back through TextControlAction::Stand", file=sys.stderr)
    sys.exit(1)
if re.search(r"(desired_mode\s*=(?!=)|desired_external_policy_key\.clear\(\))", keyboard_action_body):
    print("[Smoke][ERROR] viewer keyboard action helper must not set mode fields directly", file=sys.stderr)
    sys.exit(1)
if "apply_text_control_action_to_intent" not in source:
    print("[Smoke][ERROR] viewer text actions must apply through the shared text-control intent helper", file=sys.stderr)
    sys.exit(1)
if "command_for_control_mode" not in source:
    print("[Smoke][ERROR] viewer must sanitize non-LOCO commands through the shared command helper", file=sys.stderr)
    sys.exit(1)
for local_helper in ("mode_request_for_loco_toggle", "mode_request_for_text_control_effect"):
    if local_helper in source:
        print(
            f"[Smoke][ERROR] viewer must not apply text-control mode details locally: {local_helper}",
            file=sys.stderr,
        )
        sys.exit(1)
if re.search(r"ControlMode::Loco\s*\?\s*.*ControlMode::Stand", source):
    print("[Smoke][ERROR] viewer must not duplicate LOCO toggle mode mapping", file=sys.stderr)
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
    summary_json="$(mktemp /tmp/magicbot_viewer_http_control_XXXXXX.json)"
fi

viewer_log="$(mktemp /tmp/magicbot_viewer_http_control_XXXXXX.log)"
status_body="$(mktemp /tmp/magicbot_viewer_http_control_status_XXXXXX.json)"
viewer_pid=""

cleanup() {
    if [[ -n "${viewer_pid}" ]] && kill -0 "${viewer_pid}" >/dev/null 2>&1; then
        kill "${viewer_pid}" >/dev/null 2>&1 || true
        wait "${viewer_pid}" >/dev/null 2>&1 || true
    fi
    rm -f "${status_body}" "${viewer_log}"
    if [[ "${keep_summary}" -eq 0 ]]; then
        rm -f "${summary_json}"
    fi
}
trap cleanup EXIT

echo "[Smoke] Starting viewer HTTP control smoke via ${RUNNER} on 127.0.0.1:${camera_port}"
"${RUNNER}" \
    --duration "${duration}" \
    --paused \
    --no-realtime \
    --width 640 \
    --height 480 \
    --camera-stream \
    --camera-host 127.0.0.1 \
    --camera-port "${camera_port}" \
    --summary-json "${summary_json}" \
    "${extra_args[@]}" \
    >"${viewer_log}" 2>&1 &
viewer_pid=$!

health_url="http://127.0.0.1:${camera_port}/health"
status_url="http://127.0.0.1:${camera_port}/status"
control_url="http://127.0.0.1:${camera_port}/control"
reset_url="http://127.0.0.1:${camera_port}/reset"

ready=0
for _ in $(seq 1 360); do
    if curl -sf "${health_url}" >/dev/null; then
        ready=1
        break
    fi
    if ! kill -0 "${viewer_pid}" >/dev/null 2>&1; then
        echo "[Smoke][ERROR] viewer exited before HTTP server became ready" >&2
        sed -n '1,160p' "${viewer_log}" >&2
        exit 1
    fi
    sleep 0.1
done

if [[ "${ready}" -ne 1 ]]; then
    echo "[Smoke][ERROR] timed out waiting for ${health_url}" >&2
    sed -n '1,160p' "${viewer_log}" >&2
    exit 1
fi

post_ok() {
    local url="$1"
    local label="$2"
    local status
    status="$(curl -sS -o "${status_body}" -w '%{http_code}' -X POST "${url}")"
    if [[ "${status}" != "200" ]]; then
        echo "[Smoke][ERROR] ${label} returned HTTP ${status}" >&2
        cat "${status_body}" >&2 || true
        exit 1
    fi
}

post_status() {
    local url="$1"
    curl -sS -o "${status_body}" -w '%{http_code}' -X POST "${url}"
}

post_ok "${reset_url}" "reset"
post_ok "${control_url}?mode=passive" "passive"
post_ok "${control_url}?mode=stand" "stand"
post_ok "${control_url}?vx=0.40&vy=-0.20&wz=0.10" "stand velocity should sanitize"

stand_zero_ready=0
for _ in $(seq 1 40); do
    if curl -sf "${status_url}" -o "${status_body}" &&
       jq -e '.mode == "STAND" and .target_mode == "Position" and .paused == false and .cmd[0] == 0 and .cmd[1] == 0 and .cmd[2] == 0 and .adapter_backend == "mujoco-sim" and .adapter_command_published == true' "${status_body}" >/dev/null; then
        stand_zero_ready=1
        break
    fi
    sleep 0.1
done

if [[ "${stand_zero_ready}" -ne 1 ]]; then
    echo "[Smoke][ERROR] viewer /status did not sanitize STAND velocity-only command" >&2
    cat "${status_body}" >&2 || true
    exit 1
fi

post_ok "${control_url}?mode=walk" "walk preset"

walk_ready=0
for _ in $(seq 1 40); do
    if curl -sf "${status_url}" -o "${status_body}" &&
       jq -e '.mode == "LOCO" and .target_mode == "Position" and .paused == false and (.cmd[0] > 0.24 and .cmd[0] < 0.26) and .cmd[1] == 0 and .cmd[2] == 0 and .adapter_backend == "mujoco-sim" and .adapter_command_published == true' "${status_body}" >/dev/null; then
        walk_ready=1
        break
    fi
    sleep 0.1
done

if [[ "${walk_ready}" -ne 1 ]]; then
    echo "[Smoke][ERROR] viewer /status did not report shared walk preset" >&2
    cat "${status_body}" >&2 || true
    exit 1
fi

post_ok "${control_url}?mode=run_forward" "run-forward preset"

run_ready=0
for _ in $(seq 1 40); do
    if curl -sf "${status_url}" -o "${status_body}" &&
       jq -e '.mode == "LOCO" and .target_mode == "Position" and .paused == false and (.cmd[0] > 0.64 and .cmd[0] < 0.66) and .cmd[1] == 0 and .cmd[2] == 0 and .adapter_backend == "mujoco-sim" and .adapter_command_published == true' "${status_body}" >/dev/null; then
        run_ready=1
        break
    fi
    sleep 0.1
done

if [[ "${run_ready}" -ne 1 ]]; then
    echo "[Smoke][ERROR] viewer /status did not report shared run-forward preset" >&2
    cat "${status_body}" >&2 || true
    exit 1
fi

post_ok "${control_url}?mode=loco&vx=0.15&vy=0.05&wz=-0.05" "loco"

loco_ready=0
for _ in $(seq 1 40); do
    if curl -sf "${status_url}" -o "${status_body}" &&
       jq -e '.mode == "LOCO" and .target_mode == "Position" and .paused == false and (.cmd[0] > 0.14 and .cmd[0] < 0.16) and (.cmd[1] > 0.04 and .cmd[1] < 0.06) and (.cmd[2] > -0.06 and .cmd[2] < -0.04) and .adapter_backend == "mujoco-sim" and .adapter_command_published == true' "${status_body}" >/dev/null; then
        loco_ready=1
        break
    fi
    sleep 0.1
done

if [[ "${loco_ready}" -ne 1 ]]; then
    echo "[Smoke][ERROR] viewer /status did not report LOCO command before reset endpoint test" >&2
    cat "${status_body}" >&2 || true
    exit 1
fi

post_ok "${reset_url}" "reset after loco"

reset_endpoint_ready=0
for _ in $(seq 1 40); do
    if curl -sf "${status_url}" -o "${status_body}" &&
       jq -e '.mode == "LOCO" and .target_mode == "Position" and .paused == false and (.cmd[0] > 0.14 and .cmd[0] < 0.16) and (.cmd[1] > 0.04 and .cmd[1] < 0.06) and (.cmd[2] > -0.06 and .cmd[2] < -0.04) and .http_reset_requests >= 2 and .adapter_backend == "mujoco-sim" and .adapter_command_published == true' "${status_body}" >/dev/null; then
        reset_endpoint_ready=1
        break
    fi
    sleep 0.1
done

if [[ "${reset_endpoint_ready}" -ne 1 ]]; then
    echo "[Smoke][ERROR] viewer /reset endpoint changed mode/command instead of only resetting position" >&2
    cat "${status_body}" >&2 || true
    exit 1
fi

post_ok "${control_url}?mode=reset" "control reset"
post_ok "${control_url}?pause=1" "pause"
post_ok "${control_url}?pause=0" "resume"
post_ok "${control_url}?mode=final_damping" "final_damping"

status_ready=0
for _ in $(seq 1 40); do
    if curl -sf "${status_url}" -o "${status_body}" &&
       jq -e '.mode == "DAMPING" and .target_mode == "Damping" and .paused == false and .adapter_backend == "mujoco-sim" and .adapter_command_published == true and .http_control_commands >= 10 and .sim_steps > 0 and (.cmd | length) == 3' "${status_body}" >/dev/null; then
        status_ready=1
        break
    fi
    sleep 0.1
done

if [[ "${status_ready}" -ne 1 ]]; then
    echo "[Smoke][ERROR] viewer /status did not report final control state" >&2
    cat "${status_body}" >&2 || true
    exit 1
fi

invalid_status="$(post_status "${control_url}?mode=teleport")"
if [[ "${invalid_status}" != "400" ]]; then
    echo "[Smoke][ERROR] invalid control mode returned HTTP ${invalid_status}, expected 400" >&2
    cat "${status_body}" >&2 || true
    exit 1
fi

if ! wait "${viewer_pid}"; then
    echo "[Smoke][ERROR] viewer exited with failure" >&2
    sed -n '1,200p' "${viewer_log}" >&2
    exit 1
fi
viewer_pid=""

if [[ ! -s "${summary_json}" ]]; then
    echo "[Smoke][ERROR] summary JSON was not written: ${summary_json}" >&2
    sed -n '1,160p' "${viewer_log}" >&2
    exit 1
fi

mode="$(jq -r '.mode' "${summary_json}")"
target_mode="$(jq -r '.target_mode' "${summary_json}")"
http_control_commands="$(jq -r '.http_control_commands' "${summary_json}")"
http_reset_requests="$(jq -r '.http_reset_requests' "${summary_json}")"
sim_steps="$(jq -r '.sim_steps' "${summary_json}")"
adapter_backend="$(jq -r '.adapter_backend' "${summary_json}")"
adapter_command_published="$(jq -r '.adapter_command_published' "${summary_json}")"

if [[ "${mode}" != "DAMPING" ]]; then
    echo "[Smoke][ERROR] expected final mode DAMPING, got ${mode}" >&2
    cat "${summary_json}" >&2
    exit 1
fi
if [[ "${target_mode}" != "Damping" ]]; then
    echo "[Smoke][ERROR] expected final target_mode Damping, got ${target_mode}" >&2
    cat "${summary_json}" >&2
    exit 1
fi
if [[ "${http_control_commands}" -lt 10 ]]; then
    echo "[Smoke][ERROR] expected at least 10 HTTP control commands, got ${http_control_commands}" >&2
    cat "${summary_json}" >&2
    exit 1
fi
if [[ "${http_reset_requests}" -lt 2 ]]; then
    echo "[Smoke][ERROR] expected at least 2 HTTP reset requests, got ${http_reset_requests}" >&2
    cat "${summary_json}" >&2
    exit 1
fi
if [[ "${sim_steps}" -le 0 ]]; then
    echo "[Smoke][ERROR] expected sim to advance after loco/final commands, got sim_steps=${sim_steps}" >&2
    cat "${summary_json}" >&2
    exit 1
fi
if [[ "${adapter_backend}" != "mujoco-sim" ]]; then
    echo "[Smoke][ERROR] expected adapter_backend=mujoco-sim, got ${adapter_backend}" >&2
    cat "${summary_json}" >&2
    exit 1
fi
if [[ "${adapter_command_published}" != "true" ]]; then
    echo "[Smoke][ERROR] expected adapter_command_published=true, got ${adapter_command_published}" >&2
    cat "${summary_json}" >&2
    exit 1
fi
if python3 - <<PY
import sys
try:
    duration = float("${duration}")
except ValueError:
    sys.exit(1)
sys.exit(0 if duration >= 1.1 else 1)
PY
then
    if ! grep -Eq '\[Viewer\].*mode=DAMPING' "${viewer_log}"; then
        echo "[Smoke][ERROR] viewer stdout did not report core mode DAMPING" >&2
        sed -n '1,220p' "${viewer_log}" >&2
        exit 1
    fi
fi

echo "[Smoke] PASSED viewer HTTP control API"
echo "[Smoke] summary=${summary_json}"
jq '{mode, target_mode, paused, adapter_backend, adapter_command_published, sim_steps, http_reset_requests, http_control_commands}' "${summary_json}"
