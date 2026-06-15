#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
RUNNER="${SCRIPT_DIR}/run_mujoco_loco_viewer_native.sh"

duration="1.5"
udp_port=""
camera_port=""
summary_json=""
keep_summary=0
extra_args=()

usage() {
    cat <<EOF
Usage: $0 [options] [-- extra viewer runner args]

Smoke-test the MuJoCo viewer UDP control path. The script starts the viewer
runner with UDP control enabled, sends text-control packets, then validates the
JSON summary.

Options:
  --duration S       Viewer wall-clock duration, default ${duration}
  --runner P         Viewer runner, default ${RUNNER}
  --udp-port N       UDP control port, default: choose a free local port
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
        --udp-port)
            udp_port="$2"
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

for tool in curl jq python3 ss rg; do
    if ! command -v "${tool}" >/dev/null 2>&1; then
        echo "[Smoke][ERROR] required tool not found: ${tool}" >&2
        exit 1
    fi
done

if [[ -z "${udp_port}" ]]; then
    udp_port="$(python3 - <<'PY'
import socket

with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
    sock.bind(("127.0.0.1", 0))
    print(sock.getsockname()[1])
PY
)"
fi

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
    summary_json="$(mktemp /tmp/magicbot_viewer_udp_control_XXXXXX.json)"
fi

viewer_log="$(mktemp /tmp/magicbot_viewer_udp_control_XXXXXX.log)"
status_body="$(mktemp /tmp/magicbot_viewer_udp_control_status_XXXXXX.json)"
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

echo "[Smoke] Starting viewer UDP control smoke via ${RUNNER} on UDP 127.0.0.1:${udp_port}, HTTP 127.0.0.1:${camera_port}"
"${RUNNER}" \
    --duration "${duration}" \
    --paused \
    --no-realtime \
    --width 640 \
    --height 480 \
    --camera-stream \
    --camera-host 127.0.0.1 \
    --camera-port "${camera_port}" \
    --udp-control \
    --udp-bind 127.0.0.1 \
    --udp-port "${udp_port}" \
    --summary-json "${summary_json}" \
    "${extra_args[@]}" \
    >"${viewer_log}" 2>&1 &
viewer_pid=$!

health_url="http://127.0.0.1:${camera_port}/health"
status_url="http://127.0.0.1:${camera_port}/status"

ready=0
for _ in $(seq 1 360); do
    if curl -fsS "${health_url}" >/dev/null 2>&1 && ss -lun | rg -q ":${udp_port}\\b"; then
        ready=1
        break
    fi
    if ! kill -0 "${viewer_pid}" >/dev/null 2>&1; then
        echo "[Smoke][ERROR] viewer exited before HTTP/UDP endpoints were ready" >&2
        sed -n '1,180p' "${viewer_log}" >&2
        exit 1
    fi
    sleep 0.1
done

if [[ "${ready}" -ne 1 ]]; then
    echo "[Smoke][ERROR] timed out waiting for HTTP/UDP endpoints" >&2
    sed -n '1,180p' "${viewer_log}" >&2
    exit 1
fi

send_udp() {
    local packet="$1"
    PACKET="${packet}" UDP_PORT="${udp_port}" python3 - <<'PY'
import socket

import os

port = int(os.environ["UDP_PORT"])
packet = os.environ["PACKET"].encode()
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.sendto(packet, ("127.0.0.1", port))
PY
}

wait_status() {
    local jq_filter="$1"
    local label="$2"
    for _ in $(seq 1 40); do
        if curl -fsS "${status_url}" -o "${status_body}" &&
           jq -e "${jq_filter}" "${status_body}" >/dev/null; then
            return 0
        fi
        sleep 0.05
    done
    echo "[Smoke][ERROR] viewer /status did not report ${label}" >&2
    cat "${status_body}" >&2 || true
    sed -n '1,220p' "${viewer_log}" >&2
    exit 1
}

send_udp "walk"
wait_status '.mode == "LOCO" and .paused == false and (.cmd[0] > 0.24 and .cmd[0] < 0.26) and .cmd[1] == 0 and .cmd[2] == 0 and .adapter_backend == "mujoco-sim" and .adapter_command_published == true' "shared UDP walk preset"

send_udp "run_forward"
wait_status '.mode == "LOCO" and .paused == false and (.cmd[0] > 0.64 and .cmd[0] < 0.66) and .cmd[1] == 0 and .cmd[2] == 0 and .adapter_backend == "mujoco-sim" and .adapter_command_published == true' "shared UDP run-forward preset"

for packet in \
    "mode=loco vx=0.15 vy=0.05 wz=-0.05" \
    "pause" \
    "resume" \
    "mode=passive" \
    "mode=stand" \
    "mode=reset" \
    "mode=final_damping"; do
    send_udp "${packet}"
    sleep 0.12
done

if ! wait "${viewer_pid}"; then
    echo "[Smoke][ERROR] viewer exited with failure" >&2
    sed -n '1,220p' "${viewer_log}" >&2
    exit 1
fi
viewer_pid=""

if [[ ! -s "${summary_json}" ]]; then
    echo "[Smoke][ERROR] summary JSON was not written: ${summary_json}" >&2
    sed -n '1,180p' "${viewer_log}" >&2
    exit 1
fi

mode="$(jq -r '.mode' "${summary_json}")"
target_mode="$(jq -r '.target_mode' "${summary_json}")"
paused="$(jq -r '.paused' "${summary_json}")"
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
if [[ "${paused}" != "false" ]]; then
    echo "[Smoke][ERROR] expected final paused=false, got ${paused}" >&2
    cat "${summary_json}" >&2
    exit 1
fi
if [[ "${sim_steps}" -le 0 ]]; then
    echo "[Smoke][ERROR] expected sim to advance, got sim_steps=${sim_steps}" >&2
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

echo "[Smoke] PASSED viewer UDP control path"
echo "[Smoke] summary=${summary_json}"
jq '{mode, target_mode, paused, adapter_backend, adapter_command_published, sim_steps}' "${summary_json}"
