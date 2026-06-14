#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
PROJECT_ROOT="$( cd "${SCRIPT_DIR}/.." &> /dev/null && pwd )"
RUNNER="${SCRIPT_DIR}/run_mujoco_loco_viewer_native.sh"

duration="1.5"
camera_port=""
summary_json=""
keep_summary=0
extra_args=()

usage() {
    cat <<EOF
Usage: $0 [options] [-- extra mujoco_loco_viewer args]

Smoke-test the MuJoCo viewer HTTP control API. The script starts the native
viewer with camera/control streaming, posts reset and mode/velocity commands,
then validates the JSON summary.

Options:
  --duration S       Viewer wall-clock duration, default ${duration}
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

for tool in curl jq python3; do
    if ! command -v "${tool}" >/dev/null 2>&1; then
        echo "[Smoke][ERROR] required tool not found: ${tool}" >&2
        exit 1
    fi
done

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

echo "[Smoke] Starting viewer HTTP control smoke on 127.0.0.1:${camera_port}"
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
control_url="http://127.0.0.1:${camera_port}/control"
reset_url="http://127.0.0.1:${camera_port}/reset"

ready=0
for _ in $(seq 1 100); do
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
post_ok "${control_url}?mode=loco&vx=0.15&vy=0.05&wz=-0.05" "loco"
post_ok "${control_url}?pause=1" "pause"
post_ok "${control_url}?pause=0" "resume"
post_ok "${control_url}?mode=final_damping" "final_damping"

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
http_control_commands="$(jq -r '.http_control_commands' "${summary_json}")"
http_reset_requests="$(jq -r '.http_reset_requests' "${summary_json}")"
sim_steps="$(jq -r '.sim_steps' "${summary_json}")"

if [[ "${mode}" != "FINAL_DAMPING" ]]; then
    echo "[Smoke][ERROR] expected final mode FINAL_DAMPING, got ${mode}" >&2
    cat "${summary_json}" >&2
    exit 1
fi
if [[ "${http_control_commands}" -lt 6 ]]; then
    echo "[Smoke][ERROR] expected at least 6 HTTP control commands, got ${http_control_commands}" >&2
    cat "${summary_json}" >&2
    exit 1
fi
if [[ "${http_reset_requests}" -lt 1 ]]; then
    echo "[Smoke][ERROR] expected at least 1 HTTP reset request, got ${http_reset_requests}" >&2
    cat "${summary_json}" >&2
    exit 1
fi
if [[ "${sim_steps}" -le 0 ]]; then
    echo "[Smoke][ERROR] expected sim to advance after loco/final commands, got sim_steps=${sim_steps}" >&2
    cat "${summary_json}" >&2
    exit 1
fi

echo "[Smoke] PASSED viewer HTTP control API"
echo "[Smoke] summary=${summary_json}"
jq '{mode, paused, sim_steps, http_reset_requests, http_control_commands}' "${summary_json}"
