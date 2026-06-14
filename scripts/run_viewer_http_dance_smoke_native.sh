#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
RUNNER="${SCRIPT_DIR}/run_mujoco_loco_viewer_native.sh"

duration="0.8"
camera_port=""
summary_json=""
keep_summary=0
beyond_yaml="policies/beyond_mimic/config/BeyondMimic.yaml"
extra_args=()

usage() {
    cat <<EOF
Usage: $0 [options] [-- extra mujoco_loco_viewer args]

Smoke-test the viewer HTTP control path for DANCE/BeyondMimic. The script starts
the native viewer with BeyondMimic registered as a shared ControllerCore
external policy, posts mode=beyond to /control, then validates live /status and
the summary JSON.

Options:
  --duration S       Viewer wall-clock duration, default ${duration}
  --camera-port N    HTTP control port, default: choose a free local port
  --summary-json P   Summary JSON path, default: temp file under /tmp
  --beyond-yaml P    BeyondMimic YAML, default ${beyond_yaml}
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
        --beyond-yaml)
            beyond_yaml="$2"
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
    summary_json="$(mktemp /tmp/magicbot_viewer_http_dance_XXXXXX.json)"
fi

viewer_log="$(mktemp /tmp/magicbot_viewer_http_dance_XXXXXX.log)"
status_body="$(mktemp /tmp/magicbot_viewer_http_dance_status_XXXXXX.json)"
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

echo "[Smoke] Starting viewer HTTP DANCE smoke on 127.0.0.1:${camera_port}"
"${RUNNER}" \
    --duration "${duration}" \
    --paused \
    --no-realtime \
    --width 640 \
    --height 480 \
    --camera-stream \
    --camera-host 127.0.0.1 \
    --camera-port "${camera_port}" \
    --beyond-yaml "${beyond_yaml}" \
    --summary-json "${summary_json}" \
    "${extra_args[@]}" \
    >"${viewer_log}" 2>&1 &
viewer_pid=$!

health_url="http://127.0.0.1:${camera_port}/health"
status_url="http://127.0.0.1:${camera_port}/status"
control_url="http://127.0.0.1:${camera_port}/control"

ready=0
for _ in $(seq 1 360); do
    if curl -sf "${health_url}" >/dev/null; then
        ready=1
        break
    fi
    if ! kill -0 "${viewer_pid}" >/dev/null 2>&1; then
        echo "[Smoke][ERROR] viewer exited before HTTP server became ready" >&2
        sed -n '1,180p' "${viewer_log}" >&2
        exit 1
    fi
    sleep 0.1
done

if [[ "${ready}" -ne 1 ]]; then
    echo "[Smoke][ERROR] timed out waiting for ${health_url}" >&2
    sed -n '1,180p' "${viewer_log}" >&2
    exit 1
fi

status="$(curl -sS -o "${status_body}" -w '%{http_code}' -X POST "${control_url}?mode=beyond")"
if [[ "${status}" != "200" ]]; then
    echo "[Smoke][ERROR] beyond control returned HTTP ${status}" >&2
    cat "${status_body}" >&2 || true
    exit 1
fi

status_ready=0
for _ in $(seq 1 80); do
    if curl -sf "${status_url}" -o "${status_body}" &&
       jq -e '.mode == "DANCE" and .external_policy == "BeyondMimic" and .paused == false and .http_control_commands >= 1 and .policy_steps > 0 and .sim_steps > 0' "${status_body}" >/dev/null; then
        status_ready=1
        break
    fi
    sleep 0.1
done

if [[ "${status_ready}" -ne 1 ]]; then
    echo "[Smoke][ERROR] viewer /status did not report active DANCE" >&2
    cat "${status_body}" >&2 || true
    sed -n '1,220p' "${viewer_log}" >&2
    exit 1
fi

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

if ! jq -e '.mode == "DANCE" and .external_policy == "BeyondMimic" and .paused == false and .policy_steps > 0 and .sim_steps > 0 and .http_control_commands >= 1' "${summary_json}" >/dev/null; then
    echo "[Smoke][ERROR] summary did not report active DANCE" >&2
    cat "${summary_json}" >&2
    exit 1
fi

echo "[Smoke] PASSED viewer HTTP DANCE/BeyondMimic control"
echo "[Smoke] summary=${summary_json}"
jq '{mode, external_policy, paused, sim_steps, policy_steps, http_control_commands}' "${summary_json}"
