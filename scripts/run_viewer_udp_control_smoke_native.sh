#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
RUNNER="${SCRIPT_DIR}/run_mujoco_loco_viewer_native.sh"

duration="1.5"
udp_port=""
summary_json=""
keep_summary=0
extra_args=()

usage() {
    cat <<EOF
Usage: $0 [options] [-- extra mujoco_loco_viewer args]

Smoke-test the MuJoCo viewer UDP control path. The script starts the native
viewer with UDP control enabled, sends text-control packets, then validates the
JSON summary.

Options:
  --duration S       Viewer wall-clock duration, default ${duration}
  --udp-port N       UDP control port, default: choose a free local port
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
        --udp-port)
            udp_port="$2"
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

for tool in jq python3 ss rg; do
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

if [[ -z "${summary_json}" ]]; then
    summary_json="$(mktemp /tmp/magicbot_viewer_udp_control_XXXXXX.json)"
fi

viewer_log="$(mktemp /tmp/magicbot_viewer_udp_control_XXXXXX.log)"
viewer_pid=""

cleanup() {
    if [[ -n "${viewer_pid}" ]] && kill -0 "${viewer_pid}" >/dev/null 2>&1; then
        kill "${viewer_pid}" >/dev/null 2>&1 || true
        wait "${viewer_pid}" >/dev/null 2>&1 || true
    fi
    rm -f "${viewer_log}"
    if [[ "${keep_summary}" -eq 0 ]]; then
        rm -f "${summary_json}"
    fi
}
trap cleanup EXIT

echo "[Smoke] Starting viewer UDP control smoke on 127.0.0.1:${udp_port}"
"${RUNNER}" \
    --duration "${duration}" \
    --paused \
    --no-realtime \
    --width 640 \
    --height 480 \
    --udp-control \
    --udp-bind 127.0.0.1 \
    --udp-port "${udp_port}" \
    --summary-json "${summary_json}" \
    "${extra_args[@]}" \
    >"${viewer_log}" 2>&1 &
viewer_pid=$!

ready=0
for _ in $(seq 1 120); do
    if ss -lun | rg -q ":${udp_port}\\b"; then
        ready=1
        break
    fi
    if ! kill -0 "${viewer_pid}" >/dev/null 2>&1; then
        echo "[Smoke][ERROR] viewer exited before UDP was ready" >&2
        sed -n '1,180p' "${viewer_log}" >&2
        exit 1
    fi
    sleep 0.1
done

if [[ "${ready}" -ne 1 ]]; then
    echo "[Smoke][ERROR] timed out waiting for UDP port ${udp_port}" >&2
    sed -n '1,180p' "${viewer_log}" >&2
    exit 1
fi

python3 - <<PY
import socket
import time

port = int("${udp_port}")
packets = [
    b"mode=loco vx=0.15 vy=0.05 wz=-0.05",
    b"pause",
    b"resume",
    b"mode=passive",
    b"mode=stand",
    b"mode=final_damping",
]

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
for packet in packets:
    sock.sendto(packet, ("127.0.0.1", port))
    time.sleep(0.18)
PY

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
paused="$(jq -r '.paused' "${summary_json}")"
sim_steps="$(jq -r '.sim_steps' "${summary_json}")"

if [[ "${mode}" != "FINAL_DAMPING" ]]; then
    echo "[Smoke][ERROR] expected final mode FINAL_DAMPING, got ${mode}" >&2
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

echo "[Smoke] PASSED viewer UDP control path"
echo "[Smoke] summary=${summary_json}"
jq '{mode, paused, sim_steps}' "${summary_json}"
