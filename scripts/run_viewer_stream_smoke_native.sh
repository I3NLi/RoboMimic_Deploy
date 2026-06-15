#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
RUNNER="${SCRIPT_DIR}/run_mujoco_loco_viewer_native.sh"

duration="2.5"
camera_port=""
summary_json=""
keep_summary=0
extra_args=()

usage() {
    cat <<EOF
Usage: $0 [options] [-- extra viewer runner args]

Smoke-test the MuJoCo viewer HTTP camera endpoints. The script starts the
shared-runtime viewer, waits for /health, validates /frame.jpg, then opens
/stream.mjpg long enough to see multipart MJPEG headers and at least one JPEG
part.

Options:
  --duration S       Viewer wall-clock duration, default ${duration}
  --runner P         Viewer runner, default ${RUNNER}
  --camera-port N    HTTP camera port, default: choose a free local port
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
    summary_json="$(mktemp /tmp/magicbot_viewer_stream_XXXXXX.json)"
fi

viewer_log="$(mktemp /tmp/magicbot_viewer_stream_XXXXXX.log)"
frame_path="$(mktemp /tmp/magicbot_viewer_frame_XXXXXX.jpg)"
stream_headers="$(mktemp /tmp/magicbot_viewer_stream_headers_XXXXXX.txt)"
stream_body="$(mktemp /tmp/magicbot_viewer_stream_body_XXXXXX.bin)"
stream_error="$(mktemp /tmp/magicbot_viewer_stream_error_XXXXXX.txt)"
viewer_pid=""

cleanup() {
    if [[ -n "${viewer_pid}" ]] && kill -0 "${viewer_pid}" >/dev/null 2>&1; then
        kill "${viewer_pid}" >/dev/null 2>&1 || true
        wait "${viewer_pid}" >/dev/null 2>&1 || true
    fi
    rm -f "${viewer_log}" "${frame_path}" "${stream_headers}" "${stream_body}" "${stream_error}"
    if [[ "${keep_summary}" -eq 0 ]]; then
        rm -f "${summary_json}"
    fi
}
trap cleanup EXIT

echo "[Smoke] Starting viewer stream smoke via ${RUNNER} on 127.0.0.1:${camera_port}"
"${RUNNER}" \
    --duration "${duration}" \
    --unpaused \
    --no-realtime \
    --width 480 \
    --height 360 \
    --camera-stream \
    --camera-host 127.0.0.1 \
    --camera-port "${camera_port}" \
    --summary-json "${summary_json}" \
    "${extra_args[@]}" \
    >"${viewer_log}" 2>&1 &
viewer_pid=$!

health_url="http://127.0.0.1:${camera_port}/health"
frame_url="http://127.0.0.1:${camera_port}/frame.jpg"
stream_url="http://127.0.0.1:${camera_port}/stream.mjpg"

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

frame_ready=0
for _ in $(seq 1 80); do
    if curl -fsS --max-time 1 "${frame_url}" -o "${frame_path}" >/dev/null 2>&1 && [[ -s "${frame_path}" ]]; then
        frame_ready=1
        break
    fi
    if ! kill -0 "${viewer_pid}" >/dev/null 2>&1; then
        echo "[Smoke][ERROR] viewer exited before /frame.jpg became ready" >&2
        sed -n '1,180p' "${viewer_log}" >&2
        exit 1
    fi
    sleep 0.05
done

if [[ "${frame_ready}" -ne 1 ]]; then
    echo "[Smoke][ERROR] timed out waiting for ${frame_url}" >&2
    sed -n '1,180p' "${viewer_log}" >&2
    exit 1
fi

if ! python3 - "${frame_path}" <<'PY'
import pathlib
import sys

data = pathlib.Path(sys.argv[1]).read_bytes()
sys.exit(0 if data.startswith(b"\xff\xd8") else 1)
PY
then
    echo "[Smoke][ERROR] /frame.jpg did not start with a JPEG SOI marker" >&2
    exit 1
fi

set +e
curl -fsS --max-time 2 -D "${stream_headers}" "${stream_url}" -o "${stream_body}" 2>"${stream_error}"
stream_status=$?
set -e

if [[ "${stream_status}" -ne 0 && "${stream_status}" -ne 28 ]]; then
    echo "[Smoke][ERROR] /stream.mjpg curl failed with status ${stream_status}" >&2
    cat "${stream_error}" >&2 || true
    sed -n '1,180p' "${viewer_log}" >&2
    exit 1
fi

if ! grep -qi 'content-type: multipart/x-mixed-replace; boundary=frame' "${stream_headers}"; then
    echo "[Smoke][ERROR] /stream.mjpg did not return multipart MJPEG headers" >&2
    cat "${stream_headers}" >&2 || true
    exit 1
fi

if ! grep -a -q -- '--frame' "${stream_body}" || ! grep -a -q 'Content-Type: image/jpeg' "${stream_body}"; then
    echo "[Smoke][ERROR] /stream.mjpg did not include a JPEG multipart frame" >&2
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

if ! jq -e '.sim_steps > 0' "${summary_json}" >/dev/null; then
    echo "[Smoke][ERROR] summary did not report sim progress" >&2
    cat "${summary_json}" >&2
    exit 1
fi

if [[ "${keep_summary}" -eq 1 ]]; then
    echo "[Smoke] Summary kept at ${summary_json}"
fi

echo "[Smoke] Viewer stream smoke passed"
