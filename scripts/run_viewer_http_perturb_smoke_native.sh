#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
RUNNER="${SCRIPT_DIR}/run_mujoco_loco_viewer_native.sh"

duration="1.5"
camera_port=""
summary_json=""
keep_summary=0
push_start="0.25"
push_duration="0.12"
push_impulse_time="0.55"
extra_args=()

usage() {
    cat <<EOF
Usage: $0 [options] [-- extra viewer runner args]

Smoke-test remote viewer perturb events over the HTTP control API. The script
starts the viewer runner with camera/control streaming, posts a drag gesture to
/viewer-event, then validates that MuJoCo perturb forces were applied. Use
--runner to cover compatibility launchers such as run_python_mujoco_viewer.py.

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

for tool in curl jq python3; do
    if ! command -v "${tool}" >/dev/null 2>&1; then
        echo "[Smoke][ERROR] required tool not found: ${tool}" >&2
        exit 1
    fi
done

python3 - "${duration}" "${push_start}" "${push_duration}" "${push_impulse_time}" <<'PY'
import sys

labels = ("--duration", "--push-start", "--push-duration", "--push-impulse-time")
try:
    duration, push_start, push_duration, push_impulse_time = map(float, sys.argv[1:])
except ValueError as exc:
    raise SystemExit(f"[Smoke][ERROR] invalid numeric timing argument: {exc}")

required = max(push_start + push_duration, push_impulse_time)
if duration + 1e-9 < required:
    raise SystemExit(
        "[Smoke][ERROR] --duration must cover the full disturbance window: "
        f"duration={duration:g}s required>={required:g}s "
        f"({labels[1]} + {labels[2]} or {labels[3]})"
    )
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
    summary_json="$(mktemp /tmp/magicbot_viewer_http_perturb_XXXXXX.json)"
fi

viewer_log="$(mktemp /tmp/magicbot_viewer_http_perturb_XXXXXX.log)"
status_body="$(mktemp /tmp/magicbot_viewer_http_perturb_status_XXXXXX.json)"
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

echo "[Smoke] Starting viewer HTTP perturb smoke via ${RUNNER} on 127.0.0.1:${camera_port}"
"${RUNNER}" \
    --duration "${duration}" \
    --unpaused \
    --no-realtime \
    --width 640 \
    --height 480 \
    --camera-stream \
    --camera-host 127.0.0.1 \
    --camera-port "${camera_port}" \
    --push-body pelvis \
    --push-force 35,0,0 \
    --push-start "${push_start}" \
    --push-duration "${push_duration}" \
    --push-impulse 0,1,0 \
    --push-impulse-time "${push_impulse_time}" \
    --summary-json "${summary_json}" \
    "${extra_args[@]}" \
    >"${viewer_log}" 2>&1 &
viewer_pid=$!

health_url="http://127.0.0.1:${camera_port}/health"
status_url="http://127.0.0.1:${camera_port}/status"
event_url="http://127.0.0.1:${camera_port}/viewer-event"

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

post_ok "${event_url}?type=down&x=0&y=0&width=640&height=480&button=0" "drag down"
post_ok "${event_url}?type=move&dx=80&dy=0&width=640&height=480&button=0" "drag move"
sleep 0.4
post_ok "${status_url}" "live perturb status"
if ! jq -e '
    .push_enabled == true
    and .push_body == "pelvis"
    and .push_body_id > 0
    and .push_body_resolved == "pelvis"
    and .push_force == [35,0,0]
    and .push_impulse == [0,1,0]
    and .push_start_s == 0.25
    and .push_duration_s == 0.12
    and .push_impulse_time_s == 0.55
    and .push_force_norm == 35
    and .push_impulse_norm == 1
    and .push_force_steps > 0
    and .push_impulse_applied == true
' "${status_body}" >/dev/null; then
    echo "[Smoke][ERROR] live /status did not report scheduled push telemetry" >&2
    cat "${status_body}" >&2
    exit 1
fi
post_ok "${event_url}?type=up&width=640&height=480&button=0" "drag up"

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

sim_steps="$(jq -r '.sim_steps' "${summary_json}")"
http_viewer_events="$(jq -r '.http_viewer_events' "${summary_json}")"
mouse_perturb_steps="$(jq -r '.mouse_perturb_steps' "${summary_json}")"
last_perturb_body="$(jq -r '.last_perturb_body' "${summary_json}")"
last_perturb_body_name="$(jq -r '.last_perturb_body_name' "${summary_json}")"
push_body_resolved="$(jq -r '.push_body_resolved' "${summary_json}")"
push_force_steps="$(jq -r '.push_force_steps' "${summary_json}")"
push_impulse_applied="$(jq -r '.push_impulse_applied' "${summary_json}")"

if [[ "${sim_steps}" -le 0 ]]; then
    echo "[Smoke][ERROR] expected sim to advance, got sim_steps=${sim_steps}" >&2
    cat "${summary_json}" >&2
    exit 1
fi
if [[ "${http_viewer_events}" -lt 3 ]]; then
    echo "[Smoke][ERROR] expected at least 3 HTTP viewer events, got ${http_viewer_events}" >&2
    cat "${summary_json}" >&2
    exit 1
fi
if [[ "${mouse_perturb_steps}" -le 0 ]]; then
    echo "[Smoke][ERROR] expected mouse_perturb_steps > 0, got ${mouse_perturb_steps}" >&2
    cat "${summary_json}" >&2
    exit 1
fi
if [[ "${last_perturb_body}" -le 0 || -z "${last_perturb_body_name}" ]]; then
    echo "[Smoke][ERROR] expected perturb body to resolve, got id=${last_perturb_body} name=${last_perturb_body_name}" >&2
    cat "${summary_json}" >&2
    exit 1
fi
if [[ -z "${push_body_resolved}" || "${push_body_resolved}" == "null" ]]; then
    echo "[Smoke][ERROR] expected scheduled push body to resolve" >&2
    cat "${summary_json}" >&2
    exit 1
fi
if [[ "${push_force_steps}" -le 0 ]]; then
    echo "[Smoke][ERROR] expected scheduled push_force_steps > 0, got ${push_force_steps}" >&2
    cat "${summary_json}" >&2
    exit 1
fi
if [[ "${push_impulse_applied}" != "true" ]]; then
    echo "[Smoke][ERROR] expected scheduled push_impulse_applied=true" >&2
    cat "${summary_json}" >&2
    exit 1
fi
if ! jq -e '.push_enabled == true and .push_force_norm > 0 and .push_impulse_norm > 0 and .push_duration_s > 0' "${summary_json}" >/dev/null; then
    echo "[Smoke][ERROR] expected nonzero scheduled push/impulse summary fields" >&2
    cat "${summary_json}" >&2
    exit 1
fi
if ! jq -e '
    .push_force == [35,0,0]
    and .push_impulse == [0,1,0]
    and .push_start_s == 0.25
    and .push_duration_s == 0.12
    and .push_impulse_time_s == 0.55
' "${summary_json}" >/dev/null; then
    echo "[Smoke][ERROR] expected scheduled push vectors/timing in summary" >&2
    cat "${summary_json}" >&2
    exit 1
fi

echo "[Smoke] PASSED viewer HTTP perturb API"
echo "[Smoke] summary=${summary_json}"
jq '{sim_steps, http_viewer_events, mouse_perturb_steps, last_perturb_body, last_perturb_body_name, push_body_resolved, push_force, push_impulse, push_start_s, push_duration_s, push_impulse_time_s, push_force_steps, push_impulse_applied, push_force_norm, push_impulse_norm}' "${summary_json}"
