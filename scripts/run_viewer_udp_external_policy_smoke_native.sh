#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
PROJECT_ROOT="$( cd "${SCRIPT_DIR}/.." &> /dev/null && pwd )"
RUNNER="${SCRIPT_DIR}/run_mujoco_loco_viewer_native.sh"

duration="1.8"
udp_port=""
camera_port=""
summary_json=""
keep_summary=0
beyond_yaml="policies/beyond_mimic/config/BeyondMimic.yaml"
track_mimic_yaml="policies/track_mimic/config/BeyondMimic.yaml"
extra_args=()

usage() {
    cat <<EOF
Usage: $0 [options] [-- extra mujoco_loco_viewer args]

Smoke-test the MuJoCo viewer UDP external-policy path. The script starts the
native viewer with UDP control and HTTP status enabled, sends text-control UDP
packets for DANCE/BeyondMimic and SKILL/TrackMimic trajectory, then validates live /status
and the summary JSON.

Options:
  --duration S            Viewer wall-clock duration, default ${duration}
  --runner P              Viewer runner, default ${RUNNER}
  --udp-port N            UDP control port, default: choose a free local port
  --camera-port N         HTTP status port, default: choose a free local port
  --summary-json P        Summary JSON path, default: temp file under /tmp
  --beyond-yaml P         BeyondMimic YAML, default ${beyond_yaml}
  --track-mimic-yaml P    BeyondMimic trajectory YAML, default ${track_mimic_yaml}
  --keep-summary          Keep the temp summary path printed at the end
  -h, --help              Show this help
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
        --beyond-yaml)
            beyond_yaml="$2"
            shift 2
            ;;
        --track-mimic-yaml)
            track_mimic_yaml="$2"
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

resolve_repo_path() {
    if [[ "$1" == /* ]]; then
        printf '%s\n' "$1"
    else
        printf '%s\n' "${PROJECT_ROOT}/$1"
    fi
}

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
    summary_json="$(mktemp /tmp/magicbot_viewer_udp_external_XXXXXX.json)"
fi

viewer_log="$(mktemp /tmp/magicbot_viewer_udp_external_XXXXXX.log)"
status_body="$(mktemp /tmp/magicbot_viewer_udp_external_status_XXXXXX.json)"
track_tmp_dir="$(mktemp -d /tmp/magicbot_track_mimic_XXXXXX)"
track_mimic_runtime_yaml="${track_tmp_dir}/BeyondMimic.yaml"
viewer_pid=""

python3 "${SCRIPT_DIR}/make_track_mimic_motion_yaml.py" \
    --base-yaml "$(resolve_repo_path "${track_mimic_yaml}")" \
    --output-yaml "${track_mimic_runtime_yaml}" \
    >/dev/null

cleanup() {
    if [[ -n "${viewer_pid}" ]] && kill -0 "${viewer_pid}" >/dev/null 2>&1; then
        kill "${viewer_pid}" >/dev/null 2>&1 || true
        wait "${viewer_pid}" >/dev/null 2>&1 || true
    fi
    rm -f "${status_body}" "${viewer_log}"
    rm -rf "${track_tmp_dir}"
    if [[ "${keep_summary}" -eq 0 ]]; then
        rm -f "${summary_json}"
    fi
}
trap cleanup EXIT

echo "[Smoke] Starting viewer UDP external-policy smoke via ${RUNNER} on 127.0.0.1:${udp_port} with HTTP 127.0.0.1:${camera_port}"
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
    --beyond-yaml "${beyond_yaml}" \
    --track-mimic-yaml "${track_mimic_runtime_yaml}" \
    --summary-json "${summary_json}" \
    "${extra_args[@]}" \
    >"${viewer_log}" 2>&1 &
viewer_pid=$!

health_url="http://127.0.0.1:${camera_port}/health"
status_url="http://127.0.0.1:${camera_port}/status"

ready=0
for _ in $(seq 1 360); do
    if curl -sf "${health_url}" >/dev/null && ss -lun | rg -q ":${udp_port}\\b"; then
        ready=1
        break
    fi
    if ! kill -0 "${viewer_pid}" >/dev/null 2>&1; then
        echo "[Smoke][ERROR] viewer exited before control endpoints became ready" >&2
        sed -n '1,220p' "${viewer_log}" >&2
        exit 1
    fi
    sleep 0.1
done

if [[ "${ready}" -ne 1 ]]; then
    echo "[Smoke][ERROR] timed out waiting for HTTP/UDP endpoints" >&2
    sed -n '1,220p' "${viewer_log}" >&2
    exit 1
fi

send_udp() {
    local packet="$1"
    PACKET="${packet}" UDP_PORT="${udp_port}" python3 - <<'PY'
import os
import socket

packet = os.environ["PACKET"].encode()
port = int(os.environ["UDP_PORT"])
with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
    sock.sendto(packet, ("127.0.0.1", port))
PY
}

send_udp "mode=beyond"

dance_ready=0
for _ in $(seq 1 80); do
    if curl -sf "${status_url}" -o "${status_body}" &&
       jq -e '.mode == "DANCE" and .external_policy == "BeyondMimic" and .adapter_backend == "mujoco-sim" and .adapter_command_published == true and .paused == false and .policy_steps > 0 and .sim_steps > 0' "${status_body}" >/dev/null; then
        dance_ready=1
        break
    fi
    sleep 0.1
done

if [[ "${dance_ready}" -ne 1 ]]; then
    echo "[Smoke][ERROR] viewer /status did not report UDP DANCE/BeyondMimic" >&2
    cat "${status_body}" >&2 || true
    sed -n '1,240p' "${viewer_log}" >&2
    exit 1
fi

send_udp "mode=track_mimic"

skill_ready=0
for _ in $(seq 1 80); do
    if curl -sf "${status_url}" -o "${status_body}" &&
       jq -e '.mode == "SKILL" and .external_policy == "TrackMimic" and .adapter_backend == "mujoco-sim" and .adapter_command_published == true and .paused == false and .policy_steps > 0 and .sim_steps > 0' "${status_body}" >/dev/null; then
        skill_ready=1
        break
    fi
    sleep 0.1
done

if [[ "${skill_ready}" -ne 1 ]]; then
    echo "[Smoke][ERROR] viewer /status did not report UDP SKILL/TrackMimic trajectory" >&2
    cat "${status_body}" >&2 || true
    sed -n '1,260p' "${viewer_log}" >&2
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

if ! jq -e '.mode == "SKILL" and .external_policy == "TrackMimic" and .adapter_backend == "mujoco-sim" and .adapter_command_published == true and .paused == false and .policy_steps > 0 and .sim_steps > 0' "${summary_json}" >/dev/null; then
    echo "[Smoke][ERROR] summary did not report final UDP SKILL/TrackMimic trajectory" >&2
    cat "${summary_json}" >&2
    exit 1
fi

echo "[Smoke] PASSED viewer UDP external-policy path"
echo "[Smoke] summary=${summary_json}"
jq '{mode, external_policy, adapter_backend, adapter_command_published, paused, sim_steps, policy_steps}' "${summary_json}"
