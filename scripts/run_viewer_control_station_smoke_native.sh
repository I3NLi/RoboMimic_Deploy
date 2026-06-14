#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
PROJECT_ROOT="$( cd "${SCRIPT_DIR}/.." &> /dev/null && pwd )"
RUNNER="${SCRIPT_DIR}/run_mujoco_loco_viewer_native.sh"

duration="1.4"
camera_port=""
udp_port=""
summary_json=""
keep_summary=0
track_mimic_yaml="policies/track_mimic/config/BeyondMimic.yaml"
extra_args=()

usage() {
    cat <<EOF
Usage: $0 [options] [-- extra run_mujoco_loco_viewer_native args]

Smoke-test the --control-station preset. The script starts the native viewer
through the control-station wrapper, provides a smoke TrackMimic trajectory
YAML, then verifies HTTP control can enter DANCE/BeyondMimic and
SKILL/TrackMimic trajectory.

Options:
  --duration S       Viewer wall-clock duration, default ${duration}
  --runner P         Viewer runner, default ${RUNNER}
  --camera-port N    HTTP control port, default: choose a free local port
  --udp-port N       UDP control port, default: choose a free local port
  --summary-json P   Summary JSON path, default: temp file under /tmp
  --track-mimic-yaml P
                     Base TrackMimic YAML, default ${track_mimic_yaml}
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
        --udp-port)
            udp_port="$2"
            shift 2
            ;;
        --summary-json)
            summary_json="$2"
            keep_summary=1
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

for tool in curl jq python3; do
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

choose_port() {
    python3 - <<'PY'
import socket

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
    sock.bind(("127.0.0.1", 0))
    print(sock.getsockname()[1])
PY
}

if [[ -z "${camera_port}" ]]; then
    camera_port="$(choose_port)"
fi
if [[ -z "${udp_port}" ]]; then
    udp_port="$(choose_port)"
fi
if [[ -z "${summary_json}" ]]; then
    summary_json="$(mktemp /tmp/magicbot_viewer_control_station_XXXXXX.json)"
fi
viewer_log="$(mktemp /tmp/magicbot_viewer_control_station_XXXXXX.log)"
status_body="$(mktemp /tmp/magicbot_viewer_control_station_status_XXXXXX.json)"
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

echo "[Smoke] Starting viewer control-station smoke via ${RUNNER} on HTTP 127.0.0.1:${camera_port}, UDP 127.0.0.1:${udp_port}"
"${RUNNER}" \
    --control-station \
    --duration "${duration}" \
    --unpaused \
    --no-realtime \
    --camera-host 127.0.0.1 \
    --camera-port "${camera_port}" \
    --udp-bind 127.0.0.1 \
    --udp-port "${udp_port}" \
    --track-mimic-yaml "${track_mimic_runtime_yaml}" \
    --summary-json "${summary_json}" \
    "${extra_args[@]}" \
    >"${viewer_log}" 2>&1 &
viewer_pid=$!

health_url="http://127.0.0.1:${camera_port}/health"
control_url="http://127.0.0.1:${camera_port}/control"
status_url="http://127.0.0.1:${camera_port}/status"

ready=0
for _ in $(seq 1 80); do
    if ! kill -0 "${viewer_pid}" >/dev/null 2>&1; then
        echo "[Smoke][ERROR] viewer exited before control-station endpoints became ready" >&2
        sed -n '1,220p' "${viewer_log}" >&2
        exit 1
    fi
    if curl -fsS "${health_url}" >/dev/null 2>&1; then
        ready=1
        break
    fi
    sleep 0.05
done
if [[ "${ready}" -ne 1 ]]; then
    echo "[Smoke][ERROR] viewer control-station HTTP endpoint did not become ready" >&2
    sed -n '1,220p' "${viewer_log}" >&2
    exit 1
fi

post_control() {
    local mode="$1"
    local status
    status="$(curl -sS -o "${status_body}" -w '%{http_code}' -X POST "${control_url}?mode=${mode}")"
    if [[ "${status}" != "200" ]]; then
        echo "[Smoke][ERROR] control mode=${mode} returned HTTP ${status}" >&2
        cat "${status_body}" >&2 || true
        sed -n '1,260p' "${viewer_log}" >&2
        exit 1
    fi
}

wait_status() {
    local jq_expr="$1"
    local description="$2"
    local ready=0
    for _ in $(seq 1 80); do
        curl -fsS "${status_url}" -o "${status_body}"
        if jq -e "${jq_expr}" "${status_body}" >/dev/null; then
            ready=1
            break
        fi
        sleep 0.05
    done
    if [[ "${ready}" -ne 1 ]]; then
        echo "[Smoke][ERROR] viewer /status did not report ${description}" >&2
        cat "${status_body}" >&2 || true
        sed -n '1,260p' "${viewer_log}" >&2
        exit 1
    fi
}

post_control "beyond"
wait_status '.mode == "DANCE" and .external_policy == "BeyondMimic" and .adapter_backend == "mujoco-sim" and .adapter_command_published == true and .paused == false and .policy_steps > 0 and .sim_steps > 0' "control-station DANCE/BeyondMimic"

post_control "track_mimic"
wait_status '.mode == "SKILL" and .external_policy == "TrackMimic" and .adapter_backend == "mujoco-sim" and .adapter_command_published == true and .paused == false and .policy_steps > 0 and .sim_steps > 0 and .http_control_commands >= 2' "control-station SKILL/TrackMimic trajectory"

if ! wait "${viewer_pid}"; then
    echo "[Smoke][ERROR] viewer exited with failure" >&2
    sed -n '1,260p' "${viewer_log}" >&2
    exit 1
fi
viewer_pid=""

if [[ ! -s "${summary_json}" ]]; then
    echo "[Smoke][ERROR] missing summary JSON: ${summary_json}" >&2
    sed -n '1,220p' "${viewer_log}" >&2
    exit 1
fi

if ! jq -e '.mode == "SKILL" and .external_policy == "TrackMimic" and .adapter_backend == "mujoco-sim" and .adapter_command_published == true and .paused == false and .policy_steps > 0 and .sim_steps > 0 and .http_control_commands >= 2' "${summary_json}" >/dev/null; then
    echo "[Smoke][ERROR] summary did not report final control-station SKILL/TrackMimic trajectory" >&2
    cat "${summary_json}" >&2
    exit 1
fi

echo "[Smoke] PASSED viewer control-station external policies"
echo "[Smoke] summary=${summary_json}"
jq '{mode, external_policy, adapter_backend, adapter_command_published, paused, sim_steps, policy_steps, http_control_commands}' "${summary_json}"
