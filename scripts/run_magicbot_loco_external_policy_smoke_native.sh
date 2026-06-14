#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
PROJECT_ROOT="$( cd "${SCRIPT_DIR}/.." &> /dev/null && pwd )"
RUNNER="${SCRIPT_DIR}/run_magicbot_loco_native.sh"

duration="1.8"
udp_port=""
beyond_yaml="${PROJECT_ROOT}/policies/beyond_mimic/config/BeyondMimic.yaml"
track_mimic_yaml="${PROJECT_ROOT}/policies/track_mimic/config/BeyondMimic.yaml"
keep_log=0

usage() {
    cat <<EOF
Usage: $0 [options]

Smoke-test the real runner external-policy entry path without connecting to a
robot. The script dry-runs DANCE/BeyondMimic and SKILL/TrackMimic as a
BeyondMimic trajectory YAML loading path, then starts
magicbot_z1_loco_onnx in --input-check mode with explicit allow gates and sends
UDP text controls for DANCE/BeyondMimic and SKILL/TrackMimic trajectory.

Options:
  --duration S          Input-check duration, default ${duration}
  --udp-port N          UDP port, default: choose a free local port
  --beyond-yaml P       BeyondMimic YAML, default ${beyond_yaml}
  --track-mimic-yaml P  SKILL/TrackMimic BeyondMimic trajectory YAML, default ${track_mimic_yaml}
  --keep-log            Print and keep temp log paths
  -h, --help            Show this help
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
        --beyond-yaml)
            beyond_yaml="$2"
            shift 2
            ;;
        --track-mimic-yaml)
            track_mimic_yaml="$2"
            shift 2
            ;;
        --keep-log)
            keep_log=1
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "[Smoke][ERROR] unknown argument: $1" >&2
            usage >&2
            exit 1
            ;;
    esac
done

for tool in python3 ss rg; do
    if ! command -v "${tool}" >/dev/null 2>&1; then
        echo "[Smoke][ERROR] required tool not found: ${tool}" >&2
        exit 1
    fi
done

registry_header="${PROJECT_ROOT}/controller_cpp/include/native_external_policy_registry.h"
python3 - "${registry_header}" <<'PY'
import pathlib
import sys

header = pathlib.Path(sys.argv[1])
src = header.read_text()
start = src.find("void register_track_mimic")
if start < 0:
    print("[Smoke][ERROR] NativeBeyondMimicExternalPolicyRegistry is missing register_track_mimic", file=sys.stderr)
    sys.exit(1)
brace = src.find("{", start)
if brace < 0:
    print("[Smoke][ERROR] register_track_mimic body not found", file=sys.stderr)
    sys.exit(1)

depth = 0
end = -1
for idx in range(brace, len(src)):
    if src[idx] == "{":
        depth += 1
    elif src[idx] == "}":
        depth -= 1
        if depth == 0:
            end = idx + 1
            break
if end < 0:
    print("[Smoke][ERROR] register_track_mimic body is not balanced", file=sys.stderr)
    sys.exit(1)

body = src[brace:end]
required = [
    "std::make_unique<::BeyondMimicPolicy>",
    "::FSMStateName::SKILL_TRACK_MIMIC",
    "ControlMode::Skill",
    "kTrackMimicPolicyKey",
    "/*require_motion_file=*/true",
    "core.register_external_policy(kTrackMimicPolicyKey",
]
missing = [item for item in required if item not in body]
if missing:
    print(
        "[Smoke][ERROR] TrackMimic must stay a SKILL-keyed BeyondMimic trajectory path; missing: "
        + ", ".join(missing),
        file=sys.stderr,
    )
    sys.exit(1)
if "std::make_unique<::TrackMimicPolicy>" in body or "class TrackMimicPolicy" in src:
    print("[Smoke][ERROR] TrackMimic was split into a separate policy implementation", file=sys.stderr)
    sys.exit(1)
PY

if [[ ! -f "${beyond_yaml}" ]]; then
    echo "[Smoke][ERROR] BeyondMimic YAML not found: ${beyond_yaml}" >&2
    exit 1
fi
if [[ ! -f "${track_mimic_yaml}" ]]; then
    echo "[Smoke][ERROR] BeyondMimic trajectory/TrackMimic YAML not found: ${track_mimic_yaml}" >&2
    exit 1
fi

track_tmp_dir="$(mktemp -d /tmp/magicbot_track_mimic_XXXXXX)"
track_mimic_runtime_yaml="${track_tmp_dir}/BeyondMimic.yaml"
python3 "${SCRIPT_DIR}/make_track_mimic_motion_yaml.py" \
    --base-yaml "${track_mimic_yaml}" \
    --output-yaml "${track_mimic_runtime_yaml}" \
    >/dev/null

if [[ -z "${udp_port}" ]]; then
    udp_port="$(python3 - <<'PY'
import socket

with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
    sock.bind(("127.0.0.1", 0))
    print(sock.getsockname()[1])
PY
)"
fi

dry_log="$(mktemp /tmp/magicbot_loco_external_dry_XXXXXX.log)"
blocked_log="$(mktemp /tmp/magicbot_loco_external_blocked_XXXXXX.log)"
input_log="$(mktemp /tmp/magicbot_loco_external_input_XXXXXX.log)"
runner_pid=""

cleanup() {
    if [[ -n "${runner_pid}" ]] && kill -0 "${runner_pid}" >/dev/null 2>&1; then
        kill "${runner_pid}" >/dev/null 2>&1 || true
        wait "${runner_pid}" >/dev/null 2>&1 || true
    fi
    if [[ "${keep_log}" -eq 0 ]]; then
        rm -f "${dry_log}" "${blocked_log}" "${input_log}"
    fi
    rm -rf "${track_tmp_dir}"
}
trap cleanup EXIT

wait_for_udp_ready() {
    local port="$1"
    local log_path="$2"
    local label="$3"
    local ready=0
    for _ in $(seq 1 100); do
        if ss -lun | rg -q ":${port}\\b"; then
            ready=1
            break
        fi
        if ! kill -0 "${runner_pid}" >/dev/null 2>&1; then
            echo "[Smoke][ERROR] ${label} exited before UDP was ready" >&2
            sed -n '1,220p' "${log_path}" >&2
            exit 1
        fi
        sleep 0.1
    done
    if [[ "${ready}" -ne 1 ]]; then
        echo "[Smoke][ERROR] timed out waiting for UDP port ${port}" >&2
        sed -n '1,220p' "${log_path}" >&2
        exit 1
    fi
}

echo "[Smoke] Checking dry-run external policy YAML loading"
"${RUNNER}" \
    --dry-run \
    --beyond-yaml "${beyond_yaml}" \
    --track-mimic-yaml "${track_mimic_runtime_yaml}" \
    >"${dry_log}" 2>&1

for expected in '[DryRun] BeyondMimic loaded' '[DryRun] BeyondMimic trajectory/TrackMimic key loaded'; do
    if ! rg -F -q "${expected}" "${dry_log}"; then
        echo "[Smoke][ERROR] dry-run output missing: ${expected}" >&2
        sed -n '1,220p' "${dry_log}" >&2
        exit 1
    fi
done

echo "[Smoke] Checking external-policy YAML does not bypass allow gates"
"${RUNNER}" \
    --input-check \
    --udp-control \
    --udp-bind 127.0.0.1 \
    --udp-port "${udp_port}" \
    --duration 1.2 \
    --log-interval 0.3 \
    --beyond-yaml "${beyond_yaml}" \
    --track-mimic-yaml "${track_mimic_runtime_yaml}" \
    >"${blocked_log}" 2>&1 &
runner_pid=$!

wait_for_udp_ready "${udp_port}" "${blocked_log}" "blocked external-policy input-check"

python3 - <<PY
import socket
import time

port = int("${udp_port}")
packets = [
    b"mode=beyond",
    b"mode=track_mimic",
]

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
for packet in packets:
    sock.sendto(packet, ("127.0.0.1", port))
    time.sleep(0.25)
PY

if ! wait "${runner_pid}"; then
    echo "[Smoke][ERROR] blocked input-check exited with failure" >&2
    sed -n '1,260p' "${blocked_log}" >&2
    exit 1
fi
runner_pid=""

for expected in 'DANCE ignored; add --allow-dance' 'SKILL ignored; add --allow-skill'; do
    if ! rg -q "${expected}" "${blocked_log}"; then
        echo "[Smoke][ERROR] missing expected blocked external-policy output: ${expected}" >&2
        sed -n '1,260p' "${blocked_log}" >&2
        exit 1
    fi
done
for forbidden in 'mode=DANCE' 'mode=SKILL'; do
    if rg -q "${forbidden}" "${blocked_log}"; then
        echo "[Smoke][ERROR] external policy entered without allow gate: ${forbidden}" >&2
        sed -n '1,260p' "${blocked_log}" >&2
        exit 1
    fi
done

echo "[Smoke] Starting allowed external-policy input-check on UDP 127.0.0.1:${udp_port}"
"${RUNNER}" \
    --input-check \
    --udp-control \
    --udp-bind 127.0.0.1 \
    --udp-port "${udp_port}" \
    --duration "${duration}" \
    --log-interval 0.3 \
    --allow-dance \
    --allow-skill \
    --beyond-yaml "${beyond_yaml}" \
    --track-mimic-yaml "${track_mimic_runtime_yaml}" \
    >"${input_log}" 2>&1 &
runner_pid=$!

wait_for_udp_ready "${udp_port}" "${input_log}" "allowed external-policy input-check"

python3 - <<PY
import socket
import time

port = int("${udp_port}")
packets = [
    b"mode=beyond",
    b"mode=track_mimic",
    b"mode=final_damping",
]

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
for packet in packets:
    sock.sendto(packet, ("127.0.0.1", port))
    time.sleep(0.3)
PY

if ! wait "${runner_pid}"; then
    echo "[Smoke][ERROR] input-check exited with failure" >&2
    sed -n '1,260p' "${input_log}" >&2
    exit 1
fi
runner_pid=""

for expected in 'mode=DANCE' 'mode=SKILL' 'mode=FINAL_DAMPING'; do
    if ! rg -q "${expected}" "${input_log}"; then
        echo "[Smoke][ERROR] missing expected input-check output: ${expected}" >&2
        sed -n '1,260p' "${input_log}" >&2
        exit 1
    fi
done

for blocked in 'DANCE ignored' 'SKILL ignored'; do
    if rg -q "${blocked}" "${input_log}"; then
        echo "[Smoke][ERROR] external request was unexpectedly blocked: ${blocked}" >&2
        sed -n '1,260p' "${input_log}" >&2
        exit 1
    fi
done

echo "[Smoke] PASSED real-runner external-policy input gates"
if [[ "${keep_log}" -eq 1 ]]; then
    echo "[Smoke] dry_log=${dry_log}"
    echo "[Smoke] blocked_log=${blocked_log}"
    echo "[Smoke] input_log=${input_log}"
fi
rg 'DryRun|DANCE ignored|SKILL ignored|mode=(DANCE|SKILL|FINAL_DAMPING)' "${dry_log}" "${blocked_log}" "${input_log}"
