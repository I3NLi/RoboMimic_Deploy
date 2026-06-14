#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
RUNNER="${SCRIPT_DIR}/run_magicbot_loco_native.sh"
RUNNER_SOURCE="${SCRIPT_DIR}/../controller_cpp/src/magicbot_z1_loco_onnx.cpp"

keep_log=0

usage() {
    cat <<EOF
Usage: $0 [--keep-log]

Smoke-test the real runner safety gates without connecting to a robot. The
script verifies dry-run policy loading, the default dry-run mode, and that
--run is rejected unless the explicit high-risk --allow-loco gate is present.
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
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

for tool in rg; do
    if ! command -v "${tool}" >/dev/null 2>&1; then
        echo "[Smoke][ERROR] required tool not found: ${tool}" >&2
        exit 1
    fi
done

echo "[Smoke] Checking real runner writes through adapter boundary"
if rg -n '\brobot\.publish_(sdk24_command|damping)\(' "${RUNNER_SOURCE}"; then
    echo "[Smoke][ERROR] magicbot_z1_loco_onnx.cpp must not publish robot commands directly; use MagicbotRealAdapter" >&2
    exit 1
fi

dry_log="$(mktemp /tmp/magicbot_loco_safety_dry_XXXXXX.log)"
default_log="$(mktemp /tmp/magicbot_loco_safety_default_XXXXXX.log)"
blocked_log="$(mktemp /tmp/magicbot_loco_safety_blocked_XXXXXX.log)"
exclusive_log="$(mktemp /tmp/magicbot_loco_safety_exclusive_XXXXXX.log)"
input_source_log="$(mktemp /tmp/magicbot_loco_safety_input_source_XXXXXX.log)"
live_input_log="$(mktemp /tmp/magicbot_loco_safety_live_input_XXXXXX.log)"

cleanup() {
    if [[ "${keep_log}" -eq 0 ]]; then
        rm -f "${dry_log}" "${default_log}" "${blocked_log}" \
            "${exclusive_log}" "${input_source_log}" "${live_input_log}"
    fi
}
trap cleanup EXIT

assert_no_robot_path() {
    local log_path="$1"
    if rg -q '\[MagicBot\]|\[ConnectCheck\]|\[ReadState\]|\[State\] Robot state ready|\[Stage\]|\[Final\]|MagicBot SDK initialize failed|SetMotionControlLevel|LowLevel controller initialize' "${log_path}"; then
        echo "[Smoke][ERROR] blocked command appears to have entered robot connection path" >&2
        sed -n '1,180p' "${log_path}" >&2
        exit 1
    fi
}

echo "[Smoke] Checking explicit dry-run"
"${RUNNER}" --dry-run >"${dry_log}" 2>&1
for expected in 'Config:' 'ONNX input/output:' 'Action sample range:' 'Target sample range:'; do
    if ! rg -q "${expected}" "${dry_log}"; then
        echo "[Smoke][ERROR] dry-run output missing: ${expected}" >&2
        sed -n '1,180p' "${dry_log}" >&2
        exit 1
    fi
done

echo "[Smoke] Checking default mode is dry-run"
"${RUNNER}" >"${default_log}" 2>&1
if ! rg -q 'ONNX input/output:' "${default_log}"; then
    echo "[Smoke][ERROR] default invocation did not run dry-run" >&2
    sed -n '1,180p' "${default_log}" >&2
    exit 1
fi

echo "[Smoke] Checking --run is blocked without --allow-loco"
if "${RUNNER}" --run --duration 0.01 >"${blocked_log}" 2>&1; then
    echo "[Smoke][ERROR] --run succeeded without --allow-loco" >&2
    sed -n '1,180p' "${blocked_log}" >&2
    exit 1
fi
if ! rg -q 'refusing ONNX loco without --allow-loco' "${blocked_log}"; then
    echo "[Smoke][ERROR] --run failure did not report allow-loco gate" >&2
    sed -n '1,180p' "${blocked_log}" >&2
    exit 1
fi
assert_no_robot_path "${blocked_log}"

echo "[Smoke] Checking main modes are mutually exclusive before connection"
if "${RUNNER}" --connect-check --read-state >"${exclusive_log}" 2>&1; then
    echo "[Smoke][ERROR] mutually exclusive modes unexpectedly succeeded" >&2
    sed -n '1,180p' "${exclusive_log}" >&2
    exit 1
fi
if ! rg -q 'use only one of --dry-run, --connect-check, --read-state, --run, --input-check, --debug-entry-only' "${exclusive_log}"; then
    echo "[Smoke][ERROR] exclusive-mode failure did not report mode gate" >&2
    sed -n '1,180p' "${exclusive_log}" >&2
    exit 1
fi
assert_no_robot_path "${exclusive_log}"

echo "[Smoke] Checking --input-check requires an input source"
if "${RUNNER}" --input-check >"${input_source_log}" 2>&1; then
    echo "[Smoke][ERROR] --input-check without input source unexpectedly succeeded" >&2
    sed -n '1,180p' "${input_source_log}" >&2
    exit 1
fi
if ! rg -q -- '--input-check requires --keyboard-control, --gamepad-control or --udp-control' "${input_source_log}"; then
    echo "[Smoke][ERROR] input-check failure did not report input-source gate" >&2
    sed -n '1,180p' "${input_source_log}" >&2
    exit 1
fi
assert_no_robot_path "${input_source_log}"

echo "[Smoke] Checking live input sources are mutually exclusive before connection"
if "${RUNNER}" --run --allow-loco --keyboard-control --udp-control --duration 0.01 >"${live_input_log}" 2>&1; then
    echo "[Smoke][ERROR] multiple live input sources unexpectedly succeeded" >&2
    sed -n '1,180p' "${live_input_log}" >&2
    exit 1
fi
if ! rg -q -- 'use only one of --keyboard-control, --gamepad-control or --udp-control' "${live_input_log}"; then
    echo "[Smoke][ERROR] live-input failure did not report exclusivity gate" >&2
    sed -n '1,180p' "${live_input_log}" >&2
    exit 1
fi
assert_no_robot_path "${live_input_log}"

echo "[Smoke] PASSED real-runner safety gates"
if [[ "${keep_log}" -eq 1 ]]; then
    echo "[Smoke] dry_log=${dry_log}"
    echo "[Smoke] default_log=${default_log}"
    echo "[Smoke] blocked_log=${blocked_log}"
    echo "[Smoke] exclusive_log=${exclusive_log}"
    echo "[Smoke] input_source_log=${input_source_log}"
    echo "[Smoke] live_input_log=${live_input_log}"
fi
