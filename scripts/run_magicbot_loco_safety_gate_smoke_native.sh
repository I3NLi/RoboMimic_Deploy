#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
RUNNER="${SCRIPT_DIR}/run_magicbot_loco_native.sh"

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

dry_log="$(mktemp /tmp/magicbot_loco_safety_dry_XXXXXX.log)"
default_log="$(mktemp /tmp/magicbot_loco_safety_default_XXXXXX.log)"
blocked_log="$(mktemp /tmp/magicbot_loco_safety_blocked_XXXXXX.log)"

cleanup() {
    if [[ "${keep_log}" -eq 0 ]]; then
        rm -f "${dry_log}" "${default_log}" "${blocked_log}"
    fi
}
trap cleanup EXIT

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
if rg -q 'initialize|connect|LowLevel|Robot state ready' "${blocked_log}"; then
    echo "[Smoke][ERROR] blocked --run appears to have entered robot connection path" >&2
    sed -n '1,180p' "${blocked_log}" >&2
    exit 1
fi

echo "[Smoke] PASSED real-runner safety gates"
if [[ "${keep_log}" -eq 1 ]]; then
    echo "[Smoke] dry_log=${dry_log}"
    echo "[Smoke] default_log=${default_log}"
    echo "[Smoke] blocked_log=${blocked_log}"
fi
