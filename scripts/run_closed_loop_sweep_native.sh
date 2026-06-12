#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
PROJECT_ROOT="$( cd "${SCRIPT_DIR}/.." &> /dev/null && pwd )"
RUNNER="${SCRIPT_DIR}/run_dual_inference_rate_native.sh"

duration="2"
axis="vx"
summary_dir="${PROJECT_ROOT}/logs/closed_loop_sweep"
values=("-1" "-0.75" "-0.5" "-0.25" "0" "0.25" "0.5" "0.75" "1")
keep_going=0
extra_args=()

usage() {
    cat <<EOF
Usage: $0 [options] [-- extra dual_inference_rate args]

Closed-loop command sweep. Values are normalized; loco YAML cmd_range maps
linear speed, so vx=1 maps to the configured positive lin_vel_x cap.

Options:
  --duration S       Per-point duration, default ${duration}
  --axis vx|vy|wz    Command axis to sweep, default ${axis}
  --values "LIST"    Space-separated normalized values
  --summary-dir DIR  Output directory for per-point JSON summaries
  --keep-going       Run all points even after failures
  -h, --help         Show this help
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --duration)
            duration="$2"
            shift 2
            ;;
        --axis)
            axis="$2"
            shift 2
            ;;
        --values)
            read -r -a values <<< "$2"
            shift 2
            ;;
        --summary-dir)
            summary_dir="$2"
            shift 2
            ;;
        --keep-going)
            keep_going=1
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

case "${axis}" in
    vx|vy|wz) ;;
    *)
        echo "[Error] --axis must be vx, vy, or wz" >&2
        exit 1
        ;;
esac

mkdir -p "${summary_dir}"

failures=0
built=0
for value in "${values[@]}"; do
    safe_value="${value//-/neg}"
    safe_value="${safe_value//./p}"
    summary_json="${summary_dir}/${axis}_${safe_value}.json"
    echo "[Sweep] ${axis}=${value} summary=${summary_json}"
    cmd=(
        "${RUNNER}"
        --mode pure-sim
        --duration "${duration}"
        --no-realtime
        --closed-loop-check
        --summary-json "${summary_json}"
        "--${axis}" "${value}"
    )
    if [[ "${built}" -eq 0 ]]; then
        DUAL_RATE_SKIP_BUILD=0 "${cmd[@]}" "${extra_args[@]}" && rc=0 || rc=$?
        built=1
    else
        DUAL_RATE_SKIP_BUILD=1 "${cmd[@]}" "${extra_args[@]}" && rc=0 || rc=$?
    fi
    if [[ "${rc}" -ne 0 ]]; then
        failures=$((failures + 1))
        if [[ "${keep_going}" -eq 0 ]]; then
            echo "[Sweep] FAILED at ${axis}=${value}" >&2
            exit 2
        fi
    fi
done

if [[ "${failures}" -gt 0 ]]; then
    echo "[Sweep] FAILED ${failures}/${#values[@]} points" >&2
    exit 2
fi

echo "[Sweep] PASSED ${#values[@]} points"
