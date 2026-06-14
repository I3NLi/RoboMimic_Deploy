#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
RUNNER="${SCRIPT_DIR}/run_dual_inference_rate_native.sh"
DUAL_SOURCE="${SCRIPT_DIR}/../controller_cpp/src/dual_inference_rate.cpp"
RUNNER_WRAPPER="${SCRIPT_DIR}/run_dual_inference_rate_native.sh"

duration="1.0"
summary_json=""
keep_summary=0
push_body="pelvis"
push_force="35,0,0"
push_start="0.30"
push_duration="0.12"
push_impulse="0,1.0,0"
push_impulse_time="0.55"
extra_args=()

usage() {
    cat <<EOF
Usage: $0 [options] [-- extra dual_inference_rate args]

Smoke-test pure-sim closed-loop control with a scheduled force and impulse
disturbance. The script runs dual_inference_rate, then validates the summary
JSON to ensure both disturbance paths were applied.

Options:
  --duration S          Run duration, default ${duration}
  --summary-json P      Summary JSON path, default: temp file under /tmp
  --keep-summary        Keep the temp summary path printed at the end
  --push-body NAME      MuJoCo body to perturb, default ${push_body}
  --push-force X,Y,Z    Force vector, default ${push_force}
  --push-start S        Force start time, default ${push_start}
  --push-duration S     Force duration, default ${push_duration}
  --push-impulse X,Y,Z  Impulse vector, default ${push_impulse}
  --push-impulse-time S Impulse time, default ${push_impulse_time}
  -h, --help            Show this help
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --duration)
            duration="$2"
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
        --push-body)
            push_body="$2"
            shift 2
            ;;
        --push-force)
            push_force="$2"
            shift 2
            ;;
        --push-start)
            push_start="$2"
            shift 2
            ;;
        --push-duration)
            push_duration="$2"
            shift 2
            ;;
        --push-impulse)
            push_impulse="$2"
            shift 2
            ;;
        --push-impulse-time)
            push_impulse_time="$2"
            shift 2
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

for tool in jq rg; do
    if ! command -v "${tool}" >/dev/null 2>&1; then
        echo "[Smoke][ERROR] required tool not found: ${tool}" >&2
        exit 1
    fi
done

echo "[Smoke] Checking dual-rate sim writes through adapter boundary"
if rg -n 'data->ctrl\[[^]]+\]\s*=' "${DUAL_SOURCE}"; then
    echo "[Smoke][ERROR] dual_inference_rate.cpp must not write MuJoCo ctrl directly; use MujocoSimAdapter" >&2
    exit 1
fi
if rg -n 'OnnxLocoPolicy|\.infer\(|MotionSafety|safety\.check|torque_limited_target|clamp_and_rate_limit' "${DUAL_SOURCE}"; then
    echo "[Smoke][ERROR] dual_inference_rate.cpp must keep policy, safety, and target limiting inside ControllerCore" >&2
    exit 1
fi
if rg -n 'sim_adapter\.write_target' "${DUAL_SOURCE}"; then
    echo "[Smoke][ERROR] dual_inference_rate.cpp should publish held sim targets through ControllerRuntime" >&2
    exit 1
fi

echo "[Smoke] Checking dual-rate wrapper rebuilds on shared headers"
python3 - "${RUNNER_WRAPPER}" <<'PY'
import sys
from pathlib import Path

wrapper = Path(sys.argv[1]).read_text(encoding="utf-8")
required = [
    "needs_build()",
    'find "${CPP_DIR}/include"',
    "-name '*.h'",
    "-name '*.hpp'",
    "src/dual_inference_rate.cpp",
    "src/magicbot_loco_core.cpp",
    "src/magicbot_loco_sdk_adapter.cpp",
    '[[ "${DUAL_RATE_SKIP_BUILD:-0}" != "1" ]] || needs_build',
]
missing = [item for item in required if item not in wrapper]
if missing:
    print(
        "[Smoke][ERROR] dual-rate wrapper must rebuild when shared headers or target sources change; missing: "
        + ", ".join(missing),
        file=sys.stderr,
    )
    sys.exit(1)
PY

if [[ -z "${summary_json}" ]]; then
    summary_json="$(mktemp /tmp/magicbot_dual_push_XXXXXX.json)"
fi

cleanup() {
    if [[ "${keep_summary}" -eq 0 ]]; then
        rm -f "${summary_json}"
    fi
}
trap cleanup EXIT

echo "[Smoke] Starting dual-rate scheduled push smoke"
set +e
"${RUNNER}" \
    --mode pure-sim \
    --duration "${duration}" \
    --no-realtime \
    --closed-loop-check \
    --push-body "${push_body}" \
    --push-force "${push_force}" \
    --push-start "${push_start}" \
    --push-duration "${push_duration}" \
    --push-impulse "${push_impulse}" \
    --push-impulse-time "${push_impulse_time}" \
    --summary-json "${summary_json}" \
    "${extra_args[@]}"
rc=$?
set -e

if [[ "${rc}" -ne 0 ]]; then
    echo "[Smoke][ERROR] dual_inference_rate exited with ${rc}" >&2
    if [[ -s "${summary_json}" ]]; then
        cat "${summary_json}" >&2
    fi
    exit "${rc}"
fi

if [[ ! -s "${summary_json}" ]]; then
    echo "[Smoke][ERROR] summary JSON was not written: ${summary_json}" >&2
    exit 1
fi

pass="$(jq -r '.pass' "${summary_json}")"
sim_steps="$(jq -r '.sim_steps' "${summary_json}")"
control_steps="$(jq -r '.control_steps' "${summary_json}")"
push_enabled="$(jq -r '.push_enabled' "${summary_json}")"
push_force_steps="$(jq -r '.push_force_steps' "${summary_json}")"
push_impulse_applied="$(jq -r '.push_impulse_applied' "${summary_json}")"
push_force_norm="$(jq -r '.push_force_norm' "${summary_json}")"
push_impulse_norm="$(jq -r '.push_impulse_norm' "${summary_json}")"
expected_force_json="$(jq -cn --arg value "${push_force}" '$value | split(",") | map(tonumber)')"
expected_impulse_json="$(jq -cn --arg value "${push_impulse}" '$value | split(",") | map(tonumber)')"

if [[ "${pass}" != "true" ]]; then
    echo "[Smoke][ERROR] closed-loop check did not pass" >&2
    cat "${summary_json}" >&2
    exit 1
fi
if [[ "${sim_steps}" -le 0 || "${control_steps}" -le 0 ]]; then
    echo "[Smoke][ERROR] expected sim/control to advance, got sim_steps=${sim_steps} control_steps=${control_steps}" >&2
    cat "${summary_json}" >&2
    exit 1
fi
if [[ "${push_enabled}" != "true" ]]; then
    echo "[Smoke][ERROR] expected push_enabled=true" >&2
    cat "${summary_json}" >&2
    exit 1
fi
if [[ "${push_force_steps}" -le 0 ]]; then
    echo "[Smoke][ERROR] expected push_force_steps > 0, got ${push_force_steps}" >&2
    cat "${summary_json}" >&2
    exit 1
fi
if [[ "${push_impulse_applied}" != "true" ]]; then
    echo "[Smoke][ERROR] expected push_impulse_applied=true" >&2
    cat "${summary_json}" >&2
    exit 1
fi
if ! jq -e '.push_force_norm > 0 and .push_impulse_norm > 0' "${summary_json}" >/dev/null; then
    echo "[Smoke][ERROR] expected non-zero force and impulse norms, got force=${push_force_norm} impulse=${push_impulse_norm}" >&2
    cat "${summary_json}" >&2
    exit 1
fi
if ! jq -e \
    --arg expected_body "${push_body}" \
    --argjson expected_force "${expected_force_json}" \
    --argjson expected_impulse "${expected_impulse_json}" \
    --argjson expected_start "${push_start}" \
    --argjson expected_duration "${push_duration}" \
    --argjson expected_impulse_time "${push_impulse_time}" \
    '.push_body == $expected_body
     and .push_body_id >= 0
     and .push_body_resolved == $expected_body
     and .push_force == $expected_force
     and .push_impulse == $expected_impulse
     and .push_start_s == $expected_start
     and .push_duration_s == $expected_duration
     and .push_impulse_time_s == $expected_impulse_time' \
     "${summary_json}" >/dev/null; then
    echo "[Smoke][ERROR] expected configured push vectors/timing in summary" >&2
    cat "${summary_json}" >&2
    exit 1
fi

echo "[Smoke] PASSED dual-rate scheduled push"
echo "[Smoke] summary=${summary_json}"
jq '{pass, sim_steps, control_steps, push_body, push_body_id, push_body_resolved, push_enabled, push_force, push_impulse, push_start_s, push_duration_s, push_impulse_time_s, push_force_steps, push_impulse_applied, push_force_norm, push_impulse_norm}' "${summary_json}"
