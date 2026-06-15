#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

RUN_CORE=1
RUN_SIM=1
RUN_VIEWER=1
RUN_PYTHON=1
RUN_REAL_NO_ROBOT=1

usage() {
    cat <<EOF
Usage: $0 [options]

Run the no-hardware shared-runtime smoke suite. The suite composes existing
focused smoke scripts; it does not implement control, policy, mode, or safety
logic itself.

Default groups:
  core            text parser, mode manager, ControllerCore/ControllerRuntime
  sim             sim-adapter target modes plus pure-sim closed-loop scheduled force + impulse
  viewer          native viewer HTTP/UDP/mode/external-policy/perturb controls
  python          Python-facing viewer launcher over the same native runtime
  real-no-robot   real-adapter target modes plus real-runner dry/input/safety gates without connecting robot

Options:
  --core-only       Run only core checks
  --sim-only        Run only pure-sim checks
  --viewer-only     Run only native viewer checks
  --python-only     Run only Python-facing viewer checks
  --real-only       Run only no-robot real-runner checks
  --no-core         Skip core checks
  --no-sim          Skip pure-sim checks
  --no-viewer       Skip native viewer checks
  --no-python       Skip Python-facing viewer checks
  --no-real         Skip no-robot real-runner checks
  -h, --help        Show this help
EOF
}

only_group() {
    RUN_CORE=0
    RUN_SIM=0
    RUN_VIEWER=0
    RUN_PYTHON=0
    RUN_REAL_NO_ROBOT=0
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --core-only)
            only_group
            RUN_CORE=1
            shift
            ;;
        --sim-only)
            only_group
            RUN_SIM=1
            shift
            ;;
        --viewer-only)
            only_group
            RUN_VIEWER=1
            shift
            ;;
        --python-only)
            only_group
            RUN_PYTHON=1
            shift
            ;;
        --real-only)
            only_group
            RUN_REAL_NO_ROBOT=1
            shift
            ;;
        --no-core)
            RUN_CORE=0
            shift
            ;;
        --no-sim)
            RUN_SIM=0
            shift
            ;;
        --no-viewer)
            RUN_VIEWER=0
            shift
            ;;
        --no-python)
            RUN_PYTHON=0
            shift
            ;;
        --no-real)
            RUN_REAL_NO_ROBOT=0
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "[Suite][ERROR] unknown argument: $1" >&2
            usage >&2
            exit 1
            ;;
    esac
done

run_step() {
    echo
    echo "[Suite] $*"
    "$@"
}

if [[ "${RUN_CORE}" -eq 1 ]]; then
    run_step "${SCRIPT_DIR}/run_text_control_parser_check_native.sh"
    run_step "${SCRIPT_DIR}/run_mode_manager_check_native.sh"
    run_step "${SCRIPT_DIR}/run_controller_core_check_native.sh"
fi

if [[ "${RUN_SIM}" -eq 1 ]]; then
    run_step "${SCRIPT_DIR}/run_sim_adapter_target_mode_check_native.sh"
    run_step "${SCRIPT_DIR}/run_dual_push_smoke_native.sh" --duration 1.0
fi

if [[ "${RUN_VIEWER}" -eq 1 ]]; then
    run_step "${SCRIPT_DIR}/run_viewer_stream_smoke_native.sh" --duration 1.5
    run_step "${SCRIPT_DIR}/run_viewer_http_control_smoke_native.sh" --duration 1.5
    run_step "${SCRIPT_DIR}/run_viewer_udp_control_smoke_native.sh" --duration 1.5
    run_step "${SCRIPT_DIR}/run_viewer_http_dance_smoke_native.sh" --duration 0.8
    run_step "${SCRIPT_DIR}/run_viewer_http_skill_smoke_native.sh" --duration 0.8
    run_step "${SCRIPT_DIR}/run_viewer_udp_external_policy_smoke_native.sh" --duration 1.8
    run_step "${SCRIPT_DIR}/run_viewer_gamepad_control_smoke_native.sh" --duration 1.5
    run_step "${SCRIPT_DIR}/run_viewer_control_station_smoke_native.sh" --duration 2.0
    run_step "${SCRIPT_DIR}/run_viewer_http_perturb_smoke_native.sh" --duration 1.5
fi

if [[ "${RUN_PYTHON}" -eq 1 ]]; then
    run_step "${SCRIPT_DIR}/run_python_mujoco_viewer_smoke_native.sh"
fi

if [[ "${RUN_REAL_NO_ROBOT}" -eq 1 ]]; then
    run_step "${SCRIPT_DIR}/run_real_adapter_target_mode_check_native.sh"
    run_step "${SCRIPT_DIR}/run_magicbot_loco_safety_gate_smoke_native.sh"
    run_step "${SCRIPT_DIR}/run_magicbot_loco_input_check_smoke_native.sh"
    run_step "${SCRIPT_DIR}/run_magicbot_loco_gamepad_input_smoke_native.sh"
    run_step "${SCRIPT_DIR}/run_magicbot_loco_external_policy_smoke_native.sh"
fi

echo
echo "[Suite] PASSED shared-runtime no-hardware smoke suite"
