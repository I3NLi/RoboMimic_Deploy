#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
PROJECT_ROOT="$( cd "${SCRIPT_DIR}/.." &> /dev/null && pwd )"
CPP_DIR="${PROJECT_ROOT}/controller_cpp"
DEFAULT_CONFIG="${PROJECT_ROOT}/policies/loco_mode/config/LocoMode_lowKp.yaml"
CONFIG="${DEFAULT_CONFIG}"
CXX_BIN="${CXX:-c++}"
BUILD_DIR="$(mktemp -d /tmp/magicbot_controller_core_check_XXXXXX)"

usage() {
    cat <<EOF
Usage: $0 [--config PATH]

Build and run the shared ControllerCore check. The check loads the loco config
and policy, then verifies STAND, PASSIVE, and FINAL_DAMPING mode outputs without
connecting to MuJoCo or the real robot.
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --config)
            CONFIG="$2"
            shift 2
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "[Error] unknown argument: $1" >&2
            usage >&2
            exit 1
            ;;
    esac
done

cleanup() {
    rm -rf "${BUILD_DIR}"
}
trap cleanup EXIT

if ! command -v "${CXX_BIN}" >/dev/null 2>&1; then
    echo "[Error] C++ compiler not found: ${CXX_BIN}" >&2
    exit 1
fi

if [[ ! -f "${CONFIG}" ]]; then
    echo "[Error] config not found: ${CONFIG}" >&2
    exit 1
fi

ONNXRUNTIME_DIR="${ONNXRUNTIME_DIR:-/home/hiyio/unitree_rl_lab/deploy/thirdparty/onnxruntime-linux-x64-1.22.0}"
ONNXRUNTIME_INCLUDE_DIR="${ONNXRUNTIME_INCLUDE_DIR:-${ONNXRUNTIME_DIR}/include}"
ONNXRUNTIME_LIB="${ONNXRUNTIME_LIB:-}"

if [[ ! -f "${ONNXRUNTIME_INCLUDE_DIR}/onnxruntime_cxx_api.h" ]]; then
    for candidate in \
        "${PROJECT_ROOT}/third_party/onnxruntime/include" \
        "${HOME}/onnxruntime/include" \
        "${HOME}/.local/include/onnxruntime"; do
        if [[ -f "${candidate}/onnxruntime_cxx_api.h" ]]; then
            ONNXRUNTIME_INCLUDE_DIR="${candidate}"
            break
        fi
    done
fi

if [[ -z "${ONNXRUNTIME_LIB}" ]]; then
    if [[ -f "${ONNXRUNTIME_DIR}/lib/libonnxruntime.so.1" ]]; then
        ONNXRUNTIME_LIB="${ONNXRUNTIME_DIR}/lib/libonnxruntime.so.1"
    elif [[ -f "${ONNXRUNTIME_DIR}/lib/libonnxruntime.so" ]]; then
        ONNXRUNTIME_LIB="${ONNXRUNTIME_DIR}/lib/libonnxruntime.so"
    else
        while IFS= read -r candidate; do
            if [[ -f "${candidate}" ]]; then
                ONNXRUNTIME_LIB="${candidate}"
                break
            fi
        done < <(find "${HOME}" -path '*onnxruntime/capi/libonnxruntime.so*' -type f 2>/dev/null | sort)
    fi
fi

if [[ ! -f "${ONNXRUNTIME_INCLUDE_DIR}/onnxruntime_cxx_api.h" ]]; then
    echo "[Error] ONNXRuntime headers not found. Set ONNXRUNTIME_INCLUDE_DIR." >&2
    exit 1
fi

if [[ ! -f "${ONNXRUNTIME_LIB}" ]]; then
    echo "[Error] ONNXRuntime library not found. Set ONNXRUNTIME_LIB." >&2
    exit 1
fi

mkdir -p "${BUILD_DIR}/onnxruntime"
ORT_LINK_LIB="${BUILD_DIR}/onnxruntime/libonnxruntime.so.1"
ln -sf "${ONNXRUNTIME_LIB}" "${ORT_LINK_LIB}"

"${CXX_BIN}" \
    -std=c++17 \
    -O2 \
    -Wall \
    -Wextra \
    -I"${CPP_DIR}/include" \
    -I"${ONNXRUNTIME_INCLUDE_DIR}" \
    "${CPP_DIR}/src/controller_core_check.cpp" \
    "${CPP_DIR}/src/magicbot_loco_core.cpp" \
    "${ORT_LINK_LIB}" \
    -lyaml-cpp \
    -pthread \
    -o "${BUILD_DIR}/controller_core_check"

LD_LIBRARY_PATH="${BUILD_DIR}/onnxruntime:${LD_LIBRARY_PATH:-}" \
    "${BUILD_DIR}/controller_core_check" "${CONFIG}"
