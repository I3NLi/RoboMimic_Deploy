#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
PROJECT_ROOT="$( cd "${SCRIPT_DIR}/.." &> /dev/null && pwd )"
CPP_DIR="${PROJECT_ROOT}/controller_cpp"
BUILD_DIR="${CPP_DIR}/build_dual_rate"
NATIVE_BIN="${BUILD_DIR}/dual_inference_rate"
DEFAULT_CONFIG="${PROJECT_ROOT}/policies/loco_mode/config/LocoMode_lowKp.yaml"

has_arg() {
    local needle="$1"
    shift
    local arg
    for arg in "$@"; do
        [[ "${arg}" == "${needle}" ]] && return 0
    done
    return 1
}

args=("$@")
if ! has_arg "--config" "${args[@]}"; then
    args=(--config "${DEFAULT_CONFIG}" "${args[@]}")
fi
if ! has_arg "--mode" "${args[@]}"; then
    args=(--mode pure-sim "${args[@]}")
fi

MAGICBOT_Z1_SDK_ROOT="${MAGICBOT_Z1_SDK_ROOT:-${MAGICBOT_SDK_ROOT:-}}"
if [[ -z "${MAGICBOT_Z1_SDK_ROOT}" ]]; then
    for candidate in \
        "${HOME}/magicbot-z1_sdk-main" \
        "${HOME}/MaigcLab/magicbot-z1_sdk-main" \
        "/home/eame/magicbot-z1_sdk-main" \
        "/home/hiyio/MaigcLab/magicbot-z1_sdk-main"; do
        if [[ -f "${candidate}/include/magic_robot.h" ]]; then
            MAGICBOT_Z1_SDK_ROOT="${candidate}"
            break
        fi
    done
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

MUJOCO_ROOT="${MUJOCO_ROOT:-}"
if [[ -z "${MUJOCO_ROOT}" ]]; then
    shopt -s nullglob
    candidates=(
        "${PROJECT_ROOT}/.native_deps/mujoco"
        "${HOME}/mujoco"
        "${HOME}/.mujoco"/mujoco*
        "${HOME}/anaconda3/envs/robomimic/lib/"*/site-packages/mujoco
        "${HOME}/miniconda3/envs/robomimic/lib/"*/site-packages/mujoco
    )
    shopt -u nullglob
    for candidate in "${candidates[@]}"; do
        if [[ -f "${candidate}/include/mujoco/mujoco.h" ]] && compgen -G "${candidate}/libmujoco.so*" > /dev/null; then
            mkdir -p "${PROJECT_ROOT}/.native_deps"
            if [[ "${candidate}" != "${PROJECT_ROOT}/.native_deps/mujoco" ]]; then
                ln -sfn "${candidate}" "${PROJECT_ROOT}/.native_deps/mujoco"
            fi
            MUJOCO_ROOT="${PROJECT_ROOT}/.native_deps/mujoco"
            break
        fi
    done
fi

if [[ -z "${MUJOCO_ROOT}" || ! -f "${MUJOCO_ROOT}/include/mujoco/mujoco.h" ]]; then
    echo "[Error] MuJoCo C API not found. Set MUJOCO_ROOT." >&2
    exit 1
fi

if ! compgen -G "${MUJOCO_ROOT}/libmujoco.so*" > /dev/null; then
    echo "[Error] MuJoCo library not found under ${MUJOCO_ROOT}." >&2
    exit 1
fi

mkdir -p "${BUILD_DIR}/onnxruntime"
ORT_LINK_LIB="${BUILD_DIR}/onnxruntime/libonnxruntime.so.1"
ln -sf "${ONNXRUNTIME_LIB}" "${ORT_LINK_LIB}"
ONNXRUNTIME_LIB="${ORT_LINK_LIB}"

export LD_LIBRARY_PATH="${BUILD_DIR}/onnxruntime:${MUJOCO_ROOT}:${LD_LIBRARY_PATH:-}"

if [[ "${DUAL_RATE_SKIP_BUILD:-0}" != "1" || ! -x "${NATIVE_BIN}" ]]; then
    cmake -S "${CPP_DIR}" -B "${BUILD_DIR}" \
        -DCMAKE_BUILD_TYPE=Release \
        -Dyaml-cpp_DIR=/usr/lib/x86_64-linux-gnu/cmake/yaml-cpp \
        -DMAGICBOT_Z1_SDK_ROOT="${MAGICBOT_Z1_SDK_ROOT}" \
        -DONNXRUNTIME_DIR="${ONNXRUNTIME_DIR}" \
        -DONNXRUNTIME_INCLUDE_DIR="${ONNXRUNTIME_INCLUDE_DIR}" \
        -DONNXRUNTIME_LIB="${ONNXRUNTIME_LIB}" \
        -DMUJOCO_ROOT="${MUJOCO_ROOT}"
    cmake --build "${BUILD_DIR}" --target dual_inference_rate -j"$(nproc)"
fi

exec "${NATIVE_BIN}" "${args[@]}"
