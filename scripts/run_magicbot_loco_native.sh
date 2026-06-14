#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
PROJECT_ROOT="$( cd "${SCRIPT_DIR}/.." &> /dev/null && pwd )"
DEFAULT_CONFIG="${PROJECT_ROOT}/policies/loco_mode/config/LocoMode_lowKp.yaml"
CPP_DIR="${PROJECT_ROOT}/controller_cpp"
BUILD_DIR="${CPP_DIR}/build_native"
NATIVE_BIN="${BUILD_DIR}/magicbot_z1_loco_onnx"

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

if [[ -z "${MAGICBOT_Z1_SDK_ROOT}" ]]; then
    echo "[Error] Could not find magicbot-z1_sdk-main. Set MAGICBOT_Z1_SDK_ROOT." >&2
    exit 1
fi

ONNXRUNTIME_DIR="${ONNXRUNTIME_DIR:-/home/hiyio/unitree_rl_lab/deploy/thirdparty/onnxruntime-linux-x64-1.22.0}"
ONNXRUNTIME_INCLUDE_DIR="${ONNXRUNTIME_INCLUDE_DIR:-${ONNXRUNTIME_DIR}/include}"
ONNXRUNTIME_LIB="${ONNXRUNTIME_LIB:-}"
MUJOCO_ROOT="${MUJOCO_ROOT:-}"

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
    echo "[Error] ONNXRuntime headers not found: ${ONNXRUNTIME_INCLUDE_DIR}" >&2
    echo "        Set ONNXRUNTIME_INCLUDE_DIR to a directory containing onnxruntime_cxx_api.h." >&2
    exit 1
fi

if [[ ! -f "${ONNXRUNTIME_LIB}" ]]; then
    echo "[Error] ONNXRuntime library not found. Set ONNXRUNTIME_LIB." >&2
    exit 1
fi

needs_build() {
    [[ ! -x "${NATIVE_BIN}" ]] && return 0

    local dep
    for dep in \
        "${CPP_DIR}/CMakeLists.txt" \
        "${CPP_DIR}/src/magicbot_z1_loco_onnx.cpp" \
        "${CPP_DIR}/src/magicbot_loco_core.cpp" \
        "${CPP_DIR}/src/magicbot_loco_sdk_adapter.cpp"; do
        [[ "${dep}" -nt "${NATIVE_BIN}" ]] && return 0
    done

    while IFS= read -r dep; do
        [[ "${dep}" -nt "${NATIVE_BIN}" ]] && return 0
    done < <(find "${CPP_DIR}/include" -maxdepth 1 -type f \( -name '*.h' -o -name '*.hpp' \))

    return 1
}

mkdir -p "${BUILD_DIR}/onnxruntime"
ORT_LINK_LIB="${BUILD_DIR}/onnxruntime/libonnxruntime.so.1"
ln -sf "${ONNXRUNTIME_LIB}" "${ORT_LINK_LIB}"
ONNXRUNTIME_LIB="${ORT_LINK_LIB}"
export LD_LIBRARY_PATH="${BUILD_DIR}/onnxruntime:${LD_LIBRARY_PATH:-}"

if needs_build; then
    cmake -S "${CPP_DIR}" -B "${BUILD_DIR}" \
        -DCMAKE_BUILD_TYPE=Release \
        -Dyaml-cpp_DIR=/usr/lib/x86_64-linux-gnu/cmake/yaml-cpp \
        -DMAGICBOT_Z1_SDK_ROOT="${MAGICBOT_Z1_SDK_ROOT}" \
        -DONNXRUNTIME_DIR="${ONNXRUNTIME_DIR}" \
        -DONNXRUNTIME_INCLUDE_DIR="${ONNXRUNTIME_INCLUDE_DIR}" \
        -DONNXRUNTIME_LIB="${ONNXRUNTIME_LIB}" \
        -DMUJOCO_ROOT="${MUJOCO_ROOT}"
    cmake --build "${BUILD_DIR}" --target magicbot_z1_loco_onnx -j"$(nproc)"
fi

exec "${NATIVE_BIN}" "${args[@]}"
