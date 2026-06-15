#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
PROJECT_ROOT="$( cd "${SCRIPT_DIR}/.." &> /dev/null && pwd )"
CPP_DIR="${PROJECT_ROOT}/controller_cpp"
BUILD_DIR="${CPP_DIR}/build_virtual_remote"
NATIVE_BIN="${BUILD_DIR}/robot_controller_onnx"
DEFAULT_YAML="${PROJECT_ROOT}/policies/beyond_mimic/config/BeyondMimic.yaml"
DEFAULT_TRACK_YAML="${PROJECT_ROOT}/policies/track_mimic/config/BeyondMimic.yaml"

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
if ! has_arg "--yaml" "${args[@]}"; then
    args=(--yaml "${DEFAULT_YAML}" "${args[@]}")
fi
if ! has_arg "--track-yaml" "${args[@]}"; then
    args=(--track-yaml "${DEFAULT_TRACK_YAML}" "${args[@]}")
fi
if ! has_arg "--virtual-remote" "${args[@]}"; then
    args=(--virtual-remote "${args[@]}")
fi
if ! has_arg "--virtual-remote-bind" "${args[@]}"; then
    args=(--virtual-remote-bind 0.0.0.0 "${args[@]}")
fi
if ! has_arg "--virtual-remote-port" "${args[@]}"; then
    args=(--virtual-remote-port "${MAGICBOT_REMOTE_WIRELESS_PORT:-15001}" "${args[@]}")
fi
if [[ "${MAGICBOT_REAL_JOYSTICK:-0}" == "1" || -n "${MAGICBOT_JOYSTICK_DEV:-}" ]]; then
    if ! has_arg "--joystick" "${args[@]}"; then
        args=(--joystick "${args[@]}")
    fi
fi
if has_arg "--joystick" "${args[@]}" && ! has_arg "--joystick-dev" "${args[@]}"; then
    args=(--joystick-dev "${MAGICBOT_JOYSTICK_DEV:-/dev/input/js0}" "${args[@]}")
fi

ONNXRUNTIME_DIR="${ONNXRUNTIME_DIR:-/home/hiyio/unitree_rl_lab/deploy/thirdparty/onnxruntime-linux-x64-1.22.0}"
ONNXRUNTIME_INCLUDE_DIR="${ONNXRUNTIME_INCLUDE_DIR:-${ONNXRUNTIME_DIR}/include}"
ONNXRUNTIME_LIB="${ONNXRUNTIME_LIB:-}"
MUJOCO_ROOT="${MUJOCO_ROOT:-}"
YAML_CPP_DIR="${YAML_CPP_DIR:-}"

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
    exit 1
fi

if [[ ! -f "${ONNXRUNTIME_LIB}" ]]; then
    echo "[Error] ONNXRuntime library not found. Set ONNXRUNTIME_LIB." >&2
    exit 1
fi

if [[ -z "${YAML_CPP_DIR}" ]]; then
    multiarch="$(gcc -print-multiarch 2>/dev/null || true)"
    for candidate in \
        "/usr/lib/${multiarch}/cmake/yaml-cpp" \
        "/usr/lib/$(uname -m)-linux-gnu/cmake/yaml-cpp" \
        "/usr/lib/aarch64-linux-gnu/cmake/yaml-cpp" \
        "/usr/lib/x86_64-linux-gnu/cmake/yaml-cpp" \
        "/usr/local/lib/cmake/yaml-cpp"; do
        if [[ -f "${candidate}/yaml-cpp-config.cmake" ]]; then
            YAML_CPP_DIR="${candidate}"
            break
        fi
    done
fi

mkdir -p "${BUILD_DIR}/onnxruntime"
ORT_LINK_LIB="${BUILD_DIR}/onnxruntime/libonnxruntime.so.1"
ln -sf "${ONNXRUNTIME_LIB}" "${ORT_LINK_LIB}"
ONNXRUNTIME_LIB="${ORT_LINK_LIB}"
export LD_LIBRARY_PATH="${BUILD_DIR}/onnxruntime:${LD_LIBRARY_PATH:-}"

if [[ ! -x "${NATIVE_BIN}" \
    || "${CPP_DIR}/CMakeLists.txt" -nt "${NATIVE_BIN}" \
    || "${CPP_DIR}/src/robot_controller.cpp" -nt "${NATIVE_BIN}" \
    || "${CPP_DIR}/include/beyond_mimic_policy.h" -nt "${NATIVE_BIN}" \
    || "${CPP_DIR}/include/onnx_skill_policies.h" -nt "${NATIVE_BIN}" ]]; then
    cmake_args=(
        -DCMAKE_BUILD_TYPE=Release
        -DONNXRUNTIME_DIR="${ONNXRUNTIME_DIR}"
        -DONNXRUNTIME_INCLUDE_DIR="${ONNXRUNTIME_INCLUDE_DIR}"
        -DONNXRUNTIME_LIB="${ONNXRUNTIME_LIB}"
        -DMUJOCO_ROOT="${MUJOCO_ROOT}"
    )
    if [[ -n "${YAML_CPP_DIR}" ]]; then
        cmake_args=(-Dyaml-cpp_DIR="${YAML_CPP_DIR}" "${cmake_args[@]}")
    fi
    cmake -S "${CPP_DIR}" -B "${BUILD_DIR}" \
        "${cmake_args[@]}"
    cmake --build "${BUILD_DIR}" --target robot_controller_onnx -j"$(nproc)"
fi

exec "${NATIVE_BIN}" "${args[@]}"
