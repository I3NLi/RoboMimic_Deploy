#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
PROJECT_ROOT="$( cd "${SCRIPT_DIR}/.." &> /dev/null && pwd )"
CPP_DIR="${PROJECT_ROOT}/controller_cpp"
BUILD_DIR="${CPP_DIR}/build_mujoco_viewer"

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

export LD_LIBRARY_PATH="${MUJOCO_ROOT}:${LD_LIBRARY_PATH:-}"

cmake -S "${CPP_DIR}" -B "${BUILD_DIR}" \
    -DCMAKE_BUILD_TYPE=Release \
    -Dyaml-cpp_DIR=/usr/lib/x86_64-linux-gnu/cmake/yaml-cpp \
    -DMUJOCO_ROOT="${MUJOCO_ROOT}"
cmake --build "${BUILD_DIR}" --target mujoco_dds_simulator -j"$(nproc)"

exec "${BUILD_DIR}/mujoco_dds_simulator" "$@"
