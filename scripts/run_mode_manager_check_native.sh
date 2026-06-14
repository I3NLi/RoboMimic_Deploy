#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
PROJECT_ROOT="$( cd "${SCRIPT_DIR}/.." &> /dev/null && pwd )"
CPP_DIR="${PROJECT_ROOT}/controller_cpp"
SOURCE="${CPP_DIR}/src/mode_manager_check.cpp"
CXX_BIN="${CXX:-c++}"
BUILD_DIR="$(mktemp -d /tmp/magicbot_mode_manager_check_XXXXXX)"

cleanup() {
    rm -rf "${BUILD_DIR}"
}
trap cleanup EXIT

if ! command -v "${CXX_BIN}" >/dev/null 2>&1; then
    echo "[Error] C++ compiler not found: ${CXX_BIN}" >&2
    exit 1
fi

"${CXX_BIN}" \
    -std=c++17 \
    -O2 \
    -Wall \
    -Wextra \
    -I"${CPP_DIR}/include" \
    "${SOURCE}" \
    -o "${BUILD_DIR}/mode_manager_check"

"${BUILD_DIR}/mode_manager_check"
