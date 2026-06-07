#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
PROJECT_ROOT="$( cd "${SCRIPT_DIR}/../.." &> /dev/null && pwd )"

python_is_310() {
    local candidate="$1"
    [[ -x "${candidate}" ]] || return 1
    "${candidate}" - <<'PY' >/dev/null 2>&1
import sys
raise SystemExit(0 if sys.version_info[:2] == (3, 10) else 1)
PY
}

choose_python() {
    if [[ -n "${CONDA_PREFIX:-}" ]] && python_is_310 "${CONDA_PREFIX}/bin/python"; then
        printf '%s\n' "${CONDA_PREFIX}/bin/python"
        return 0
    fi

    local candidate
    for candidate in \
        "/home/hiyio/anaconda3/envs/env_isaaclab/bin/python" \
        "/home/hiyio/anaconda3/envs/LeggedLab/bin/python" \
        "/home/hiyio/anaconda3/envs/robomimic/bin/python" \
        "$(command -v python3.10 || true)" \
        "$(command -v python3 || true)"; do
        if [[ -n "${candidate}" ]] && python_is_310 "${candidate}"; then
            printf '%s\n' "${candidate}"
            return 0
        fi
    done

    echo "[Error] No Python 3.10 interpreter found." >&2
    return 1
}

PYTHON_EXE="$(choose_python)"

exec "${PYTHON_EXE}" "${PROJECT_ROOT}/python_reference/legacy_robot/magicbot_loco_reference.py" "$@"
