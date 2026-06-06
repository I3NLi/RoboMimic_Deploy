#!/usr/bin/env bash
# start_compare.sh
# Python MuJoCo DDS bridge vs C++ deploy_real_onnx shadow compare launcher.
#
# Modes
# -----
# 1) interactive (default):
#      keep both processes running, view live diff, Ctrl+C to stop.
# 2) verify:
#      headless auto-run with PASS/FAIL exit code and summary json/csv.
#
# Backward compatible usage:
#   bash start_compare.sh [BeyondMimic.yaml] [net_iface]
#
# Verify usage (recommended):
#   bash start_compare.sh --verify --yaml policy/beyond_mimic/config/BeyondMimic.yaml --net lo

set -euo pipefail

usage() {
    cat <<'EOF'
Usage:
  bash start_compare.sh [YAML] [NET] [options]

Options:
  --verify                    Headless auto verification with PASS/FAIL exit code.
  --interactive               Interactive compare mode (default).
  --yaml PATH                 BeyondMimic.yaml path.
  --track-yaml PATH           TrackMimic yaml path (used with --shadow-state track).
  --shadow-state STATE        Shadow FSM state: beyond|track (default: beyond).
  --net IFACE                 DDS network interface.
  --joints N                  Robot joint count passed to C++ shadow (default: 24).
  --sync-lowstate             Force C++ shadow to step on new LowState ticks.
  --no-sync-lowstate          Disable C++ tick sync.
  --conda-env NAME            Conda env for Python runner (default: robomimic).
  --lag-search N              Use best match in recent N steps (default verify: 3).
  --cmd-wait-ms N             Wait N ms for fresh C++ cmd after each publish (default: 5).
  --max-steps N               Verification steps (default: 380).
  --warmup-steps N            Ignore first N compare samples (default: 80).
  --min-cmp-steps N           Minimum compare samples (default: 200).
  --q-tol X                   q max-abs tolerance (default: 5e-5).
  --mean-q-tol X              q mean tolerance (default: 5e-6).
  --kp-tol X                  kp tolerance (default: 1e-6).
  --kd-tol X                  kd tolerance (default: 1e-6).
  --print-every N             Print every N compare lines (default: 80).
  --csv PATH                  CSV output path (verify mode).
  --summary-json PATH         JSON summary path (verify mode).
  -h, --help                  Show this help.
EOF
}

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
MODE="interactive"
YAML="${SCRIPT_DIR}/policy/beyond_mimic/config/BeyondMimic.yaml"
TRACK_YAML=""
SHADOW_STATE="beyond"
NET="wlp3s0"
JOINTS="24"
CPP_BIN="${SCRIPT_DIR}/deploy_real_c/build_z1/deploy_real_onnx"
if [ ! -f "$CPP_BIN" ]; then
    CPP_BIN="${SCRIPT_DIR}/deploy_real_c/build/deploy_real_onnx"
fi
PY_SCRIPT="${SCRIPT_DIR}/deploy_mujoco/deploy_mujoco_dds.py"
CONDA_ENV="robomimic"
SYNC_LOWSTATE=0
LAG_SEARCH=-1
CMD_WAIT_MS="5.0"
MAX_STEPS="380"
WARMUP_STEPS="80"
MIN_CMP_STEPS="200"
Q_TOL="5e-5"
MEAN_Q_TOL="5e-6"
KP_TOL="1e-6"
KD_TOL="1e-6"
PRINT_EVERY="80"
CSV_PATH=""
SUMMARY_JSON=""

POSITIONAL=()
while [ $# -gt 0 ]; do
    case "$1" in
        --verify) MODE="verify"; shift ;;
        --interactive) MODE="interactive"; shift ;;
        --yaml) YAML="$2"; shift 2 ;;
        --track-yaml) TRACK_YAML="$2"; shift 2 ;;
        --shadow-state) SHADOW_STATE="$2"; shift 2 ;;
        --net) NET="$2"; shift 2 ;;
        --joints) JOINTS="$2"; shift 2 ;;
        --sync-lowstate) SYNC_LOWSTATE=1; shift ;;
        --no-sync-lowstate) SYNC_LOWSTATE=0; shift ;;
        --conda-env) CONDA_ENV="$2"; shift 2 ;;
        --lag-search) LAG_SEARCH="$2"; shift 2 ;;
        --cmd-wait-ms) CMD_WAIT_MS="$2"; shift 2 ;;
        --max-steps) MAX_STEPS="$2"; shift 2 ;;
        --warmup-steps) WARMUP_STEPS="$2"; shift 2 ;;
        --min-cmp-steps) MIN_CMP_STEPS="$2"; shift 2 ;;
        --q-tol) Q_TOL="$2"; shift 2 ;;
        --mean-q-tol) MEAN_Q_TOL="$2"; shift 2 ;;
        --kp-tol) KP_TOL="$2"; shift 2 ;;
        --kd-tol) KD_TOL="$2"; shift 2 ;;
        --print-every) PRINT_EVERY="$2"; shift 2 ;;
        --csv) CSV_PATH="$2"; shift 2 ;;
        --summary-json) SUMMARY_JSON="$2"; shift 2 ;;
        -h|--help) usage; exit 0 ;;
        --*) echo "[ERROR] Unknown option: $1"; usage; exit 2 ;;
        *) POSITIONAL+=("$1"); shift ;;
    esac
done

if [ ${#POSITIONAL[@]} -ge 1 ]; then
    YAML="${POSITIONAL[0]}"
fi
if [ ${#POSITIONAL[@]} -ge 2 ]; then
    NET="${POSITIONAL[1]}"
fi
if [ ${#POSITIONAL[@]} -gt 2 ]; then
    echo "[ERROR] Too many positional args: ${POSITIONAL[*]}"
    usage
    exit 2
fi

if [ "$MODE" = "verify" ] && [ "$LAG_SEARCH" = "-1" ]; then
    LAG_SEARCH="3"
fi
if [ "$LAG_SEARCH" = "-1" ]; then
    LAG_SEARCH="0"
fi
if [ "$MODE" = "verify" ] && [ "$SYNC_LOWSTATE" -eq 0 ]; then
    SYNC_LOWSTATE=1
fi

if [ -z "$CSV_PATH" ] && [ "$MODE" = "verify" ]; then
    CSV_PATH="/tmp/compare_diff_$(date +%Y%m%d_%H%M%S).csv"
fi
if [ -z "$SUMMARY_JSON" ] && [ "$MODE" = "verify" ]; then
    SUMMARY_JSON="/tmp/compare_summary_$(date +%Y%m%d_%H%M%S).json"
fi

if [ ! -f "$YAML" ]; then
    echo "[ERROR] BeyondMimic YAML not found: $YAML"
    exit 1
fi
if [ "$SHADOW_STATE" = "track" ]; then
    if [ -z "$TRACK_YAML" ]; then
        TRACK_YAML="${SCRIPT_DIR}/policy/track_mimic/config/BeyondMimic.yaml"
    fi
    if [ ! -f "$TRACK_YAML" ]; then
        echo "[ERROR] TrackMimic YAML not found: $TRACK_YAML"
        exit 1
    fi
fi
if [ ! -f "$CPP_BIN" ]; then
    echo "[ERROR] C++ binary not found: $CPP_BIN"
    echo "        Run: cd deploy_real_c && cmake -B build_z1 && cmake --build build_z1"
    exit 1
fi
if [ ! -f "$PY_SCRIPT" ]; then
    echo "[ERROR] Python compare script not found: $PY_SCRIPT"
    exit 1
fi

PYTHON_CMD=(python)
if command -v conda >/dev/null 2>&1; then
    if conda env list | awk '{print $1}' | grep -qx "$CONDA_ENV"; then
        PYTHON_CMD=(conda run -n "$CONDA_ENV" python)
    fi
fi

CPP_PID=""
PY_PID=""
cleanup() {
    echo ""
    echo "[compare] Stopping processes..."
    if [ -n "${CPP_PID:-}" ]; then
        kill "$CPP_PID" 2>/dev/null || true
        wait "$CPP_PID" 2>/dev/null || true
    fi
    if [ -n "${PY_PID:-}" ]; then
        kill "$PY_PID" 2>/dev/null || true
        wait "$PY_PID" 2>/dev/null || true
    fi
    echo "[compare] Done."
}
trap cleanup EXIT INT TERM

echo "========================================"
echo " RoboMimic: Python vs C++ Policy Compare"
echo "========================================"
echo "  Mode : $MODE"
echo "  YAML : $YAML"
echo "  TYML : ${TRACK_YAML:-(none)}"
echo "  State: $SHADOW_STATE"
echo "  NET  : $NET"
echo "  JNTS : $JOINTS"
echo "  C++  : $CPP_BIN"
echo "  Py   : $PY_SCRIPT"
echo "  PyCmd: ${PYTHON_CMD[*]}"
echo ""

CPP_ARGS=(--shadow --net "$NET" --joints "$JOINTS" --yaml "$YAML" --shadow-state "$SHADOW_STATE")
if [ -n "$TRACK_YAML" ]; then
    CPP_ARGS+=(--track-yaml "$TRACK_YAML")
fi
if [ "$SYNC_LOWSTATE" -eq 1 ]; then
    CPP_ARGS+=(--sync-lowstate)
fi
PY_ARGS=(--yaml "$YAML" --net "$NET" --shadow-state "$SHADOW_STATE")
if [ -n "$TRACK_YAML" ]; then
    PY_ARGS+=(--track-yaml "$TRACK_YAML")
fi

echo "[compare] Starting C++ shadow..."
"$CPP_BIN" "${CPP_ARGS[@]}" > >(sed 's/^/[C++] /') 2>&1 &
CPP_PID=$!
echo "[compare] C++ PID = $CPP_PID"
sleep 1

if [ "$MODE" = "interactive" ]; then
    echo "[compare] Starting Python bridge (interactive)..."
    "${PYTHON_CMD[@]}" "$PY_SCRIPT" \
        "${PY_ARGS[@]}" \
        --lag-search "$LAG_SEARCH" \
        --cmd-wait-ms "$CMD_WAIT_MS" \
        > >(sed 's/^/[Py]  /') 2>&1 &
    PY_PID=$!
    echo "[compare] Python PID = $PY_PID"
    echo "[compare] Running. Press Ctrl+C to stop."
    wait -n "$CPP_PID" "$PY_PID" 2>/dev/null || true
    echo "[compare] One process exited."
    exit 0
fi

echo "[compare] Running verification..."
set +e
"${PYTHON_CMD[@]}" "$PY_SCRIPT" \
    "${PY_ARGS[@]}" \
    --headless \
    --no-joystick \
    --shadow-sync \
    --lag-search "$LAG_SEARCH" \
    --cmd-wait-ms "$CMD_WAIT_MS" \
    --max-steps "$MAX_STEPS" \
    --warmup-steps "$WARMUP_STEPS" \
    --min-cmp-steps "$MIN_CMP_STEPS" \
    --print-every "$PRINT_EVERY" \
    --q-tol "$Q_TOL" \
    --mean-q-tol "$MEAN_Q_TOL" \
    --kp-tol "$KP_TOL" \
    --kd-tol "$KD_TOL" \
    --csv "$CSV_PATH" \
    --summary-json "$SUMMARY_JSON"
PY_RC=$?
set -e

echo "[compare] Verification exit code: $PY_RC"
echo "[compare] CSV         : $CSV_PATH"
echo "[compare] Summary JSON: $SUMMARY_JSON"
if [ -f "$SUMMARY_JSON" ]; then
    cat "$SUMMARY_JSON"
fi
exit "$PY_RC"
