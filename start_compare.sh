#!/usr/bin/env bash
# start_compare.sh
# 同时启动 MuJoCo Python 仿真（带 DDS 桥接）和 C++ shadow 进程，
# 实时对比两者的策略输出是否一致。
#
# 用法：
#   bash start_compare.sh [BeyondMimic.yaml 路径] [网络接口]
#
# 示例：
#   bash start_compare.sh \
#       policy/beyond_mimic/config/BeyondMimic.yaml \
#       wlp3s0
#
# 手柄操作流程（两侧同步触发）：
#   1. 无需按键 —— shadow 模式下 C++ 自动进入 SKILL_BEYOND_MIMIC
#   2. Python 侧需手动：START→FixedPose → R1+A→Loco → Y+R1→BeyondMimic
#   3. 两侧都在 SKILL_BEYOND_MIMIC 后开始打印 diff 统计

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
YAML="${1:-${SCRIPT_DIR}/policy/beyond_mimic/config/BeyondMimic.yaml}"
NET="${2:-wlp3s0}"
CPP_BIN="${SCRIPT_DIR}/deploy_real_c/build/deploy_real_onnx"

# ── 检查文件是否存在 ──────────────────────────────────────────────────────────
if [ ! -f "$YAML" ]; then
    echo "[ERROR] BeyondMimic YAML not found: $YAML"
    exit 1
fi
if [ ! -f "$CPP_BIN" ]; then
    echo "[ERROR] C++ binary not found: $CPP_BIN"
    echo "        Run: cd deploy_real_c && cmake -B build && cmake --build build"
    exit 1
fi

# ── 清理函数 ──────────────────────────────────────────────────────────────────
cleanup() {
    echo ""
    echo "[compare] Stopping all processes..."
    kill "$CPP_PID" 2>/dev/null || true
    kill "$PY_PID"  2>/dev/null || true
    wait "$CPP_PID" 2>/dev/null || true
    wait "$PY_PID"  2>/dev/null || true
    echo "[compare] Done."
}
trap cleanup EXIT INT TERM

echo "========================================"
echo " RoboMimic: Python vs C++ Policy Compare"
echo "========================================"
echo "  YAML : $YAML"
echo "  NET  : $NET"
echo "  C++  : $CPP_BIN"
echo "  Py   : $SCRIPT_DIR/deploy_mujoco/deploy_mujoco_dds.py"
echo ""

# ── 启动 C++ shadow 进程 ──────────────────────────────────────────────────────
echo "[compare] Starting C++ shadow process..."
"$CPP_BIN" \
    --shadow \
    --net "$NET" \
    --yaml "$YAML" \
    2>&1 | sed 's/^/[C++] /' &
CPP_PID=$!
echo "[compare] C++ PID = $CPP_PID"

# 等待 C++ 进程初始化（DDS 发现需要一点时间）
sleep 1

# ── 启动 Python MuJoCo + DDS 桥接 ────────────────────────────────────────────
echo "[compare] Starting Python MuJoCo bridge..."
cd "$SCRIPT_DIR/deploy_mujoco"
python deploy_mujoco_dds.py \
    --yaml "$YAML" \
    --net  "$NET" \
    2>&1 | sed 's/^/[Py]  /' &
PY_PID=$!
echo "[compare] Python PID = $PY_PID"

# ── 等待任意一个进程退出 ──────────────────────────────────────────────────────
echo ""
echo "[compare] Both processes running. Press Ctrl+C to stop."
wait -n "$CPP_PID" "$PY_PID" 2>/dev/null || true
echo "[compare] One process exited — stopping."
