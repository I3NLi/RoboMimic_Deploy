#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
PROJECT_ROOT="$( cd "${SCRIPT_DIR}/.." &> /dev/null && pwd )"
REAL_ADAPTER="${PROJECT_ROOT}/controller_cpp/include/magicbot_real_adapter.h"
SDK_ADAPTER="${PROJECT_ROOT}/controller_cpp/src/magicbot_loco_sdk_adapter.cpp"

usage() {
    cat <<EOF
Usage: $0

Check the no-hardware MagicBot real-adapter target-mode contract:
  Position   -> publish_sdk24_command(..., target q/kp/kd, damping_only=false)
  ZeroTorque -> publish_damping(..., 0.0f)
  Damping    -> publish_damping(..., target.damping_kd)

This is intentionally a source contract check because MagicbotSdkAdapter is the
real SDK boundary and should not be replaced with a fake in production code.
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
    usage
    exit 0
fi

for tool in python3; do
    if ! command -v "${tool}" >/dev/null 2>&1; then
        echo "[Smoke][ERROR] required tool not found: ${tool}" >&2
        exit 1
    fi
done

python3 - "${REAL_ADAPTER}" "${SDK_ADAPTER}" <<'PY'
import re
import sys
from pathlib import Path

real_path = Path(sys.argv[1])
sdk_path = Path(sys.argv[2])
real = real_path.read_text(encoding="utf-8")
sdk = sdk_path.read_text(encoding="utf-8")

def fail(message):
    raise SystemExit(f"[Smoke][ERROR] {message}")

def function_body(source, marker):
    start = source.find(marker)
    if start < 0:
        fail(f"could not locate function marker: {marker}")
    brace = source.find("{", start)
    if brace < 0:
        fail(f"could not locate function body for: {marker}")
    depth = 0
    for index in range(brace, len(source)):
        char = source[index]
        if char == "{":
            depth += 1
        elif char == "}":
            depth -= 1
            if depth == 0:
                return source[brace + 1:index]
    fail(f"could not parse function body for: {marker}")

def if_branch(source, condition):
    pattern = f"if ({condition})"
    start = source.find(pattern)
    if start < 0:
        fail(f"could not locate branch: {condition}")
    brace = source.find("{", start)
    if brace < 0:
        fail(f"could not locate branch body: {condition}")
    depth = 0
    for index in range(brace, len(source)):
        char = source[index]
        if char == "{":
            depth += 1
        elif char == "}":
            depth -= 1
            if depth == 0:
                return source[brace + 1:index], index + 1
    fail(f"could not parse branch body: {condition}")

def require_contains(source, needle, label):
    if needle not in source:
        fail(f"{label} missing: {needle}")

def require_not_contains(source, needle, label):
    if needle in source:
        fail(f"{label} must not contain: {needle}")

require_contains(
    real,
    "class MagicbotRealAdapter final : public RobotAdapter",
    "real adapter must remain the RobotAdapter implementation",
)

write_target = function_body(real, "void write_target(const JointTarget& target) override")
zero_body, zero_end = if_branch(write_target, "target.mode == JointTargetMode::ZeroTorque")
damping_body, damping_end = if_branch(write_target, "target.mode == JointTargetMode::Damping")
position_body = write_target[damping_end:]

require_contains(zero_body, "robot_.publish_damping(snap.counts, 0.0f);", "ZeroTorque branch")
require_contains(zero_body, "command_published_ = true;", "ZeroTorque branch")
require_contains(zero_body, "return;", "ZeroTorque branch")
require_not_contains(zero_body, "publish_sdk24_command", "ZeroTorque branch")
require_not_contains(zero_body, "target.damping_kd", "ZeroTorque branch")

require_contains(damping_body, "robot_.publish_damping(snap.counts, target.damping_kd);", "Damping branch")
require_contains(damping_body, "command_published_ = true;", "Damping branch")
require_contains(damping_body, "return;", "Damping branch")
require_not_contains(damping_body, "publish_sdk24_command", "Damping branch")
require_not_contains(damping_body, "0.0f", "Damping branch")

for needle in (
    "robot_.publish_sdk24_command(",
    "snap.counts",
    "target.q",
    "target.gains.kp",
    "target.gains.kd",
    "false",
    "target.damping_kd",
    "command_published_ = true;",
):
    require_contains(position_body, needle, "Position branch")
require_not_contains(position_body, "publish_damping", "Position branch")

write_damping = function_body(real, "void write_damping(float damping_kd) override")
require_contains(
    write_damping,
    "robot_.publish_damping(state_.snapshot().counts, damping_kd);",
    "explicit write_damping",
)
require_contains(write_damping, "command_published_ = true;", "explicit write_damping")
require_not_contains(write_damping, "publish_sdk24_command", "explicit write_damping")

sdk_publish_damping = function_body(sdk, "void MagicbotSdkAdapter::publish_damping(const Counts& counts, float damping_kd)")
for needle in (
    "JointArray zero{};",
    "publish_sdk24_command(counts, zero, zero, zero, true, damping_kd);",
):
    require_contains(sdk_publish_damping, needle, "MagicbotSdkAdapter::publish_damping")

sdk_publish_command = function_body(
    sdk,
    "void MagicbotSdkAdapter::publish_sdk24_command(",
)
require_contains(
    sdk_publish_command,
    "damping_only ? damping_kd : 0.0f",
    "SDK damping-only fill path",
)
require_contains(
    sdk_publish_command,
    "set_joint_command(command->joints[static_cast<size_t>(group_idx)], 0.0f, 0.0f, damping_kd, 3);",
    "SDK damping-only motor path",
)

print("[real_adapter_target_mode_check] PASS")
PY
