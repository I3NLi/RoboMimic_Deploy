#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
PROJECT_ROOT="$( cd "${SCRIPT_DIR}/.." &> /dev/null && pwd )"
SIM_ADAPTER="${PROJECT_ROOT}/controller_cpp/include/mujoco_sim_adapter.h"

usage() {
    cat <<EOF
Usage: $0

Check the no-hardware MuJoCo sim-adapter target-mode contract:
  Position   -> PD torque into data->ctrl with shared target/gains
  ZeroTorque -> data->ctrl = 0
  Damping    -> write_damping(target.damping_kd)
  write_damping(kd) -> -dq * kd torque into data->ctrl

Viewer and sim tools must publish shared ControllerCore targets through this
adapter instead of duplicating policy, mode, safety, or target-limit logic.
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
    usage
    exit 0
fi

if ! command -v python3 >/dev/null 2>&1; then
    echo "[Smoke][ERROR] required tool not found: python3" >&2
    exit 1
fi

python3 - "${SIM_ADAPTER}" <<'PY'
import sys
from pathlib import Path

path = Path(sys.argv[1])
source = path.read_text(encoding="utf-8")

def fail(message):
    raise SystemExit(f"[Smoke][ERROR] {message}")

def function_body(marker):
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

def if_branch(body, condition):
    pattern = f"if ({condition})"
    start = body.find(pattern)
    if start < 0:
        fail(f"could not locate branch: {condition}")
    brace = body.find("{", start)
    if brace < 0:
        fail(f"could not locate branch body: {condition}")
    depth = 0
    for index in range(brace, len(body)):
        char = body[index]
        if char == "{":
            depth += 1
        elif char == "}":
            depth -= 1
            if depth == 0:
                return body[brace + 1:index], index + 1
    fail(f"could not parse branch body: {condition}")

def require_contains(text, needle, label):
    if needle not in text:
        fail(f"{label} missing: {needle}")

def require_not_contains(text, needle, label):
    if needle in text:
        fail(f"{label} must not contain: {needle}")

require_contains(
    source,
    "class MujocoSimAdapter final : public RobotAdapter",
    "sim adapter must remain the RobotAdapter implementation",
)
require_contains(source, 'out.backend = name();', "sim adapter telemetry")
require_contains(source, 'snap.counts = Counts{12, 14, 1, 2};', "sim snapshot counts")

write_target = function_body("void write_target(const JointTarget& target) override")
zero_body, zero_end = if_branch(write_target, "target.mode == JointTargetMode::ZeroTorque")
damping_body, damping_end = if_branch(write_target, "target.mode == JointTargetMode::Damping")
position_body = write_target[damping_end:]

require_contains(zero_body, "data_->ctrl[i] = 0.0;", "ZeroTorque branch")
require_contains(zero_body, "command_published_ = true;", "ZeroTorque branch")
require_contains(zero_body, "return;", "ZeroTorque branch")
for forbidden in (
    "write_damping",
    "target.damping_kd",
    "target.gains",
    "clamp_tau_limit",
):
    require_not_contains(zero_body, forbidden, "ZeroTorque branch")

require_contains(damping_body, "write_damping(target.damping_kd);", "Damping branch")
require_contains(damping_body, "return;", "Damping branch")
for forbidden in (
    "data_->ctrl[i]",
    "target.gains",
    "clamp_tau_limit",
):
    require_not_contains(damping_body, forbidden, "Damping branch")

for needle in (
    "const double q = data_->qpos[7 + options_.qpos_idx[static_cast<size_t>(i)]];",
    "const double dq = data_->qvel[6 + options_.qvel_idx[static_cast<size_t>(i)]];",
    "(options_.zero_head_target && i == kHeadMotorIndex) ? 0.0 : static_cast<double>(target.q[i])",
    "double tau = (target_q - q) * target.gains.kp[i] - dq * target.gains.kd[i];",
    "clamp_tau_limit(i, target.gains.tau_limit, tau);",
    "clamp_actuator(i, tau);",
    "data_->ctrl[i] = tau;",
    "command_published_ = true;",
):
    require_contains(position_body, needle, "Position branch")
require_not_contains(position_body, "write_damping", "Position branch")

write_damping = function_body("void write_damping(float damping_kd) override")
for needle in (
    "const double dq = data_->qvel[6 + options_.qvel_idx[static_cast<size_t>(i)]];",
    "double tau = -dq * damping_kd;",
    "clamp_actuator(i, tau);",
    "data_->ctrl[i] = tau;",
    "command_published_ = true;",
):
    require_contains(write_damping, needle, "write_damping")
for forbidden in (
    "target.q",
    "target.gains",
    "clamp_tau_limit",
):
    require_not_contains(write_damping, forbidden, "write_damping")

print("[sim_adapter_target_mode_check] PASS")
PY
