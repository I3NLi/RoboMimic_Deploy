#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
PROJECT_ROOT="$( cd "${SCRIPT_DIR}/.." &> /dev/null && pwd )"
RUNNER="${SCRIPT_DIR}/run_magicbot_loco_native.sh"
RUNNER_SOURCE="${SCRIPT_DIR}/../controller_cpp/src/magicbot_z1_loco_onnx.cpp"
README_PATH="${PROJECT_ROOT}/README.md"
README_ZH_PATH="${PROJECT_ROOT}/README_zh.md"
RUNNER_WRAPPER="${SCRIPT_DIR}/run_magicbot_loco_native.sh"

keep_log=0

usage() {
    cat <<EOF
Usage: $0 [--keep-log]

Smoke-test the real runner safety gates without connecting to a robot. The
script verifies dry-run policy loading, the default dry-run mode, and that
--run is rejected unless the explicit high-risk --allow-loco gate is present.
It also checks that --pd-stand-only remains a lower-risk run path that does not
need --allow-loco, while still stopping at network preflight before any robot
connection when the local IP is not assigned.
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --keep-log)
            keep_log=1
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "[Smoke][ERROR] unknown argument: $1" >&2
            usage >&2
            exit 1
            ;;
    esac
done

for tool in python3 rg; do
    if ! command -v "${tool}" >/dev/null 2>&1; then
        echo "[Smoke][ERROR] required tool not found: ${tool}" >&2
        exit 1
    fi
done

echo "[Smoke] Checking real runner writes through adapter boundary"
if rg -n 'publish_(sdk24_command|damping)\(' "${RUNNER_SOURCE}"; then
    echo "[Smoke][ERROR] magicbot_z1_loco_onnx.cpp must not publish robot commands directly; use MagicbotRealAdapter" >&2
    exit 1
fi
if rg -n 'OnnxLocoPolicy[[:space:]]+[A-Za-z_]|\.infer\(' "${RUNNER_SOURCE}"; then
    echo "[Smoke][ERROR] magicbot_z1_loco_onnx.cpp must run policy inference through ControllerCore" >&2
    exit 1
fi
if rg -n 'MotionSafety[[:space:]]+[A-Za-z_]|safety\.check' "${RUNNER_SOURCE}"; then
    echo "[Smoke][ERROR] magicbot_z1_loco_onnx.cpp must run motion safety through ControllerCore" >&2
    exit 1
fi

echo "[Smoke] Checking README real safety ladder order"
python3 - "${README_PATH}" "${README_ZH_PATH}" <<'PY'
import sys
from pathlib import Path

required = [
    "--dry-run",
    "--connect-check",
    "--read-state",
    "--debug-entry-only",
    "--pd-stand-only",
    "scripts/run_dual_push_smoke_native.sh",
    "--allow-loco",
]

sections = [
    (Path(sys.argv[1]), "## Real-Robot Safety Ladder", "## Runtime Notes", "README"),
    (Path(sys.argv[2]), "## 真机安全阶梯", "## 运行说明", "README_zh"),
]

for path, start_marker, end_marker, label in sections:
    readme = path.read_text(encoding="utf-8")
    start = readme.find(start_marker)
    end = readme.find(end_marker, start)
    if start < 0 or end < 0:
        print(f"[Smoke][ERROR] {label} is missing the real safety ladder section", file=sys.stderr)
        sys.exit(1)

    section = readme[start:end]
    offset = 0
    for item in required:
        next_pos = section.find(item, offset)
        if next_pos < 0:
            print(f"[Smoke][ERROR] {label} safety ladder missing or misordered step: {item}", file=sys.stderr)
            sys.exit(1)
        offset = next_pos + len(item)

    if section.count("--pd-stand-only") != 1:
        print(f"[Smoke][ERROR] {label} safety ladder should contain exactly one PD stand step", file=sys.stderr)
        sys.exit(1)
    if "--allow-loco" in section[: section.find("scripts/run_dual_push_smoke_native.sh")]:
        print(f"[Smoke][ERROR] {label} safety ladder must not allow LOCO before sim closed-loop smoke", file=sys.stderr)
        sys.exit(1)
PY

echo "[Smoke] Checking real runner wrapper rebuilds on shared headers"
python3 - "${RUNNER_WRAPPER}" <<'PY'
import sys
from pathlib import Path

wrapper = Path(sys.argv[1]).read_text(encoding="utf-8")
required = [
    "needs_build()",
    'find "${CPP_DIR}/include"',
    "-name '*.h'",
    "-name '*.hpp'",
    "src/magicbot_z1_loco_onnx.cpp",
    "src/magicbot_loco_core.cpp",
    "src/magicbot_loco_sdk_adapter.cpp",
]
missing = [item for item in required if item not in wrapper]
if missing:
    print(
        "[Smoke][ERROR] real runner wrapper must rebuild when shared headers or target sources change; missing: "
        + ", ".join(missing),
        file=sys.stderr,
    )
    sys.exit(1)
PY

echo "[Smoke] Checking safety-wall final damping path"
python3 - "${RUNNER_SOURCE}" <<'PY'
import re
import sys

source = open(sys.argv[1], encoding="utf-8").read()

def function_extent(name):
    marker = f"int {name}"
    start = source.find(marker)
    if start < 0:
        print(f"[Smoke][ERROR] could not locate {name}", file=sys.stderr)
        sys.exit(1)
    brace = source.find("{", start)
    depth = 0
    for index in range(brace, len(source)):
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
            if depth == 0:
                return source[start : index + 1]
    print(f"[Smoke][ERROR] could not parse {name}", file=sys.stderr)
    sys.exit(1)

def require_allow_gate(body, state_name, prefix, label):
    dance_pattern = (
        rf"if \({state_name}\.mode_request\.mode == ml::ControlMode::Dance\) \{{\s*"
        rf"allowed = dance_request_allowed\(args, \"{re.escape(prefix)}\"\);"
    )
    skill_pattern = (
        rf"else if \({state_name}\.mode_request\.mode == ml::ControlMode::Skill\) \{{\s*"
        rf"allowed = skill_request_allowed\(args, \"{re.escape(prefix)}\"\);"
    )
    allowed_pattern = (
        rf"if \(allowed\) \{{\s*"
        rf".*?= {state_name}\.mode_request\.mode;"
    )
    if not re.search(dance_pattern, body):
        print(f"[Smoke][ERROR] {label} must gate DANCE through dance_request_allowed", file=sys.stderr)
        sys.exit(1)
    if not re.search(skill_pattern, body):
        print(f"[Smoke][ERROR] {label} must gate SKILL through skill_request_allowed", file=sys.stderr)
        sys.exit(1)
    if not re.search(allowed_pattern, body, re.S):
        print(f"[Smoke][ERROR] {label} must apply DANCE/SKILL mode requests only inside if (allowed)", file=sys.stderr)
        sys.exit(1)

body = function_extent("run_robot_with_finally")
input_check_body = function_extent("input_check_only")
require_allow_gate(input_check_body, "state", "[InputCheck]", "input-check path")
require_allow_gate(body, "input", "[Input]", "real run input path")

lambda_match = re.search(r"auto final_damping = \[&\]\(\).*?;\n\n    try", body, re.S)
if not lambda_match:
    print("[Smoke][ERROR] could not locate final_damping lambda", file=sys.stderr)
    sys.exit(1)
lambda_body = lambda_match.group(0)
if "ControllerRuntime runtime(core, real_adapter)" not in lambda_body or "runtime.write_damping(args.damping_kd)" not in lambda_body:
    print("[Smoke][ERROR] final damping must publish through ControllerRuntime.write_damping", file=sys.stderr)
    sys.exit(1)
if re.search(r"real_adapter\.write_damping|robot\.publish_damping|robot\.publish_sdk24_command", lambda_body):
    print("[Smoke][ERROR] final damping must not bypass runtime/adapter boundary", file=sys.stderr)
    sys.exit(1)

catch_pos = body.find("} catch (const std::exception& exc)")
if catch_pos < 0 or "[SafetyWall]" not in body[catch_pos:]:
    print("[Smoke][ERROR] run_robot_with_finally must keep a SafetyWall catch", file=sys.stderr)
    sys.exit(1)
tail = body[catch_pos:]
final_pos = tail.find("final_damping();")
disconnect_pos = tail.find("robot.disconnect")
if final_pos < 0:
    print("[Smoke][ERROR] SafetyWall path must call final_damping()", file=sys.stderr)
    sys.exit(1)
if disconnect_pos < 0 or final_pos > disconnect_pos:
    print("[Smoke][ERROR] final_damping() must run before robot.disconnect()", file=sys.stderr)
    sys.exit(1)

stand_marker = "ml::JointArray stand_interpolation"
stand_start = source.find(stand_marker)
if stand_start < 0:
    print("[Smoke][ERROR] could not locate stand_interpolation", file=sys.stderr)
    sys.exit(1)
stand_brace = source.find("{", stand_start)
depth = 0
stand_end = -1
for index in range(stand_brace, len(source)):
    if source[index] == "{":
        depth += 1
    elif source[index] == "}":
        depth -= 1
        if depth == 0:
            stand_end = index + 1
            break
if stand_end < 0:
    print("[Smoke][ERROR] could not parse stand_interpolation", file=sys.stderr)
    sys.exit(1)

hold_marker = "ml::JointArray hold_default_stand"
hold_start = source.find(hold_marker)
if hold_start < 0:
    print("[Smoke][ERROR] could not locate hold_default_stand", file=sys.stderr)
    sys.exit(1)
hold_brace = source.find("{", hold_start)
depth = 0
hold_end = -1
for index in range(hold_brace, len(source)):
    if source[index] == "{":
        depth += 1
    elif source[index] == "}":
        depth -= 1
        if depth == 0:
            hold_end = index + 1
            break
if hold_end < 0:
    print("[Smoke][ERROR] could not parse hold_default_stand", file=sys.stderr)
    sys.exit(1)
hold_body = source[hold_start:hold_end]
if "ControllerRuntime runtime(core, real_adapter)" not in hold_body or "runtime.tick(tick_input)" not in hold_body:
    print("[Smoke][ERROR] stand hold must publish through ControllerRuntime.tick", file=sys.stderr)
    sys.exit(1)
for forbidden in (
    "torque_limited_target",
    "clamp_and_rate_limit",
    "safety.check",
    "real_adapter.write_target",
):
    if forbidden in hold_body:
        print(
            "[Smoke][ERROR] hold_default_stand must not duplicate ControllerCore safety/limit/write logic: "
            + forbidden,
            file=sys.stderr,
        )
        sys.exit(1)

for direct_pattern in (
    r"torque_limited_target",
    r"clamp_and_rate_limit",
    r"real_adapter\.write_target",
):
    for match in re.finditer(direct_pattern, source):
        if not (stand_start <= match.start() < stand_end):
            print(
                "[Smoke][ERROR] direct target limiting/writes are only allowed in stand_interpolation: "
                + direct_pattern,
                file=sys.stderr,
            )
            sys.exit(1)
PY

dry_log="$(mktemp /tmp/magicbot_loco_safety_dry_XXXXXX.log)"
default_log="$(mktemp /tmp/magicbot_loco_safety_default_XXXXXX.log)"
blocked_log="$(mktemp /tmp/magicbot_loco_safety_blocked_XXXXXX.log)"
pd_stand_preflight_log="$(mktemp /tmp/magicbot_loco_safety_pd_stand_XXXXXX.log)"
exclusive_log="$(mktemp /tmp/magicbot_loco_safety_exclusive_XXXXXX.log)"
input_source_log="$(mktemp /tmp/magicbot_loco_safety_input_source_XXXXXX.log)"
live_input_log="$(mktemp /tmp/magicbot_loco_safety_live_input_XXXXXX.log)"

cleanup() {
    if [[ "${keep_log}" -eq 0 ]]; then
        rm -f "${dry_log}" "${default_log}" "${blocked_log}" \
            "${pd_stand_preflight_log}" "${exclusive_log}" \
            "${input_source_log}" "${live_input_log}"
    fi
}
trap cleanup EXIT

assert_no_robot_path() {
    local log_path="$1"
    if rg -q '\[MagicBot\]|\[ConnectCheck\]|\[ReadState\]|\[State\] Robot state ready|\[Stage\]|\[Final\]|MagicBot SDK initialize failed|SetMotionControlLevel|LowLevel controller initialize' "${log_path}"; then
        echo "[Smoke][ERROR] blocked command appears to have entered robot connection path" >&2
        sed -n '1,180p' "${log_path}" >&2
        exit 1
    fi
}

echo "[Smoke] Checking explicit dry-run"
"${RUNNER}" --dry-run >"${dry_log}" 2>&1
for expected in 'Config:' 'ONNX input/output:' 'Raw target sample range:' 'Command target sample range:'; do
    if ! rg -q "${expected}" "${dry_log}"; then
        echo "[Smoke][ERROR] dry-run output missing: ${expected}" >&2
        sed -n '1,180p' "${dry_log}" >&2
        exit 1
    fi
done

echo "[Smoke] Checking default mode is dry-run"
"${RUNNER}" >"${default_log}" 2>&1
if ! rg -q 'ONNX input/output:' "${default_log}"; then
    echo "[Smoke][ERROR] default invocation did not run dry-run" >&2
    sed -n '1,180p' "${default_log}" >&2
    exit 1
fi

echo "[Smoke] Checking --run is blocked without --allow-loco"
if "${RUNNER}" --run --duration 0.01 >"${blocked_log}" 2>&1; then
    echo "[Smoke][ERROR] --run succeeded without --allow-loco" >&2
    sed -n '1,180p' "${blocked_log}" >&2
    exit 1
fi
if ! rg -q 'refusing ONNX loco without --allow-loco' "${blocked_log}"; then
    echo "[Smoke][ERROR] --run failure did not report allow-loco gate" >&2
    sed -n '1,180p' "${blocked_log}" >&2
    exit 1
fi
assert_no_robot_path "${blocked_log}"

echo "[Smoke] Checking --pd-stand-only bypasses allow-loco but not network preflight"
if "${RUNNER}" --run --pd-stand-only --duration 0.01 --local-ip 203.0.113.254 >"${pd_stand_preflight_log}" 2>&1; then
    echo "[Smoke][ERROR] --pd-stand-only unexpectedly passed with an unassigned local IP" >&2
    sed -n '1,180p' "${pd_stand_preflight_log}" >&2
    exit 1
fi
if rg -q 'refusing ONNX loco without --allow-loco' "${pd_stand_preflight_log}"; then
    echo "[Smoke][ERROR] --pd-stand-only was incorrectly blocked by the allow-loco gate" >&2
    sed -n '1,180p' "${pd_stand_preflight_log}" >&2
    exit 1
fi
if ! rg -q 'local IP 203\.0\.113\.254 is not assigned to this machine' "${pd_stand_preflight_log}"; then
    echo "[Smoke][ERROR] --pd-stand-only did not stop at local-IP preflight" >&2
    sed -n '1,180p' "${pd_stand_preflight_log}" >&2
    exit 1
fi
assert_no_robot_path "${pd_stand_preflight_log}"

echo "[Smoke] Checking main modes are mutually exclusive before connection"
if "${RUNNER}" --connect-check --read-state >"${exclusive_log}" 2>&1; then
    echo "[Smoke][ERROR] mutually exclusive modes unexpectedly succeeded" >&2
    sed -n '1,180p' "${exclusive_log}" >&2
    exit 1
fi
if ! rg -q 'use only one of --dry-run, --connect-check, --read-state, --run, --input-check, --debug-entry-only' "${exclusive_log}"; then
    echo "[Smoke][ERROR] exclusive-mode failure did not report mode gate" >&2
    sed -n '1,180p' "${exclusive_log}" >&2
    exit 1
fi
assert_no_robot_path "${exclusive_log}"

echo "[Smoke] Checking --input-check requires an input source"
if "${RUNNER}" --input-check >"${input_source_log}" 2>&1; then
    echo "[Smoke][ERROR] --input-check without input source unexpectedly succeeded" >&2
    sed -n '1,180p' "${input_source_log}" >&2
    exit 1
fi
if ! rg -q -- '--input-check requires --keyboard-control, --gamepad-control or --udp-control' "${input_source_log}"; then
    echo "[Smoke][ERROR] input-check failure did not report input-source gate" >&2
    sed -n '1,180p' "${input_source_log}" >&2
    exit 1
fi
assert_no_robot_path "${input_source_log}"

echo "[Smoke] Checking live input sources are mutually exclusive before connection"
if "${RUNNER}" --run --allow-loco --keyboard-control --udp-control --duration 0.01 >"${live_input_log}" 2>&1; then
    echo "[Smoke][ERROR] multiple live input sources unexpectedly succeeded" >&2
    sed -n '1,180p' "${live_input_log}" >&2
    exit 1
fi
if ! rg -q -- 'use only one of --keyboard-control, --gamepad-control or --udp-control' "${live_input_log}"; then
    echo "[Smoke][ERROR] live-input failure did not report exclusivity gate" >&2
    sed -n '1,180p' "${live_input_log}" >&2
    exit 1
fi
assert_no_robot_path "${live_input_log}"

echo "[Smoke] PASSED real-runner safety gates"
if [[ "${keep_log}" -eq 1 ]]; then
    echo "[Smoke] dry_log=${dry_log}"
    echo "[Smoke] default_log=${default_log}"
    echo "[Smoke] blocked_log=${blocked_log}"
    echo "[Smoke] pd_stand_preflight_log=${pd_stand_preflight_log}"
    echo "[Smoke] exclusive_log=${exclusive_log}"
    echo "[Smoke] input_source_log=${input_source_log}"
    echo "[Smoke] live_input_log=${live_input_log}"
fi
