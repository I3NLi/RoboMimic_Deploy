# ControllerCore refactor notes

## Target shape

The control brain must be shared by simulation and the real robot:

```text
Viewer / Control Station
  -> UDP or control API
  -> ControllerCore
  -> SimAdapter or RealAdapter
  -> MuJoCo or MagicBot SDK
```

`ControllerCore` owns observation construction, ONNX policy execution, mode requests,
motion safety, target limiting, gains, and telemetry. Adapters only translate between
backend I/O and `RobotSnapshot` / `JointTarget`.

## Current reusable pieces

- `magicbot_loco_core.h` already contains the shared `RobotSnapshot`, `LocoConfig`,
  `OnnxLocoPolicy`, `MotionSafety`, target torque limiting, joint limit clipping,
  and rate limiting utilities.
- `magicbot_loco_sdk_adapter.h` is already a real-robot I/O boundary for SDK state
  and command publication.
- `dual_inference_rate.cpp`, `mujoco_loco_viewer.cpp`, and
  `magicbot_z1_loco_onnx.cpp` currently duplicate parts of the LOCO step:
  projected gravity, ONNX inference, safety checks, torque limiting, and command
  target rate limiting.

## First refactor cut

`controller_core.h` introduces the shared data contract:

- `Command`
- `ModeRequest`
- `JointTarget`
- `JointGains`
- `ControllerTelemetry`
- `ControllerCore`

`mode_manager.h` owns shared mode state and transition metadata for
`PASSIVE`, `STAND`, `LOCO`, and `FINAL_DAMPING`. It marks transitions that must
reset policy history, zero commands, or seed targets from the current robot
state. `DANCE` and `SKILL` remain explicit modes but are rejected until a skill
policy adapter is installed.

`policy_adapter.h` defines the external policy contract for `DANCE` / `SKILL`.
Adapters return a raw motor-space target, an optional gains override, and a
completion hint. `ControllerCore` still owns policy-history reset, motion safety,
torque limiting, rate limiting, and final `JointTarget` generation. Optional
external gains are held across the 500Hz adapter write ticks between lower-rate
policy evaluations.

`fsm_external_policy_adapter.h` is a bridge for the existing native FSM policies
such as Dance and BeyondMimic. It copies `RobotSnapshot` / velocity command into
the old `StateAndCmd`, runs the wrapped `FSMState`, then converts `PolicyOutput`
back into `ExternalPolicyOutput` for the shared core.

`native_fsm_policy_types.h` provides the small FSM type surface needed by those
legacy policy headers without pulling in the old monolithic controller.

`robot_adapter.h` introduces the backend boundary:

- `RobotAdapter::read_snapshot()`
- `RobotAdapter::write_target()`
- `RobotAdapter::write_damping()`
- `AdapterTelemetry`

Concrete adapters must keep backend details local. A MuJoCo adapter may include
MuJoCo headers in its `.cpp`; a real adapter may include MagicBot SDK headers in
its `.cpp`; neither side should put those details into `ControllerCore`.

`magicbot_real_adapter.h` is the first concrete adapter. It wraps the existing
`MagicbotSdkAdapter` and `SdkRobotState`, translating shared `JointTarget`
outputs into SDK low-level commands or damping commands. It deliberately does
not inspect mode requests, run policies, or perform safety checks.

`mujoco_sim_adapter.h` is the first concrete sim adapter. It reads MuJoCo
`qpos` / `qvel` into `RobotSnapshot` and writes `JointTarget` as PD torques into
`mjData::ctrl`, preserving the shared torque-limit clamp before MuJoCo actuator
range clamping. It does not step the simulation, handle viewer input, or run
policy logic.

`dual_inference_rate.cpp` now routes pure-sim through `ControllerRuntime` and
`MujocoSimAdapter`, and routes real-state-sim policy stepping through
`ControllerCore`. It remains a validation tool only and still never publishes
commands to the real robot. Pure-sim validation also supports configurable
world-frame push tests with `--push-body`, `--push-force`, `--push-start`,
`--push-duration`, `--push-impulse`, and `--push-impulse-time`; `RATE_SUMMARY`
JSON records whether force/impulse disturbance was applied.

`controller_runtime.h` adds the shared one-tick runtime flow:
`RobotAdapter::read_snapshot()` -> `ControllerCore::step()` ->
`RobotAdapter::write_target()`. It is glue around the shared control brain, not
a second place for policy, mode, or safety logic.

`magicbot_z1_loco_onnx.cpp` now routes the real-robot STAND/LOCO runtime loop
through `ControllerRuntime` and `MagicbotRealAdapter`. The existing staged safety
flow is preserved: dry-run, connect-check, read-state, debug/passive damping,
stand interpolation, and PD stand-only remain outside high-risk LOCO execution.
LOCO still requires the explicit `--allow-loco` gate. When `--beyond-yaml PATH`
is supplied, the runner registers BeyondMimic as the shared `DANCE` external
policy and accepts `mode=beyond` / `mode=dance` from UDP, `B` from keyboard input,
or an explicitly configured gamepad dance button.

`mujoco_loco_viewer.cpp` now routes its closed-loop STAND/LOCO/PASSIVE execution
through the same `ControllerRuntime` and `MujocoSimAdapter`. Viewer input, UDP
commands, pause/reset, camera streaming, rendering, and ground correction remain
viewer responsibilities; ONNX inference, mode requests, target limiting, gains,
and MuJoCo PD torque publication now use the shared runtime path. Viewer-specific
initial-pose YAML overrides still feed the shared core default target and gains.

The first `ControllerCore` implementation supports `PASSIVE`, `STAND`, `LOCO`,
and `FINAL_DAMPING`. `DANCE` and `SKILL` are represented in the shared mode enum
but intentionally reject requests until the skill policy adapter is wired.

This keeps the real-robot safety ladder intact: high-risk LOCO still requires
the existing CLI `--allow-loco` gate, and adapters remain responsible only for
state/command I/O.

## Validation commands

Baseline closed-loop smoke:

```bash
scripts/run_dual_inference_rate_native.sh --mode pure-sim --duration 1.0 \
  --no-realtime --closed-loop-check \
  --summary-json /tmp/dual_no_push_refactor_smoke.json
```

Disturbance closed-loop smoke:

```bash
scripts/run_dual_inference_rate_native.sh --mode pure-sim --duration 1.0 \
  --no-realtime --closed-loop-check \
  --push-body pelvis --push-force 35,0,0 --push-start 0.30 \
  --push-duration 0.12 --push-impulse 0,1.0,0 --push-impulse-time 0.55 \
  --summary-json /tmp/dual_push_refactor_smoke.json
```

## Next cuts

1. Register additional concrete skill policies through `FsmExternalPolicyAdapter`
   where those entrypoints need them.
2. Move viewer mode/control API code to send requests only; it must not duplicate
   core policy or safety logic.
