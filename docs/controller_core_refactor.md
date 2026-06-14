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
policy adapter is installed. The shared mode helper also owns the default
external-policy request mapping, including `DANCE -> BeyondMimic` and
`SKILL -> TrackMimic`, so entrypoints do not each encode different external
policy request keys.

`policy_adapter.h` defines the external policy contract for `DANCE` / `SKILL`.
Adapters return a raw motor-space target, an optional gains override, and a
completion hint. `ControllerCore` still owns policy-history reset, motion safety,
torque limiting, rate limiting, and final `JointTarget` generation. Optional
external gains are held across the 500Hz adapter write ticks between lower-rate
policy evaluations. External policies are registered by `(mode, key)`, so
`DANCE` and `SKILL` can host multiple concrete adapters without overwriting each
other. `ModeRequest::enter_external(mode, key)` selects the concrete adapter;
`ModeRequest::enter(mode)` remains valid for the default adapter for that mode.

`fsm_external_policy_adapter.h` is a bridge for the existing native FSM policies
such as Dance and BeyondMimic. It copies `RobotSnapshot` / velocity command into
the old `StateAndCmd`, runs the wrapped `FSMState`, then converts `PolicyOutput`
back into `ExternalPolicyOutput` for the shared core. TrackMimic is treated as a
BeyondMimic trajectory variant: the same `BeyondMimicPolicy` implementation is
registered under the shared `SKILL` mode/key, and can consume a `motion_file`
trajectory from its YAML when that artifact is available.

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
LOCO still requires the explicit `--allow-loco` gate; real-runner DANCE requests
are ignored unless `--allow-dance` is also set with `--beyond-yaml PATH`. When
`--beyond-yaml PATH` is supplied, the runner registers BeyondMimic as the keyed
shared `DANCE` external policy and accepts gated `mode=beyond` / `mode=dance`
from UDP, `B` from keyboard input, or an explicitly configured gamepad dance
button. When `--track-mimic-yaml PATH` is supplied, the runner registers the
BeyondMimic trajectory variant as the keyed shared `SKILL` external policy;
`mode=skill` / `mode=track_mimic`, `T` from keyboard input, or an explicitly
configured gamepad skill button still require the separate `--allow-skill` gate
on real hardware.
The same UDP input now accepts `mode=passive` / `mode=damping` for shared
`PASSIVE` and `mode=final`, `mode=finaldamping`, or `mode=final_damping` for
shared `FINAL_DAMPING`; both clear velocity commands and route through
`ControllerCore`, not through runner-local control logic.

`mujoco_loco_viewer.cpp` now routes its closed-loop STAND/LOCO/PASSIVE execution
through the same `ControllerRuntime` and `MujocoSimAdapter`. Viewer input, UDP
commands, pause/reset, camera streaming, rendering, and ground correction remain
viewer responsibilities; ONNX inference, mode requests, target limiting, gains,
and MuJoCo PD torque publication now use the shared runtime path. Viewer-specific
initial-pose YAML overrides still feed the shared core default target and gains.
The viewer can write a telemetry JSON summary with `--summary-json`, run the same
scheduled push/impulse disturbance inputs as the rate smoke, and apply interactive
MuJoCo perturb forces with `Shift+left` / `Shift+middle` mouse dragging on a
selected body. When `--camera-stream` is enabled, the same HTTP server also
accepts `POST /reset`, `POST /control?...`, and `POST /viewer-event?...`; this
lets the web/Electron control station reset the sim, send mode/velocity requests,
and forward remote drag gestures into MuJoCo perturb/camera operations without
moving control logic out of `ControllerCore`.
When `--beyond-yaml PATH` is supplied, the viewer registers BeyondMimic as the
same keyed shared `DANCE` external policy used by the real runner; `B` and UDP
`mode=beyond` / `mode=dance` enter it.
When `--track-mimic-yaml PATH` is supplied, the viewer registers the BeyondMimic
trajectory variant as the keyed shared `SKILL` external policy; `T`, UDP text,
or HTTP `mode=skill` / `mode=track_mimic` enter it.
Viewer UDP and the real runner UDP input now share `text_control_command.h` for
text command tokenization and mode aliases (`loco`, `stand`, `passive`,
`final_damping`, `beyond`, `track_mimic`, `pause`, `resume`, `stop`, and
`zero`). The consumers still apply those parsed operations to their own input
state; the parser does not own policy, safety, or adapter behavior. It does own
the shared action effect for parsed text controls: which controls request a
mode, choose the external policy key, clear velocity, pause/resume, stop, toggle
LOCO, or request a re-stand/reset. Local viewer keyboard shortcuts `M` / `N`
request the same shared passive and final-damping modes.
The HTTP control endpoint accepts the same mode vocabulary through query
parameters, for example `POST /control?mode=final_damping` or
`POST /control?mode=loco&vx=0.2&wz=-0.1`; the main viewer loop consumes those
requests, maps mode aliases through the same shared action parser used by UDP,
and then still routes each tick through the shared runtime. The viewer now
builds a `ModeRequest` only when the operator-desired mode differs from
`ControllerCore::mode()`; steady ticks pass `ModeRequest::none()` so the viewer
does not continuously re-command the mode manager.
The same HTTP server exposes `GET /status` for control-station telemetry:
current `ControllerCore` mode, active external policy key/name, pause state,
velocity command, sim/policy steps, base pose, queue depths, and
disturbance/control counters. This is display/telemetry only; it does not move
policy or safety logic out of `ControllerCore`.
Both the viewer and real runner now build `ModeRequest` objects through the
shared mode helper, including the default `DANCE -> BeyondMimic` external policy
key and default `SKILL -> TrackMimic` external policy key, so entrypoints no
longer duplicate those mappings. TrackMimic is still a BeyondMimic-trained
policy path; the TrackMimic key selects the BeyondMimic implementation with its
extra trajectory input/config. The viewer also compares the requested external
key when deciding whether to send a new request, so the same `SKILL` mode can
switch trajectory variants without leaving and re-entering the mode first. The
dual-rate validation tool also uses the helper for LOCO entry requests.

The first `ControllerCore` implementation supports `PASSIVE`, `STAND`, `LOCO`,
and `FINAL_DAMPING`. `DANCE` and `SKILL` are represented in the shared mode enum
and intentionally reject requests until a matching external policy adapter is
registered.

This keeps the real-robot safety ladder intact: high-risk LOCO and DANCE require
the existing CLI `--allow-loco` and `--allow-dance` gates, SKILL/TrackMimic
requires `--allow-skill`, and adapters remain responsible only for
state/command I/O.

## Validation commands

Shared text-control parser check:

```bash
scripts/run_text_control_parser_check_native.sh
```

Shared mode transition check:

```bash
scripts/run_mode_manager_check_native.sh
```

Shared ControllerCore mode output check:

```bash
scripts/run_controller_core_check_native.sh
```

This loads the loco YAML and ONNX policy, then verifies that shared
`ControllerCore` STAND stays position-targeted while PASSIVE and FINAL_DAMPING
produce damping-only targets seeded from the current robot state. It also runs
`ControllerRuntime` with a fake adapter to verify the shared
`read_snapshot -> core.step -> write_target` flow and `publish_target=false`
behavior, and registers fake DANCE/SKILL external policies to verify keyed
policy selection, reset/step calls, entry command zeroing, and completion back
to a damping mode.

Real-runner UDP input smoke (no robot connection):

```bash
scripts/run_magicbot_loco_input_check_smoke_native.sh
```

Real-runner safety-gate smoke (no robot connection):

```bash
scripts/run_magicbot_loco_safety_gate_smoke_native.sh
```

This verifies explicit dry-run policy loading, the default dry-run mode, and
that `--run` is rejected before any robot connection path unless
`--allow-loco` is present.

Baseline closed-loop smoke:

```bash
scripts/run_dual_inference_rate_native.sh --mode pure-sim --duration 1.0 \
  --no-realtime --closed-loop-check \
  --summary-json /tmp/dual_no_push_refactor_smoke.json
```

Disturbance closed-loop smoke:

```bash
scripts/run_dual_push_smoke_native.sh --duration 1.0 --keep-summary
```

This wraps the native dual-rate runner in pure-sim mode, applies both a
scheduled force and an impulse to `pelvis`, and checks the summary for
`pass == true`, advancing sim/control steps, non-zero `push_force_steps`, and
`push_impulse_applied == true`.

Viewer disturbance smoke:

```bash
controller_cpp/build_mujoco_viewer/mujoco_loco_viewer --duration 0.4 \
  --unpaused --no-realtime --width 640 --height 480 \
  --push-body pelvis --push-force 25,0,0 --push-start 0.05 \
  --push-duration 0.08 --push-impulse 0,0.8,0 --push-impulse-time 0.18 \
  --summary-json /tmp/viewer_push_summary.json
```

Viewer BeyondMimic load smoke:

```bash
controller_cpp/build_mujoco_viewer/mujoco_loco_viewer --duration 0.2 \
  --paused --no-realtime --width 640 --height 480 \
  --beyond-yaml policies/beyond_mimic/config/BeyondMimic.yaml \
  --summary-json /tmp/viewer_beyond_summary.json
```

Viewer HTTP control smoke:

```bash
scripts/run_viewer_http_control_smoke_native.sh --duration 1.5 --keep-summary
```

The script starts the viewer HTTP server, posts reset plus
`passive -> stand -> loco -> pause -> resume -> final_damping`, verifies an invalid mode returns
HTTP 400, checks live `/status` for `mode == FINAL_DAMPING`, `paused == false`,
and `http_control_commands >= 6`, then checks the summary for
`mode == FINAL_DAMPING`, `http_control_commands >= 6`,
`http_reset_requests >= 1`, and advancing `sim_steps`.

Viewer HTTP SKILL / TrackMimic smoke:

```bash
scripts/run_viewer_http_skill_smoke_native.sh --duration 0.8 --keep-summary
```

The script registers the BeyondMimic trajectory variant as the shared `SKILL`
external policy in the viewer, posts `mode=track_mimic` through `/control`, then
checks live `/status` and the summary for `mode == SKILL`, advancing
`sim_steps` and `policy_steps`, active `external_policy == TrackMimic`, and at
least one HTTP control command. This is an entry-path smoke, not a stability
acceptance test.

Viewer UDP control smoke:

```bash
scripts/run_viewer_udp_control_smoke_native.sh --duration 1.5 --keep-summary
```

The script sends viewer UDP text-control packets for
`loco -> pause -> resume -> passive -> stand -> final_damping`, then checks the
summary for `mode == FINAL_DAMPING`, `paused == false`, and advancing
`sim_steps`.

Viewer HTTP remote perturb smoke:

```bash
scripts/run_viewer_http_perturb_smoke_native.sh --duration 1.5 --keep-summary
```

The script posts `type=down`, `type=move`, and `type=up` to `/viewer-event`,
keeps the drag active for a short wall-clock interval, and checks the summary
for `mouse_perturb_steps > 0`, resolved perturb body metadata, and advancing
`sim_steps`.

## Next cuts

1. Register additional concrete BeyondMimic skill/trajectory variants through
   `FsmExternalPolicyAdapter` where those entrypoints need them.
2. Move viewer mode/control API code to send requests only; it must not duplicate
   core policy or safety logic.
