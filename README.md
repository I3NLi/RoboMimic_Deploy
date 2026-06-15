# RoboMimic Deploy MagicBot

Native C++ deployment workspace for MagicBot Z1 ONNX policies.

The active runtime is `controller_cpp/build_native/magicbot_z1_loco_onnx`. The old interpreted reference stack has been removed from this branch. Real-robot execution must go through the staged safety flow below.

## Layout

```text
controller_cpp/
  include/magicbot_loco_core.h          # config, observation, ONNX, limits, safety
  include/magicbot_loco_sdk_adapter.h   # MagicBot SDK state and command adapter
  src/magicbot_loco_core.cpp
  src/magicbot_loco_sdk_adapter.cpp
  src/magicbot_z1_loco_onnx.cpp         # CLI and staged runtime
policies/
  loco_mode/config/LocoMode_lowKp.yaml
  loco_mode/model/policy.onnx
scripts/
  run_magicbot_loco_native.sh           # native launcher
  run_mujoco_loco_viewer_native.sh      # native MuJoCo interactive viewer
  run_mujoco_dds_sim_native.sh          # native MuJoCo DDS bridge
  run_onnx_benchmark_native.sh          # native ONNX Runtime benchmark
  run_dual_inference_rate_native.sh     # native sim / real-state rate test
```

## Shared Runtime Contract

`ControllerCore` owns observation, policy stepping, mode transitions, motion
safety, and target limiting. It emits a `JointTarget` with an explicit
`JointTargetMode`: `Position` for STAND/LOCO/DANCE/SKILL PD targets,
`ZeroTorque` for PASSIVE, and `Damping` for FINAL_DAMPING/safety-exit light
damping. Sim and real adapters only translate that target mode into MuJoCo or
MagicBot SDK I/O; they must not duplicate mode, policy, safety, or limit logic.

## Dependencies

- CMake 3.14+
- GCC/G++ with C++17 support
- `yaml-cpp`
- ONNX Runtime C/C++ headers and shared library
- MagicBot Z1 SDK headers and `libmagicbot_z1_sdk.so`

Useful environment variables:

```bash
export MAGICBOT_Z1_SDK_ROOT=/home/eame/magicbot-z1_sdk-main
export ONNXRUNTIME_INCLUDE_DIR=/home/eame/onnxruntime/include
export ONNXRUNTIME_LIB=/path/to/libonnxruntime.so.1
```

`scripts/run_magicbot_loco_native.sh` also searches common local paths and creates a stable runtime symlink under `controller_cpp/build_native/onnxruntime/`.

## Build And Dry Run

```bash
scripts/run_magicbot_loco_native.sh --dry-run
```

Expected shape for the current loco policy:

```text
ONNX input/output: 82 -> 24
```

## MuJoCo Interactive Viewer

Local simulation only, no real-robot connection:

```bash
scripts/run_mujoco_loco_viewer_native.sh
```

Window controls:

```text
L      toggle STAND / LOCO
Space  pause/resume
R      reset pose
F      toggle camera follow
X      zero vx/vy/wz
W/S    adjust vx
Q/E    adjust vy
A/D    adjust wz
Esc    close
```

Short checks:

```bash
scripts/run_mujoco_loco_viewer_native.sh --duration 3 --paused
scripts/run_mujoco_loco_viewer_native.sh --duration 5 --unpaused --loco
```

Camera HTTP stream:

```bash
scripts/run_mujoco_loco_viewer_native.sh --camera-stream --camera-port 18080
```

Control-station mode opens the camera HTTP stream, the viewer HTTP control API,
UDP velocity/mode control, and the BeyondMimic DANCE plus trajectory-conditioned
TrackMimic entrypoints when their YAML files exist. TrackMimic is a
BeyondMimic-trained path with an extra trajectory input, not a separate control
architecture:

```bash
scripts/run_mujoco_loco_viewer_native.sh --control-station
```

With viewer `--gamepad-control`, button 9/R3 toggles the runtime motion-safety
wall; override it with `--gamepad-safety-button N`.

Python-facing entrypoint for the same shared-runtime viewer:

```bash
scripts/run_python_mujoco_viewer.py --control-station
```

Endpoints:

```text
/health
/status         JSON: viewer, adapter, command, mode, target_mode, and safety telemetry
/frame.jpg
/frame.png
/stream.mjpg
/control        POST: mode/vx/vy/wz/pause plus safety=on|off|toggle
/reset          POST: request a simulation reset
/viewer-event   POST: forward remote pointer drag events for perturb/camera control
```

`scripts/run_viewer_stream_smoke_native.sh` validates `/frame.jpg` plus the
multipart MJPEG `/stream.mjpg` endpoint used by the virtual remote.

ROS2 image publishing:

```bash
scripts/run_mujoco_loco_viewer_native.sh --camera-ros2
```

Default topics:

```text
/z1/head_camera/rgb
/z1/head_camera/rgba
```

## Native Tools

ONNX inference benchmark:

```bash
scripts/run_onnx_benchmark_native.sh --iters 5000 --warmup 200 --threads 1 --obs-dim 82
```

MuJoCo DDS bridge:

```bash
scripts/run_mujoco_dds_sim_native.sh --net lo --dry-run --max-steps 100
```

Dual inference rate/load test:

```bash
scripts/run_dual_inference_rate_native.sh --mode pure-sim --duration 10
scripts/run_dual_inference_rate_native.sh --mode real-state-sim --real-forward-only --duration 10 --local-ip 192.168.54.119
```

`real-state-sim` only subscribes LowLevel state and runs local inference/MuJoCo timing. It does not publish joint commands.

Closed-loop acceptance smoke test:

```bash
scripts/run_dual_inference_rate_native.sh \
  --mode pure-sim \
  --duration 2 \
  --no-realtime \
  --closed-loop-check \
  --summary-json logs/closed_loop_smoke.json
```

`--closed-loop-check` validates control frequency, deadline misses, inference p99, base height, attitude drift, and joint velocity. A failed check exits with code `2`, which makes it usable as a CI or pre-robot gate.

`--vx/--vy/--wz` are normalized command inputs. The physical linear speed cap comes from the loco YAML `cmd_range`; `lin_vel_x/lin_vel_y` are set to `[-2.5, 2.5]`, so `--vx 1` maps to +2.5 m/s.

Speed-envelope sweep:

```bash
scripts/run_closed_loop_sweep_native.sh --axis vx --values "0 0.1 0.2 0.3 0.5 1" --duration 2 --keep-going
```

Each point writes a JSON summary under `logs/closed_loop_sweep/`; the script exits `2` if any point fails.

Manual build:

```bash
cmake -S controller_cpp -B controller_cpp/build_native \
  -DCMAKE_BUILD_TYPE=Release \
  -DMAGICBOT_Z1_SDK_ROOT=/home/eame/magicbot-z1_sdk-main \
  -DONNXRUNTIME_INCLUDE_DIR=/home/eame/onnxruntime/include \
  -DONNXRUNTIME_LIB=/path/to/libonnxruntime.so.1

cmake --build controller_cpp/build_native --target magicbot_z1_loco_onnx -j"$(nproc)"
```

## Real-Robot Safety Ladder

Run every step deliberately. Do not skip directly to loco after code, model, or robot-state changes.

```bash
scripts/run_magicbot_loco_native.sh --dry-run

scripts/run_magicbot_loco_native.sh \
  --connect-check \
  --local-ip 192.168.54.119

scripts/run_magicbot_loco_native.sh \
  --read-state \
  --duration 3 \
  --prepare-gait none \
  --local-ip 192.168.54.119

scripts/run_magicbot_loco_native.sh \
  --debug-entry-only \
  --local-ip 192.168.54.119 \
  --debug-entry-wait-s 1 \
  --debug-entry-passive-s 2 \
  --debug-entry-tts "Native passive damping test. Please keep clear."

scripts/run_magicbot_loco_native.sh \
  --run \
  --pd-stand-only \
  --duration 3 \
  --vx 0 --vy 0 --wz 0 \
  --local-ip 192.168.54.119 \
  --debug-entry \
  --debug-entry-wait-s 1 \
  --debug-entry-passive-s 2 \
  --stand-time 2 \
  --final-stand-time 1 \
  --final-stand-hold-s 0.5

scripts/run_dual_push_smoke_native.sh --duration 1.0
```

Only after the above is clean, start with a short zero-command LOCO run:

```bash
scripts/run_magicbot_loco_native.sh \
  --run \
  --allow-loco \
  --duration 5 \
  --vx 0 --vy 0 --wz 0 \
  --local-ip 192.168.54.119 \
  --debug-entry \
  --debug-entry-wait-s 3 \
  --debug-entry-passive-s 2 \
  --stand-time 2 \
  --pre-stand-hold-s 1 \
  --final-stand-time 1 \
  --final-stand-hold-s 0.5 \
  --rate-watchdog-min-hz 180 \
  --rate-watchdog-max-gap-ms 80 \
  --motion-safety-joint-scope body \
  --motion-max-joint-vel 25 \
  --motion-max-ang-vel 8 \
  --motion-max-gravity-xy 0.95 \
  --motion-max-default-dev 1.5 \
  --motion-max-target-error 1.2 \
  --motion-max-policy-target-dev 0 \
  --motion-max-policy-target-jump 0
```

Interactive input is only applied after the staged stand sequence. With keyboard/gamepad enabled, the run loop starts in `STAND` and requires an explicit `LOCO` request. Keyboard control:

```bash
scripts/run_magicbot_loco_native.sh \
  --run \
  --allow-loco \
  --keyboard-control \
  --duration 5 \
  --vx 0 --vy 0 --wz 0 \
  --local-ip 192.168.54.119
```

Keyboard map: `L` toggles `STAND/LOCO`, `R` resets the current policy/target without changing mode, `W/S` adjusts `vx`, `Q/E` adjusts `vy`, `A/D` adjusts `wz`, `X` zeros command, `Space/P` pause-zeros, and `Esc` exits the run loop.

Gamepad control:

```bash
scripts/run_magicbot_loco_native.sh \
  --input-check \
  --gamepad-control \
  --gamepad-device /dev/input/js0 \
  --duration 10

scripts/run_magicbot_loco_native.sh \
  --run \
  --allow-loco \
  --gamepad-control \
  --gamepad-device /dev/input/js0 \
  --duration 5 \
  --local-ip 192.168.54.119
```

Default Xbox-style gamepad map: left stick Y is `vx`, left stick X is `vy`, right stick X is `wz`. Button 0/A enters `LOCO`, button 1/B enters zero-torque `PASSIVE`, button 2/X pause-zeros, button 3/Y enters `STAND`, button 4/LB requests BeyondMimic, button 5/RB requests TrackMimic, button 6/Back resets the current policy/target without changing mode, button 7/Start toggles pause-zero, button 8/L3 exits the run loop, and button 9/R3 toggles the runtime motion-safety wall. Deadman is disabled by default; set `--gamepad-deadman-button N` if you want one. Axis and button indices are CLI-configurable.

## Runtime Notes

- `--allow-loco` is required for ONNX loco mode.
- `--input-check` reads keyboard/gamepad input only and does not connect to the robot.
- `--pd-stand-only` runs default-pose standing without ONNX loco.
- `--duration <= 0` holds the selected mode until interrupted.
- `--keyboard-control` and `--gamepad-control` are mutually exclusive; both drive `STAND/LOCO/reset/zero/stop` and normalized velocity commands.
- `PASSIVE` publishes zero torque, while `FINAL_DAMPING`/`damping` publishes light damping. The runtime safety wall can be controlled with text UDP `safety=on|off|toggle`, viewer HTTP `/control`, or gamepad R3.
- On exit or safety trip, the runner publishes a final damping command.
- The policy runs at the configured `policy_dt`; the low-level command loop targets 500 Hz.

## Current Validation

On the MagicBot host:

- native dry-run passes with `82 -> 24`;
- connect-check passes;
- read-state reports leg 12, arm 14, waist 1, head 2;
- passive damping command publishing passes;
- zero-command loco for 5 seconds exits cleanly with no safety trip.
