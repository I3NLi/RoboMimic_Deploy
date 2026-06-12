# Native Controller

This directory contains the C++ runtime for MagicBot Z1 policy deployment.

## Main Target

```text
magicbot_z1_loco_onnx
```

The target is split into three layers:

```text
include/magicbot_loco_core.h
src/magicbot_loco_core.cpp
  YAML config loading, ONNX Runtime session, observation construction,
  target limiting, rate watchdog, and motion safety checks.

include/magicbot_loco_sdk_adapter.h
src/magicbot_loco_sdk_adapter.cpp
  MagicBot Z1 SDK connection, state subscriptions, TTS, and grouped
  JointCommand publishing.

src/magicbot_z1_loco_onnx.cpp
  CLI, staged robot flow, PD stand, loco loop, final stand, and final damping.
```

## Build

Use the repository launcher:

```bash
../scripts/run_magicbot_loco_native.sh --dry-run
```

Or build directly:

```bash
cmake -S . -B build_native \
  -DCMAKE_BUILD_TYPE=Release \
  -DMAGICBOT_Z1_SDK_ROOT=/home/eame/magicbot-z1_sdk-main \
  -DONNXRUNTIME_INCLUDE_DIR=/home/eame/onnxruntime/include \
  -DONNXRUNTIME_LIB=/path/to/libonnxruntime.so.1

cmake --build build_native --target magicbot_z1_loco_onnx -j"$(nproc)"
```

## Runtime

```bash
build_native/magicbot_z1_loco_onnx --dry-run \
  --config ../policies/loco_mode/config/LocoMode_lowKp.yaml
```

Real-robot execution should be started through:

```bash
../scripts/run_magicbot_loco_native.sh
```

That launcher handles common SDK and ONNX Runtime locations and keeps the runtime library path stable.

Live input for the real-robot run loop is explicit:

```bash
../scripts/run_magicbot_loco_native.sh --input-check --gamepad-control --gamepad-device /dev/input/js0 --duration 10
../scripts/run_magicbot_loco_native.sh --run --allow-loco --keyboard-control --duration 5
../scripts/run_magicbot_loco_native.sh --run --allow-loco --gamepad-control --gamepad-device /dev/input/js0 --duration 5
```

`--input-check` does not connect to the robot. With live input enabled, the run loop starts in `STAND`; `L` or gamepad button 0 enters `LOCO`. Keyboard uses `L`, `R`, `W/S`, `Q/E`, `A/D`, `X`, `Space/P`, and `Esc`. Gamepad defaults to left-stick Y/X for `vx/vy`, right-stick X for `wz`, button 0 as LOCO, button 3 as STAND, button 6 as re-stand/reset, button 4 as deadman, and button 1 as stop.

## Native Simulation And Tools

```bash
../scripts/run_mujoco_loco_viewer_native.sh
```

Builds and runs `mujoco_loco_viewer`, an X11/GLX MuJoCo window with native loco inference. Keys:

```text
L      toggle STAND / LOCO
Space  pause
R      reset
F      camera follow
X      zero command
W/S    vx
Q/E    vy
A/D    wz
Esc    close
```

The viewer starts paused by default. Use Space to run, or pass `--unpaused`.

The same target can publish the configured head-camera stream:

```bash
../scripts/run_mujoco_loco_viewer_native.sh --camera-stream --camera-port 18080
```

It serves `/health`, `/frame.jpg`, `/frame.png`, and `/stream.mjpg`.

If ROS2 Humble is installed, the same viewer can publish RGB/RGBA image topics:

```bash
../scripts/run_mujoco_loco_viewer_native.sh --camera-ros2
```

```bash
../scripts/run_mujoco_dds_sim_native.sh --net lo --dry-run --max-steps 100
```

Builds and runs `mujoco_dds_simulator`, the native MuJoCo DDS bridge.

```bash
../scripts/run_onnx_benchmark_native.sh --iters 5000 --warmup 200 --threads 1 --obs-dim 82
```

Builds and runs `onnx_benchmark`.

```bash
../scripts/run_dual_inference_rate_native.sh --mode pure-sim --duration 10
../scripts/run_dual_inference_rate_native.sh --mode real-state-sim --real-forward-only --duration 10
```

Builds and runs `dual_inference_rate`. The real-state mode subscribes LowLevel state only; it does not publish joint commands.

```bash
../scripts/run_dual_inference_rate_native.sh \
  --mode pure-sim \
  --duration 2 \
  --no-realtime \
  --closed-loop-check \
  --summary-json ../logs/closed_loop_smoke.json
```

Runs a closed-loop acceptance smoke test. Failed thresholds return exit code `2`.

`--vx/--vy/--wz` are normalized command inputs; the loco YAML maps `lin_vel_x/lin_vel_y` to `[-2.5, 2.5]`.

```bash
../scripts/run_closed_loop_sweep_native.sh --axis vx --values "0 0.1 0.2 0.3 0.5 1" --duration 2 --keep-going
```

Runs a speed-envelope sweep and stores per-point summaries in `logs/closed_loop_sweep/`.

`robot_controller`, `robot_controller_onnx`, `onnx_benchmark`, `mujoco_dds_simulator`, `mujoco_loco_viewer`, and `dual_inference_rate` are built when their native dependencies are available. The MagicBot SDK loco path does not require Unitree DDS.
