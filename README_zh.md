# RoboMimic Deploy MagicBot

MagicBot Z1 ONNX 策略的原生 C++ 部署仓库。

本分支已经移除旧解释器参考栈，当前主入口是：

```text
controller_cpp/build_native/magicbot_z1_loco_onnx
```

真机测试必须按下面的安全阶梯走。

## 项目目标

这个分支的目标是一套 sim/real 共用的控制栈：

- `ControllerCore` 输入 `RobotSnapshot + Command + ModeRequest`，输出
  `JointTarget + Gains + Telemetry`。
- PASSIVE、STAND、LOCO、DANCE、SKILL、FINAL_DAMPING 共享 `ControllerCore`
  里的策略步进、模式切换、安全检查和 target limit。
- `SimAdapter` 和 `RealAdapter` 只负责共享 target/state 与 MuJoCo 或
  MagicBot SDK I/O 之间的翻译。
- Viewer/control station 只做操作台：渲染、键盘、手柄、UDP/HTTP 控制、
  相机/视频显示、telemetry 和扰动注入；不能复制或分叉
  `ControllerCore` 的控制脑逻辑。
- Python MuJoCo viewer 要和 native shared-runtime viewer 对齐：支持
  pause/reset、模式切换、UDP/control API、camera stream，以及拖拽、外力、
  impulse 扰动检查和 JSON summary。

真机高风险模式继续显式 gated。LOCO、DANCE、SKILL 必须带 explicit allow，
并且必须先走完本 README 的安全阶梯。

## 目录

```text
controller_cpp/
  include/magicbot_loco_core.h          # 配置、观测、ONNX、限幅、安全墙
  include/magicbot_loco_sdk_adapter.h   # MagicBot SDK 状态与命令适配
  src/magicbot_loco_core.cpp
  src/magicbot_loco_sdk_adapter.cpp
  src/magicbot_z1_loco_onnx.cpp         # CLI 与分阶段真机流程
policies/
  loco_mode/config/LocoMode_lowKp.yaml
  loco_mode/model/policy.onnx
scripts/
  run_magicbot_loco_native.sh           # 原生启动脚本
  run_mujoco_loco_viewer_native.sh      # 原生 MuJoCo 交互窗口
  run_mujoco_dds_sim_native.sh          # 原生 MuJoCo DDS bridge
  run_onnx_benchmark_native.sh          # 原生 ONNX Runtime 基准
  run_dual_inference_rate_native.sh     # 原生仿真/真机状态速率测试
```

## 共享运行时契约

`ControllerCore` 负责 observation、策略步进、模式切换、运动安全墙和
target limit。它输出带显式 `JointTargetMode` 的 `JointTarget`：
`Position` 用于 STAND/LOCO/DANCE/SKILL 的 PD 目标，`ZeroTorque` 用于
PASSIVE 零力矩，`Damping` 用于 FINAL_DAMPING/安全退出时的轻阻尼。sim 和
real adapter 只把这个 target mode 翻译成 MuJoCo 或 MagicBot SDK I/O，
不能复制 mode、policy、safety 或 limit 逻辑。

## 依赖

- CMake 3.14+
- 支持 C++17 的 GCC/G++
- `yaml-cpp`
- ONNX Runtime C/C++ 头文件与动态库
- MagicBot Z1 SDK 头文件与 `libmagicbot_z1_sdk.so`

常用环境变量：

```bash
export MAGICBOT_Z1_SDK_ROOT=/home/eame/magicbot-z1_sdk-main
export ONNXRUNTIME_INCLUDE_DIR=/home/eame/onnxruntime/include
export ONNXRUNTIME_LIB=/path/to/libonnxruntime.so.1
```

`scripts/run_magicbot_loco_native.sh` 会查找常见本地路径，并在 `controller_cpp/build_native/onnxruntime/` 下创建稳定的运行时软链接。

## 构建与 Dry Run

```bash
scripts/run_magicbot_loco_native.sh --dry-run
```

当前 loco 策略应显示：

```text
ONNX input/output: 82 -> 24
```

## MuJoCo 交互窗口

只看本机仿真，不连接真机：

```bash
scripts/run_mujoco_loco_viewer_native.sh
```

窗口键盘控制：

```text
L      切换 STAND / LOCO
Space  暂停/继续
R      重置姿态
F      相机跟随开关
X      清零 vx/vy/wz
W/S    vx 增减
Q/E    vy 增减
A/D    wz 增减
Esc    关闭窗口
```

短时验证：

```bash
scripts/run_mujoco_loco_viewer_native.sh --duration 3 --paused
scripts/run_mujoco_loco_viewer_native.sh --duration 5 --unpaused --loco
```

相机 HTTP 流：

```bash
scripts/run_mujoco_loco_viewer_native.sh --camera-stream --camera-port 18080
```

viewer 开启 `--gamepad-control` 时，button 9/R3 会切换运行时 safety wall；
可用 `--gamepad-safety-button N` 覆盖。

端点：

```text
/health
/status         JSON: viewer、adapter、command、mode、target_mode、safety 和 camera telemetry
/frame.jpg
/frame.png
/stream.mjpg
/control        POST: mode/vx/vy/wz/pause，以及 safety=on|off|toggle
/reset          POST: 只请求仿真位置重置
/viewer-event   POST: 遥控器拖拽扰动/相机事件
```

`scripts/run_viewer_stream_smoke_native.sh` 会验证 `/frame.jpg` 和虚拟遥控器使用的
multipart MJPEG `/stream.mjpg` 端点。

ROS2 图像发布：

```bash
scripts/run_mujoco_loco_viewer_native.sh --camera-ros2
```

默认发布：

```text
/z1/head_camera/rgb
/z1/head_camera/rgba
```

## 原生工具

ONNX 推理基准：

```bash
scripts/run_onnx_benchmark_native.sh --iters 5000 --warmup 200 --threads 1 --obs-dim 82
```

MuJoCo DDS bridge：

```bash
scripts/run_mujoco_dds_sim_native.sh --net lo --dry-run --max-steps 100
```

双推理速率/负载测试：

```bash
scripts/run_dual_inference_rate_native.sh --mode pure-sim --duration 10
scripts/run_dual_inference_rate_native.sh --mode real-state-sim --real-forward-only --duration 10 --local-ip 192.168.54.119
```

`real-state-sim` 只订阅 LowLevel 状态，在本地跑推理和 MuJoCo 计时，不发布关节命令。

闭环验收烟测：

```bash
scripts/run_dual_inference_rate_native.sh \
  --mode pure-sim \
  --duration 2 \
  --no-realtime \
  --closed-loop-check \
  --summary-json logs/closed_loop_smoke.json
```

`--closed-loop-check` 会检查控制频率、deadline miss、推理 p99、base height、姿态漂移和关节速度等指标；失败时进程返回 `2`，适合接入 CI 或真机前置门槛。

当前 `--vx/--vy/--wz` 是归一化命令，实际线速度范围由 loco YAML 的 `cmd_range` 映射；`lin_vel_x/lin_vel_y` 已按 `[-2.5, 2.5]` 配置，所以 `--vx 1` 对应 +2.5 m/s。

速度包线 sweep：

```bash
scripts/run_closed_loop_sweep_native.sh --axis vx --values "0 0.1 0.2 0.3 0.5 1" --duration 2 --keep-going
```

每个点会输出独立 JSON 到 `logs/closed_loop_sweep/`；任一速度点失败时脚本返回 `2`。

无硬件 shared-runtime 烟测套件：

```bash
scripts/run_shared_runtime_smoke_suite_native.sh --core-only
scripts/run_shared_runtime_smoke_suite_native.sh --sim-only
scripts/run_shared_runtime_smoke_suite_native.sh --viewer-only
scripts/run_shared_runtime_smoke_suite_native.sh --python-only
scripts/run_shared_runtime_smoke_suite_native.sh --real-only
```

完整套件会运行上面全部分组：

```bash
scripts/run_shared_runtime_smoke_suite_native.sh
```

`--core-only` 检查文本控制、模式切换、ControllerCore 输出模式、
LOCO 退出到非策略模式、ControllerRuntime，以及不依赖后端的共享头文件。
`--sim-only` 检查
MuJoCo adapter 的 `JointTargetMode` 翻译，然后运行 pure-sim 闭环外力/impulse
烟测。`--viewer-only` 检查 native viewer 的 HTTP/UDP/手柄、视频流、
DANCE/SKILL、HTTP 和手柄安全墙控制，以及扰动 API。`--python-only` 通过
Python-facing launcher 运行同一组 viewer 烟测。`--real-only` 不连接机器人；
它检查 MagicBot real adapter target-mode 翻译、real operator loop 的
`ControllerRuntime.tick` 边界，以及 dry/input/手柄安全墙/allow-gate 行为。

手动构建：

```bash
cmake -S controller_cpp -B controller_cpp/build_native \
  -DCMAKE_BUILD_TYPE=Release \
  -DMAGICBOT_Z1_SDK_ROOT=/home/eame/magicbot-z1_sdk-main \
  -DONNXRUNTIME_INCLUDE_DIR=/home/eame/onnxruntime/include \
  -DONNXRUNTIME_LIB=/path/to/libonnxruntime.so.1

cmake --build controller_cpp/build_native --target magicbot_z1_loco_onnx -j"$(nproc)"
```

## 真机安全阶梯

每次换代码、换模型、换机器状态后都从头走。不要直接跳到 loco。

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

上面都正常后，再进入 5 秒零速度 loco：

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

交互输入只在完成前置站立后生效，不能跳过前面的站立和安全墙。启用键盘/手柄时，run loop 默认先保持 `STAND`，需要显式切到 `LOCO`。键盘控制：

```bash
scripts/run_magicbot_loco_native.sh \
  --run \
  --allow-loco \
  --keyboard-control \
  --duration 5 \
  --vx 0 --vy 0 --wz 0 \
  --local-ip 192.168.54.119
```

键盘映射：`L` 切换 `STAND/LOCO`，`R` 只重置当前 policy/target 且不切换模式，`W/S` 调 `vx`，`Q/E` 调 `vy`，`A/D` 调 `wz`，`X` 清零，`Space/P` 暂停清零，`Esc` 退出 run loop。

手柄控制：

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

默认 Xbox 风格手柄映射：左摇杆 Y 为 `vx`，左摇杆 X 为 `vy`，右摇杆 X 为 `wz`；button 0/A 进 `LOCO`，button 1/B 进零力矩 `PASSIVE`，button 2/X 暂停清零，button 3/Y 进 `STAND`，button 4/LB 请求 BeyondMimic，button 5/RB 请求 TrackMimic，button 6/Back 只重置当前 policy/target 且不切换模式，button 7/Start 切换暂停清零，button 8/L3 退出 run loop，button 9/R3 切换运行时 safety wall。默认不需要 deadman；如果需要可设置 `--gamepad-deadman-button N`。轴和按键编号都可以用命令行参数改。

完整 FSM 虚拟遥控器：

`/home/hiyio/MaigcLab/magicbot-virtual-remote` 提供 Xbox 风格 Electron 遥控器。对 `robot_controller_onnx` 主线，建议走 `wireless_remote` 兼容 UDP 帧，而不是 Linux `/dev/input/js*`：

```bash
scripts/run_robot_controller_virtual_remote_native.sh \
  --net lo \
  --virtual-remote-port 15001
```

遥控器可以在另一台主机上运行，把页面里的 `Remote Host` 填成控制器主机 IP，`FSM UDP` 填 `15001`。按键语义对齐 C++ FSM：`START=POS_RESET`，`R1+A=LOCO`，`R1+X/Y/B=SKILL_1/2/3`，`L1+Y/B/X/A=SKILL_4/5/6/7`，`F1=PASSIVE`，`F2/R3=safety=toggle`，`UP release=PAUSE`，`SELECT=退出`。控制器侧 UDP 超时会自动归零遥控器状态。

## 运行说明

- `--allow-loco` 是进入 ONNX loco 的显式开关。
- `--input-check` 只检查键盘/手柄输入，不连接机器人。
- `--pd-stand-only` 只做默认站立，不进入 loco。
- `--duration <= 0` 表示一直保持，直到收到停止信号。
- `--keyboard-control` 和 `--gamepad-control` 不能同时启用；二者会接管 `STAND/LOCO/reset/zero/stop` 和归一化速度命令。
- `PASSIVE` 是零力矩，`FINAL_DAMPING`/`damping` 是轻阻尼；运行时 safety wall 可以通过文本 UDP `safety=on|off|toggle`、viewer HTTP `/control` 或手柄 R3 切换。
- 正常退出或安全墙触发时都会发布最终阻尼命令。
- ONNX 按配置里的 `policy_dt` 运行，低层命令循环目标为 500 Hz。

## 当前验证

在 MagicBot 主机上已经验证：

- native dry-run 通过，模型维度为 `82 -> 24`；
- connect-check 通过；
- read-state 能收到腿 12、臂 14、腰 1、头 2；
- passive damping 发布链路通过；
- 零速度 loco 5 秒正常退出，没有触发安全墙。
