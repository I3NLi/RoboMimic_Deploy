# RoboMimic Deploy MagicBot

MagicBot Z1 ONNX 策略的原生 C++ 部署仓库。

本分支已经移除旧解释器参考栈，当前主入口是：

```text
controller_cpp/build_native/magicbot_z1_loco_onnx
```

真机测试必须按下面的安全阶梯走。

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

端点：

```text
/health
/frame.jpg
/frame.png
/stream.mjpg
```

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

键盘映射：`L` 切换 `STAND/LOCO`，`R` 重新插值回站姿并 reset policy，`W/S` 调 `vx`，`Q/E` 调 `vy`，`A/D` 调 `wz`，`X` 清零，`Space/P` 暂停清零，`Esc` 退出 run loop。

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
  --gamepad-deadman-button 4 \
  --gamepad-stop-button 1 \
  --duration 5 \
  --local-ip 192.168.54.119
```

默认手柄映射：左摇杆 Y 为 `vx`，左摇杆 X 为 `vy`，右摇杆 X 为 `wz`；button 0 进 `LOCO`，button 3 回 `STAND`，button 6 重新插值回站姿并 reset policy，button 2 暂停清零，button 7 切换暂停清零。默认必须按住 button 4 才会输出非零命令，button 1 退出 run loop。轴和按键编号都可以用命令行参数改。

## 运行说明

- `--allow-loco` 是进入 ONNX loco 的显式开关。
- `--input-check` 只检查键盘/手柄输入，不连接机器人。
- `--pd-stand-only` 只做默认站立，不进入 loco。
- `--duration <= 0` 表示一直保持，直到收到停止信号。
- `--keyboard-control` 和 `--gamepad-control` 不能同时启用；二者会接管 `STAND/LOCO/reset/zero/stop` 和归一化速度命令。
- 正常退出或安全墙触发时都会发布最终阻尼命令。
- ONNX 按配置里的 `policy_dt` 运行，低层命令循环目标为 500 Hz。

## 当前验证

在 MagicBot 主机上已经验证：

- native dry-run 通过，模型维度为 `82 -> 24`；
- connect-check 通过；
- read-state 能收到腿 12、臂 14、腰 1、头 2；
- passive damping 发布链路通过；
- 零速度 loco 5 秒正常退出，没有触发安全墙。
