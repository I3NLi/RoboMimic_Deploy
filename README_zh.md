# RoboMimic Deploy for MagicBot Z1

<p align="center">
  <a href="README.md">English</a> | <strong>中文</strong>
</p>

本仓库是 RoboMimic Deploy 在 MagicBot Z1 上的部署工作区。当前代码已经从原始 G1/29DoF 项目迁移为 MagicBot Z1/24DoF 主线，重点目标是：

- Python MuJoCo 仿真部署与调试；
- BeyondMimic 全身跟踪策略在仿真中的稳定复现；
- Python 与 C++ ONNX 控制器的 shadow compare；
- MagicBot 官方 SDK 下的 Z1 loco 真机分阶段检查。

当前不要把这个仓库理解成“所有策略都已经可以直接上真机”的部署包。仿真是已验证主路径；真机侧目前只建议按 MagicBot SDK loco 后端的安全流程逐级检查。

## 当前状态

| 模块 | 状态 | 入口/配置 |
| --- | --- | --- |
| Python MuJoCo 仿真 | 当前主路径 | `python_reference/simulation/mujoco_reference.py` |
| Z1 MuJoCo 模型 | 24DoF | `assets/robots/magicbot_z1/scene.xml` |
| BeyondMimic | 当前 Z1 dance/全身跟踪入口 | `policies/beyond_mimic/config/BeyondMimic.yaml` |
| LocoMode | Z1 24DoF ONNX 行走入口，仍需仿真确认后再考虑实机 | `policies/loco_mode/config/LocoMode_lowKp.yaml` |
| Python/C++ 对比 | 可用 | `scripts/compare_python_cpp.sh --verify --net lo` |
| MagicBot SDK 真机 loco | 保守可检查路径 | `python_reference/legacy_robot/run_magicbot_loco_reference.sh` |
| Unitree DDS 旧入口 | 保留作历史/对比，不是当前 Z1 SDK 真机主路径 | `python_reference/legacy_robot/dds_robot_legacy.py` |

## 仓库结构

```text
RoboMimic_Deploy_magicbot/
├── assets/robots/magicbot_z1/      # Z1 MuJoCo XML 与 mesh
├── configs/
│   ├── simulation/                 # MuJoCo、初始姿态、安全配置
│   └── robot/                      # 旧 DDS/机器人侧配置
├── policies/
│   ├── passive/                    # 阻尼保护
│   ├── fixedpose/                  # 站姿插值
│   ├── loco_mode/                  # Z1 loco ONNX
│   ├── beyond_mimic/               # Z1 BeyondMimic ONNX
│   └── skill_cooldown/             # mimic 结束后的站姿回归
├── python_reference/
│   ├── simulation/                 # MuJoCo 仿真与 DDS 对比桥
│   ├── fsm/                        # 状态机
│   ├── runtime/                    # 控制、DDS、渲染、对比拆分模块
│   └── legacy_robot/               # MagicBot SDK loco wrapper 与 Unitree DDS 旧入口
├── controller_cpp/                 # C++ DDS/ONNX 控制器
├── scripts/compare_python_cpp.sh   # Python vs C++ shadow compare
└── docs/                           # 运行时拆分、benchmark、调试记录
```

## 环境准备

代码里已经使用 Python 3.10 语法，建议统一使用 Python 3.10。旧 README 中的 Python 3.8 不再作为当前基线。

```bash
conda create -n robomimic python=3.10
conda activate robomimic
pip install -r requirements.txt
```

可选依赖：

```bash
# 需要 Unitree DDS shadow compare 或 legacy DDS 入口时
git clone https://github.com/unitreerobotics/unitree_sdk2_python.git
cd unitree_sdk2_python
pip install -e .
```

MagicBot 真机 loco 后端还需要本机能找到 `magicbot-z1_sdk-main`。脚本会按 `MAGICBOT_SDK_ROOT`、`~/magicbot-z1_sdk-main`、`~/MaigcLab/magicbot-z1_sdk-main` 等路径搜索。

## 快速运行仿真

```bash
cd /home/hiyio/MaigcLab/RoboMimic_Deploy_magicbot
conda activate robomimic
python -u python_reference/simulation/mujoco_reference.py
```

默认配置来自：

- `configs/simulation/mujoco.yaml`
- `configs/simulation/magicbot_z1_stand.yaml`
- `configs/simulation/safety.yaml`

关键参数：

- MuJoCo XML：`assets/robots/magicbot_z1/scene.xml`
- 仿真步长：`simulation_dt: 0.002`
- 控制降采样：`control_decimation: 10`
- 控制周期：`0.02s`，即 50 Hz
- 初始命令：`PASSIVE`
- ghost 参考显示：默认开启，`ghost.mode: mesh`

没有手柄时，仿真入口会降级为中立输入 `NullJoyStick`，但此时只能观察默认状态，不能通过按键切换模式。

## 手柄操作

当前推荐只使用下面这条 Z1 主线：

| 操作 | 状态/效果 |
| --- | --- |
| 启动程序 | 默认进入 `PassiveMode` 阻尼保护 |
| 长按 `START` | 进入 `FixedPose`，2 秒插值到默认站姿 |
| 长按 `R1 + A` | 进入 `LocoMode` |
| 在 `FixedPose` 或 `LocoMode` 长按 `R1 + X` | 进入 `BeyondMimic` |
| 在 `FixedPose` 或 `LocoMode` 长按 `L1 + Y` | 进入 `BeyondMimic` |
| 在 mimic 中释放 `UP` | 暂停/恢复参考帧 |
| 在 mimic 中长按 `R1 + A` | 经 `SkillCooldown` 返回 `LocoMode` |
| 在 mimic 中长按 `START` | 回到 `FixedPose` |
| 释放 `L3` | 回到 `PassiveMode` |
| 按 `SELECT` | 退出仿真程序 |

代码里仍保留了一些旧技能命令映射，例如 `SKILL_2`、`SKILL_6`、`SKILL_7`。在当前 Python Z1 主路径里，这些不是推荐操作入口；旧 Dance/KungFu/Kick/TrackMimic 状态在 FSM 中主要作为兼容别名保留。

## 策略基线

### BeyondMimic

当前 Z1 dance/全身跟踪入口是 `BeyondMimic`：

- 模型：`policies/beyond_mimic/model/policy.onnx`
- 配置：`policies/beyond_mimic/config/BeyondMimic.yaml`
- 输入：`obs [1,124]` 与 `time_step [1,1]`
- 输出：24DoF action 与内嵌参考轨迹信息
- `motion_length: 3309`
- `switch_to_loco_delay_s: -1.0`，动作结束后保持在 mimic，不自动切回 loco
- `command_joint_indices` 排除了 ankle command，需与训练导出的观测保持一致
- `mj2lab` 是 LeggedLab/IsaacLab joint index 到 MuJoCo actuator index 的映射

如需临时切换 BeyondMimic YAML，可用环境变量：

```bash
BEYOND_MIMIC_CONFIG_PATH=/abs/path/to/BeyondMimic.yaml \
python -u python_reference/simulation/mujoco_reference.py
```

### LocoMode

当前 LocoMode 使用：

- 模型：`policies/loco_mode/model/z1_flat_reset_reasons_model_25800.onnx`
- 配置：`policies/loco_mode/config/LocoMode_lowKp.yaml`
- 输入维度：`82`
- 输出维度：`24`
- command 维度：`4`
- `root_height_command: 0.69`

LocoMode 已经是 Z1 24DoF 配置，但真机使用前仍要重新确认观测、归一化、关节顺序、动作缩放、限幅和初始姿态。

### FixedPose / Passive / SkillCooldown

- `PassiveMode`：零 `kp`，只保留阻尼 `kd`。
- `FixedPose`：从当前关节位置插值到默认站姿，默认 2 秒。
- `SkillCooldown`：当前是 blend-only，把 mimic 后姿态回归到 Z1 固定站姿，再返回 loco。

## 安全配置

安全滤波器支持动作限幅、动作增量限幅、增益限幅、增益增量限幅、故障阻尼回退和 dry-run。

当前 YAML 里 `enable: false`，也就是默认不主动 clamp 策略输出；但 `dry_run` 会被仿真主循环单独识别。

```yaml
# configs/simulation/safety.yaml
enable: false
dry_run: false
command_hold_frames: 2
max_action_abs: 3.5
max_action_delta: 0.3
damping_kd: 8.0
```

调试建议：

- 只想看策略计算、不想施加力矩：把 `dry_run: true`。
- 要观察安全限幅日志：把 `enable: true` 和 `log_clamps: true`。
- 真机前必须重新设置并验证 `configs/robot/safety.yaml`，不要直接沿用仿真参数。

## Python/C++ Shadow Compare

推荐一键验证：

```bash
cd /home/hiyio/MaigcLab/RoboMimic_Deploy_magicbot
bash scripts/compare_python_cpp.sh --verify --net lo
```

脚本会启动：

- C++ `robot_controller_onnx --shadow`
- Python `mujoco_dds_compare.py --headless --no-joystick --shadow-sync`
- DDS `rt/lowstate` / `rt/lowcmd` 对比桥
- 差异 CSV 与 summary JSON

如果 C++ 二进制不存在，先构建：

```bash
cmake -S controller_cpp -B controller_cpp/build_z1
cmake --build controller_cpp/build_z1 -j
```

C++ 子系统更多参数见 [`controller_cpp/README.md`](controller_cpp/README.md)。

## C++ 构建注意

`controller_cpp` 依赖：

- `unitree_sdk2`
- `yaml-cpp`
- `zlib`
- ONNX Runtime C/C++ 包
- 可选 MuJoCo C API，用于 `mujoco_dds_simulator`

`controller_cpp/CMakeLists.txt` 里有本机 ONNX Runtime 和 MuJoCo 的默认路径；如果你的机器路径不同，用 CMake 参数覆盖：

```bash
cmake -S controller_cpp -B controller_cpp/build_z1 \
  -DONNXRUNTIME_DIR=/path/to/onnxruntime-linux-x64 \
  -DMUJOCO_ROOT=/path/to/mujoco
cmake --build controller_cpp/build_z1 -j
```

## MagicBot Z1 真机 loco 检查

真机路径优先使用 MagicBot 官方 SDK 后端：

```bash
cd /home/hiyio/MaigcLab/RoboMimic_Deploy_magicbot
python_reference/legacy_robot/run_magicbot_loco_reference.sh --dry-run
python_reference/legacy_robot/run_magicbot_loco_reference.sh --connect-check --local-ip 192.168.54.119
python_reference/legacy_robot/run_magicbot_loco_reference.sh --read-state --local-ip 192.168.54.119 --duration 3
```

安全顺序：

1. 机器人吊起，确认急停、断电、网络隔离和人工接管链路可用。
2. `--dry-run`：只加载 YAML/ONNX 并跑一次推理，不连接机器人。
3. `--connect-check`：只连接/断开 SDK，不切 LowLevel。
4. `--read-state`：按 `HighLevel -> GAIT_RECOVERY_STAND -> LowLevel` 进入，只订阅低层状态，不发布 `JointCommand`。
5. 前面全部通过后，才允许显式运行 `--run --stand-only --duration N`。
6. 站立检查通过后，才考虑短时 `--run --duration N --vx 0 --vy 0 --wz 0`。

示例：

```bash
python_reference/legacy_robot/run_magicbot_loco_reference.sh \
  --run --stand-only --local-ip 192.168.54.119 --duration 2
```

不要直接把 BeyondMimic/dance 策略上真机。当前 BeyondMimic 真机 SDK 后端仍需要从仿真策略继续迁移与审计。

## Legacy DDS 与 C++ 真机入口

`python_reference/legacy_robot/dds_robot_legacy.py` 和 `controller_cpp` 保留了 Unitree DDS 风格的 `rt/lowcmd` / `rt/lowstate` 链路。这些代码主要用于历史兼容、C++ shadow compare 和 DDS 控制器开发，不是当前 MagicBot Z1 官方 SDK 真机主入口。

如果确实要走 DDS 真机链路，必须先重新确认：

- 网卡与 DDS domain；
- 机器人实际低层话题；
- 24 个电机的物理顺序；
- LowLevel 切换方式；
- 急停和阻尼命令是否真的生效。

## 常见问题

### 启动时报 `No joystick connected`

`mujoco_reference.py` 会自动降级到 `NullJoyStick`。要实际切换策略，需要连接手柄；自动对比时用 `mujoco_dds_compare.py --no-joystick`。

### C++ compare 一直等待 DDS

检查：

- `--net` 是否一致，建议本机 shadow compare 用 `lo`；
- `controller_cpp/build_z1/robot_controller_onnx` 是否存在；
- `unitree_sdk2` 与 CycloneDDS 运行库是否能被找到；
- Python 侧是否已经启动并发布 `rt/lowstate`。

### ONNX 加载失败

检查 YAML 中的 `onnx_path`。相对路径会按对应 policy 目录下的 `model/` 解析，例如 BeyondMimic 默认解析到 `policies/beyond_mimic/model/policy.onnx`。

### MuJoCo 出现 NaN/Inf 或机器人发散

先切回 `PassiveMode` 或关闭程序。然后检查：

- 初始姿态是否来自 `configs/simulation/magicbot_z1_stand.yaml`；
- 策略和 `mj2lab` 是否匹配；
- `safety.yaml` 是否需要打开 `enable: true` 或 `dry_run: true`；
- 是否切到了不推荐的 legacy 技能入口。

## 相关文档

- [`docs/runtime_split.md`](docs/runtime_split.md)：运行时分层说明
- [`docs/inference_benchmark_2026-06-07.md`](docs/inference_benchmark_2026-06-07.md)：ONNX 推理 benchmark
- [`controller_cpp/README.md`](controller_cpp/README.md)：C++ 控制器构建与 shadow compare 细节

## 一句话原则

先仿真，再 shadow compare，再 dry-run，再 connect-check，再 read-state；只有每一步都可解释、可复现、可急停，才进入真机命令发布。
