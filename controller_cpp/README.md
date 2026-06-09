# controller_cpp 文档

`controller_cpp` 是当前仓库里的 C++ DDS/ONNX 控制器与 shadow compare 工具集。它主要用于：

- 对齐 Python `BeyondMimic` 与 C++ ONNX 推理输出；
- 通过 DDS `rt/lowstate` / `rt/lowcmd` 做 MuJoCo shadow compare；
- 验证 C++ 侧 LocoMode、BeyondMimic、TrackMimic 等当前实际存在的 Z1 策略配置。

注意：当前 MagicBot Z1 真机优先使用主 README 中的 MagicBot 官方 SDK loco 后端。`controller_cpp` 仍是 DDS 风格链路，不应被当成当前 Z1 真机主入口，更不要直接用它跑 BeyondMimic 真机。

## 当前覆盖范围

当前工作区实际存在并可被文档化的策略目录如下：

| 状态/策略 | 当前 C++ 侧说明 |
| --- | --- |
| `PASSIVE` | 已实现，阻尼/保护类状态 |
| `FIXEDPOSE` | 已实现，固定站姿/插值类状态 |
| `LOCOMODE` | ONNX 实现，配置来自 `policies/loco_mode/config/LocoMode_lowKp.yaml` |
| `SKILL_BEYOND_MIMIC` | ONNX 实现，通过 `--yaml policies/beyond_mimic/config/BeyondMimic.yaml` 注册 |
| `SKILL_TRACK_MIMIC` | 可选 ONNX 实现，通过 `--track-yaml policies/track_mimic/config/BeyondMimic.yaml` 注册 |
| `SKILL_COOLDOWN` | 已实现当前 Z1 24DoF 回归/过渡配置 |
| `JOINT_ZERO_CHECK` | 已实现 |
| `IMU_CALIB` | 已实现，运行时复用 LocoMode 相关输出 |

以下 legacy 技能在 C++ 代码里仍有注册钩子或 stub，但当前仓库没有对应 policy 目录，不作为当前能力宣传：

- `policies/dance/`
- `policies/kungfu/`
- `policies/kick/`
- `policies/kungfu2/`
- `policies/skill_cast/`

## 构建

在仓库根目录执行：

```bash
cmake -S controller_cpp -B controller_cpp/build_z1
cmake --build controller_cpp/build_z1 -j
```

生成的主要可执行文件：

- `controller_cpp/build_z1/robot_controller`
- `controller_cpp/build_z1/robot_controller_onnx`
- `controller_cpp/build_z1/onnx_benchmark`
- `controller_cpp/build_z1/mujoco_dds_simulator`，仅在找到 MuJoCo C API 时生成

如果 ONNX Runtime 或 MuJoCo 不在默认路径，配置时显式覆盖：

```bash
cmake -S controller_cpp -B controller_cpp/build_z1 \
  -DONNXRUNTIME_DIR=/path/to/onnxruntime-linux-x64 \
  -DMUJOCO_ROOT=/path/to/mujoco
cmake --build controller_cpp/build_z1 -j
```

## Shadow Compare

推荐从仓库根目录使用脚本：

```bash
bash scripts/compare_python_cpp.sh --verify --net lo
```

该脚本会自动启动：

- C++ `robot_controller_onnx --shadow --sync-lowstate`
- Python `python_reference/simulation/mujoco_dds_compare.py`
- DDS LowState/LowCmd 桥
- diff CSV 与 summary JSON

默认验证对象是 BeyondMimic：

```bash
./controller_cpp/build_z1/robot_controller_onnx \
  --shadow \
  --sync-lowstate \
  --net lo \
  --joints 24 \
  --yaml policies/beyond_mimic/config/BeyondMimic.yaml \
  --shadow-state beyond
```

TrackMimic 对比：

```bash
bash scripts/compare_python_cpp.sh --verify --net lo \
  --shadow-state track \
  --track-yaml policies/track_mimic/config/BeyondMimic.yaml
```

## 命令行参数

- `--net IFACE`：DDS 网卡，例如 `lo`、`enp4s0`。
- `--joints N`：关节数，当前 Z1 默认 `24`。
- `--yaml PATH`：BeyondMimic YAML 配置路径。
- `--track-yaml PATH`：TrackMimic YAML 配置路径。
- `--shadow-state {beyond|track}`：`--shadow` 启动后 force 到的 FSM 状态。
- `--shadow`：跳过真机式启动等待，用于仿真/shadow compare。
- `--sync-lowstate`：按 LowState tick 驱动控制循环，推荐用于 compare。
- `--safety`：启用 C++ 安全过滤器。
- `--dry-run`：只计算，不发布命令。

## 手柄/遥控映射

与 Python 参考控制器保持一致的核心映射：

| 输入 | 命令 |
| --- | --- |
| `F1` | `PASSIVE` |
| `UP` 释放 | `PAUSE`，仅 mimic 类策略生效 |
| `START` 长按 | `POS_RESET` |
| `R1 + A` 长按 | `LOCO` |
| `R1 + X` 长按 | `SKILL_1`，当前 Z1 Python 主线映射到 BeyondMimic |
| `L1 + Y` 长按 | `SKILL_4` / BeyondMimic |
| `L1 + A` 长按 | `SKILL_7` / TrackMimic，需注册 `--track-yaml` |
| `SELECT` | 退出程序 |

`R1 + Y`、`B` 组合等 legacy 技能命令仍可能存在于代码路径中，但当前仓库没有对应策略目录，调试时不建议使用。

## ONNX Benchmark

纯 ONNXRuntime benchmark：

```bash
./controller_cpp/build_z1/onnx_benchmark \
  --model policies/beyond_mimic/model/policy.onnx \
  --obs-dim 124 \
  --iters 5000 \
  --warmup 500 \
  --threads 1
```

Python benchmark 入口：

```bash
python python_reference/tools/onnx_benchmark.py \
  --model policies/beyond_mimic/model/policy.onnx \
  --obs-dim 124 \
  --iters 5000 \
  --warmup 500 \
  --threads 1
```

已有记录见：`docs/inference_benchmark_2026-06-07.md`。

## 常见问题

### 一直等待 LowState

现象：`[Controller] Waiting for robot connection...`

排查：

- shadow compare 是否使用了 `--shadow`；
- `--net` 是否和 Python/DDS 侧一致；
- Python 侧是否已经发布 `rt/lowstate`；
- 本机 compare 建议统一使用 `--net lo`。

### ONNX 加载失败

排查：

- `--yaml` 或 `--track-yaml` 路径是否存在；
- YAML 内 `onnx_path` 是否能解析到真实模型；
- `ONNXRUNTIME_DIR` 是否指向包含 `include/` 与 `lib/` 的 ONNX Runtime 包；
- 模型输入输出维度是否和 YAML 中的 `num_obs`、`num_actions` 一致。

### `skill_cast` 相关路径不存在

这是当前工作区的正常状态。C++ 代码保留了 legacy `SkillCast` 注册钩子，但仓库没有 `policies/skill_cast/`，因此文档不再把它列为当前支持策略。

### 是否可以用 C++ 跑真机 BeyondMimic

当前不建议。请先使用 Python MuJoCo 仿真与 `scripts/compare_python_cpp.sh --verify --net lo` 完成验证。MagicBot Z1 真机 loco 检查走 `python_reference/legacy_robot/run_magicbot_loco_reference.sh` 的 SDK 分阶段流程。
