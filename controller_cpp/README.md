# controller_cpp 文档

## 1. 目标与范围

`controller_cpp` 是 `python_reference/legacy_robot/dds_robot_legacy.py` 的 C++ 版控制入口，当前按 MagicBot Z1（24 DoF）配置。

当前重点对齐路径：

- `SKILL_4 / BeyondMimic` 真机部署链路
- MuJoCo shadow 对比验证链路（`scripts/compare_python_cpp.sh --verify`）
- 主要技能策略（Loco/Dance/KungFu/Kick/KungFu2/SkillCooldown/SkillCast）ONNX 化部署链路

## 2. 构建

在仓库根目录执行：

```bash
cd controller_cpp
cmake -B build
cmake --build build -j
```

生成可执行文件：

- `build/robot_controller`
- `build/robot_controller_onnx`

说明：

- `robot_controller_onnx` 支持全策略 ONNX 注册（含 Loco/Dance/KungFu/Kick/KungFu2/SkillCooldown/SkillCast/BeyondMimic/TrackMimic）。
- `robot_controller` 为无 ONNX 依赖的基础版本。

## 3. 启动

### 3.1 真机启动（BeyondMimic）

```bash
cd /home/hiyio/MaigcLab/RoboMimic_Deploy_magicbot
./controller_cpp/build/robot_controller_onnx \
  --net enp4s0 \
  --yaml policies/beyond_mimic/config/BeyondMimic.yaml
```

### 3.2 可选注册 TrackMimic

```bash
./controller_cpp/build/robot_controller_onnx \
  --net enp4s0 \
  --yaml policies/beyond_mimic/config/BeyondMimic.yaml \
  --track-yaml policies/track_mimic/config/BeyondMimic.yaml
```

### 3.3 MuJoCo shadow 对比模式

```bash
./controller_cpp/build/robot_controller_onnx \
  --shadow \
  --sync-lowstate \
  --net lo \
  --yaml policies/beyond_mimic/config/BeyondMimic.yaml \
  --shadow-state beyond
```

TrackMimic 对比模式：

```bash
./controller_cpp/build/robot_controller_onnx \
  --shadow \
  --sync-lowstate \
  --net lo \
  --yaml policies/beyond_mimic/config/BeyondMimic.yaml \
  --track-yaml policies/track_mimic/config/BeyondMimic.yaml \
  --shadow-state track
```

## 4. 命令行参数

- `--net IFACE`：DDS 网卡（例如 `enp4s0`、`lo`）
- `--joints N`：关节数（默认 24）
- `--yaml PATH`：BeyondMimic YAML 配置路径
- `--track-yaml PATH`：TrackMimic YAML 配置路径（可选）
- `--shadow-state {beyond|track}`：`--shadow` 启动后自动 force 的 FSM 状态（默认 `beyond`）
- `--shadow`：跳过 `zero_torque_state()` 启动等待，并启用 LowState 非阻塞预热（用于仿真对比）
- `--sync-lowstate`：按 LowState tick 驱动控制循环（推荐用于 shadow compare）
- `--safety`：启用安全过滤器
- `--dry-run`：只算不发

## 5. 手柄映射（与 Python 版一致）

- `F1` -> `PASSIVE`
- `UP` 释放 -> `PAUSE`（仅 mimic 类策略生效）
- `START`（长按）-> `POS_RESET`
- `R1 + A`（长按）-> `LOCO`
- `R1 + X`（长按）-> `SKILL_1`
- `R1 + Y`（长按）-> `SKILL_2`
- `L1 + Y`（长按）-> `SKILL_4`（BeyondMimic）
- `L1 + X`（长按）-> `SKILL_6`（ImuCalib）
- `L1 + A`（长按）-> `SKILL_7`（TrackMimic）
- `SELECT` -> 退出程序

## 6. 状态实现覆盖

- `PASSIVE`：已实现
- `FIXEDPOSE`：已实现
- `LOCOMODE`：ONNX 实现（对应 `policies/loco_mode/config/LocoMode_lowKp.yaml`）
- `SKILL_DANCE`：ONNX 实现
- `SKILL_KUNGFU`：ONNX 实现
- `SKILL_KICK`：ONNX 实现
- `SKILL_KUNGFU2`：ONNX 实现
- `SKILL_CAST`：ONNX 实现（按配置关节集合插值）
- `SKILL_COOLDOWN`：ONNX/占位实现（按 Z1 24DoF 固定姿态回归）
- `SKILL_BEYOND_MIMIC`：ONNX 实现，已用于严格对比验证
- `SKILL_TRACK_MIMIC`：ONNX + `motion_file(npz)` 实现；需通过 `--track-yaml` 注册，未注册时回退占位模式
- `JOINT_ZERO_CHECK`：已实现（配置镜像）
- `IMU_CALIB`：已实现（运行时复用 LocoMode 输出）

## 7. 一键验证（推荐）

在仓库根目录执行：

```bash
bash scripts/compare_python_cpp.sh --verify --net lo
```

TrackMimic 验证：

```bash
bash scripts/compare_python_cpp.sh --verify --net lo \
  --shadow-state track \
  --track-yaml policies/track_mimic/config/BeyondMimic.yaml
```

该命令会：

- 启动 C++ shadow
- 启动 Python MuJoCo DDS bridge
- 生成差异 CSV/JSON
- 给出 `PASS/FAIL` 退出码

默认严格配置：

- `warmup_steps=80`
- `min_cmp_steps=200`
- `q_tol=1e-5`
- `mean_q_tol=1e-6`

## 8. 常见问题

### 8.1 一直等待连接

现象：`[Controller] Waiting for robot connection...`

排查：

- 网卡是否正确（`--net`）
- DDS 话题是否存在（`rt/lowstate`）
- 真机/仿真端是否已启动并发布 LowState

### 8.2 ONNX 加载失败

现象：`[Main][WARN] BeyondMimic load failed ...`

排查：

- `--yaml` 路径是否正确
- YAML 内 `onnx_path` 是否可解析
- ONNXRuntime 版本与模型是否兼容

### 8.3 verify 偶发首段误差偏大

建议保持默认 `warmup_steps=80`，避免初始两帧姿态对齐阶段影响统计。

在 `--shadow --sync-lowstate` 下，控制器会先完成策略加载再等待 LowState tick 驱动，从而避免 TrackMimic 因启动相位错位导致的早期大误差。

### 8.4 SkillCast/SkillCooldown 模型格式

这两个策略在 Python 侧原始模型是 `.pt`。C++ ONNX 版本默认加载对应的同名 `.onnx`：

- `policies/skill_cast/model/policy_stand_15dof.onnx`
- `policies/skill_cooldown/model/policy_15dof.onnx`

如果只存在 `.pt` 而没有 `.onnx`，C++ 会报加载失败。
