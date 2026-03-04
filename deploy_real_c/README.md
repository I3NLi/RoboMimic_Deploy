# deploy_real_c 文档

## 1. 目标与范围

`deploy_real_c` 是 `deploy_real/deploy_real.py` 的 C++ 版控制入口，面向 Unitree G1（29 DoF）。

当前重点对齐路径：

- `SKILL_4 / BeyondMimic` 真机部署链路
- MuJoCo shadow 对比验证链路（`start_compare.sh --verify`）
- 主要技能策略（Loco/Dance/KungFu/Kick/KungFu2/SkillCooldown/SkillCast）ONNX 化部署链路

## 2. 构建

在仓库根目录执行：

```bash
cd deploy_real_c
cmake -B build
cmake --build build -j
```

生成可执行文件：

- `build/deploy_real`
- `build/deploy_real_onnx`

说明：

- `deploy_real_onnx` 支持全策略 ONNX 注册（含 Loco/Dance/KungFu/Kick/KungFu2/SkillCooldown/SkillCast/BeyondMimic/TrackMimic）。
- `deploy_real` 为无 ONNX 依赖的基础版本。

## 3. 启动

### 3.1 真机启动（BeyondMimic）

```bash
cd /home/hiyio/RoboMimic_Deploy
./deploy_real_c/build/deploy_real_onnx \
  --net enp4s0 \
  --yaml policy/beyond_mimic/config/BeyondMimic.yaml
```

### 3.2 可选注册 TrackMimic

```bash
./deploy_real_c/build/deploy_real_onnx \
  --net enp4s0 \
  --yaml policy/beyond_mimic/config/BeyondMimic.yaml \
  --track-yaml policy/track_mimic/config/BeyondMimic.yaml
```

### 3.3 MuJoCo shadow 对比模式

```bash
./deploy_real_c/build/deploy_real_onnx \
  --shadow \
  --sync-lowstate \
  --net lo \
  --yaml policy/beyond_mimic/config/BeyondMimic.yaml
```

## 4. 命令行参数

- `--net IFACE`：DDS 网卡（例如 `enp4s0`、`lo`）
- `--joints N`：关节数（默认 29）
- `--yaml PATH`：BeyondMimic YAML 配置路径
- `--track-yaml PATH`：TrackMimic YAML 配置路径（可选）
- `--shadow`：跳过 `zero_torque_state()` 启动等待（用于仿真对比）
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
- `LOCOMODE`：ONNX 实现（对应 `policy/loco_mode/config/LocoMode_lowKp.yaml`）
- `SKILL_DANCE`：ONNX 实现
- `SKILL_KUNGFU`：ONNX 实现
- `SKILL_KICK`：ONNX 实现
- `SKILL_KUNGFU2`：ONNX 实现
- `SKILL_CAST`：ONNX 实现（15DoF 下肢 + 上肢插值）
- `SKILL_COOLDOWN`：ONNX 实现（15DoF 下肢 + 上肢回归）
- `SKILL_BEYOND_MIMIC`：ONNX 实现，已用于严格对比验证
- `SKILL_TRACK_MIMIC`：可通过 `--track-yaml` 注册 ONNX；未注册时回退到占位模式
- `JOINT_ZERO_CHECK`：已实现（配置镜像）
- `IMU_CALIB`：已实现（配置镜像）

## 7. 一键验证（推荐）

在仓库根目录执行：

```bash
bash start_compare.sh --verify --net lo
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

### 8.4 SkillCast/SkillCooldown 模型格式

这两个策略在 Python 侧原始模型是 `.pt`。C++ ONNX 版本默认加载对应的同名 `.onnx`：

- `policy/skill_cast/model/policy_stand_15dof.onnx`
- `policy/skill_cooldown/model/policy_15dof.onnx`

如果只存在 `.pt` 而没有 `.onnx`，C++ 会报加载失败。
