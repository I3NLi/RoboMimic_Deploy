<div align="center">
  <h1 align="center">RoboMimic Deploy</h1>
  <p align="center">
    <a href="README.md">🌎 English</a> | <span>🇨🇳 中文</span>
  </p>
</div>

<p align="center">
  🎮🚪 <strong>RoboMimic Deploy 是一个基于状态切换机制的机器人多策略部署框架。当前工作区已适配 MagicBot Z1 24DoF 仿真部署。</strong> 🚪🎮
</p>

## 写在前面

- **当前 `/home/hiyio/MaigcLab/RoboMimic_Deploy_magicbot` 工作区使用 MagicBot Z1 24 自由度模型，不再是原始 G1 29DoF 配置。**

- **MuJoCo XML 使用 `/home/hiyio/HoloMotion/thirdparties/GMR/assets/magicbot_z1/mjcf/MAGICBOTZ1.xml`。**

- **当前验证重点是仿真环境。真机部署前必须重新确认 SDK、电机顺序、限幅、初始姿态和急停链路。**

- **[视频教程](https://www.bilibili.com/video/BV1VTKHzSE6C/?vd_source=713b35f59bdf42930757aea07a44e7cb#reply114743994027967)**

## 安装配置

## 1. 创建虚拟环境

建议在虚拟环境中运行训练或部署程序，推荐使用 Conda 创建虚拟环境。

### 1.1 创建新环境

使用以下命令创建虚拟环境：

```bash
conda create -n robomimic python=3.8
```

### 1.2 激活虚拟环境

```bash
conda activate robomimic
```

---

## 2. 安装依赖

### 2.1 安装 PyTorch

PyTorch 是一个神经网络计算框架，用于模型训练和推理。使用以下命令安装：

```bash
conda install pytorch==2.3.1 torchvision==0.18.1 torchaudio==2.3.1 pytorch-cuda=12.1 -c pytorch -c nvidia
```

### 2.2 安装 RoboMimic_Deploy

#### 2.2.1 下载

通过 Git 克隆仓库：

```bash
git clone https://github.com/ccrpRepo/RoboMimic_Deploy.git
```

#### 2.2.2 安装组件

进入目录并安装：

```bash
cd RoboMimic_Deploy
pip install numpy==1.20.0
pip install onnx onnxruntime
```
#### 2.2.3 安装unitree_sdk2_python

```bash
git clone https://github.com/unitreerobotics/unitree_sdk2_python.git
cd unitree_sdk2_python
pip install -e .
```
---
## 运行代码

## C++ 真机部署文档

- `deploy_real_c` 的编译、启动、对比验证说明见：
  - [`deploy_real_c/README.md`](deploy_real_c/README.md)

## 1. 运行Mujoco仿真代码
```bash
/home/hiyio/anaconda3/envs/robomimic/bin/python -u deploy_mujoco/deploy_mujoco.py
```
---
## 2. Policy 说明
| 模式名称          | 描述                                                                 |
|------------------|----------------------------------------------------------------------|
| **PassiveMode**  | 阻尼保护模式                                                         |
| **FixedPose**    | 位控恢复至默认关节值                                                 |
| **LocoMode**     | Z1 行走入口仍需按 LeggedLab 配置重新适配；当前 dance 测试可绕开该模式 |
| **BeyondMimic**  | Z1 24DoF 舞蹈/全身跟踪策略，模型为 `policy/beyond_mimic/model/policy.onnx` |
| **Dance**        | 旧入口已不作为当前 Z1 舞蹈入口；当前从 FSM 兼容映射到 BeyondMimic |
| **KungFu**       | 武术动作                                                             |
| **KungFu2**      | 训练失败的武术动作                                                   |
| **Kick**         | 拿来凑数的动作                                                       |
| **SkillCast**    | 下肢+腰部稳定站立，上肢位控至特定关节角，一般在执行Mimic策略前执行   |
| **SkillCooldown**| 下肢+腰部持续平衡，上肢恢复至默认关节角，一般在执行Mimic策略后执行    |

---
## 3. 仿真操作说明

1. 连接Xbox手柄

2. 运行仿真程序：
```bash
/home/hiyio/anaconda3/envs/robomimic/bin/python -u deploy_mujoco/deploy_mujoco.py
```
3. 默认进入 `PassiveMode` 阻尼保护模式

4. 长按 `START` 进入 `FixedPose` 位控站姿

5. 长按 `R1 + A` 可进入 `LocoMode`，但当前 walk 尚未按 LeggedLab 配置完整重适配，dance 测试不需要使用该模式

6. 在 `FixedPose` 或 `LocoMode` 下，长按 `R1 + X` 或 `L1 + Y` 进入 `BeyondMimic`，执行当前 Z1 dance/全身跟踪策略

7. 在 `BeyondMimic` 下，按 `UP` 可暂停/恢复参考帧；长按 `R1 + A` 返回行走；按 `START` 回到位控；按 `L3` 进入阻尼保护

8. 按住 `SELECT` 会退出 MuJoCo 控制程序

### 当前 Z1 策略基线

- 行走：暂未切到最新 LeggedLab 策略，后续需要按 `/home/hiyio/LeggedLab` 的观测、归一化、关节顺序和动作缩放重新适配
- 舞蹈/全身跟踪：`policy/beyond_mimic/model/policy.onnx`
- BeyondMimic 来源：
  `/home/hiyio/whole_body_tracking/logs/rsl_rl/magicbot_z1_flat/2026-05-03_13-51-53_magicbot_spike_smooth_head_aligned_to_aiming1_height+Tracking-Flat-MagicBot-Z1-Wo-State-Estimation-v0_resume-model_124000/exported/policy.onnx`
- ONNX 形态：`obs [1,124]` + `time_step [1,1]`，输出 24DoF actions 和内嵌参考轨迹，motion length 为 `3309`
---
## 4. 真机操作说明
当前 Z1 真机入口优先使用 MagicBot 官方 SDK 后端：

```bash
cd /home/hiyio/MaigcLab/RoboMimic_Deploy_magicbot
deploy_real/run_magicbot_loco.sh --dry-run
deploy_real/run_magicbot_loco.sh --connect-check --local-ip 192.168.54.119
deploy_real/run_magicbot_loco.sh --read-state --local-ip 192.168.54.119 --duration 3
```

安全顺序：

1. 机器人吊起，确认急停/断电链路可用。
2. `--dry-run`：只加载 RoboMimic loco YAML/ONNX，不连接机器人。
3. `--connect-check`：只连接/断开 SDK，不切 LowLevel。
4. `--read-state`：按 `HighLevel -> GAIT_RECOVERY_STAND -> LowLevel` 顺序切入，只订阅低层状态，不发布 `JointCommand`。
5. 只有在上述检查通过后，才允许显式使用 `--run --stand-only --duration N` 或 `--run --duration N`。

旧的 `deploy_real/deploy_real.py` 仍是 Unitree DDS `rt/lowcmd`/`rt/lowstate` 架构，不是当前 MagicBot Z1 实机入口。

---
## 注意事项
### 1. 框架兼容性说明
原始框架面向 G1；当前工作区已改为 MagicBot Z1 仿真适配。真机侧必须使用 MagicBot Z1 SDK 和实际电机顺序重新核对部署链路。

- `deploy_real/run_magicbot_loco.sh` 是当前新增的 MagicBot SDK loco 后端。
- `deploy_real/deploy_real.py` 和 `deploy_real_c` 仍保留 Unitree DDS/shadow compare 逻辑，不能直接视为 MagicBot Z1 真机后端。
- BeyondMimic/舞蹈真机 SDK 后端仍需继续从仿真策略迁移，当前不要直接上实机。
- 运行时分层说明见：`Docs/runtime_split.md`。

### 2. Mimic策略可靠性警告
Mimic策略不保证100%成功率，特别是在湿滑/沙地等复杂地面上。若出现机器人失控情况：
- 按下`F1`键激活**阻尼保护模式**(PassiveMode)
- 按下`Select`键立即终止控制程序

### 3. Z1 BeyondMimic Dance - 当前稳定基线
当前 Z1 舞蹈入口是 `BeyondMimic`，不是旧 `policy/dance/Dance.py`：

⚠️ **重要注意事项**：
- **模型路径**：`policy/beyond_mimic/model/policy.onnx`
- **控制入口**：`R1 + X` 或 `L1 + Y`
- **起止稳定需求**：舞蹈开始/结束时可能需要短暂人工稳定
- **舞蹈后过渡**：虽然可以切换至**行走模式/位控模式/阻尼模式**，但建议：
  - 先切换至**位控模式**或**阻尼模式**
  - 过渡期间需提供人工稳定

### 4. 其他动作建议
其他所有动作目前均**不建议**在真机上部署。

### 5. 强烈建议
**务必**先在仿真环境中熟练操作，再尝试真机部署。

### 6. 调试安全开关（新增）
- `deploy_real/config/safety.yaml` 与 `deploy_mujoco/config/safety.yaml` 可配置调试安全参数。
- 默认开启动作/增益限幅、长按触发（防误触）与故障降级到阻尼模式。
- 需要干跑时，将 `dry_run: true`（只计算不下发指令）。
