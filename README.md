<div align="center">
  <h1 align="center">RoboMimic Deploy</h1>
  <p align="center">
    <span> 🌎English </span> | <a href="README_zh.md"> 🇨🇳中文 </a>
  </p>
</div>

<p align="center">
  <strong>​RoboMimic Deploy​​ is a multi-policy robot deployment framework based on a state-switching mechanism. This workspace is currently adapted for MagicBot Z1 24-DoF MuJoCo deployment.</strong> 
</p>

## Preface

- **This `/home/hiyio/MaigcLab/RoboMimic_Deploy_magicbot` workspace uses the MagicBot Z1 24-DoF configuration, not the original G1 29-DoF setup.**

- **The MuJoCo XML is `/home/hiyio/HoloMotion/thirdparties/GMR/assets/magicbot_z1/mjcf/MAGICBOTZ1.xml`.**
  
- **The current validated target is simulation. For real-robot deployment, re-check the Z1 SDK, motor order, limits, initial pose, and emergency stop path first.**

- **[video instruction](https://www.bilibili.com/video/BV1VTKHzSE6C/?vd_source=713b35f59bdf42930757aea07a44e7cb#reply114743994027967)**

## Installation and Configuration

## 1. Create a Virtual Environment

It is recommended to run training or deployment programs in a virtual environment. We suggest using Conda to create one.

### 1.1 Create a New Environment

Use the following command to create a virtual environment:
```bash
conda create -n robomimic python=3.8
```

### 1.2 Activate the Virtual Environment

```bash
conda activate robomimic
```

---

## 2. Install Dependencies

### 2.1 Install PyTorch
PyTorch is a neural network computation framework used for model training and inference. Install it with the following command:
```bash
conda install pytorch==2.3.1 torchvision==0.18.1 torchaudio==2.3.1 pytorch-cuda=12.1 -c pytorch -c nvidia
```

### 2.2 Install RoboMimic_Deploy

#### 2.2.1 Download
Clone the repository via git:

```bash
git clone https://github.com/ccrpRepo/RoboMimic_Deploy.git
```

#### 2.2.2 Install Components

Navigate to the directory and install:
```bash
cd RoboMimic_Deploy
pip install numpy==1.20.0
pip install onnx onnxruntime
```

#### 2.2.3 Install unitree_sdk2_python

```bash
git clone https://github.com/unitreerobotics/unitree_sdk2_python.git
cd unitree_sdk2_python
pip install -e .
```
---
## Running the Code

## C++ Deploy Real Documentation

- For `deploy_real_c` build/run/verify instructions, see:
  - [`deploy_real_c/README.md`](deploy_real_c/README.md)

## 1. Run Mujoco Simulation
```bash
/home/hiyio/anaconda3/envs/robomimic/bin/python -u deploy_mujoco/deploy_mujoco.py
```

## 2. Policy Descriptions
| Mode Name        | Description                                                                 |
|------------------|-----------------------------------------------------------------------------|
| **PassiveMode**  | Damping protection mode                                                     |
| **FixedPose**    | Position control reset to default joint values                              |
| **LocoMode**     | Z1 locomotion entry still needs re-adaptation from the LeggedLab config; current dance testing can bypass it |
| **BeyondMimic**  | Z1 24-DoF dance / whole-body tracking policy: `policy/beyond_mimic/model/policy.onnx` |
| **Dance**        | Legacy entry; current Z1 dance routing is mapped to BeyondMimic             |
| **KungFu**       | Martial arts movement                                                       |
| **KungFu2**      | Failed martial arts training                                     |
| **Kick**         | Bad mimic policy                                     |
| **SkillCast**    | Lower body + waist stabilization with upper limbs positioned to specific joint angles (typically executed before Mimic strategy) |
| **SkillCooldown**| Lower body + waist continuous balancing with upper limbs reset to default angles (typically executed after Mimic strategy) |


---
## 3. Operation Instructions in Simulation
1. Connect an Xbox controller.
2. Run the simulation program:
```bash
/home/hiyio/anaconda3/envs/robomimic/bin/python -u deploy_mujoco/deploy_mujoco.py
```
3. The program starts in `PassiveMode`.
4. Hold `START` to enter `FixedPose`.
5. Hold `R1 + A` to enter `LocoMode`; walk still needs full re-adaptation from the LeggedLab config, so dance testing does not require this mode.
6. In `FixedPose` or `LocoMode`, hold `R1 + X` or `L1 + Y` to enter `BeyondMimic` and run the current Z1 dance / whole-body tracking policy.
7. In `BeyondMimic`, press `UP` to pause/resume the reference frame, hold `R1 + A` to return to locomotion, press `START` for position reset, or press `L3` for damping protection.
8. Hold `SELECT` to exit the MuJoCo control program.

### Current Z1 Policy Baseline

- Locomotion: not switched to the latest LeggedLab strategy yet; it needs re-adaptation of observations, normalization, joint order, and action scaling from `/home/hiyio/LeggedLab`
- Dance / whole-body tracking: `policy/beyond_mimic/model/policy.onnx`
- BeyondMimic source:
  `/home/hiyio/whole_body_tracking/logs/rsl_rl/magicbot_z1_flat/2026-05-03_13-51-53_magicbot_spike_smooth_head_aligned_to_aiming1_height+Tracking-Flat-MagicBot-Z1-Wo-State-Estimation-v0_resume-model_124000/exported/policy.onnx`
- ONNX shape: `obs [1,124]` + `time_step [1,1]`, 24-DoF action output with embedded reference trajectory, motion length `5515`.
---
## 4. Real Robot Operation Instructions

1. Power on the robot and suspend it (e.g., with a harness). and then hold L2+R2
2. Run the deploy_real program:
```bash
python deploy_real/deploy_real.py
```
3. Press the ​​Start​​ button to enter position control mode.
4. Subsequent operations are the same as in simulation.

---
## Debug Safety (New)
- `deploy_real/config/safety.yaml` and `deploy_mujoco/config/safety.yaml` control safety limits.
- Default enables action/gain clamping, hold-to-confirm for mode switches, and damping fallback on faults.
- Set `dry_run: true` for compute-only (no command output).

---
## Important Notes
### 1. Framework Compatibility Notice
The original framework targeted G1; this workspace is now adapted for MagicBot Z1 simulation. Real-robot deployment still needs a full Z1 SDK and motor-order audit. For onboard Orin deployment, we recommend the following alternative solution:

- Replace with [unitree_sdk2](https://github.com/unitreerobotics/unitree_sdk2) (official C++ SDK)
- Implement a dual-node ROS architecture:
  - **C++ Node**: Handles data transmission between robot and controller
  - **Python Node**: Dedicated to policy inference

### 2. Mimic Policy Reliability Warning
The Mimic policy does not guarantee 100% success rate, particularly on slippery/sandy surfaces. In case of robot instability:
- Press `F1` to activate **PassiveMode** (damping protection)
- Press `Select` to immediately terminate the control program

### 3. Z1 BeyondMimic Dance - Current Baseline
The current Z1 dance entry is `BeyondMimic`, not the legacy `policy/dance/Dance.py` path:

⚠️ **Important Precautions**:
- **Model Path**: `policy/beyond_mimic/model/policy.onnx`
- **Control Entry**: `R1 + X` or `L1 + Y`
- **Initial/Final Stabilization**: Brief manual stabilization may be required when starting/ending the dance
- **Post-Dance Transition**: While switching to **Locomotion/PositionControl/PassiveMode** is possible, we recommend:
  - First transition to **PositionControl** or **PassiveMode**
  - Provide manual stabilization during transition

### 4. Other Movement Advisories
All other movements are currently **not recommended** for physical robot deployment.

### 5. Strong Recommendation
**Always** master operations in simulation before attempting physical robot deployment.
