# RoboMimic Deploy for MagicBot Z1

<p align="center">
  <strong>English</strong> | <a href="README_zh.md">中文</a>
</p>

This repository is the MagicBot Z1 deployment workspace derived from RoboMimic Deploy. The current code path is no longer the original G1/29-DoF setup; it is centered on MagicBot Z1 24-DoF simulation, policy playback, and verification.

The validated main path is Python MuJoCo simulation. Real-robot work is currently limited to staged MagicBot SDK loco checks. Do not treat every policy in this tree as ready for direct physical deployment.

## Current Status

| Module | Status | Entry/config |
| --- | --- | --- |
| Python MuJoCo simulation | Main validated path | `python_reference/simulation/mujoco_reference.py` |
| Z1 MuJoCo model | 24 DoF | `assets/robots/magicbot_z1/scene.xml` |
| BeyondMimic | Current Z1 dance / whole-body tracking entry | `policies/beyond_mimic/config/BeyondMimic.yaml` |
| LocoMode | Z1 24-DoF ONNX locomotion entry; still validate in sim before robot use | `policies/loco_mode/config/LocoMode_lowKp.yaml` |
| Python/C++ compare | Available | `scripts/compare_python_cpp.sh --verify --net lo` |
| MagicBot SDK real loco | Conservative staged-check path | `python_reference/legacy_robot/run_magicbot_loco_reference.sh` |
| Unitree DDS legacy path | Kept for history/compare; not the current Z1 SDK robot path | `python_reference/legacy_robot/dds_robot_legacy.py` |

## Layout

```text
RoboMimic_Deploy_magicbot/
├── assets/robots/magicbot_z1/      # Z1 MuJoCo XML and meshes
├── configs/
│   ├── simulation/                 # MuJoCo, initial pose, safety config
│   └── robot/                      # legacy DDS / robot-side config
├── policies/
│   ├── passive/                    # damping protection
│   ├── fixedpose/                  # stand-pose interpolation
│   ├── loco_mode/                  # Z1 loco ONNX
│   ├── beyond_mimic/               # Z1 BeyondMimic ONNX
│   └── skill_cooldown/             # post-mimic stand recovery
├── python_reference/
│   ├── simulation/                 # MuJoCo sim and DDS compare bridge
│   ├── fsm/                        # state machine
│   ├── runtime/                    # control, DDS, rendering, compare modules
│   └── legacy_robot/               # MagicBot SDK loco wrapper and old DDS path
├── controller_cpp/                 # C++ DDS/ONNX controller
├── scripts/compare_python_cpp.sh   # Python vs C++ shadow compare
└── docs/                           # runtime split, benchmarks, debug notes
```

## Environment

Use Python 3.10. The codebase now uses Python 3.10 syntax, so the old Python 3.8 setup is no longer the baseline.

```bash
conda create -n robomimic python=3.10
conda activate robomimic
pip install -r requirements.txt
```

Optional dependencies:

```bash
# Required for Unitree DDS shadow compare or legacy DDS entry.
git clone https://github.com/unitreerobotics/unitree_sdk2_python.git
cd unitree_sdk2_python
pip install -e .
```

The MagicBot real loco wrapper also needs `magicbot-z1_sdk-main`. Set `MAGICBOT_SDK_ROOT` if it is not in one of the default searched locations.

## Run Simulation

```bash
cd /home/hiyio/MaigcLab/RoboMimic_Deploy_magicbot
conda activate robomimic
python -u python_reference/simulation/mujoco_reference.py
```

Default config files:

- `configs/simulation/mujoco.yaml`
- `configs/simulation/magicbot_z1_stand.yaml`
- `configs/simulation/safety.yaml`

Key runtime settings:

- MuJoCo XML: `assets/robots/magicbot_z1/scene.xml`
- simulation step: `simulation_dt: 0.002`
- control decimation: `control_decimation: 10`
- control period: `0.02s` / 50 Hz
- initial command: `PASSIVE`
- reference ghost display: enabled by default with `ghost.mode: mesh`

If no controller is connected, the simulator falls back to neutral `NullJoyStick`. You can observe the default state, but you cannot switch modes from the controller.

## Controller Map

Recommended Z1 path:

| Input | Effect |
| --- | --- |
| Program start | Enter `PassiveMode` damping protection |
| Hold `START` | Enter `FixedPose`, interpolating to stand pose over 2 seconds |
| Hold `R1 + A` | Enter `LocoMode` |
| Hold `R1 + X` in `FixedPose` or `LocoMode` | Enter `BeyondMimic` |
| Hold `L1 + Y` in `FixedPose` or `LocoMode` | Enter `BeyondMimic` |
| Release `UP` in mimic | Pause/resume the reference frame |
| Hold `R1 + A` in mimic | Return to `LocoMode` through `SkillCooldown` |
| Hold `START` in mimic | Return to `FixedPose` |
| Release `L3` | Return to `PassiveMode` |
| Press `SELECT` | Exit the simulation |

Legacy skill commands such as `SKILL_2`, `SKILL_6`, and `SKILL_7` still exist in the code, but they are not the recommended Python Z1 main path. Old Dance/KungFu/Kick/TrackMimic states are mostly kept as compatibility aliases.

## Policy Baseline

### BeyondMimic

Current Z1 dance / whole-body tracking entry:

- model: `policies/beyond_mimic/model/policy.onnx`
- config: `policies/beyond_mimic/config/BeyondMimic.yaml`
- inputs: `obs [1,124]` and `time_step [1,1]`
- outputs: 24-DoF action plus embedded reference trajectory data
- `motion_length: 3309`
- `switch_to_loco_delay_s: -1.0`, so the policy holds in mimic instead of auto-returning to loco
- `command_joint_indices` excludes ankle commands and must match the exported training observation
- `mj2lab` maps LeggedLab/IsaacLab joint index to MuJoCo actuator index

To temporarily use another BeyondMimic YAML:

```bash
BEYOND_MIMIC_CONFIG_PATH=/abs/path/to/BeyondMimic.yaml \
python -u python_reference/simulation/mujoco_reference.py
```

### LocoMode

Current loco config:

- model: `policies/loco_mode/model/z1_flat_reset_reasons_model_25800.onnx`
- config: `policies/loco_mode/config/LocoMode_lowKp.yaml`
- input dimension: `82`
- action dimension: `24`
- command dimension: `4`
- `root_height_command: 0.69`

LocoMode is configured for Z1 24 DoF, but real-robot use still requires a fresh audit of observations, normalization, joint order, action scaling, limits, and initial pose.

### FixedPose / Passive / SkillCooldown

- `PassiveMode`: zero `kp`, damping only.
- `FixedPose`: interpolate from current joints to the default stand pose, currently 2 seconds.
- `SkillCooldown`: blend-only recovery from mimic pose back to the Z1 fixed-pose target, then return to loco.

## Safety Config

The safety filter supports action clamps, action-delta clamps, gain clamps, gain-delta clamps, damping fallback, and dry-run.

The current YAML has `enable: false`, so it does not clamp policy outputs by default. `dry_run` is still handled directly by the simulation loop.

```yaml
# configs/simulation/safety.yaml
enable: false
dry_run: false
command_hold_frames: 2
max_action_abs: 3.5
max_action_delta: 0.3
damping_kd: 8.0
```

Debug tips:

- Use `dry_run: true` to compute policy output without applying actuator control.
- Use `enable: true` and `log_clamps: true` to inspect safety clamp behavior.
- Re-tune and re-verify `configs/robot/safety.yaml` before any physical robot command publishing.

## Python/C++ Shadow Compare

Recommended one-command verification:

```bash
cd /home/hiyio/MaigcLab/RoboMimic_Deploy_magicbot
bash scripts/compare_python_cpp.sh --verify --net lo
```

This starts:

- C++ `robot_controller_onnx --shadow`
- Python `mujoco_dds_compare.py --headless --no-joystick --shadow-sync`
- DDS `rt/lowstate` / `rt/lowcmd` bridge
- diff CSV and summary JSON output

If the C++ binary does not exist, build it first:

```bash
cmake -S controller_cpp -B controller_cpp/build_z1
cmake --build controller_cpp/build_z1 -j
```

See [`controller_cpp/README.md`](controller_cpp/README.md) for more C++ details.

## C++ Build Notes

`controller_cpp` depends on:

- `unitree_sdk2`
- `yaml-cpp`
- `zlib`
- ONNX Runtime C/C++ package
- optional MuJoCo C API for `mujoco_dds_simulator`

`controller_cpp/CMakeLists.txt` contains local default paths for ONNX Runtime and MuJoCo. Override them if needed:

```bash
cmake -S controller_cpp -B controller_cpp/build_z1 \
  -DONNXRUNTIME_DIR=/path/to/onnxruntime-linux-x64 \
  -DMUJOCO_ROOT=/path/to/mujoco
cmake --build controller_cpp/build_z1 -j
```

## MagicBot Z1 Real Loco Checks

For physical robot loco work, prefer the MagicBot SDK backend:

```bash
cd /home/hiyio/MaigcLab/RoboMimic_Deploy_magicbot
python_reference/legacy_robot/run_magicbot_loco_reference.sh --dry-run
python_reference/legacy_robot/run_magicbot_loco_reference.sh --connect-check --local-ip 192.168.54.119
python_reference/legacy_robot/run_magicbot_loco_reference.sh --read-state --local-ip 192.168.54.119 --duration 3
```

Safe sequence:

1. Suspend the robot and verify emergency stop, power-off, network isolation, and manual takeover.
2. `--dry-run`: load YAML/ONNX and run one inference, with no robot connection.
3. `--connect-check`: connect/disconnect through the SDK only, without LowLevel switch.
4. `--read-state`: switch `HighLevel -> GAIT_RECOVERY_STAND -> LowLevel`, subscribe low-level states, and publish no `JointCommand`.
5. Only after those pass, explicitly run `--run --stand-only --duration N`.
6. After stand-only passes, consider very short `--run --duration N --vx 0 --vy 0 --wz 0`.

Example:

```bash
python_reference/legacy_robot/run_magicbot_loco_reference.sh \
  --run --stand-only --local-ip 192.168.54.119 --duration 2
```

Do not run BeyondMimic/dance directly on the physical robot. The BeyondMimic MagicBot SDK backend still needs migration and audit from the simulation policy.

## Legacy DDS And C++ Robot Paths

`python_reference/legacy_robot/dds_robot_legacy.py` and `controller_cpp` keep a Unitree DDS-style `rt/lowcmd` / `rt/lowstate` path. These files are useful for compatibility, C++ shadow compare, and DDS controller development, but they are not the current MagicBot Z1 SDK robot entry.

Before any DDS robot run, re-check:

- network interface and DDS domain;
- actual low-level robot topics;
- physical order of all 24 motors;
- LowLevel switching path;
- emergency stop and damping command behavior.

## Troubleshooting

### `No joystick connected`

`mujoco_reference.py` falls back to `NullJoyStick`. Connect a controller to switch modes, or use `mujoco_dds_compare.py --no-joystick` for automated compare runs.

### C++ compare waits for DDS forever

Check:

- `--net` matches on both sides; local shadow compare should use `lo`;
- `controller_cpp/build_z1/robot_controller_onnx` exists;
- `unitree_sdk2` and CycloneDDS runtime libraries are resolvable;
- the Python side is running and publishing `rt/lowstate`.

### ONNX loading fails

Check the YAML `onnx_path`. Relative paths are resolved under the policy's `model/` directory, for example BeyondMimic resolves to `policies/beyond_mimic/model/policy.onnx`.

### MuJoCo NaN/Inf or instability

Return to `PassiveMode` or stop the program. Then check:

- initial pose comes from `configs/simulation/magicbot_z1_stand.yaml`;
- policy and `mj2lab` mapping match;
- `safety.yaml` should be set to `enable: true` or `dry_run: true` during diagnosis;
- no unsupported legacy skill entry was triggered.

## More Docs

- [`docs/runtime_split.md`](docs/runtime_split.md): runtime layering
- [`docs/inference_benchmark_2026-06-07.md`](docs/inference_benchmark_2026-06-07.md): ONNX inference benchmark
- [`controller_cpp/README.md`](controller_cpp/README.md): C++ controller and shadow compare details

## Rule Of Thumb

Sim first, then shadow compare, then dry-run, then connect-check, then read-state. Publish real robot commands only when every previous step is explainable, repeatable, and recoverable.
