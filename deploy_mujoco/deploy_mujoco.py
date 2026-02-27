import sys
from pathlib import Path
sys.path.append(str(Path(__file__).parent.parent.absolute()))

from common.path_config import PROJECT_ROOT

import time
import mujoco.viewer
import mujoco
import numpy as np
import yaml
import os
from common.ctrlcomp import *
from FSM.FSM import *
from common.utils import get_gravity_orientation
from common.joystick import JoyStick, JoystickButton
from common.safety import load_safety_config, SafetyFilter, HoldToConfirm

try:
    from deploy_mujoco.head_camera_stream import HeadCameraStreamServer
except Exception:
    HeadCameraStreamServer = None

try:
    from deploy_mujoco.head_camera_ros2 import HeadCameraRos2Publisher
except Exception:
    HeadCameraRos2Publisher = None


def pd_control(target_q, q, kp, target_dq, dq, kd):
    """Calculates torques from position commands"""
    return (target_q - q) * kp + (target_dq - dq) * kd


def sanitize_ctrl(ctrl, model, fallback=None):
    """Ensure finite actuator controls and clamp to actuator ctrlrange if available."""
    out = np.asarray(ctrl, dtype=np.float32).reshape(-1)
    if fallback is None:
        fallback = np.zeros_like(out)
    fallback = np.asarray(fallback, dtype=np.float32).reshape(-1)
    if fallback.size != out.size:
        fallback = np.zeros_like(out)

    finite_mask = np.isfinite(out)
    if not np.all(finite_mask):
        out = np.where(finite_mask, out, fallback)
    out = np.nan_to_num(out, nan=0.0, posinf=0.0, neginf=0.0)

    ctrl_range = getattr(model, "actuator_ctrlrange", None)
    if ctrl_range is not None and ctrl_range.shape[0] == out.size:
        lo = ctrl_range[:, 0]
        hi = ctrl_range[:, 1]
        # Some models may leave ctrlrange unset (lo==hi==0), fallback in that case.
        if np.any(hi > lo):
            return np.clip(out, lo, hi).astype(np.float32)

    return np.clip(out, -300.0, 300.0).astype(np.float32)


if __name__ == "__main__":
    current_dir = os.path.dirname(os.path.abspath(__file__))
    mujoco_yaml_path = os.path.join(current_dir, "config", "mujoco.yaml")
    with open(mujoco_yaml_path, "r") as f:
        config = yaml.load(f, Loader=yaml.FullLoader)
        xml_path = os.path.join(PROJECT_ROOT, config["xml_path"])
        simulation_dt = config["simulation_dt"]
        control_decimation = config["control_decimation"]

    safety_yaml_path = os.path.join(current_dir, "config", "safety.yaml")
    safety_cfg = load_safety_config(safety_yaml_path)

    head_cam_cfg = config.get("head_camera_stream", {})
    head_cam_enable = bool(head_cam_cfg.get("enable", False))
    head_cam_fps = float(head_cam_cfg.get("fps", 20))
    head_cam_every_n_steps = max(1, int(round(1.0 / max(simulation_dt * head_cam_fps, 1e-6))))

    ros2_cam_cfg = config.get("head_camera_ros2", {})
    ros2_cam_enable = bool(ros2_cam_cfg.get("enable", False))
    ros2_cam_fps = float(ros2_cam_cfg.get("fps", 20))
    ros2_cam_every_n_steps = max(1, int(round(1.0 / max(simulation_dt * ros2_cam_fps, 1e-6))))

    m = mujoco.MjModel.from_xml_path(xml_path)
    d = mujoco.MjData(m)
    m.opt.timestep = simulation_dt
    mj_per_step_duration = simulation_dt * control_decimation
    num_joints = m.nu
    policy_output_action = np.zeros(num_joints, dtype=np.float32)
    kps = np.zeros(num_joints, dtype=np.float32)
    kds = np.zeros(num_joints, dtype=np.float32)
    sim_counter = 0

    state_cmd = StateAndCmd(num_joints)
    policy_output = PolicyOutput(num_joints)
    FSM_controller = FSM(state_cmd, policy_output)
    safety = SafetyFilter(num_joints, safety_cfg)
    command_gate = HoldToConfirm(safety_cfg.command_hold_frames)

    head_cam_stream = None
    if head_cam_enable:
        if HeadCameraStreamServer is None:
            print("[CameraStream] disabled: stream module import failed.")
        else:
            try:
                head_cam_stream = HeadCameraStreamServer(
                    m,
                    d,
                    camera_name=head_cam_cfg.get("camera_name", "head_rgba_camera"),
                    host=head_cam_cfg.get("host", "0.0.0.0"),
                    port=int(head_cam_cfg.get("port", 18080)),
                    width=int(head_cam_cfg.get("width", 640)),
                    height=int(head_cam_cfg.get("height", 480)),
                    jpeg_quality=int(head_cam_cfg.get("jpeg_quality", 80)),
                )
                head_cam_stream.start()
            except Exception as e:
                head_cam_stream = None
                print(f"[CameraStream] failed to start: {e}")

    head_cam_ros2 = None
    if ros2_cam_enable:
        if HeadCameraRos2Publisher is None:
            print("[CameraROS2] disabled: ROS2 publisher module import failed.")
        else:
            try:
                head_cam_ros2 = HeadCameraRos2Publisher(
                    m,
                    d,
                    camera_name=ros2_cam_cfg.get("camera_name", "head_rgba_camera"),
                    width=int(ros2_cam_cfg.get("width", 640)),
                    height=int(ros2_cam_cfg.get("height", 480)),
                    topic_rgb=ros2_cam_cfg.get("topic_rgb", "/g1/head_camera/rgb"),
                    topic_rgba=ros2_cam_cfg.get("topic_rgba", "/g1/head_camera/rgba"),
                    frame_id=ros2_cam_cfg.get("frame_id", "head_rgba_camera"),
                    node_name=ros2_cam_cfg.get("node_name", "g1_head_camera_publisher"),
                    qos_depth=int(ros2_cam_cfg.get("qos_depth", 5)),
                )
            except Exception as e:
                head_cam_ros2 = None
                print(f"[CameraROS2] failed to start: {e}")

    joystick = JoyStick()
    Running = True
    try:
        with mujoco.viewer.launch_passive(m, d) as viewer:
            sim_start_time = time.time()
            while viewer.is_running() and Running:
                try:
                    if(joystick.is_button_pressed(JoystickButton.SELECT)):
                        Running = False

                    joystick.update()
                    if joystick.is_button_released(JoystickButton.L3):
                        state_cmd.skill_cmd = FSMCommand.PASSIVE
                    if joystick.is_button_released(JoystickButton.UP):
                        state_cmd.skill_cmd = FSMCommand.PAUSE
                    if command_gate.trigger("POS_RESET", joystick.is_button_pressed(JoystickButton.START)):
                        state_cmd.skill_cmd = FSMCommand.POS_RESET
                    if command_gate.trigger(
                        "LOCO",
                        joystick.is_button_pressed(JoystickButton.A) and joystick.is_button_pressed(JoystickButton.R1),
                    ):
                        state_cmd.skill_cmd = FSMCommand.LOCO
                    if command_gate.trigger(
                        "SKILL_1",
                        joystick.is_button_pressed(JoystickButton.X) and joystick.is_button_pressed(JoystickButton.R1),
                    ):
                        state_cmd.skill_cmd = FSMCommand.SKILL_1
                    if command_gate.trigger(
                        "SKILL_2",
                        joystick.is_button_pressed(JoystickButton.Y) and joystick.is_button_pressed(JoystickButton.R1),
                    ):
                        state_cmd.skill_cmd = FSMCommand.SKILL_2
                    if joystick.is_button_released(JoystickButton.B) and joystick.is_button_pressed(JoystickButton.R1):
                        state_cmd.skill_cmd = FSMCommand.SKILL_3
                    if command_gate.trigger(
                        "SKILL_4",
                        joystick.is_button_pressed(JoystickButton.Y) and joystick.is_button_pressed(JoystickButton.L1),
                    ):
                        state_cmd.skill_cmd = FSMCommand.SKILL_4
                    if joystick.is_button_released(JoystickButton.B) and joystick.is_button_pressed(JoystickButton.L1):
                        state_cmd.skill_cmd = FSMCommand.SKILL_5
                    if command_gate.trigger(
                        "SKILL_6",
                        joystick.is_button_pressed(JoystickButton.X) and joystick.is_button_pressed(JoystickButton.L1),
                    ):
                        state_cmd.skill_cmd = FSMCommand.SKILL_6
                    if command_gate.trigger(
                        "SKILL_7",
                        joystick.is_button_pressed(JoystickButton.A) and joystick.is_button_pressed(JoystickButton.L1),
                    ):
                        state_cmd.skill_cmd = FSMCommand.SKILL_7

                    state_cmd.vel_cmd[0] = -joystick.get_axis_value(1)
                    state_cmd.vel_cmd[1] = -joystick.get_axis_value(0)
                    state_cmd.vel_cmd[2] = -joystick.get_axis_value(3)

                    step_start = time.time()

                    raw_tau = pd_control(policy_output_action, d.qpos[7:], kps, np.zeros_like(kps), d.qvel[6:], kds)
                    fallback_tau = -safety_cfg.damping_kd * np.asarray(d.qvel[6:], dtype=np.float32)
                    tau = sanitize_ctrl(raw_tau, m, fallback=fallback_tau)
                    if (not np.isfinite(raw_tau).all()) or np.max(np.abs(raw_tau)) > 1e4:
                        print("[Safety] abnormal torque detected; sanitized before applying ctrl.")
                    if safety_cfg.dry_run:
                        d.ctrl[:] = 0
                    else:
                        d.ctrl[:] = tau
                    # Apply mouse drag perturbations from the viewer in passive mode.
                    if hasattr(viewer, "pert"):
                        d.xfrc_applied[:] = 0
                        mujoco.mjv_applyPerturbForce(m, d, viewer.pert)
                    mujoco.mj_step(m, d)
                    sim_counter += 1

                    if head_cam_stream is not None and (sim_counter % head_cam_every_n_steps == 0):
                        head_cam_stream.update()
                    if head_cam_ros2 is not None and (sim_counter % ros2_cam_every_n_steps == 0):
                        head_cam_ros2.publish()

                    if sim_counter % control_decimation == 0:

                        qj = d.qpos[7:]
                        dqj = d.qvel[6:]
                        quat = d.qpos[3:7]

                        omega = d.qvel[3:6]
                        gravity_orientation = get_gravity_orientation(quat)

                        state_cmd.q = qj.copy()
                        state_cmd.dq = dqj.copy()
                        state_cmd.gravity_ori = gravity_orientation.copy()
                        state_cmd.base_quat = quat.copy()
                        state_cmd.ang_vel = omega.copy()

                        FSM_controller.run()
                        policy_output_action = policy_output.actions.copy()
                        kps = policy_output.kps.copy()
                        kds = policy_output.kds.copy()

                        policy_output_action, kps, kds, force_damping = safety.filter_actions(
                            policy_output_action, kps, kds
                        )
                        if force_damping:
                            policy_output_action = d.qpos[7:].copy()
                            kps = np.zeros_like(kps)
                            kds = np.ones_like(kds) * safety_cfg.damping_kd
                        if safety_cfg.dry_run:
                            kps = np.zeros_like(kps)
                            kds = np.zeros_like(kds)
                except ValueError as e:
                    print(str(e))

                viewer.sync()
                time_until_next_step = m.opt.timestep - (time.time() - step_start)
                if time_until_next_step > 0:
                    time.sleep(time_until_next_step)
    finally:
        if head_cam_stream is not None:
            head_cam_stream.close()
        if head_cam_ros2 is not None:
            head_cam_ros2.close()
