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
import queue
from common.ctrlcomp import *
from FSM.FSM import *
from common.utils import get_gravity_orientation
from common.joystick import JoyStick, JoystickButton
from common.safety import load_safety_config, SafetyFilter, HoldToConfirm
from policy.beyond_mimic.BeyondMimic import BeyondMimic

try:
    import glfw
except Exception:
    glfw = None


def _read_config_value(path, key):
    """Reads a simple top-level YAML key without altering formatting."""
    try:
        with open(path, "r") as f:
            for line in f:
                if line.lstrip().startswith(f"{key}:"):
                    return line.split(":", 1)[1].strip().strip('"').strip("'")
    except FileNotFoundError:
        pass
    return None


def _update_config_value(path, key, value):
    """Updates a simple top-level YAML key while preserving formatting."""
    with open(path, "r") as f:
        lines = f.read().splitlines()
    for idx, line in enumerate(lines):
        if line.lstrip().startswith(f"{key}:"):
            indent = line[: len(line) - len(line.lstrip())]
            lines[idx] = f'{indent}{key}: "{value}"'
            with open(path, "w") as f:
                f.write("\n".join(lines) + "\n")
            return True
    return False


class ModelMenu:
    """In-window model selector (keyboard-driven) with overlay text."""

    def __init__(self, model_dir, config_path):
        self.model_dir = model_dir
        self.config_path = config_path
        self.model_files = []
        self.selected_index = 0
        self.open = False
        self.apply_requested = False
        self.refresh()

    def refresh(self):
        self.model_files = sorted(
            [p for p in os.listdir(self.model_dir) if p.endswith(".onnx")],
            key=lambda name: os.path.getmtime(os.path.join(self.model_dir, name)),
            reverse=True,
        )
        current = _read_config_value(self.config_path, "onnx_path")
        if current in self.model_files:
            self.selected_index = self.model_files.index(current)
        elif self.model_files:
            self.selected_index = 0

    def current_name(self):
        if not self.model_files:
            return None
        return self.model_files[self.selected_index]

    def handle_key(self, key):
        if not self.model_files:
            return
        if key == glfw.KEY_M:
            self.open = not self.open
            return
        if key == glfw.KEY_R:
            self.refresh()
            return
        if not self.open:
            return
        if key == glfw.KEY_UP:
            self.selected_index = (self.selected_index - 1) % len(self.model_files)
        elif key == glfw.KEY_DOWN:
            self.selected_index = (self.selected_index + 1) % len(self.model_files)
        elif key in (glfw.KEY_ENTER, glfw.KEY_KP_ENTER):
            self.apply_requested = True
            self.open = False
        elif key == glfw.KEY_ESCAPE:
            self.open = False

    def build_overlay(self):
        current = self.current_name() or "N/A"
        lines = []
        lines.append("Model selector: M=toggle, R=refresh, UP/DOWN=select, Enter=apply")
        lines.append(f"Current: {current}")
        if self.open:
            lines.append("Available models:")
            preview = self.model_files[:10]
            for idx, name in enumerate(preview):
                marker = ">" if idx == self.selected_index else " "
                lines.append(f"{marker} {name}")
            if len(self.model_files) > len(preview):
                lines.append("... (showing latest 10)")
        return "\n".join(lines)



def pd_control(target_q, q, kp, target_dq, dq, kd):
    """Calculates torques from position commands"""
    return (target_q - q) * kp + (target_dq - dq) * kd

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
    
    joystick = JoyStick()
    Running = True
    menu = None
    menu_events = None
    if glfw is not None:
        menu = ModelMenu(
            model_dir=os.path.join(PROJECT_ROOT, "policy", "beyond_mimic", "model"),
            config_path=os.path.join(PROJECT_ROOT, "policy", "beyond_mimic", "config", "BeyondMimic.yaml"),
        )
        menu_events = queue.Queue()

        def _on_key(key):
            # Collect key events on the UI thread to process in the sim loop.
            try:
                menu_events.put_nowait(key)
            except queue.Full:
                pass

        key_callback = _on_key
    else:
        key_callback = None
    with mujoco.viewer.launch_passive(m, d, key_callback=key_callback) as viewer:
        sim_start_time = time.time()
        while viewer.is_running() and Running:
            try:
                if menu is not None:
                    # Consume queued UI events and update menu state.
                    while not menu_events.empty():
                        menu.handle_key(menu_events.get_nowait())
                    if menu.apply_requested:
                        selected = menu.current_name()
                        if selected:
                            _update_config_value(
                                os.path.join(PROJECT_ROOT, "policy", "beyond_mimic", "config", "BeyondMimic.yaml"),
                                "onnx_path",
                                selected,
                            )
                            # Recreate the BeyondMimic policy so the new ONNX is loaded.
                            new_policy = BeyondMimic(state_cmd, policy_output)
                            FSM_controller.beyond_mimic_policy = new_policy
                            if getattr(FSM_controller.cur_policy, "name_str", "") == "beyond_mimic":
                                FSM_controller.cur_policy.exit()
                                FSM_controller.cur_policy = new_policy
                                FSM_controller.cur_policy.enter()
                        menu.apply_requested = False
                    # Overlay menu text in the viewer.
                    viewer.set_texts((None, mujoco.mjtGridPos.mjGRID_TOPLEFT, menu.build_overlay(), ""))
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
                
                tau = pd_control(policy_output_action, d.qpos[7:], kps, np.zeros_like(kps), d.qvel[6:], kds)
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
        
