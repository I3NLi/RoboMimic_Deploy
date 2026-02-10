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
    with mujoco.viewer.launch_passive(m, d) as viewer:
        sim_start_time = time.time()
        while viewer.is_running() and Running:
            try:
                if(joystick.is_button_pressed(JoystickButton.SELECT)):
                    Running = False

                joystick.update()
                if joystick.is_button_released(JoystickButton.L3):
                    state_cmd.skill_cmd = FSMCommand.PASSIVE
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
        
