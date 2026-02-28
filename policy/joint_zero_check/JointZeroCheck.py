from common.path_config import PROJECT_ROOT

from FSM.FSMState import FSMStateName, FSMState
from common.ctrlcomp import StateAndCmd, PolicyOutput
from common.utils import FSMCommand
import numpy as np
import yaml
import os


class JointZeroCheck(FSMState):
    def __init__(self, state_cmd: StateAndCmd, policy_output: PolicyOutput):
        super().__init__()
        self.state_cmd = state_cmd
        self.policy_output = policy_output
        self.name = FSMStateName.JOINT_ZERO_CHECK
        self.name_str = "joint_zero_check"

        current_dir = os.path.dirname(os.path.abspath(__file__))
        config_path = os.path.join(current_dir, "config", "JointZeroCheck.yaml")
            with open(config_path, "r", encoding="utf-8") as f:
            config = yaml.load(f, Loader=yaml.FullLoader)
            self.kds = np.array(config["kds"], dtype=np.float32)
            self.kps = np.array(config["kps"], dtype=np.float32)
            self.default_angles = np.array(config["default_angles"], dtype=np.float32)
            self.joint2motor_idx = np.array(config["joint2motor_idx"], dtype=np.int32)
            self.control_dt = config["control_dt"]
            self.hold_time = config.get("hold_time", 1.0)
            self.settle_time = config.get("settle_time", 0.3)

        self.hold_steps = max(1, int(self.hold_time / self.control_dt))
        self.settle_steps = max(0, int(self.settle_time / self.control_dt))
        self.cur_joint = 0
        self.step_in_joint = 0

    def enter(self):
        self.cur_joint = 0
        self.step_in_joint = 0
        print("[JointZeroCheck] Start joint zero check (MuJoCo).")

    def run(self):
        # Hold all joints at default angles
        for j in range(len(self.joint2motor_idx)):
            motor_idx = self.joint2motor_idx[j]
            self.policy_output.actions[motor_idx] = self.default_angles[j]
            self.policy_output.kps[motor_idx] = self.kps[j]
            self.policy_output.kds[motor_idx] = self.kds[j]

        self.step_in_joint += 1

        # After settling, print offset for current joint
        if self.step_in_joint == self.settle_steps:
            motor_idx = self.joint2motor_idx[self.cur_joint]
            q = self.state_cmd.q[motor_idx]
            target = self.default_angles[self.cur_joint]
            offset = q - target
            print(f"[JointZeroCheck] joint_idx={self.cur_joint} motor_idx={motor_idx} q={q:.4f} target={target:.4f} offset={offset:.4f}")

        # Move to next joint after hold time
        if self.step_in_joint >= self.hold_steps:
            self.step_in_joint = 0
            self.cur_joint += 1
            if self.cur_joint >= len(self.joint2motor_idx):
                self.cur_joint = 0

    def exit(self):
        print("[JointZeroCheck] Exit joint zero check.")

    def checkChange(self):
        if self.state_cmd.skill_cmd == FSMCommand.LOCO:
            self.state_cmd.skill_cmd = FSMCommand.INVALID
            return FSMStateName.LOCOMODE
        if self.state_cmd.skill_cmd == FSMCommand.PASSIVE:
            self.state_cmd.skill_cmd = FSMCommand.INVALID
            return FSMStateName.PASSIVE
        return FSMStateName.JOINT_ZERO_CHECK
