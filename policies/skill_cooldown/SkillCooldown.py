from shared.path_config import PROJECT_ROOT

from fsm.state import FSMStateName, FSMState
from shared.ctrlcomp import StateAndCmd, PolicyOutput, FSMCommand
import numpy as np
import yaml
import torch
import os

class SkillCooldown(FSMState):
    def __init__(self, state_cmd:StateAndCmd, policy_output:PolicyOutput):
        super().__init__()
        self.state_cmd = state_cmd
        self.policy_output = policy_output
        self.name = FSMStateName.SKILL_COOLDOWN
        self.name_str = "skill_cooldown"
        self.alpha = 0.
        self.cur_step = 0
        self.control_dt = 0.02
        self.num_motors = int(getattr(self.state_cmd, "num_joints", len(self.policy_output.actions)))
        self.blend_only = False
        current_dir = os.path.dirname(os.path.abspath(__file__))
        config_path = os.path.join(current_dir, "config", "SkillCooldown.yaml")
        with open(config_path, "r") as f:
            config = yaml.load(f, Loader=yaml.FullLoader)
            policy_path = str(config.get("policy_path", "")).strip()
            self.policy_path = os.path.join(current_dir, "model", policy_path) if policy_path else ""
            self.blend_only = bool(config.get("blend_only", self.blend_only))
            self.kps = np.array(config["kps"], dtype=np.float32)
            self.kds = np.array(config["kds"], dtype=np.float32)
            self.default_angles =  np.array(config["default_angles"], dtype=np.float32)
            self.joint2motor_idx =  np.array(config["joint2motor_idx"], dtype=np.int32)
            self.upper_body_motor_idx =  np.array(config["upper_body_motor_idx"], dtype=np.int32)
            self.lower_body_motor_idx =  np.array(config["lower_body_motor_idx"], dtype=np.int32)
            self.tau_limit =  np.array(config["tau_limit"], dtype=np.float32)
            self.num_actions = config["num_actions"]
            self.num_obs = config["num_obs"]
            self.ang_vel_scale = config["ang_vel_scale"]
            self.dof_pos_scale = config["dof_pos_scale"]
            self.dof_vel_scale = config["dof_vel_scale"]
            self.action_scale = config["action_scale"]
            self.total_time = config["total_time"]
            self.period = config["period"]

            if self.blend_only or (
                self.kps.size != self.num_motors
                or self.kds.size != self.num_motors
                or self.default_angles.size != self.num_motors
                or np.any(self.upper_body_motor_idx >= self.num_motors)
                or np.any(self.lower_body_motor_idx >= self.num_motors)
            ):
                self.blend_only = True
                fixedpose_path = os.path.join(
                    os.path.dirname(current_dir),
                    "fixedpose",
                    "config",
                    "FixedPose.yaml",
                )
                self.kps, self.kds, self.default_angles = self._load_motor_order_targets(
                    fixedpose_path,
                    self.num_motors,
                    fallback_kps=self.kps,
                    fallback_kds=self.kds,
                    fallback_default=self.default_angles,
                )
                self.upper_body_motor_idx = np.arange(self.num_motors, dtype=np.int32)
                self.lower_body_motor_idx = np.array([], dtype=np.int32)
                print(
                    "[SkillCooldown] 24DoF fallback enabled: "
                    "blend current pose to Z1 fixed pose instead of using legacy cooldown policy."
                )
            
            self.qj_obs = np.zeros(self.num_actions, dtype=np.float32)
            self.dqj_obs = np.zeros(self.num_actions, dtype=np.float32)
            self.obs = np.zeros(self.num_obs)
            self.action = np.zeros(self.num_actions)
            
            # load policy
            self.policy = None if self.blend_only else torch.jit.load(self.policy_path)
            
            if self.policy is not None:
                for _ in range(50):
                    with torch.inference_mode():
                        obs_tensor = self.obs.reshape(1, -1)
                        obs_tensor = obs_tensor.astype(np.float32)
                        self.policy(torch.from_numpy(obs_tensor))
                    
            print("SkillCooldown policy initializing ...")

    @staticmethod
    def _fit(values, size, fill=0.0):
        out = np.full(size, fill, dtype=np.float32)
        arr = np.asarray(values, dtype=np.float32).reshape(-1)
        n = min(size, arr.size)
        if n > 0:
            out[:n] = arr[:n]
        return out

    @classmethod
    def _load_motor_order_targets(cls, yaml_path, num_motors, fallback_kps, fallback_kds, fallback_default):
        kps = cls._fit(fallback_kps, num_motors)
        kds = cls._fit(fallback_kds, num_motors)
        default_angles = cls._fit(fallback_default, num_motors)
        if not os.path.isfile(yaml_path):
            return kps, kds, default_angles

        with open(yaml_path, "r") as f:
            cfg = yaml.load(f, Loader=yaml.FullLoader) or {}

        joint2motor_idx = np.array(cfg.get("joint2motor_idx", []), dtype=np.int32)
        raw_kps = np.array(cfg.get("kps", []), dtype=np.float32)
        raw_kds = np.array(cfg.get("kds", []), dtype=np.float32)
        raw_default = np.array(cfg.get("default_angles", []), dtype=np.float32)
        if joint2motor_idx.size == 0:
            return (
                cls._fit(raw_kps, num_motors),
                cls._fit(raw_kds, num_motors),
                cls._fit(raw_default, num_motors),
            )

        for policy_idx, motor_idx in enumerate(joint2motor_idx):
            if not (0 <= motor_idx < num_motors):
                continue
            if policy_idx < raw_kps.size:
                kps[motor_idx] = raw_kps[policy_idx]
            if policy_idx < raw_kds.size:
                kds[motor_idx] = raw_kds[policy_idx]
            if policy_idx < raw_default.size:
                default_angles[motor_idx] = raw_default[policy_idx]
        return kps, kds, default_angles
                
    
    def enter(self):    
        self.num_step = int(self.total_time / self.control_dt)
        self.upper_dof_size = len(self.upper_body_motor_idx)
        self.upper_init_dof_pos = np.zeros(self.upper_dof_size, dtype=np.float32)
        self.alpha = 0.
        self.cur_step = 0
        for i in range(self.upper_dof_size):
            self.upper_init_dof_pos[i] = self.state_cmd.q[self.upper_body_motor_idx[i]]
            
    
    def run(self):
        self.gravity_orientation = self.state_cmd.gravity_ori
        self.qj = self.state_cmd.q.copy()
        self.dqj = self.state_cmd.dq.copy()
        self.ang_vel = self.state_cmd.ang_vel.copy()
        self.cmd = np.zeros(3)

        if self.blend_only:
            self.cur_step += 1
            self.alpha = min(self.cur_step / self.num_step, 1.0)
            self.policy_output.actions = (
                self.upper_init_dof_pos * (1 - self.alpha) + self.default_angles * self.alpha
            ).astype(np.float32)
            self.policy_output.kps = self.kps.copy()
            self.policy_output.kds = self.kds.copy()
            return
            
        self.qj_obs = (self.qj[self.lower_body_motor_idx] - self.default_angles[self.lower_body_motor_idx]) * self.dof_pos_scale
        self.dqj_obs = self.dqj[self.lower_body_motor_idx] * self.dof_vel_scale
        self.ang_vel = self.ang_vel * self.ang_vel_scale
        
        count = self.cur_step * self.control_dt
        phase = count % self.period / self.period
        sin_phase = np.sin(2 * np.pi * phase)
        cos_phase = np.cos(2 * np.pi * phase)
        
        self.obs[:3] = self.ang_vel.copy()
        self.obs[3:6] = self.gravity_orientation.copy()
        self.obs[6:9] = self.cmd.copy()
        self.obs[9: 9 + self.num_actions] = self.qj_obs.copy()
        self.obs[9 + self.num_actions: 9 + self.num_actions * 2] = self.dqj_obs.copy()
        self.obs[9 + self.num_actions * 2: 9 + self.num_actions * 3] = self.action.copy()
        self.obs[9 + 3 * self.num_actions : 9 + 3 * self.num_actions + 2] = np.array([sin_phase, cos_phase])
        
        obs_tensor = self.obs.reshape(1, -1)
        obs_tensor = obs_tensor.astype(np.float32)
        self.action = self.policy(torch.from_numpy(obs_tensor)).detach().numpy().squeeze()
        loco_action = self.action * self.action_scale + self.default_angles[self.lower_body_motor_idx]

        n = min(self.lower_body_motor_idx.size, loco_action.size)
        self.policy_output.actions[self.lower_body_motor_idx[:n]] = loco_action[:n].copy()
        self.policy_output.kps = self.kps.copy()
        self.policy_output.kds = self.kds.copy()
        
        ###########################################################
            
        self.cur_step += 1
        self.alpha = min(self.cur_step / self.num_step, 1.0)
        for j in range(self.upper_dof_size):
            motor_idx = self.upper_body_motor_idx[j]
            target_pos = self.default_angles[motor_idx]
            self.policy_output.actions[motor_idx] = self.upper_init_dof_pos[j] * (1 - self.alpha) + target_pos * self.alpha
        
    
    def exit(self):
        pass
    
    def checkChange(self):
        if(self.cur_step >= self.num_step):
            self.state_cmd.skill_cmd = FSMCommand.INVALID
            return FSMStateName.LOCOMODE
        elif(self.state_cmd.skill_cmd == FSMCommand.PASSIVE):
            self.state_cmd.skill_cmd = FSMCommand.INVALID
            return FSMStateName.PASSIVE
        else:
            self.state_cmd.skill_cmd = FSMCommand.INVALID
            return FSMStateName.SKILL_COOLDOWN
        
