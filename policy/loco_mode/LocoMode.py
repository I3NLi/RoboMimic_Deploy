from common.path_config import PROJECT_ROOT

from FSM.FSMState import FSMStateName, FSMState
from common.ctrlcomp import StateAndCmd, PolicyOutput, FSMCommand
from common.utils import scale_values
import numpy as np
import yaml
import torch
import os

try:
    import onnxruntime
except Exception:
    onnxruntime = None

class LocoMode(FSMState):
    def __init__(self, state_cmd:StateAndCmd, policy_output:PolicyOutput):
        super().__init__()
        self.state_cmd = state_cmd
        self.policy_output = policy_output
        self.name = FSMStateName.LOCOMODE
        self.name_str = "Loco_mode"
        
        current_dir = os.path.dirname(os.path.abspath(__file__))
        config_path = os.path.join(current_dir, "config", "LocoMode_lowKp.yaml")
        with open(config_path, "r") as f:
            config = yaml.load(f, Loader=yaml.FullLoader)
            model_rel_path = config.get("policy_path", "")
            if not model_rel_path:
                raise ValueError("LocoMode config must provide `policy_path`.")
            self.policy_path = os.path.join(current_dir, "model", model_rel_path)
            self.kps = np.array(config["kps"], dtype=np.float32)
            self.kds = np.array(config["kds"], dtype=np.float32)
            self.default_angles =  np.array(config["default_angles"], dtype=np.float32)
            self.joint2motor_idx =  np.array(config["joint2motor_idx"], dtype=np.int32)
            self.tau_limit =  np.array(config["tau_limit"], dtype=np.float32)
            self.tau_limit_scale = float(config.get("tau_limit_scale", 1.0))
            self.num_actions = int(config["num_actions"])
            self.num_obs = int(config["num_obs"])
            self.ang_vel_scale = config["ang_vel_scale"]
            self.dof_pos_scale = config["dof_pos_scale"]
            self.dof_vel_scale = config["dof_vel_scale"]
            self.action_scale = np.array(config["action_scale"], dtype=np.float32)
            self.cmd_scale = np.array(config["cmd_scale"], dtype=np.float32).reshape(-1)
            self.command_dim = int(config.get("command_dim", self.cmd_scale.size))
            if self.command_dim not in (3, 4):
                raise ValueError(f"LocoMode supports 3D or 4D commands, got command_dim={self.command_dim}.")
            if self.cmd_scale.size != self.command_dim:
                raise ValueError(
                    f"LocoMode command_dim={self.command_dim} does not match cmd_scale len={self.cmd_scale.size}."
                )
            if self.command_dim >= 4 and "root_height_command" not in config:
                raise ValueError("LocoMode 4D command config must provide `root_height_command`.")
            self.root_height_command = float(config.get("root_height_command", 0.0))
            cmd_deadzone = config.get("cmd_deadzone", 0.0)
            self.cmd_deadzone = np.array(cmd_deadzone, dtype=np.float32)
            if self.cmd_deadzone.size == 1:
                self.cmd_deadzone = np.full(3, float(self.cmd_deadzone), dtype=np.float32)
            self.cmd_range = config["cmd_range"]
            self.range_velx = np.array([self.cmd_range["lin_vel_x"][0], self.cmd_range["lin_vel_x"][1]], dtype=np.float32)
            self.range_vely = np.array([self.cmd_range["lin_vel_y"][0], self.cmd_range["lin_vel_y"][1]], dtype=np.float32)
            self.range_velz = np.array([self.cmd_range["ang_vel_z"][0], self.cmd_range["ang_vel_z"][1]], dtype=np.float32)
            self._validate_config(config_path, config)
            
            self.qj_obs = np.zeros(self.num_actions, dtype=np.float32)
            self.dqj_obs = np.zeros(self.num_actions, dtype=np.float32)
            self.cmd = self._fit_command(config.get("cmd_init", [0.0, 0.0, 0.0]))
            self.obs = np.zeros(self.num_obs, dtype=np.float32)
            self.action = np.zeros(self.num_actions, dtype=np.float32)

            # load policy backend
            self._use_onnx = self.policy_path.lower().endswith(".onnx")
            if self._use_onnx:
                if onnxruntime is None:
                    raise ImportError(
                        f"onnxruntime is required for ONNX policy: {self.policy_path}"
                    )
                self.ort_session = onnxruntime.InferenceSession(self.policy_path)
                model_input = self.ort_session.get_inputs()[0]
                self.input_name = model_input.name
                model_obs_dim = model_input.shape[-1]
                if isinstance(model_obs_dim, int) and model_obs_dim != self.num_obs:
                    raise ValueError(
                        f"LocoMode num_obs={self.num_obs} does not match ONNX input dim={model_obs_dim}: "
                        f"{self.policy_path}"
                    )
                model_output = self.ort_session.get_outputs()[0]
                model_action_dim = model_output.shape[-1]
                if isinstance(model_action_dim, int) and model_action_dim != self.num_actions:
                    raise ValueError(
                        f"LocoMode num_actions={self.num_actions} does not match ONNX output dim={model_action_dim}: "
                        f"{self.policy_path}"
                    )
            else:
                self.policy = torch.jit.load(self.policy_path)

            for _ in range(50):
                obs_tensor = self.obs.reshape(1, -1).astype(np.float32)
                self._policy_forward(obs_tensor)
                    
            backend = "ONNX" if self._use_onnx else "TorchScript"
            print(
                f"Locomotion policy initializing ... ({backend}) "
                f"policy={os.path.basename(self.policy_path)} num_obs={self.num_obs} "
                f"num_actions={self.num_actions} command_dim={self.command_dim}"
            )

    def _policy_forward(self, obs_tensor: np.ndarray) -> np.ndarray:
        if self._use_onnx:
            out = self.ort_session.run(None, {self.input_name: obs_tensor})[0]
            out = np.asarray(out, dtype=np.float32)
        else:
            with torch.inference_mode():
                out = self.policy(torch.from_numpy(obs_tensor).clip(-100, 100))
            out = out.detach().cpu().numpy().astype(np.float32)
        return np.clip(out, -100.0, 100.0)

    def _fit_command(self, cmd) -> np.ndarray:
        out = np.zeros(self.command_dim, dtype=np.float32)
        arr = np.asarray(cmd, dtype=np.float32).reshape(-1)
        n = min(3, arr.size)
        if n > 0:
            out[:n] = arr[:n]
        if self.command_dim >= 4:
            out[3] = float(arr[3]) if arr.size > 3 else self.root_height_command
        return out

    def _validate_config(self, config_path: str, config: dict):
        expected_obs = 6 + self.command_dim + self.num_actions * 3
        if self.num_obs != expected_obs:
            raise ValueError(
                f"{config_path}: num_obs={self.num_obs} does not match LocoMode obs layout "
                f"6 + command_dim({self.command_dim}) + 3*num_actions({self.num_actions}) = {expected_obs}."
            )

        action_fields = {
            "kps": self.kps,
            "kds": self.kds,
            "default_angles": self.default_angles,
            "joint2motor_idx": self.joint2motor_idx,
            "tau_limit": self.tau_limit,
        }
        for name, values in action_fields.items():
            if values.size != self.num_actions:
                raise ValueError(
                    f"{config_path}: {name} len={values.size} must match num_actions={self.num_actions}."
                )

        if self.action_scale.size not in (1, self.num_actions):
            raise ValueError(
                f"{config_path}: action_scale len={self.action_scale.size} must be scalar or num_actions={self.num_actions}."
            )

        joint_ids = self.joint2motor_idx.astype(np.int32).tolist()
        if sorted(joint_ids) != list(range(self.num_actions)):
            raise ValueError(
                f"{config_path}: joint2motor_idx must be a permutation of 0..{self.num_actions - 1}."
            )

        if self.cmd_deadzone.size != 3:
            raise ValueError(f"{config_path}: cmd_deadzone must be scalar or len=3.")

        cmd_init = np.asarray(config.get("cmd_init", []), dtype=np.float32).reshape(-1)
        if cmd_init.size != self.command_dim:
            raise ValueError(
                f"{config_path}: cmd_init len={cmd_init.size} must match command_dim={self.command_dim}."
            )

        for key in ("lin_vel_x", "lin_vel_y", "ang_vel_z"):
            if key not in self.cmd_range or len(self.cmd_range[key]) != 2:
                raise ValueError(f"{config_path}: cmd_range.{key} must be [min, max].")
                
    @staticmethod
    def _apply_deadzone(cmd: np.ndarray, deadzone: np.ndarray) -> np.ndarray:
        deadzone = np.clip(deadzone, 0.0, 0.95)
        out = cmd.copy()
        for i in range(min(out.size, deadzone.size)):
            dz = deadzone[i]
            v = out[i]
            if abs(v) <= dz:
                out[i] = 0.0
            else:
                out[i] = np.sign(v) * (abs(v) - dz) / max(1e-6, (1.0 - dz))
        return out
    
    def _scale_cmd(self, cmd: np.ndarray) -> np.ndarray:
        ranges = (self.range_velx, self.range_vely, self.range_velz)
        out = np.zeros(self.command_dim, dtype=np.float32)
        for i, (val, rng) in enumerate(zip(cmd, ranges)):
            lo = float(rng[0])
            hi = float(rng[1])
            if lo < 0.0 < hi:
                out[i] = val * (hi if val >= 0.0 else abs(lo))
            else:
                out[i] = scale_values([val], [(lo, hi)])[0]
        if self.command_dim >= 4:
            out[3] = self.root_height_command
        return out
    
    def enter(self):
        self.kps_reorder = np.zeros_like(self.kps)
        self.kds_reorder = np.zeros_like(self.kds)
        self.default_angles_reorder = np.zeros_like(self.default_angles)
        self.tau_limit_reorder = np.zeros_like(self.tau_limit)
        for i in range(len(self.joint2motor_idx)):
            motor_idx = self.joint2motor_idx[i]
            self.kps_reorder[motor_idx] = self.kps[i]
            self.kds_reorder[motor_idx] = self.kds[i]
            self.default_angles_reorder[motor_idx] = self.default_angles[i]
            self.tau_limit_reorder[motor_idx] = self.tau_limit[i]
            
    
    def run(self):
        self.gravity_orientation = self.state_cmd.gravity_ori
        self.qj = self.state_cmd.q.copy()
        self.dqj = self.state_cmd.dq.copy()
        self.ang_vel = self.state_cmd.ang_vel.copy()
        joycmd = self.state_cmd.vel_cmd.copy()
        joycmd = self._apply_deadzone(joycmd, self.cmd_deadzone)
        self.cmd = self._scale_cmd(joycmd)
        
        for i in range(len(self.joint2motor_idx)):
            self.qj_obs[i] = self.qj[self.joint2motor_idx[i]]
            self.dqj_obs[i] = self.dqj[self.joint2motor_idx[i]]
            
        self.qj_obs = (self.qj_obs - self.default_angles) * self.dof_pos_scale
        self.dqj_obs = self.dqj_obs * self.dof_vel_scale
        self.ang_vel = self.ang_vel * self.ang_vel_scale
        self.cmd = self.cmd * self.cmd_scale
        
        self.obs[:3] = self.ang_vel.copy()
        self.obs[3:6] = self.gravity_orientation.copy()
        cmd_end = 6 + self.command_dim
        joint_pos_start = cmd_end
        joint_vel_start = joint_pos_start + self.num_actions
        action_start = joint_vel_start + self.num_actions
        self.obs[6:cmd_end] = self.cmd.copy()
        self.obs[joint_pos_start: joint_pos_start + self.num_actions] = self.qj_obs.copy()
        self.obs[joint_vel_start: joint_vel_start + self.num_actions] = self.dqj_obs.copy()
        self.obs[action_start: action_start + self.num_actions] = self.action.copy()
        
        obs_tensor = self.obs.reshape(1, -1).astype(np.float32)
        self.action = self._policy_forward(obs_tensor).squeeze()
        target_dof_pos = self.action * self.action_scale + self.default_angles
        target_dof_pos_mj = target_dof_pos.copy()
        for i in range(len(self.joint2motor_idx)):
            motor_idx = self.joint2motor_idx[i]
            target_dof_pos_mj[motor_idx] = target_dof_pos[i]

        tau_limit = self.tau_limit_reorder * self.tau_limit_scale
        kp = self.kps_reorder
        kd = self.kds_reorder
        tau = (target_dof_pos_mj - self.qj) * kp + (0.0 - self.dqj) * kd
        tau = np.clip(tau, -tau_limit, tau_limit)
        safe_kp = np.where(kp > 1e-6, kp, 1.0)
        target_dof_pos_mj_limited = self.qj + (tau + kd * self.dqj) / safe_kp
        target_dof_pos_mj = np.where(kp > 1e-6, target_dof_pos_mj_limited, target_dof_pos_mj)
        
        self.policy_output.actions = target_dof_pos_mj.copy()
        self.policy_output.kps = self.kps_reorder.copy()
        self.policy_output.kds = self.kds_reorder.copy()
        # print("actions: ", self.policy_output.actions)
    
    def exit(self):
        pass
    
    def checkChange(self):
        if(self.state_cmd.skill_cmd == FSMCommand.SKILL_1):
            return FSMStateName.SKILL_BEYOND_MIMIC
        elif(self.state_cmd.skill_cmd == FSMCommand.SKILL_4):
            return FSMStateName.SKILL_BEYOND_MIMIC
        elif(self.state_cmd.skill_cmd == FSMCommand.PASSIVE):
            return FSMStateName.PASSIVE
        else:
            return FSMStateName.LOCOMODE
        
