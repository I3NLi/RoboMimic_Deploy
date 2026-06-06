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
            self.num_actions = config["num_actions"]
            self.num_obs = config["num_obs"]
            self.ang_vel_scale = config["ang_vel_scale"]
            self.dof_pos_scale = config["dof_pos_scale"]
            self.dof_vel_scale = config["dof_vel_scale"]
            self.action_scale = np.array(config["action_scale"], dtype=np.float32)
            self.cmd_scale = np.array(config["cmd_scale"], dtype=np.float32)
            cmd_deadzone = config.get("cmd_deadzone", 0.0)
            self.cmd_deadzone = np.array(cmd_deadzone, dtype=np.float32)
            if self.cmd_deadzone.size == 1:
                self.cmd_deadzone = np.full(3, float(self.cmd_deadzone), dtype=np.float32)
            self.cmd_range = config["cmd_range"]
            self.range_velx = np.array([self.cmd_range["lin_vel_x"][0], self.cmd_range["lin_vel_x"][1]], dtype=np.float32)
            self.range_vely = np.array([self.cmd_range["lin_vel_y"][0], self.cmd_range["lin_vel_y"][1]], dtype=np.float32)
            self.range_velz = np.array([self.cmd_range["ang_vel_z"][0], self.cmd_range["ang_vel_z"][1]], dtype=np.float32)
            
            self.qj_obs = np.zeros(self.num_actions, dtype=np.float32)
            self.dqj_obs = np.zeros(self.num_actions, dtype=np.float32)
            self.cmd = np.array(config["cmd_init"], dtype=np.float32)
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
                self.input_name = self.ort_session.get_inputs()[0].name
            else:
                self.policy = torch.jit.load(self.policy_path)

            for _ in range(50):
                obs_tensor = self.obs.reshape(1, -1).astype(np.float32)
                self._policy_forward(obs_tensor)
                    
            backend = "ONNX" if self._use_onnx else "TorchScript"
            print(f"Locomotion policy initializing ... ({backend})")

    def _policy_forward(self, obs_tensor: np.ndarray) -> np.ndarray:
        if self._use_onnx:
            out = self.ort_session.run(None, {self.input_name: obs_tensor})[0]
            out = np.asarray(out, dtype=np.float32)
        else:
            with torch.inference_mode():
                out = self.policy(torch.from_numpy(obs_tensor).clip(-100, 100))
            out = out.detach().cpu().numpy().astype(np.float32)
        return np.clip(out, -100.0, 100.0)
                
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
        out = np.zeros_like(cmd, dtype=np.float32)
        for i, (val, rng) in enumerate(zip(cmd, ranges)):
            lo = float(rng[0])
            hi = float(rng[1])
            if lo < 0.0 < hi:
                out[i] = val * (hi if val >= 0.0 else abs(lo))
            else:
                out[i] = scale_values([val], [(lo, hi)])[0]
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
        self.obs[6:9] = self.cmd.copy()
        self.obs[9: 9 + self.num_actions] = self.qj_obs.copy()
        self.obs[9 + self.num_actions: 9 + self.num_actions * 2] = self.dqj_obs.copy()
        self.obs[9 + self.num_actions * 2: 9 + self.num_actions * 3] = self.action.copy()
        
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
        
