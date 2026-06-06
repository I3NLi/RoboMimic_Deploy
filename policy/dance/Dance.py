from common.path_config import PROJECT_ROOT

from FSM.FSMState import FSMStateName, FSMState
from common.ctrlcomp import StateAndCmd, PolicyOutput
import numpy as np
import yaml
from common.utils import FSMCommand, progress_bar
import onnxruntime
import os


class Dance(FSMState):
    def __init__(self, state_cmd:StateAndCmd, policy_output:PolicyOutput):
        super().__init__()
        self.state_cmd = state_cmd
        self.policy_output = policy_output
        self.name = FSMStateName.SKILL_Dance
        self.name_str = "skill_dance"
        self.motion_phase = 0
        self.counter_step = 0
        self.ref_motion_phase = 0
        
        current_dir = os.path.dirname(os.path.abspath(__file__))
        config_path = os.path.join(current_dir, "config", "Dance.yaml")
        with open(config_path, "r") as f:
            config = yaml.load(f, Loader=yaml.FullLoader)
            self.onnx_path = os.path.join(current_dir, "model", config["onnx_path"])
            trajectory_path = os.path.expanduser(os.path.expandvars(config["trajectory_path"]))
            if not os.path.isabs(trajectory_path):
                trajectory_path = os.path.join(current_dir, trajectory_path)
            self.trajectory_path = os.path.normpath(trajectory_path)
            self.kps = np.array(config["kps"], dtype=np.float32)
            self.kds = np.array(config["kds"], dtype=np.float32)
            self.default_angles = np.array(config["default_angles"], dtype=np.float32)
            self.joint2motor_idx = np.array(config["joint2motor_idx"], dtype=np.int32)
            self.command_joint_indices = np.array(config["command_joint_indices"], dtype=np.int32)
            self.num_actions = config["num_actions"]
            self.num_obs = config["num_obs"]
            self.obs_clip = float(config.get("obs_clip", 100.0))
            self.action_clip = float(config.get("action_clip", 100.0))
            self.action_scale = np.array(config["action_scale"], dtype=np.float32)
            self.control_dt = float(config.get("control_dt", 0.02))
            self.resident_control = bool(config.get("resident_control", False))
            
            self.qj_obs = np.zeros(self.num_actions, dtype=np.float32)
            self.dqj_obs = np.zeros(self.num_actions, dtype=np.float32)
            self.obs = np.zeros(self.num_obs, dtype=np.float32)
            self.action = np.zeros(self.num_actions, dtype=np.float32)

            trajectory = np.load(self.trajectory_path)
            self.ref_joint_pos_all = trajectory["joint_pos"].astype(np.float32)
            self.ref_joint_vel_all = trajectory["joint_vel"].astype(np.float32)
            self.ref_body_quat_w_all = trajectory["body_quat_w"].astype(np.float32)
            fps = trajectory["fps"].reshape(-1)
            self.trajectory_fps = float(fps[0]) if fps.size else 50.0
            self.max_policy_step = int(self.ref_joint_pos_all.shape[0] - 1)
            self.motion_length = self.ref_joint_pos_all.shape[0] / max(self.trajectory_fps, 1e-6)
            
            # load policy
            self.ort_session = onnxruntime.InferenceSession(self.onnx_path)
            self.input_name = self.ort_session.get_inputs()[0].name
            for _ in range(50):
                self.ort_session.run(None, {self.input_name: self.obs.reshape(1, -1)})[0]
                    
            print(
                "Dance policy initializing ... "
                f"(Z1 WBT, obs_dim={self.num_obs}, actions={self.num_actions}, frames={self.max_policy_step + 1})"
            )

    @staticmethod
    def _quat_to_matrix(q):
        q = np.asarray(q, dtype=np.float64).reshape(4)
        norm = np.linalg.norm(q)
        if norm < 1e-9:
            return np.eye(3, dtype=np.float32)
        w, x, y, z = q / norm
        return np.array(
            [
                [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
                [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
                [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
            ],
            dtype=np.float32,
        )

    @staticmethod
    def _yaw_matrix_from_rotation(rot):
        yaw = np.arctan2(rot[1, 0], rot[0, 0])
        c = np.cos(yaw)
        s = np.sin(yaw)
        return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=np.float32)

    def _policy_to_motor(self, values):
        out = np.zeros(self.state_cmd.num_joints, dtype=np.float32)
        for policy_idx, motor_idx in enumerate(self.joint2motor_idx):
            if policy_idx < len(values) and 0 <= motor_idx < out.size:
                out[motor_idx] = values[policy_idx]
        return out

    def _motor_to_policy(self, values):
        values = np.asarray(values, dtype=np.float32).reshape(-1)
        out = np.zeros(self.num_actions, dtype=np.float32)
        for policy_idx, motor_idx in enumerate(self.joint2motor_idx):
            if 0 <= motor_idx < values.size:
                out[policy_idx] = values[motor_idx]
        return out

    def _motion_anchor_ori_b(self, step):
        ref_rot_w = self._quat_to_matrix(self.ref_body_quat_w_all[step])
        base_quat = getattr(self.state_cmd, "base_quat", np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32))
        body_rot_w = self._quat_to_matrix(base_quat)
        ref_aligned = self.ref_init_yaw_rot.T @ ref_rot_w
        body_aligned = self.body_init_yaw_rot.T @ body_rot_w
        anchor_ori_rot_b = body_aligned.T @ ref_aligned
        return anchor_ori_rot_b[:, :2].reshape(-1).astype(np.float32)
    
    def enter(self):
        self.action = np.zeros(self.num_actions, dtype=np.float32)
        self.ref_motion_phase = 0.0
        self.counter_step = 0
        self.obs = np.zeros(self.num_obs, dtype=np.float32)
        self.kps_reorder = self._policy_to_motor(self.kps)
        self.kds_reorder = self._policy_to_motor(self.kds)

        ref_rot_w = self._quat_to_matrix(self.ref_body_quat_w_all[0])
        base_quat = getattr(self.state_cmd, "base_quat", np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32))
        body_rot_w = self._quat_to_matrix(base_quat)
        self.ref_init_yaw_rot = self._yaw_matrix_from_rotation(ref_rot_w)
        self.body_init_yaw_rot = self._yaw_matrix_from_rotation(body_rot_w)
        
        
    def run(self):
        step = min(self.counter_step, self.max_policy_step)
        qj = self._motor_to_policy(self.state_cmd.q)
        dqj = self._motor_to_policy(self.state_cmd.dq)
        joint_pos = qj - self.default_angles
        joint_vel = dqj
        command = np.concatenate(
            (
                self.ref_joint_pos_all[step, self.command_joint_indices],
                self.ref_joint_vel_all[step, self.command_joint_indices],
            ),
            axis=-1,
            dtype=np.float32,
        )
        obs = np.concatenate(
            (
                command,
                self._motion_anchor_ori_b(step),
                self.state_cmd.gravity_ori.reshape(-1).astype(np.float32),
                self.state_cmd.ang_vel.reshape(-1).astype(np.float32),
                joint_pos.astype(np.float32),
                joint_vel.astype(np.float32),
                self.action.astype(np.float32),
            ),
            axis=-1,
            dtype=np.float32,
        )
        if obs.size != self.num_obs:
            raise ValueError(f"Dance obs dim mismatch: got {obs.size}, expected {self.num_obs}")
        self.obs = np.clip(obs, -self.obs_clip, self.obs_clip).astype(np.float32)

        self.action = np.squeeze(self.ort_session.run(None, {self.input_name: self.obs.reshape(1, -1)})[0])
        self.action = np.clip(self.action.astype(np.float32), -self.action_clip, self.action_clip)
        if self.resident_control:
            target_dof_pos = self.ref_joint_pos_all[step] + self.action * self.action_scale
        else:
            target_dof_pos = self.default_angles + self.action * self.action_scale

        self.policy_output.actions = self._policy_to_motor(target_dof_pos)
        self.policy_output.kps = self.kps_reorder.copy()
        self.policy_output.kds = self.kds_reorder.copy()
        
        # update motion phase
        self.counter_step += 1
        motion_time = min(self.counter_step / max(self.trajectory_fps, 1e-6), self.motion_length)
        self.ref_motion_phase = motion_time / self.motion_length
        print(progress_bar(motion_time, self.motion_length), end="", flush=True)
    
    def exit(self):
        self.action = np.zeros(self.num_actions, dtype=np.float32)
        self.ref_motion_phase = 0.0
        self.counter_step = 0
        print()

    
    def checkChange(self):
        if(self.state_cmd.skill_cmd == FSMCommand.LOCO):
            self.state_cmd.skill_cmd = FSMCommand.INVALID
            return FSMStateName.LOCOMODE
        elif(self.state_cmd.skill_cmd == FSMCommand.PASSIVE):
            self.state_cmd.skill_cmd = FSMCommand.INVALID
            return FSMStateName.PASSIVE
        elif(self.state_cmd.skill_cmd == FSMCommand.POS_RESET):
            self.state_cmd.skill_cmd = FSMCommand.INVALID
            return FSMStateName.FIXEDPOSE
        else:
            self.state_cmd.skill_cmd = FSMCommand.INVALID
            return FSMStateName.SKILL_Dance
