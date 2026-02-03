from common.path_config import PROJECT_ROOT

from FSM.FSMState import FSMStateName, FSMState
from common.ctrlcomp import StateAndCmd, PolicyOutput
import numpy as np
import yaml
from common.utils import FSMCommand, progress_bar
import onnx
import onnxruntime
import torch
import os


class BeyondMimic(FSMState):
    def __init__(self, state_cmd:StateAndCmd, policy_output:PolicyOutput):
        super().__init__()
        self.state_cmd = state_cmd
        self.policy_output = policy_output
        self.name = FSMStateName.SKILL_BEYOND_MIMIC
        self.name_str = "beyond_mimic"
        self.motion_phase = 0
        self.counter_step = 0
        self.ref_motion_phase = 0
        
        current_dir = os.path.dirname(os.path.abspath(__file__))
        config_path = os.path.join(current_dir, "config", "BeyondMimic.yaml")
        with open(config_path, "r") as f:
            config = yaml.load(f, Loader=yaml.FullLoader)
            self.onnx_path = os.path.join(current_dir, "model", config["onnx_path"])
            self.kps_lab = np.array(config["kp_lab"], dtype=np.float32)
            self.kds_lab = np.array(config["kd_lab"], dtype=np.float32)
            self.default_angles_lab =  np.array(config["default_angles_lab"], dtype=np.float32)
            self.mj2lab =  np.array(config["mj2lab"], dtype=np.int32)
            self.tau_limit =  np.array(config["tau_limit"], dtype=np.float32)
            self.num_actions = config["num_actions"]
            self.num_obs = config["num_obs"]
            self.action_scale_lab = np.array(config["action_scale_lab"], dtype=np.float32)
            self.motion_length = config["motion_length"]
            
            self.qj_obs = np.zeros(self.num_actions, dtype=np.float32)
            self.dqj_obs = np.zeros(self.num_actions, dtype=np.float32)
            self.obs = np.zeros(self.num_obs)
            self.action = np.zeros(self.num_actions)
            
            self.ref_joint_pos = np.zeros(self.num_actions, dtype=np.float32)
            self.ref_joint_vel = np.zeros(self.num_actions, dtype=np.float32)
            self.ref_body_pos_w = np.zeros((1, 14, 3), dtype=np.float32)
            self.ref_body_quat_w = np.zeros((1, 14, 4), dtype=np.float32)
            self.ref_body_lin_vel_w = np.zeros((1, 14, 3), dtype=np.float32)
            self.ref_body_ang_vel_w = np.zeros((1, 14, 3), dtype=np.float32)
            # load policy
            self.onnx_model = onnx.load(self.onnx_path)
            self.ort_session = onnxruntime.InferenceSession(self.onnx_path)
            ort_inputs = self.ort_session.get_inputs()
            self.input_name = [getattr(inpt, "name", inpt) for inpt in ort_inputs]

            metadata = {p.key: p.value for p in self.onnx_model.metadata_props}
            self.obs_names = self._parse_csv_list(metadata.get("observation_names", ""))
            if not self.obs_names:
                self.obs_names = [
                    "command",
                    "motion_anchor_ori_b",
                    "base_ang_vel",
                    "joint_pos",
                    "joint_vel",
                    "actions",
                ]

            obs_input_dim = None
            if ort_inputs:
                first = ort_inputs[0]
                first_shape = getattr(first, "shape", None)
                if first_shape:
                    try:
                        dim_val = first_shape[-1]
                        if isinstance(dim_val, int) and dim_val > 0:
                            obs_input_dim = dim_val
                    except Exception:
                        pass
                if obs_input_dim is None:
                    first_type = getattr(first, "type", None)
                    tensor_type = getattr(first_type, "tensor_type", None)
                    shape = getattr(tensor_type, "shape", None)
                    dims = getattr(shape, "dim", None)
                    if dims:
                        obs_input_dim = dims[-1].dim_value

            history_list = self._parse_csv_ints(metadata.get("observation_history_lengths", ""))
            self.per_frame_dim = self._per_frame_dim_from_names(self.obs_names)
            self.history_length = 1
            if history_list and len(set(history_list)) == 1:
                self.history_length = history_list[0]
            elif obs_input_dim and self.per_frame_dim > 0 and obs_input_dim % self.per_frame_dim == 0:
                self.history_length = obs_input_dim // self.per_frame_dim

            if obs_input_dim:
                self.num_obs = int(obs_input_dim)
            else:
                self.num_obs = int(self.per_frame_dim * self.history_length)

            action_scale = self._parse_csv_floats(metadata.get("action_scale", ""))
            if action_scale.size == self.num_actions:
                self.action_scale_lab = action_scale

            default_joint_pos = self._parse_csv_floats(metadata.get("default_joint_pos", ""))
            if default_joint_pos.size == self.num_actions:
                self.default_angles_lab = default_joint_pos

            body_names = self._parse_csv_list(metadata.get("body_names", ""))
            anchor_body_name = metadata.get("anchor_body_name", "")
            if body_names and anchor_body_name in body_names:
                self.anchor_body_idx = body_names.index(anchor_body_name)
            else:
                self.anchor_body_idx = 7

            print("BeyondMimic-like policy initializing ...")
    
    def enter(self):
        self.ref_motion_phase = 0.
        self.motion_time = 0
        self.counter_step = 0

        observation = {}
        observation[self.input_name[0]] = np.zeros((1, self.num_obs), dtype=np.float32)
        observation[self.input_name[1]] = np.zeros((1, 1), dtype=np.float32)
        outputs_result = self.ort_session.run(None, observation)
        # 处理多个输出
        self.action, self.ref_joint_pos, self.ref_joint_vel, _, self.ref_body_quat_w, _, _ = outputs_result

        self.qj_obs = np.zeros(self.num_actions, dtype=np.float32)
        self.dqj_obs = np.zeros(self.num_actions, dtype=np.float32)
        self.obs = np.zeros(self.num_obs, dtype=np.float32)
        self.obs_history = np.zeros((self.history_length, self.per_frame_dim), dtype=np.float32)

        # self.action = np.zeros(self.num_actions)

        pass
        
    def quat_mul(self, q1, q2):
        w1, x1, y1, z1 = q1[0], q1[1], q1[2], q1[3]
        w2, x2, y2, z2 = q2[0], q2[1], q2[2], q2[3]
        # perform multiplication
        ww = (z1 + x1) * (x2 + y2)
        yy = (w1 - y1) * (w2 + z2)
        zz = (w1 + y1) * (w2 - z2)
        xx = ww + yy + zz
        qq = 0.5 * (xx + (z1 - x1) * (x2 - y2))
        w = qq - ww + (z1 - y1) * (y2 - z2)
        x = qq - xx + (x1 + w1) * (x2 + w2)
        y = qq - yy + (w1 - x1) * (y2 + z2)
        z = qq - zz + (z1 + y1) * (w2 - x2)
        return np.array([w, x, y, z])
        
    def matrix_from_quat(self, q):
        w, x, y, z = q
        return np.array([
            [1 - 2 * (y**2 + z**2), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x**2 + z**2), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x**2 + y**2)]
        ])

    def yaw_quat(self, q):
        w, x, y, z = q
        yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y**2 + z**2))
        return np.array([np.cos(yaw / 2), 0, 0, np.sin(yaw / 2)])
    
    def euler_single_axis_to_quat(self, angle, axis, degrees=False):
        """
        将单个欧拉角转换为四元数
        
        参数:
            angle: 旋转角度
            axis: 旋转轴，可以是 'x', 'y', 'z' 或者单位向量 [x, y, z]
            degrees: 如果为True，输入角度为度数；如果为False，输入角度为弧度
        
        返回:
            四元数 (w, x, y, z)
        """
        # 转换角度为弧度
        if degrees:
            angle = np.radians(angle)
        
        # 计算半角
        half_angle = angle * 0.5
        cos_half = np.cos(half_angle)
        sin_half = np.sin(half_angle)
        
        # 根据旋转轴确定四元数分量
        if isinstance(axis, str):
            if axis.lower() == 'x':
                return np.array([cos_half, sin_half, 0.0, 0.0])
            elif axis.lower() == 'y':
                return np.array([cos_half, 0.0, sin_half, 0.0])
            elif axis.lower() == 'z':
                return np.array([cos_half, 0.0, 0.0, sin_half])
            else:
                raise ValueError("axis must be 'x', 'y', 'z' or a 3D unit vector")
        else:
            # 假设axis是一个3D向量 [x, y, z]
            axis = np.array(axis, dtype=np.float32)
            # 归一化轴向量
            axis_norm = np.linalg.norm(axis)
            if axis_norm == 0:
                raise ValueError("axis vector cannot be zero")
            axis = axis / axis_norm
            
            # 计算四元数分量
            w = cos_half
            x = sin_half * axis[0]
            y = sin_half * axis[1]
            z = sin_half * axis[2]
            
            return np.array([w, x, y, z])

    def quat_apply_inverse(self, q, v):
        q = np.array(q, dtype=np.float32)
        v = np.array(v, dtype=np.float32)
        q_conj = np.array([q[0], -q[1], -q[2], -q[3]], dtype=np.float32)
        v_quat = np.array([0.0, v[0], v[1], v[2]], dtype=np.float32)
        return self.quat_mul(self.quat_mul(q_conj, v_quat), q)[1:]

    def _gravity_from_quat(self, q):
        gravity_w = np.array([0.0, 0.0, -1.0], dtype=np.float32)
        return self.quat_apply_inverse(q, gravity_w)

    def _parse_csv_list(self, value: str):
        if not value:
            return []
        return [v.strip() for v in value.split(',') if v.strip()]

    def _parse_csv_floats(self, value: str):
        if not value:
            return np.array([], dtype=np.float32)
        return np.array([float(v) for v in value.split(',') if v != ''], dtype=np.float32)

    def _parse_csv_ints(self, value: str):
        if not value:
            return []
        return [int(float(v)) for v in value.split(',') if v != '']

    def _per_frame_dim_from_names(self, names):
        dim = 0
        for name in names:
            if name == 'command':
                dim += self.num_actions * 2
            elif name in ('motion_anchor_ori_b',):
                dim += 6
            elif name in ('motion_anchor_pos_b',):
                dim += 3
            elif name in ('base_gravity', 'base_lin_vel', 'base_ang_vel'):
                dim += 3
            elif name in ('joint_pos', 'joint_vel', 'actions'):
                dim += self.num_actions
        return dim

    def _pad_or_trim(self, obs, target_size):
        if obs.size == target_size:
            return obs
        if obs.size < target_size:
            return np.pad(obs, (0, target_size - obs.size))
        return obs[:target_size]

    def _term_vector(self, name, motion_anchor_ori_b, base_gravity, base_ang_vel, qj, dqj):
        if name == 'command':
            return np.concatenate((self.ref_joint_pos.squeeze(0), self.ref_joint_vel.squeeze(0)), axis=-1)
        if name == 'motion_anchor_ori_b':
            return motion_anchor_ori_b[:, :2].reshape(-1)
        if name == 'motion_anchor_pos_b':
            return np.zeros(3, dtype=np.float32)
        if name == 'base_gravity':
            return base_gravity
        if name == 'base_lin_vel':
            base_lin_vel = getattr(self.state_cmd, 'lin_vel', None)
            if base_lin_vel is None:
                return np.zeros(3, dtype=np.float32)
            base_lin_vel = np.asarray(base_lin_vel, dtype=np.float32).reshape(-1)
            if base_lin_vel.size != 3:
                base_lin_vel = self._pad_or_trim(base_lin_vel, 3)
            return base_lin_vel
        if name == 'base_ang_vel':
            base_ang_vel = np.asarray(base_ang_vel, dtype=np.float32).reshape(-1)
            if base_ang_vel.size != 3:
                base_ang_vel = self._pad_or_trim(base_ang_vel, 3)
            return base_ang_vel
        if name == 'joint_pos':
            return qj
        if name == 'joint_vel':
            return dqj
        if name == 'actions':
            action_vec = self.action.squeeze(0).astype(np.float32)
            return action_vec
        return np.zeros(0, dtype=np.float32)

    def _build_frame_obs(self, motion_anchor_ori_b, base_gravity, base_ang_vel, qj, dqj):
        parts = []
        for name in self.obs_names:
            vec = self._term_vector(name, motion_anchor_ori_b, base_gravity, base_ang_vel, qj, dqj)
            if vec.size:
                parts.append(vec)
        if not parts:
            return np.zeros(self.per_frame_dim, dtype=np.float32)
        return np.concatenate(parts, axis=-1).astype(np.float32)

    def _append_history(self, frame_obs):
        if self.history_length <= 1:
            return frame_obs
        self.obs_history = np.roll(self.obs_history, -1, axis=0)
        self.obs_history[-1] = frame_obs
        return self.obs_history.reshape(-1)

    def run(self):
        robot_quat = self.state_cmd.base_quat
        
        qj = self.state_cmd.q[self.mj2lab]
        qj = (qj - self.default_angles_lab)

        base_troso_yaw = qj[2]
        base_troso_roll = qj[5]
        base_troso_pitch = qj[8]
        
        # beyond mimic使用torso姿态作为姿态输入，需要根据腰部位置将pelvis数据转到torso
        quat_yaw = self.euler_single_axis_to_quat(base_troso_yaw, 'z', degrees=False)
        quat_roll = self.euler_single_axis_to_quat(base_troso_roll, 'x', degrees=False)
        quat_pitch = self.euler_single_axis_to_quat(base_troso_pitch, 'y', degrees=False)
        temp1 = self.quat_mul(quat_roll, quat_pitch)
        temp2 = self.quat_mul(quat_yaw, temp1)
        robot_quat = self.quat_mul(robot_quat, temp2)
        ref_anchor_ori_w = self.ref_body_quat_w[:, self.anchor_body_idx].squeeze(0)

        # 在第一帧提取当前机器人yaw方向，与参考动作yaw方向做差（与beyond mimic一致）
        if(self.counter_step < 2):
            init_to_anchor = self.matrix_from_quat(self.yaw_quat(ref_anchor_ori_w))
            world_to_anchor = self.matrix_from_quat(self.yaw_quat(robot_quat))
            self.init_to_world = world_to_anchor @ init_to_anchor.T
            print("self.init_to_world: ", self.init_to_world)
            self.counter_step += 1
            return

        motion_anchor_ori_b = self.matrix_from_quat(robot_quat).T @ self.init_to_world @ self.matrix_from_quat(ref_anchor_ori_w)

        ang_vel = np.asarray(self.state_cmd.ang_vel, dtype=np.float32).reshape(-1)
        if ang_vel.size != 3:
            ang_vel = self._pad_or_trim(ang_vel, 3)
        dqj = self.state_cmd.dq
        dqj = dqj[self.mj2lab]

        base_gravity = getattr(self.state_cmd, "gravity_ori", None)
        if base_gravity is None or len(base_gravity) != 3:
            base_gravity = self._gravity_from_quat(robot_quat)
        base_gravity = np.asarray(base_gravity, dtype=np.float32).reshape(-1)
        if base_gravity.size != 3:
            base_gravity = self._pad_or_trim(base_gravity, 3)

        frame_obs = self._build_frame_obs(motion_anchor_ori_b, base_gravity, ang_vel, qj, dqj)
        obs_input = self._append_history(frame_obs)
        if obs_input.size != self.num_obs:
            obs_input = self._pad_or_trim(obs_input, self.num_obs)

        observation = {}

        # obs0 是网络观测，obs1 是当前时间步，用于输出参考动作信息
        observation[self.input_name[0]] = obs_input.reshape(1, -1).astype(np.float32)
        observation[self.input_name[1]] = np.array([[self.counter_step]], dtype=np.float32)
        outputs_result = self.ort_session.run(None, observation)

        # 处理多个输出
        self.action, self.ref_joint_pos, self.ref_joint_vel, _, self.ref_body_quat_w, _, _ = outputs_result
        target_dof_pos_mj = np.zeros(self.num_actions, dtype=np.float32)
        action_vec = self.action.squeeze(0).astype(np.float32)
        target_dof_pos_lab = action_vec * self.action_scale_lab + self.default_angles_lab
        target_dof_pos_mj[self.mj2lab] = target_dof_pos_lab
        
        self.policy_output.actions = target_dof_pos_mj
        self.policy_output.kps[self.mj2lab] = self.kps_lab
        self.policy_output.kds[self.mj2lab] = self.kds_lab
        
        # update motion phase
        self.counter_step += 1

    def exit(self):
        self.action = np.zeros(self.num_actions, dtype=np.float32)
        self.obs_history = np.zeros((self.history_length, self.per_frame_dim), dtype=np.float32)
        self.ref_motion_phase = 0.
        self.motion_time = 0
        self.counter_step = 0
        
        print("exited")

    
    def checkChange(self):
        if(self.state_cmd.skill_cmd == FSMCommand.LOCO):
            self.state_cmd.skill_cmd = FSMCommand.INVALID
            return FSMStateName.SKILL_COOLDOWN
        elif(self.state_cmd.skill_cmd == FSMCommand.PASSIVE):
            self.state_cmd.skill_cmd = FSMCommand.INVALID
            return FSMStateName.PASSIVE
        elif(self.state_cmd.skill_cmd == FSMCommand.POS_RESET):
            self.state_cmd.skill_cmd = FSMCommand.INVALID
            return FSMStateName.FIXEDPOSE
        else:
            self.state_cmd.skill_cmd = FSMCommand.INVALID
            return FSMStateName.SKILL_BEYOND_MIMIC