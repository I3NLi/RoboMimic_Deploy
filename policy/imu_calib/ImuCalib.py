from common.path_config import PROJECT_ROOT

from FSM.FSMState import FSMStateName, FSMState
from common.ctrlcomp import StateAndCmd, PolicyOutput
from common.utils import FSMCommand
import numpy as np
import yaml
import os
import onnx


class ImuCalib(FSMState):
    def __init__(self, state_cmd: StateAndCmd, policy_output: PolicyOutput, loco_policy):
        super().__init__()
        self.state_cmd = state_cmd
        self.policy_output = policy_output
        self.loco_policy = loco_policy
        self.name = FSMStateName.IMU_CALIB
        self.name_str = "imu_calib"

        current_dir = os.path.dirname(os.path.abspath(__file__))
        config_path = os.path.join(current_dir, "config", "ImuCalib.yaml")
        with open(config_path, "r", encoding="utf-8") as f:
            config = yaml.load(f, Loader=yaml.FullLoader)
            self.control_dt = config.get("control_dt", 0.02)
            self.settle_time = config.get("settle_time", 2.0)
            self.sample_time = config.get("sample_time", 3.0)
            self.onnx_path = config.get("onnx_path", "")

        self.motor_names = None
        if self.onnx_path:
            try:
                model = onnx.load(self.onnx_path)
                meta = {p.key: p.value for p in model.metadata_props}
                names = meta.get("joint_names", "")
                if names:
                    self.motor_names = [n.strip() for n in names.split(",") if n.strip()]
            except Exception:
                self.motor_names = None

        self.settle_steps = max(1, int(self.settle_time / self.control_dt))
        self.sample_steps = max(1, int(self.sample_time / self.control_dt))
        self.cur_step = 0
        self.done = False

        # cache defaults from loco config
        self.default_angles = self.loco_policy.default_angles
        self.joint2motor_idx = self.loco_policy.joint2motor_idx

        self.sum_q = None
        self.sum_g = None
        self.sum_w = None
        self.sample_count = 0

    def enter(self):
        self.cur_step = 0
        self.done = False
        self.sample_count = 0
        self.sum_q = np.zeros_like(self.state_cmd.q, dtype=np.float32)
        self.sum_g = np.zeros(3, dtype=np.float32)
        self.sum_w = np.zeros(3, dtype=np.float32)
        self.state_cmd.skill_cmd = FSMCommand.INVALID
        print("[ImuCalib] Start: using loco mode to stabilize on flat ground.")

    def run(self):
        # use loco policy to keep standing
        self.loco_policy.run()

        self.cur_step += 1
        if self.cur_step <= self.settle_steps:
            return

        # collect samples
        g = np.asarray(self.state_cmd.gravity_ori, dtype=np.float32).reshape(-1)
        if g.size != 3:
            g = g[:3] if g.size >= 3 else np.pad(g, (0, 3 - g.size))
        w = np.asarray(self.state_cmd.ang_vel, dtype=np.float32).reshape(-1)
        if w.size != 3:
            w = w[:3] if w.size >= 3 else np.pad(w, (0, 3 - w.size))

        self.sum_q += self.state_cmd.q.astype(np.float32)
        self.sum_g += g
        self.sum_w += w
        self.sample_count += 1

        if self.sample_count >= self.sample_steps:
            self._report()
            self.done = True

    def _report(self):
        mean_q = self.sum_q / max(1, self.sample_count)
        mean_g = self.sum_g / max(1, self.sample_count)
        mean_w = self.sum_w / max(1, self.sample_count)

        # compute roll/pitch from gravity vector (body frame)
        gx, gy, gz = mean_g.tolist()
        g_norm = np.sqrt(gx * gx + gy * gy + gz * gz) + 1.0e-8
        gx, gy, gz = gx / g_norm, gy / g_norm, gz / g_norm
        # enforce gravity pointing downward (avoid pi jump in roll)
        if gz > 0:
            gx, gy, gz = -gx, -gy, -gz
        roll = np.arctan2(gy, gz)
        pitch = np.arctan2(-gx, np.sqrt(gy * gy + gz * gz))

        # correction quaternion from roll/pitch (yaw=0)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        # q = qy(pitch)*qx(roll) in wxyz
        q_corr = np.array([cp * cr, cp * sr, sp * cr, -sp * sr], dtype=np.float32)

        # build default angles in motor order
        default_motor = np.zeros_like(mean_q)
        for i in range(len(self.joint2motor_idx)):
            motor_idx = self.joint2motor_idx[i]
            default_motor[motor_idx] = self.default_angles[i]
        offsets = mean_q - default_motor

        print("[ImuCalib] ===== Report =====")
        print(f"[ImuCalib] mean_gravity = [{gx:.4f}, {gy:.4f}, {gz:.4f}]")
        print(f"[ImuCalib] mean_ang_vel = [{mean_w[0]:.4f}, {mean_w[1]:.4f}, {mean_w[2]:.4f}]")
        print(f"[ImuCalib] roll_offset(rad) = {roll:.5f}, pitch_offset(rad) = {pitch:.5f}")
        print(f"[ImuCalib] q_correction(wxyz) = [{q_corr[0]:.6f}, {q_corr[1]:.6f}, {q_corr[2]:.6f}, {q_corr[3]:.6f}]")
        print("[ImuCalib] joint_offsets (motor order, |offset| > 0.1):")
        for i, off in enumerate(offsets.tolist()):
            if abs(off) <= 0.1:
                continue
            name = None
            if self.motor_names and i < len(self.motor_names):
                name = self.motor_names[i]
            if name:
                print(f"  motor_idx={i:02d} name={name} offset={off:.5f}")
            else:
                print(f"  motor_idx={i:02d} offset={off:.5f}")
        print("[ImuCalib] ===== End =====")

    def exit(self):
        print("[ImuCalib] Exit.")

    def checkChange(self):
        if self.done:
            return FSMStateName.LOCOMODE
        if self.state_cmd.skill_cmd == FSMCommand.PASSIVE:
            self.state_cmd.skill_cmd = FSMCommand.INVALID
            return FSMStateName.PASSIVE
        return FSMStateName.IMU_CALIB
