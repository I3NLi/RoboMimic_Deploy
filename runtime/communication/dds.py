"""DDS shadow bridge used to compare Python and C++ policy outputs."""

from __future__ import annotations

import struct
import threading
from enum import IntEnum

import numpy as np

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_ as LowCmdHG
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_ as LowStateHG

try:
    from common.joystick import JoystickButton
except Exception:

    class JoystickButton(IntEnum):
        A = 0
        B = 1
        X = 2
        Y = 3
        L1 = 4
        R1 = 5
        SELECT = 6
        START = 7
        L3 = 8
        R3 = 9
        HOME = 10
        UP = 11
        DOWN = 12
        LEFT = 13
        RIGHT = 14


class WirelessRemoteBuilder:
    """Encode joystick state into the 40-byte wireless_remote array."""

    R1 = 0
    L1 = 1
    start = 2
    select = 3
    R2 = 4
    L2 = 5
    F1 = 6
    F2 = 7
    A = 8
    B = 9
    X = 10
    Y = 11
    up = 12
    right = 13
    down = 14
    left = 15

    _JB_MAP = {
        JoystickButton.A: A,
        JoystickButton.B: B,
        JoystickButton.X: X,
        JoystickButton.Y: Y,
        JoystickButton.L1: L1,
        JoystickButton.R1: R1,
        JoystickButton.SELECT: select,
        JoystickButton.START: start,
        JoystickButton.UP: up,
        JoystickButton.DOWN: down,
        JoystickButton.LEFT: left,
        JoystickButton.RIGHT: right,
    }

    @classmethod
    def from_joystick(cls, joy) -> list:
        bits = 0
        for jb, kbit in cls._JB_MAP.items():
            if joy.is_button_pressed(jb):
                bits |= 1 << kbit

        lx = float(joy.get_axis_value(0))
        ly = float(-joy.get_axis_value(1))
        rx = float(joy.get_axis_value(3))
        ry = 0.0

        data = bytearray(40)
        struct.pack_into("H", data, 2, bits & 0xFFFF)
        struct.pack_into("f", data, 4, lx)
        struct.pack_into("f", data, 8, rx)
        struct.pack_into("f", data, 12, ry)
        struct.pack_into("f", data, 20, ly)
        return list(data)

    @classmethod
    def neutral(cls) -> list:
        return [0] * 40


class DDSBridge:
    """Publish rt/lowstate and subscribe rt/lowcmd for shadow compare."""

    LOWSTATE_TOPIC = "rt/lowstate"
    LOWCMD_TOPIC = "rt/lowcmd"

    def __init__(
        self,
        num_joints: int,
        mj2lab: np.ndarray,
        state_lab_order: bool = False,
        cmd_lab_order: bool = False,
    ):
        self.num_joints = num_joints
        self.mj2lab = np.asarray(mj2lab, dtype=np.int32)
        self.state_lab_order = bool(state_lab_order)
        self.cmd_lab_order = bool(cmd_lab_order)
        self._lock = threading.Lock()
        self._latest_cmd: LowCmdHG | None = None
        self._cmd_recv_count = 0

        self._state_pub = ChannelPublisher(self.LOWSTATE_TOPIC, LowStateHG)
        self._state_pub.Init()
        self._cmd_sub = ChannelSubscriber(self.LOWCMD_TOPIC, LowCmdHG)
        self._cmd_sub.Init(self._on_lowcmd, 10)
        self._tick = 1

    def _on_lowcmd(self, msg: LowCmdHG):
        with self._lock:
            self._latest_cmd = msg
            self._cmd_recv_count += 1

    def publish_state(
        self,
        qpos_mj: np.ndarray,
        qvel_mj: np.ndarray,
        quat_wxyz: np.ndarray,
        omega: np.ndarray,
        wireless_remote: list,
        mode_machine: int = 0,
    ):
        ls = unitree_hg_msg_dds__LowState_()
        ls.tick = self._tick
        self._tick += 1
        ls.mode_machine = mode_machine

        if self.state_lab_order:
            for lab_i in range(self.num_joints):
                mj_i = int(self.mj2lab[lab_i])
                ls.motor_state[lab_i].q = float(qpos_mj[mj_i])
                ls.motor_state[lab_i].dq = float(qvel_mj[mj_i])
        else:
            for i in range(self.num_joints):
                ls.motor_state[i].q = float(qpos_mj[i])
                ls.motor_state[i].dq = float(qvel_mj[i])

        ls.imu_state.quaternion[0] = float(quat_wxyz[0])
        ls.imu_state.quaternion[1] = float(quat_wxyz[1])
        ls.imu_state.quaternion[2] = float(quat_wxyz[2])
        ls.imu_state.quaternion[3] = float(quat_wxyz[3])
        ls.imu_state.gyroscope[0] = float(omega[0])
        ls.imu_state.gyroscope[1] = float(omega[1])
        ls.imu_state.gyroscope[2] = float(omega[2])
        ls.wireless_remote = wireless_remote
        self._state_pub.Write(ls)

    def get_latest_cmd(self):
        with self._lock:
            return self._latest_cmd, self._cmd_recv_count

    def cmd_to_mj_arrays(self, cmd: LowCmdHG):
        target_q_mj = np.zeros(self.num_joints, dtype=np.float32)
        kp_mj = np.zeros(self.num_joints, dtype=np.float32)
        kd_mj = np.zeros(self.num_joints, dtype=np.float32)
        if self.cmd_lab_order:
            for lab_i in range(self.num_joints):
                mj_i = int(self.mj2lab[lab_i])
                target_q_mj[mj_i] = float(cmd.motor_cmd[lab_i].q)
                kp_mj[mj_i] = float(cmd.motor_cmd[lab_i].kp)
                kd_mj[mj_i] = float(cmd.motor_cmd[lab_i].kd)
        else:
            for i in range(self.num_joints):
                target_q_mj[i] = float(cmd.motor_cmd[i].q)
                kp_mj[i] = float(cmd.motor_cmd[i].kp)
                kd_mj[i] = float(cmd.motor_cmd[i].kd)
        return target_q_mj, kp_mj, kd_mj


def initialize_dds(domain_id: int, net: str):
    """Initialize the Unitree DDS channel factory for one transport interface."""
    ChannelFactoryInitialize(int(domain_id), str(net))
