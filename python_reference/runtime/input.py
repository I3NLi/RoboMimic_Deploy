"""Input adapters shared by simulation, shadow compare, and real deploy."""

from __future__ import annotations

import json
import socket
import time


BUTTON_NAME_TO_ID = {
    "A": 0,
    "B": 1,
    "X": 2,
    "Y": 3,
    "L1": 4,
    "R1": 5,
    "SELECT": 6,
    "START": 7,
    "L3": 8,
    "R3": 9,
    "HOME": 10,
    "UP": 11,
    "DOWN": 12,
    "LEFT": 13,
    "RIGHT": 14,
}


class NullJoyStick:
    """Joystick stub for headless/CI runs."""

    def update(self):
        return None

    def is_button_pressed(self, _button_id):
        return False

    def is_button_released(self, _button_id):
        return False

    def get_axis_value(self, _axis_id):
        return 0.0


class VirtualJoyStick:
    """UDP-backed joystick compatible with ``shared.joystick.JoyStick``.

    The virtual remote sends JSON packets with button names and axis values. The
    simulator can then consume the same ``is_button_pressed`` /
    ``is_button_released`` / ``get_axis_value`` API as the physical pygame
    joystick path.
    """

    def __init__(self, host="127.0.0.1", port=8765, stale_timeout_s=0.5):
        self.host = str(host)
        self.port = int(port)
        self.stale_timeout_s = float(stale_timeout_s)
        self.button_count = max(BUTTON_NAME_TO_ID.values()) + 1
        self.axis_count = 4
        self.button_states = [False] * self.button_count
        self.button_released = [False] * self.button_count
        self.axis_states = [0.0] * self.axis_count
        self.last_packet_time = 0.0
        self.packet_count = 0

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((self.host, self.port))
        self.sock.setblocking(False)
        print(f"[VirtualJoyStick] listening on udp://{self.host}:{self.port}")

    def update(self):
        self.button_released = [False] * self.button_count
        received = False

        while True:
            try:
                data, _addr = self.sock.recvfrom(8192)
            except BlockingIOError:
                break

            received = True
            self.packet_count += 1
            self.last_packet_time = time.monotonic()
            try:
                packet = json.loads(data.decode("utf-8"))
            except Exception as exc:
                print(f"[VirtualJoyStick][WARN] invalid packet: {exc}")
                continue
            self._apply_packet(packet)

        if not received and self.last_packet_time > 0.0:
            if time.monotonic() - self.last_packet_time > self.stale_timeout_s:
                self._clear_state()
                self.last_packet_time = 0.0

    def is_button_pressed(self, button_id):
        idx = self._button_index(button_id)
        if idx is None:
            return False
        return self.button_states[idx]

    def is_button_released(self, button_id):
        idx = self._button_index(button_id)
        if idx is None:
            return False
        return self.button_released[idx]

    def get_axis_value(self, axis_id):
        if 0 <= int(axis_id) < self.axis_count:
            return self.axis_states[int(axis_id)]
        return 0.0

    def _apply_packet(self, packet):
        axes = packet.get("axes") or {}
        self.axis_states[0] = self._clamp_axis(axes.get("axis0", axes.get("lx", 0.0)))
        self.axis_states[1] = self._clamp_axis(axes.get("axis1", -float(axes.get("ly", 0.0) or 0.0)))
        self.axis_states[2] = self._clamp_axis(axes.get("axis2", 0.0))
        self.axis_states[3] = self._clamp_axis(axes.get("axis3", axes.get("rx", 0.0)))

        next_states = [False] * self.button_count
        for button in packet.get("buttons") or []:
            idx = self._button_index(button)
            if idx is not None:
                next_states[idx] = True

        for idx, was_pressed in enumerate(self.button_states):
            if was_pressed and not next_states[idx]:
                self.button_released[idx] = True
        self.button_states = next_states

    def _clear_state(self):
        for idx, was_pressed in enumerate(self.button_states):
            if was_pressed:
                self.button_released[idx] = True
        self.button_states = [False] * self.button_count
        self.axis_states = [0.0] * self.axis_count

    def _button_index(self, button):
        try:
            idx = int(button)
        except Exception:
            idx = BUTTON_NAME_TO_ID.get(str(button).strip().upper())
        if idx is None or idx < 0 or idx >= self.button_count:
            return None
        return idx

    @staticmethod
    def _clamp_axis(value):
        try:
            number = float(value)
        except Exception:
            return 0.0
        return max(-1.0, min(1.0, number))
