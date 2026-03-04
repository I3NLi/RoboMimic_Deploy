import struct
import threading


class KeyMap:
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


class RemoteController:
    def __init__(self):
        self._lock = threading.Lock()
        self.lx = 0
        self.ly = 0
        self.rx = 0
        self.ry = 0
        self.button = [0] * 16
        
        self.button_states = [False] * 16
        self.button_pressed = [False] * 16 
        self.button_released = [False] * 16 

    def set(self, data):
        with self._lock:
            # wireless_remote
            keys = struct.unpack("H", data[2:4])[0]
            for i in range(16):
                self.button[i] = (keys & (1 << i)) >> i
            self.lx = struct.unpack("f", data[4:8])[0]
            self.rx = struct.unpack("f", data[8:12])[0]
            self.ry = struct.unpack("f", data[12:16])[0]
            self.ly = struct.unpack("f", data[20:24])[0]

            for i in range(16):
                current_state = self.button[i] == 1
                if (not self.button_states[i]) and current_state:
                    self.button_pressed[i] = True
                if self.button_states[i] and (not current_state):
                    self.button_released[i] = True
                self.button_states[i] = current_state
            
    def is_button_pressed(self, button_id):
        """detect button pressed"""
        with self._lock:
            if 0 <= button_id < 16:
                return self.button_states[button_id]
            return False

    def is_button_released(self, button_id):
        """detect button released"""
        with self._lock:
            if 0 <= button_id < 16:
                released = self.button_released[button_id]
                # Latch semantics: keep release event until consumed once.
                self.button_released[button_id] = False
                return released
            return False

    def get_axis_value(self, axis_id):
        """get joystick axis value"""
        with self._lock:
            return self.lx, self.rx, self.ry, self.ly
