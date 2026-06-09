from shared.path_config import PROJECT_ROOT

import pygame
import os
from pygame.locals import *
from enum import IntEnum, unique

@unique
class JoystickButton(IntEnum):
    # Standard PlayStation/Xbox Layout
    A = 0      # PS: Cross(×), Xbox: A
    B = 1      # PS: Circle(○), Xbox: B
    X = 2      # PS: Square(□), Xbox: X
    Y = 3      # PS: Triangle(△), Xbox: Y
    L1 = 4     # Left Bumper (L1 on PS)
    R1 = 5     # Right Bumper (R1 on PS)
    SELECT = 6   # Select/Share button
    START = 7  # Start/Options button
    L3 = 8     # Left Stick Press
    R3 = 9     # Right Stick Press
    HOME = 10  # PS: PS FSMCommand, Xbox: Xbox FSMCommand
    UP = 11    # D-pad Up (if mapped as separate button)
    DOWN = 12  # D-pad Down
    LEFT = 13  # D-pad Left
    RIGHT = 14 # D-pad Right

class JoyStick:
    def __init__(self):
        pygame.init()
        pygame.joystick.init()
        
        joystick_count = pygame.joystick.get_count()
        if joystick_count == 0:
            raise RuntimeError("No joystick connected!")
        
        preferred_name = os.environ.get("MAGICBOT_JOYSTICK_NAME", "MagicBot Virtual Gamepad")
        joystick_index = 0
        if preferred_name:
            preferred_name_lower = preferred_name.lower()
            for i in range(joystick_count):
                candidate = pygame.joystick.Joystick(i)
                candidate.init()
                if preferred_name_lower in candidate.get_name().lower():
                    joystick_index = i
                    break
                candidate.quit()

        self.joystick = pygame.joystick.Joystick(joystick_index)
        self.joystick.init()
        print(f"[Joystick] using #{joystick_index}: {self.joystick.get_name()}")
        
        self.button_count = self.joystick.get_numbuttons()
        self.button_states = [False] * self.button_count  
        self.button_pressed = [False] * self.button_count  
        self.button_released = [False] * self.button_count 

        self.axis_count = self.joystick.get_numaxes()
        self.axis_states = [0.0] * self.axis_count
        
        self.hat_count = self.joystick.get_numhats()
        self.hat_states = [(0, 0)] * self.hat_count
        self.dpad_state = {"up": False, "down": False, "left": False, "right": False}
        self.dpad_pressed = {"up": False, "down": False, "left": False, "right": False}
        self.dpad_released = {"up": False, "down": False, "left": False, "right": False}
        
        
    def update(self):
        """update joystick state"""
        pygame.event.pump()  
        
        self.button_released = [False] * self.button_count
        for k in self.dpad_pressed:
            self.dpad_pressed[k] = False
            self.dpad_released[k] = False
        
        for i in range(self.button_count):
            current_state = self.joystick.get_button(i) == 1
            if self.button_states[i] and not current_state:
                self.button_released[i] = True
            self.button_states[i] = current_state

        for i in range(self.axis_count):
            self.axis_states[i] = self.joystick.get_axis(i)
        
        prev_dpad = self.dpad_state.copy()
        new_dpad = {"up": False, "down": False, "left": False, "right": False}
        for i in range(self.hat_count):
            hat = self.joystick.get_hat(i)
            self.hat_states[i] = hat
            hx, hy = hat
            if hy == 1:
                new_dpad["up"] = True
            if hy == -1:
                new_dpad["down"] = True
            if hx == -1:
                new_dpad["left"] = True
            if hx == 1:
                new_dpad["right"] = True
        self.dpad_state = new_dpad
        for k in self.dpad_state:
            if self.dpad_state[k] and not prev_dpad[k]:
                self.dpad_pressed[k] = True
            if prev_dpad[k] and not self.dpad_state[k]:
                self.dpad_released[k] = True

    def is_button_pressed(self, button_id):
        """detect button pressed"""
        if button_id == JoystickButton.UP:
            return self.dpad_state["up"]
        if button_id == JoystickButton.DOWN:
            return self.dpad_state["down"]
        if button_id == JoystickButton.LEFT:
            return self.dpad_state["left"]
        if button_id == JoystickButton.RIGHT:
            return self.dpad_state["right"]
        if 0 <= button_id < self.button_count:
            return self.button_states[button_id]
        return False

    def is_button_released(self, button_id):
        """detect button released"""
        if button_id == JoystickButton.UP:
            return self.dpad_released["up"]
        if button_id == JoystickButton.DOWN:
            return self.dpad_released["down"]
        if button_id == JoystickButton.LEFT:
            return self.dpad_released["left"]
        if button_id == JoystickButton.RIGHT:
            return self.dpad_released["right"]
        if 0 <= button_id < self.button_count:
            return self.button_released[button_id]
        return False

    def get_axis_value(self, axis_id):
        """get joystick axis value"""
        if 0 <= axis_id < self.axis_count:
            return self.axis_states[axis_id]
        return 0.0

    def get_hat_direction(self, hat_id=0):
        """get joystick hat direction"""
        if 0 <= hat_id < self.hat_count:
            return self.hat_states[hat_id]
        return (0, 0)
