"""Input adapters shared by simulation, shadow compare, and real deploy."""


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

