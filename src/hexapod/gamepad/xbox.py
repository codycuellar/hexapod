"""
Xbox-style gamepad via the `inputs` library.

Uses the same axis/button codes as typical Xbox controllers on Linux/Windows.
"""

from inputs import get_gamepad

from hexapod.engine import Vec2d
from hexapod.gamepad.base import Gamepad


class XboxController(Gamepad):
    """Xbox (or compatible) controller; state updated in a background thread."""

    # Evdev codes -> unified button name
    _BUTTONS: dict[str, str] = {
        "BTN_TL": "bumper_l",
        "BTN_TR": "bumper_r",
        "BTN_THUMBL": "joy_l_click",
        "BTN_THUMBR": "joy_r_click",
        "BTN_SOUTH": "btn_south",
        "BTN_EAST": "btn_east",
        "BTN_WEST": "btn_west",
        "BTN_NORTH": "btn_north",
    }

    def __init__(self) -> None:
        super().__init__()
        self._dpad_x = 0
        self._dpad_y = 0

    def _update_dpad_buttons(self, x: int, y: int) -> None:
        """Emit press/release for dpad directions based on hat state change."""
        # x: -1 left, 0 neutral, 1 right
        # y: -1 down, 0 neutral, 1 up (hat convention; we use dpad.y=1 for up)
        prev_x, prev_y = self._dpad_x, self._dpad_y
        self._dpad_x, self._dpad_y = x, y

        # Up/down
        if y != prev_y:
            if prev_y == 1:
                self._record_button_release("dpad_up")
            if prev_y == -1:
                self._record_button_release("dpad_down")
            if y == 1:
                self._record_button_press("dpad_up")
            if y == -1:
                self._record_button_press("dpad_down")

        # Left/right
        if x != prev_x:
            if prev_x == -1:
                self._record_button_release("dpad_left")
            if prev_x == 1:
                self._record_button_release("dpad_right")
            if x == -1:
                self._record_button_press("dpad_left")
            if x == 1:
                self._record_button_press("dpad_right")

    def _read_loop(self) -> None:
        while not self._stop:
            events = get_gamepad()
            for e in events:
                if e.ev_type == "Sync":
                    continue

                if e.code == "ABS_X":
                    self.joy_l = Vec2d(self._scale_joy(e.state), self.joy_l.y)
                elif e.code == "ABS_Y":
                    self.joy_l = Vec2d(self.joy_l.x, self._scale_joy(e.state))
                elif e.code == "ABS_RX":
                    self.joy_r = Vec2d(self._scale_joy(e.state), self.joy_r.y)
                elif e.code == "ABS_RY":
                    self.joy_r = Vec2d(self.joy_r.x, self._scale_joy(e.state))
                elif e.code == "ABS_Z":
                    self.trigger_l = self._scale_z(e.state)
                elif e.code == "ABS_RZ":
                    self.trigger_r = self._scale_z(e.state)
                elif e.code == "ABS_HAT0X":
                    x = -1 if e.state < 0 else (1 if e.state > 0 else 0)
                    self.dpad = Vec2d(
                        float(x),
                        self.dpad.y,
                    )
                    self._update_dpad_buttons(x, self._dpad_y)
                elif e.code == "ABS_HAT0Y":
                    # Hat Y: negative = up, positive = down (match DS4: up -> dpad.y=1)
                    y = 1 if e.state < 0 else (-1 if e.state > 0 else 0)
                    self.dpad = Vec2d(
                        self.dpad.x,
                        float(y),
                    )
                    self._update_dpad_buttons(self._dpad_x, y)
                elif e.code in self._BUTTONS:
                    name = self._BUTTONS[e.code]
                    if e.state == 1:
                        self._record_button_press(name)
                    else:
                        self._record_button_release(name)
