"""
Xbox-style gamepad via the `inputs` library.

Uses the same axis/button codes as typical Xbox controllers on Linux/Windows.
"""

from inputs import get_gamepad

from hexapod.engine import Vec2d
from hexapod.gamepad.base import Gamepad


class XboxController(Gamepad):
    """Xbox (or compatible) controller; state updated in a background thread."""

    # Evdev codes -> logical button name
    _BUTTONS: dict[str, str] = {
        "BTN_TL": "bumper_l",
        "BTN_TR": "bumper_r",
        "BTN_THUMBL": "l3",
        "BTN_THUMBR": "r3",
        "BTN_SOUTH": "a",
        "BTN_EAST": "b",
        "BTN_WEST": "x",
        "BTN_NORTH": "y",
    }

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
                    self.dpad = Vec2d(
                        -1.0 if e.state < 0 else (1.0 if e.state > 0 else 0.0),
                        self.dpad.y,
                    )
                elif e.code == "ABS_HAT0Y":
                    # Hat Y: negative = up, positive = down (match DS4: up -> dpad.y=1)
                    self.dpad = Vec2d(
                        self.dpad.x,
                        1.0 if e.state < 0 else (-1.0 if e.state > 0 else 0.0),
                    )
                elif e.code in self._BUTTONS:
                    name = self._BUTTONS[e.code]
                    if e.state == 1:
                        self._record_button_press(name)
                    else:
                        self._record_button_release(name)
