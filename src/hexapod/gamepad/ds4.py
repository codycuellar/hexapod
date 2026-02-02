"""
PS4 DualShock 4 controller via Linux joystick API (/dev/input/js*).

Uses the Linux kernel js_event format (8 bytes): time (4), value (2), type (1), number (1).
See https://docs.kernel.org/input/joydev/joystick-api.html
"""

import logging
import os
import struct
import time

from hexapod.engine import Vec2d
from hexapod.gamepad.base import Gamepad

logger = logging.getLogger(__name__)

# --- Event mapping: Linux js_event (value, type, number) ---


class _DS4Event:
    """
    One joystick event: value (s16), type (u8), number (u8) from kernel js_event.
    Unpack with "IhBB" -> (time, value, type, number); we use value, type, number.
    """

    def __init__(self, value: int, ev_type: int, button_id: int) -> None:
        self.value = value
        self.button_type = ev_type
        self.button_id = button_id

    def L3_event(self) -> bool:
        return self.button_type == 2 and self.button_id in (0, 1)

    def L3_y_at_rest(self) -> bool:
        return self.button_id == 1 and self.value == 0

    def L3_x_at_rest(self) -> bool:
        return self.button_id == 0 and self.value == 0

    def L3_up(self) -> bool:
        return self.button_id == 1 and self.value < 0

    def L3_down(self) -> bool:
        return self.button_id == 1 and self.value > 0

    def L3_left(self) -> bool:
        return self.button_id == 0 and self.value < 0

    def L3_right(self) -> bool:
        return self.button_id == 0 and self.value > 0

    def R3_event(self) -> bool:
        return self.button_type == 2 and self.button_id in (3, 4)

    def R3_y_at_rest(self) -> bool:
        return self.button_id == 4 and self.value == 0

    def R3_x_at_rest(self) -> bool:
        return self.button_id == 3 and self.value == 0

    def R3_up(self) -> bool:
        return self.button_id == 4 and self.value < 0

    def R3_down(self) -> bool:
        return self.button_id == 4 and self.value > 0

    def R3_left(self) -> bool:
        return self.button_id == 3 and self.value < 0

    def R3_right(self) -> bool:
        return self.button_id == 3 and self.value > 0

    def L3_pressed(self) -> bool:
        return self.button_id == 11 and self.button_type == 1 and self.value == 1

    def L3_released(self) -> bool:
        return self.button_id == 11 and self.button_type == 1 and self.value == 0

    def R3_pressed(self) -> bool:
        return self.button_id == 12 and self.button_type == 1 and self.value == 1

    def R3_released(self) -> bool:
        return self.button_id == 12 and self.button_type == 1 and self.value == 0

    # Face buttons: DS4 over joydev typically x=0, circle=1, triangle=2, square=3
    def circle_pressed(self) -> bool:
        return self.button_id == 1 and self.button_type == 1 and self.value == 1

    def circle_released(self) -> bool:
        return self.button_id == 1 and self.button_type == 1 and self.value == 0

    def x_pressed(self) -> bool:
        return self.button_id == 0 and self.button_type == 1 and self.value == 1

    def x_released(self) -> bool:
        return self.button_id == 0 and self.button_type == 1 and self.value == 0

    def triangle_pressed(self) -> bool:
        return self.button_id == 2 and self.button_type == 1 and self.value == 1

    def triangle_released(self) -> bool:
        return self.button_id == 2 and self.button_type == 1 and self.value == 0

    def square_pressed(self) -> bool:
        return self.button_id == 3 and self.button_type == 1 and self.value == 1

    def square_released(self) -> bool:
        return self.button_id == 3 and self.button_type == 1 and self.value == 0

    def options_pressed(self) -> bool:
        return self.button_id == 9 and self.button_type == 1 and self.value == 1

    def options_released(self) -> bool:
        return self.button_id == 9 and self.button_type == 1 and self.value == 0

    def share_pressed(self) -> bool:
        return self.button_id == 8 and self.button_type == 1 and self.value == 1

    def share_released(self) -> bool:
        return self.button_id == 8 and self.button_type == 1 and self.value == 0

    def L1_pressed(self) -> bool:
        return self.button_id == 4 and self.button_type == 1 and self.value == 1

    def L1_released(self) -> bool:
        return self.button_id == 4 and self.button_type == 1 and self.value == 0

    def R1_pressed(self) -> bool:
        return self.button_id == 5 and self.button_type == 1 and self.value == 1

    def R1_released(self) -> bool:
        return self.button_id == 5 and self.button_type == 1 and self.value == 0

    def L2_pressed(self) -> bool:
        return self.button_id == 2 and self.button_type == 2 and -32766 <= self.value <= 32767

    def L2_released(self) -> bool:
        return self.button_id == 2 and self.button_type == 2 and self.value == -32767

    def R2_pressed(self) -> bool:
        return self.button_id == 5 and self.button_type == 2 and -32766 <= self.value <= 32767

    def R2_released(self) -> bool:
        return self.button_id == 5 and self.button_type == 2 and self.value == -32767

    def up_arrow_pressed(self) -> bool:
        return self.button_id == 7 and self.button_type == 2 and self.value == -32767

    def down_arrow_pressed(self) -> bool:
        return self.button_id == 7 and self.button_type == 2 and self.value == 32767

    def up_down_arrow_released(self) -> bool:
        return self.button_id == 7 and self.button_type == 2 and self.value == 0

    def left_arrow_pressed(self) -> bool:
        return self.button_id == 6 and self.button_type == 2 and self.value == -32767

    def right_arrow_pressed(self) -> bool:
        return self.button_id == 6 and self.button_type == 2 and self.value == 32767

    def left_right_arrow_released(self) -> bool:
        return self.button_id == 6 and self.button_type == 2 and self.value == 0

    def playstation_button_pressed(self) -> bool:
        return self.button_id == 10 and self.button_type == 1 and self.value == 1

    def playstation_button_released(self) -> bool:
        return self.button_id == 10 and self.button_type == 1 and self.value == 0


# --- DS4 controller: read from /dev/input/js* and update state ---

# Linux kernel js_event: time(u32), value(s16), type(u8), number(u8) = 8 bytes
# https://docs.kernel.org/input/joydev/joystick-api.html
EVENT_FORMAT = "IhBB"
L2_R2_RANGE = 32767 - (-32767)  # 65534


def _scale_trigger(val: float) -> float:
    """L2/R2 raw value -32767..32767 -> 0..1."""
    return (val - (-32767)) / L2_R2_RANGE


class DS4Controller(Gamepad):
    """
    PS4 DualShock 4 over /dev/input/js* (Linux joystick API).

    Same interface as Gamepad: joy_l, joy_r, dpad, trigger_l/r, and buttons
    (bumper_l/r, l3, r3, a, b, x, y) via button_held/consume_button.
    """

    def __init__(self, interface: str = "/dev/input/js0") -> None:
        super().__init__()
        self.interface = interface
        self.is_connected = False
        self._event_size = struct.calcsize(EVENT_FORMAT)

    def _handle_event(self, value: int, ev_type: int, button_id: int) -> None:
        ev = _DS4Event(value, ev_type, button_id)

        if ev.R3_event():
            if ev.R3_y_at_rest():
                self.joy_r = Vec2d(self.joy_r.x, 0.0)
            elif ev.R3_x_at_rest():
                self.joy_r = Vec2d(0.0, self.joy_r.y)
            elif ev.R3_up():
                self.joy_r = Vec2d(self.joy_r.x, self._scale_joy(ev.value))
            elif ev.R3_down():
                self.joy_r = Vec2d(self.joy_r.x, self._scale_joy(ev.value))
            elif ev.R3_left():
                self.joy_r = Vec2d(self._scale_joy(ev.value), self.joy_r.y)
            elif ev.R3_right():
                self.joy_r = Vec2d(self._scale_joy(ev.value), self.joy_r.y)
        elif ev.L3_event():
            if ev.L3_y_at_rest():
                self.joy_l = Vec2d(self.joy_l.x, 0.0)
            elif ev.L3_x_at_rest():
                self.joy_l = Vec2d(0.0, self.joy_l.y)
            elif ev.L3_up():
                self.joy_l = Vec2d(self.joy_l.x, self._scale_joy(ev.value))
            elif ev.L3_down():
                self.joy_l = Vec2d(self.joy_l.x, self._scale_joy(ev.value))
            elif ev.L3_left():
                self.joy_l = Vec2d(self._scale_joy(ev.value), self.joy_l.y)
            elif ev.L3_right():
                self.joy_l = Vec2d(self._scale_joy(ev.value), self.joy_l.y)
        elif ev.L2_pressed():
            self.trigger_l = _scale_trigger(ev.value)
        elif ev.L2_released():
            self.trigger_l = 0.0
        elif ev.R2_pressed():
            self.trigger_r = _scale_trigger(ev.value)
        elif ev.R2_released():
            self.trigger_r = 0.0
        elif ev.L1_pressed():
            self._record_button_press("bumper_l")
        elif ev.L1_released():
            self._record_button_release("bumper_l")
        elif ev.R1_pressed():
            self._record_button_press("bumper_r")
        elif ev.R1_released():
            self._record_button_release("bumper_r")
        elif ev.L3_pressed():
            self._record_button_press("l3")
        elif ev.L3_released():
            self._record_button_release("l3")
        elif ev.R3_pressed():
            self._record_button_press("r3")
        elif ev.R3_released():
            self._record_button_release("r3")
        elif ev.x_pressed():
            self._record_button_press("a")  # DS4 X = south = a
        elif ev.circle_pressed():
            self._record_button_press("b")
        elif ev.square_pressed():
            self._record_button_press("x")
        elif ev.triangle_pressed():
            self._record_button_press("y")
        elif ev.up_arrow_pressed():
            self.dpad = Vec2d(self.dpad.x, 1.0)
        elif ev.down_arrow_pressed():
            self.dpad = Vec2d(self.dpad.x, -1.0)
        elif ev.up_down_arrow_released():
            self.dpad = Vec2d(self.dpad.x, 0.0)
        elif ev.left_arrow_pressed():
            self.dpad = Vec2d(-1.0, self.dpad.y)
        elif ev.right_arrow_pressed():
            self.dpad = Vec2d(1.0, self.dpad.y)
        elif ev.left_right_arrow_released():
            self.dpad = Vec2d(0.0, self.dpad.y)

    def _read_loop(self) -> None:
        """Background thread: wait for device, then read events until stop or disconnect."""
        while not self._stop:
            # Wait for interface to appear
            for _ in range(30):
                if self._stop or os.path.exists(self.interface):
                    break
                time.sleep(1)
            if self._stop:
                break
            if not os.path.exists(self.interface):
                logger.warning("DS4: interface %s not found, retrying...", self.interface)
                continue

            logger.info("DS4 bound to %s", self.interface)
            self.is_connected = True
            try:
                with open(self.interface, "rb") as f:
                    while not self._stop:
                        raw = f.read(self._event_size)
                        if not raw:
                            break
                        tup = struct.unpack(EVENT_FORMAT, raw)
                        value, ev_type, button_id = tup[1], tup[2], tup[3]
                        self._handle_event(value, ev_type, button_id)
            except OSError as e:
                logger.warning("DS4 interface lost: %s", e)
            finally:
                self.is_connected = False
