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

# Linux kernel js_event: time(u32), value(s16), type(u8), number(u8) = 8 bytes
EVENT_FORMAT = "IhBB"
L2_R2_RANGE = 32767 - (-32767)  # 65534


def _scale_trigger(val: float) -> float:
    """L2/R2 raw value -32767..32767 -> 0..1."""
    return (val - (-32767)) / L2_R2_RANGE


class DS4Controller(Gamepad):
    """
    PS4 DualShock 4 over /dev/input/js* (Linux joystick API).

    Same interface as Gamepad: joy_l, joy_r, dpad, trigger_l/r, and buttons
    via get_bumper_*(), get_btn_*(), get_dpad_*(), get_joy_*_click().
    """

    # DS4 often has center drift; use larger deadzone than Xbox
    JOY_DEADZONE = 0.2

    def __init__(self, interface: str = "/dev/input/js0") -> None:
        super().__init__()
        self.interface = interface
        self.is_connected = False
        self._event_size = struct.calcsize(EVENT_FORMAT)

    def _handle_event(self, value: int, ev_type: int, btn: int) -> None:
        # Axis events (ev_type 2): L3=0,1 R3=3,4 L2=2 R2=5 hat_x=6 hat_y=7
        if ev_type == 2:
            if btn == 0:  # L3 X
                if value == 0:
                    self.joy_l = Vec2d(0.0, self.joy_l.y)
                else:
                    self.joy_l = Vec2d(self._scale_joy(value), self.joy_l.y)
            elif btn == 1:  # L3 Y
                if value == 0:
                    self.joy_l = Vec2d(self.joy_l.x, 0.0)
                else:
                    s = self._scale_joy(value)
                    self.joy_l = Vec2d(self.joy_l.x, 0.0 if s == 0 else -s)
            elif btn == 2:  # L2
                if value == -32767:
                    self.trigger_l = 0.0
                else:
                    self.trigger_l = _scale_trigger(value)
            elif btn == 3:  # R3 X
                if value == 0:
                    self.joy_r = Vec2d(0.0, self.joy_r.y)
                else:
                    self.joy_r = Vec2d(self._scale_joy(value), self.joy_r.y)
            elif btn == 4:  # R3 Y
                if value == 0:
                    self.joy_r = Vec2d(self.joy_r.x, 0.0)
                else:
                    s = self._scale_joy(value)
                    self.joy_r = Vec2d(self.joy_r.x, 0.0 if s == 0 else -s)
            elif btn == 5:  # R2
                if value == -32767:
                    self.trigger_r = 0.0
                else:
                    self.trigger_r = _scale_trigger(value)
            elif btn == 6:  # hat X
                if value == -32767:
                    self.dpad = Vec2d(-1.0, self.dpad.y)
                    self._record_button_press("dpad_left")
                elif value == 32767:
                    self.dpad = Vec2d(1.0, self.dpad.y)
                    self._record_button_press("dpad_right")
                else:
                    self.dpad = Vec2d(0.0, self.dpad.y)
                    self._record_button_release("dpad_left")
                    self._record_button_release("dpad_right")
            elif btn == 7:  # hat Y
                if value == -32767:
                    self.dpad = Vec2d(self.dpad.x, 1.0)
                    self._record_button_press("dpad_up")
                elif value == 32767:
                    self.dpad = Vec2d(self.dpad.x, -1.0)
                    self._record_button_press("dpad_down")
                else:
                    self.dpad = Vec2d(self.dpad.x, 0.0)
                    self._record_button_release("dpad_up")
                    self._record_button_release("dpad_down")
        # Button events (ev_type 1)
        elif ev_type == 1:
            if btn == 0 and value == 1:
                self._record_button_press("btn_south")
            elif btn == 0 and value == 0:
                self._record_button_release("btn_south")
            elif btn == 1 and value == 1:
                self._record_button_press("btn_east")
            elif btn == 1 and value == 0:
                self._record_button_release("btn_east")
            elif btn == 2 and value == 1:
                self._record_button_press("btn_north")
            elif btn == 2 and value == 0:
                self._record_button_release("btn_north")
            elif btn == 3 and value == 1:
                self._record_button_press("btn_west")
            elif btn == 3 and value == 0:
                self._record_button_release("btn_west")
            elif btn == 4 and value == 1:
                self._record_button_press("bumper_l")
            elif btn == 4 and value == 0:
                self._record_button_release("bumper_l")
            elif btn == 5 and value == 1:
                self._record_button_press("bumper_r")
            elif btn == 5 and value == 0:
                self._record_button_release("bumper_r")
            elif btn == 11 and value == 1:
                self._record_button_press("joy_l_click")
            elif btn == 11 and value == 0:
                self._record_button_release("joy_l_click")
            elif btn == 12 and value == 1:
                self._record_button_press("joy_r_click")
            elif btn == 12 and value == 0:
                self._record_button_release("joy_r_click")

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
