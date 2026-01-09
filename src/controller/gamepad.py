import math
import threading

from inputs import get_gamepad

from hexapod.engine import Vec2d


class GamePad:
    JOY_DEADZONE = 0.15
    JOY_MAX = 32767  # actually goes to -32768
    Z_MAX = 255

    def __init__(self):
        self.joy_l = Vec2d()
        self.joy_r = Vec2d()
        self.dpad = Vec2d()
        self.trigger_l = 0.0
        self.trigger_r = 0.0
        self.bumper_l = 0.0
        self.bumper_r = 0.0

        self._read_thread = threading.Thread(target=self._read_loop, daemon=True)

    def start_reading(self):
        self._read_thread.start()

    def _read_loop(self):
        while True:
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
                elif e.code == "BTN_TL":
                    self.bumper_l = float(e.state)
                elif e.code == "BTN_TR":
                    self.bumper_r = float(e.state)

    def _scale_joy(self, val: float):
        scaled = val / self.JOY_MAX
        if abs(scaled) < self.JOY_DEADZONE:
            return 0.0
        return math.copysign(
            (abs(scaled) - self.JOY_DEADZONE) / (1 - self.JOY_DEADZONE), scaled
        )

    def _scale_z(self, val: float):
        return val / self.Z_MAX
