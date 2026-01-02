import time
import sys
import math
import threading
from collections import deque

from inputs import get_gamepad
import matplotlib.pyplot as plt

from hexapod.engine import Vec2d


class GamePad:
    JOY_DEADZONE = 0.15
    JOY_MAX = 32767  # actually goes to -32768
    Z_MAX = 255

    def __init__(self):
        self.joy_l = Vec2d()
        self.joy_r = Vec2d()
        self.trigger_l = 0.0
        self.trigger_r = 0.0
        self._read_thread = threading.Thread(target=self.update, daemon=True)

    def start_reading(self):
        self._read_thread.start()

    def update(self):
        events = get_gamepad()
        for e in events:
            if e.ev_type == "Sync":
                continue

            if e.code == "ABS_X":
                self.joy_l.x = self._scale_joy(e.state)
            elif e.code == "ABS_Y":
                self.joy_l.y = self._scale_joy(e.state)
            elif e.code == "ABS_RX":
                self.joy_r.x = self._scale_joy(e.state)
            elif e.code == "ABS_RY":
                self.joy_r.y = self._scale_joy(e.state)
            elif e.code == "ABS_Z":
                self.trigger_l = self._scale_z(e.state)
            elif e.code == "ABS_RZ":
                self.trigger_r = self._scale_z(e.state)
        return e

    def _scale_joy(self, val: float):
        scaled = val / self.JOY_MAX
        if abs(scaled) < self.JOY_DEADZONE:
            return 0.0
        return math.copysign(
            (abs(scaled) - self.JOY_DEADZONE) / (1 - self.JOY_DEADZONE), scaled
        )

    def _scale_z(self, val: float):
        return val / self.Z_MAX


gp = GamePad()
gp.start_reading()
try:
    plt.ion()
    fig = plt.figure(figsize=(14, 10))
    ax = fig.add_subplot(111)
    ax.set_xlim(-1, 1)
    ax.set_ylim(-1, 1)
    ax.set_aspect("equal")

    trail_len = 20
    trail = deque(maxlen=trail_len)

    scatters = [ax.plot(0, 0, "ro", alpha=0)[0] for _ in range(trail_len)]

    # create the point ONCE
    (dot,) = ax.plot(0, 0, "ro")

    last_draw = 0
    draw_interval = 1 / 30  # 60 Hz

    while True:
        e = gp.update()
        j = gp.joy_l
        trail.append((j.x, j.y))

        dot.set_data([j.x], [j.y])

        now = time.time()
        if now - last_draw > draw_interval:
            for i, (x, y) in enumerate(trail):
                alpha = (i + 1) / trail_len  # oldest faint, newest brightest
                scatters[i].set_data([x], [y])
                scatters[i].set_alpha(alpha)
            fig.canvas.draw_idle()
            fig.canvas.flush_events()
            last_draw = now
        if e.code == "ABS_X":
            sys.stdout.write(f"\r {j.x:+7.3f}, {j.y:+7.3f}")
        sys.stdout.flush()
except:
    print("xmin", gp.x_min)
    print("xmax", gp.x_max)
    print("ymin", gp.y_min)
    print("ymax", gp.x_max)
