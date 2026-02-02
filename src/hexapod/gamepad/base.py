"""
Base gamepad interface for hexapod control.

All controller implementations (Xbox, DS4, etc.) inherit from Gamepad and expose
the same properties. A background thread updates these properties; the main
event loop reads current values only.

- Joysticks, triggers, dpad: live snapshot; read current value anytime.
- Buttons (bumpers, thumbstick clicks, face): discrete on/off. Use button_held(name)
  for current state (e.g. modifier keys) and consume_button(name) for one-shot
  press events so a fixed-interval loop does not miss a press.
"""

from abc import ABC, abstractmethod
import math
import threading

from hexapod.engine import Vec2d


class Gamepad(ABC):
    """
    Common interface for gamepad input.

    Subclasses run a read loop in a daemon thread and update instance
    attributes. The main script reads joy_l, joy_r, triggers, dpad on each
    tick. For discrete buttons, use button_held(name) or consume_button(name).
    """

    JOY_DEADZONE = 0.15
    JOY_MAX = 32767  # raw axis range is -32768..32767
    Z_MAX = 255  # trigger axis range

    def __init__(self) -> None:
        self.joy_l = Vec2d()
        self.joy_r = Vec2d()
        self.dpad = Vec2d()
        self.trigger_l = 0.0
        self.trigger_r = 0.0
        self._stop = False
        self._read_thread: threading.Thread | None = None
        self._button_pending: dict[str, bool] = {}
        self._button_held: dict[str, bool] = {}
        self._button_lock = threading.Lock()

    def start_reading(self) -> None:
        """Start the background thread that continuously updates gamepad state."""
        if self._read_thread is not None and self._read_thread.is_alive():
            return
        self._stop = False
        self._read_thread = threading.Thread(target=self._read_loop, daemon=True)
        self._read_thread.start()

    def stop_reading(self, timeout: float = 5.0) -> None:
        """
        Stop the read loop. Blocks until the thread exits (up to timeout seconds).
        Call start_reading() to restart.
        """
        self._stop = True
        if self._read_thread is not None and self._read_thread.is_alive():
            self._read_thread.join(timeout=timeout)

    def button_held(self, name: str) -> bool:
        """Return True if the button is currently held (e.g. modifier keys)."""
        with self._button_lock:
            return self._button_held.get(name, False)

    def consume_button(self, name: str) -> bool:
        """
        Return True if the button had a press since last consume, and clear it.

        Use for one-shot actions (e.g. A = dance) so you do not miss a press
        between event loop ticks, and so multiple rapid presses do not queue.
        """
        with self._button_lock:
            return self._button_pending.pop(name, False)

    def _record_button_press(self, name: str) -> None:
        """Subclasses call on press (0->1). At most one pending per name until consumed."""
        with self._button_lock:
            self._button_pending[name] = True
            self._button_held[name] = True

    def _record_button_release(self, name: str) -> None:
        """Subclasses call on release (1->0)."""
        with self._button_lock:
            self._button_held[name] = False

    @abstractmethod
    def _read_loop(self) -> None:
        """
        Subclasses implement: loop while not self._stop, updating joy_*, trigger_*, dpad, buttons.
        Exit when self._stop becomes True.
        """
        ...

    def _scale_joy(self, val: float) -> float:
        """Scale raw axis to [-1, 1] with deadzone."""
        scaled = val / self.JOY_MAX
        if abs(scaled) < self.JOY_DEADZONE:
            return 0.0
        return float(
            math.copysign(
                (abs(scaled) - self.JOY_DEADZONE) / (1.0 - self.JOY_DEADZONE),
                scaled,
            )
        )

    def _scale_z(self, val: float) -> float:
        """Scale trigger axis to [0, 1]."""
        return val / self.Z_MAX
