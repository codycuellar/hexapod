"""
Base gamepad interface for hexapod control.

All controller implementations (Xbox, DS4, etc.) inherit from Gamepad and expose
the same unified API. A background thread updates state; the main loop reads
via get_* methods.

- Joysticks, triggers: get_joy_l(), get_joy_r(), get_trigger_l(), get_trigger_r()
- Buttons: get_bumper_l(), get_btn_north(), etc. Each returns (hit, held) where
  hit=True if pressed since last get (consumed), held=True if currently down.
  State changes (press/release) are queued so rapid release+press between frames
  still shows the release for one frame before the press.
"""

from abc import ABC, abstractmethod
import math
import threading

from hexapod.engine import Vec2d


class Gamepad(ABC):
    """
    Common interface for gamepad input.

    Subclasses run a read loop in a daemon thread and update instance
    attributes. Read state via get_joy_*(), get_trigger_*(), get_bumper_*(), etc.
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
        self._button_queue: dict[str, list[bool]] = {}
        self._button_held: dict[str, bool] = {}
        self._button_lock = threading.Lock()
        self._raw_joy = False

    def set_raw_joy(self, enabled: bool) -> None:
        """When True, _scale_joy returns raw values (no deadzone). For debugging."""
        self._raw_joy = enabled

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

    # --- Continuous inputs ---

    def get_joy_l(self) -> Vec2d:
        """Left joystick position, [-1, 1] per axis with deadzone."""
        return self.joy_l

    def get_joy_r(self) -> Vec2d:
        """Right joystick position, [-1, 1] per axis with deadzone."""
        return self.joy_r

    def get_trigger_l(self) -> float:
        """Left trigger, 0.0 to 1.0."""
        return self.trigger_l

    def get_trigger_r(self) -> float:
        """Right trigger, 0.0 to 1.0."""
        return self.trigger_r

    # --- Discrete buttons: each returns (hit, held) ---

    def _get_button(self, name: str) -> tuple[bool, bool]:
        """
        Return (hit, held) for a button. State changes are queued so rapid
        release+press between frames shows the release for one frame first.
        """
        with self._button_lock:
            queue = self._button_queue.setdefault(name, [])
            if queue:
                state = queue.pop(0)
                return (state, state)
            return (False, self._button_held.get(name, False))

    def get_bumper_l(self) -> tuple[bool, bool]:
        """Left bumper. Returns (hit, held)."""
        return self._get_button("bumper_l")

    def get_bumper_r(self) -> tuple[bool, bool]:
        """Right bumper. Returns (hit, held)."""
        return self._get_button("bumper_r")

    def get_btn_north(self) -> tuple[bool, bool]:
        """North face button (Y/Triangle). Returns (hit, held)."""
        return self._get_button("btn_north")

    def get_btn_south(self) -> tuple[bool, bool]:
        """South face button (A/Cross). Returns (hit, held)."""
        return self._get_button("btn_south")

    def get_btn_east(self) -> tuple[bool, bool]:
        """East face button (B/Circle). Returns (hit, held)."""
        return self._get_button("btn_east")

    def get_btn_west(self) -> tuple[bool, bool]:
        """West face button (X/Square). Returns (hit, held)."""
        return self._get_button("btn_west")

    def get_dpad_up(self) -> tuple[bool, bool]:
        """D-pad up. Returns (hit, held)."""
        return self._get_button("dpad_up")

    def get_dpad_down(self) -> tuple[bool, bool]:
        """D-pad down. Returns (hit, held)."""
        return self._get_button("dpad_down")

    def get_dpad_left(self) -> tuple[bool, bool]:
        """D-pad left. Returns (hit, held)."""
        return self._get_button("dpad_left")

    def get_dpad_right(self) -> tuple[bool, bool]:
        """D-pad right. Returns (hit, held)."""
        return self._get_button("dpad_right")

    def get_joy_l_click(self) -> tuple[bool, bool]:
        """Left stick click (L3). Returns (hit, held)."""
        return self._get_button("joy_l_click")

    def get_joy_r_click(self) -> tuple[bool, bool]:
        """Right stick click (R3). Returns (hit, held)."""
        return self._get_button("joy_r_click")

    def _record_button_press(self, name: str) -> None:
        """Subclasses call on press (0->1). Appends to queue for ordered reporting."""
        with self._button_lock:
            self._button_queue.setdefault(name, []).append(True)
            self._button_held[name] = True

    def _record_button_release(self, name: str) -> None:
        """Subclasses call on release (1->0). Appends to queue for ordered reporting."""
        with self._button_lock:
            self._button_queue.setdefault(name, []).append(False)
            self._button_held[name] = False

    @abstractmethod
    def _read_loop(self) -> None:
        """
        Subclasses implement: loop while not self._stop, updating joy_*, trigger_*, dpad, buttons.
        Exit when self._stop becomes True.
        """
        ...

    def _scale_joy(self, val: float) -> float:
        """Scale raw axis to [-1, 1] with deadzone. No-op when set_raw_joy(True)."""
        if self._raw_joy:
            return float(val)
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
