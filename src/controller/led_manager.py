from plasma import WS2812
from servo import servo2040

import utime

from utils import log_led


class LEDColor:
    colors = {
        "red": 0.0,
        "orange": 0.08,
        "yellow": 0.16,
        "green": 0.33,
        "blue": 0.66,
    }

    def __init__(self, h: str = "blue", s: float = 1.0, v: float = 0.0):
        self.values = (self.colors[h], s, v)


class LEDEffect:
    def update(self, now_ms: int) -> "tuple[float, float, float] | None":
        return


# naming semantics
class LEDOff(LEDEffect):
    pass


class LEDSolid(LEDEffect):
    def __init__(self, color: LEDColor):
        self.color = color.values

    def update(self, now_ms: int):
        return self.color


class LEDBlink(LEDEffect):
    def __init__(self, color: LEDColor, interval_s: float = 1.0):
        self.color = color.values
        self.interval_ms = int(interval_s * 1000)
        self.interval_half_ms = self.interval_ms // 2

    def update(self, now_ms: int):
        if self.interval_half_ms <= 0:
            return None

        if (now_ms // self.interval_half_ms) & 1 == 0:
            return self.color

        return None


class LEDPulse(LEDEffect):
    def __init__(self, color: LEDColor, duration_s: float = 5.0):
        self.color = color.values
        self.end_ms = utime.ticks_ms() + int(duration_s * 1000)

    def update(self, now_ms: int):
        if utime.ticks_diff(self.end_ms, now_ms) > 0:
            return self.color

        return None


class LedManager:
    def __init__(self, pio: int, sm: int):
        self.leds = WS2812(servo2040.NUM_LEDS, pio, sm, servo2040.LED_DATA)
        self.leds.start()
        self.effects = [LEDEffect()] * servo2040.NUM_LEDS
        self.last_cmd = [
            (0, 0, 0)
        ] * servo2040.NUM_LEDS  # type: list[tuple[float, float, float] | None]

    def set_effect(self, idx: int, effect: LEDEffect):
        self.effects[idx] = effect

    def step(self, now_ms):
        for i, effect in enumerate(self.effects):
            color = effect.update(now_ms)
            if color != self.last_cmd[i]:
                self.last_cmd[i] = color
                if color is None:
                    self.leds.set_rgb(i, 0, 0, 0)
                else:
                    h, s, v = color
                    self.leds.set_hsv(i, h, s, v)
