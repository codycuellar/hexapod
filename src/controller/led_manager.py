from plasma import WS2812
from servo import servo2040


class LedManager:
    colors = {
        "red": (0.0, 1.0),
        "orange": (0.08, 1.0),
        "yellow": (0.16, 1.0),
        "green": (0.33, 1.0),
        "blue": (0.66, 1.0),
    }

    def __init__(self, num_leds: int, pio: int, sm: int):
        self.leds = WS2812(num_leds, pio, sm, servo2040.LED_DATA)
        self.leds.start()
        # Available states are:
        # 'on', 'off', 'blink', 'pulse', blink turns on and off based on the time.
            # of time.
        self.states = {
            i: {
                "effect": "off",
                "color": "green",
                "brightness": 0.0,
                "duration": 0.0,  # used for blink and pulse
                "timer": 0.0,  # internal for blink
            }
            for i in range(num_leds)
        }

    def set_on(self, idx: int, color: str, brightness: float = 0.5):
        self.states[idx] = {"effect": "on", "color": color, "brightness": brightness}

    def set_off(self, idx: int):
        # Preserve existing state structure, just change effect
        if idx in self.states:
            self.states[idx]["effect"] = "off"
        else:
            self.states[idx] = {"effect": "off"}

    def set_blink(self, idx: int, color: str, duration: float, brightness: float = 0.5):
        # If already blinking, preserve the timer to let the cycle continue naturally
        # Only update if color/brightness/duration actually changed
        state = self.states[idx]
        if state.get("effect") == "blink":
            # Only update if values changed to avoid unnecessary state updates
            if state.get("color") != color:
                state["color"] = color
            if state.get("brightness") != brightness:
                state["brightness"] = brightness
            if state.get("duration") != duration:
                # If duration changes, reset timer to maintain phase
                state["duration"] = duration
                state["timer"] = 0.0
        else:
            # Starting a new blink - initialize everything
            self.states[idx] = {
                "effect": "blink",
                "color": color,
                "brightness": brightness,
                "duration": duration,
                "timer": 0.0,
            }

    def set_pulse(self, idx: int, color: str, duration: float, brightness: float = 0.5):
        self.states[idx] = {
            "effect": "pulse",
            "color": color,
            "brightness": brightness,
            "duration": duration,
            "timer": duration,
        }

    def step(self, dt: float):
        for idx, state in self.states.items():
            effect = state["effect"]
            if effect == "off":
                self._turn_off(idx)
            elif effect == "on":
                self._turn_on(idx, state["color"], state["brightness"])
            elif effect == "blink":
                duration = state.get("duration", 1.0)
                if duration > 0:
                    state["timer"] = state.get("timer", 0.0) + dt
                    state["timer"] = state["timer"] % duration
                    phase = state["timer"] / duration
                    if phase < 0.5:
                        self._turn_on(idx, state.get("color", "blue"), state.get("brightness", 0.5))
                    else:
                        self._turn_off(idx)
                else:
                    # Invalid duration, just turn off
                    self._turn_off(idx)
            elif effect == "pulse":
                if state["timer"] > 0:
                    self._turn_on(idx, state["color"], state["brightness"])
                    state["timer"] -= dt
                else:
                    state["effect"] = "off"

    def _turn_on(self, idx: int, color: str, brightness: float):
        h, s = self.colors[color]
        self.leds.set_hsv(idx, h, s, brightness)

    def _turn_off(self, idx: int):
        self.leds.set_rgb(idx, 0, 0, 0, 0)
