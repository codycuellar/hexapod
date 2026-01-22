import gc
import sys
import time
import uselect
from servo import ServoCluster, servo2040
from plasma import WS2812
from servo import servo2040


STATUS_COLORS = {}


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
        self.states[idx] = {"effect": "off"}

    def set_blink(self, idx: int, color: str, duration: float, brightness: float = 0.5):
        if self.states[idx]["effect"] != "blink":
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
                state["timer"] += dt
                state["timer"] = state["timer"] % state["duration"]
                phase = state["timer"] / state["duration"]
                if phase < 0.5:
                    self._turn_on(idx, state["color"], state["brightness"])
                else:
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


def main():
    gc.collect()

    # Use ONE cluster to save all hardware resources
    pins = list(range(servo2040.SERVO_1, servo2040.SERVO_18 + 1))
    cluster = ServoCluster(0, 0, pins)

    spoll = uselect.poll()
    spoll.register(sys.stdin, uselect.POLLIN)

    led = LedManager(servo2040.NUM_LEDS, 0, 1)

    log_to_file("Servo2040 Booting Up...")
    last_t = time.ticks_ms()
    last_rx = time.ticks_ms()

    led.set_on(0, "green")
    try:
        while True:
            now = time.ticks_ms()
            dt = time.ticks_diff(now, last_t) / 1000.0
            last_t = now

            led.step(dt)

            if time.ticks_diff(now, last_rx) > 1000:
                led.set_off(3)

            if spoll.poll(0):
                command = sys.stdin.readline().strip()
                last_rx = now

                led.set_blink(3, "blue", 1.0)

                if not command:
                    continue
                elif command == "PING":
                    led.set_pulse(2, "orange", 2.0)
                    print("PONG")
                    continue

                try:
                    angles_raw = command.split(",")
                    for angle_raw in angles_raw:
                        if ":" not in angle_raw:
                            continue
                        pin, angle = angle_raw.split(":")
                        cluster.value(int(pin), float(angle))
                except Exception as e:
                    led.set_pulse(5, "red", 5.0, 1.0)
                    print("RUNTIME_ERROR:", e)
                    log_to_file("Runtime: " + str(e))
    finally:
        for i in range(6):
            led.set_off(i)


def log_to_file(error_msg):
    try:
        with open("log.txt", "a") as f:  # "a" for append so you don't lose old logs
            f.write(str(error_msg) + "\n")
    except:
        # If we can't write to file (e.g. no memory), at least we tried
        pass


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        # This catches "Fatal" crashes that stop the whole script
        print("FATAL_ERROR:", e)
        log_to_file("Fatal: " + str(e))
