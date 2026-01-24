import gc
import sys
import time
import uselect

from servo import ServoCluster, servo2040

from serial_buffer import SerialBuffer
from serial_parser import CommandProcessor
from led_manager import LedManager
from utils import log_to_file, LogLevel


def main():
    gc.collect()
    # Print a clear start marker for the host to see
    print("\n--- HEXAPOD_READY ---")
    log_to_file(LogLevel.INFO, "Servo2040 Booting Up...")

    # Use ONE cluster to save all hardware resources
    pins = list(range(servo2040.SERVO_1, servo2040.SERVO_18 + 1))
    log_to_file(LogLevel.INFO, "Initializing ServoCluster with pins: {}".format(pins))
    cluster = ServoCluster(0, 0, pins)

    log_to_file(LogLevel.INFO, "Initializing LEDs")
    led = LedManager(servo2040.NUM_LEDS, 0, 1)

    log_to_file(LogLevel.INFO, "Initializing serial buffer")
    spoll = uselect.poll()
    spoll.register(sys.stdin, uselect.POLLIN)
    serial_buffer = SerialBuffer(sys.stdin.buffer)
    cmd_processor = CommandProcessor(cluster, sys.stdout)

    last_t = time.ticks_ms()
    last_rx = time.ticks_ms()

    led.set_on(0, "green")
    try:
        while True:
            now = time.ticks_ms()
            dt = time.ticks_diff(now, last_t) / 1000.0
            last_t = now

            led.step(dt)

            # Read available bytes from serial
            if spoll.poll(0):
                data = serial_buffer.read_bytes()
                if data:
                    last_rx = now
                    led.set_blink(1, "blue", 1.0)

                success = cmd_processor.dispatch(data)
                if not success:
                    led.set_pulse(5, "red", 5.0, 1.0)

            # Update LED status based on time since last RX
            if time.ticks_diff(now, last_rx) > 1000:
                led.set_off(1)
    finally:
        for i in range(6):
            led.set_off(i)


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        # Standard MicroPython traceback will still go to stdout/serial
        import sys
        import io
        buf = io.StringIO()
        sys.print_exception(e, buf)
        error_str = buf.getvalue()

        print("\n--- HEXAPOD_CRASH ---")
        print(error_str)
        log_to_file(LogLevel.FATAL, error_str)

        # Keep the REPL alive but visible
        while True:
            time.sleep(1)
