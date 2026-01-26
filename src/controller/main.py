import gc
import usys
import utime
import uselect

from servo import ServoCluster, servo2040

from serial_buffer import SerialBuffer
from command_processor import CommandProcessor
from led_manager import LedManager
from utils import log_to_file, clear_log, LogLevel


def main():
    log_to_file(LogLevel.INFO, "Servo2040 Booting Up...")

    clear_log()
    gc.collect()

    # Use ONE cluster to save all hardware resources
    pins = list(range(servo2040.SERVO_1, servo2040.SERVO_18 + 1))
    log_to_file(LogLevel.INFO, "Initializing ServoCluster with pins: {}".format(pins))
    cluster = ServoCluster(0, 0, pins)

    log_to_file(LogLevel.INFO, "Initializing LEDs")
    led = LedManager(servo2040.NUM_LEDS, 0, 1)
    for i in range(6):
        led.set_off(i)

    log_to_file(LogLevel.INFO, "Initializing serial buffer")
    spoll = uselect.poll()
    spoll.register(usys.stdin, uselect.POLLIN)
    serial_buffer = SerialBuffer()
    cmd_processor = CommandProcessor(cluster, usys.stdout)

    last_t = utime.ticks_ms()
    last_rx = utime.ticks_ms()

    led.set_on(0, "blue", 0.25)
    log_to_file(LogLevel.INFO, "Entering main loop")
    try:
        while True:
            now = utime.ticks_ms()
            dt = utime.ticks_diff(now, last_t) / 1000.0
            last_t = now

            led.step(dt)

            if spoll.poll(0):
                while True:  # read all availble bytes
                    byte_data = usys.stdin.buffer.read(1)
                    if not byte_data or len(byte_data) == 0:
                        break
                    byte = byte_data[0]

                    try:
                        packet = serial_buffer.feed(byte)
                        if packet:
                            last_rx = now
                            led.set_blink(1, "green", 1.0, 0.2)

                            success = cmd_processor.dispatch(packet)
                            if not success:
                                led.set_pulse(5, "red", 5.0, 1.0)
                            break
                    except Exception as e:
                        log_to_file(LogLevel.WARN, e)
                        break

            # Update LED status based on time since last RX
            if utime.ticks_diff(now, last_rx) > 1000:
                led.set_off(1)
    except Exception as e:
        log_to_file(LogLevel.FATAL, e)
    finally:
        for i in range(6):
            led.set_off(i)


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        # Standard MicroPython traceback will still go to stdout/serial
        import io

        buf = io.StringIO()
        usys.print_exception(e, buf)  # type: ignore | this exists in micropython
        error_str = buf.getvalue()

        print("\n--- HEXAPOD_CRASH ---")
        print(error_str)
        log_to_file(LogLevel.FATAL, error_str)

        # Keep the REPL alive but visible
        while True:
            utime.sleep(1)
