import gc
import usys
import utime
import uselect

from servo import ServoCluster, servo2040

from serial_buffer import SerialBuffer
from command_processor import CommandProcessor
from led_manager import LedManager, LEDBlink, LEDPulse, LEDSolid, LEDOff, LEDColor
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
    receive_data_led_timeout = 1000 # ms
    led_idle_time = 1000 * 60
    LED_RUNNING = (0, LEDSolid(LEDColor("blue", 1.0, 0.25)))
    LED_IDLE = (0, LEDSolid(LEDColor("yellow", 1.0, 0.2)))
    LED_RCV_DATA = (1, LEDBlink(LEDColor("green", 1.0, 0.2), 0.3))
    LED_NO_DATA = (1, LEDOff())
    LED_ERROR = (2, LEDPulse(LEDColor("red", 1.0, 1.0), 3.0))
    led_manager = LedManager(1, 0)

    log_to_file(LogLevel.INFO, "Initializing serial buffer")
    spoll = uselect.poll()
    spoll.register(usys.stdin, uselect.POLLIN)
    serial_buffer = SerialBuffer()
    cmd_processor = CommandProcessor(cluster, usys.stdout)

    last_rx = utime.ticks_ms()

    led_manager.set_effect(*LED_RUNNING)
    log_to_file(LogLevel.INFO, "Entering main loop")

    short_idle = True
    long_idle = True
    try:
        while True:
            now = utime.ticks_ms()

            led_manager.step(now)

            if spoll.poll(0):
                while True:  # read all availble bytes
                    byte_data = usys.stdin.buffer.read(1)
                    if not byte_data or len(byte_data) == 0:
                        break
                    byte = byte_data[0]

                    try:
                        packet = serial_buffer.feed(byte)
                        if packet:
                            short_idle = False
                            long_idle = False
                            last_rx = now
                            led_manager.set_effect(*LED_RCV_DATA)

                            success = cmd_processor.dispatch(packet)
                            if not success:
                                led_manager.set_effect(*LED_ERROR)
                            break
                    except Exception as e:
                        led_manager.set_effect(*LED_ERROR)
                        log_to_file(LogLevel.WARN, e)
                        break

            # Update LED status based on time since last RX
            diff = utime.ticks_diff(now, last_rx)
            if not short_idle and diff > receive_data_led_timeout:
                short_idle = True
                log_to_file(LogLevel.INFO, "Have not received data in {}ms".format(diff))
                led_manager.set_effect(*LED_NO_DATA)
            if not long_idle and diff > led_idle_time:
                long_idle = True
                log_to_file(LogLevel.INFO, "Have not received data in {}ms, going idle".format(diff))
                led_manager.set_effect(*LED_IDLE)
    except Exception as e:
        log_to_file(LogLevel.FATAL, e)
    finally:
        for i in range(6):
            led_manager.set_effect(i, LEDOff())


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
