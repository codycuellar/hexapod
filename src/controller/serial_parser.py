from servo import ServoCluster
from commands import *  # type: ignore

from utils import log_to_file, LogLevel


class CommandProcessor:
    """Processes commands and executes appropriate actions."""

    def __init__(self, servo_cluster: ServoCluster, output_stream):
        self.servo_cluster = servo_cluster
        self.output_stream = output_stream

    def dispatch(self, cmd_data):
        cmd = cmd_data["cmd"]
        data = cmd_data["data"]
        try:
            if cmd == CMD_PING:
                self.output_stream.buffer.write(bytes([CMD_PONG, 0x00]))
                self.output_stream.flush()
                return True

            elif cmd == CMD_SET_SERVO:
                servos = self.parse_set_servo(data)
                success = True
                for pin, angle in servos:
                    complete = self.servo_cluster.value(int(pin), float(angle))
                    if not complete:
                        success = False
                self.output_stream.buffer.write(
                    bytes([CMD_RESPONSE, ACK if success else NACK, 0x00])
                )
                return success

            else:
                # Unknown command
                return False

        except Exception as e:
            log_to_file(LogLevel.CRIT, e)
            try:
                msg = str(e).encode("utf-8")
                buff = bytearray([CMD_MESSAGE, len(msg)])
                buff.extend(msg)
                self.output_stream.buffer.write(buff)
                self.output_stream.flush()
            except Exception as _e:
                log_to_file(
                    LogLevel.CRIT,
                    "An error occurred attempting to send the previous error log to host {}".format(
                        _e
                    ),
                )

            return False

    def parse_set_servo(self, data: bytearray):
        servo_angles = []
        slice_len = 3

        for i in range(data[0]):
            pin_num = data[i * slice_len] & LOW_MASK
            value = data[i * slice_len + 1] & LOW_MASK
            value |= (data[i * slice_len + 2] & LOW_MASK) << 7
            # value comes in offset so that -90 is 0 and +90 is 180. It's also
            # multiplied by 10 to preserve one decimal of precision. Two decimals
            # exceeds the 14bit buffer and is unnecessarily precise.
            servo_angles.append((pin_num, float(value) / 10.0 - 90.0))

        return servo_angles
