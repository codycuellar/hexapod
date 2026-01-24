from servo import ServoCluster
from commands import *  # type: ignore

from utils import log_to_file, LogLevel


class CommandProcessor:
    """Processes commands and executes appropriate actions."""

    def __init__(self, servo_cluster: ServoCluster, output_stream):
        self.servo_cluster = servo_cluster
        self.output_stream = output_stream

    def dispatch(self, packet):
        cmd = packet.cmd
        data = packet.data
        try:
            if cmd == CMD_PING:
                log_to_file(LogLevel.INFO, "received PING")
                # PONG with zero length (encoded as high bit)
                self.output_stream.buffer.write(bytes([CMD_PONG, 0x80]))  # Length 0 encoded as 0x80
                # Flush may not exist on all MicroPython stdout implementations
                try:
                    self.output_stream.flush()
                except AttributeError:
                    pass  # Some MicroPython implementations don't have flush
                return True

            elif cmd == CMD_SET_SERVO:
                servos = self.parse_set_servo(data)
                success = True
                for pin, angle in servos:
                    pin_int = int(pin)
                    # Validate pin number before calling servo cluster
                    if pin_int < 0 or pin_int > 17:
                        log_to_file(LogLevel.CRIT, "Invalid servo pin: {} (must be 0-17)".format(pin_int))
                        success = False
                        continue
                    complete = self.servo_cluster.value(pin_int, float(angle))
                    if not complete:
                        success = False
                # Response: [CMD, ACK/NACK (ASCII), length (high bit)]
                self.output_stream.buffer.write(
                    bytes([CMD_RESPONSE, ACK if success else NACK, 0x80])  # Length 0 encoded as 0x80
                )
                try:
                    self.output_stream.flush()
                except AttributeError:
                    pass
                return success

            else:
                # Unknown command
                return False

        except Exception as e:
            log_to_file(LogLevel.CRIT, e)
            try:
                msg = str(e).encode("utf-8")
                # Encode message: [CMD, length (high bit), data... (high bit encoded)]
                buff = bytearray([CMD_MESSAGE, 0x80 | (len(msg) & 0x7F)])
                for byte in msg:
                    buff.append(0x80 | (byte & 0x7F))
                self.output_stream.buffer.write(buff)
                try:
                    self.output_stream.buffer.flush()
                except AttributeError:
                    pass
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

        if len(data) == 0:
            log_to_file(LogLevel.CRIT, "parse_set_servo: empty data")
            return servo_angles

        num_servos = data[0]
        if len(data) < 1 + num_servos * slice_len:
            log_to_file(LogLevel.CRIT, "parse_set_servo: data too short. Expected {} bytes, got {}".format(
                1 + num_servos * slice_len, len(data)))
            return servo_angles

        for i in range(num_servos):
            idx = 1 + i * slice_len
            if idx + 2 >= len(data):
                log_to_file(LogLevel.CRIT, "parse_set_servo: index out of range at servo {}".format(i))
                break
            pin_num = data[idx] & LOW_MASK
            value = data[idx + 1] & LOW_MASK
            value |= (data[idx + 2] & LOW_MASK) << 7
            # value comes in offset so that -90 is 0 and +90 is 180. It's also
            # multiplied by 10 to preserve one decimal of precision. Two decimals
            # exceeds the 14bit buffer and is unnecessarily precise.
            servo_angles.append((pin_num, float(value) / 10.0 - 90.0))

        return servo_angles
