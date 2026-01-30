try:
    from typing import TextIO
except:
    pass

from servo import ServoCluster
from commands import *
from serial_buffer import SerialPacket, SerialBuffer, FRAME_START
from utils import log_to_file, LogLevel


class CommandProcessor:
    def __init__(self, servo_cluster: ServoCluster, output_stream: "TextIO"):
        self.servo_cluster = servo_cluster
        self.output_stream = output_stream

    def dispatch(self, packet: SerialPacket):
        cmd = packet.cmd
        data = packet.data

        try:
            if cmd == CMD_PING:
                log_to_file(LogLevel.INFO, "Received PING")
                self._send_frame(CMD_PONG, bytes())
                return True

            elif cmd == CMD_SET_SERVO:
                servos = self._parse_servo_data(data)
                for pin, angle in servos:
                    self.servo_cluster.value(pin, float(angle))

                self._send_frame(CMD_RESPONSE, bytes([ACK]))
                return True

            else:
                self._send_frame(
                    CMD_MESSAGE,
                    "Unknown command: 0x{:2X}".format(hex(cmd)).encode("UTF-8"),
                )
                log_to_file(LogLevel.WARN, "Unknown command: {}".format(chr(cmd)))
                return False

        except Exception as e:
            log_to_file(LogLevel.CRIT, "Exception in dispatch: {}".format(e))
            try:
                # Send error message as UTF-8 text
                msg = str(e).encode("utf-8")
                self._send_frame(CMD_MESSAGE, msg)
            except Exception:
                pass
            return False

    def _parse_servo_data(self, data: bytearray):
        if len(data) < 5:
            raise Exception("Servo data too short")

        count = data[0]

        count_bytes = 1
        # Each servo data is 24b (3B)
        servo_bytes = 3
        expected_bytes = count_bytes + count * servo_bytes
        if len(data) != expected_bytes:
            raise Exception(
                "Invalid servo payload length: expected {} hex chars, got {}".format(
                    expected_bytes, len(data)
                )
            )

        servos = []
        for i in range(count):
            offset = count_bytes + i * servo_bytes  # Skip count, then 6 chars per servo
            pin = data[offset]
            # Convert from wire format: (raw / 10.0) - 90.0
            angle_raw = (data[offset + 1] << 8) | data[offset + 2]
            angle = float(angle_raw) / 10.0 - 90.0
            servos.append((pin, angle))

        return servos

    def _send_frame(self, cmd: int, data: bytes):
        """
        Send an ASCII protocol frame: !<CMD><LEN><DATA>

        Args:
            cmd: Command byte (ASCII character)
            data: Raw data bytes (will be hex-encoded automatically)
        """
        frame = SerialBuffer.build_frame(cmd, data)
        self.output_stream.buffer.write(frame)
