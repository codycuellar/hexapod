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

    def _parse_servo_data(self, data: bytes):
        """
        Parse hex-encoded servo command data.

        Format: [count_hex_pair, pin_hex_pair, angle_high_hex_pair, angle_low_hex_pair, ...]
        Each servo takes 6 hex characters (3 bytes decoded).
        """
        if len(data) < 2:
            raise Exception("Servo data too short")

        # Decode count (first 2 hex chars)
        count = SerialBuffer.decode_hex_pair(data[0], data[1])

        # Each servo is 6 hex chars (3 decoded bytes: pin, angle_high, angle_low)
        expected_hex_chars = 2 + count * 6
        if len(data) != expected_hex_chars:
            raise Exception("Invalid servo payload length: expected {} hex chars, got {}".format(
                expected_hex_chars, len(data)
            ))

        servos = []
        for i in range(count):
            offset = 2 + i * 6  # Skip count, then 6 chars per servo
            pin = SerialBuffer.decode_hex_pair(data[offset], data[offset + 1])
            high = SerialBuffer.decode_hex_pair(data[offset + 2], data[offset + 3])
            low = SerialBuffer.decode_hex_pair(data[offset + 4], data[offset + 5])

            # Convert from wire format: (raw / 10.0) - 90.0
            angle_raw = (high << 8) | low
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
