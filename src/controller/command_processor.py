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
                msg = str(e).encode("utf-8")
                char_byte = bytearray()
                for byte in msg:
                    char_byte.extend(SerialBuffer.byte_to_hex_bytes(byte))
                self._send_frame(CMD_MESSAGE, bytes(char_byte))
            except Exception:
                pass
            return False

    def _parse_servo_data(self, data: bytes):
        count = SerialBuffer.hex_bytes_to_byte(data[0], data[1])
        offset = 2
        expected = offset + count * 6
        if len(data) != expected:
            raise Exception("Invalid servo payload length")

        servos = []
        for i in range(count):
            j = i * 6 + offset
            pin = SerialBuffer.hex_bytes_to_byte(data[j], data[j + 1])
            high = SerialBuffer.hex_bytes_to_byte(data[j + 2], data[j + 3])
            low = SerialBuffer.hex_bytes_to_byte(data[j + 4], data[j + 5])
            angle = float(high << 8 | low)
            angle = angle / 10.0 - 90.0
            servos.append((pin, angle))
        return servos

    def _send_frame(self, cmd: int, data: bytes):
        """
        Send an ASCII protocol frame: !<CMD><LEN>:<DATA>\n
        """
        # Build frame
        frame = bytearray([FRAME_START, cmd])
        frame.extend(SerialBuffer.byte_to_hex_bytes(len(data)))
        frame.extend(data)

        self.output_stream.buffer.write(frame)
        self.output_stream.flush()
