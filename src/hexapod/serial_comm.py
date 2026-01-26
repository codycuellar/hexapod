import logging
import platform
import time
import serial

from hexapod.common.commands import *
from hexapod.common.serial_buffer import SerialBuffer, SerialPacket


logger = logging.getLogger(__name__)


class CommandError(Exception):
    """Raised when a command fails (NACK received)."""

    pass


class HexapodSerial:
    def __init__(self, ports: list[str] | None = None, timeout: int = 5):
        self.ports = ports or self._default_ports()
        self.timeout = timeout
        self.conn = None
        self.serial_buffer = None
        self.text_buffer = ""

    def connect(self):
        """Attempts to find and connect to the hexapod servo2040 board."""
        start_time = time.time()
        while time.time() - start_time < self.timeout:
            for port in self.ports:
                try:
                    logger.info(f"Scanning {port}...")
                    self.conn = serial.Serial(port, 115200, timeout=0.5)
                    self.serial_buffer = SerialBuffer()
                    self.text_buffer = ""

                    time.sleep(0.5)
                    logger.info("reading available bytes")
                    packets, text = self._read_available()
                    logger.info(
                        "read packets and info: len of packets ({}), text: {}".format(
                            len(packets), text
                        )
                    )
                    if "Traceback" in text or "--- HEXAPOD_CRASH ---" in text:
                        logger.error(
                            f"CRITICAL: Device {port} crashed. Traceback:\n{text}"
                        )
                        self.close()
                        continue

                    logger.debug(f"Sending PING to {port}")
                    self._send_frame(CMD_PING, bytearray())
                    self.conn.flush()

                    time.sleep(0.2)
                    packets, _ = self._read_available()
                    for packet in packets:
                        if packet.cmd == CMD_PONG:
                            logger.info(f"Connected to {port}")
                            return True

                    self.close()
                except (serial.SerialException, OSError) as e:
                    if "Access is denied" in str(e):
                        logger.warning(f"Port {port} is busy")
                    self.close()
                    continue
            time.sleep(1)
        return False

    def send_servos(self, servos: list[tuple[int, float]]):
        """
        Send servo command.

        Args:
            servos: List of (pin, angle_degrees) tuples

        Returns:
            True if sent successfully, False otherwise

        Raises:
            CommandError: If NACK received (command failed on device)
        """
        if not self.conn:
            return False

        try:
            # Encode servo data: [count, pin, angle_high, angle_low, ...]
            data = bytearray([len(servos)])
            for pin, angle in servos:
                # Encode angle: (angle + 90) * 10 maps -90°→0, 0°→900, +90°→1800
                angle_raw = int((angle + 90.0) * 10.0)
                data.extend([pin, (angle_raw >> 8) & 0xFF, angle_raw & 0xFF])

            self._send_frame(CMD_SET_SERVO, data)
            self.conn.flush()

            # Check for ACK/NACK response
            time.sleep(0.01)  # Brief wait for response
            packets, _ = self._read_available()
            for packet in packets:
                if packet.cmd == CMD_RESPONSE:
                    if len(packet.data) >= 2:
                        code = SerialBuffer.decode_hex_pair(
                            packet.data[0], packet.data[1]
                        )
                        if code == NACK:
                            raise CommandError(
                                "Servo command rejected by device (NACK)"
                            )
                        # ACK is success, no exception

            return True
        except CommandError:
            raise
        except (serial.SerialException, OSError) as e:
            logger.error(f"Write error: {e}")
            self.close()
            return False

    def check_messages(self) -> list[str]:
        """
        Check for incoming messages from the board.

        Returns:
            List of message strings (empty if none)
        """
        packets, text = self._read_available()
        messages = []

        # Process protocol packets
        for packet in packets:
            if packet.cmd == CMD_MESSAGE:
                try:
                    raw_bytes = SerialBuffer.decode_hex_string(packet.data)
                    msg = raw_bytes.decode("utf-8", errors="replace")
                    messages.append(msg)
                    logger.info(f"PICO: {msg}")
                except Exception as e:
                    logger.warning(f"Failed to decode message: {e}")

        # Process text tracebacks
        if text:
            self.text_buffer += text
            if "\n" in self.text_buffer:
                lines = self.text_buffer.split("\n")
                self.text_buffer = lines[-1]
                complete_text = "\n".join(lines[:-1])

                if "Traceback" in complete_text:
                    logger.error(f"PICO CRASHED:\n{complete_text}")
                elif "--- HEXAPOD_CRASH ---" in complete_text:
                    logger.error(f"PICO FATAL CRASH:\n{complete_text}")
                elif "--- HEXAPOD_READY ---" in complete_text:
                    logger.info("PICO Rebooted")

        return messages

    def close(self):
        if self.conn:
            try:
                self.conn.close()
            except:
                pass
            self.conn = None
        self.serial_buffer = None
        self.text_buffer = ""

    def _send_frame(self, cmd: int, data: bytearray):
        """Send ASCII protocol frame."""
        if not self.conn:
            return
        frame = SerialBuffer.build_frame(cmd, data)
        self.conn.write(frame)

    def _read_available(self) -> tuple[list[SerialPacket], str]:
        """Read available data and return (packets, text)."""
        if not self.conn or not self.serial_buffer:
            return [], ""

        packets = []
        raw_data = b""

        if self.conn.in_waiting > 0:
            raw_data = self.conn.read(self.conn.in_waiting)
            for byte in raw_data:
                packet = self.serial_buffer.feed(byte)
                if packet:
                    packets.append(packet)

        text = raw_data.decode("utf-8", errors="replace")
        return packets, text

    def _default_ports(self):
        system = platform.system()
        if system == "Windows":
            return [f"COM{i}" for i in range(3, 8)]
        else:
            return ["/dev/ttyACM0", "/dev/ttyUSB0", "/dev/ttyUSB1"]
