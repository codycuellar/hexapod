import logging
import platform
import time
import serial

from hexapod.common.commands import *
from hexapod.common.serial_buffer import FRAME_START, SerialBuffer, SerialPacket

logger = logging.getLogger(__name__)


class HexapodSerial:
    def __init__(self, ports: list[str] | None = None, timeout: int = 5):
        self.ports = ports or self._default_ports()
        self.timeout = timeout
        self.conn = None
        self.serial_buffer = None
        self.last_error = None
        self.text_buffer = ""  # Accumulate text across multiple reads for tracebacks

    def connect(self):
        """Attempts to find and connect to the hexapod servo2040 board."""
        start_time = time.time()
        while time.time() - start_time < self.timeout:
            for port in self.ports:
                try:
                    logger.info(f"Scanning {port}...")
                    self.conn = serial.Serial(port, 115200, timeout=0.5)
                    self.serial_buffer = SerialBuffer()
                    self.text_buffer = ""  # Reset text buffer on new connection

                    # 1. Look for immediate irrecoverable crash
                    time.sleep(0.5)
                    packets, text = self._read_available()
                    if "Traceback" in text or "--- HEXAPOD_CRASH ---" in text:
                        logger.error(
                            f"CRITICAL: Device {port} crashed and likely cannot recover without software"
                            + " changes. Traceback:\n{text}"
                        )
                        self.close()
                        continue

                    # we will ping/pong for ready state
                    logger.debug(f"Sending PING to {port}")
                    # Send PING: !P00:\n
                    self._send_frame(CMD_PING, bytearray())
                    self.conn.flush()

                    time.sleep(0.2)
                    packets, _ = self._read_available()
                    for packet in packets:
                        if packet.cmd == CMD_PONG:
                            logger.info(
                                f"Received PONG, successfully conntected to {port}"
                            )
                            return True

                    self.close()
                except (serial.SerialException, OSError) as e:
                    if "Access is denied" in str(e):
                        logger.warning(
                            f"Port {port} is busy (is another program like Thonny or MicroPico open?)"
                        )
                    self.close()
                    continue
            time.sleep(1)
        return False

    def send_servos(self, servo_data: bytearray):
        """Send servo command using ASCII protocol: !S<len>:<hex_data>\n"""
        if not self.conn:
            return False
        try:
            self._send_frame(CMD_SET_SERVO, servo_data)
            self.conn.flush()
            return True
        except (serial.SerialException, OSError) as e:
            logger.error(f"Write error: {e}")
            self.close()
            return False

    def check_messages(self):
        """Checks for incoming messages or errors from the board."""
        packets, text = self._read_available()

        # Process binary packets
        for packet in packets:
            if packet.cmd == CMD_MESSAGE:
                msg = packet.data.decode("utf-8", errors="replace")
                logger.info(f"PICO: {msg}")
            else:
                logger.debug(f"PICO BINARY CMD: {packet.cmd:#02x}")

        # Accumulate text for tracebacks (they come in multiple chunks)
        if text:
            self.text_buffer += text
            # Check for complete traceback lines
            if "\n" in self.text_buffer:
                lines = self.text_buffer.split("\n")
                # Keep the last incomplete line in buffer, process complete lines
                self.text_buffer = lines[-1]
                complete_text = "\n".join(lines[:-1])

                # Process complete lines
                if "Traceback" in complete_text:
                    logger.error(f"PICO CRASHED:\n{complete_text}")
                    self.text_buffer = ""  # Clear buffer after logging
                elif "--- HEXAPOD_CRASH ---" in complete_text:
                    logger.error(f"PICO Reported a Fatal Crash:\n{complete_text}")
                    self.text_buffer = ""
                elif "--- HEXAPOD_READY ---" in complete_text:
                    logger.info("PICO Rebooted (Ready marker received).")
                    self.text_buffer = ""

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
        if not self.conn:
            return

        frame = bytearray([FRAME_START, cmd])
        frame.extend(SerialBuffer.byte_to_hex_bytes(len(data)))
        for char_byte in data:
            frame.extend(SerialBuffer.byte_to_hex_bytes(char_byte))
        self.conn.write(bytes(frame))

    @staticmethod
    def _int_to_hex(value: int) -> int:
        """Convert 0-15 to ASCII hex character."""
        if 0 <= value <= 9:
            return ord("0") + value
        else:
            return ord("A") + (value - 10)

    def _read_available(self) -> tuple[list[SerialPacket], str]:
        """Reads all currently available data and returns (packets, text)."""
        if not self.conn or not self.serial_buffer:
            return [], ""

        packets: list[SerialPacket] = []
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
