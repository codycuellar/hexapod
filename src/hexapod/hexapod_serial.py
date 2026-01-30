import logging
import platform
import time
import serial

from hexapod.common.commands import *
from hexapod.common.serial_buffer import SerialBuffer, SerialPacket
from hexapod.servos import ServoAngles

logger = logging.getLogger(__name__)


class CommandError(Exception):
    """Raised when a command fails (NACK received)."""

    pass


class HexapodSerial:
    def __init__(self, timeout: float = 20.0):
        system = platform.system()
        if system == "Windows":
            self.ports = [f"COM{i}" for i in range(3, 8)]
        else:
            self.ports = ["/dev/ttyACM0", "/dev/ttyUSB0", "/dev/ttyUSB1"]
        self.timeout = timeout
        self.conn = None
        self.serial_buffer = None
        self.text_buffer = ""
        self.last_servo_angles: dict[int, float] = {}

    def connect(self):
        """Attempts to find and connect to the hexapod servo2040 board."""
        start_time = time.time()
        while time.time() - start_time < self.timeout:
            for port in self.ports:
                try:
                    self._connect_to_port(port)
                    return True
                except (serial.SerialException, OSError) as e:
                    logger.error(str(e))
                    self.close()
                    continue
            time.sleep(1)
        return False

    def close(self):
        if self.conn:
            try:
                self.conn.close()
            except:
                pass
            self.conn = None
        self.serial_buffer = None
        self.text_buffer = ""

    def send_servos(self, servos: ServoAngles):
        if not self.conn:
            return False

        new_servos: dict[int, float] = {}
        try:
            # Encode servo data: [count, pin, angle_high, angle_low, ...]
            data_bytes = bytearray()
            for pin, angle in servos.items():
                # check if the servo angle is new since the last frame, angle cannot be 999
                # so we use that as a default incase the servo wasn't update last frame.
                new_angle = round(angle, 1)
                if round(self.last_servo_angles.get(pin, 999), 1) != new_angle:
                    self.last_servo_angles[pin] = new_angle
                    new_servos[pin] = new_angle
                    # Encode angles as positive value int with 1 decimal precision:
                    # -90.0 to 90.0 becomes 0-1800
                    angle_raw = int((new_angle + 90.0) * 10.0)
                    data_bytes.append(pin)
                    data_bytes.extend(SerialBuffer.u16_to_bytes(angle_raw))

            # do nothing if we have no new servos to update
            num_servos_to_update = len(new_servos.keys())
            if num_servos_to_update == 0:
                return True

            logger.debug("Servos to update: %s", new_servos)

            # add the count of servos we're sending
            data_bytes.insert(0, num_servos_to_update)
            t0 = time.perf_counter()
            self._send_frame(CMD_SET_SERVO, data_bytes)
            write_ms = (time.perf_counter() - t0) * 1000
            if write_ms > 5.0:
                frame_bytes = 2 + 4 + 2 * len(data_bytes)  # ! CMD, 4-char LEN, hex DATA
                logger.debug(
                    "serial.write took %.1f ms (%d bytes)", write_ms, frame_bytes
                )

            # Check for ACK/NACK response
            packet = self._wait_for_response(CMD_RESPONSE)
            if packet:
                if packet.data[0] == NACK:
                    raise CommandError("Servo command rejected by device (NACK)")
                return True
            else:
                return False
        except (serial.SerialException, OSError) as e:
            logger.error(f"Write error: {e}")
            self.close()
            return False

    def _send_frame(self, cmd: int, data: bytearray):
        """Send ASCII protocol frame."""
        if not self.conn:
            return
        frame = SerialBuffer.build_frame(cmd, data)
        logger.debug("writing frame: %s", frame.decode("ASCII"))
        self.conn.write(frame)
        self.conn.flush()

    def _wait_for_response(self, cmd: int, timeout: float = 0.1):
        end_time = time.time() + timeout
        while time.time() < end_time:
            packet = self._read_available()
            if packet and packet.cmd == cmd:
                return packet
        logger.error(f"Did not receive a servo command response after {timeout}s")

    def _read_available(self) -> SerialPacket | None:
        """Read available data and return (packets, text)."""
        if not self.conn or not self.serial_buffer:
            return

        while True:
            if self.conn.in_waiting == 0:
                return
            byte = self.conn.read(1)
            packet = self.serial_buffer.feed(byte[0])
            if packet and packet.cmd == CMD_MESSAGE:
                logger.info("Message from Servo2040: %s", packet.data.decode("utf-8"))
                return

            if packet:
                return packet

    def _connect_to_port(self, port: str):
        """
        Attempt to connect to the given port and verify the connection by sending
        a PING and waiting for a PONG. If the connection is not established within
        the timeout, raise an exception.

        Args:
            port: The port to connect to

        Returns:
            True if the connection is established, otherwise raises an exception.

        Raises:
            serial.SerialException: If the connection is not established within the timeout period.
        """
        self.conn = serial.Serial(port, 115200, timeout=0.5)
        logger.info(f"Scanning {port}...")
        self.serial_buffer = SerialBuffer()
        self.text_buffer = ""

        logger.debug(f"Sending PING to {port}")
        self._send_frame(CMD_PING, bytearray())

        timeout = 1.0
        packet = self._wait_for_response(CMD_PONG, timeout)
        if packet and packet.cmd == CMD_PONG:
            logger.info(f"Connected to {port}")
            return

        raise serial.SerialException(f"Could not establish connection in {timeout}s.")
