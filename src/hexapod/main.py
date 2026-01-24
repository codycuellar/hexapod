"""
Hexapod main controller entry point.

This module provides the main control loop for the hexapod robot.
It handles gamepad input, motion planning, and serial communication with the Servo2040.
"""

import time
import logging
import serial
import platform

from hexapod.common.commands import CMD_SET_SERVO, CMD_PING, CMD_PONG, CMD_MESSAGE
from hexapod.common.serial_buffer import SerialBuffer, SerialPacket
from hexapod.gamepad import GamePad
from hexapod.rigid_body import Body, Leg, LegID, LegConfig
from hexapod.engine import Frame, Vec3d, Vec2d, Rotation
from hexapod.servos import Servo
from hexapod.motion_planner import MotionPlanner


logger = logging.getLogger(__name__)
logging.basicConfig(
    level=logging.DEBUG,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
)


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
                    # PING with zero length (encoded as high bit)
                    self.conn.write(bytes([CMD_PING, 0x80]))  # Length 0 encoded as 0x80
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
        if not self.conn:
            return False
        try:
            # Protocol: [CMD (ASCII), LEN (high bit), DATA... (high bit encoded)]
            # Encode length and data with high bits to avoid control chars
            buff = bytearray([CMD_SET_SERVO])
            # Encode length: 0x80 + length
            buff.append(0x80 | (len(servo_data) & 0x7F))
            # Encode data bytes: 0x80 + byte
            for byte in servo_data:
                buff.append(0x80 | (byte & 0x7F))
            self.conn.write(buff)
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


def create_hexapod() -> Body:
    """
    Create a hexapod with hardcoded servo calibration parameters.
    This matches the physical configuration of the hexapod hardware.
    """
    # Joint lengths (mm)
    COX_LEN = 40.0
    FEM_LEN = 65.0
    TIB_LEN = 90.0

    origin_frame = Frame()
    distance = Vec3d(100, 0, 0)

    ids = [LegID.RM, LegID.RF, LegID.LF, LegID.LM, LegID.LB, LegID.RB]
    servos = {
        LegID.RM: (
            Servo(6, -7.0, inverted=True),
            Servo(7, -30.0, inverted=False),
            Servo(8, 65.0, inverted=True),
        ),
        LegID.RF: (
            Servo(3, -6.0, inverted=True),
            Servo(4, -30.0, inverted=False),
            Servo(5, 67.0, inverted=True),
        ),
        LegID.LF: (
            Servo(0, 2.0, inverted=True),
            Servo(1, 24.0, inverted=True),
            Servo(2, -80.0, inverted=False),
        ),
        LegID.LM: (
            Servo(15, -4.0, inverted=True),
            Servo(16, 27.0, inverted=True),
            Servo(17, -85.0, inverted=False),
        ),
        LegID.LB: (
            Servo(12, -4.0, inverted=True),
            Servo(13, 31.0, inverted=True),
            Servo(14, -77.0, inverted=False),
        ),
        LegID.RB: (
            Servo(9, 0.0, inverted=True),
            Servo(10, -26.0, inverted=False),
            Servo(11, 73.0, inverted=True),
        ),
    }

    legs: dict[LegID, Leg] = {}

    # Position each leg 60 degrees apart around the origin
    for id in ids:
        mount_pos = origin_frame.local_pos_to_world(distance)
        frame = Frame(origin=mount_pos, rotation=origin_frame.rotation)
        origin_frame.rotate(Rotation.degrees(0.0, 0.0, 60.0))
        cox, fem, tib = servos[id]
        config = LegConfig(
            coxa=(COX_LEN, cox), femur=(FEM_LEN, fem), tibia=(TIB_LEN, tib)
        )
        legs[id] = Leg(id, frame, config)

    return Body(Frame(), legs)


def main():
    """Main control loop for hexapod operation."""
    running = True

    logger.info("Creating hexapod geometry...")
    body = create_hexapod()

    logger.info("Creating motion planner...")
    motion_planner = MotionPlanner(body, Vec3d(140, 0, -80))
    motion_planner.initialize()

    logger.info("Initializing gamepad...")
    gamepad = GamePad()
    gamepad.start_reading()

    # Setup serial communication with Servo2040
    hp_serial = HexapodSerial(timeout=10)

    DT = 1 / 50  # Control loop frequency: 50 Hz
    next_time = time.perf_counter()

    logger.info("Starting control loop...")

    try:
        while running:
            # Check for connection
            if not hp_serial.conn:
                if not hp_serial.connect():
                    # Still not connected, wait a bit and try again
                    time.sleep(1)
                    continue

            gait_vec = Vec2d()
            gait_turn = 0.0

            body_offset_trans = Vec3d()
            body_offset_rot = Vec3d()  # pitch, roll, yaw

            # LEFT STICK
            if gamepad.bumper_l:
                body_offset_trans = gamepad.joy_l.to_3d()
            else:
                gait_vec = gamepad.joy_l

            # RIGHT STICK
            if gamepad.bumper_l:
                body_offset_trans = Vec3d(
                    body_offset_trans.x, body_offset_trans.y, gamepad.joy_r.y
                )
            else:
                body_offset_rot = Vec3d(
                    -gamepad.joy_r.y, gamepad.joy_r.x, body_offset_rot.z
                )

            # TRIGGERS
            trigger_turn = gamepad.trigger_l - gamepad.trigger_r
            if gamepad.bumper_r:
                body_offset_rot = Vec3d(
                    body_offset_rot.x, body_offset_rot.y, trigger_turn
                )
            else:
                gait_turn = trigger_turn

            # Update motion planner
            motion_planner.update_gait(DT, gait_vec, gait_turn)
            motion_planner.offset_body(DT, body_offset_trans, body_offset_rot)
            motion_planner.step(DT)

            # Send servo commands to hardware
            servo_data = body.get_servo_command()
            if not hp_serial.send_servos(servo_data):
                logger.warning("Failed to send servo data, will attempt reconnect...")

            # Check for messages (logs, tracebacks) from Pico
            hp_serial.check_messages()

            # Maintain control loop timing
            next_time += DT
            sleep_time = next_time - time.perf_counter()
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        logger.info("Interrupted by user")
    finally:
        logger.info("Shutting down...")
        hp_serial.close()
        logger.info("Shutdown complete")


if __name__ == "__main__":
    main()
