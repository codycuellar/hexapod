"""
Hexapod main controller entry point.

This module provides the main control loop for the hexapod robot.
It handles gamepad input, motion planning, and serial communication with the Servo2040.
"""

import time
import logging
import serial
import signal
import platform

from hexapod.common.commands import CMD_SET_SERVO, CMD_PING, CMD_PONG, CMD_MESSAGE
# from hexapod.common.serial_buffer import SerialBuffer
from hexapod.gamepad import GamePad
from hexapod.rigid_body import Body, Leg, LegID, LegConfig
from hexapod.engine import Frame, Vec3d, Vec2d, Rotation
from hexapod.servos import Servo
from hexapod.motion_planner import MotionPlanner


logger = logging.getLogger(__name__)
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
)


class HexapodSerial:
    def __init__(self, ports=None, timeout=5):
        self.ports = ports or self._default_ports()
        self.timeout = timeout
        self.conn = None
        self.last_error = None

    def _default_ports(self):
        system = platform.system()
        if system == "Windows":
            return [f"COM{i}" for i in range(3, 15)]
        else:
            return ["/dev/ttyACM0", "/dev/ttyUSB0", "/dev/ttyUSB1"]

    def connect(self):
        """Attempts to find and connect to the hexapod board."""
        start_time = time.time()
        while time.time() - start_time < self.timeout:
            for port in self.ports:
                try:
                    logger.info(f"Scanning {port}...")
                    s = serial.Serial(port, 115200, timeout=0.5)

                    # 1. Listen for a moment to see if it's already talking (traceback or READY)
                    time.sleep(0.5)
                    if s.in_waiting:
                        data = s.read(s.in_waiting).decode("utf-8", errors="replace")
                        if "Traceback" in data or "--- HEXAPOD_CRASH ---" in data:
                            logger.error(f"CRITICAL: Found traceback on {port}:\n{data}")
                            s.close()
                            continue
                        if "--- HEXAPOD_READY ---" in data:
                            logger.info(f"Found ready board on {port}")
                            self.conn = s
                            return True

                    # 2. Try to PING
                    logger.debug(f"Sending PING to {port}")
                    s.write(bytes([CMD_PING, 0x00]))
                    s.flush()

                    time.sleep(0.2)
                    if s.in_waiting:
                        response = s.read(s.in_waiting)
                        # Protocol check: [CMD_PONG, 0x00]
                        if len(response) >= 2 and response[0] == CMD_PONG:
                            logger.info(f"Connected to Hexapod on {port}")
                            self.conn = s
                            return True

                    s.close()
                except (serial.SerialException, OSError) as e:
                    if "Access is denied" in str(e):
                        logger.warning(f"Port {port} is busy (is another program like Thonny or MicroPico open?)")
                    continue
            time.sleep(1)
        return False

    def send_servos(self, servo_data):
        if not self.conn:
            return False
        try:
            # Protocol: [CMD, LEN, DATA...]
            buff = bytearray([CMD_SET_SERVO, len(servo_data)])
            buff.extend(servo_data)
            self.conn.write(buff)
            self.conn.flush()
            return True
        except (serial.SerialException, OSError) as e:
            logger.error(f"Write error: {e}")
            self.close()
            return False

    def check_messages(self):
        """Checks for incoming messages or errors from the board."""
        if not self.conn or self.conn.in_waiting == 0:
            return

        try:
            # This is a simple read - in a full implementation we'd use SerialBuffer
            data = self.conn.read(self.conn.in_waiting)

            # Check for binary protocol messages
            i = 0
            while i < len(data):
                cmd = data[i]
                if cmd == CMD_MESSAGE and i + 1 < len(data):
                    length = data[i+1]
                    msg = data[i+2 : i+2+length].decode("utf-8", errors="replace")
                    logger.info(f"PICO: {msg}")
                    i += 2 + length
                elif cmd == 0x0A or cmd == 0x0D: # Newlines/CR
                    i += 1
                else:
                    # Might be raw text (traceback)
                    text = data[i:].decode("utf-8", errors="replace")
                    if "Traceback" in text:
                        logger.error(f"PICO CRASHED:\n{text}")
                    else:
                        logger.debug(f"PICO RAW: {text}")
                    break
        except Exception as e:
            logger.error(f"Error reading messages: {e}")

    def close(self):
        if self.conn:
            try:
                self.conn.close()
            except:
                pass
            self.conn = None


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
        gamepad.stop_reading()
        logger.info("Shutdown complete")


if __name__ == "__main__":
    main()
