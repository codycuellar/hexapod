"""
Hexapod main controller entry point.

This module provides the main control loop for the hexapod robot.
It handles gamepad input, motion planning, and serial communication with the Servo2040.
"""

import time
import logging
import serial
import signal

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
    frames: dict[LegID, Frame] = {}

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


def find_serial_port():
    """
    Attempt to find the Servo2040 serial port.
    On Windows: typically COM3, COM4, etc.
    On Linux: typically /dev/ttyACM0, /dev/ttyUSB0, etc.
    """
    import platform

    system = platform.system()
    if system == "Windows":
        # Try common COM ports
        for port_num in range(3, 10):
            port = f"COM{port_num}"
            try:
                test_serial = serial.Serial(port, 115200, timeout=0.1)
                test_serial.close()
                return port
            except (serial.SerialException, OSError):
                continue
    elif system == "Linux":
        # Try common Linux serial devices
        for device in ["/dev/ttyACM0", "/dev/ttyUSB0", "/dev/ttyUSB1"]:
            try:
                test_serial = serial.Serial(device, 115200, timeout=0.1)
                test_serial.close()
                return device
            except (serial.SerialException, OSError):
                continue

    return None


def main():
    """Main control loop for hexapod operation."""
    # Setup signal handlers for graceful shutdown
    running = True

    def signal_handler(sig, frame):
        nonlocal running
        logger.info("Received shutdown signal, stopping...")
        running = False

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    logger.info("Creating hexapod...")
    body = create_hexapod()

    logger.info("Initializing gamepad...")
    gamepad = GamePad()
    gamepad.start_reading()

    logger.info("Creating motion planner...")
    motion_planner = MotionPlanner(body, Vec3d(140, 0, -80))
    motion_planner.initialize()

    # Setup serial communication with Servo2040
    serial_port = find_serial_port()
    comport = None
    if serial_port:
        try:
            logger.info(f"Opening serial port: {serial_port}")
            comport = serial.Serial(serial_port, 115200, timeout=0.1)
            comport.reset_input_buffer()
            logger.info("Serial port opened successfully")
        except (serial.SerialException, OSError) as e:
            logger.warning(f"Failed to open serial port: {e}")
            logger.warning("Continuing without hardware control (simulation mode)")
            comport = None
    else:
        logger.warning("No serial port found. Continuing without hardware control (simulation mode)")

    DT = 1 / 50  # Control loop frequency: 50 Hz
    next_time = time.perf_counter()

    logger.info("Starting control loop...")

    try:
        while running:
            gait_vec = Vec2d()
            gait_turn = 0.0

            body_translation_cmd = Vec3d()
            body_rotation_cmd = Vec3d()  # pitch, roll, yaw

            # LEFT STICK
            if gamepad.bumper_l:
                body_translation_cmd = gamepad.joy_l.to_3d()
            else:
                gait_vec = gamepad.joy_l

            # RIGHT STICK
            if gamepad.bumper_l:
                body_translation_cmd = Vec3d(
                    body_translation_cmd.x, body_translation_cmd.y, gamepad.joy_r.y
                )
            else:
                body_rotation_cmd = Vec3d(
                    -gamepad.joy_r.y, gamepad.joy_r.x, body_rotation_cmd.z
                )

            # TRIGGERS
            trigger_turn = gamepad.trigger_l - gamepad.trigger_r
            if gamepad.bumper_r:
                body_rotation_cmd = Vec3d(
                    body_rotation_cmd.x, body_rotation_cmd.y, trigger_turn
                )
            else:
                gait_turn = trigger_turn

            # Update motion planner
            motion_planner.update_gait(DT, gait_vec, gait_turn)
            motion_planner.offset_body(DT, body_translation_cmd, body_rotation_cmd)
            motion_planner.step(DT)

            # Send servo commands to hardware
            if comport:
                msg = body.get_command_message()
                try:
                    comport.write((msg + "\n").encode("utf-8"))
                    # Optional: read response from Servo2040
                    if comport.in_waiting > 0:
                        response = comport.readline().decode("utf-8").strip()
                        if response.startswith("RUNTIME_ERROR:") or response.startswith("FATAL_ERROR:"):
                            logger.error(f"Servo2040 error: {response}")
                except (serial.SerialException, OSError) as e:
                    logger.error(f"Serial communication error: {e}")
                    # Optionally try to reopen the port
                    comport = None

            # Maintain control loop timing
            next_time += DT
            sleep_time = next_time - time.perf_counter()
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        logger.info("Interrupted by user")
    finally:
        logger.info("Shutting down...")
        if comport:
            comport.close()
        gamepad.stop_reading()
        logger.info("Shutdown complete")


if __name__ == "__main__":
    main()

