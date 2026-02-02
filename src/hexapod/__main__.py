"""
Hexapod main controller entry point.

This module provides the main control loop for the hexapod robot.
It handles gamepad input, motion planning, and serial communication with the Servo2040.
"""

import time
import logging

from hexapod.engine import Frame, Vec3d, Vec2d, Rotation
from hexapod.gamepad import get_controller
from hexapod.motion_planner import MotionPlanner
from hexapod.rigid_body import Body, Leg, LegID, LegConfig
from hexapod.hexapod_serial import HexapodSerial, CommandError
from hexapod.servos import Servo

FPS = 20

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
    gamepad = get_controller()  # uses GAMEPAD env or "xbox"
    gamepad.start_reading()

    # Setup serial communication with Servo2040
    hp_serial = HexapodSerial(connect_timeout=10)
    hp_serial.connect()

    DT = 1 / FPS
    prev_time = time.perf_counter()

    logger.info("Starting control loop...")

    try:
        while running:
            # Check for connection
            if not hp_serial.conn:
                hp_serial.connect()

            gait_vec = Vec2d()
            gait_turn = 0.0

            body_offset_trans = Vec3d()
            body_offset_rot = Vec3d()  # pitch, roll, yaw

            # LEFT STICK
            _, bumper_l_held = gamepad.get_bumper_l()
            if bumper_l_held:
                body_offset_trans = gamepad.get_joy_l().to_3d()
            else:
                gait_vec = gamepad.get_joy_l()

            # RIGHT STICK
            if bumper_l_held:
                body_offset_trans = Vec3d(
                    body_offset_trans.x, body_offset_trans.y, gamepad.get_joy_r().y
                )
            else:
                body_offset_rot = Vec3d(
                    -gamepad.get_joy_r().y, gamepad.get_joy_r().x, body_offset_rot.z
                )

            # TRIGGERS
            trigger_turn = gamepad.get_trigger_l() - gamepad.get_trigger_r()
            _, bumper_r_held = gamepad.get_bumper_r()
            if bumper_r_held:
                body_offset_rot = Vec3d(
                    body_offset_rot.x, body_offset_rot.y, trigger_turn
                )
            else:
                gait_turn = trigger_turn

            # Update motion planner
            motion_planner.update_gait(DT, Vec2d(0.0, 1.0), gait_turn)
            motion_planner.offset_body(DT, body_offset_trans, body_offset_rot)
            motion_planner.step(DT)

            # Send servo commands to hardware
            servo_data = body.get_servo_angles()
            try:
                hp_serial.send_servos(servo_data)
            except CommandError as e:
                logger.error(f"Command failed: {e}")
                # Could trigger reconnect or other error handling here

            now = time.perf_counter()
            time_left = prev_time + DT - now
            if time_left > 0:
                time.sleep(time_left)
            else:
                logger.warning(
                    "Frame overran by %.4f ms (budget %.4f ms)",
                    -time_left * 1000,
                    DT * 1000,
                )
            prev_time = time.perf_counter()

    except KeyboardInterrupt:
        logger.info("Interrupted by user")
    finally:
        logger.info("Shutting down...")
        gamepad.stop_reading()
        hp_serial.close()
        logger.info("Shutdown complete")


if __name__ == "__main__":
    main()
