"""
Hexapod main controller entry point.

This module provides the main control loop for the hexapod robot.
It handles gamepad input, motion planning, and serial communication with the Servo2040.
"""

import argparse
import logging
import sys
import time
from collections import OrderedDict

from hexapod.engine import Frame, Vec3d, Vec2d, Rotation
from hexapod.gamepad import get_controller
from hexapod.hexapod_serial import HexapodSerial, CommandError
from hexapod.motion_planner import MotionPlanner
from hexapod.rigid_body import Body, Leg, LegID, LegConfig
from hexapod.servos import Servo

DEFAULT_FPS = 20

logger = logging.getLogger(__name__)


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


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Hexapod control loop: gamepad input, motion planning, servo output."
    )
    parser.add_argument(
        "--fps",
        type=float,
        default=DEFAULT_FPS,
        help=f"Target loop rate in Hz (default {DEFAULT_FPS}).",
    )
    parser.add_argument(
        "--disable-servos",
        action="store_true",
        help="Run without connecting to servos (motion planner only).",
    )
    parser.add_argument(
        "--loglevel",
        choices=["DEBUG", "INFO", "WARNING", "ERROR"],
        default="INFO",
        help="Logging level (default INFO).",
    )
    parser.add_argument(
        "--profile",
        action="store_true",
        help="Enable loop profiling. Outputs timing to stderr (separate from logging). Use --loglevel ERROR to avoid logging overhead affecting results.",
    )
    parser.add_argument(
        "--profile-interval",
        type=int,
        default=60,
        metavar="N",
        help="Print profile summary every N frames when --profile (default 60).",
    )
    return parser.parse_args()


def _print_profile(accum: dict[str, list[float]], budget_ms: float) -> None:
    """Print profile summary to stderr. Uses print() not logging to avoid affecting timings."""
    total = 0.0
    lines: list[str] = []
    for name in accum.keys():
        vals = accum.get(name, [])
        if vals:
            avg = sum(vals) / len(vals)
            total += avg
            lines.append(f"  {name}: avg={avg:.2f}ms max={max(vals):.2f}")
    if lines:
        print("[profile] ---", file=sys.stderr)
        print("\n".join(lines), file=sys.stderr)
        print(f"  total: {total:.2f} ms  budget: {budget_ms:.2f} ms", file=sys.stderr)
        print("[profile] ---", file=sys.stderr)


def main() -> None:
    """Main control loop for hexapod operation."""
    args = _parse_args()
    logging.basicConfig(
        level=getattr(logging, args.loglevel),
        format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
    )

    running = True
    DT = 1.0 / args.fps

    logger.info("Creating hexapod geometry...")
    body = create_hexapod()

    logger.info("Creating motion planner...")
    motion_planner = MotionPlanner(body, Vec3d(140, 0, -80))
    motion_planner.initialize()

    logger.info("Initializing gamepad...")
    gamepad = get_controller()
    gamepad.start_reading()

    hp_serial: HexapodSerial | None = None
    if not args.disable_servos:
        hp_serial = HexapodSerial(connect_timeout=10)
        if not hp_serial.connect():
            logger.info("Hexapod not connected; loop will retry every 5s")
    else:
        logger.info("Servos disabled (--disable-servos)")

    prev_time = time.perf_counter()
    last_overrun_log = 0.0  # Rate-limit overrun warnings
    last_reconnect_attempt = 0.0  # Cooldown to avoid blocking loop on connect()

    # Profiling state (only used when --profile; avoids overhead when disabled)
    profile = getattr(args, "profile", False)
    profile_interval = getattr(args, "profile_interval", 60)
    profile_accum: OrderedDict[str, list[float]] = OrderedDict()
    frame_count = 0

    logger.info("Starting control loop...")
    if profile:
        print(
            "[profile] enabled, output every",
            profile_interval,
            "frames",
            file=sys.stderr,
        )

    try:
        while running:
            if hp_serial is not None and not hp_serial.conn:
                now_ = time.perf_counter()
                if now_ - last_reconnect_attempt >= 5.0:
                    last_reconnect_attempt = now_
                    hp_serial.connect()

            pt = time.perf_counter() if profile else 0.0

            gait_vec = Vec2d()
            gait_turn = 0.0

            body_offset_trans = Vec3d()
            body_offset_rot = Vec3d()  # pitch, roll, yaw

            # LEFT STICK
            _, bumper_l_held = gamepad.get_bumper_l()
            joy_l = gamepad.get_joy_l()
            joy_r = gamepad.get_joy_r()
            if bumper_l_held:
                body_offset_trans = joy_l.to_3d()
            else:
                gait_vec = joy_l

            # RIGHT STICK
            if bumper_l_held:
                body_offset_trans = Vec3d(
                    body_offset_trans.x, body_offset_trans.y, joy_r.y
                )
            else:
                body_offset_rot = Vec3d(-joy_r.y, joy_r.x, body_offset_rot.z)

            # TRIGGERS
            trigger_turn = gamepad.get_trigger_l() - gamepad.get_trigger_r()
            _, bumper_r_held = gamepad.get_bumper_r()
            if bumper_r_held:
                body_offset_rot = Vec3d(
                    body_offset_rot.x, body_offset_rot.y, trigger_turn
                )
            else:
                gait_turn = trigger_turn

            if profile:
                t = time.perf_counter()
                profile_accum.setdefault("gamepad", []).append((t - pt) * 1000)
                pt = t

            # Update motion planner
            motion_planner.update_gait(DT, gait_vec, gait_turn)
            motion_planner.offset_body(DT, body_offset_trans, body_offset_rot)
            t = time.perf_counter()
            profile_accum.setdefault("motion_update", []).append((t - pt) * 1000)
            pt = t
            motion_profiles = motion_planner.step(DT)

            if profile:
                for p in motion_profiles:
                    profile_accum.setdefault(p[0], []).append((p[1] - pt) * 1000)
                    pt = p[1]

            angles_start = time.perf_counter()
            servo_data = None
            if hp_serial is not None:
                servo_data = body.get_servo_angles()

            if profile:
                t = time.perf_counter()
                profile_accum.setdefault("angles", []).append((t - angles_start) * 1000)
                pt = t

            if hp_serial is not None and servo_data:
                try:
                    calc_t, send_t = hp_serial.send_servos(servo_data)
                    if profile:
                        profile_accum.setdefault("serial_calc", []).append(
                            (calc_t - pt) * 1000
                        )
                        profile_accum.setdefault("serial_send", []).append(
                            (send_t - calc_t) * 1000
                        )
                        pt = send_t
                except CommandError as e:
                    logger.error("Command failed: %s", e)

            now = time.perf_counter()
            time_left = prev_time + DT - now
            if time_left > 0:
                time.sleep(time_left)
            else:
                # Rate-limit overrun warnings to avoid log I/O bottleneck
                if now - last_overrun_log >= 0.5:
                    logger.warning(
                        "Frame overran by %.4f ms (budget %.4f ms)",
                        -time_left * 1000,
                        DT * 1000,
                    )
                    last_overrun_log = now

            if profile:
                t = time.perf_counter()
                profile_accum.setdefault("sleep", []).append((t - pt) * 1000)
                frame_count += 1
                if frame_count >= profile_interval:
                    _print_profile(profile_accum, DT * 1000)
                    profile_accum.clear()
                    frame_count = 0

            prev_time = time.perf_counter()

    except KeyboardInterrupt:
        logger.info("Interrupted by user")
    finally:
        logger.info("Shutting down...")
        gamepad.stop_reading()
        if hp_serial is not None:
            hp_serial.close()
        logger.info("Shutdown complete")


if __name__ == "__main__":
    main()
