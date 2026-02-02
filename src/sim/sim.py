"""
Real-time 3D visualization of hexapod walking simulation.

Hardcoded hexapod creation for simulation - no config needed.
"""

import time
from typing import cast

import logging

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

from hexapod.gamepad import get_controller
from hexapod.rigid_body import Body, Leg, LegID, LegConfig
from hexapod.engine import Frame, Vec3d, Vec2d, Rotation, Transform
from hexapod.servos import Servo
from hexapod.motion_planner import MotionPlanner


logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO)


def create_simulation_hexapod() -> Body:
    """
    Create a hexapod for simulation with hardcoded parameters.
    No config needed - just builds the frame hierarchy directly.
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

    # get the position in the frame's local coordinates to the global coordinates,
    # and rotate the frame 60 degrees for each leg
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


def draw_hexapod(ax, body: Body, leg_lines, body_line):
    for leg_id, leg in body.legs.items():
        leg.coxa_frame
        p0 = leg.coxa_frame.get_origin_in_world()
        p1 = leg.femur_frame.get_origin_in_world()
        p2 = leg.tibia_frame.get_origin_in_world()
        p3 = leg.foot_frame.get_origin_in_world()

        xs = [p0.x, p1.x, p2.x, p3.x]
        ys = [p0.y, p1.y, p2.y, p3.y]
        zs = [p0.z, p1.z, p2.z, p3.z]

        line = leg_lines[leg_id]
        line.set_data(xs, ys)
        line.set_3d_properties(zs)

    # order of legs: make sure consistent
    coxa_points = [leg.coxa_frame.get_origin_in_world() for leg in body.legs.values()]
    # repeat first point at end if you want a loop
    coxa_points.append(coxa_points[0])

    xs = [p.x for p in coxa_points]
    ys = [p.y for p in coxa_points]
    zs = [p.z for p in coxa_points]

    body_line.set_data(xs, ys)
    body_line.set_3d_properties(zs)


GRID_SPACING = 50.0
GRID_COUNT = 50  # how many lines in each direction


def create_ground_grid(ax):
    """Pre-create grid lines and return them"""
    lines = []

    half_count = GRID_COUNT // 2
    for i in range(-half_count, half_count + 1):
        # X lines along Y axis
        (line,) = ax.plot([], [], [], color="gray", lw=0.5)
        lines.append(("x", i, line))
        # Y lines along X axis
        (line,) = ax.plot([], [], [], color="gray", lw=0.5)
        lines.append(("y", i, line))
    return lines


def update_ground_grid_accumulated(lines: list, ground_frame: Transform):
    """
    Update ground grid based on accumulated ground_frame.
    - Ground starts at Z = -60
    - Rotates with ground_frame.rotation
    - Translates with ground_frame.origin
    """
    half_count = GRID_COUNT // 2
    z0 = 0.0  # local plane Z (we embed -60 in ground_frame.origin)

    for axis, i, line in lines:
        if axis == "x":
            start_local = Vec3d(i * GRID_SPACING, -half_count * GRID_SPACING, z0)
            end_local = Vec3d(i * GRID_SPACING, half_count * GRID_SPACING, z0)
        else:  # "y"
            start_local = Vec3d(-half_count * GRID_SPACING, i * GRID_SPACING, z0)
            end_local = Vec3d(half_count * GRID_SPACING, i * GRID_SPACING, z0)

        # Apply rotation then translation
        start_world = ground_frame.rotation @ start_local + ground_frame.translation
        end_world = ground_frame.rotation @ end_local + ground_frame.translation

        line.set_clip_on(False)
        line.set_data([start_world.x, end_world.x], [start_world.y, end_world.y])
        line.set_3d_properties([start_world.z, end_world.z])


def main():
    """
    Visualization main loop.
    """
    logger.info("Creating hexapod for visualization...")
    body = create_simulation_hexapod()

    gamepad = get_controller()
    gamepad.start_reading()

    logger.info("Creating motion planner for visualization...")
    motion_planner = MotionPlanner(body, Vec3d(140, 0, -80))
    motion_planner.initialize()

    plt.ion()
    fig = plt.figure(figsize=(14, 10))
    ax: Axes3D = cast(Axes3D, fig.add_subplot(111, projection="3d"))

    ax.set_xlim(-200, 200)
    ax.set_ylim(-200, 200)
    ax.set_zlim(-200, 200)

    ax.set_axis_off()

    plt.show(block=False)

    leg_lines = {}
    for leg_id in body.legs:
        (line,) = ax.plot([], [], [], "o-", lw=2)
        leg_lines[leg_id] = line
    (body_line,) = ax.plot([], [], [], "k-", lw=2)  # black line

    grid_lines = create_ground_grid(ax)
    ground_frame = Frame()

    DT = 1 / 15  # simulating can't go much faster

    next_time = time.perf_counter()

    logger.info("Starting visualization loop...")

    running = True

    try:
        while running:
            gait_vec = Vec2d()
            gait_turn = 0.0

            body_translation_cmd = Vec3d()
            body_rotation_cmd = Vec3d()  # pitch, roll, yaw

            # LEFT STICK
            _, bumper_l_held = gamepad.get_bumper_l()
            if bumper_l_held:
                body_translation_cmd = gamepad.get_joy_l().to_3d()
            else:
                gait_vec = gamepad.get_joy_l()

            # RIGHT STICK
            if bumper_l_held:
                body_translation_cmd = Vec3d(
                    body_translation_cmd.x, body_translation_cmd.y, gamepad.get_joy_r().y
                )
            else:
                body_rotation_cmd = Vec3d(
                    -gamepad.get_joy_r().y, gamepad.get_joy_r().x, body_rotation_cmd.z
                )

            # TRIGGERS
            trigger_turn = gamepad.get_trigger_l() - gamepad.get_trigger_r()
            _, bumper_r_held = gamepad.get_bumper_r()
            if bumper_r_held:
                body_rotation_cmd = Vec3d(
                    body_rotation_cmd.x, body_rotation_cmd.y, trigger_turn
                )
            else:
                gait_turn = trigger_turn

            # Update motion planner with gamepad inputs
            motion_planner.update_gait(DT, gait_vec, gait_turn)
            motion_planner.offset_body(DT, body_translation_cmd, body_rotation_cmd)
            motion_planner.step(DT)

            ground_frame.apply_transform(motion_planner.ground_transform)

            # Draw ground grid using the accumulated frame
            update_ground_grid_accumulated(grid_lines, ground_frame.as_transform())
            draw_hexapod(ax, body, leg_lines, body_line)

            fig.canvas.draw()
            plt.pause(0.001)  # GUI event pump only

            next_time += DT
            sleep_time = next_time - time.perf_counter()

            if sleep_time > 0:
                time.sleep(sleep_time)

            # Check if window is still open
            if not plt.get_fignums():
                running = False

    except KeyboardInterrupt:
        logger.info("Visualization interrupted by user")
    finally:
        gamepad.stop_reading()


if __name__ == "__main__":
    main()
