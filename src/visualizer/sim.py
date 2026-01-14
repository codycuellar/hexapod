"""
Real-time 3D visualization of hexapod walking simulation.

Hardcoded hexapod creation for simulation - no config needed.
"""

import time
from typing import cast

import serial
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

from hexapod.gamepad import GamePad
from hexapod.rigid_body import Body, Leg, LegID
from hexapod.engine import Frame, Vec3d, Vec2d, Rotation, Transform
from hexapod.servos import MockServo, Servo
from hexapod.motion_planner import MotionPlanner


def create_simulation_hexapod() -> Body:
    """
    Create a hexapod for simulation with hardcoded parameters.
    No config needed - just builds the frame hierarchy directly.
    """
    # Joint lengths (mm)
    COXA_LEN = 40.0
    FEMUR_LEN = 65.0
    TIBIA_LEN = 90.0

    origin_frame = Frame()
    distance = Vec3d(100, 0, 0)

    # get the position in the frame's local coordinates to the global coordinates,
    # and rotate the frame 60 degrees for each leg
    ids = [LegID.RM, LegID.RF, LegID.LF, LegID.LM, LegID.LB, LegID.RB]
    leg_setup = [COXA_LEN, FEMUR_LEN, TIBIA_LEN, MockServo(), MockServo(), MockServo()]

    legs = {}

    for id in ids:
        mount_pos = origin_frame.local_pos_to_world(distance)
        frame = Frame(origin=mount_pos, rotation=origin_frame.rotation)
        legs[id] = Leg(id, frame, *leg_setup)
        origin_frame.rotate(Rotation.degrees(0.0, 0.0, 60.0))
    legs[LegID.RF] = Leg(
        LegID.RF,
        legs[LegID.RF].frame,
        COXA_LEN,
        FEMUR_LEN,
        TIBIA_LEN,
        Servo(3, -6.0, inverted=True),
        Servo(4, -30.0, inverted=False),
        Servo(5, 67.0, inverted=True),
    )
    legs[LegID.RM] = Leg(
        LegID.RM,
        legs[LegID.RM].frame,
        COXA_LEN,
        FEMUR_LEN,
        TIBIA_LEN,
        Servo(6, -7.0, inverted=True),
        Servo(7, -30.0, inverted=False),
        Servo(8, 65.0, inverted=True),
    )
    legs[LegID.RB] = Leg(
        LegID.RB,
        legs[LegID.RB].frame,
        COXA_LEN,
        FEMUR_LEN,
        TIBIA_LEN,
        Servo(9, 0.0, inverted=True),
        Servo(10, -26.0, inverted=False),
        Servo(11, 73.0, inverted=True),
    )
    legs[LegID.LF] = Leg(
        LegID.LF,
        legs[LegID.LF].frame,
        COXA_LEN,
        FEMUR_LEN,
        TIBIA_LEN,
        Servo(0, 2.0, inverted=True),
        Servo(1, 24.0, inverted=True),
        Servo(2, -80.0, inverted=False),
    )
    legs[LegID.LM] = Leg(
        LegID.LM,
        legs[LegID.LM].frame,
        COXA_LEN,
        FEMUR_LEN,
        TIBIA_LEN,
        Servo(15, -4.0, inverted=True),
        Servo(16, 27.0, inverted=True),
        Servo(17, -85.0, inverted=False),
    )
    legs[LegID.LB] = Leg(
        LegID.LB,
        legs[LegID.LB].frame,
        COXA_LEN,
        FEMUR_LEN,
        TIBIA_LEN,
        Servo(12, -4.0, inverted=True),
        Servo(13, 31.0, inverted=True),
        Servo(14, -77.0, inverted=False),
    )

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
    comport = serial.Serial("COM3", 115200)
    comport.reset_input_buffer()

    print("Creating hexapod...")
    body = create_simulation_hexapod()

    gamepad = GamePad()
    gamepad.start_reading()

    print("Creating path planner...")
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

    DT = 1 / 20  # simulating can't go much faster

    next_time = time.perf_counter()

    running = True

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

        # right stick
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

        motion_planner.update_gait(
            DT,
            gait_vec,
            gait_turn,
        )

        motion_planner.offset_body(
            DT,
            body_translation_cmd,
            body_rotation_cmd,
        )

        motion_planner.step(DT)

        ground_frame = motion_planner.ground_transform @ ground_frame

        # draw ground grid using the accumulated frame
        update_ground_grid_accumulated(grid_lines, ground_frame)
        draw_hexapod(ax, body, leg_lines, body_line)

        msg = body.get_command_message()
        # print(msg)
        comport.write((msg + "\n").encode("utf-8"))
        if comport.in_waiting > 0:
            response = comport.readline().decode("utf-8").strip()
            print(f"Pico says: {response}")

        fig.canvas.draw()
        plt.pause(0.001)  # <-- GUI event pump only

        next_time += DT
        sleep_time = next_time - time.perf_counter()

        if sleep_time > 0:
            time.sleep(sleep_time)


if __name__ == "__main__":
    main()
