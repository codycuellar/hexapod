"""
Real-time 3D visualization of hexapod walking simulation.

Hardcoded hexapod creation for simulation - no config needed.
"""

import time
from typing import cast

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

from hexapod.hexpod import Body, Leg, LegID
from hexapod.engine import Frame, Vec2d, Vec3d, Rotation
from hexapod.servos import MockServo
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

    return Body(Frame(), legs)


def draw_hexapod(ax, body: Body, leg_lines):
    for leg_id, leg in body.legs.items():
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


def main():
    print("Creating hexapod...")
    hexapod = create_simulation_hexapod()

    print("Creating path planner...")
    controller = MotionPlanner(hexapod, Vec3d(140, 0, -80))
    controller.initialize()

    plt.ion()
    fig = plt.figure(figsize=(14, 10))
    ax: Axes3D = cast(Axes3D, fig.add_subplot(111, projection="3d"))

    ax.set_xlim(-200, 200)
    ax.set_ylim(-200, 200)
    ax.set_zlim(0, 200)

    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")

    plt.show(block=False)

    leg_lines = {}
    for leg_id in hexapod.legs:
        (line,) = ax.plot([], [], [], "o-", lw=2)
        leg_lines[leg_id] = line

    DT = 0.1  # simulating can't go much faster
    total_time = 0.0

    next_time = time.perf_counter()
    prev_time = time.perf_counter()

    gait_vector = Vec2d(0, 0.1)
    running = True

    while running:
        now = time.perf_counter()

        actual_dt = now - prev_time
        prev_time = now

        total_time += actual_dt

        if actual_dt > DT * 1.1:
            print(f"OVERRUN: {actual_dt*1000:.2f} ms")

        controller.step(DT)

        if total_time > 10:
            gait_vector = Vec2d(0.2, -0.2)

        controller.update_gait(gait_vector)
        controller.step(DT)

        draw_hexapod(ax, hexapod, leg_lines)
        fig.canvas.draw_idle()
        fig.canvas.flush_events()
        plt.pause(0.001)  # <-- GUI event pump only

        next_time += DT
        sleep_time = next_time - time.perf_counter()

        if sleep_time > 0:
            time.sleep(sleep_time)


if __name__ == "__main__":
    main()
