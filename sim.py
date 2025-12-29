"""
Real-time 3D visualization of hexapod walking simulation.

Hardcoded hexapod creation for simulation - no config needed.
"""

import numpy as np
import matplotlib.pyplot as plt

from hexapod.hexpod import Body, Leg, LegID, Joint
from hexapod.engine import Frame, Vec3d, Rotation
from hexapod.servos import MockServo
from hexapod.controller import HexapodController


def create_simulation_hexapod() -> Body:
    """
    Create a hexapod for simulation with hardcoded parameters.
    No config needed - just builds the frame hierarchy directly.
    """
    # Joint lengths (mm)
    COXA_LENGTH = 40.0
    FEMUR_LENGTH = 65.0
    TIBIA_LENGTH = 90.0
    coxa = Joint(MockServo(), "z", COXA_LENGTH)
    femur = Joint(MockServo(), "y", FEMUR_LENGTH)
    tibia = Joint(MockServo(), "y", TIBIA_LENGTH)

    joint_f = Frame(position=Vec3d(100, 0, 0))
    rot = Rotation.degrees(0.0, 0.0, 60.0)

    # the leg connection point updates the lm_frame by 60 degrees, then copies it.
    legs = {
        LegID.RM: Leg(LegID.RM, coxa, femur, tibia, joint_f.copy()),
        LegID.RF: Leg(LegID.RF, coxa, femur, tibia, joint_f.rotate(rot).copy()),
        LegID.LF: Leg(LegID.LF, coxa, femur, tibia, joint_f.rotate(rot).copy()),
        LegID.LM: Leg(LegID.LM, coxa, femur, tibia, joint_f.rotate(rot).copy()),
        LegID.LB: Leg(LegID.LB, coxa, femur, tibia, joint_f.rotate(rot).copy()),
        LegID.RB: Leg(LegID.RB, coxa, femur, tibia, joint_f.rotate(rot).copy()),
    }

    # Create body
    return Body(Frame(position=Vec3d(0, 0, 80)), legs)


def draw_hexapod(ax, body, clear=True):
    """Draw the hexapod in 3D space - static visualization."""
    if clear:
        ax.clear()

    body_pos = body.frame.origin

    # Draw body center as a dot
    ax.scatter(
        [body_pos.x],
        [body_pos.y],
        [body_pos.z],
        color="black",
        s=200,
        marker="o",
        label="Body",
    )

    circle_radius = 80.0

    # Draw XY plane circle around body
    theta = np.linspace(0, 2 * np.pi, 100)
    circle_x = body_pos.x + circle_radius * np.cos(theta)
    circle_y = body_pos.y + circle_radius * np.sin(theta)
    circle_z = np.full(100, body_pos.z)
    ax.plot(circle_x, circle_y, circle_z, "b-", alpha=0.3, linewidth=1)

    # Color mapping: each leg gets a unique color
    leg_colors = {
        "LF": "red",
        "RF": "orange",
        "LM": "blue",
        "RM": "cyan",
        "LR": "green",
        "RR": "purple",
    }

    # Draw leg segments with color coding
    for leg_id in LegID:
        if leg_id not in body.legs:
            continue

        leg = body.legs[leg_id]
        leg_name = leg_id.name
        leg_color = leg_colors.get(leg_name, "gray")

        # Get positions in body frame (world coordinates)
        coxa_pos = leg.coxa.frame.get_position_in_frame(body.frame)
        femur_pos = leg.femur.frame.get_position_in_frame(body.frame)
        tibia_pos = leg.tibia.frame.get_position_in_frame(body.frame)
        foot_pos = leg.foot_frame.get_position_in_frame(body.frame)

        # Draw connecting lines
        ax.plot(
            [body_pos.x, coxa_pos.x],
            [body_pos.y, coxa_pos.y],
            [body_pos.z, coxa_pos.z],
            color=leg_color,
            linewidth=1,
            alpha=0.5,
            linestyle="--",
        )
        ax.plot(
            [coxa_pos.x, femur_pos.x],
            [coxa_pos.y, femur_pos.y],
            [coxa_pos.z, femur_pos.z],
            color=leg_color,
            linewidth=2,
            alpha=0.8,
        )
        ax.plot(
            [femur_pos.x, tibia_pos.x],
            [femur_pos.y, tibia_pos.y],
            [femur_pos.z, tibia_pos.z],
            color=leg_color,
            linewidth=2,
            alpha=0.8,
        )
        ax.plot(
            [tibia_pos.x, foot_pos.x],
            [tibia_pos.y, foot_pos.y],
            [tibia_pos.z, foot_pos.z],
            color=leg_color,
            linewidth=2,
            alpha=0.8,
        )

        # Draw all joints as dots (label once per leg for legend)
        ax.scatter(
            [coxa_pos.x],
            [coxa_pos.y],
            [coxa_pos.z],
            color=leg_color,
            s=100,
            marker="o",
            label=leg_name,
        )
        ax.scatter(
            [femur_pos.x],
            [femur_pos.y],
            [femur_pos.z],
            color=leg_color,
            s=80,
            marker="s",
        )
        ax.scatter(
            [tibia_pos.x],
            [tibia_pos.y],
            [tibia_pos.z],
            color=leg_color,
            s=80,
            marker="^",
        )
        ax.scatter(
            [foot_pos.x], [foot_pos.y], [foot_pos.z], color=leg_color, s=150, marker="*"
        )

    # Set labels and limits
    ax.set_xlabel("X (mm)")
    ax.set_ylabel("Y (mm)")
    ax.set_zlabel("Z (mm)")
    ax.set_title("Hexapod Frame Visualization")

    # Set limits first
    xlim = [-350, 350]
    ylim = [-350, 350]
    zlim = [0, 200]
    ax.set_xlim(xlim)
    ax.set_ylim(ylim)
    ax.set_zlim(zlim)

    # Calculate aspect ratio based on limits to ensure equal scaling
    # This ensures 1 unit on each axis appears the same length
    x_range = xlim[1] - xlim[0]
    y_range = ylim[1] - ylim[0]
    z_range = zlim[1] - zlim[0]
    max_range = max(x_range, y_range, z_range)

    # Normalize to make all ranges appear equal
    ax.set_box_aspect([x_range / max_range, y_range / max_range, z_range / max_range])

    # Add legend
    ax.legend(loc="upper left", bbox_to_anchor=(1.05, 1))


def main():
    # Create hexapod directly (hardcoded for simulation)
    print("Creating hexapod for simulation...")
    hexapod = create_simulation_hexapod()

    # Create controller and snap to walking gait
    print("Creating controller...")
    controller = HexapodController(hexapod)
    controller.initialize()

    # # Set to tripod gait and snap to a walking position
    # print("Snapping to tripod gait position...")
    # controller.set_gait("tripod")
    # controller.gait_phase = 0.25  # Set to a specific phase (0.25 = mid-swing for group 1)
    # controller.update(0.0)  # Update with dt=0 to snap to position (no animation)

    # Set up matplotlib for static plotting
    fig = plt.figure(figsize=(14, 10))
    ax = fig.add_subplot(111, projection="3d")

    # Draw hexapod (static)
    draw_hexapod(ax, hexapod, clear=False)

    # Show the plot (blocks until window is closed)
    plt.show()


if __name__ == "__main__":
    main()
