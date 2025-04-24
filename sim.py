import math
import sys
from unittest.mock import MagicMock

sys.modules["servo"] = MagicMock()

import matplotlib.pyplot as plt

from hexapod.utils import Point, Transform3D
from hexapod.body import Body
from hexapod.leg import Leg
from hexapod.servo import Servo


class MockServo(Servo):
    def __init__(
        self,
        name: str,
        upper_limit: int = 90,
        lower_limit: int = -90,
        zeroed_angle: int = 0,
        inverted=False,
    ):
        self.name = name
        self.zeroed_angle = zeroed_angle  # Zeroed angle is kept as it is
        self.inverted = inverted

        # Adjust limits relative to the zeroed angle
        if inverted:
            # Inverted servo: limits are adjusted in the opposite direction
            upper_limit = -upper_limit + self.zeroed_angle
            lower_limit = -lower_limit - self.zeroed_angle
        else:
            # Non-inverted servo: limits stay as they are
            upper_limit = upper_limit - self.zeroed_angle
            lower_limit = lower_limit + self.zeroed_angle

        self.pos_limit = max(upper_limit, lower_limit)
        self.neg_limit = min(upper_limit, lower_limit)

    def set_angle(self, angle):
        self.angle = angle  # Update the angle
        return angle  # Return the angle for consistency

    def get_raw_angle(self, desired_angle):
        """
        Convert a body-relative angle to the actual servo command angle.

        :param desired_angle: The target angle relative to the body.
        :return: The raw servo angle.
        """
        if self.inverted:
            # Inverted servo: subtract the desired angle from the zeroed angle
            raw_angle = self.zeroed_angle - desired_angle
        else:
            # Non-inverted servo: add the desired angle to the zeroed angle
            raw_angle = desired_angle - self.zeroed_angle

        # Clamp the raw angle to the servo limits
        return self._clamp(raw_angle)

    def _clamp(self, value):
        """
        Clamp the value to the servo motor limits.
        :param value: Value to be clamped.
        :return: Clamped value within the servo motor limits.
        """
        return max(min(value, self.pos_limit), self.neg_limit)


leg_dimensions = {"coxa_len": 40, "femur_len": 65, "tibia_len": 90}

hexapod = Body(
    Leg(
        "Right Front",
        MockServo("Coxa", 30, -60, -5, True),
        MockServo("Femur", 60, -45, -26, True),
        MockServo("Tibia", 10, 180, 108),
        Transform3D(),
        **leg_dimensions,
    ),
    Leg(
        "Right Center",
        MockServo("Coxa", 45, -45, -6, True),
        MockServo("Femur", 60, -45, -30, True),
        MockServo("Tibia", 10, 180, 105),
        Transform3D(),
        **leg_dimensions,
    ),
    Leg(
        "Right Back",
        MockServo("Coxa", 60, -30, 0, True),
        MockServo("Femur", 60, -45, -20, True),
        MockServo("Tibia", 10, 180, 90),
        Transform3D(),
        **leg_dimensions,
    ),
    Leg(
        "Left Front",
        MockServo("Coxa", 60, -30, 2, True),
        MockServo("Femur", 60, -45, -22),
        MockServo("Tibia", 10, 180, 90, True),
        Transform3D(),
        **leg_dimensions,
    ),
    Leg(
        "Left Center",
        MockServo("Coxa", 45, -45, -5, True),
        MockServo("Femur", 60, -45, -25),
        MockServo("Tibia", 10, 180, 90, True),
        Transform3D(),
        **leg_dimensions,
    ),
    Leg(
        "Left Back",
        MockServo("Coxa", 30, -60, 0, True),
        MockServo("Femur", 60, -45, -20),
        MockServo("Tibia", 10, 180, 90, True),
        Transform3D(),
        **leg_dimensions,
    ),
)

trajectory = []  # Store the leg tip positions for visualization
time_steps = 100

for t in range(time_steps):
    hexapod.walk()


fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")

# Plot the body center
ax.scatter(0, 0, 0, color="red", label="Body Center")

# Plot each leg's trajectory
for i, leg_traj in enumerate(leg_trajectories):
    x_vals, y_vals, z_vals = zip(*leg_traj)
    ax.plot(x_vals, y_vals, z_vals, label=f"Leg {i} Trajectory")

# Set labels and legend
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
ax.legend()
plt.show()
