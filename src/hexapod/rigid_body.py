import math
import logging
from dataclasses import dataclass

from hexapod.servos import JointControl, ServoAngles
from hexapod.engine import Frame, Vec3d, Vec2d, Rotation
from enum import Enum

logger = logging.getLogger()

HIGH_MASK = 0x80
LOW_MASK = 0x7F


class LegID(Enum):
    LF = 0
    LM = 1
    LB = 2
    RF = 3
    RM = 4
    RB = 5


class Body:
    def __init__(self, frame: Frame, legs: dict[LegID, "Leg"]):
        """
        Initialize the hexapod body.

        :param frame: Body coordinate frame
        :param legs: Dictionary of legs by LegID
        """
        self.frame = frame
        self.legs = legs
        self.leg_ids = list(legs.keys())

        for key in self.legs:
            self.legs[key].frame.parent = self.frame

    def set_foot_position(self, leg_id: LegID, position: Vec3d):
        """
        Set foot position for a specific leg in body-relative coordinates.
        """
        leg = self.legs[leg_id]
        leg_local_pos = leg.frame.world_pos_to_local(
            self.frame.local_pos_to_world(position)
        )
        leg.set_foot_pos(leg_local_pos)

    def get_foot_position(self, leg_id: LegID) -> Vec3d:
        """
        Get current foot position in body-relative coordinates.
        """
        leg = self.legs[leg_id]
        return leg.foot_frame.get_origin_in_frame(self.frame)

    def get_servo_angles(self):
        """
        Get current servo positions as list of (pin, angle) tuples.

        Returns:
            List of (pin_number, angle_degrees) tuples
        """
        servos: ServoAngles = {}
        for leg in self.legs.values():
            for servo in [leg.coxa_control, leg.femur_control, leg.tibia_control]:
                pin = servo.pin_number
                angle = servo.get_angle()
                servos[pin] = angle
        return servos


@dataclass
class LegConfig:
    coxa: tuple[float, JointControl]
    femur: tuple[float, JointControl]
    tibia: tuple[float, JointControl]


class Leg:
    def __init__(
        self,
        leg_id: LegID,
        mount_frame: Frame,
        config: LegConfig,
    ):
        self.id = leg_id
        self.name = leg_id
        self.frame = mount_frame

        self.coxa_length, self.coxa_control = config.coxa
        self.femur_length, self.femur_control = config.femur
        self.tibia_length, self.tibia_control = config.tibia

        self.coxa_frame = Frame(parent=self.frame)
        self.femur_frame = Frame(
            origin=Vec3d(self.coxa_length, 0, 0), parent=self.coxa_frame
        )
        self.tibia_frame = Frame(
            origin=Vec3d(self.femur_length, 0, 0), parent=self.femur_frame
        )
        self.foot_frame = Frame(
            origin=Vec3d(self.tibia_length, 0, 0), parent=self.tibia_frame
        )

    def set_foot_pos(self, position: Vec3d):
        """
        Set the foot position relative to the leg's coxa frame which is the mount
        point on the body.
        :param position: Position vector in the coxa frame's coordinate system.
        """
        # Calculate RAW IK angles
        cox_a, fem_a, tib_a = self._calculate_ik(position)

        # fem_a is the 2d positive angle (counter clockwise), but in 3d, with y
        # axis away, this is a negative rotation
        fem_a = -fem_a
        # tib_a is the openness angle of the tibia, but our reference is straight out,
        # which would be 180 degrees.
        tib_a = -tib_a

        # Set joint angles and update frames
        self.coxa_control.set_angle(cox_a)
        self.femur_control.set_angle(fem_a)
        self.tibia_control.set_angle(tib_a)
        self.coxa_frame.rotation = Rotation.degrees(z=cox_a)
        self.femur_frame.rotation = Rotation.degrees(y=fem_a)
        self.tibia_frame.rotation = Rotation.degrees(y=tib_a)

    def _calculate_ik(self, position: Vec3d) -> list[float]:
        """
        Calculates the raw angles from a base 'home' position. Home is considered
        every joint frame lying on the positive x axis of the mount frame, with all
        x axes parallel. This is angle of 0 for each joint. positive angles rotate
        the children downward (in z space) about the y axis, and negative angles
        rotate upward (in z space) from this starting home position. The coxa
        rotation angle positve rotates away (in y space) about the z axis, and
        negative rotates towards.

        | z-axis
        |
        | / y-axis
        |/
        o------- x-axis

        cox------->fem------->tib------->foot

        Args:
            position: The position relative to the coxa frame. This MUST be converted
                from the originating relative frame to this leg's coxa frame.

        Returns:
            The raw angles of each frame.
        """
        # The ground plane position (when observing from the top)
        xy_pos = position.to_2d()

        coxa_angle = xy_pos.angle_x()

        xy_dist = abs(xy_pos.length() - self.coxa_length)
        hypot_len = Vec2d(position.z, xy_dist).length()

        femur_sqr = self.femur_length**2
        tibia_sqr = self.tibia_length**2
        hypot_squared = hypot_len**2

        femur_theta = (femur_sqr + hypot_squared - tibia_sqr) / (
            2 * self.femur_length * hypot_len
        )

        femur_interior_angle = math.acos(clamp(femur_theta, -1.0, 1.0))
        foot_angle_from_x_axis = math.atan2(position.z, xy_dist)

        femur_angle = femur_interior_angle + foot_angle_from_x_axis

        tibia_angle = (femur_sqr + tibia_sqr - hypot_len**2) / (
            2 * self.tibia_length * self.femur_length
        )

        tibia_angle = math.acos(clamp(tibia_angle, -1.0, 1.0))

        return [
            math.degrees(a) for a in [coxa_angle, femur_angle, tibia_angle - math.pi]
        ]

    def clamp_reach(self, position: Vec3d, femur_to_foot_dist: float):
        # Clamp reach distance to maximum if it exceeds (with small tolerance)
        fem_tib_reach = self.femur_length + self.tibia_length

        if femur_to_foot_dist > fem_tib_reach + 1e-6:
            # Scale the position vector to be within reach
            # Keep the direction, just reduce the magnitude
            position_mag = position.length()
            if position_mag > 0:
                # Calculate what the max reachable distance is from coxa
                # TODO: This doesn't factor in the rigid Z axis of the coxa.
                direction = position.normalize()
                new_position = direction * (self.coxa_length + fem_tib_reach)
                # Recalculate after scaling
                xy_dist = max(0, new_position.to_2d().length() - self.coxa_length)
                femur_to_foot_dist = Vec2d(new_position.z, xy_dist).length()
                logger.warning(
                    f"Leg {self.id} maximum reach attempted! Clamping {position} to {new_position}"
                )
                return new_position

        return position


def clamp(value: float, _min: float, _max: float) -> float:
    """Hard clamp a value between min and max."""
    return max(_min, min(_max, value))
