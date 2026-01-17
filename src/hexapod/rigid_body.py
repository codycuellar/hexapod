import math
import logging
from dataclasses import dataclass

from hexapod.servos import JointControl
from hexapod.engine import Frame, Vec3d, Vec2d, Rotation
from enum import Enum

logger = logging.getLogger()


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

    def get_command_message(self):
        angle_strs: list[str] = []
        for leg in self.legs.values():
            servos: list[str] = []
            servos.append(leg.coxa_control.get_command_message())
            servos.append(leg.femur_control.get_command_message())
            servos.append(leg.tibia_control.get_command_message())
            for servo in servos:
                if servo:
                    angle_strs.append(servo)

        return ",".join(angle_strs)


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
        tib_a = -(tib_a - 180)

        # Set joint angles and update frames
        self.coxa_control.set_angle(cox_a)
        self.femur_control.set_angle(fem_a)
        self.tibia_control.set_angle(tib_a)
        self.coxa_frame.rotation = Rotation.degrees(z=cox_a)
        # negative because in 3d world, clockwise looking from -y is a
        # positive rotation angle.
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
        cox_len = self.coxa_length
        fem_len = self.femur_length
        tib_len = self.tibia_length

        vec2 = Vec2d(position.x, position.y)

        a1 = vec2.angle_x()

        xy_len = max(0, vec2.length() - cox_len)
        zH = Vec2d(position.z, xy_len).length()

        max_reach = fem_len + tib_len
        # Clamp reach distance to maximum if it exceeds (with small tolerance)
        if zH > max_reach + 1e-6:
            # Scale the position vector to be within reach
            # Keep the direction, just reduce the magnitude
            position_mag = position.length()
            if position_mag > 0:
                # Calculate what the max reachable distance is from coxa
                # TODO: This doesn't factor in the rigid Z axis of the coxa.
                direction = position.normalize()
                new_position = direction * (cox_len + max_reach)
                # Recalculate after scaling
                xy_len = max(
                    0, Vec2d(new_position.x, new_position.y).length() - cox_len
                )
                zH = Vec2d(new_position.z, xy_len).length()
                logger.warning(
                    f"Leg {self.id} maximum reach attempted! Clamping {position} to {new_position}"
                )
                position = new_position

        a2cos = (fem_len**2 + zH**2 - tib_len**2) / (2 * fem_len * zH)
        a2 = math.acos(max(-1.0, min(1.0, a2cos)))
        a2 = a2 + math.atan2(position.z, xy_len)

        a3cos = (fem_len**2 + tib_len**2 - zH**2) / (2 * tib_len * fem_len)
        a3 = math.acos(max(-1.0, min(1.0, a3cos)))

        return [math.degrees(a) for a in [a1, a2, a3]]
