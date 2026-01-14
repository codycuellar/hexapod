import math

from hexapod.servos import JointControl
from hexapod.engine import Frame, Vec3d, Vec2d, Rotation
from enum import Enum


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

    def update(self):
        """Update all leg controllers."""
        for leg in self.legs.values():
            leg.update()


class Leg:
    def __init__(
        self,
        leg_id: LegID,
        mount_frame: Frame,
        coxa_length: float,
        femur_length: float,
        tibia_length: float,
        coxa_control: JointControl,
        femur_control: JointControl,
        tibia_control: JointControl,
    ):
        """
        Docstring for __init__
        :param leg_id: The ID for the leg.
        :param mount_frame: The coordinate frame of the coxa mount point.
        :param standing_foot_pos:
            The coxa-frame relative position of the foot for neutral standing
            position.
        :param coxa_length: Description
        :param femur_length: Description
        :param tibia_length: Description
        :param coxa_control: Description
        :param femur_control: Description
        :param tibia_control: Description
        """
        self.id = leg_id
        self.name = leg_id
        self.frame = mount_frame

        self.coxa_length = coxa_length
        self.femur_length = femur_length
        self.tibia_length = tibia_length

        self.coxa_control = coxa_control
        self.femur_control = femur_control
        self.tibia_control = tibia_control

        self.coxa_frame = Frame(parent=self.frame)
        self.femur_frame = Frame(
            origin=Vec3d(coxa_length, 0, 0), parent=self.coxa_frame
        )
        self.tibia_frame = Frame(
            origin=Vec3d(femur_length, 0, 0), parent=self.femur_frame
        )
        self.foot_frame = Frame(
            origin=Vec3d(tibia_length, 0, 0), parent=self.tibia_frame
        )

    def set_foot_pos(self, position: Vec3d):
        """
        Set the foot position relative to the leg's coxa frame which is the mount
        point on the body.
        :param position: Position vector in the coxa frame's coordinate system.
        """
        # Calculate RAW IK angles
        cox_a, fem_a, tib_a = self._calculate_ik(position)

        tib_a = tib_a - 180
        # Set joint angles and update frames
        self.coxa_control.set_angle(cox_a)
        self.femur_control.set_angle(fem_a)
        self.tibia_control.set_angle(tib_a)
        self.coxa_frame.rotation = Rotation.degrees(z=cox_a)
        # negative because in 3d world, counter clockwise looking from -y is a
        # negative rotation angle.
        self.femur_frame.rotation = Rotation.degrees(y=-fem_a)
        self.tibia_frame.rotation = Rotation.degrees(y=-tib_a)

    def update(self):
        """Update all joint controllers (send commands to hardware)."""
        self.coxa_control.update()
        self.femur_control.update()
        self.tibia_control.update()

    def _calculate_ik(self, position: Vec3d) -> list[float]:
        """
        Calculates the angles from the leg hip joint to the tip point in 3d space,
        with x axis being parallel to the ground plane, perpindicular to the mount point.
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
                print(
                    f"Leg {self.id} maximum reach attempted! Clamping {position} to {new_position}"
                )
                position = new_position

        a2cos = (fem_len**2 + zH**2 - tib_len**2) / (2 * fem_len * zH)
        a2 = math.acos(max(-1.0, min(1.0, a2cos)))
        a2 = a2 + math.atan2(position.z, xy_len)

        a3cos = (fem_len**2 + tib_len**2 - zH**2) / (2 * tib_len * fem_len)
        a3 = math.acos(max(-1.0, min(1.0, a3cos)))

        return [math.degrees(a) for a in [a1, a2, a3]]
