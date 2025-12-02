import math

from hexapod.engine import Frame, Transform, Vector, Rotation
from hexapod.joint_control import RawJointControl
from enum import Enum


class LegID(Enum):
    LF = 0
    LM = 1
    LR = 2
    RF = 3
    RM = 4
    RR = 5


class Body:
    def __init__(self, frame: Frame, legs: dict[LegID, "Leg"]):
        """legs are ordered in right front, clockwise around the body."""
        self.frame = frame

        for key in legs:
            legs[key].frame.parent = frame

        self.legs = legs
        self.foot_frames = self._set_foot_frames(legs)

    def go_to_home(self):
        """
        When powered on, leg positions cannot accurately be read if they
        have been physically moved, so this assumes the body is in contact with
        the ground, and we can home the legs to a known position off the ground.
        This will happen as fast as the servos can move, so it's the safest way to
        initialize known positions.
        """
        pass

    def update(self):
        pass

    def update_velocity(self, velocity: Vector):
        self.current_velocity = velocity.normalize()

    def change_gait(self, gait=None):
        """
        Change the gait of the hexapod. If no gait is specified, it will cycle to the next one.
        param gait: The name of the gait to switch to from Body.gaits.
                    If None, cycles to the next gait.
        """
        if gait is None:
            next_gait = (self.gaits.index(self.current_gait) + 1) % len(self.gaits)
            self.current_gait = self.gaits[next_gait]

        if gait not in self.gaits:
            raise ValueError(f"Gait {gait} not found.")

        self.current_gait = gait

    def _set_foot_frames(self, legs: dict[str, Leg]):
        foot_frames = {}
        for name, leg in legs.items():
            leg.change_global_position(self.relative_position)
            base = leg.pos_from_global
            offset = leg.coxa_len + (leg.femur_len + leg.tib_len) / 2
            foot_frames[name] = (
                base.position() + base.translation().normalize() * offset
            )
        return foot_frames

    def _move(self, t: float):
        if self.current_gait == "tripod":
            self._tripod_gait(t)
        else:
            raise ValueError(f"Gait {self.current_gait} not implemented.")

    def _tripod_gait(self, t: float):
        """
        Move the hexapod in a tripod gait pattern.
        """
        pass


class Joint:
    def __init__(self, frame: Frame, control: RawJointControl, length_to_child: float):
        super().__init__()
        self.frame = frame
        self.control = control
        self.length_to_child = length_to_child


class Leg:
    def __init__(
        self,
        id: LegID,
        frame: Frame,
        coxa: Joint,
        femur: Joint,
        tibia: Joint,
    ):
        """
        Initialize the leg with servo pin numbers for each joint (coxa, femur, tibia). Offsets
        specify the angle of the Pointinate plane relative to the body and ground plane when
        the servo is zeroed out on a scale of -90 to 90.
        """
        self.id = id
        self.frame = frame

        self.coxa = coxa
        self.femur = femur
        self.tibia = tibia
        self.foot = (
            Frame()
        )  # how do we know where this is initially based on the angles and lengths of components?
        self.coxa.frame.parent = self.frame
        self.femur.frame.parent = self.coxa.frame
        self.tibia.frame.parent = self.femur.frame
        self.foot.parent = self.tibia.frame

    def set_foot_pos(self, position: Vector):
        foot_local = self.foot.to_local_position(position, self.coxa.frame)

        angles = self._calculate_ik(position)
        self._set_servo_angles(*angles)
        self._update()

    def set_angles(self, s1, s2, s3):
        if self.enabled == False:
            return
        self.coxa.control.set_angle(s1)
        self.femur.control.set_angle(s2)
        self.tibia.control.set_angle(s3)

    def enable(self):
        self.enabled = True

    def disable(self):
        self.enabled = False

    def zero_servos(self):
        if self.enabled == False:
            return
        self._set_servo_angles(0, 0, 0)

    def _calculate_ik(self, position: Vector) -> tuple[float, float, float]:
        """
        Calculates the angles from the leg hip joint to the tip point in 3d space,
        with x axis being parallel to the ground plane, perpindicular to the mount point.
        """
        cox_len = self.coxa.length_to_child
        fem_len = self.femur.length_to_child
        tib_len = self.tibia.length_to_child

        a1 = math.degrees(math.atan2(position.y, position.x))

        xyH = max(0, math.sqrt(position.y**2 + position.x**2) - cox_len)
        zH = math.sqrt(position.z**2 + xyH**2)
        if (fem_len + tib_len) <= zH:
            raise ValueError(f"Reach distance {zH} exceeds femur + tibia length.")
        z_theta = math.atan2(position.z, xyH)
        a2cos = (fem_len**2 + zH**2 - tib_len**2) / (2 * fem_len * zH)
        a2 = math.degrees(math.acos(a2cos) + z_theta)

        a3 = math.degrees(
            math.acos((fem_len**2 + tib_len**2 - zH**2) / (2 * tib_len * fem_len))
        )

        return (a1, a2, a3)

    def _clamp(self, value, min_val, max_val):
        """Ensures the servo stays within it's calibrated limits."""
        return max(min(value, max_val), min_val)

    def _set_servo_angles(self, ca, fa, ta):
        """Convert IK angles to raw angle positions and update the servo."""
        # s1 = self.coxa.control.get_raw_value(a1)
        # s2 = self.femur.control.get_raw_value(a2)
        # s3 = self.tibia.control.get_raw_value(a3)
        return (
            self.coxa.control.set_angle(ca),
            self.femur.control.set_angle(fa),
            self.tibia.control.set_angle(ta),
        )

    def _update(self):
        self.coxa.control.update()
        self.femur.control.update()
        self.tibia.control.update()
