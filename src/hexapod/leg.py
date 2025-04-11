from hexapod.utils import Coord, Transform3D
from hexapod.servo import Servo
import math


class Leg:
    enabled = True

    def __init__(
        self,
        name: str,
        coxa: Servo,
        femur: Servo,
        tibia: Servo,
        mount_offset: Transform3D,
        coxa_len=50,
        femur_len=50,
        tibia_len=50,
    ):
        """
        Initialize the leg with servo pin numbers for each joint (coxa, femur, tibia). Offsets
        specify the angle of the coordinate plane relative to the body and ground plane when
        the servo is zeroed out on a scale of -90 to 90.
        """
        self.name = name

        self.mount_offset = mount_offset.invert()
        self.pos_from_global = self.mount_offset

        self.coxa = coxa
        self.femur = femur
        self.tibia = tibia

        self.coxa_len = coxa_len
        self.femur_len = femur_len
        self.tib_len = tibia_len

    def change_global_position(self, transform: Transform3D):
        """
        Change the reference point of this leg with respect to the global coordinate system.
        The leg already knows it's relative transform from the body center, but as the body
        moves, this function will update the body's transform relative to the global origin.
        """
        self.pos_from_global = self.mount_offset.dot(transform.invert())

    def set_position(self, position: Coord):
        """
        Set a position for the leg tip in global space. The leg will compute the relative
        position using the pos_from_global which is updated by the controlling body.
        """
        if self.enabled == False:
            return

        # Get the gloobal coordinate relative to the leg's current position
        # in global space.
        position = self.pos_from_global.apply(position)
        angles = self._calculate_ik(position)
        return self._set_servo_angles(*angles)

    def set_angles(self, s1, s2, s3):
        if self.enabled == False:
            return
        self.coxa.set_angle(s1)
        self.femur.set_angle(s2)
        self.tibia.set_angle(s3)

    def enable(self):
        self.enabled = True

    def disable(self):
        self.enabled = False

    def get_servo_name(self, servo):
        return f"{self.name} {servo.name}"

    def zero_servos(self):
        if self.enabled == False:
            return
        self._set_servo_angles(0, 0, 0)

    def _calculate_ik(self, position: Coord):
        """
        Calculates the angles from the leg hip joint to the tip point in 3d space,
        with x axis being parallel to the ground plane, perpindicular to the mount point.
        """
        a1 = math.degrees(math.atan2(position.y, position.x))

        xyH = max(0, math.sqrt(position.y**2 + position.x**2) - self.coxa_len)
        zH = math.sqrt(position.z**2 + xyH**2)
        if (self.femur_len + self.tib_len) <= zH:
            raise ValueError(f"Reach distance {zH} exceeds femur + tibia length.")
        z_theta = math.atan2(position.z, xyH)
        a2cos = (self.femur_len**2 + zH**2 - self.tib_len**2) / (
            2 * self.femur_len * zH
        )
        a2 = math.degrees(math.acos(a2cos) + z_theta)

        a3 = math.degrees(
            math.acos(
                (self.femur_len**2 + self.tib_len**2 - zH**2)
                / (2 * self.tib_len * self.femur_len)
            )
        )

        return (a1, a2, a3)

    def _clamp(self, value, min_val, max_val):
        """Ensures the servo stays within it's calibrated limits."""
        return max(min(value, max_val), min_val)

    def _set_servo_angles(self, a1, a2, a3):
        """Convert IK angles to raw angle positions and update the servo."""
        s1 = self.coxa.get_raw_angle(a1)
        s2 = self.femur.get_raw_angle(a2)
        s3 = self.tibia.get_raw_angle(a3)
        return (
            self.coxa.set_angle(s1),
            self.femur.set_angle(s2),
            self.tibia.set_angle(s3),
        )
