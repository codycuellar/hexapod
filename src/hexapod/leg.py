from hexapod.utils import Coord3D
import math
from servo import Servo

class ServoConfig:
    def __init__(self, pin_number, upper_limit, lower_limit, zeroed_angle, inverted = False):
        self.pin_number = pin_number
        self.upper_limit = upper_limit
        self.lower_limit = lower_limit
        self.zeroed_angle = zeroed_angle
        self.inverted = inverted

class Leg:
    """
    The leg relative to the ground plane, y axis running front-to-back of the 
    body, x perpindicular, and z up and down perpendicular to the ground plane.
    """

    """Offsets for the pivot base of the leg. """
    translation_offsets = { "x": 0, "y": 0, "z": 0 }
    rotation_offsets = { "x": 0, "y": 0, "z": 0 }

    def __init__(self, coxa: ServoConfig, femur: ServoConfig, tibia: ServoConfig, 
                 coxa_len=50, femur_len=50, tibia_len=50, 
                 leg_rotation_offset=0, leg_translation_offset=None):
        """
        Initialize the leg with servo pin numbers for each joint (coxa, femur, tibia). Offsets
        specify the angle of the coordinate plane relative to the body and ground plane when 
        the servo is zeroed out on a scale of -90 to 90.
        """
        self.scoxa = Servo(coxa.pin_number)
        self.scoxa.enable()
        self.sfemur = Servo(femur.pin_number)
        self.sfemur.enable()
        self.stibia = Servo(tibia.pin_number)
        self.stibia.enable()

        self.coxa_config = coxa
        self.femur_config = femur
        self.tibia_config = tibia

        self.coxa_len = coxa_len
        self.femur_len = femur_len
        self.tib_len = tibia_len

        self.leg_rotation_offset = leg_rotation_offset
        self.leg_translation_offset = leg_translation_offset

    def set_position(self, position: Coord3D):
        pos = Coord3D(position.x, position.y, position.z)
        angles = self._calculate_ik(pos)
        angles = self._normalize_relative_angles(*angles)
        return self._set_servo_angles(*angles)

    def get_angles(self):
        return (self.scoxa.value(), self.sfemur.value(), self.stibia.value())
    
    def set_angles(self, s1, s2, s3):
        self.scoxa.value(s1)
        self.sfemur.value(s2)
        self.stibia.value(s3)

    def zero_servos(self):
        self._set_servo_angles(0, 0, 0)

    def _apply_leg_offset(self, position: Coord3D):
        """
        Applies the rotation offset to the position of the leg in 3d space.
        """

        # Apply rotation around the z-axis (yaw)
        print('position before offset:', position.as_list())
        x = position.x
        y = position.y
        theta = math.radians(self.leg_rotation_offset)
        position.x = x * math.cos(theta) - y * math.sin(theta)
        position.y = x * math.sin(theta) + y * math.cos(theta)
        print('position after offset:', position.as_list())
        if self.leg_translation_offset:
            position.x += self.leg_translation_offset.x
            position.y += self.leg_translation_offset.y
            position.z += self.leg_translation_offset.z
        print('position after translation:', position.as_list())

    def _calculate_ik(self, position: Coord3D):
        """
        Calculates the angles from the leg hip joint to the tip point in 3d space,
        with x axis being parallel to the ground plane, perpindicular to the body.
        Z is up/down, X is in/out from body, y is forward/backwards.
        """
        self._apply_leg_offset(position)

        a1 = math.degrees(math.atan2(position.y, position.x))
        xyH = max(0, math.sqrt(position.y**2 + position.x**2) - self.coxa_len)
        zH = math.sqrt(position.z**2 + xyH**2)
        z_theta = math.atan2(position.z, xyH)
        a2cos = (self.femur_len**2 + zH**2 - self.tib_len**2) / (2 * self.femur_len * zH)
        a2 = math.degrees(math.acos(a2cos) + z_theta)

        a3 = math.degrees(math.acos(
            (self.femur_len**2 + self.tib_len**2 - zH**2) / (2 * self.tib_len * self.femur_len)))
        
        return (a1, a2, a3)

    def _normalize_relative_angles(self, a1, a2, a3):
        if not self.coxa_config:
            coxa_angle = a1 - self.coxa_config.zeroed_angle
        else:
            coxa_angle = self.coxa_config.zeroed_angle - a1
        coxa_limits = (self.coxa_config.upper_limit, self.coxa_config.lower_limit)
        coxa_angle = self._clamp(coxa_angle, min(coxa_limits), max(coxa_limits))

        if not self.femur_config.inverted:
            femur_angle = a2 - self.femur_config.zeroed_angle
        else:
            femur_angle = self.femur_config.zeroed_angle - a2
        femur_limits = (self.femur_config.upper_limit, self.femur_config.lower_limit)
        femur_angle = self._clamp(femur_angle, min(femur_limits), max(femur_limits))

        if not self.tibia_config.inverted:
            tibia_angle = a3 - self.tibia_config.zeroed_angle
        else:
            tibia_angle = self.tibia_config.zeroed_angle - a3
        tibia_limits = (self.tibia_config.upper_limit, self.tibia_config.lower_limit)
        tibia_angle = self._clamp(tibia_angle, min(tibia_limits), max(tibia_limits))

        return (coxa_angle, femur_angle, tibia_angle)

    def _clamp(self, value, min_val, max_val):
        return max(min(value, max_val), min_val)
    
    def _set_servo_angles(self, s1, s2, s3):
        self.scoxa.value(s1)
        self.sfemur.value(s2)
        self.stibia.value(s3)
        # print(f"Setting angles: {s1}, {s2}, {s3}")
        return (s1, s2, s3)
