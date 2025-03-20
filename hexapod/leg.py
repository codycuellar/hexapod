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

    """ (forward, backward) absolute angle limits for the coxa joint. """
    coxa_limits = (80, -80)
    """ (upward, downward) absolute angle limits for the femur joint. """
    femur_limits = (-47, 90)
    femur_offset = 27.65
    """ (upward, downward) absolute angle limits for the tibia joint. """
    tibia_limits = (-70, 90)

    def __init__(self, coxa: ServoConfig, femur: ServoConfig, tibia: ServoConfig, femur_len=50, tibia_len=50):
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

        self.femur_len = femur_len
        self.tib_len = tibia_len

    def set_position(self, x, y, z):
        angles = self._calculate_ik(x, y, z)
        angles = self._normalize_relative_angles(*angles)
        return self._set_servo_angles(*angles)
    
    def set_angles(self, s1, s2, s3):
        self.scoxa.value(s1)
        self.sfemur.value(s2)
        self.stibia.value(s3)

    def zero_servos(self):
        self._set_servo_angles(0, 0, 0)

    def _calculate_ik(self, x, y, z):
        """
        Calculates the angles from the leg hip joint to the tip point in 3d space,
        with x axis being parallel to the ground plane, perpindicular to the body.
        Z is up/down, X is in/out from body, y is forward/backwards.
        """
        a1 = math.degrees(math.atan2(y, x))
        xyL = math.sqrt(y**2 + x**2)
        zH = math.sqrt(z**2 + xyL**2)
        z_theta = math.atan2(z, xyL)
        a2cos = (self.femur_len**2 + zH**2 - self.tib_len**2) / (2 * self.femur_len * zH)
        a2 = math.degrees(math.acos(a2cos) + z_theta)

        a3 = math.degrees(math.acos(
            (self.femur_len**2 + self.tib_len**2 - zH**2) / (2 * self.tib_len * self.femur_len)))
        
        return (a1, a2, a3)

    def _normalize_relative_angles(self, a1, a2, a3):
        coxa_angle = a1 - self.coxa_config.zeroed_angle if not self.coxa_config.inverted else self.coxa_config.zeroed_angle - a1
        coxa_angle = self._clamp(coxa_angle, min(self.coxa_limits), max(self.coxa_limits))
        print('a2', a2, 'femur_zero', self.femur_config.zeroed_angle, 'inverted', self.femur_config.inverted)
        femur_angle = a2 - self.femur_config.zeroed_angle if not self.femur_config.inverted else self.femur_config.zeroed_angle - a2
        femur_angle = self._clamp(femur_angle, min(self.femur_limits), max(self.femur_limits))

        tibia_angle = 90 - a3 - self.tibia_config.zeroed_angle if not self.tibia_config.inverted else self.tibia_config.zeroed_angle - (90 - a3)
        tibia_angle = self._clamp(tibia_angle, min(self.tibia_limits), max(self.tibia_limits))

        print('normalized angles:', coxa_angle, femur_angle, tibia_angle)
        return (coxa_angle, femur_angle, tibia_angle)

    def _clamp(self, value, min_val, max_val):
        return max(min(value, max_val), min_val)
    
    def _set_servo_angles(self, s1, s2, s3):
        # self.scoxa.value(s1)
        # self.sfemur.value(s2)
        # self.stibia.value(s3)
        return (s1, s2, s3)
