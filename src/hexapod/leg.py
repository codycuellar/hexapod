from hexapod.utils import Coord3D
import math
from servo import Servo

class ServoConfig:
    def __init__(self, pin_number, upper_limit, lower_limit, zeroed_angle, inverted = False):
        """
        Configuration for a servo motor, including pin number, limits, and zeroed angle.
        param pin_number: Pin number for the servo motor.
        param upper_limit: Upper limit for the servo motor angle.
        param lower_limit: Lower limit for the servo motor angle.
        param zeroed_angle: Angle at which the servo motor is considered zeroed.
        param inverted: Boolean indicating if the servo motor is inverted.
        """
        self.pin_number = pin_number
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
    
    def get_raw_angle(self, desired_angle):
        """
        Convert a body-relative angle to the actual servo command angle.

        :param desired_angle: The target angle relative to the body.
        :return: The raw servo angle.
        """
        # Offset the desired angle by the zeroed angle (since it's the real position of the servo when at 0)
        if self.inverted:
            # Inverted servo: subtract the desired angle from the zeroed angle
            return self._clamp(self.zeroed_angle - desired_angle)
        else:
            # Non-inverted servo: add the desired angle to the zeroed angle
            return self._clamp(desired_angle - self.zeroed_angle)
    
    def _clamp(self, value):
        """
        Clamp the value to the servo motor limits.
        param value: Value to be clamped.
        return: Clamped value within the servo motor limits.
        """
        return max(min(value, self.pos_limit), self.neg_limit)

class Leg:
    """
    The leg relative to the ground plane, y axis running front-to-back of the 
    body, x perpindicular, and z up and down perpendicular to the ground plane.
    """

    """Offsets for the pivot base of the leg. """
    translation_offsets = { "x": 0, "y": 0, "z": 0 }
    enabled = True

    def __init__(self, coxa: ServoConfig, femur: ServoConfig, tibia: ServoConfig, 
                 coxa_len=50, femur_len=50, tibia_len=50):
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

    def set_position(self, position: Coord3D):
        if self.enabled == False:
            return
        pos = Coord3D(position.x, position.y, position.z) # make a copy
        angles = self._calculate_ik(pos)
        coxa_angle = self.coxa_config.get_raw_angle(angles[0])
        femur_angle = self.femur_config.get_raw_angle(angles[1])
        tibia_angle = self.tibia_config.get_raw_angle(angles[2])
        return self._set_servo_angles(coxa_angle, femur_angle, tibia_angle)

    def get_angles(self):
        return (self.scoxa.value(), self.sfemur.value(), self.stibia.value())
    
    def set_angles(self, s1, s2, s3):
        if self.enabled == False:
            return
        self.scoxa.value(s1)
        self.sfemur.value(s2)
        self.stibia.value(s3)
    
    def enable(self):
        self.enabled = True
    
    def disable(self):
        self.enabled = False

    def zero_servos(self):
        if self.enabled == False:
            return
        self._set_servo_angles(0, 0, 0)

    def _calculate_ik(self, position: Coord3D):
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
        a2cos = (self.femur_len**2 + zH**2 - self.tib_len**2) / (2 * self.femur_len * zH)
        a2 = math.degrees(math.acos(a2cos) + z_theta)
        a3 = math.degrees(math.acos(
            (self.femur_len**2 + self.tib_len**2 - zH**2) / (2 * self.tib_len * self.femur_len)))
        
        return (a1, a2, a3)

    def _clamp(self, value, min_val, max_val):
        return max(min(value, max_val), min_val)
    
    def _set_servo_angles(self, s1, s2, s3):
        self.scoxa.value(s1)
        self.sfemur.value(s2)
        self.stibia.value(s3)
        # print(f"Setting angles: {s1}, {s2}, {s3}")
        return (s1, s2, s3)
