from servo import ServoCluster as RawCluster

class Servo:
    
    @staticmethod
    def create_cluster(pin_numbers:list):
        return RawCluster(0, 0, pin_numbers)

    def __init__(self, name:str, cluster:RawCluster, pin_number:int, upper_limit:int, lower_limit:int, zeroed_angle:int, inverted = False):
        """
        Configuration for a servo motor.
        param name: The servo name for logging purposes.
        param cluster: The servo cluster class to which the servo motor belongs.
        param pin_number: GPIO Pin number for the servo motor to request servo updates from the cluster.
        param upper_limit: Upper limit for the servo motor angle.
        param lower_limit: Lower limit for the servo motor angle.
        param zeroed_angle: Physical angle of the kinematics refernce frame when the servo is at 0 degrees.
        param inverted: Boolean indicating if the servo degrees are inverted from the specified upper and lower limits.
        """
        self.name = name
        self.pin_number = pin_number
        self.cluster = cluster
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
        return self.cluster.value(self.pin_number, angle)

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
