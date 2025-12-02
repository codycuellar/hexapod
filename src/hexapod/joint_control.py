class RawJointControl:
    def __init__(self):
        self.angle: float = 0

    def set_angle(self, angle: float):
        """Set the angle in degrees of the joint controller"""
        self.angle = angle

    def update(self):
        """Move the joint to the current angle."""
        pass
