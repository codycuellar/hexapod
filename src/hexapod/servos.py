"""
Joint actuators for hexapod control.

This module provides different actuator implementations:
- Servo: Hardware servo control
- MockControl: Simulation/testing control (no hardware)
- VisualizationControl: Real-time 3D visualization control
"""


class JointCalibration:
    """
    Helper class for frame-to-servo angle conversion.
    Handles calibration parameters and conversions.
    """
    
    def __init__(
        self,
        zero_offset: float = 0.0,
        min_angle: float = -90.0,
        max_angle: float = 90.0,
        inverted: bool = False,
    ):
        """
        Initialize calibration parameters.
        
        :param zero_offset: Servo angle (degrees) when frame angle = 0
        :param min_angle: Minimum frame angle (degrees)
        :param max_angle: Maximum frame angle (degrees)
        :param inverted: Whether servo direction is inverted
        """
        self.zero_offset = zero_offset
        self.min_angle = min_angle
        self.max_angle = max_angle
        self.inverted = inverted
    
    def frame_to_servo_angle(self, frame_angle: float) -> float:
        """Convert frame angle to servo angle."""
        if self.inverted:
            return self.zero_offset - frame_angle
        else:
            return self.zero_offset + frame_angle
    
    def servo_to_frame_angle(self, servo_angle: float) -> float:
        """Convert servo angle to frame angle."""
        if self.inverted:
            return self.zero_offset - servo_angle
        else:
            return servo_angle - self.zero_offset
    
    def clamp_frame_angle(self, angle: float) -> float:
        """Clamp frame angle to limits."""
        return max(min(angle, self.max_angle), self.min_angle)


class Servo:
    """
    Hardware servo implementation.
    
    Converts frame angles to servo angles and sends commands to hardware.
    """
    
    @staticmethod
    def create_cluster(pin_numbers: list):
        """Create a servo cluster for hardware control."""
        from servo import ServoCluster as RawCluster
        return RawCluster(0, 0, pin_numbers)
    
    def __init__(
        self,
        cluster,
        pin_number: int,
        zero_offset: float,
        min_angle: float,
        max_angle: float,
        inverted: bool = False,
    ):
        self.cluster = cluster
        self.pin_number = pin_number
        self.calibration = JointCalibration(zero_offset, min_angle, max_angle, inverted)
        self.frame_angle: float = 0.0
    
    def set_angle(self, frame_angle: float):
        """
        Set the angle in frame space (degrees).
        Angle is clamped to limits automatically.
        """
        self.frame_angle = self.calibration.clamp_frame_angle(frame_angle)
    
    def get_frame_angle(self) -> float:
        """Get current angle in frame space."""
        return self.frame_angle
    
    def get_control_angle(self) -> float:
        """
        Get the current raw control angle from hardware.
        For Servo, this reads the actual servo position.
        Returns the raw servo angle (not frame angle).
        """
        # In a real implementation, this would read from hardware:
        # return self.cluster.read_angle(self.pin_number)
        # For now, calculate from stored frame_angle
        return self.calibration.frame_to_servo_angle(self.frame_angle)
    
    def update(self):
        """
        Send the current frame angle to the servo hardware.
        Converts frame angle to servo angle and sends command.
        """
        servo_angle = self.calibration.frame_to_servo_angle(self.frame_angle)
        return self.cluster.value(self.pin_number, servo_angle)

class MockServo:
    """
    Minimal control for simulation that just tracks frame angles.
    No calibration, no hardware - just angle tracking.
    """
    
    def __init__(self, initial_angle: float = 0.0):
        self.frame_angle: float = initial_angle
    
    def set_angle(self, frame_angle: float):
        """Set the angle in frame space (degrees)."""
        self.frame_angle = frame_angle
    
    def get_frame_angle(self) -> float:
        """Get current angle in frame space."""
        return self.frame_angle
    
    def get_control_angle(self) -> float:
        """Get control angle (same as frame angle for simple control)."""
        return self.frame_angle
    
    def update(self):
        """Update method (no-op for simulation)."""
        pass

