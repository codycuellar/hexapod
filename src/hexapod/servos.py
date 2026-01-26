"""
Joint actuators for hexapod control.

This module provides different actuator implementations:
- Servo: Hardware servo control
- MockControl: Simulation/testing control (no hardware)
- VisualizationControl: Real-time 3D visualization control
"""

from typing import TypeAlias, Dict


ServoAngles: TypeAlias = Dict[int, float]


class JointControl:
    pin_number = 0

    def set_angle(self, frame_angle: float):
        return

    def get_angle(self) -> float:
        return 0.0


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


class Servo(JointControl):
    """
    Hardware servo implementation.

    Converts frame angles to servo angles and sends commands to hardware.
    """

    def __init__(
        self,
        pin_number: int,
        raw_zero_offset: float,
        min_angle: float = -90,
        max_angle: float = 90,
        inverted: bool = False,
    ):
        self.pin_number = pin_number
        self.calibration = JointCalibration(
            raw_zero_offset, min_angle, max_angle, inverted
        )
        self.frame_angle: float = 0.0

    def set_angle(self, frame_angle: float):
        """
        Set the angle in frame space (degrees).
        Angle is clamped to limits automatically.
        """
        self.frame_angle = frame_angle

    def get_angle(self) -> float:
        servo_angle = self.calibration.frame_to_servo_angle(self.frame_angle)
        return self.calibration.clamp_frame_angle(servo_angle)


class MockServo(JointControl):
    """
    Minimal control for simulation that just tracks frame angles.
    No calibration, no hardware - just angle tracking.
    """

    def __init__(self, initial_angle: float = 0.0):
        self.frame_angle: float = initial_angle

    def set_angle(self, frame_angle: float):
        """Set the angle in frame space (degrees)."""
        self.frame_angle = frame_angle

    def get_angle(self) -> float:
        """Get control angle (same as frame angle for simple control)."""
        return self.frame_angle
