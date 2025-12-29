import math

from hexapod.engine import Frame, Transform, Vec3d, Rotation
from enum import Enum


class LegID(Enum):
    LF = 0
    LM = 1
    LB = 2
    RF = 3
    RM = 4
    RB = 5


class Body:
    def __init__(
        self,
        frame: Frame,
        legs: dict[LegID, "Leg"],
    ):
        """
        Initialize the hexapod body.

        :param frame: Body coordinate frame
        :param legs: Dictionary of legs by LegID
        """
        self.frame = frame
        self.legs = legs

        for key in self.legs:
            self.legs[key].coxa.frame.parent = self.frame

    def set_foot_position(self, leg_id: LegID, position: Vec3d):
        """
        Set foot position for a specific leg.
        Position is in body-relative coordinates.
        """
        leg = self.legs[leg_id]
        # Convert body-relative position to leg-relative (coxa frame)
        leg_local_pos = leg.frame.world_to_local(self.frame.local_to_world(position))
        leg.set_foot_pos(leg_local_pos)

    def get_foot_position(self, leg_id: LegID) -> Vec3d:
        """
        Get current foot position in body-relative coordinates.
        """
        leg = self.legs[leg_id]
        # Get foot position in world/body frame
        return leg.foot_frame.get_local_position_in(self.frame)

    def update_all(self):
        """Update all leg controllers."""
        for leg in self.legs.values():
            leg.update()


class Joint:
    def __init__(
        self,
        control,  # Any object with set_angle() and update() methods
        joint_axis: str,
        length_to_child: float,
    ):
        self.frame = Frame()
        self.joint_axis = joint_axis
        self.control = control
        self.length_to_child = length_to_child

    def set_angle(self, frame_angle: float):
        """
        Set the joint angle. Updates both control and frame.
        :param frame_angle: Angle in frame space (degrees).
        """
        # Update control
        x = frame_angle if self.joint_axis == "x" else 0.0
        y = frame_angle if self.joint_axis == "y" else 0.0
        z = frame_angle if self.joint_axis == "z" else 0.0
        self.frame.rotate(Rotation.degrees(x, y, z))
        self.control.set_angle(frame_angle)

    def get_angle(self) -> float:
        """Get current joint angle in frame space."""
        return self.control.get_frame_angle()

    # def sync_from_control(self):
    #     """
    #     Sync frame to match current control position.
    #     Reads raw control angle, converts to frame angle, updates frame only.
    #     Use this on startup to align frames with actual control positions.
    #     Does not modify the control - it already has the correct value.
    #     """
    #     # Read raw control angle
    #     raw_angle = self.control.get_control_angle()

    #     # Convert to frame angle
    #     # For Servo: raw_angle is servo angle, needs conversion
    #     # For MockControl: raw_angle is already frame angle (get_control_angle returns frame_angle)
    #     if hasattr(self.control, 'cluster'):
    #         # It's a Servo - convert servo angle to frame angle
    #         frame_angle = self.control.calibration.servo_to_frame_angle(raw_angle)
    #     else:
    #         # It's a MockControl - raw_angle is already frame angle
    #         frame_angle = raw_angle

    #     # Update frame to match (don't update control - it already has this value)
    #     self._update_frame_from_angle(frame_angle)

    #     # Also update control's frame_angle to match (for consistency)
    #     self.control.frame_angle = frame_angle

    def update(self):
        """
        Update hardware to match current angle, and ensure frame matches.
        This is called to send commands to hardware.
        """
        # Get current angle from control
        current_angle = self.control.get_frame_angle()

        # # Ensure frame matches (in case it was modified externally)
        # self._update_frame_from_angle(current_angle)

        # Send to hardware
        self.control.update()

    # def _update_frame_from_angle(self, angle: float):
    #     """Update frame rotation based on current angle and rotation axis."""
    #     # For mount frames, we need to preserve the mount rotation and compose the joint rotation
    #     # For non-mount frames, we just set the rotation directly
    #     if self.is_mount_frame and self._mount_base_rotation is not None:
    #         # Mount frame: compose mount base rotation with joint rotation
    #         if self.rotation_axis == "X":
    #             joint_rot = Rotation.degrees(angle, 0, 0)
    #         elif self.rotation_axis == "Y":
    #             joint_rot = Rotation.degrees(0, angle, 0)
    #         elif self.rotation_axis == "Z":
    #             joint_rot = Rotation.degrees(0, 0, angle)
    #         else:
    #             raise ValueError(f"Invalid rotation_axis: {self.rotation_axis}")
    #         # Compose: mount_base_rotation @ joint_rotation
    #         self.frame.rotation = self._mount_base_rotation @ joint_rot
    #     else:
    #         # Non-mount frame or mount rotation not set yet: just set the rotation directly
    #         if self.rotation_axis == "X":
    #             self.frame.rotation = Rotation.degrees(angle, 0, 0)
    #         elif self.rotation_axis == "Y":
    #             self.frame.rotation = Rotation.degrees(0, angle, 0)
    #         elif self.rotation_axis == "Z":
    #             self.frame.rotation = Rotation.degrees(0, 0, angle)
    #         else:
    #             raise ValueError(f"Invalid rotation_axis: {self.rotation_axis}")

    #     # Update origin (joint extends along X axis)
    #     # Don't set origin for mount frames (coxa) - they already have the correct mount position
    #     if not self.is_mount_frame:
    #         self.frame.origin = Vector([self.length_to_child, 0, 0])


class Leg:
    def __init__(
        self, id: LegID, coxa: Joint, femur: Joint, tibia: Joint, mount_frame: Frame
    ):
        self.id = id
        self.name = LegID(id)
        self.coxa = coxa
        self.femur = femur
        self.tibia = tibia

        self.femur.frame.position = Vec3d(self.coxa.length_to_child, 0, 0)
        self.tibia.frame.position = Vec3d(self.femur.length_to_child, 0, 0)
        self.foot_frame = Frame(position=Vec3d(self.tibia.length_to_child, 0, 0))

        self.coxa.frame = mount_frame
        self.femur.frame.parent = self.coxa.frame
        self.tibia.frame.parent = self.femur.frame
        self.foot_frame.parent = self.tibia.frame

    @property
    def frame(self) -> Frame:
        return self.coxa.frame

    @frame.setter
    def frame(self, value: Frame):
        self.coxa.frame = value
        self.femur.frame.parent = self.coxa.frame

    def set_foot_pos(self, position: Vec3d):
        """
        Set the foot position relative to the leg's coxa frame.
        :param position: Position vector in the coxa frame's coordinate system.
        """
        # Calculate IK angles for the desired position
        angles = self._calculate_ik(position)

        # Set joint angles and update frames
        self.coxa.set_angle(angles[0])
        self.femur.set_angle(angles[1])
        self.tibia.set_angle(angles[2])

        # # Update frame rotations and origins based on angles
        # self._update_frames_from_angles()

    def update(self):
        """Update all joint controllers (send commands to hardware)."""
        self.coxa.update()
        self.femur.update()
        self.tibia.update()

    # def _update_frames_from_angles(self):
    #     """Update frame rotations based on current joint angles."""
    #     # Update coxa frame rotation (rotate around Z axis)
    #     coxa_angle = self.coxa.get_angle()
    #     # Get mount rotation (stored when frame was set)
    #     if hasattr(self.coxa, "_mount_base_rotation"):
    #         mount_rot = self.coxa._mount_base_rotation
    #     else:
    #         # Store mount rotation if not already stored
    #         self.coxa._mount_base_rotation = self.coxa.frame.rotation
    #         mount_rot = self.coxa.frame.rotation
    #     # Compose mount rotation with coxa joint rotation
    #     joint_rot = Rotation.degrees(0, 0, coxa_angle)
    #     self.coxa.frame.rotation = mount_rot @ joint_rot

    #     # Update femur frame (rotate around Y axis)
    #     femur_angle = self.femur.get_angle()
    #     self.femur.frame.rotation = Rotation.degrees(0, femur_angle, 0)
    #     # Origin is in parent's local coordinates (doesn't change with rotation)
    #     self.femur.frame.origin = Vector([self.coxa.length_to_child, 0, 0])

    #     # Update tibia frame (rotate around Y axis)
    #     tibia_angle = self.tibia.get_angle()
    #     self.tibia.frame.rotation = Rotation.degrees(0, tibia_angle, 0)
    #     # Origin is in parent's local coordinates
    #     self.tibia.frame.origin = Vector([self.femur.length_to_child, 0, 0])

    #     # Update foot frame (no rotation, just position)
    #     self.foot_frame.origin = Vector([self.tibia.length_to_child, 0, 0])

    def _calculate_ik(self, position: Vec3d) -> tuple[float, float, float]:
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

        max_reach = fem_len + tib_len
        # Clamp reach distance to maximum if it exceeds (with small tolerance)
        if zH > max_reach + 1e-6:
            # Scale the position vector to be within reach
            # Keep the direction, just reduce the magnitude
            position_mag = math.sqrt(position.x**2 + position.y**2 + position.z**2)
            if position_mag > 0:
                # Calculate what the max reachable distance is from coxa
                max_position_mag = cox_len + max_reach
                scale_factor = max_position_mag / position_mag
                position = Vec3d(
                    position.x * scale_factor,
                    position.y * scale_factor,
                    position.z * scale_factor,
                )
                # Recalculate after scaling
                xyH = max(0, math.sqrt(position.y**2 + position.x**2) - cox_len)
                zH = math.sqrt(position.z**2 + xyH**2)

        z_theta = math.atan2(position.z, xyH)
        a2cos = (fem_len**2 + zH**2 - tib_len**2) / (2 * fem_len * zH)
        a2cos = max(-1.0, min(1.0, a2cos))

        a2 = math.degrees(math.acos(a2cos) + z_theta)

        a3cos = (fem_len**2 + tib_len**2 - zH**2) / (2 * tib_len * fem_len)
        a3cos = max(-1.0, min(1.0, a3cos))
        a3 = math.degrees(math.acos(a3cos))

        return (a1, a2, a3)
