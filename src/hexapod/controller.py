"""
Hexapod Controller - Handles gait patterns and motion planning.

The controller's job is to:
1. Take high-level commands (velocity, rotation)
2. Generate gait patterns
3. Plan foot trajectories in body-relative coordinates
4. Coordinate leg movements over time
"""

import math
from hexapod.hexpod import Body, LegID
from hexapod.engine import Vector
from hexapod.interpolation import lerp, cosine_ease_t


class HexapodController:
    """
    Controller for hexapod locomotion.
    Handles gait patterns and converts velocity/rotation commands into foot positions.
    """

    def __init__(self, body: Body):
        """
        Create a controller. Call initialize() to sync with hardware and set up positions.
        """
        self.body = body
        self.current_gait = "tripod"
        self.velocity = Vector([0, 0, 0])  # Body-relative velocity (x, y, z)
        self.rotation = Vector([0, 0, 0])  # Body rotation rates (roll, pitch, yaw)

        # Gait timing
        self.gait_phase = 0.0  # 0.0 to 1.0, cycles through gait pattern
        self.gait_speed = 1.0  # Multiplier for gait cycle speed

        # These will be initialized by initialize()
        self.default_foot_positions: dict[LegID, Vector] = {}
        self.target_foot_positions: dict[LegID, Vector] = {}

    def initialize(self):
        """
        Initialize the controller by syncing with hardware and calculating positions.
        Call this after creating the controller to probe controls and set up initial state.
        """
        for id in self.body.legs:
            leg = self.body.legs[id]
            leg.tibia.set_angle(90)

    def set_velocity(self, velocity: Vector):
        """Set desired body-relative velocity."""
        self.velocity = velocity

    def set_rotation(self, rotation: Vector):
        """Set desired body rotation rates (roll, pitch, yaw in deg/s)."""
        self.rotation = rotation

    def set_gait(self, gait: str):
        """Change the current gait pattern."""
        if gait not in ["tripod", "wave", "ripple"]:
            raise ValueError(f"Unknown gait: {gait}")
        self.current_gait = gait

    def update(self, dt: float):
        """
        Update the controller and set foot positions.
        Should be called regularly (e.g., every 20-50ms).

        :param dt: Time delta since last update (in seconds)
        """
        # Normal gait operation
        # Update gait phase
        self.gait_phase = (self.gait_phase + dt * self.gait_speed) % 1.0

        # Generate foot positions based on current gait
        if self.current_gait == "tripod":
            self._tripod_gait(dt)
        elif self.current_gait == "wave":
            self._wave_gait(dt)
        elif self.current_gait == "ripple":
            self._ripple_gait(dt)
        else:
            # Default: just maintain current positions
            self._stand_still()

        # Apply foot positions to body
        for leg_id, position in self.target_foot_positions.items():
            self.body.set_foot_position(leg_id, position)

        # Update all leg controllers
        self.body.update_all()

    def _tripod_gait(self, dt: float):
        """
        Tripod gait: 3 legs on ground, 3 legs in air.
        Legs move in two groups: (LF, RM, RR) and (RF, LM, LR)
        """
        # Determine which legs are in swing phase (lifting/moving)
        # Phase 0.0-0.5: Group 1 swings, Group 2 supports
        # Phase 0.5-1.0: Group 2 swings, Group 1 supports

        swing_group_1 = [LegID.LF, LegID.RM, LegID.RR]
        swing_group_2 = [LegID.RF, LegID.LM, LegID.LR]

        if self.gait_phase < 0.5:
            swing_legs = swing_group_1
            support_legs = swing_group_2
            swing_phase = self.gait_phase * 2.0  # 0.0 to 1.0
        else:
            swing_legs = swing_group_2
            support_legs = swing_group_1
            swing_phase = (self.gait_phase - 0.5) * 2.0  # 0.0 to 1.0

        # Step parameters
        step_length = 30.0  # mm
        step_height = 20.0  # mm

        # Calculate foot positions
        for leg_id in swing_legs:
            # Swing phase: lift foot, move forward, lower
            base_pos = self.default_foot_positions[leg_id]

            # Forward movement based on velocity
            forward_offset = Vector(
                [
                    self.velocity.x * dt * 1000,  # Convert m/s to mm/s
                    self.velocity.y * dt * 1000,
                    0,
                ]
            )

            # Trajectory: lift up, move forward, lower down
            if swing_phase < 0.5:
                # Lifting phase
                z_offset = step_height * (swing_phase * 2.0)
                forward_progress = 0.0
            else:
                # Lowering phase
                z_offset = step_height * (2.0 - swing_phase * 2.0)
                forward_progress = (swing_phase - 0.5) * 2.0

            # Move foot forward during swing
            forward_move = Vector(
                [
                    forward_offset.x * forward_progress,
                    forward_offset.y * forward_progress,
                    -z_offset,
                ]
            )

            self.target_foot_positions[leg_id] = base_pos + forward_move

        for leg_id in support_legs:
            # Support phase: move backward to push body forward
            base_pos = self.default_foot_positions[leg_id]
            backward_progress = swing_phase  # Move back as swing leg moves forward

            backward_move = Vector(
                [
                    -self.velocity.x * dt * 1000 * backward_progress,
                    -self.velocity.y * dt * 1000 * backward_progress,
                    0,
                ]
            )

            self.target_foot_positions[leg_id] = base_pos + backward_move

    def _wave_gait(self, dt: float):
        """
        Wave gait: One leg lifts at a time, creating a wave motion.
        Most stable but slowest gait.
        """
        # Each leg gets 1/6 of the cycle
        leg_cycle_length = 1.0 / 6.0

        for i, leg_id in enumerate(LegID):
            leg_phase = (self.gait_phase - i * leg_cycle_length) % 1.0

            if leg_phase < 0.5:
                # Support phase
                base_pos = self.default_foot_positions[leg_id]
                backward_progress = leg_phase * 2.0
                backward_move = Vector(
                    [
                        -self.velocity.x * dt * 1000 * backward_progress,
                        -self.velocity.y * dt * 1000 * backward_progress,
                        0,
                    ]
                )
                self.target_foot_positions[leg_id] = base_pos + backward_move
            else:
                # Swing phase
                swing_phase = (leg_phase - 0.5) * 2.0
                base_pos = self.default_foot_positions[leg_id]

                step_height = 20.0
                if swing_phase < 0.5:
                    z_offset = step_height * (swing_phase * 2.0)
                    forward_progress = 0.0
                else:
                    z_offset = step_height * (2.0 - swing_phase * 2.0)
                    forward_progress = (swing_phase - 0.5) * 2.0

                forward_move = Vector(
                    [
                        self.velocity.x * dt * 1000 * forward_progress,
                        self.velocity.y * dt * 1000 * forward_progress,
                        -z_offset,
                    ]
                )

                self.target_foot_positions[leg_id] = base_pos + forward_move

    def _ripple_gait(self, dt: float):
        """
        Ripple gait: Two groups of three legs, but with offset timing.
        Faster than wave, more stable than tripod.
        """
        # Similar to tripod but with phase offset
        # This is a simplified version - full ripple is more complex
        self._tripod_gait(dt)  # For now, use tripod as placeholder

    def _stand_still(self):
        """Maintain default standing positions."""
        self.target_foot_positions = self.default_foot_positions.copy()
