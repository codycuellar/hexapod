"""
Hexapod Controller - Handles gait patterns and motion planning.

The controller's job is to:
1. Take high-level commands (velocity, rotation)
2. Generate gait patterns
3. Plan foot trajectories in body-relative coordinates
4. Coordinate leg movements over time
"""

from enum import Enum, auto

from hexapod.hexpod import Body, LegID
from hexapod.engine import Vec3d
from hexapod.interpolation import lerp_3d


class GaitState(Enum):
    STARTUP = auto()
    STANDING = auto()
    WALKING = auto()


class MotionPlanner:
    """
    Controller for hexapod locomotion.
    Handles gait patterns and converts velocity/rotation commands into foot positions.
    """

    max_velocity = 120  # mm/s

    def __init__(self, body: Body, leg_relative_stand_position: Vec3d):
        self.body = body

        self.velocity = Vec3d.zero()  # Body-relative velocity (x, y, z)
        self.rotation = Vec3d.zero()  # Body rotation rates (roll, pitch, yaw)

        self.gait_phase = 0.5  # 0.0 to 1.0, cycles through gait pattern

        # These will be initialized by initialize()
        self.leg_relative_stand_position = leg_relative_stand_position
        self.standing_foot_positions: dict[LegID, Vec3d] = {}
        self.target_foot_positions: dict[LegID, Vec3d] = {}

        self.state = GaitState.STARTUP
        self.transitioning = False
        self.time_in_transition = 0.0
        self.transition_target_time = 0.0
        self.transition_start_position: dict[LegID, Vec3d] = {}
        self.min_transition_velocity = {
            GaitState.STARTUP: 0,
            GaitState.STANDING: 80,
            GaitState.WALKING: self.max_velocity,
        }

    def initialize(self):
        """
        Initialize the controller by syncing with hardware and calculating positions.
        Call this after creating the controller to probe controls and set up initial state.
        """
        for id, leg in self.body.legs.items():
            stand_pos = leg.frame.local_to_frame(
                self.body.frame, self.leg_relative_stand_position
            )
            self.standing_foot_positions[id] = stand_pos
            # initial power-up position directly above standing position, slightly above ground
            safe_pos = Vec3d(stand_pos.x, stand_pos.y, 10)
            self.body.set_foot_position(id, safe_pos)

        self._start_transition(GaitState.STANDING)

    def set_velocity(self, velocity: Vec3d):
        """Set desired body-relative normalized velocity vector."""
        self.velocity = velocity

    def set_rotation(self, rotation: Vec3d):
        """Set desired body rotation rates (roll, pitch, yaw in deg/s)."""
        self.rotation = rotation

    def step(self, dt: float):
        if self.transitioning:
            self._do_transition(dt)
        # elif self.state == GaitState.WALKING:
        #     self._tripod_gait(dt)

    def _get_next_initial_pos(self, leg_id: LegID, state: GaitState):
        # if state == GaitState.STANDING:
        return self.standing_foot_positions[leg_id]
        # elif state == GaitState.WALKING:
        #     return self.

    def _start_transition(self, state: GaitState):
        self.state = state
        self.transitioning = True
        self.time_in_transition = 0.0

        max_distance = 0.0
        for id in self.body.legs.keys():
            start = self.body.get_foot_position(id)
            target = self._get_next_initial_pos(id, state)
            self.transition_start_position[id] = start
            self.target_foot_positions[id] = target
            max_distance = max(max_distance, start.distance_to(target))

        self.transition_target_time = max_distance / self.min_transition_velocity[state]

    def _do_transition(self, dt: float):
        self.time_in_transition += dt
        # print(dt)
        t = self.time_in_transition / self.transition_target_time
        if t <= 1.0:
            for id in self.body.leg_ids:
                start = self.transition_start_position[id]
                target = self.target_foot_positions[id]
                next_position = lerp_3d(start, target, t)
                self.body.set_foot_position(id, next_position)
        else:
            self._clear_transition()

    def _clear_transition(self):
        self.transitioning = False
        self.time_in_transition = 0.0
        self.transition_start_position = {}

    # def _tripod_gait(self, dt: float):
    #     """
    #     Tripod gait: 3 legs on ground, 3 legs in air.
    #     Legs move in two groups: (LF, RM, RR) and (RF, LM, LR)
    #     """
    #     # Determine which legs are in swing phase (lifting/moving)
    #     # Phase 0.0-0.5: Group 1 swings, Group 2 supports
    #     # Phase 0.5-1.0: Group 2 swings, Group 1 supports
    #     swing_group_1 = [LegID.LF, LegID.RM, LegID.RB]
    #     swing_group_2 = [LegID.RF, LegID.LM, LegID.LB]

    #     if self.gait_phase < 0.5:
    #         swing_legs = swing_group_1
    #         support_legs = swing_group_2
    #         swing_phase = self.gait_phase * 2.0  # 0.0 to 1.0
    #     else:
    #         swing_legs = swing_group_2
    #         support_legs = swing_group_1
    #         swing_phase = (self.gait_phase - 0.5) * 2.0  # 0.0 to 1.0

    #     # Step parameters
    #     step_length = 30.0  # mm
    #     step_height = 20.0  # mm

    #     # Calculate foot positions
    #     for leg_id in swing_legs:
    #         # Swing phase: lift foot, move forward, lower
    #         base_pos = self.default_foot_positions[leg_id]

    #         # Forward movement based on velocity
    #         forward_offset = Vec3d(
    #             self.velocity.x * dt * 1000,  # Convert m/s to mm/s
    #             self.velocity.y * dt * 1000,
    #             0,
    #         )

    #         # Trajectory: lift up, move forward, lower down
    #         if swing_phase < 0.5:
    #             # Lifting phase
    #             z_offset = step_height * (swing_phase * 2.0)
    #             forward_progress = 0.0
    #         else:
    #             # Lowering phase
    #             z_offset = step_height * (2.0 - swing_phase * 2.0)
    #             forward_progress = (swing_phase - 0.5) * 2.0

    #         # Move foot forward during swing
    #         forward_move = Vec3d(
    #             forward_offset.x * forward_progress,
    #             forward_offset.y * forward_progress,
    #             -z_offset,
    #         )

    #         self.target_foot_positions[leg_id] = base_pos + forward_move

    #     for leg_id in support_legs:
    #         # Support phase: move backward to push body forward
    #         base_pos = self.default_foot_positions[leg_id]
    #         backward_progress = swing_phase  # Move back as swing leg moves forward

    #         backward_move = Vec3d(
    #             -self.velocity.x * dt * 1000 * backward_progress,
    #             -self.velocity.y * dt * 1000 * backward_progress,
    #             0,
    #         )

    #         self.target_foot_positions[leg_id] = base_pos + backward_move
