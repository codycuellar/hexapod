"""
Hexapod Controller - Handles gait patterns and motion planning.

The controller's job is to:
1. Take high-level commands (velocity, rotation)
2. Generate gait patterns
3. Plan foot trajectories in body-relative coordinates
4. Coordinate leg movements over time
"""

import math

from enum import Enum, auto

from hexapod.hexpod import Body, LegID
from hexapod.engine import Vec3d, Vec2d, Frame, Rotation
from hexapod.interpolation import lerp_3d


class GaitState(Enum):
    STARTUP = auto()
    STANDING = auto()
    WALKING = auto()


class TripodGait:
    leg_relative_position = Vec3d(150, 0, -60)
    max_velocity = 120  # mm/s
    gait_radius = 25  # mm
    step_height = 25  # mm

    # max_rotation_angle = 20  # +/- degrees
    # max_rotation_speed = 45  # deg/s

    stride_control_start = Vec3d(z=leg_relative_position.z)
    swing_control_start = Vec3d(z=leg_relative_position.z + step_height)

    def __init__(self):
        self.stride_control = Frame(origin=self.stride_control_start)
        self.swing_control = Frame(origin=-self.stride_control_start)

        self.gait_vector = Vec2d()
        self.last_nonzero_vector = Vec2d()
        self.rotation_velocity = 0.0

        offset = Vec3d(220, 0, 0)
        rm_direction_frame = Frame(offset, parent=self.stride_control)
        lf_direction_frame = Frame(
            Rotation.degrees(z=120) @ offset, parent=self.stride_control
        )
        lb_direction_frame = Frame(
            Rotation.degrees(z=-120) @ offset, parent=self.stride_control
        )
        rf_direction_frame = Frame(
            Rotation.degrees(z=60) @ offset, parent=self.swing_control
        )
        lm_direction_frame = Frame(
            Rotation.degrees(z=180) @ offset, parent=self.swing_control
        )
        rb_direction_frame = Frame(
            Rotation.degrees(z=-60) @ offset, parent=self.swing_control
        )

        self.stride_group = {
            LegID.RM: Frame(parent=rm_direction_frame),
            LegID.LF: Frame(parent=lf_direction_frame),
            LegID.LB: Frame(parent=lb_direction_frame),
        }
        self.swing_group = {
            LegID.RF: Frame(parent=rf_direction_frame),
            LegID.LM: Frame(parent=lm_direction_frame),
            LegID.RB: Frame(parent=rb_direction_frame),
        }

    def get_foot_global_positions(self):
        legs = list(self.stride_group.items()) + list(self.swing_group.items())
        return {leg_id: frame.get_origin_in_world() for leg_id, frame in legs}

    def update(self, gait_vector: Vec2d, rotation_velocity: float):
        l = gait_vector.length()
        if l > 1.0:
            gait_vector /= l
        self.gait_vector = gait_vector

        if l > 0.0:
            self.last_nonzero_vector = gait_vector

        self.rotation_velocity = min(1.0, max(-1.0, rotation_velocity))

    def step(self, dt: float):
        velocity = self.gait_vector.length() * self.max_velocity
        distance_to_move = velocity * dt

        foot_pos: Vec3d
        foot_angle = self.last_nonzero_vector.degree_y()

        for frame in self.stride_group.values():
            if not frame.parent:
                raise ValueError("Incorrect gait setup, no parent for foot frame.")
            frame.parent.rotation = Rotation.degrees(z=foot_angle)
            frame.origin -= Vec3d(y=distance_to_move)
            foot_pos = frame.origin

        for frame in self.swing_group.values():
            if not frame.parent:
                raise ValueError("Incorrect gait setup, no parent for foot frame.")
            frame.parent.rotation = Rotation.degrees(z=foot_angle)
            frame.origin += Vec3d(y=distance_to_move)

        phase = abs(self.gait_radius - foot_pos.y) / (self.gait_radius * 2)
        if phase >= 1.0:
            self._flip_groups()

        # ease on start, top, and back down to bottom.
        z = self.step_height * 0.5 * (1 - math.cos(2 * math.pi * phase))
        self.swing_control.origin = Vec3d(z=self.leg_relative_position.z + z)

    def _flip_groups(self):
        swing_c = self.swing_control
        swing_g = self.swing_group
        self.swing_control = self.stride_control
        self.swing_group = self.stride_group
        self.stride_control = swing_c
        self.stride_group = swing_g

        self.stride_control.origin = self.stride_control_start
        self.swing_control.origin = -self.stride_control_start

        for frame in self.stride_group.values():
            frame.origin = Vec3d(y=self.gait_radius)

        for frame in self.swing_group.values():
            frame.origin = Vec3d(y=-self.gait_radius)


class MotionPlanner:
    """
    Controller for hexapod locomotion.
    Handles gait patterns and converts velocity/rotation commands into foot positions.
    """

    def __init__(self, body: Body, leg_relative_stand_position: Vec3d):
        self.body = body

        self.rotation = Vec3d()  # Body rotation rates (roll, pitch, yaw)

        self.gait = TripodGait()

        # These will be configured by initialize()
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
            GaitState.WALKING: 80,
        }

    def initialize(self):
        """
        Initialize the controller by syncing with hardware and calculating positions.
        Call this after creating the controller to probe controls and set up initial state.
        """
        for id, leg in self.body.legs.items():
            stand_pos = leg.frame.local_pos_to_frame(
                self.body.frame, self.leg_relative_stand_position
            )
            self.standing_foot_positions[id] = stand_pos
            # initial power-up position directly above standing position, slightly above ground
            safe_pos = Vec3d(stand_pos.x, stand_pos.y, 10)
            self.body.set_foot_position(id, safe_pos)

        self._start_transition(GaitState.STANDING)

    def update_gait(self, gait_vector: Vec2d, rotation_velocity):
        """Set desired body-relative normalized velocity vector."""
        self.gait.update(gait_vector, rotation_velocity)

    def update_body_position(self, rotation: Vec3d):
        """Set desired body rotation rates (roll, pitch, yaw in deg/s)."""
        self.rotation = rotation

    def step(self, dt: float):
        # TODO: Accumulate inactive motion time.

        # if self.transitioning:
        #     self._do_transition(dt)
        # else:
        # velocity = self.gait_vector.length()
        # if self.state != GaitState.WALKING and velocity > 0:
        #     print("going to walking")
        #     self._start_transition(GaitState.WALKING)
        # elif self.state != GaitState.STANDING and velocity == 0:
        #     self._start_transition(GaitState.STANDING)
        # else:
        self.gait.step(dt)
        positions = self.gait.get_foot_global_positions()
        for id, pos in positions.items():
            self.body.set_foot_position(id, self.body.frame.world_pos_to_local(pos))

    def _get_next_initial_pos(self, state: GaitState):
        if state == GaitState.STANDING:
            return self.standing_foot_positions
        else:
            positions = self.gait.get_foot_global_positions()
            return {
                id: self.body.frame.world_pos_to_local(val)
                for id, val in positions.items()
            }

    def _start_transition(self, state: GaitState):
        self.state = state
        self.transitioning = True
        self.time_in_transition = 0.0

        targets = self._get_next_initial_pos(state)
        max_distance = 0.0
        for id in self.body.legs.keys():
            start = self.body.get_foot_position(id)
            self.transition_start_position[id] = start
            self.target_foot_positions[id] = targets[id]
            max_distance = max(max_distance, start.distance_to(targets[id]))

        self.transition_target_time = max_distance / self.min_transition_velocity[state]

    def _do_transition(self, dt: float):
        self.time_in_transition += dt
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
