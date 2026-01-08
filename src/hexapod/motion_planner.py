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
from dataclasses import dataclass

from hexapod.hexpod import Body, LegID
from hexapod.engine import Vec3d, Vec2d, Frame, Rotation
from hexapod.interpolation import lerp_3d, cubic_bez_3d, lerp


class GaitState(Enum):
    STARTUP = auto()
    STANDING = auto()
    WALKING = auto()


@dataclass
class RotationParameters:
    velocity = 0.0  # unit scale
    safe_angle = 10  # degrees
    max_angle = 15  # degrees
    speed = 20  # degrees/second


@dataclass
class StrideParameters:
    vector = Vec3d()


class TripodGait:
    gait_working_radius = 35.0  # mm
    gait_radius_max = 60.0
    gait_velocity_max = 175.0  # mm/s
    gait_speed_scale_min = 0.2  # seconds
    gait_swing_velocity_max = 250.0  # mm/s

    swing_duration_resting = 0.35  # seconds
    swing_duration_min = 0.35  # seconds
    swing_radius_scale_max = 0.9  # unit vector scale

    rotation_working_angle = 10  # +/- degrees
    rotation_angle_max = 15  # +/- degrees
    rotation_velocity_max = 20  # deg/second
    rotation_angle_scale_min = 0.5

    step_height = 25.0  # mm
    input_filter_rate = 2.0  # change/seconds
    rest_trigger_time = 0.75  # seconds

    def __init__(self, reference_frame: Frame, leg_offset: Vec3d):
        self.state = GaitState.STANDING
        self.time_resting = 0.0

        self.gait_input_vector = Vec3d()  # normalized to unit-range
        self.gait_direction = Vec3d()
        self.gait_magnitude = 0.0
        self.previous_gait_vector = Vec3d()

        self.rotation_input_velocity = 0.0  # unit-range factor

        self.swing_phase = 0.0
        self.swing_elapsed = 0.0
        self.swing_duration = 0.0
        self.swing_path = (Vec3d(), Vec3d(), Vec3d(), Vec3d())
        self.swing_rotation_path = (0.0, 0.0)

        self.reference_frame = reference_frame
        self.rotate_group_a = Frame(parent=reference_frame)
        self.stride_group_a = Frame(parent=self.rotate_group_a)
        self.rotate_group_b = Frame(parent=reference_frame)
        self.stride_group_b = Frame(parent=self.rotate_group_b)

        self.swing_group: Frame | None = None
        self.stride_groups: list[Frame] = [self.stride_group_a, self.stride_group_b]

        rd = Rotation.degrees
        self.foot_frames = {
            LegID.RM: Frame(leg_offset, parent=self.stride_group_a),
            LegID.LF: Frame(rd(z=120) @ leg_offset, parent=self.stride_group_a),
            LegID.LB: Frame(rd(z=-120) @ leg_offset, parent=self.stride_group_a),
            LegID.RF: Frame(rd(z=60) @ leg_offset, parent=self.stride_group_b),
            LegID.LM: Frame(rd(z=180) @ leg_offset, parent=self.stride_group_b),
            LegID.RB: Frame(rd(z=-60) @ leg_offset, parent=self.stride_group_b),
        }

        self.print_group = self.stride_group_a

    @property
    def leg_swinging(self):
        return self.swing_group is not None

    def print(self, group: Frame, msg: str):
        if group is self.print_group:
            print(msg)

    def get_foot_global_positions(self):
        return {
            leg_id: frame.get_origin_in_world()
            for leg_id, frame in self.foot_frames.items()
        }

    def update(self, gait_vector: Vec3d, rotation_velocity: float):
        l = gait_vector.length()
        if l > 1.0:
            # clamp to unit range just in case
            gait_vector /= l

        self.previous_gait_vector = self.gait_input_vector
        self.gait_input_vector = gait_vector

        self.rotation_input_velocity = min(1.0, max(-1.0, rotation_velocity))

        if l > 0.0 or rotation_velocity != 0.0:
            self.time_resting = 0.0

    def step(self, dt: float):
        self._filter_gait_vector(dt)

        if self.state == GaitState.STANDING:
            if self.gait_magnitude > 0.0 or self.rotation_input_velocity != 0.0:
                self.state = GaitState.WALKING
                self._queue_swing(self.stride_groups[0])

        elif self.state == GaitState.WALKING:
            # we're not receiving inputs
            if self.leg_swinging:
                self._perform_swing(dt)

            if self.gait_magnitude > 0.0:
                self._perform_stride(dt, self.gait_magnitude)
            if self.rotation_input_velocity != 0.0:
                self._perform_rotation(dt)

            if (
                not self.leg_swinging
                and self.gait_magnitude == 0.0
                and self.rotation_input_velocity == 0.0
            ):
                self._queue_rest_position(dt)
                return

    def _perform_swing(self, dt: float):
        if not self.swing_group:
            raise ValueError("Leg swing performed, but no swing group assigned.")
        if not self.swing_group.parent:
            return

        self.swing_elapsed += dt
        self.swing_phase = min(1.0, self.swing_elapsed / self.swing_duration)
        pos = cubic_bez_3d(self.swing_phase, *self.swing_path)

        self.swing_group.origin = pos

        angle = lerp(self.swing_phase, *self.swing_rotation_path)
        self.swing_group.parent.rotation = Rotation.degrees(z=angle)
        self.print(
            self.swing_group,
            f"swinging angle current {angle:0.2f} - heights: {self.swing_group.origin.z:.3f}, {self.swing_group.parent.origin.z:.3f}",
        )

        if self.swing_phase >= 1.0:
            self._end_swing()

    def _queue_swing(
        self, group: Frame, end: Vec3d | None = None, is_rest: bool = False
    ):
        if self.swing_group:
            return
        elif not group.parent:
            raise ValueError("TripodGait incorrectly setup without a rotation parent.")

        self.swing_group = group
        self.stride_groups.remove(group)
        self.swing_phase = 0.0
        self.swing_elapsed = 0.0

        start = group.origin
        if not end:
            end = (
                self.gait_direction
                * self.gait_working_radius
                * self.swing_radius_scale_max
            )

        # TODO: sample all feet in this group with rotation applied to get the max step
        # distance of any foot for more accurate timing estimation
        distance = start.distance_to(end)

        self.swing_path = (
            start,
            Vec3d(start.x, start.y, start.z + self.step_height),
            Vec3d(end.x, end.y, end.z + self.step_height),
            end,
        )

        # always look to recenter first
        current_rotation_angle = self._get_group_angle(group.parent)
        self.swing_rotation_path = (current_rotation_angle, 0.0)

        if is_rest:
            self.swing_duration = self.swing_duration_resting
        else:
            speed_scale = max(self.gait_magnitude, self.gait_speed_scale_min)
            effective_swing_velocity = self.gait_swing_velocity_max * speed_scale
            stride_duration = max(
                distance / effective_swing_velocity, self.swing_duration_min
            )

            rotation_duration = 0.0
            # configure swing rotation
            if self.rotation_input_velocity != 0.0:
                velocity_clamped = math.copysign(
                    max(
                        self.rotation_angle_scale_min, abs(self.rotation_input_velocity)
                    ),
                    self.rotation_input_velocity,
                )
                swing_rotation_end = velocity_clamped * self.rotation_working_angle
                self.swing_rotation_path = (current_rotation_angle, swing_rotation_end)
                speed_scale = max(0.2, abs(self.rotation_input_velocity))
                effective_rotation_velocity = self.rotation_velocity_max * speed_scale
                rotation_duration = (
                    abs(swing_rotation_end) / effective_rotation_velocity
                )

            self.swing_duration = max(stride_duration, rotation_duration)

    def _perform_stride(self, dt: float, gait_magnitude: float):
        for group in self.stride_groups:
            # get the current stride position
            current_position = group.origin

            gait_dir = self.gait_direction
            max_step_distance = self.gait_velocity_max * gait_magnitude * dt
            step_vector = -gait_dir * max_step_distance
            next_position = current_position + step_vector

            # if we've exited the inner stride radius, trigger a swing if possible
            step_radius = next_position.length()

            if step_radius > self.gait_working_radius:
                self._queue_swing(group)

                # clamp to outer working area if we're still waiting for the other
                # group to complete its swing.
                t = (step_radius - self.gait_working_radius) / (
                    self.gait_radius_max - self.gait_working_radius
                )
                t = min(max(t, 0.0), 1.0)

                # smoothstep easing
                ease = 1.0 - (3 * t * t - 2 * t * t * t)
                next_position = current_position + step_vector * ease

            group.origin = next_position

    def _perform_rotation(self, dt: float):
        for stride_group in self.stride_groups:
            group = stride_group.parent
            if not group:
                return

            current_angle = self._get_group_angle(group)
            delta = -self.rotation_input_velocity * self.rotation_velocity_max * dt
            final_angle = current_angle + delta

            # clamp to outer limit
            if abs(final_angle) > self.rotation_angle_max:
                final_angle = math.copysign(self.rotation_angle_max, final_angle)
                delta = final_angle - current_angle

            group.rotate(Rotation.degrees(z=delta))

            self.print(stride_group, f"striding angle {final_angle:0.4f}")

            if abs(final_angle) > self.rotation_working_angle:
                self._queue_swing(stride_group)

    def _end_swing(self):
        if self.swing_group:
            self.stride_groups.append(self.swing_group)
        self.swing_group = None

    def _queue_rest_position(self, dt: float):
        self.time_resting += dt
        if self.time_resting <= self.rest_trigger_time:
            return

        furthest_distance = 2.0  # start with small epsilon
        furthest_foot: Frame | None = None

        for foot in self.foot_frames.values():
            rest = self.reference_frame.local_pos_to_world(foot.origin)
            current = foot.get_origin_in_world()
            delta = rest.distance_to(current)

            if delta > furthest_distance:
                furthest_distance = delta
                furthest_foot = foot

        if not furthest_foot:
            self.state = GaitState.STANDING
            self.time_resting = 0.0
            return

        for group in [self.stride_group_a, self.stride_group_b]:
            if group.has_child(furthest_foot):
                self._queue_swing(group, end=Vec3d(), is_rest=True)
                return

    def _get_group_angle(self, group: Frame):
        return (
            group.local_pos_to_frame(self.reference_frame, Vec3d(1, 0, 0))
            .to_2d()
            .degree_x()
        )

    def _filter_gait_vector(self, dt: float):
        """
        Filters the raw input vector so it cannot surpass a specified max
        rate of change.
        """
        next = self.gait_input_vector
        current = self.previous_gait_vector

        max_change = self.input_filter_rate * dt

        delta = next - current
        delta_len = delta.length()

        if delta_len > 0.0 and delta_len > max_change:
            delta = delta * (max_change / delta_len)
            self.gait_input_vector = self.previous_gait_vector + delta

        self.gait_direction = self.gait_input_vector.normalize()
        self.gait_magnitude = self.gait_input_vector.length()


class MotionPlanner:
    """
    Controller for hexapod locomotion.
    Handles gait patterns and converts velocity/rotation commands into foot positions.
    """

    def __init__(self, body: Body, leg_relative_stand_position: Vec3d):
        self.body = body

        self.gait = TripodGait(Frame(origin=Vec3d(0, 0, -60)), Vec3d(220, 0, 0))

        self.body_roll_offset = Vec3d()  # Body rotation rates (roll, pitch, yaw)

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

    def update_gait(self, gait_vector: Vec2d, rotation_velocity: float):
        """Set desired body-relative normalized velocity vector."""
        self.gait.update(Vec3d(gait_vector.x, gait_vector.y), rotation_velocity)

    def update_body_position(self, rotation: Vec3d):
        """Set desired body rotation rates (roll, pitch, yaw in deg/s)."""
        self.body_roll_offset = rotation

    def step(self, dt: float):
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
                next_position = lerp_3d(t, start, target)
                self.body.set_foot_position(id, next_position)
        else:
            self._clear_transition()

    def _clear_transition(self):
        self.transitioning = False
        self.time_in_transition = 0.0
        self.transition_start_position = {}
