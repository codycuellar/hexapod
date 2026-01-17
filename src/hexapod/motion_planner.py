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

from hexapod.rigid_body import Body, LegID
from hexapod.engine import Vec3d, Vec2d, Frame, Rotation, Transform
import hexapod.interpolation as lerp


class GaitState(Enum):
    STARTUP = auto()
    STANDING = auto()
    WALKING = auto()


@dataclass
class GaitGeometry:
    step_height = 45.0  # mm
    safe_radius = 35.0  # mm
    max_radius = 65.0  # mm
    safe_angle = 10  # degrees
    max_angle = 18  # degrees


@dataclass
class GaitMotion:
    max_velocity = 225.0  # mm/s
    max_rot_velocity = 35  # degrees / second
    swing_velocity_scale = 1.65  # factor of max velocity
    min_swing_velocity_factor = 0.4  # factor of max velocity

    max_swing_velocity = max_velocity * swing_velocity_scale
    min_swing_velocity = max_swing_velocity * min_swing_velocity_factor
    max_swing_rot_velocity = max_rot_velocity * swing_velocity_scale
    min_swing_rot_velocity = max_swing_rot_velocity * min_swing_velocity_factor


@dataclass
class GaitTiming:
    swing_duration_resting = 0.35  # seconds
    max_swing_duration = 1.0  # seconds
    rest_trigger_time = 0.75  # seconds


class TripodGait:
    def __init__(
        self,
        reference_frame: Frame,
        leg_offset: Vec3d,
        geometry: GaitGeometry,
        motion: GaitMotion,
        timing: GaitTiming,
    ):
        self.reference_frame = reference_frame
        self.leg_offset = leg_offset
        self.geometry = geometry
        self.motion = motion
        self.timing = timing

        self.state = GaitState.STANDING

        self.time_resting = 0.0

        self.input_vector = Vec3d()  # normalized to unit-range
        self.rotation_input_velocity = 0.0  # unit-range factor

        self.swing_phase = 0.0
        self.swing_path = tuple([Vec3d()] * 8)
        self.swing_v_distance = 0.0
        self.swing_rotation_path = (0.0, 0.0)
        self.swing_rotation_offset = Vec3d()

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

        # this is just a simulation helper to provide the ground transformation
        # synced with the stride movment.
        self.ground_transform = Transform.identity()

    @property
    def leg_swinging(self):
        return self.swing_group is not None

    def get_foot_global_positions(self):
        return {
            leg_id: frame.get_origin_in_world()
            for leg_id, frame in self.foot_frames.items()
        }

    def update(self, gait_vector: Vec3d, rotation_velocity: float):
        self.input_vector = gait_vector
        self.rotation_input_velocity = rotation_velocity

        if gait_vector.length() > 0.0 or rotation_velocity != 0.0:
            self.time_resting = 0.0

    def step(self, dt: float):
        mag = self.input_vector.length()

        if self.state == GaitState.STANDING:
            if mag > 0.0 or self.rotation_input_velocity != 0.0:
                self.state = GaitState.WALKING
                self._queue_swing(self.stride_groups[0])

        elif self.state == GaitState.WALKING:
            # we're not receiving inputs
            if self.leg_swinging:
                self._perform_swing(dt)

            if mag > 0.0:
                self._perform_stride(dt)
            else:
                self.ground_transform.translation = Vec3d()

            if self.rotation_input_velocity != 0.0:
                self._perform_rotation(dt)
            else:
                self.ground_transform.rotation = Rotation.identity()

            if (
                not self.leg_swinging
                and mag == 0.0
                and self.rotation_input_velocity == 0.0
            ):
                self._queue_rest_position(dt)

    def _perform_stride(self, dt: float):
        gait_dir = self.input_vector.normalize()
        gait_mag = self.input_vector.length()

        # the maximum distance the foot can travel in world space this timestep
        max_step_distance = self.motion.max_velocity * gait_mag * dt

        transform_applied = False
        for group in self.stride_groups:
            current_position = group.origin
            step_vector = -gait_dir * max_step_distance
            next_position = current_position + step_vector
            step_radius = next_position.length()

            # if we've left the safe zone, attempt to trigger a swing, and start
            # an eased ramp into the outer clipping zone.
            if step_radius > self.geometry.safe_radius:
                self._queue_swing(group)
                next_position = current_position + self._smooth_clamp_stride_limit(
                    step_radius, step_vector
                )

            group.origin = next_position

            # Apply the translation. Only do it once since it's the same for all
            # feet currently on the ground.
            if not transform_applied:
                self.ground_transform.translation = next_position - current_position
                transform_applied = True

    def _perform_rotation(self, dt: float):
        for stride_group in self.stride_groups:
            rot_group = stride_group.parent
            if not rot_group:
                return

            current_angle = self._get_rotation_group_angle(rot_group)
            delta = -self.rotation_input_velocity * self.motion.max_rot_velocity * dt
            final_angle = current_angle + delta

            # clamp to outer limit
            if abs(final_angle) > self.geometry.max_angle:
                final_angle = math.copysign(self.geometry.max_angle, final_angle)
                delta = final_angle - current_angle

            delta_rotation = Rotation.degrees(z=delta)
            rot_group.rotate(delta_rotation)

            if abs(final_angle) > self.geometry.safe_angle:
                self._queue_swing(stride_group)

            self.ground_transform.rotation = delta_rotation

    def _perform_swing(self, dt: float):
        if not self.swing_group or not self.swing_group.parent:
            return

        trans_phase_inc = (self._current_swing_speed() * dt) / self.swing_v_distance
        rotation_angle = abs(self.swing_rotation_path[1] - self.swing_rotation_path[0])

        if rotation_angle > 0.01:
            arc_len = math.radians(rotation_angle) * self.leg_offset.length()
            rot_speed_rad = math.radians(self._current_swing_rot_speed())
            rot_phase_inc = (rot_speed_rad * self.leg_offset.length() * dt) / arc_len
        else:
            rot_phase_inc = trans_phase_inc

        phase_inc = min(trans_phase_inc, rot_phase_inc)
        self.swing_phase = min(self.swing_phase + phase_inc, 1.0)

        if self.swing_phase < 0.5:
            local_t = self.swing_phase / 0.5
            segment = self.swing_path[0:4]  # (start, p1, p2, midpoint)
        else:
            local_t = (self.swing_phase - 0.5) / 0.5
            segment = self.swing_path[4:8]  # (midpoint, p5, p6, end)

        self.swing_group.origin = lerp.cubic_bez_3d(local_t, *segment)
        angle = lerp.lerp(self.swing_phase, *self.swing_rotation_path)
        self.swing_group.parent.rotation = Rotation.degrees(z=angle)

        if self.swing_phase >= 1.0:
            self._end_swing()

    def _current_ground_velocity(self):
        return -self.input_vector * self.motion.max_rot_velocity

    def _current_swing_speed(self):
        return max(
            self.motion.min_swing_velocity,
            self.input_vector.length() * self.motion.max_swing_velocity,
        )

    def _current_swing_rot_speed(self):
        return max(
            self.motion.min_swing_rot_velocity,
            abs(self.rotation_input_velocity) * self.motion.max_swing_rot_velocity,
        )

    def _make_swing_path(self, start: Vec3d, end: Vec3d | None = None):
        if not end:
            end = (
                self.input_vector.normalize()
                * self.geometry.safe_radius
                * 0.95  # small shirinkage so we don't plant ON the safe zone edge
            )

        midpoint = ((end + start) / 2).replace(z=self.geometry.step_height)

        self.swing_v_distance = start.distance_to(midpoint) + midpoint.distance_to(end)

        swing_speed = self._current_swing_speed()
        segment_t = (self.swing_v_distance / swing_speed) / 2

        transition_point = (self._current_ground_velocity() * segment_t) / 3
        p1 = start - transition_point

        midpoint_velocity = (end - start).normalize() * swing_speed
        midpoint_offset = (midpoint_velocity * segment_t) / 3

        p2 = midpoint - midpoint_offset
        p5 = midpoint + midpoint_offset

        p6 = end - transition_point
        return (start, p1, p2, midpoint, midpoint, p5, p6, end)

    def _queue_swing(
        self, group: Frame, end: Vec3d | None = None, is_rest: bool = False
    ):
        if self.swing_group:
            # if we already have a swing group, we can't queue up yet, try again next timestep
            return
        elif not group.parent:
            raise ValueError("TripodGait incorrectly setup without a rotation parent.")

        self.swing_group = group
        self.stride_groups.remove(group)
        self.swing_phase = 0.0

        self.swing_path = self._make_swing_path(group.origin, end)

        swing_rotation_end = 0.0
        if not is_rest and self.rotation_input_velocity != 0.0:
            swing_rotation_end = (
                self.rotation_input_velocity * self.geometry.safe_angle * 0.95
            )

        self.swing_rotation_path = (
            self._get_rotation_group_angle(group.parent),
            swing_rotation_end,
        )

    def _end_swing(self):
        if self.swing_group:
            self.stride_groups.append(self.swing_group)
        self.swing_group = None

    def _queue_rest_position(self, dt: float):
        self.time_resting += dt
        if self.time_resting <= self.timing.rest_trigger_time:
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

    def _smooth_clamp_stride_limit(self, step_radius: float, delta_vector: Vec3d):
        # TODO: Maybe check where the actual stride limit is in the direction
        # of the current gait for more possible range.
        # clamp to outer working area if we're still waiting for the other
        # group to complete its swing.
        t = (step_radius - self.geometry.safe_radius) / (
            self.geometry.max_radius - self.geometry.safe_radius
        )
        t = min(max(t, 0.0), 1.0)
        ease = 1.0 - (t * t * (3 - 2 * t))  # smoothstep inverted
        return delta_vector * ease

    def _get_rotation_group_angle(self, group: Frame):
        return (
            group.local_pos_to_frame(self.reference_frame, Vec3d(1, 0, 0))
            .to_2d()
            .degree_x()
        )


class MotionPlanner:
    """
    Controller for hexapod locomotion.
    Handles gait patterns and converts velocity/rotation commands into foot positions.
    """

    max_pos_offset = Vec3d(40, 40, 50)  # mm
    max_rot_offset = Vec3d(20, 20, 40)  # degrees
    pos_offset_roc = 18  # mm/s
    rot_offset_roc = 18  # deg/s

    reference_frame_pos = Vec3d(0, 0, 0)
    foot_offset = Vec3d(210, 0, 0)

    input_filter_rate = 4.5  # change/seconds

    def __init__(self, body: Body, leg_relative_stand_position: Vec3d):
        self.body = body

        self.gait = TripodGait(
            Frame(origin=self.reference_frame_pos),
            self.foot_offset,
            GaitGeometry(),
            GaitMotion(),
            GaitTiming(),
        )

        self.body_pos_offset_fixed = Vec3d(0, 0, 65)  # fixed body position offset in mm
        self.body_rot_offset_fixed = Vec3d()  # fixed body rotation offset in degrees
        self.body_pos_input_offset = (
            Vec3d()
        )  # current step body position offset as unit vector
        self.body_rot_input_offset = (
            Vec3d()
        )  # current step body rotation offset as unit vector

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
        self.ground_transform = Transform.identity()

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

    def update_gait(self, dt: float, gait_vector: Vec2d, rotation_velocity: float):
        """Set desired body-relative normalized velocity vector."""
        # clamp to unit length if it has exceeded for some reason.
        l = gait_vector.length()
        if l > 1.0:
            gait_vector = gait_vector.normalize()

        # clamp the rotationaly velocity if needed
        if rotation_velocity < -1.0:
            rotation_velocity = max(-1.0, rotation_velocity)
        if rotation_velocity > 1.0:
            rotation_velocity = min(1.0, rotation_velocity)

        # filter the input rate of change
        vector = lerp.rate_limit_2d(
            dt, self.gait.input_vector.to_2d(), gait_vector, self.input_filter_rate
        )
        rotation_velocity = lerp.rate_limit(
            dt,
            self.gait.rotation_input_velocity,
            rotation_velocity,
            self.input_filter_rate,
        )

        self.gait.update(vector.to_3d(), rotation_velocity)

    def offset_body(self, dt: float, offset: Vec3d, rotation: Vec3d):
        """Set desired body rotation rates (roll, pitch, yaw in deg/s)."""
        self.body_pos_input_offset = lerp.rate_limit_3d(
            dt, self.body_pos_input_offset, offset, self.input_filter_rate
        )
        self.body_rot_input_offset = lerp.rate_limit_3d(
            dt, self.body_rot_input_offset, rotation, self.input_filter_rate
        )

    def trim_body(self, offset: Vec3d, rotation: Vec3d):
        self.body_pos_offset_fixed += offset * self.pos_offset_roc
        self.body_rot_offset_fixed += rotation * self.rot_offset_roc

    def step(self, dt: float):
        self.gait.step(dt)
        positions = self.gait.get_foot_global_positions()

        self.body.frame.origin = self.body_pos_offset_fixed + (
            self.body_pos_input_offset.elementwise("mul", self.max_pos_offset)
        )
        self.body.frame.rotation = Rotation.degrees_vec(
            self.body_rot_offset_fixed
            + self.body_rot_input_offset.elementwise("mul", self.max_rot_offset)
        )

        for id, pos in positions.items():
            self.body.set_foot_position(id, self.body.frame.world_pos_to_local(pos))

        self.ground_transform = self.gait.ground_transform

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
