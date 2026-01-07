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
from hexapod.engine import Vec3d, Vec2d, Frame, Rotation
from hexapod.interpolation import lerp_3d, cubic_bez_3d


class GaitState(Enum):
    STARTUP = auto()
    STANDING = auto()
    WALKING = auto()


class TripodGait:
    leg_relative_position = Vec3d(150, 0, -60)
    max_swing_velocity = 250.0  # mm/s
    max_velocity = 150.0  # mm/s
    gait_radius = 35.0  # mm
    gait_radius_max = 55.0
    step_height = 25.0  # mm
    rest_time = 1.0  # seconds

    max_gait_accel = 4.0  # change/seconds

    # max_rotation_angle = 20  # +/- degrees
    # max_rotation_speed = 45  # deg/s

    def __init__(self, reference_frame: Frame, leg_offset: Vec3d):
        self.state = GaitState.STANDING
        self.time_resting = 0.0

        self.gait_vector = Vec3d()  # normalized to unit-range
        self.previous_gait_vector = Vec3d()

        self.rotation_velocity = 0.0  # unit-range factor

        self.leg_swinging = False
        self.swing_phase = 0.0
        self.swing_path = (Vec3d(), Vec3d(), Vec3d(), Vec3d())
        self.swing_distance = 0.0

        self.control_a = Frame(parent=reference_frame)
        self.control_b = Frame(parent=reference_frame)

        self.swing_group: Frame | None = None
        self.stride_groups: list[Frame] = [self.control_a, self.control_b]

        rd = Rotation.degrees
        self.leg_frames = {
            LegID.RM: Frame(leg_offset, parent=self.control_a),
            LegID.LF: Frame(rd(z=120) @ leg_offset, parent=self.control_a),
            LegID.LB: Frame(rd(z=-120) @ leg_offset, parent=self.control_a),
            LegID.RF: Frame(rd(z=60) @ leg_offset, parent=self.control_b),
            LegID.LM: Frame(rd(z=180) @ leg_offset, parent=self.control_b),
            LegID.RB: Frame(rd(z=-60) @ leg_offset, parent=self.control_b),
        }

    def get_foot_global_positions(self):
        return {
            leg_id: frame.get_origin_in_world()
            for leg_id, frame in self.leg_frames.items()
        }

    def update(self, gait_vector: Vec3d, rotation_velocity: float):
        l = gait_vector.length()
        if l > 1.0:
            # clamp to unit range just in case
            gait_vector /= l

        self.previous_gait_vector = self.gait_vector
        self.gait_vector = gait_vector

        self.rotation_velocity = min(1.0, max(-1.0, rotation_velocity))

    def step(self, dt: float):
        self._filter_gait_vector(dt)

        gait_magnitude = self.gait_vector.length()

        if self.state == GaitState.STANDING:
            if gait_magnitude > 0.0:
                self.state = GaitState.WALKING
                print("initial swing")
                self._queue_swing(self.stride_groups[0])

        elif self.state == GaitState.WALKING:
            # we're not receiving inputs
            if self.leg_swinging:
                self._perform_swing(dt)

            elif gait_magnitude == 0.0:
                print("at rest")
                self._queue_rest_position(dt)
                return

            self._perform_stride(dt, gait_magnitude)

    def _perform_swing(self, dt: float):
        if not self.swing_group:
            raise ValueError("Leg swing performed, but no swing group assigned.")

        print(f"swing rate {self.gait_vector.length()}")
        scaled_velocity = self.max_swing_velocity * max(0.4, self.gait_vector.length())
        self.swing_phase += (scaled_velocity * dt) / self.swing_distance

        pos = cubic_bez_3d(min(1.0, self.swing_phase), *self.swing_path)
        self.swing_group.origin = pos
        if self.swing_phase >= 1.0:
            print(f"ending swing - {pos}")
            self._end_swing()

    def _perform_stride(self, dt: float, gait_magnitude: float):
        for group in self.stride_groups:
            # get the current stride position
            current_position = group.origin

            gait_dir = self.gait_vector.normalize()
            max_step_distance = self.max_velocity * gait_magnitude * dt

            # Clamp the motion vector to the max step distance if needed.
            step_vector = -gait_dir * max_step_distance

            # don't allow the step to travel past step_radius_max
            next_position = current_position + step_vector

            # if we've exited the stride radius, flip the groups and use the distance
            # we are outside the radius as the starting distance from the radius edge.
            step_radius = next_position.length()
            if step_radius > self.gait_radius:
                print(f"leg outside of radius {step_radius} - {next_position}")
                self._queue_swing(group)

            if step_radius >= self.gait_radius_max:
                next_position = current_position
                print(f"clipping motion to {next_position}")

            group.origin = next_position

    def _queue_swing(self, group: Frame):
        if self.swing_group:
            return

        gait_len = max(0.5, self.gait_vector.length())
        start = group.origin
        end = self.gait_vector.normalize() * gait_len * self.gait_radius

        self.leg_swinging = True
        self.swing_group = group
        self.stride_groups.remove(group)
        self.swing_distance = start.distance_to(end)

        height = (start.distance_to(end) / (self.gait_radius * 2)) * self.step_height
        height = max(height, self.step_height / 2)

        self.swing_path = (
            start,
            Vec3d(start.x, start.y, start.z + height),
            Vec3d(end.x, end.y, end.z + height),
            end,
        )
        print(f"swing start {start}, end {end}, height: {height}")

    def _end_swing(self):
        self.leg_swinging = False
        self.swing_phase = 0.0
        if self.swing_group:
            self.stride_groups.append(self.swing_group)
        self.swing_group = None
        self.swing_path = (Vec3d(), Vec3d(), Vec3d(), Vec3d())
        self.swing_distance = 0.0

    def _queue_rest_position(self, dt: float):
        self.time_resting += dt
        if self.time_resting <= self.rest_time:
            return

        # pick the stride group with the largest displacement from origin
        group = max(
            self.stride_groups,
            key=lambda g: g.origin.length(),
            default=None,
        )

        if group and group.origin.length() > 0.0:
            print(f"queueing rest {group}")
            self._queue_swing(group)
        else:
            print(f"ending rest")
            self.state = GaitState.STANDING
            self.time_resting = 0.0

    def _filter_gait_vector(self, dt: float):
        next = self.gait_vector
        current = self.previous_gait_vector

        max_change = self.max_gait_accel * dt

        delta = next - current
        delta_len = delta.length()

        if delta_len == 0.0 or delta_len < max_change:
            return
        else:
            delta = delta * (max_change / delta_len)

        self.gait_vector = self.previous_gait_vector + delta

        print(f"filtering gait vector:{next} to {self.gait_vector}")


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
                next_position = lerp_3d(t, start, target)
                self.body.set_foot_position(id, next_position)
        else:
            self._clear_transition()

    def _clear_transition(self):
        self.transitioning = False
        self.time_in_transition = 0.0
        self.transition_start_position = {}
