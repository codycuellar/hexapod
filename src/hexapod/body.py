from time import ticks_ms

from hexapod.utils import Coord, Transform3D, Vector
from hexapod.interpolation import lerp_3d, quad_bez_3d
from hexapod.leg import Leg


class Body:
    gaits = ["tripod"]

    def __init__(
        self,
        rf: Leg,
        rc: Leg,
        rb: Leg,
        lf: Leg,
        lc: Leg,
        lb: Leg,
        initial_position: Transform3D | None = None,
        initial_rotation: float = 0,
        update_frequency: float = 1 / 50,
    ):
        """legs are ordered in right front, clockwise around the body."""
        self.legs = [rf, rc, rb, lb, lc, lf]
        self.foot_frames = [Coord(0, 0, 0) for _ in range(len(self.legs))]

        self.update_frequency = update_frequency

        self.realtive_position = (
            Transform3D(Coord(0, 0, 0), 0, 0, 0)
            if initial_position is None
            else initial_position
        )
        self.relative_rotation = initial_rotation
        self.last_update = ticks_ms()

        self.current_gait = self.gaits[0]
        self.current_velocity = Vector(0, 0, 0)

    def go_to_home(self):
        """
        When powered on, leg positions cannot accurately be read if they
        have been physically moved, so this assumes the body is in contact with
        the ground, and we can home the legs to a known position off the ground.
        This will happen as fast as the servos can move, so it's the safest way to
        initialize known positions.
        """

    def update(self):
        if self.current_velocity.length() > 0:
            self._move(t)

    def update_velocity(self, velocity: Vector):
        self.current_velocity = velocity.normalize()

    def change_gait(self, gait=None):
        """
        Change the gait of the hexapod. If no gait is specified, it will cycle to the next one.
        param gait: The name of the gait to switch to from Body.gaits.
                    If None, cycles to the next gait.
        """
        if gait is None:
            next_gait = (self.gaits.index(self.current_gait) + 1) % len(self.gaits)
            self.current_gait = self.gaits[next_gait]

        if gait not in self.gaits:
            raise ValueError(f"Gait {gait} not found.")

        self.current_gait = gait

    def _move(self, t: float):
        if self.current_gait == "tripod":
            self._tripod_gait(t)
        else:
            raise ValueError(f"Gait {self.current_gait} not implemented.")

    def _tripod_gait(self, t: float):
        """
        Move the hexapod in a tripod gait pattern.
        """
        # Define the leg positions in the tripod gait
        legs = [
            self.rf_leg,
            self.rc_leg,
            self.rb_leg,
            self.lf_leg,
            self.lc_leg,
            self.lb_leg,
        ]

        # Move the legs in a tripod pattern
        for i in range(0, len(legs), 2):
            legs[i].set_position(direction)
