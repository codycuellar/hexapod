from hexapod.geometry_3d import Point, Transform, Vector, Rotation
from hexapod.interpolation import lerp_3d, quad_bez_3d
from hexapod.leg import Leg


class Body:
    gaits = ["tripod"]

    def __init__(
        self,
        legs: dict[str, Leg],
        initial_position: Transform | None = None,
        initial_rotation: float = 0,
        update_frequency: float = 1 / 50,
        max_velocity: float = 20,
    ):
        """legs are ordered in right front, clockwise around the body."""
        self.legs = legs
        self.foot_frames = self._set_foot_frames(legs)

        self.update_frequency = update_frequency
        self.max_velocity = max_velocity

        self.relative_position = (
            Transform(Vector(0, 0, 0), Rotation(0, 0, 0))
            if initial_position is None
            else initial_position
        )
        self.relative_rotation = initial_rotation

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
        pass

    def update(self):
        pass

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

    def _set_foot_frames(self, legs: dict[str, Leg]):
        foot_frames = {}
        for name, leg in legs.items():
            leg.change_global_position(self.relative_position)
            base = leg.pos_from_global
            offset = leg.coxa_len + (leg.femur_len + leg.tib_len) / 2
            foot_frames[name] = (
                base.position() + base.translation().normalize() * offset
            )
        return foot_frames

    def _move(self, t: float):
        if self.current_gait == "tripod":
            self._tripod_gait(t)
        else:
            raise ValueError(f"Gait {self.current_gait} not implemented.")

    def _tripod_gait(self, t: float):
        """
        Move the hexapod in a tripod gait pattern.
        """
        pass
