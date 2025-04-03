from hexapod.utils import Coord3D
from hexapod.interpolation import lerp_3d, quad_bez_3d
from hexapod.leg import Leg

from servo import ServoCluster

class Body:
    _walk_cycle_handles = [
        Coord3D(105, 30, -75),
        Coord3D(105, -30, -75),
        Coord3D(105, 0, -75)
    ]
    _shimmy_cycle_handles = [
        Coord3D(105, 30, -75),
        Coord3D(105, -30, -75),
    ]

    hip_rotation = 60

    def __init__(self, rf:Leg, rc:Leg, rb:Leg, lf:Leg, lc:Leg, lb:Leg):
        self.rf_leg = rf
        self.rc_leg = rc
        self.rb_leg = rb
        self.lf_leg = lf
        self.lc_leg = lc
        self.lb_leg = lb
        self._build_standard_walk_cycle()
        self._build_shimmy_cycle()
    
    def set_legs_active(self, legs):
        self.rf_leg.enabled = legs[0]
        self.rc_leg.enabled = legs[1]
        self.rb_leg.enabled = legs[2]
        self.lf_leg.enabled = legs[3]
        self.lc_leg.enabled = legs[4]
        self.lb_leg.enabled = legs[5]

    def walk(self, t):
        opposite_t = (t + .5) % 1
        # opposite_t = t
        self.rf_leg.set_position(self._standard_walk_cycle(t, *self._leg_walk_handles['rf']))
        self.rc_leg.set_position(self._standard_walk_cycle(opposite_t, *self._leg_walk_handles['rc']))
        self.rb_leg.set_position(self._standard_walk_cycle(t, *self._leg_walk_handles['rb']))

        self.lf_leg.set_position(self._standard_walk_cycle(opposite_t, *self._leg_walk_handles['lf']))
        self.lc_leg.set_position(self._standard_walk_cycle(t, *self._leg_walk_handles['lc']))
        self.lb_leg.set_position(self._standard_walk_cycle(opposite_t, *self._leg_walk_handles['lb']))

    def shimmy(self, t):
        self.rf_leg.set_position(self._shimmy_cycle(t, *self._leg_shimmy_handles['rf']))
        self.rc_leg.set_position(self._shimmy_cycle(t, *self._leg_shimmy_handles['rc']))
        self.rb_leg.set_position(self._shimmy_cycle(t, *self._leg_shimmy_handles['rb']))

        self.lf_leg.set_position(self._shimmy_cycle(t, *self._leg_shimmy_handles['lf']))
        self.lc_leg.set_position(self._shimmy_cycle(t, *self._leg_shimmy_handles['lc']))
        self.lb_leg.set_position(self._shimmy_cycle(t, *self._leg_shimmy_handles['lb']))

    def _build_shimmy_cycle(self):
        self._leg_shimmy_handles = {
            'rf': (
                self._shimmy_cycle_handles[0].translate(Coord3D(20, 45, 0)).rotate(-self.hip_rotation, 'z'), 
                self._shimmy_cycle_handles[1].translate(Coord3D(20, 45, 0)).rotate(-self.hip_rotation, 'z')
            ),
            'rc': (
                self._shimmy_cycle_handles[0], 
                self._shimmy_cycle_handles[1]
            ),
            'rb': (
                self._shimmy_cycle_handles[0].translate(Coord3D(20, -45, 0)).rotate(self.hip_rotation, 'z'),
                self._shimmy_cycle_handles[1].translate(Coord3D(20, -45, 0)).rotate(self.hip_rotation, 'z')
            ),
            'lf': (
                self._shimmy_cycle_handles[1].translate(Coord3D(20, -45, 0)).rotate(self.hip_rotation, 'z'),
                self._shimmy_cycle_handles[0].translate(Coord3D(20, -45, 0)).rotate(self.hip_rotation, 'z')            ),
            'lc': (
                self._shimmy_cycle_handles[1], 
                self._shimmy_cycle_handles[0]
            ),
            'lb': (
                self._shimmy_cycle_handles[1].translate(Coord3D(20, 45, 0)).rotate(-self.hip_rotation, 'z'),
                self._shimmy_cycle_handles[0].translate(Coord3D(20, 45, 0)).rotate(-self.hip_rotation, 'z')
            ),
        }

    def _build_standard_walk_cycle(self):
        self._leg_walk_handles = {
            'rf': (
                self._walk_cycle_handles[0].translate(Coord3D(45, 45, 0)).rotate(-self.hip_rotation, 'z'), 
                self._walk_cycle_handles[1].translate(Coord3D(45, 45, 0)).rotate(-self.hip_rotation, 'z'),
                self._walk_cycle_handles[2].translate(Coord3D(45, 45, 0)).rotate(-self.hip_rotation, 'z')
            ),
            'rc': (
                self._walk_cycle_handles[0], 
                self._walk_cycle_handles[1], 
                self._walk_cycle_handles[2]
            ),
            'rb': (
                self._walk_cycle_handles[0].translate(Coord3D(45, -45, 0)).rotate(self.hip_rotation, 'z'),
                self._walk_cycle_handles[1].translate(Coord3D(45, -45, 0)).rotate(self.hip_rotation, 'z'),
                self._walk_cycle_handles[2].translate(Coord3D(45, -45, 0)).rotate(self.hip_rotation, 'z')
            ),
            'lf': (
                self._walk_cycle_handles[1].translate(Coord3D(45, -45, 0)).rotate(self.hip_rotation, 'z'),
                self._walk_cycle_handles[0].translate(Coord3D(45, -45, 0)).rotate(self.hip_rotation, 'z'),
                self._walk_cycle_handles[2].translate(Coord3D(45, -45, 0)).rotate(self.hip_rotation, 'z')
            ),
            'lc': (
                self._walk_cycle_handles[1], 
                self._walk_cycle_handles[0], 
                self._walk_cycle_handles[2]
            ),
            'lb': (
                self._walk_cycle_handles[1].translate(Coord3D(45, 45, 0)).rotate(-self.hip_rotation, 'z'),
                self._walk_cycle_handles[0].translate(Coord3D(45, 45, 0)).rotate(-self.hip_rotation, 'z'),
                self._walk_cycle_handles[2].translate(Coord3D(45, 45, 0)).rotate(-self.hip_rotation, 'z')
            ),
        }
        print(self._leg_walk_handles)

    def _shimmy_cycle(self, t:float, forward_point: Coord3D, backward_point: Coord3D):
        if t<.5:
            t = t * 2
            return lerp_3d(forward_point, backward_point, t)
        else:
            t = (t - .5) * 2
            return lerp_3d(backward_point, forward_point, t)
        
    def _standard_walk_cycle(self, t: float, forward_point: Coord3D, backward_point: Coord3D, lift_point: Coord3D):
        if t < .5:
            t = t * 2
            return lerp_3d(forward_point, backward_point, t)
        else:
            t = (t - .5) * 2
            return quad_bez_3d(backward_point, lift_point, forward_point, t)
