from hexapod.utils import Coord3D
from hexapod.interpolation import lerp_3d, quad_bez_3d

class Body:
    walk_cycle_points = [
        Coord3D(80, 30, -50),
        Coord3D(80, -30, -50),
        Coord3D(80, 0, 0)
    ]

    def __init__(self, fr, cr, br, fl, cl, bl):
        self.fl_leg = fl
        self.fr_leg = fr
        self.cl_leg = cl
        self.cr_leg = cr
        self.bl_leg = bl
        self.br_leg = br
        self._build_standard_walk_cycle()
    
    def set_legs_active(self, legs):
        self.fr_leg.enabled = legs[0]
        self.cr_leg.enabled = legs[1]
        self.br_leg.enabled = legs[2]
        self.fl_leg.enabled = legs[3]
        self.cl_leg.enabled = legs[4]
        self.bl_leg.enabled = legs[5]

    def walk(self, t):
        opposite_t = (t + .5) % 1
        self.fr_leg.set_position(self._standard_walk_cycle(t, *self.standard_walk_cycle_points['fr']))
        self.cr_leg.set_position(self._standard_walk_cycle(opposite_t, *self.standard_walk_cycle_points['cr']))
        self.br_leg.set_position(self._standard_walk_cycle(t, *self.standard_walk_cycle_points['br']))

        self.fl_leg.set_position(self._standard_walk_cycle(opposite_t, *self.standard_walk_cycle_points['fl']))
        self.cl_leg.set_position(self._standard_walk_cycle(t, *self.standard_walk_cycle_points['cl']))
        self.bl_leg.set_position(self._standard_walk_cycle(opposite_t, *self.standard_walk_cycle_points['bl']))

    def _build_standard_walk_cycle(self):
        front_r_translate = Coord3D(45, 45, 0)
        front_l_translate = front_r_translate.mirror('y')
        back_r_translate = front_r_translate.mirror('x')
        back_l_translate = front_l_translate.mirror('x')
        self.standard_walk_cycle_points = {
            'fr': (
                self.walk_cycle_points[0].translate(front_r_translate).rotate(-60, 'z'), 
                self.walk_cycle_points[1].translate(front_r_translate).rotate(-60, 'z'),
                self.walk_cycle_points[2].translate(front_r_translate).rotate(-60, 'z')
            ),
            'cr': (
                self.walk_cycle_points[0], 
                self.walk_cycle_points[1], 
                self.walk_cycle_points[2]
            ),
            'br': (
                self.walk_cycle_points[0].translate(back_r_translate).rotate(60, 'z'),
                self.walk_cycle_points[1].translate(back_r_translate).rotate(60, 'z'),
                self.walk_cycle_points[2].translate(back_r_translate).rotate(60, 'z')
            ),
            'fl': (
                self.walk_cycle_points[0].mirror('y').translate(front_l_translate).rotate(60, 'z'),
                self.walk_cycle_points[1].mirror('y').translate(front_l_translate).rotate(60, 'z'),
                self.walk_cycle_points[2].mirror('y').translate(front_l_translate).rotate(60, 'z')
            ),
            'cl': (
                self.walk_cycle_points[0].mirror('y'), 
                self.walk_cycle_points[1].mirror('y'), 
                self.walk_cycle_points[2].mirror('y')
            ),
            'bl': (
                self.walk_cycle_points[0].mirror('y').translate(back_l_translate).rotate(-60, 'z'),
                self.walk_cycle_points[1].mirror('y').translate(back_l_translate).rotate(-60, 'z'),
                self.walk_cycle_points[2].mirror('y').translate(back_l_translate).rotate(-60, 'z')
            )
        }

    def _standard_walk_cycle(self, t: float, forward_point: Coord3D, backward_point: Coord3D, lift_point: Coord3D):
        if t < .5:
            t = t * 2
            return lerp_3d(forward_point, backward_point, t)
        else:
            t = (t - .5) * 2
            return quad_bez_3d(backward_point, lift_point, forward_point, t)
