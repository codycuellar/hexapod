from hexapod.interpolation import lerp_3d, quad_bez_3d
from hexapod.utils import Coord3D
import math

def walk_cycle(t: float, forward_point: Coord3D, backward_point: Coord3D, lift_point: Coord3D):
    if t < .5:
        t = t * 2
        return lerp_3d(forward_point, backward_point, t)
    else:
        t = (t - .5) * 2
        return quad_bez_3d(backward_point, lift_point, forward_point, t)

def circle_pattern(t, center_point):
    x = center_point[0] + 50 * math.cos(t)
    y = center_point[1] + 50 * math.sin(t)
    z = center_point[2]
    return x, y, z