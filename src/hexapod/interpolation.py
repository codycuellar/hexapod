import math

class Point:
    def __init__(self, x: float, y: float):
        self.x = x
        self.y = y

    def as_list(self):
        return [self.x, self.y]

def lerp(v1:float, v2:float, t:float):
    return v1 + (v2 - v1) * t

def plerp(p1:Point, p2:Point, t:float):
    return Point(lerp(p1.x, p2.x, t), lerp(p1.y, p2.y, t))

def nlerp(p1:Point, p2:Point, t, planar:bool=False):
    if planar:
        return ([lerp(p1.x, p2.x, _t) for _t in t], [lerp(p1.y, p2.y, _t) for _t in t])
    else:
        return [plerp(p1, p2, i).as_list() for i in t]

def quad_bez(v1:float, v2:float, v3:float, t:float):
    return lerp(lerp(v1, v2, t), lerp(v2, v3, t), t)

def pquad_bez(p1:Point, p2:Point, p3:Point, t:float):
    return plerp(plerp(p1, p2, t), plerp(p2, p3, t), t)

def nquad_bez(p1:Point, p2:Point, p3:Point, t, planar:bool=False):
    if planar:
        return ([quad_bez(p1.x, p2.x, p3.x, _t) for _t in t], [quad_bez(p1.y, p2.y, p3.y, _t) for _t in t])
    else:
        return [pquad_bez(p1, p2, p3, i).as_list() for i in t]

def cubic_bez(v1:float, v2:float, v3:float, v4:float, t:float, style='decasteljau'):
    if style == 'decasteljau':
        return lerp(quad_bez(v1, v2, v3, t), quad_bez(v2, v3, v4, t), t)
    elif 'bernstein':
        return v1 * (-math.pow(t, 3) + (3 * math.pow(t, 2)) - (3 * t) + 1) + \
               v2 * (3 * math.pow(t, 3) - (6 * math.pow(t, 2)) + (3 * t)) + \
               v3 * (-3 * math.pow(t, 3) + 3 * math.pow(t, 2)) + \
               v4 * math.pow(t, 3)

def pcubic_bez(p1:Point, p2:Point, p3:Point, p4:Point, t:float):
    return plerp(pquad_bez(p1, p2, p3, t), pquad_bez(p2, p3, p4, t), t)

def ncubic_bez(p1:Point, p2:Point, p3:Point, p4:Point, t, planar:bool=False):
    if planar:
        return ([cubic_bez(p1.x, p2.x, p3.x, p4.x, _t) for _t in t], [cubic_bez(p1.y, p2.y, p3.y, p4.y, _t) for _t in t])
    else:
        return [pcubic_bez(p1, p2, p3, p4, i).as_list() for i in t]

def cosine_ease(t:float):
    return (1 - math.cos(t * math.pi)) / 2

def ncosine_ease(t, planar:bool=False):
    if planar:
        last = t[-1]
        return  ([_t/last for _t in t], [cosine_ease(_t/last) for _t in t])
    else:
        return [cosine_ease(_t) for _t in t]
