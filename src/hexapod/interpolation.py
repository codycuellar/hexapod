import math

from hexapod.matmath import Vector
from hexapod.engine import Vec3d


def lerp(v1: float, v2: float, t: float):
    return v1 + (v2 - v1) * t


def lerp_2d(p1: Vector, p2: Vector, t: float):
    return Vector([lerp(p1[0], p2[0], t), lerp(p1[1], p2[1], t)])


def lerp_3d(p1: Vec3d, p2: Vec3d, t: float):
    return Vec3d(lerp(p1.x, p2.x, t), lerp(p1.y, p2.y, t), lerp(p1.z, p2.z, t))


def quad_bez(v1: float, v2: float, v3: float, t: float):
    return lerp(lerp(v1, v2, t), lerp(v2, v3, t), t)


def quad_bez_2d(p1: Vector, p2: Vector, p3: Vector, t: float):

    return Vector([quad_bez(p1[0], p2[0], p3[0], t), quad_bez(p1[1], p2[1], p3[1], t)])


def quad_bez_3d(p1: Vec3d, p2: Vec3d, p3: Vec3d, t: float):
    return Vec3d(
        quad_bez(p1.x, p2.x, p3.x, t),
        quad_bez(p1.y, p2.y, p3.y, t),
        quad_bez(p1.z, p2.z, p3.z, t),
    )


def cubic_bez(
    v1: float, v2: float, v3: float, v4: float, t: float, style="decasteljau"
):
    if style == "decasteljau":
        return lerp(quad_bez(v1, v2, v3, t), quad_bez(v2, v3, v4, t), t)
    elif "bernstein":
        return (
            v1 * (-math.pow(t, 3) + (3 * math.pow(t, 2)) - (3 * t) + 1)
            + v2 * (3 * math.pow(t, 3) - (6 * math.pow(t, 2)) + (3 * t))
            + v3 * (-3 * math.pow(t, 3) + 3 * math.pow(t, 2))
            + v4 * math.pow(t, 3)
        )
    else:
        raise ValueError(f"Unknown style: {style}")


def cubic_bez_2d(
    p1: Vector, p2: Vector, p3: Vector, p4: Vector, t: float, style="decasteljau"
):
    return Vector(
        [
            cubic_bez(p1[0], p2[0], p3[0], p4[0], t, style),
            cubic_bez(p1[1], p2[1], p3[1], p4[1], t, style),
        ]
    )


def cubic_bez_3d(
    p1: Vec3d, p2: Vec3d, p3: Vec3d, p4: Vec3d, t: float, style="decasteljau"
):
    return Vec3d(
        cubic_bez(p1.x, p2.x, p3.x, p4.x, t, style),
        cubic_bez(p1.y, p2.y, p3.y, p4.y, t, style),
        cubic_bez(p1.z, p2.z, p3.z, p4.z, t, style),
    )


def cosine_ease_t(t: float):
    return (1 - math.cos(t * math.pi)) / 2
