import math
from typing import Literal, Union, TypeAlias

from hexapod.matmath import Vector
from hexapod.engine import Vec2d, Vec3d


def lerp(t: float, v1: float, v2: float) -> float:
    return v1 + (v2 - v1) * t


def lerp_2d(t: float, p1: Vec2d, p2: Vec2d) -> Vec2d:
    return Vec2d(lerp(t, p1.x, p2.x), lerp(t, p1.y, p2.y))


def lerp_3d(t: float, p1: Vec3d, p2: Vec3d) -> Vec3d:
    return Vec3d(lerp(t, p1.x, p2.x), lerp(t, p1.y, p2.y), lerp(t, p1.z, p2.z))


def quad_bez(t: float, v1: float, v2: float, v3: float) -> float:
    return lerp(t, lerp(t, v1, v2), lerp(t, v2, v3))


def quad_bez_2d(t: float, p1: Vec2d, p2: Vec2d, p3: Vec2d) -> Vec2d:
    return Vec2d(quad_bez(t, p1.x, p2.x, p3.x), quad_bez(t, p1.y, p2.y, p3.y))


def quad_bez_3d(t: float, p1: Vec3d, p2: Vec3d, p3: Vec3d) -> Vec3d:
    return Vec3d(
        quad_bez(t, p1.x, p2.x, p3.x),
        quad_bez(t, p1.y, p2.y, p3.y),
        quad_bez(t, p1.z, p2.z, p3.z),
    )


LerpStyle: TypeAlias = Union[Literal["decasteljau"], Literal["bernstein"]]


def cubic_bez(
    t: float,
    v1: float,
    v2: float,
    v3: float,
    v4: float,
    style: LerpStyle = "decasteljau",
) -> float:
    if style == "decasteljau":
        return lerp(t, quad_bez(t, v1, v2, v3), quad_bez(t, v2, v3, v4))
    elif style == "bernstein":
        return (
            v1 * (-math.pow(t, 3) + (3 * math.pow(t, 2)) - (3 * t) + 1)
            + v2 * (3 * math.pow(t, 3) - (6 * math.pow(t, 2)) + (3 * t))
            + v3 * (-3 * math.pow(t, 3) + 3 * math.pow(t, 2))
            + v4 * math.pow(t, 3)
        )
    else:
        raise ValueError(f"Unknown style: {style}")


def cubic_bez_2d(
    t: float,
    p1: Vector,
    p2: Vector,
    p3: Vector,
    p4: Vector,
    style: LerpStyle = "decasteljau",
) -> Vec2d:
    return Vec2d(
        cubic_bez(t, p1[0], p2[0], p3[0], p4[0], style),
        cubic_bez(t, p1[1], p2[1], p3[1], p4[1], style),
    )


def cubic_bez_3d(
    t: float,
    p1: Vec3d,
    p2: Vec3d,
    p3: Vec3d,
    p4: Vec3d,
    style: LerpStyle = "decasteljau",
) -> Vec3d:
    return Vec3d(
        cubic_bez(t, p1.x, p2.x, p3.x, p4.x, style),
        cubic_bez(t, p1.y, p2.y, p3.y, p4.y, style),
        cubic_bez(t, p1.z, p2.z, p3.z, p4.z, style),
    )


def cosine_ease_t(t: float) -> float:
    return (1 - math.cos(t * math.pi)) / 2
