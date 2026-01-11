import math

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


def cubic_bez(
    t: float,
    v1: float,
    v2: float,
    v3: float,
    v4: float,
) -> float:
    return lerp(t, quad_bez(t, v1, v2, v3), quad_bez(t, v2, v3, v4))


def cubic_bez_2d(
    t: float,
    p1: Vector,
    p2: Vector,
    p3: Vector,
    p4: Vector,
) -> Vec2d:
    return Vec2d(
        cubic_bez(t, p1[0], p2[0], p3[0], p4[0]),
        cubic_bez(t, p1[1], p2[1], p3[1], p4[1]),
    )


def cubic_bez_3d(
    t: float,
    p1: Vec3d,
    p2: Vec3d,
    p3: Vec3d,
    p4: Vec3d,
) -> Vec3d:
    return Vec3d(
        cubic_bez(t, p1.x, p2.x, p3.x, p4.x),
        cubic_bez(t, p1.y, p2.y, p3.y, p4.y),
        cubic_bez(t, p1.z, p2.z, p3.z, p4.z),
    )


def cosine_ease_t(t: float) -> float:
    return (1 - math.cos(t * math.pi)) / 2


def rate_limit(t: float, current: float, target: float, max_rate: float) -> float:
    delta = target - current
    max_delta = max_rate * t

    if abs(delta) > max_delta:
        delta = math.copysign(max_delta, delta)

    return current + delta


def rate_limit_vec(
    t: float, current: Vector, target: Vector, max_rate: float
) -> Vector:
    delta = target - current
    delta_len = delta.length()

    max_delta = max_rate * t

    if delta_len > max_delta and delta_len > 0.0:
        delta = delta * (max_delta / delta_len)

    return current + delta


def rate_limit_2d(t: float, current: Vec2d, target: Vec2d, max_rate: float) -> Vec2d:
    return Vec2d(*rate_limit_vec(t, current, target, max_rate))


def rate_limit_3d(t: float, current: Vec3d, target: Vec3d, max_rate: float) -> Vec3d:
    return Vec3d(*rate_limit_vec(t, current, target, max_rate))
