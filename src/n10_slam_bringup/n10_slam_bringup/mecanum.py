import math
from typing import Iterable


def twist_to_wheel_rpm(
    vx: float,
    vy: float,
    omega: float,
    wheel_diameter: float,
    half_length: float,
    half_width: float,
) -> tuple[float, float, float, float]:
    if wheel_diameter <= 0.0:
        raise ValueError('wheel_diameter must be positive')
    rotation_radius = half_length + half_width
    metres_per_second_to_rpm = 60.0 / (math.pi * wheel_diameter)
    wheel_speeds = (
        vx - vy - omega * rotation_radius,
        vx + vy + omega * rotation_radius,
        vx + vy - omega * rotation_radius,
        vx - vy + omega * rotation_radius,
    )
    return tuple(value * metres_per_second_to_rpm for value in wheel_speeds)


def limit_wheel_rpm(
    wheel_rpm: Iterable[float], max_wheel_rpm: float
) -> tuple[float, float, float, float]:
    values = tuple(float(value) for value in wheel_rpm)
    if len(values) != 4:
        raise ValueError('exactly four wheel RPM values are required')
    if max_wheel_rpm <= 0.0:
        raise ValueError('max_wheel_rpm must be positive')
    peak = max(abs(value) for value in values)
    if peak <= max_wheel_rpm:
        return values
    scale = max_wheel_rpm / peak
    return tuple(value * scale for value in values)


def wheel_distances_to_body_delta(
    wheel_distances: Iterable[float],
    half_length: float,
    half_width: float,
) -> tuple[float, float, float]:
    lf, rf, lr, rr = tuple(float(value) for value in wheel_distances)
    rotation_radius = half_length + half_width
    if rotation_radius <= 0.0:
        raise ValueError('half_length + half_width must be positive')
    dx = (lf + rf + lr + rr) / 4.0
    dy = (-lf + rf + lr - rr) / 4.0
    dyaw = (-lf + rf - lr + rr) / (4.0 * rotation_radius)
    return dx, dy, dyaw
