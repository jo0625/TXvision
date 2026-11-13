"""Gravity-only low-arc solver, following SP Vision's Trajectory formulation.

Inputs use metres and seconds in a gravity-aligned frame, relative to the
projectile exit. Output elevation is in radians, positive upward.
See the package's LICENSE.sp_vision for the upstream MIT notice.
"""

from __future__ import annotations

import math


def solve_ballistic_pitch(
    point, projectile_speed: float, gravity: float = 9.81
) -> tuple[float, float]:
    """Return the low-arc elevation and flight time for a point relative to the muzzle."""
    if not all(math.isfinite(value) for value in (
        point.x, point.y, point.z, projectile_speed, gravity
    )):
        raise ValueError("ballistic inputs must be finite")
    if projectile_speed <= 0.0 or gravity <= 0.0:
        raise ValueError("projectile speed and gravity must be positive")

    distance = math.hypot(point.x, point.y)
    if distance <= 0.0:
        raise ValueError("ballistic target must have positive horizontal distance")
    a = gravity * distance * distance / (2.0 * projectile_speed * projectile_speed)
    discriminant = distance * distance - 4.0 * a * (a + point.z)
    if discriminant < 0.0:
        raise ValueError("ballistic target is unreachable")

    # The smaller root gives the shorter flight time. This form avoids cancellation
    # when the target is close to the launch height.
    tangent = 2.0 * (a + point.z) / (distance + math.sqrt(discriminant))
    pitch = math.atan(tangent)
    flight_time = distance / (projectile_speed * math.cos(pitch))
    if not math.isfinite(flight_time):
        raise ValueError("ballistic flight time is not finite")
    return pitch, flight_time
