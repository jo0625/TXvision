"""Pure geometry used by the basic armor aimer."""

from __future__ import annotations

from dataclasses import dataclass
import math

# Preserve the original import path used by callers and existing tests.
from ballistic_solver import solve_ballistic_pitch


@dataclass(frozen=True)
class Point3:
    x: float
    y: float
    z: float


def reconstruct_armor_points(
    center: Point3,
    velocity: Point3,
    yaw: float,
    yaw_velocity: float,
    armor_count: int,
    radius_1: float,
    radius_2: float,
    dz: float,
    prediction_time: float = 0.0,
) -> list[Point3]:
    values = (
        center.x,
        center.y,
        center.z,
        velocity.x,
        velocity.y,
        velocity.z,
        yaw,
        yaw_velocity,
        radius_1,
        radius_2,
        dz,
        prediction_time,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("target state must be finite")
    if armor_count not in (2, 3, 4):
        raise ValueError("armor_count must be 2, 3, or 4")
    if radius_1 <= 0.0 or (armor_count == 4 and radius_2 <= 0.0):
        raise ValueError("armor radii must be positive")
    if prediction_time < 0.0:
        raise ValueError("prediction_time must not be negative")

    predicted_center = Point3(
        center.x + velocity.x * prediction_time,
        center.y + velocity.y * prediction_time,
        center.z + velocity.z * prediction_time,
    )
    predicted_yaw = yaw + yaw_velocity * prediction_time
    points = []
    for index in range(armor_count):
        armor_yaw = predicted_yaw + index * 2.0 * math.pi / armor_count
        use_first_pair = armor_count != 4 or index % 2 == 0
        radius = radius_1 if use_first_pair else radius_2
        height_offset = 0.0 if use_first_pair else dz
        points.append(
            Point3(
                predicted_center.x - radius * math.cos(armor_yaw),
                predicted_center.y - radius * math.sin(armor_yaw),
                predicted_center.z + height_offset,
            )
        )
    return points


def select_closest_forward_point(
    points: list[Point3], minimum_forward: float, minimum_distance: float
) -> Point3:
    valid = []
    for point in points:
        distance_squared = point.x * point.x + point.y * point.y + point.z * point.z
        if (
            all(math.isfinite(value) for value in (point.x, point.y, point.z))
            and point.x > minimum_forward
            and distance_squared >= minimum_distance * minimum_distance
        ):
            valid.append((distance_squared, point))
    if not valid:
        raise ValueError("no forward armor point is valid")
    return min(valid, key=lambda item: item[0])[1]


def geometric_angles(point: Point3) -> tuple[float, float]:
    if not all(math.isfinite(value) for value in (point.x, point.y, point.z)):
        raise ValueError("aim point must be finite")
    horizontal = math.hypot(point.x, point.y)
    if horizontal == 0.0:
        raise ValueError("aim point has no horizontal direction")
    return math.atan2(point.y, point.x), math.atan2(point.z, horizontal)
