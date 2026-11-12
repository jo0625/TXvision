"""Check solutions against projectile motion, independent of the root formula."""

import math

import pytest

from aimer_geometry import Point3
from ballistic_solver import solve_ballistic_pitch


@pytest.mark.parametrize("point,speed", [
    (Point3(5.0, 0.0, 0.0), 20.0),
    (Point3(3.0, 4.0, 1.0), 20.0),
    (Point3(5.0, 0.0, -2.0), 20.0),
    (Point3(1.0, 0.0, 0.0), 30.0),
])
def test_low_arc_arrives_at_target_in_three_dimensions(point, speed):
    gravity = 9.7833
    pitch, flight_time = solve_ballistic_pitch(point, speed, gravity)
    yaw = math.atan2(point.y, point.x)
    assert flight_time > 0.0
    assert speed * math.cos(pitch) * math.cos(yaw) * flight_time == pytest.approx(point.x)
    assert speed * math.cos(pitch) * math.sin(yaw) * flight_time == pytest.approx(point.y)
    assert (speed * math.sin(pitch) * flight_time
            - gravity * flight_time**2 / 2.0) == pytest.approx(point.z, abs=1e-10)
    # Compare against the slower second physical arc.
    distance = math.hypot(point.x, point.y)
    a = gravity * distance**2 / (2.0 * speed**2)
    high_tangent = (distance + math.sqrt(distance**2 - 4*a*(a+point.z))) / (2*a)
    high_time = distance / (speed * math.cos(math.atan(high_tangent)))
    assert flight_time < high_time


def test_maximum_level_range_has_one_solution():
    pitch, flight_time = solve_ballistic_pitch(Point3(40.0, 0.0, 0.0), 20.0, 10.0)
    assert pitch == pytest.approx(math.pi / 4.0)
    assert flight_time == pytest.approx(2.0 * 20.0 * math.sin(pitch) / 10.0)
    with pytest.raises(ValueError, match="unreachable"):
        solve_ballistic_pitch(Point3(40.01, 0.0, 0.0), 20.0, 10.0)


@pytest.mark.parametrize("speed,gravity", [
    (-1.0, 9.81), (0.0, 9.81), (math.inf, 9.81), (math.nan, 9.81),
    (20.0, 0.0), (20.0, -9.81), (20.0, math.nan),
])
def test_invalid_physical_parameters_are_rejected(speed, gravity):
    with pytest.raises(ValueError):
        solve_ballistic_pitch(Point3(5.0, 0.0, 0.0), speed, gravity)
