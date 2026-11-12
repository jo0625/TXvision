import math

import pytest

from aimer_geometry import (
    Point3,
    geometric_angles,
    reconstruct_armor_points,
    select_closest_forward_point,
    solve_ballistic_pitch,
)


def test_reconstructs_four_armor_geometry_and_alternating_height():
    points = reconstruct_armor_points(
        center=Point3(5.0, 0.0, 0.5),
        velocity=Point3(0.0, 0.0, 0.0),
        yaw=0.0,
        yaw_velocity=0.0,
        armor_count=4,
        radius_1=0.2,
        radius_2=0.3,
        dz=0.1,
    )
    assert points[0] == Point3(4.8, 0.0, 0.5)
    assert points[1].x == pytest.approx(5.0)
    assert points[1].y == pytest.approx(-0.3)
    assert points[1].z == pytest.approx(0.6)
    assert points[2].x == pytest.approx(5.2)
    assert points[2].z == pytest.approx(0.5)
    assert points[3].y == pytest.approx(0.3)
    assert points[3].z == pytest.approx(0.6)


def test_prediction_projects_center_and_yaw():
    points = reconstruct_armor_points(
        Point3(4.0, 0.0, 0.0),
        Point3(1.0, 0.0, 0.0),
        0.0,
        math.pi / 2.0,
        2,
        0.2,
        0.2,
        0.0,
        prediction_time=1.0,
    )
    assert points[0].x == pytest.approx(5.0)
    assert points[0].y == pytest.approx(-0.2)


def test_selects_closest_forward_armor():
    point = select_closest_forward_point(
        [Point3(-1.0, 0.0, 0.0), Point3(4.8, 0.0, 0.5), Point3(5.2, 0.0, 0.5)],
        minimum_forward=0.05,
        minimum_distance=0.1,
    )
    assert point == Point3(4.8, 0.0, 0.5)


def test_geometric_angles_use_forward_left_up_convention():
    yaw, pitch = geometric_angles(Point3(1.0, 1.0, 1.0))
    assert yaw == pytest.approx(math.pi / 4.0)
    assert pitch == pytest.approx(math.atan2(1.0, math.sqrt(2.0)))


@pytest.mark.parametrize("armor_count", [0, 1, 5])
def test_rejects_unsupported_armor_count(armor_count):
    with pytest.raises(ValueError):
        reconstruct_armor_points(
            Point3(1.0, 0.0, 0.0),
            Point3(0.0, 0.0, 0.0),
            0.0,
            0.0,
            armor_count,
            0.2,
            0.2,
            0.0,
        )


def test_rejects_nonfinite_and_nonforward_points():
    with pytest.raises(ValueError):
        select_closest_forward_point(
            [Point3(-1.0, 0.0, 0.0), Point3(math.nan, 0.0, 0.0)], 0.05, 0.1
        )


def test_ballistic_low_arc_reaches_target():
    point = Point3(5.0, 0.0, 0.5)
    pitch, flight_time = solve_ballistic_pitch(point, 20.0)
    assert pitch > math.atan2(point.z, point.x)
    assert 20.0 * math.cos(pitch) * flight_time == pytest.approx(5.0)
    assert (20.0 * math.sin(pitch) * flight_time
            - 0.5 * 9.81 * flight_time**2) == pytest.approx(0.5)
    assert flight_time < 1.0


@pytest.mark.parametrize("point,speed", [
    (Point3(100.0, 0.0, 0.0), 10.0),
    (Point3(1.0, 0.0, 0.0), 0.0),
    (Point3(math.nan, 0.0, 0.0), 20.0),
    (Point3(0.0, 0.0, 1.0), 20.0),
])
def test_ballistic_rejects_invalid_or_unreachable_input(point, speed):
    with pytest.raises(ValueError):
        solve_ballistic_pitch(point, speed)
