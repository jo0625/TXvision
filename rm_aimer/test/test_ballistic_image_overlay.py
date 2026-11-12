import math
from types import SimpleNamespace

import numpy as np
import pytest
from geometry_msgs.msg import Transform
from sensor_msgs.msg import CameraInfo

from ballistic_image_overlay import ballistic_points, matching_solution, project_points
from ballistic_solver import solve_ballistic_pitch


def calibration():
    info = CameraInfo(width=1440, height=1080)
    info.header.frame_id = "camera_optical_frame"
    info.k = [1740.0, 0.0, 720.0, 0.0, 1740.0, 540.0, 0.0, 0.0, 1.0]
    info.d = [0.0] * 5
    info.distortion_model = "plumb_bob"
    return info


def test_debug_resize_projection_and_points_behind_camera():
    transform = Transform()
    transform.rotation.w = 1.0
    pixels = project_points([[0, 0, 5], [1, 0, 5], [0, 0, -1]], transform, calibration(), 960, 720)
    np.testing.assert_allclose(pixels[:2], [[480, 360], [712, 360]])
    assert np.isnan(pixels[2]).all()
    with pytest.raises(ValueError, match="aspect"):
        project_points([[0, 0, 5]], transform, calibration(), 960, 700)


def test_optical_rotation_and_camera_translation():
    transform = Transform()
    transform.rotation.z = math.sin(math.pi / 4)
    transform.rotation.w = math.cos(math.pi / 4)
    transform.translation.x = 0.5
    pixels = project_points([[1, 0, 5]], transform, calibration(), 1440, 1080)
    np.testing.assert_allclose(pixels, [[894, 888]], atol=1e-9)


def test_distortion_is_applied_before_debug_resize():
    transform = Transform()
    transform.rotation.w = 1.0
    info = calibration()
    info.d = [0.1, 0.0, 0.0, 0.0, 0.0]
    pixels = project_points([[1, 0, 5]], transform, info, 960, 720)
    np.testing.assert_allclose(pixels, [[712.928, 360]])


def test_trajectory_reaches_solver_target_and_aim_marker_includes_gravity_drop():
    relative = SimpleNamespace(x=5.0, y=0.7, z=0.4)
    pitch, flight = solve_ballistic_pitch(relative, 19.0)
    muzzle = np.array([0.025, 0.0, 0.2])
    target = muzzle + [relative.x, relative.y, relative.z]
    data = dict(schema=1, valid=True, ballistics_enabled=True,
                muzzle_position_gravity_m=muzzle.tolist(), target_position_gravity_m=target.tolist(),
                relative_target_gravity_m=[relative.x, relative.y, relative.z],
                projectile_speed_mps=19.0, gravity_mps2=9.81, flight_time_s=flight,
                ballistic_pitch_gravity_rad=pitch)
    trajectory, aim = ballistic_points(data)
    np.testing.assert_allclose(trajectory[0], muzzle)
    np.testing.assert_allclose(trajectory[-1], target, atol=1e-9)
    np.testing.assert_allclose(aim - trajectory[-1], [0, 0, 0.5 * 9.81 * flight**2], atol=1e-9)
    with pytest.raises(ValueError, match="no valid"):
        ballistic_points(dict(data, valid=False))
    with pytest.raises(ValueError, match="endpoint"):
        ballistic_points(dict(data, target_position_gravity_m=[20, 0, 0]))


def test_only_fresh_exact_stamp_ballistic_solutions_are_marked():
    stamp = 1_000_000_000
    data = dict(valid=True, ballistics_enabled=True, source_stamp_ns=stamp)
    assert matching_solution(data, stamp, stamp + 50_000_000)
    assert not matching_solution(data, stamp + 1, stamp + 50_000_000)
    assert not matching_solution(data, stamp, stamp + 600_000_000)
    assert not matching_solution(data, stamp, stamp - 1)
    assert not matching_solution(dict(data, valid=False), stamp, stamp)
    assert not matching_solution(dict(data, ballistics_enabled=False), stamp, stamp)
