import math

import pytest

from gimbal_kinematics import calibrated_pitch, quaternion_from_rpy


def rotate_vector(quaternion, vector):
    x, y, z, w = quaternion
    vx, vy, vz = vector
    # q * v * q^-1, expanded to avoid a runtime dependency in this unit test.
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + y * tz - z * ty,
        vy + w * ty + z * tx - x * tz,
        vz + w * tz + x * ty - y * tx,
    )


def test_pitch_calibration_zero_and_scale():
    offset = 0.319451511
    scale = 1.023925824751
    ten_degrees = math.radians(10.0)

    assert calibrated_pitch(offset, offset, scale) == pytest.approx(0.0)
    assert calibrated_pitch(offset + scale * ten_degrees, offset, scale) == pytest.approx(
        ten_degrees
    )
    assert calibrated_pitch(offset - scale * ten_degrees, offset, scale) == pytest.approx(
        -ten_degrees
    )


@pytest.mark.parametrize("scale", [0.0, math.nan, math.inf, -math.inf])
def test_pitch_calibration_rejects_invalid_scale(scale):
    with pytest.raises(ValueError):
        calibrated_pitch(0.3, 0.3, scale)


@pytest.mark.parametrize("value", [math.nan, math.inf, -math.inf])
def test_pitch_calibration_rejects_nonfinite_angles(value):
    with pytest.raises(ValueError):
        calibrated_pitch(value, 0.3, 1.0)
    with pytest.raises(ValueError):
        calibrated_pitch(0.3, value, 1.0)


def test_positive_yaw_turns_forward_toward_robot_left():
    quaternion = quaternion_from_rpy(0.0, 0.0, math.pi / 2.0)
    assert rotate_vector(quaternion, (1.0, 0.0, 0.0)) == pytest.approx(
        (0.0, 1.0, 0.0), abs=1e-12
    )


def test_negative_ros_pitch_turns_forward_up():
    quaternion = quaternion_from_rpy(0.0, -math.pi / 2.0, 0.0)
    assert rotate_vector(quaternion, (1.0, 0.0, 0.0)) == pytest.approx(
        (0.0, 0.0, 1.0), abs=1e-12
    )


def test_standard_optical_frame_axes():
    quaternion = quaternion_from_rpy(-math.pi / 2.0, 0.0, -math.pi / 2.0)
    assert rotate_vector(quaternion, (0.0, 0.0, 1.0)) == pytest.approx(
        (1.0, 0.0, 0.0), abs=1e-12
    )
    assert rotate_vector(quaternion, (1.0, 0.0, 0.0)) == pytest.approx(
        (0.0, -1.0, 0.0), abs=1e-12
    )
    assert rotate_vector(quaternion, (0.0, 1.0, 0.0)) == pytest.approx(
        (0.0, 0.0, -1.0), abs=1e-12
    )
