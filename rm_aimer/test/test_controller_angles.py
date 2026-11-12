import math

import pytest

from controller_angles import FeedbackSample, controller_targets, match_feedback


@pytest.mark.parametrize('yaw', [-4 * math.pi, 0.0, math.pi - 0.01, 6 * math.pi])
@pytest.mark.parametrize('joint_pitch', [-0.4, 0.0, 0.3])
def test_center_ray_preserves_controller_coordinates(yaw, joint_pitch):
    offset, scale = 0.319451511, 1.023925824751
    raw_pitch = offset + scale * joint_pitch
    assert controller_targets(0.0, 0.0, yaw, raw_pitch, offset, scale) == pytest.approx(
        (yaw, raw_pitch)
    )


def test_head_up_and_yaw_crossing_pi_use_controller_conventions():
    yaw, pitch = controller_targets(0.1, 0.2, math.pi - 0.01, 0.32, 0.32, 1.02)
    assert yaw == pytest.approx(math.pi + 0.09)
    assert pitch == pytest.approx(0.32 - 1.02 * 0.2)


@pytest.mark.parametrize('joint_pitch, local_yaw, elevation', [
    (-0.5, 0.5, 0.2), (0.3, -0.6, -0.2), (0.4, 0.8, 0.3),
])
def test_combined_motion_preserves_sight_direction(joint_pitch, local_yaw, elevation):
    target_yaw, target_pitch = controller_targets(
        local_yaw, elevation, 0.0, joint_pitch, 0.0, 1.0
    )
    x = math.cos(elevation) * math.cos(local_yaw)
    y = math.cos(elevation) * math.sin(local_yaw)
    z = math.sin(elevation)
    desired = (math.cos(joint_pitch) * x + math.sin(joint_pitch) * z,
               y, -math.sin(joint_pitch) * x + math.cos(joint_pitch) * z)
    achieved = (math.cos(target_yaw) * math.cos(target_pitch),
                math.sin(target_yaw) * math.cos(target_pitch), -math.sin(target_pitch))
    assert achieved == pytest.approx(desired)
    # A simple independent angle addition is wrong for combined yaw/pitch motion.
    assert abs(target_yaw - local_yaw) > 0.01


@pytest.mark.parametrize('args', [
    (math.nan, 0, 0, 0, 0, 1), (0, math.inf, 0, 0, 0, 1),
    (0, 0, 0, 0, 0, 0), (0, 0, 0, math.pi / 2, 0, 1),
    (0, math.pi / 2, 0, 0, 0, 1), (0, 2.0, 0, 0, 0, 1),
])
def test_rejects_invalid_or_singular_geometry(args):
    with pytest.raises(ValueError):
        controller_targets(*args)


def test_feedback_is_paired_to_target_time_not_latest_arrival():
    old = FeedbackSample(10.02, 0.1, 0.2)
    new = FeedbackSample(10.10, 0.5, 0.7)
    assert match_feedback([old, new], 10.03, 10.12, 0.2, 0.05) is old


@pytest.mark.parametrize('stamp, samples, reason', [
    (0.0, [], 'command_timestamp'),
    (9.0, [], 'command_timestamp'),
    (10.2, [], 'command_timestamp'),
    (10.1, [], 'no_fresh_feedback'),
    (10.1, [FeedbackSample(9.0, 0, 0)], 'no_fresh_feedback'),
    (10.1, [FeedbackSample(10.0, 0, 0)], 'feedback_timestamp_mismatch'),
])
def test_rejects_stale_unstamped_future_or_unpaired_data(stamp, samples, reason):
    with pytest.raises(ValueError, match=reason):
        match_feedback(samples, stamp, 10.15, 0.2, 0.05)
