from dataclasses import replace
import math
import pytest
from fire_permission import Command, Config, Feedback, FirePermission


def policy(**overrides):
    return FirePermission(replace(Config(enabled=True, calibration_verified=True), **overrides))


def samples(t):
    return Command(100 + t, t, 0.1, 0.2), Feedback(100 + t, t, 0.1, 0.2)


def step(p, t, command=None, feedback=None):
    a, b = samples(t)
    return p.evaluate(command or a, feedback or b, now=t, ros_now=100 + t)


def ready(p):
    assert not step(p, 0).allowed
    assert not step(p, 0.08).allowed
    result = step(p, 0.16)
    assert result.allowed and result.reason == "ready"


def test_defaults_require_explicit_enable_and_calibration():
    assert step(FirePermission(), 0).reason == "disabled"
    assert step(policy(calibration_verified=False), 0).reason == "calibration_unverified"


def test_missing_inputs():
    assert policy().evaluate(None, None, now=0, ros_now=100).reason == "waiting_for_data"


def test_stable_sequence_allows_preview():
    ready(policy())


def test_one_sample_republished_does_not_satisfy_sample_count():
    p = policy()
    a, b = samples(0)
    for t in (0, 0.08, 0.16):
        assert not p.evaluate(a, b, now=t, ros_now=100+t).allowed


def test_loss_revokes_permission_and_recovery_must_settle():
    p = policy()
    ready(p)
    a, b = samples(0.17)
    assert step(p, 0.17, replace(a, valid=False), b).reason == "invalid_command"
    assert not step(p, 0.18).allowed
    assert not step(p, 0.26).allowed
    assert step(p, 0.34).allowed


@pytest.mark.parametrize("field,value,reason", [
    ("frame", "pitch_joint", "frame_mismatch"),
    ("yaw", float("nan"), "nonfinite_command"),
    ("pitch", float("inf"), "nonfinite_command"),
    ("yaw", 0.2, "not_aligned"),
    ("pitch", 0.3, "not_aligned"),
    ("stamp", 99.0, "stale_or_future_command"),
    ("stamp", 101.0, "stale_or_future_command"),
    ("received", -1.0, "receive_timeout_command"),
])
def test_bad_command_immediately_revokes(field, value, reason):
    p = policy()
    ready(p)
    a, b = samples(0.17)
    result = step(p, 0.17, replace(a, **{field: value}), b)
    assert not result.allowed and result.reason == reason


@pytest.mark.parametrize("field,value,reason", [
    ("mode", 3, "wrong_mode"),
    ("yaw", float("nan"), "nonfinite_feedback"),
    ("stamp", 99.0, "stale_or_future_feedback"),
    ("received", -1.0, "receive_timeout_feedback"),
])
def test_bad_feedback_immediately_revokes(field, value, reason):
    p = policy()
    ready(p)
    a, b = samples(0.17)
    result = step(p, 0.17, a, replace(b, **{field: value}))
    assert not result.allowed and result.reason == reason


def test_timeout_with_paused_ros_clock():
    p = policy()
    ready(p)
    a, b = samples(0.16)
    result = p.evaluate(a, b, now=0.4, ros_now=100.16)
    assert not result.allowed and result.reason == "receive_timeout_command"


def test_long_processing_gap_restarts_settling():
    p = policy()
    ready(p)
    assert step(p, 1.0).reason == "settling"


def test_continuous_yaw_does_not_wrap_full_turn():
    a, b = samples(0)
    assert step(policy(), 0, replace(a, yaw=b.yaw+2*math.pi), b).reason == "not_aligned"


def test_skew_rejected_even_if_both_inputs_fresh():
    a, b = samples(0.15)
    assert step(policy(), 0.15, replace(a, stamp=100), b).reason == "timestamp_skew"


def test_out_of_order_rejected():
    p = policy()
    ready(p)
    a, b = samples(0.17)
    assert step(p, 0.17, replace(a, stamp=100.15), b).reason == "out_of_order"


def test_clock_rollback_revokes_and_resets():
    p = policy()
    ready(p)
    assert step(p, 0.1).reason == "clock_reset"
    assert step(p, 0.11).reason == "settling"


@pytest.mark.parametrize("kwargs", [
    {"max_age": 0}, {"settle_time": -1}, {"yaw_tolerance": float("nan")},
    {"pitch_tolerance": float("inf")}, {"min_samples": 1}, {"expected_frame": ""},
])
def test_bad_configuration_rejected(kwargs):
    with pytest.raises(ValueError):
        policy(**kwargs)
