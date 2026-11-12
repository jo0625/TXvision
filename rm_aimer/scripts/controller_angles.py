"""Candidate controller angles for the existing yaw/pitch kinematic model."""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class FeedbackSample:
    stamp: float
    yaw: float
    pitch: float


def match_feedback(samples, command_stamp, now, max_age, max_skew):
    if not all(math.isfinite(v) for v in (command_stamp, now, max_age, max_skew)):
        raise ValueError("nonfinite_timestamp")
    if command_stamp <= 0 or not 0 <= now - command_stamp <= max_age:
        raise ValueError("command_timestamp_not_current")
    candidates = [s for s in samples
                  if s.stamp > 0 and 0 <= now - s.stamp <= max_age]
    if not candidates:
        raise ValueError("no_fresh_feedback")
    sample = min(candidates, key=lambda s: abs(s.stamp - command_stamp))
    if abs(sample.stamp - command_stamp) > max_skew:
        raise ValueError("feedback_timestamp_mismatch")
    return sample


def controller_targets(
    geometric_yaw, geometric_elevation, current_yaw, current_pitch,
    pitch_offset, pitch_scale, yaw_sign=1.0,
):
    """Map a ray in pitch_joint to continuous yaw and raw pitch candidates.

    Assumes feedback yaw is the controller's continuous target coordinate and
    feedback pitch shares the target encoder coordinate. These assumptions
    must be confirmed against the flashed firmware before transmitting.
    """
    values = (geometric_yaw, geometric_elevation, current_yaw, current_pitch,
              pitch_offset, pitch_scale, yaw_sign)
    if not all(math.isfinite(v) for v in values):
        raise ValueError("nonfinite_angle")
    if pitch_scale == 0 or yaw_sign not in (-1.0, 1.0):
        raise ValueError("invalid_axis_calibration")
    if abs(geometric_yaw) > math.pi or abs(geometric_elevation) > math.pi / 2:
        raise ValueError("invalid_geometric_angle")
    joint_pitch = (current_pitch - pitch_offset) / pitch_scale
    if abs(joint_pitch) >= math.pi / 2:
        raise ValueError("pitch_outside_supported_range")

    # Rotate the sight ray out of the pitched joint into the unpitched yaw axes.
    x = math.cos(geometric_elevation) * math.cos(geometric_yaw)
    y = math.cos(geometric_elevation) * math.sin(geometric_yaw)
    z = math.sin(geometric_elevation)
    cp, sp = math.cos(joint_pitch), math.sin(joint_pitch)
    yaw_x = cp * x + sp * z
    yaw_z = -sp * x + cp * z
    horizontal = math.hypot(yaw_x, y)
    if horizontal < 1e-8:
        raise ValueError("vertical_ray_has_no_yaw")
    delta_yaw = math.atan2(y, yaw_x)
    target_joint_pitch = -math.atan2(yaw_z, horizontal)
    return (current_yaw + yaw_sign * delta_yaw,
            pitch_offset + pitch_scale * target_joint_pitch)
