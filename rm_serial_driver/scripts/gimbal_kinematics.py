"""Pure kinematic conversions used by the gimbal TF broadcaster."""

from __future__ import annotations

import math


def calibrated_pitch(raw_pitch: float, offset: float, scale: float) -> float:
    """Convert the controller pitch field to a ROS joint angle in radians."""
    if not all(math.isfinite(value) for value in (raw_pitch, offset, scale)):
        raise ValueError("pitch calibration values must be finite")
    if scale == 0.0:
        raise ValueError("pitch calibration scale must be non-zero")
    return (raw_pitch - offset) / scale


def quaternion_from_rpy(
    roll: float, pitch: float, yaw: float
) -> tuple[float, float, float, float]:
    """Return an xyzw quaternion for fixed-axis roll, pitch and yaw."""
    if not all(math.isfinite(value) for value in (roll, pitch, yaw)):
        raise ValueError("roll, pitch and yaw must be finite")

    cr = math.cos(roll / 2.0)
    sr = math.sin(roll / 2.0)
    cp = math.cos(pitch / 2.0)
    sp = math.sin(pitch / 2.0)
    cy = math.cos(yaw / 2.0)
    sy = math.sin(yaw / 2.0)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )
