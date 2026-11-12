"""Pure decision policy. No serial, actuator or firmware dependencies."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Config:
    enabled: bool = False
    calibration_verified: bool = False
    expected_frame: str = "controller_joint_targets_preview"
    required_mode: int = 6
    max_age: float = 0.2
    max_pair_skew: float = 0.1
    yaw_tolerance: float = math.radians(1.0)
    pitch_tolerance: float = math.radians(1.0)
    settle_time: float = 0.15
    min_samples: int = 3

    def __post_init__(self):
        values = (self.max_age, self.max_pair_skew, self.yaw_tolerance,
                  self.pitch_tolerance, self.settle_time)
        if not all(math.isfinite(v) and v > 0 for v in values):
            raise ValueError("Timeouts, tolerances and settle_time must be positive and finite")
        if not self.expected_frame or not 0 <= self.required_mode <= 255 or self.min_samples < 2:
            raise ValueError("Invalid frame, mode or sample count")


@dataclass(frozen=True)
class Command:
    stamp: float
    received: float
    yaw: float
    pitch: float
    valid: bool = True
    frame: str = "controller_joint_targets_preview"


@dataclass(frozen=True)
class Feedback:
    stamp: float
    received: float
    yaw: float
    pitch: float
    mode: int = 6


@dataclass(frozen=True)
class Decision:
    allowed: bool
    reason: str
    yaw_error: float | None = None
    pitch_error: float | None = None


class FirePermission:
    def __init__(self, config=Config()):
        self.config = config
        self._reset()
        self.last_tick = None
        self.last_ros = None
        self.highest_command_stamp = 0.0
        self.highest_feedback_stamp = 0.0

    def _reset(self):
        self.since = None
        self.samples = 0
        self.sample_stamp = None

    def deny(self, reason, yaw_error=None, pitch_error=None):
        self._reset()
        return Decision(False, reason, yaw_error, pitch_error)

    def evaluate(self, command, feedback, *, now, ros_now):
        c = self.config
        if not all(math.isfinite(v) for v in (now, ros_now)):
            return self.deny("invalid_clock")
        clock_reset = ((self.last_tick is not None and now < self.last_tick)
                       or (self.last_ros is not None and ros_now < self.last_ros))
        if self.last_tick is not None and now - self.last_tick > c.max_age:
            self._reset()
        self.last_tick, self.last_ros = now, ros_now
        if clock_reset:
            self.highest_command_stamp = self.highest_feedback_stamp = 0.0
            return self.deny("clock_reset")
        if not c.enabled:
            return self.deny("disabled")
        if not c.calibration_verified:
            return self.deny("calibration_unverified")
        if command is None or feedback is None:
            return self.deny("waiting_for_data")
        if not command.valid:
            return self.deny("invalid_command")
        if command.frame != c.expected_frame:
            return self.deny("frame_mismatch")
        for name, item in (("command", command), ("feedback", feedback)):
            if not all(math.isfinite(v) for v in
                       (item.stamp, item.received, item.yaw, item.pitch)):
                return self.deny("nonfinite_" + name)
            if item.stamp <= 0 or not 0 <= ros_now - item.stamp < c.max_age:
                return self.deny("stale_or_future_" + name)
            if not 0 <= now - item.received < c.max_age:
                return self.deny("receive_timeout_" + name)
        if (command.stamp < self.highest_command_stamp
                or feedback.stamp < self.highest_feedback_stamp):
            return self.deny("out_of_order")
        self.highest_command_stamp = command.stamp
        self.highest_feedback_stamp = feedback.stamp
        if feedback.mode != c.required_mode:
            return self.deny("wrong_mode")
        if abs(command.stamp - feedback.stamp) > c.max_pair_skew:
            return self.deny("timestamp_skew")
        # Both inputs are continuous MCU target/feedback angles in radians.
        # Do not wrap yaw: a whole-turn discrepancy must not count as alignment.
        yaw_error = command.yaw - feedback.yaw
        pitch_error = command.pitch - feedback.pitch
        if abs(yaw_error) > c.yaw_tolerance or abs(pitch_error) > c.pitch_tolerance:
            return self.deny("not_aligned", yaw_error, pitch_error)
        if self.since is None:
            self.since = now
        if command.stamp != self.sample_stamp:
            self.sample_stamp = command.stamp
            self.samples += 1
        ready = now - self.since >= c.settle_time and self.samples >= c.min_samples
        return Decision(ready, "ready" if ready else "settling", yaw_error, pitch_error)
