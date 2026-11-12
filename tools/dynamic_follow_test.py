#!/usr/bin/env python3
"""Time-bounded supervised dynamic-follow relay and recorder."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import signal
import time

import numpy as np
import rclpy
from auto_aim_interfaces.msg import AimCommand, Armors, GimbalFeedback, Target
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool, String


def describe(values):
    if not values:
        return None
    data = np.asarray(values, dtype=float)
    return {
        "n": len(values),
        "mean": float(data.mean()),
        "p95": float(np.percentile(data, 95)),
        "max": float(data.max()),
    }


class DynamicFollowTest(Node):
    def __init__(self, duration: float, start_delay: float, output: Path,
                 angle_deadband_deg: float = 0.0,
                 max_error_deg: float = math.degrees(0.6),
                 max_excursion_deg: float = 5.0,
                 max_yaw_error_deg: float | None = None,
                 max_pitch_error_deg: float | None = None,
                 max_yaw_excursion_deg: float | None = None,
                 max_pitch_excursion_deg: float | None = None,
                 enforce_angle_limits: bool = True):
        if not math.isfinite(angle_deadband_deg) or angle_deadband_deg < 0.0:
            raise ValueError("angle_deadband_deg must be finite and nonnegative")
        if not math.isfinite(max_error_deg) or max_error_deg <= 0.0:
            raise ValueError("max_error_deg must be finite and positive")
        if not math.isfinite(max_excursion_deg) or max_excursion_deg <= 0.0:
            raise ValueError("max_excursion_deg must be finite and positive")
        axis_limits = {
            "max_yaw_error_deg": max_yaw_error_deg,
            "max_pitch_error_deg": max_pitch_error_deg,
            "max_yaw_excursion_deg": max_yaw_excursion_deg,
            "max_pitch_excursion_deg": max_pitch_excursion_deg,
        }
        for name, value in axis_limits.items():
            if value is not None and (not math.isfinite(value) or value <= 0.0):
                raise ValueError(f"{name} must be finite and positive")
        super().__init__("dynamic_follow_test")
        self.duration = duration
        self.start_delay = start_delay
        self.output = output
        self.created_at = time.monotonic()
        self.started_at = None
        self.finished = False
        self.last_feedback_at = float("-inf")
        self.last_preview_at = float("-inf")
        self.preview_status = "waiting_for_data"
        self.feedback = None
        self.command_active = False
        self.last_invalid_at = float("-inf")
        self.angle_deadband_deg = angle_deadband_deg
        self.angle_deadband_rad = math.radians(angle_deadband_deg)
        self.enforce_angle_limits = enforce_angle_limits
        self.max_error_deg = max_error_deg
        self.max_error_rad = math.radians(max_error_deg)
        self.max_excursion_deg = max_excursion_deg
        self.max_excursion_rad = math.radians(max_excursion_deg)
        self.max_yaw_error_deg = (
            max_error_deg if max_yaw_error_deg is None else max_yaw_error_deg)
        self.max_pitch_error_deg = (
            max_error_deg if max_pitch_error_deg is None else max_pitch_error_deg)
        self.max_yaw_excursion_deg = (
            max_excursion_deg
            if max_yaw_excursion_deg is None else max_yaw_excursion_deg)
        self.max_pitch_excursion_deg = (
            max_excursion_deg
            if max_pitch_excursion_deg is None else max_pitch_excursion_deg)
        self.max_yaw_error_rad = math.radians(self.max_yaw_error_deg)
        self.max_pitch_error_rad = math.radians(self.max_pitch_error_deg)
        self.max_yaw_excursion_rad = math.radians(self.max_yaw_excursion_deg)
        self.max_pitch_excursion_rad = math.radians(self.max_pitch_excursion_deg)
        self.baseline_angles = None
        self.excursion_rejections = 0
        self.held_angles = None
        self.rows = {name: [] for name in (
            "detector", "tracker", "feedback", "preview", "forwarded", "command_active"
        )}

        self.command_pub = self.create_publisher(
            AimCommand, "/serial_bridge/aim_command", qos_profile_sensor_data)
        self.create_subscription(
            Armors, "/detector/armors", self.detector_callback, qos_profile_sensor_data)
        self.create_subscription(
            Target, "/tracker/target", self.tracker_callback, qos_profile_sensor_data)
        self.create_subscription(
            GimbalFeedback, "/serial_bridge/feedback", self.feedback_callback,
            qos_profile_sensor_data)
        self.create_subscription(
            AimCommand, "/controller_command_preview/command", self.preview_callback,
            qos_profile_sensor_data)
        self.create_subscription(
            String, "/controller_command_preview/status", self.status_callback, 1)
        self.create_subscription(
            Bool, "/serial_bridge/command_active", self.active_callback, 1)
        self.create_timer(0.01, self.tick)

    def relative_time(self):
        if self.started_at is None:
            return None
        return time.monotonic() - self.started_at

    def in_test(self):
        elapsed = self.relative_time()
        return elapsed is not None and 0 <= elapsed < self.duration

    def detector_callback(self, msg):
        if self.in_test():
            self.rows["detector"].append((self.relative_time(), len(msg.armors)))

    def tracker_callback(self, msg):
        if self.in_test():
            self.rows["tracker"].append((self.relative_time(), bool(msg.tracking)))

    def feedback_callback(self, msg):
        now = time.monotonic()
        if msg.mode != 6 or now - self.last_feedback_at > 0.15:
            self.held_angles = None
        self.feedback = msg
        self.last_feedback_at = now
        if self.in_test():
            if (self.baseline_angles is None and msg.mode == 6
                    and math.isfinite(msg.yaw) and math.isfinite(msg.pitch)):
                self.baseline_angles = (msg.yaw, msg.pitch)
            self.rows["feedback"].append((
                self.relative_time(), msg.sequence, msg.mcu_timestamp_ms,
                msg.yaw, msg.pitch, msg.mode))

    def status_callback(self, msg):
        self.preview_status = msg.data

    def active_callback(self, msg):
        self.command_active = bool(msg.data)
        if self.in_test():
            self.rows["command_active"].append((self.relative_time(), self.command_active))

    def publish_invalid(self):
        self.held_angles = None
        now = time.monotonic()
        if now - self.last_invalid_at < 0.02:
            return
        self.last_invalid_at = now
        self.command_pub.publish(AimCommand(valid=False))

    def preview_callback(self, msg):
        now = time.monotonic()
        if now - self.last_preview_at > 0.1:
            self.held_angles = None
        self.last_preview_at = now
        if self.in_test():
            self.rows["preview"].append((
                self.relative_time(), msg.yaw, msg.pitch, self.preview_status, msg.valid))
        feedback = self.feedback
        if (self.baseline_angles is None and self.in_test() and feedback is not None
                and feedback.mode == 6 and math.isfinite(feedback.yaw)
                and math.isfinite(feedback.pitch)):
            self.baseline_angles = (feedback.yaw, feedback.pitch)
        baseline = self.baseline_angles
        basic_allowed = (
            self.in_test()
            # Status is a separate ROS topic and can arrive after this payload.
            and msg.valid
            and feedback is not None
            and feedback.mode == 6
            and now - self.last_feedback_at <= 0.15
            and all(math.isfinite(value) for value in (
                msg.yaw, msg.pitch, feedback.yaw, feedback.pitch))
        )
        angle_allowed = (
            not self.enforce_angle_limits
            or (
                basic_allowed
                and baseline is not None
                and abs(math.remainder(msg.yaw - feedback.yaw, 2 * math.pi))
                <= self.max_yaw_error_rad
                and abs(msg.pitch - feedback.pitch) <= self.max_pitch_error_rad
                and abs(math.remainder(msg.yaw - baseline[0], 2 * math.pi))
                <= self.max_yaw_excursion_rad
                and abs(msg.pitch - baseline[1]) <= self.max_pitch_excursion_rad
                and abs(math.remainder(feedback.yaw - baseline[0], 2 * math.pi))
                <= self.max_yaw_excursion_rad
                and abs(feedback.pitch - baseline[1]) <= self.max_pitch_excursion_rad
            )
        )
        allowed = basic_allowed and angle_allowed
        if not allowed:
            if (self.enforce_angle_limits and baseline is not None
                    and msg.valid and feedback is not None and (
                    abs(math.remainder(msg.yaw - baseline[0], 2 * math.pi))
                    > self.max_yaw_excursion_rad
                    or abs(msg.pitch - baseline[1]) > self.max_pitch_excursion_rad
                    or abs(math.remainder(feedback.yaw - baseline[0], 2 * math.pi))
                    > self.max_yaw_excursion_rad
                    or abs(feedback.pitch - baseline[1])
                    > self.max_pitch_excursion_rad)):
                self.excursion_rejections += 1
            self.publish_invalid()
            return
        yaw, pitch = msg.yaw, msg.pitch
        if self.held_angles is not None:
            held_yaw, held_pitch = self.held_angles
            # Compare against the last forwarded target so slow motion accumulates.
            if abs(math.remainder(yaw - held_yaw, 2 * math.pi)) < self.angle_deadband_rad:
                yaw = held_yaw
            if abs(pitch - held_pitch) < self.angle_deadband_rad:
                pitch = held_pitch
        if (self.enforce_angle_limits
                and (abs(math.remainder(yaw - feedback.yaw, 2 * math.pi))
                     > self.max_yaw_error_rad
                     or abs(pitch - feedback.pitch) > self.max_pitch_error_rad)):
            self.publish_invalid()
            return
        command = AimCommand()
        command.header = msg.header
        command.valid = True
        command.yaw = yaw
        command.pitch = pitch
        # Holding an angle must not stop the fresh-command heartbeat.
        self.command_pub.publish(command)
        self.held_angles = (yaw, pitch)
        self.rows["forwarded"].append((
            self.relative_time(), command.yaw, command.pitch,
            feedback.yaw, feedback.pitch))

    def tick(self):
        now = time.monotonic()
        if self.started_at is None:
            if now - self.created_at >= self.start_delay:
                self.started_at = now
                print(f"TEST_STARTED duration={self.duration:.0f}s "
                      f"angle_deadband_deg={self.angle_deadband_deg:g}", flush=True)
            return
        if now - self.last_preview_at > 0.1:
            self.publish_invalid()
        if now - self.started_at >= self.duration:
            self.finish()

    @staticmethod
    def gaps(rows):
        return [1000 * (b[0] - a[0]) for a, b in zip(rows, rows[1:])]

    @staticmethod
    def longest_false_run(rows, truth_index):
        longest = current = 0
        for row in rows:
            if row[truth_index]:
                current = 0
            else:
                current += 1
                longest = max(longest, current)
        return longest

    def finish(self):
        if self.finished:
            return
        self.finished = True
        for _ in range(3):
            self.command_pub.publish(AimCommand(valid=False))
        feedback = self.rows["feedback"]
        forwarded = self.rows["forwarded"]
        detector = self.rows["detector"]
        tracker = self.rows["tracker"]
        summary = {
            "duration_s": self.duration,
            "detector": {
                "messages": len(detector),
                "fps": len(detector) / self.duration,
                "nonempty": sum(row[1] > 0 for row in detector),
                "interval_ms": describe(self.gaps(detector)),
            },
            "tracker": {
                "messages": len(tracker),
                "fps": len(tracker) / self.duration,
                "tracking": sum(row[1] for row in tracker),
                "longest_nontracking_frames": self.longest_false_run(tracker, 1),
                "interval_ms": describe(self.gaps(tracker)),
            },
            "feedback": {
                "messages": len(feedback),
                "hz": len(feedback) / self.duration,
                "mode6": sum(row[5] == 6 for row in feedback),
                "interval_ms": describe(self.gaps(feedback)),
                "yaw_span_rad": max((r[3] for r in feedback), default=0)
                - min((r[3] for r in feedback), default=0),
                "pitch_span_rad": max((r[4] for r in feedback), default=0)
                - min((r[4] for r in feedback), default=0),
            },
            "commands": {
                "angle_limits_enabled": self.enforce_angle_limits,
                "angle_deadband_deg": self.angle_deadband_deg,
                "max_error_deg": self.max_error_deg,
                "max_excursion_deg": self.max_excursion_deg,
                "max_yaw_error_deg": self.max_yaw_error_deg,
                "max_pitch_error_deg": self.max_pitch_error_deg,
                "max_yaw_excursion_deg": self.max_yaw_excursion_deg,
                "max_pitch_excursion_deg": self.max_pitch_excursion_deg,
                "excursion_rejections": self.excursion_rejections,
                "baseline_rad": self.baseline_angles,
                "forwarded": len(forwarded),
                "active_true_events": sum(row[1] for row in self.rows["command_active"]),
                "yaw_error_rad": describe([
                    abs(math.remainder(r[1] - r[3], 2 * math.pi)) for r in forwarded]),
                "pitch_error_rad": describe([abs(r[2] - r[4]) for r in forwarded]),
            },
        }
        payload = {"summary": summary, "rows": self.rows}
        self.output.parent.mkdir(parents=True, exist_ok=True)
        self.output.write_text(json.dumps(payload, indent=2) + "\n")
        print("TEST_FINISHED", flush=True)
        print(json.dumps(summary, indent=2), flush=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=180.0)
    parser.add_argument("--start-delay", type=float, default=5.0)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--angle-deadband-deg", type=float, default=0.0,
        help="Per-axis target change threshold in degrees; 0 disables holding")
    parser.add_argument(
        "--max-error-deg", type=float, default=math.degrees(0.6),
        help="Maximum command-to-feedback error on either axis")
    parser.add_argument(
        "--max-excursion-deg", type=float, default=5.0,
        help="Maximum command and feedback excursion from the starting pose")
    parser.add_argument(
        "--max-yaw-error-deg", type=float,
        help="Yaw command-to-feedback limit; defaults to --max-error-deg")
    parser.add_argument(
        "--max-pitch-error-deg", type=float,
        help="Pitch command-to-feedback limit; defaults to --max-error-deg")
    parser.add_argument(
        "--max-yaw-excursion-deg", type=float,
        help="Yaw excursion limit; defaults to --max-excursion-deg")
    parser.add_argument(
        "--max-pitch-excursion-deg", type=float,
        help="Pitch excursion limit; defaults to --max-excursion-deg")
    parser.add_argument(
        "--no-angle-limits", dest="enforce_angle_limits", action="store_false",
        help="Let the lower controller enforce mechanical angle limits")
    args = parser.parse_args()
    rclpy.init()
    node = DynamicFollowTest(
        args.duration, args.start_delay, args.output, args.angle_deadband_deg,
        args.max_error_deg, args.max_excursion_deg,
        max_yaw_error_deg=args.max_yaw_error_deg,
        max_pitch_error_deg=args.max_pitch_error_deg,
        max_yaw_excursion_deg=args.max_yaw_excursion_deg,
        max_pitch_excursion_deg=args.max_pitch_excursion_deg,
        enforce_angle_limits=args.enforce_angle_limits)

    def stop(*_):
        node.finish()
        rclpy.shutdown()

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.finish()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
