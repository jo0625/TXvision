#!/usr/bin/env python3
"""Persist command and feedback timelines without publishing ROS messages."""

import argparse
import json
from pathlib import Path
import signal
import time

import rclpy
from auto_aim_interfaces.msg import AimCommand, GimbalFeedback
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool


class ScopeRecorder(Node):
    def __init__(self):
        super().__init__("gimbal_scope_recorder", enable_rosout=False,
                         start_parameter_services=False)
        self.started = time.monotonic()
        self.rows = {
            "feedback": [],
            "forwarded": [],
            "preview": [],
            "aimer": [],
            "command_active": [],
        }
        self.create_subscription(
            GimbalFeedback, "/serial_bridge/feedback", self.feedback_callback,
            qos_profile_sensor_data)
        self.create_subscription(
            AimCommand, "/serial_bridge/aim_command",
            lambda message: self.command_callback("forwarded", message),
            qos_profile_sensor_data)
        self.create_subscription(
            AimCommand, "/controller_command_preview/command",
            lambda message: self.command_callback("preview", message),
            qos_profile_sensor_data)
        self.create_subscription(
            AimCommand, "/aimer/aim_command",
            lambda message: self.command_callback("aimer", message),
            qos_profile_sensor_data)
        self.create_subscription(
            Bool, "/serial_bridge/command_active", self.active_callback,
            qos_profile_sensor_data)

    def elapsed(self):
        return time.monotonic() - self.started

    @staticmethod
    def wall_ms():
        return time.time_ns() // 1_000_000

    def feedback_callback(self, message):
        self.rows["feedback"].append([
            self.elapsed(), message.sequence, message.mcu_timestamp_ms,
            message.yaw, message.pitch, message.mode, self.wall_ms()
        ])

    def command_callback(self, name, message):
        if message.valid:
            self.rows[name].append([
                self.elapsed(), message.yaw, message.pitch, self.wall_ms()
            ])

    def active_callback(self, message):
        self.rows["command_active"].append([
            self.elapsed(), bool(message.data), self.wall_ms()
        ])

    def save(self, output):
        payload = {
            "rows": self.rows,
            "summary": {
                "duration_s": self.elapsed(),
                "counts": {name: len(rows) for name, rows in self.rows.items()},
            },
            "scope": {
                "source": "gimbal_scope_recorder",
                "time_base": "host_monotonic_receive",
                "wall_time_indices": {
                    "feedback": 6,
                    "forwarded": 3,
                    "preview": 3,
                    "aimer": 3,
                    "command_active": 2,
                },
                "wall_time_source": "observer_host_receive_unix_ms",
                "publishes_ros_topics": False,
            },
        }
        output.parent.mkdir(parents=True, exist_ok=True)
        temporary = output.with_suffix(output.suffix + ".tmp")
        temporary.write_text(json.dumps(payload, separators=(",", ":")))
        temporary.replace(output)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=900.0)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--autosave", type=float, default=5.0)
    args = parser.parse_args()
    if args.duration <= 0 or args.autosave <= 0:
        parser.error("duration and autosave must be positive")

    running = True

    def stop(*_args):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    rclpy.init()
    node = ScopeRecorder()
    deadline = time.monotonic() + args.duration
    next_save = time.monotonic() + args.autosave
    print(f"SCOPE_RECORDING {args.output} duration={args.duration}s", flush=True)
    try:
        while running and rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if time.monotonic() >= next_save:
                node.save(args.output)
                next_save = time.monotonic() + args.autosave
    finally:
        node.save(args.output)
        node.destroy_node()
        rclpy.shutdown()
    print(f"SCOPE_SAVED {args.output}", flush=True)


if __name__ == "__main__":
    main()
