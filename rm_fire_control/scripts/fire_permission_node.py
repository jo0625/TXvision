#!/usr/bin/env python3
"""Fire permission publisher; preview by default, never opens a serial port."""
import json
import math
import time

import rclpy
from auto_aim_interfaces.msg import AimCommand, GimbalFeedback
from rcl_interfaces.msg import SetParametersResult
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String

from fire_permission import Command, Config, Feedback, FirePermission


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class FirePermissionNode(Node):
    def __init__(self, parameter_overrides=None):
        super().__init__("fire_permission_preview", parameter_overrides=parameter_overrides)
        def param(name, default):
            return self.declare_parameter(name, default).value
        self.policy = FirePermission(Config(
            enabled=param("enabled", False),
            calibration_verified=param("calibration_verified", False),
            expected_frame=param("expected_frame", "controller_joint_targets_preview"),
            required_mode=param("required_mode", 6),
            max_age=param("max_age", 0.2),
            max_pair_skew=param("max_pair_skew", 0.1),
            yaw_tolerance=math.radians(param("yaw_tolerance_deg", 1.0)),
            pitch_tolerance=math.radians(param("pitch_tolerance_deg", 1.0)),
            settle_time=param("settle_time", 0.15),
            min_samples=param("min_samples", 3),
        ))
        command_topic = param("command_topic", "/controller_command_preview/command")
        feedback_topic = param("feedback_topic", "/serial_bridge/feedback")
        self.preview_only = bool(param("preview_only", True))
        self.command_stamp_ns = None
        self.feedback_stamp_ns = None
        self.command = self.feedback = None
        self.publisher = self.create_publisher(String, param("decision_topic", "~/decision"), 1)
        self.create_subscription(AimCommand, command_topic, self.on_command, qos_profile_sensor_data)
        self.create_subscription(GimbalFeedback, feedback_topic, self.on_feedback, qos_profile_sensor_data)
        # A steady timer still clears permission when /clock pauses.
        self.timer_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(0.02, self.publish_decision, clock=self.timer_clock)
        self.add_on_set_parameters_callback(lambda _: SetParametersResult(
            successful=False, reason="Restart the preview node to apply parameters"))
        self.publish_decision()
        self.get_logger().info("Preview only: no serial output" if self.preview_only else
                               "Control permits enabled: a gated serial bridge can transmit these angles")

    def on_command(self, msg):
        # Evaluate old data first so a gap cannot inherit the previous settling period.
        self.publish_decision()
        self.command_stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        self.command = Command(seconds(msg.header.stamp), time.monotonic(),
                               msg.yaw, msg.pitch, msg.valid, msg.header.frame_id)
        self.publish_decision()

    def on_feedback(self, msg):
        self.publish_decision()
        self.feedback_stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        self.feedback = Feedback(seconds(msg.header.stamp), time.monotonic(),
                                 msg.yaw, msg.pitch, msg.mode)
        self.publish_decision()

    def publish_decision(self):
        stamp_ns = self.get_clock().now().nanoseconds
        ros_now = stamp_ns * 1e-9
        decision = self.policy.evaluate(self.command, self.feedback,
                                        now=time.monotonic(), ros_now=ros_now)
        def finite_or_none(value):
            return value if value is not None and math.isfinite(value) else None
        payload = dict(schema=1, preview_only=self.preview_only,
                       allowed=decision.allowed, reason=decision.reason,
                       stamp_ns=stamp_ns, command_stamp_ns=self.command_stamp_ns,
                       command_frame=self.command.frame if self.command else None,
                       command_yaw=finite_or_none(self.command.yaw if self.command else None),
                       command_pitch=finite_or_none(self.command.pitch if self.command else None),
                       stamp=ros_now, command_stamp=finite_or_none(
                           self.command.stamp if self.command else None),
                       yaw_error_rad=finite_or_none(decision.yaw_error),
                       pitch_error_rad=finite_or_none(decision.pitch_error),
                       feedback_stamp_ns=self.feedback_stamp_ns,
                       feedback_yaw=finite_or_none(self.feedback.yaw if self.feedback else None),
                       feedback_pitch=finite_or_none(self.feedback.pitch if self.feedback else None),
                       feedback_mode=self.feedback.mode if self.feedback else None)
        self.publisher.publish(String(data=json.dumps(payload, allow_nan=False)))


def main():
    rclpy.init()
    node = None
    try:
        node = FirePermissionNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
