#!/usr/bin/env python3
"""Fail-closed guard for tracker targets.

The tracker publishes to a private raw topic. This node republishes fresh
messages to the public target topic and emits ``tracking=false`` if tracker
output stops. It can also reset the tracker after an outage so a long time gap
is not fed into the existing EKF when data resumes.
"""

from __future__ import annotations

import time

import rclpy
from auto_aim_interfaces.msg import Target
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_srvs.srv import Trigger


class TargetWatchdogNode(Node):
    def __init__(self) -> None:
        super().__init__("target_watchdog")
        self.input_topic = str(
            self.declare_parameter("input_topic", "/tracker/raw_target").value
        )
        self.output_topic = str(
            self.declare_parameter("output_topic", "/tracker/target").value
        )
        self.timeout = float(self.declare_parameter("timeout", 0.5).value)
        self.check_period = float(self.declare_parameter("check_period", 0.05).value)
        self.target_frame = str(
            self.declare_parameter("target_frame", "odom").value
        )
        self.reset_tracker_on_timeout = bool(
            self.declare_parameter("reset_tracker_on_timeout", True).value
        )
        self.reset_service = str(
            self.declare_parameter("reset_service", "/tracker/reset").value
        )
        if self.timeout <= 0.0:
            raise ValueError("timeout must be positive")
        if self.check_period <= 0.0:
            raise ValueError("check_period must be positive")

        self.publisher = self.create_publisher(Target, self.output_topic, qos_profile_sensor_data)
        self.subscription = self.create_subscription(
            Target, self.input_topic, self.target_callback, qos_profile_sensor_data
        )
        self.reset_client = (
            self.create_client(Trigger, self.reset_service)
            if self.reset_tracker_on_timeout
            else None
        )
        self.timer = self.create_timer(self.check_period, self.check_timeout)
        self.last_input_time = 0.0
        self.last_frame_id = self.target_frame
        self.have_input = False
        self.timed_out = False

        self.get_logger().info(
            f"Guarding {self.input_topic} -> {self.output_topic}; "
            f"timeout={self.timeout:.3f}s"
        )

    def target_callback(self, message: Target) -> None:
        self.last_input_time = time.monotonic()
        self.have_input = True
        self.timed_out = False
        if message.header.frame_id:
            self.last_frame_id = message.header.frame_id
        self.publisher.publish(message)

    def check_timeout(self) -> None:
        if not self.have_input or self.timed_out:
            return
        elapsed = time.monotonic() - self.last_input_time
        if elapsed <= self.timeout:
            return

        safe_target = Target()
        safe_target.header.stamp = self.get_clock().now().to_msg()
        safe_target.header.frame_id = self.last_frame_id
        safe_target.tracking = False
        self.publisher.publish(safe_target)
        self.timed_out = True
        self.get_logger().error(
            f"Tracker target timed out after {elapsed:.3f}s; published tracking=false"
        )

        if self.reset_client is not None:
            if self.reset_client.service_is_ready():
                self.reset_client.call_async(Trigger.Request())
                self.get_logger().warning("Requested tracker reset after target timeout")
            else:
                self.get_logger().warning(
                    f"Tracker reset service is unavailable: {self.reset_service}"
                )


def main() -> None:
    rclpy.init()
    node = None
    try:
        node = TargetWatchdogNode()
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
