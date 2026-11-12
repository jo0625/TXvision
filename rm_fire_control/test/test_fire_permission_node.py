"""Synthetic ROS messages only; no serial node or hardware is started."""
import json
import time

import rclpy
from auto_aim_interfaces.msg import AimCommand, GimbalFeedback
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String

from fire_permission_node import FirePermissionNode


def test_ros_preview_lifecycle():
    rclpy.init()
    executor = SingleThreadedExecutor()
    probe = Node("fire_permission_synthetic_probe")
    command_pub = probe.create_publisher(AimCommand, "/fire_test/command", qos_profile_sensor_data)
    feedback_pub = probe.create_publisher(GimbalFeedback, "/fire_test/feedback", qos_profile_sensor_data)
    received = []
    probe.create_subscription(String, "/fire_permission_preview/decision",
                              lambda msg: received.append(json.loads(msg.data)), 100)
    overrides = [Parameter("command_topic", value="/fire_test/command"),
                 Parameter("feedback_topic", value="/fire_test/feedback")]
    node = FirePermissionNode(parameter_overrides=overrides)
    executor.add_node(probe)
    executor.add_node(node)

    def pump(duration, valid=True, mode=6, send=True):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            if send:
                stamp = probe.get_clock().now().to_msg()
                command = AimCommand()
                command.header.stamp = stamp
                command.header.frame_id = "controller_joint_targets_preview"
                command.valid = valid
                command.yaw, command.pitch = 0.1, 0.2
                feedback = GimbalFeedback()
                feedback.header.stamp = stamp
                feedback.yaw, feedback.pitch, feedback.mode = 0.1, 0.2, mode
                feedback_pub.publish(feedback)
                command_pub.publish(command)
            until = time.monotonic() + 0.02
            while time.monotonic() < until:
                executor.spin_once(timeout_sec=0.002)

    try:
        pump(0.5)
        assert received and all(not d["allowed"] for d in received)
        assert received[-1]["reason"] == "disabled"
        executor.remove_node(node)
        node.destroy_node()
        # Verification true here refers ONLY to internally generated test coordinates.
        node = FirePermissionNode(parameter_overrides=overrides + [
            Parameter("enabled", value=True), Parameter("calibration_verified", value=True)])
        executor.add_node(node)
        received.clear()
        pump(0.7)
        assert any(d["reason"] == "settling" for d in received)
        assert received[-1]["allowed"] and received[-1]["preview_only"]
        pump(0.08, valid=False)
        assert not received[-1]["allowed"]
        assert received[-1]["reason"] == "invalid_command"
        pump(0.3)
        assert received[-1]["allowed"]
        pump(0.08, mode=3)
        assert not received[-1]["allowed"] and received[-1]["reason"] == "wrong_mode"
        pump(0.3)
        assert received[-1]["allowed"]
        pump(0.3, send=False)
        assert not received[-1]["allowed"]
        assert "timeout" in received[-1]["reason"] or "stale" in received[-1]["reason"]
        assert node.set_parameters([Parameter("enabled", value=False)])[0].successful is False
    finally:
        executor.remove_node(node)
        executor.remove_node(probe)
        node.destroy_node()
        probe.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
