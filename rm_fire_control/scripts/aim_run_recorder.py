#!/usr/bin/env python3
"""Observe the complete aiming pipeline without publishing actuator commands."""
import json
from pathlib import Path
import time

import rclpy
from auto_aim_interfaces.msg import AimCommand, Armors, GimbalFeedback, Target
from rcl_interfaces.msg import ParameterEvent
from rcl_interfaces.srv import GetParameters, ListParameters
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import parameter_value_to_python
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.convert import message_to_ordereddict
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Bool, String
from tf2_msgs.msg import TFMessage

from run_log import RunLog


OBSERVED_TOPICS = (
    ("/detector/armors", Armors),
    ("/tracker/raw_target", Target),
    ("/tracker/target", Target),
    ("/aimer/aim_command", AimCommand),
    ("/aimer/solution", String),
    ("/controller_command_preview/command", AimCommand),
    ("/controller_command_preview/conversion", String),
    ("/controller_command_preview/status", String),
    ("/serial_bridge/feedback", GimbalFeedback),
    ("/serial_bridge/connected", Bool),
    ("/serial_bridge/feedback_alive", Bool),
    ("/serial_bridge/command_active", Bool),
    ("/serial_bridge/tx_event", String),
    ("/fire_control/decision", String),
    ("/camera_info", CameraInfo),
    ("/tf", TFMessage),
    ("/tf_static", TFMessage),
    ("/parameter_events", ParameterEvent),
)
SNAPSHOT_NODES = [
    "/aimer", "/controller_command_preview", "/gimbal_tf", "/serial_bridge",
    "/fire_permission_control", "/armor_tracker", "/target_watchdog",
    "/yolo_armor_detector", "/hik_camera",
]


class AimRunRecorder(Node):
    def __init__(self, parameter_overrides=None):
        super().__init__("aim_run_recorder", parameter_overrides=parameter_overrides)
        root = self.declare_parameter("output_root", str(Path.home() / ".ros/aim_runs")).value
        max_mb = int(self.declare_parameter("max_log_mb", 512).value)
        self.log = RunLog(root, max_bytes=max_mb * 1024 * 1024)
        self.reported_error = None
        sensor_qos = QoSProfile(depth=200, reliability=ReliabilityPolicy.BEST_EFFORT)
        static_qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.log.topic_types = {
            topic: message_type.__module__.split(".")[0] + "/msg/" + message_type.__name__
            for topic, message_type in OBSERVED_TOPICS}
        for topic, message_type in OBSERVED_TOPICS:
            qos = static_qos if topic == "/tf_static" else sensor_qos
            self.create_subscription(
                message_type, topic, lambda msg, topic=topic: self.observe(topic, msg), qos)
        self.parameter_requests = {}
        self.log.expected_nodes = list(self.declare_parameter("snapshot_nodes", SNAPSHOT_NODES).value)
        for name in self.log.expected_nodes:
            self.parameter_requests[name] = dict(
                list_client=self.create_client(ListParameters, name + "/list_parameters"),
                get_client=self.create_client(GetParameters, name + "/get_parameters"),
                future=None, stage="list", started=0.0, names=[])
        self.io_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(1.0, self.tick, clock=self.io_clock)
        self.get_logger().info(f"Run log directory: {self.log.directory}")

    def observe(self, topic, message):
        received_ns = time.monotonic_ns()
        data = message_to_ordereddict(message)
        if isinstance(message, String):
            try:
                data = json.loads(message.data)
            except (ValueError, TypeError):
                pass
        self.log.record(topic, data, self.get_clock().now().nanoseconds, received_ns)

    def poll_parameters(self):
        for name, state in list(self.parameter_requests.items()):
            future = state["future"]
            if future is None:
                client = state[state["stage"] + "_client"]
                if not client.service_is_ready():
                    continue
                request = (ListParameters.Request() if state["stage"] == "list"
                           else GetParameters.Request(names=state["names"]))
                state["future"] = client.call_async(request)
                state["started"] = time.monotonic()
                continue
            if not future.done():
                if time.monotonic() - state["started"] > 5.0:
                    state[state["stage"] + "_client"].remove_pending_request(future)
                    future.cancel()
                    state["future"] = None
                continue
            try:
                result = future.result()
                if state["stage"] == "list":
                    state.update(stage="get", names=list(result.result.names), future=None)
                else:
                    values = {key: parameter_value_to_python(value)
                              for key, value in zip(state["names"], result.values)}
                    self.log.parameters[name] = values
                    self.log.record("/_parameter_snapshot", dict(node=name, parameters=values),
                                    self.get_clock().now().nanoseconds)
                    del self.parameter_requests[name]
            except Exception as exc:
                state.update(stage="list", future=None)
                self.get_logger().warning(f"Parameter snapshot retry for {name}: {exc}")

    def tick(self):
        self.poll_parameters()
        try:
            self.log.checkpoint()
        except OSError as exc:
            self.log.error = f"log_flush_failed: {exc}"
        if self.log.error != self.reported_error:
            self.reported_error = self.log.error
            if self.reported_error:
                self.get_logger().error(f"Run recording stopped: {self.reported_error}")

    def destroy_node(self):
        try:
            self.log.close()
        except OSError as exc:
            self.get_logger().error(f"Could not finish run log: {exc}")
        return super().destroy_node()


def main():
    rclpy.init()
    node = None
    try:
        node = AimRunRecorder()
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
