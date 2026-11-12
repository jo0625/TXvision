import json
import time

import rclpy
from auto_aim_interfaces.msg import AimCommand, GimbalFeedback, Target
from geometry_msgs.msg import TransformStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import Bool, String
from tf2_msgs.msg import TFMessage

from aim_run_recorder import AimRunRecorder


def test_passive_recorder_captures_chain_static_tf_and_effective_parameters(tmp_path):
    rclpy.init()
    probe = rclpy.create_node("recording_test_source")
    probe.declare_parameter("pitch_zero", -0.001259736872589)
    static_pub = probe.create_publisher(
        TFMessage, "/tf_static",
        QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                   durability=DurabilityPolicy.TRANSIENT_LOCAL))
    transform = TransformStamped(child_frame_id="muzzle")
    transform.header.frame_id = "pitch_joint"
    transform.transform.rotation.w = 1.0
    transform.transform.translation.z = 0.2
    static_pub.publish(TFMessage(transforms=[transform]))
    recorder = AimRunRecorder(parameter_overrides=[
        Parameter("output_root", value=str(tmp_path)),
        Parameter("snapshot_nodes", value=["/recording_test_source"]),
    ])
    executor = SingleThreadedExecutor()
    executor.add_node(probe)
    executor.add_node(recorder)
    inputs = [
        ("/tracker/target", Target(tracking=True, id="3")),
        ("/aimer/aim_command", AimCommand(valid=True, yaw=0.1, pitch=0.2)),
        ("/controller_command_preview/command", AimCommand(valid=True, yaw=0.1, pitch=-0.2)),
        ("/serial_bridge/feedback", GimbalFeedback(sequence=7, yaw=0.12, pitch=-0.19, mode=6)),
        ("/fire_control/decision", String(data=json.dumps(
            dict(allowed=False, reason="not_aligned", command_stamp_ns=123)))),
        ("/serial_bridge/command_active", Bool(data=False)),
        ("/serial_bridge/tx_event", String(data=json.dumps(dict(
            event="host_write_returned", event_sequence=1, command_stamp_ns=123)))),
    ]
    publishers = []
    for topic, message in inputs:
        if hasattr(message, "header"):
            message.header.stamp.nanosec = 123
        publishers.append(probe.create_publisher(type(message), topic, qos_profile_sensor_data))
    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=0.005)
    try:
        spin(0.4)
        for publisher, (_, message) in zip(publishers, inputs):
            publisher.publish(message)
        spin(0.3)
        for _ in range(8):
            recorder.poll_parameters()
            spin(0.06)
        recorder.tick()
        directory = recorder.log.directory
        rows = [json.loads(line) for line in (directory / "events.jsonl").read_text().splitlines()]
        by_topic = {row["topic"]: row["message"] for row in rows}
        assert all(topic in by_topic for topic, _ in inputs)
        assert by_topic["/tracker/target"]["header"]["stamp"]["nanosec"] == 123
        assert by_topic["/aimer/aim_command"]["pitch"] == 0.2
        assert by_topic["/serial_bridge/feedback"]["sequence"] == 7
        assert by_topic["/fire_control/decision"]["reason"] == "not_aligned"
        assert by_topic["/serial_bridge/command_active"]["data"] is False
        assert by_topic["/tf_static"]["transforms"][0]["child_frame_id"] == "muzzle"
        summary = json.loads((directory / "summary.json").read_text())
        assert summary["effective_parameters"]["/recording_test_source"]["pitch_zero"] == -0.001259736872589
        published_topics = {
            name for name, _ in probe.get_publisher_names_and_types_by_node("aim_run_recorder", "/")}
        assert published_topics <= {"/rosout", "/parameter_events"}
    finally:
        executor.remove_node(recorder)
        executor.remove_node(probe)
        recorder.destroy_node()
        probe.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
