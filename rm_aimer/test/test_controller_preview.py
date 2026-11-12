import math
import os
from pathlib import Path
import time

os.environ['ROS_DOMAIN_ID'] = '194'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
os.environ['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'
os.environ['CYCLONEDDS_URI'] = (
    f"file://{Path(__file__).resolve().parents[2] / 'tools/handeye_calibration/cyclonedds_wsl_local.xml'}"
)

import pytest
import rclpy
from auto_aim_interfaces.msg import AimCommand, GimbalFeedback
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String

from controller_command_preview import ControllerCommandPreview


def test_preview_loads_its_yaml_without_creating_serial_output():
    config = Path(__file__).resolve().parents[1] / 'config/controller_preview.yaml'
    rclpy.init(args=['--ros-args', '--params-file', str(config)])
    node = None
    try:
        node = ControllerCommandPreview()
        assert node.expected_frame == 'pitch_joint'
        assert node.offset == pytest.approx(-0.001259736872589)
        assert node.scale == pytest.approx(1.023925824751)
        assert node.command_pub.topic_name == '/controller_command_preview/command'
        assert node.status == 'waiting_for_data'
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def test_preview_roundtrip_invalid_inputs_and_timeout_without_serial():
    rclpy.init()
    node = ControllerCommandPreview(parameter_overrides=[
        Parameter('pitch_raw_offset_rad', value=0.0),
        Parameter('pitch_raw_scale', value=1.0),
    ])
    probe = rclpy.create_node('controller_preview_test')
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(probe)
    outputs, statuses = [], []
    probe.create_subscription(AimCommand, '/controller_command_preview/command',
                              outputs.append, qos_profile_sensor_data)
    probe.create_subscription(String, '/controller_command_preview/status',
                              lambda m: statuses.append(m.data), 10)
    feedback_pub = probe.create_publisher(GimbalFeedback, '/serial_bridge/feedback',
                                          qos_profile_sensor_data)
    aim_pub = probe.create_publisher(AimCommand, '/aimer/aim_command', qos_profile_sensor_data)

    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=0.005)

    try:
        spin(0.3)
        for _ in range(3):
            stamp = probe.get_clock().now().to_msg()
            feedback = GimbalFeedback(yaw=math.pi - 0.01, pitch=0.0)
            feedback.header.stamp = stamp
            feedback_pub.publish(feedback)
            spin(0.01)
            command = AimCommand(valid=True, yaw=0.1, pitch=0.2)
            command.header.frame_id = 'pitch_joint'
            command.header.stamp = stamp
            aim_pub.publish(command)
            spin(0.03)
        assert 'candidate_unverified' in statuses
        assert outputs[-1].yaw == pytest.approx(math.pi + 0.09)
        assert outputs[-1].pitch == pytest.approx(-0.2)
        assert outputs[-1].header.stamp == stamp
        assert outputs[-1].valid
        assert not probe.get_publishers_info_by_topic('/serial_bridge/aim_command')

        spin(0.25)
        assert statuses[-1] == 'data_timeout'
        assert not outputs[-1].valid
        assert outputs[-1].yaw == 0.0

        command.header.frame_id = 'gimbal_link'
        aim_pub.publish(command)
        spin(0.03)
        assert statuses[-1] == 'input_frame_mismatch'

        command.header.frame_id = 'pitch_joint'
        aim_pub.publish(command)
        spin(0.03)
        assert statuses[-1] == 'command_timestamp_not_current'

        command.valid = False
        aim_pub.publish(command)
        spin(0.03)
        assert statuses[-1] == 'invalid_input_command'
        assert not outputs[-1].valid

        feedback.yaw = math.nan
        feedback_pub.publish(feedback)
        spin(0.03)
        assert statuses[-1] == 'invalid_feedback'
        assert not outputs[-1].valid
        assert not node.samples
    finally:
        executor.shutdown()
        node.destroy_node()
        probe.destroy_node()
        rclpy.shutdown()


def test_relay_uses_message_validity_when_status_topic_is_late(tmp_path, monkeypatch):
    import importlib.util

    script = Path(__file__).resolve().parents[2] / 'tools/dynamic_follow_test.py'
    spec = importlib.util.spec_from_file_location('dynamic_follow_test', script)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    rclpy.init()
    relay = module.DynamicFollowTest(
        30, 0, tmp_path / 'unused.json', max_excursion_deg=30)
    emitted = []
    monkeypatch.setattr(relay.command_pub, 'publish', emitted.append)
    try:
        relay.started_at = time.monotonic()
        relay.feedback = GimbalFeedback(mode=6, yaw=0.3, pitch=0.1)
        relay.last_feedback_at = time.monotonic()
        # The invalid payload arrives before the status topic changes.
        relay.preview_status = 'candidate_unverified'
        relay.preview_callback(AimCommand(valid=False, yaw=0.0, pitch=0.0))
        assert emitted and not emitted[-1].valid
        assert not relay.rows['forwarded']

        # A real, explicitly valid zero-angle target remains legal.
        relay.preview_status = 'invalid_input_command'
        relay.preview_callback(AimCommand(valid=True, yaw=0.0, pitch=0.0))
        assert emitted[-1].valid
        assert len(relay.rows['forwarded']) == 1

        relay.preview_callback(AimCommand(valid=True, yaw=0.001, pitch=0.002))
        assert (emitted[-1].yaw, emitted[-1].pitch) == (0.001, 0.002)
    finally:
        relay.destroy_node()
        rclpy.shutdown()


def test_relay_rejects_cumulative_excursion_from_starting_pose(tmp_path, monkeypatch):
    import importlib.util

    script = Path(__file__).resolve().parents[2] / 'tools/dynamic_follow_test.py'
    spec = importlib.util.spec_from_file_location('dynamic_follow_excursion_test', script)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    rclpy.init()
    relay = module.DynamicFollowTest(
        30, 0, tmp_path / 'unused.json', max_error_deg=20,
        max_excursion_deg=5)
    emitted = []
    monkeypatch.setattr(relay.command_pub, 'publish', emitted.append)
    try:
        relay.started_at = time.monotonic()
        relay.feedback = GimbalFeedback(mode=6, yaw=0.0, pitch=0.0)
        relay.last_feedback_at = time.monotonic()

        relay.preview_callback(AimCommand(
            valid=True, yaw=math.radians(4.0), pitch=0.0))
        assert emitted[-1].valid

        relay.preview_callback(AimCommand(
            valid=True, yaw=math.radians(6.0), pitch=0.0))
        assert not emitted[-1].valid
        assert len(relay.rows['forwarded']) == 1
        assert relay.excursion_rejections == 1
    finally:
        relay.destroy_node()
        rclpy.shutdown()


def test_relay_without_angle_limits_keeps_non_angle_interlocks(tmp_path, monkeypatch):
    import importlib.util

    script = Path(__file__).resolve().parents[2] / 'tools/dynamic_follow_test.py'
    spec = importlib.util.spec_from_file_location('dynamic_follow_no_angle_limits_test', script)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    rclpy.init()
    relay = module.DynamicFollowTest(
        30, 0, tmp_path / 'unused.json', max_error_deg=1,
        max_excursion_deg=1, enforce_angle_limits=False)
    emitted = []
    monkeypatch.setattr(relay.command_pub, 'publish', emitted.append)
    try:
        relay.started_at = time.monotonic()
        relay.preview_callback(AimCommand(valid=True))
        assert emitted and not emitted[-1].valid

        relay.feedback = GimbalFeedback(mode=6, yaw=0.0, pitch=0.0)
        relay.last_feedback_at = time.monotonic()

        relay.preview_callback(AimCommand(
            valid=True, yaw=math.radians(90), pitch=math.radians(30)))
        assert emitted[-1].valid
        assert len(relay.rows['forwarded']) == 1
        assert relay.excursion_rejections == 0

        for command, feedback_age, mode in (
                (AimCommand(valid=False), 0.0, 6),
                (AimCommand(valid=True), 0.2, 6),
                (AimCommand(valid=True), 0.0, 3)):
            relay.last_invalid_at = float('-inf')
            relay.feedback.mode = mode
            relay.last_feedback_at = time.monotonic() - feedback_age
            relay.preview_callback(command)
            assert not emitted[-1].valid
        assert len(relay.rows['forwarded']) == 1
    finally:
        relay.destroy_node()
        rclpy.shutdown()


@pytest.fixture
def deadband_relay(tmp_path, monkeypatch):
    import importlib.util

    script = Path(__file__).resolve().parents[2] / 'tools/dynamic_follow_test.py'
    spec = importlib.util.spec_from_file_location('dynamic_follow_deadband_test', script)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    rclpy.init()
    relay = None
    try:
        relay = module.DynamicFollowTest(
            30, 0, tmp_path / 'unused.json', angle_deadband_deg=0.5)
        relay.started_at = time.monotonic()
        relay.feedback = GimbalFeedback(mode=6, yaw=0.0, pitch=0.0)
        relay.last_feedback_at = time.monotonic()
        emitted = []
        monkeypatch.setattr(relay.command_pub, 'publish', emitted.append)
        yield relay, emitted
    finally:
        if relay is not None:
            relay.destroy_node()
        rclpy.shutdown()


def test_deadband_accumulates_from_held_target_and_updates_axes_independently(deadband_relay):
    relay, emitted = deadband_relay
    samples = [(0.0, 0.0), (0.2, 0.6), (0.4, 0.8), (0.5, 0.9), (0.9, 1.2)]
    expected = [(0.0, 0.0), (0.0, 0.6), (0.0, 0.6), (0.5, 0.6), (0.5, 1.2)]
    for index, ((yaw, pitch), target) in enumerate(zip(samples, expected)):
        msg = AimCommand(valid=True, yaw=math.radians(yaw), pitch=math.radians(pitch))
        msg.header.stamp.sec = index + 1
        relay.preview_callback(msg)
        assert emitted[-1].valid
        assert (emitted[-1].yaw, emitted[-1].pitch) == pytest.approx(
            tuple(math.radians(v) for v in target))
        assert emitted[-1].header.stamp == msg.header.stamp
    assert len(emitted) == len(samples)
    assert len(relay.rows['forwarded']) == len(samples)
    assert relay.rows['preview'][-1][1] == pytest.approx(math.radians(0.9))


def test_deadband_yaw_wrap_uses_shortest_angle(deadband_relay):
    relay, emitted = deadband_relay
    relay.feedback.yaw = math.radians(179.9)
    relay.preview_callback(AimCommand(valid=True, yaw=math.radians(179.9), pitch=0.0))
    relay.preview_callback(AimCommand(valid=True, yaw=math.radians(-179.9), pitch=0.0))
    assert emitted[-1].yaw == pytest.approx(math.radians(179.9))
    relay.preview_callback(AimCommand(valid=True, yaw=math.radians(-179.4), pitch=0.0))
    assert emitted[-1].yaw == pytest.approx(math.radians(-179.4))


@pytest.mark.parametrize('reason', ['invalid', 'preview_timeout', 'feedback_timeout', 'mode'])
def test_deadband_does_not_keep_target_after_invalid_or_timeout(deadband_relay, reason):
    relay, emitted = deadband_relay
    relay.preview_callback(AimCommand(valid=True, yaw=0.0, pitch=0.0))
    if reason == 'preview_timeout':
        relay.last_preview_at = time.monotonic() - 0.2
        relay.tick()
    else:
        if reason == 'feedback_timeout':
            relay.last_feedback_at = time.monotonic() - 0.2
        if reason == 'mode':
            relay.feedback.mode = 3
        relay.preview_callback(AimCommand(
            valid=reason != 'invalid', yaw=math.radians(0.2), pitch=0.0))
    assert not emitted[-1].valid
    assert len(relay.rows['forwarded']) == 1

    relay.feedback.mode = 6
    relay.last_feedback_at = time.monotonic()
    relay.preview_callback(AimCommand(valid=True, yaw=math.radians(0.2), pitch=0.0))
    assert emitted[-1].valid
    assert emitted[-1].yaw == pytest.approx(math.radians(0.2))


def test_deadband_resets_on_preview_gap_even_before_timeout_timer_runs(deadband_relay):
    relay, emitted = deadband_relay
    relay.preview_callback(AimCommand(valid=True, yaw=0.0, pitch=0.0))
    relay.last_preview_at = time.monotonic() - 0.2
    relay.preview_callback(AimCommand(valid=True, yaw=math.radians(0.2), pitch=0.0))
    assert emitted[-1].yaw == pytest.approx(math.radians(0.2))


@pytest.mark.parametrize('axis', ['yaw', 'pitch'])
def test_held_target_cannot_bypass_existing_feedback_angle_limit(deadband_relay, axis):
    relay, emitted = deadband_relay
    relay.preview_callback(AimCommand(valid=True, yaw=0.0, pitch=0.0))
    setattr(relay.feedback, axis, 0.604)
    msg = AimCommand(valid=True, yaw=0.0, pitch=0.0)
    setattr(msg, axis, 0.006)
    relay.preview_callback(msg)
    assert not emitted[-1].valid
    assert len(relay.rows['forwarded']) == 1
