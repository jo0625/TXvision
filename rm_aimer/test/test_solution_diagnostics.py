import json
import math
import time

import pytest
import rclpy
from auto_aim_interfaces.msg import AimCommand, GimbalFeedback, Target
from rclpy.parameter import Parameter

from basic_aimer_node import BasicAimerNode
from controller_command_preview import ControllerCommandPreview


def test_ballistic_diagnostics_match_output_and_explain_invalid_samples(monkeypatch):
    rclpy.init()
    node = BasicAimerNode(parameter_overrides=[
        Parameter("aim_frame", value="muzzle"),
        Parameter("gravity_frame", value="muzzle"),
        Parameter("muzzle_frame", value="muzzle"),
        Parameter("ballistics_enabled", value=True),
        Parameter("projectile_speed", value=20.0),
        Parameter("publish_diagnostics", value=True),
    ])
    solutions, commands = [], []
    monkeypatch.setattr(node.solution_pub, "publish", lambda msg: solutions.append(json.loads(msg.data)))
    monkeypatch.setattr(node.publisher, "publish", commands.append)
    target = Target(tracking=True, id="3", armors_num=4, radius_1=0.2, radius_2=0.3)
    target.header.frame_id = "muzzle"
    target.header.stamp = node.get_clock().now().to_msg()
    target.position.x = 5.0
    target.position.z = 0.5
    try:
        node.target_callback(target)
        data = solutions[-1]
        assert data["valid"] and commands[-1].valid
        assert data["source_stamp_ns"] == target.header.stamp.sec * 1_000_000_000 + target.header.stamp.nanosec
        assert data["pitch_rad"] == commands[-1].pitch
        assert data["flight_time_s"] > 0
        assert data["line_of_sight_pitch_gravity_rad"] == pytest.approx(math.atan2(0.5, 4.8))
        assert data["gravity_compensation_rad"] == pytest.approx(
            commands[-1].pitch - math.atan2(0.5, 4.8))
        assert data["distance_from_muzzle_m"] == pytest.approx(math.hypot(4.8, 0.5))
        target.tracking = False
        node.target_callback(target)
        assert solutions[-1]["reason"] == "not_tracking"
        assert not solutions[-1]["valid"] and not commands[-1].valid
        node.last_target_at = time.monotonic() - 1.0
        node.check_timeout()
        assert solutions[-1]["reason"] == "target_timeout"
        assert [row["event_sequence"] for row in solutions] == [1, 2, 3]
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_conversion_records_the_feedback_sample_actually_used(monkeypatch):
    rclpy.init()
    node = ControllerCommandPreview(parameter_overrides=[
        Parameter("pitch_raw_offset_rad", value=0.0),
        Parameter("pitch_raw_scale", value=1.0),
        Parameter("publish_diagnostics", value=True),
    ])
    conversions, commands = [], []
    monkeypatch.setattr(node.conversion_pub, "publish", lambda msg: conversions.append(json.loads(msg.data)))
    monkeypatch.setattr(node.command_pub, "publish", commands.append)
    stamp = node.get_clock().now().to_msg()
    feedback = GimbalFeedback(sequence=91, yaw=0.3, pitch=0.0)
    feedback.header.stamp = stamp
    command = AimCommand(valid=True, yaw=0.1, pitch=0.2)
    command.header.stamp = stamp
    command.header.frame_id = "pitch_joint"
    try:
        node.feedback_callback(feedback)
        node.command_callback(command)
        data = conversions[-1]
        assert commands[-1].valid and data["valid"]
        assert data["feedback_sequence"] == 91
        assert data["feedback_stamp_ns"] == stamp.sec * 1_000_000_000 + stamp.nanosec
        assert data["source_stamp_ns"] == data["feedback_stamp_ns"]
        assert data["yaw_rad"] == commands[-1].yaw == pytest.approx(0.4)
        assert data["pitch_rad"] == commands[-1].pitch == pytest.approx(-0.2)
        assert data["input_pitch_rad"] == 0.2
    finally:
        node.destroy_node()
        rclpy.shutdown()
