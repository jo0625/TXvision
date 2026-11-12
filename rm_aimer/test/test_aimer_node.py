import math
import os
import time
from types import SimpleNamespace

import pytest
import rclpy
from auto_aim_interfaces.msg import AimCommand, Target
from geometry_msgs.msg import TransformStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data

from basic_aimer_node import BasicAimerNode
from aimer_geometry import Point3, solve_ballistic_pitch

os.environ.setdefault("ROS_DOMAIN_ID", "90")


def test_node_publishes_geometry_invalid_target_and_timeout():
    rclpy.init()
    aimer = BasicAimerNode(
        parameter_overrides=[
            Parameter("target_timeout", value=0.1),
            Parameter("check_period", value=0.01),
            Parameter("aim_frame", value="gimbal_link"),
        ]
    )
    probe = rclpy.create_node("aimer_test_probe")
    commands = []
    probe.create_subscription(
        AimCommand, "/aimer/aim_command", commands.append, qos_profile_sensor_data
    )
    publisher = probe.create_publisher(Target, "/tracker/target", qos_profile_sensor_data)
    executor = SingleThreadedExecutor()
    executor.add_node(aimer)
    executor.add_node(probe)

    def spin_for(seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.01)

    try:
        spin_for(0.5)
        target = Target()
        target.header.stamp = probe.get_clock().now().to_msg()
        target.header.frame_id = "gimbal_link"
        target.tracking = True
        target.armors_num = 4
        target.position.x = 5.0
        target.position.z = 0.5
        target.radius_1 = 0.2
        target.radius_2 = 0.3
        for _ in range(3):
            publisher.publish(target)
            spin_for(0.03)
        valid = [command for command in commands if command.valid]
        assert valid
        assert valid[-1].yaw == 0.0
        assert valid[-1].pitch == pytest.approx(math.atan2(0.5, 4.8))
        assert valid[-1].header.stamp == target.header.stamp

        spin_for(0.15)
        assert commands[-1].valid is False

        target.tracking = False
        publisher.publish(target)
        spin_for(0.05)
        assert commands[-1].valid is False
    finally:
        executor.remove_node(probe)
        executor.remove_node(aimer)
        probe.destroy_node()
        aimer.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


def test_cross_frame_target_uses_registered_point_transform():
    rclpy.init()
    aimer = BasicAimerNode(parameter_overrides=[Parameter('aim_frame', value='pitch_joint')])
    commands = []
    original_publisher = aimer.publisher
    aimer.publisher = SimpleNamespace(publish=commands.append)
    try:
        transform = TransformStamped()
        transform.header.frame_id = 'pitch_joint'
        transform.child_frame_id = 'odom'
        transform.transform.translation.y = 1.0
        transform.transform.translation.z = 0.5
        transform.transform.rotation.w = 1.0
        aimer.tf_buffer.set_transform_static(transform, 'test')
        target = Target()
        target.header.frame_id = 'odom'
        target.header.stamp = aimer.get_clock().now().to_msg()
        target.tracking = True
        target.armors_num = 4
        target.position.x = 5.0
        target.radius_1 = 0.2
        target.radius_2 = 0.3
        aimer.target_callback(target)
        assert commands[-1].valid
        assert commands[-1].header.stamp == target.header.stamp
        assert commands[-1].yaw == pytest.approx(math.atan2(1.0, 4.8))
        assert commands[-1].pitch == pytest.approx(math.atan2(0.5, math.hypot(4.8, 1.0)))
        target.header.frame_id = 'missing_frame'
        aimer.target_callback(target)
        assert not commands[-1].valid
    finally:
        aimer.publisher = original_publisher
        aimer.destroy_node()
        rclpy.shutdown()


def test_ballistic_mode_publishes_compensated_pitch_and_rejects_unreachable_target():
    rclpy.init()
    aimer = BasicAimerNode(parameter_overrides=[
        Parameter('aim_frame', value='muzzle'),
        Parameter('ballistics_enabled', value=True),
        Parameter('projectile_speed', value=20.0),
        Parameter('muzzle_frame', value='muzzle'),
        Parameter('gravity_frame', value='muzzle'),
    ])
    commands = []
    original_publisher = aimer.publisher
    aimer.publisher = SimpleNamespace(publish=commands.append)
    try:
        target = Target()
        target.header.frame_id = 'muzzle'
        target.tracking = True
        target.armors_num = 4
        target.position.x = 5.0
        target.position.z = 0.5
        target.radius_1 = 0.2
        target.radius_2 = 0.3
        aimer.target_callback(target)
        assert commands[-1].valid
        assert commands[-1].pitch > math.atan2(0.5, 4.8)

        target.velocity.y = 1.0
        aimer.target_callback(target)
        assert commands[-1].valid
        moving_yaw = commands[-1].yaw
        moving_pitch = commands[-1].pitch
        assert moving_yaw > 0.0
        assert moving_pitch > 0.0
        flight_time = math.hypot(4.8, math.tan(moving_yaw) * 4.8) / (
            20.0 * math.cos(moving_pitch)
        )
        assert moving_yaw == pytest.approx(math.atan2(flight_time, 4.8), abs=0.001)

        target.velocity.y = 0.0
        target.v_yaw = 1.0
        aimer.target_callback(target)
        assert commands[-1].valid
        assert commands[-1].yaw < 0.0

        target.position.x = 100.0
        aimer.target_callback(target)
        assert not commands[-1].valid
    finally:
        aimer.publisher = original_publisher
        aimer.destroy_node()
        rclpy.shutdown()


def test_ballistics_uses_offset_muzzle_and_returns_angles_in_pitched_frame():
    rclpy.init()
    aimer = BasicAimerNode(parameter_overrides=[
        Parameter('aim_frame', value='pitch_joint'),
        Parameter('ballistics_enabled', value=True),
        Parameter('projectile_speed', value=20.0),
        Parameter('muzzle_frame', value='muzzle'),
        Parameter('gravity_frame', value='odom'),
    ])
    commands = []
    original_publisher = aimer.publisher
    aimer.publisher = SimpleNamespace(publish=commands.append)
    try:
        pitch = 0.3
        joint = TransformStamped()
        joint.header.frame_id = 'odom'
        joint.child_frame_id = 'pitch_joint'
        joint.transform.rotation.y = -math.sin(pitch / 2.0)
        joint.transform.rotation.w = math.cos(pitch / 2.0)
        aimer.tf_buffer.set_transform_static(joint, 'test')

        muzzle = TransformStamped()
        muzzle.header.frame_id = 'pitch_joint'
        muzzle.child_frame_id = 'muzzle'
        muzzle.transform.translation.x = 0.2
        muzzle.transform.rotation.w = 1.0
        aimer.tf_buffer.set_transform_static(muzzle, 'test')

        target = Target()
        target.header.frame_id = 'odom'
        target.tracking = True
        target.armors_num = 4
        target.position.x = 5.0
        target.position.z = 0.5
        target.radius_1 = 0.2
        target.radius_2 = 0.3
        aimer.target_callback(target)
        assert commands[-1].valid
        world_point = Point3(4.8 - 0.2 * math.cos(pitch), 0.0,
                             0.5 - 0.2 * math.sin(pitch))
        world_pitch, _ = solve_ballistic_pitch(world_point, 20.0)
        assert commands[-1].yaw == pytest.approx(0.0)
        assert commands[-1].pitch == pytest.approx(world_pitch - pitch)
    finally:
        aimer.publisher = original_publisher
        aimer.destroy_node()
        rclpy.shutdown()
