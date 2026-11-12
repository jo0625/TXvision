"""Check transport selection without opening hardware or starting a helper."""

import os
from pathlib import Path
import time

os.environ['ROS_DOMAIN_ID'] = '193'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
os.environ['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'
os.environ['CYCLONEDDS_URI'] = (
    f"file://{Path(__file__).resolve().parents[2] / 'tools/handeye_calibration/cyclonedds_wsl_local.xml'}"
)

import pytest
import rclpy
from auto_aim_interfaces.msg import AimCommand

import serial_bridge_node


@pytest.mark.parametrize('transport, receive_only', [
    ('windows', False), ('windows', True), ('windows_duplex', True),
])
def test_readonly_modes_never_select_a_writer(monkeypatch, transport, receive_only):
    opened = []

    class Reader:
        def __init__(self, port, baud):
            opened.append((port, baud))

        def close(self):
            pass

        def write(self, *args, **kwargs):
            pytest.fail('Receive-only mode attempted a serial write')

    def forbidden_writer(*args, **kwargs):
        pytest.fail('Receive-only mode constructed the duplex helper')

    monkeypatch.setattr(serial_bridge_node, 'WindowsSerialReader', Reader)
    monkeypatch.setattr('windows_duplex.WindowsSerialDuplex', forbidden_writer)
    rclpy.init(args=['--ros-args', '-p', f'transport:={transport}', '-p', 'port:=COM3',
                     '-p', f'receive_only:={str(receive_only).lower()}'])
    node = None
    try:
        node = serial_bridge_node.SerialBridgeNode()
        node.open_serial(time.monotonic())
        assert opened == [('COM3', 115200)]
        assert node.receive_only
        assert not list(node.subscriptions)
        node.command_callback(AimCommand(valid=True, yaw=0.1, pitch=0.2))
        node.write_command()
        assert not node.command_active_state
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def test_io_timers_continue_when_ros_clock_moves_backwards():
    from collections import Counter
    from rclpy.time import Time

    counts = Counter()

    class TimerProbe(serial_bridge_node.SerialBridgeNode):
        def read_serial(self):
            counts['read'] += 1

        def write_command(self):
            counts['write'] += 1

        def update_health(self):
            counts['health'] += 1

        def republish_states(self):
            counts['state'] += 1

    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true'])
    node = TimerProbe()
    try:
        node.get_clock().set_ros_time_override(Time(seconds=100))
        deadline = time.monotonic() + .1
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.01)
        before = counts.copy()
        node.get_clock().set_ros_time_override(Time(seconds=97.5))
        deadline = time.monotonic() + 1.15
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.01)
        assert counts['read'] - before['read'] >= 10
        assert counts['write'] - before['write'] >= 5
        assert counts['health'] - before['health'] >= 2
        assert counts['state'] - before['state'] >= 1
        assert node.get_clock().now().nanoseconds == 97_500_000_000
    finally:
        node.destroy_node()
        rclpy.shutdown()
