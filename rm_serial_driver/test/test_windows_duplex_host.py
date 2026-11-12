"""Run the real Windows helper with an in-memory endpoint, without hardware."""

import os
from pathlib import Path
import time

import pytest

from protocol import FeedbackParser, encode_aim_command
from windows_serial import POWERSHELL
from windows_duplex import WindowsSerialDuplex
from windows_duplex_mock import mock_helper_script


pytestmark = pytest.mark.skipif(
    not Path(POWERSHELL).is_file(), reason='Windows interop is required'
)


def read_frames(port, count=1):
    parser = FeedbackParser()
    frames = []
    deadline = time.monotonic() + 3.0
    while len(frames) < count and time.monotonic() < deadline:
        waiting = port.in_waiting
        if waiting:
            frames.extend(parser.feed(port.read(waiting)))
        time.sleep(0.002)
    assert len(frames) >= count
    return frames


def test_actual_windows_pipe_roundtrip(monkeypatch):
    monkeypatch.setattr('windows_duplex.helper_script', mock_helper_script)
    port = WindowsSerialDuplex('COM3', 115200)
    process = port.process
    try:
        assert read_frames(port)[0].sequence == 41
        packet = encode_aim_command(0.6, -0.3)
        assert port.write(packet, timeout=0.5) == 11
        frame = read_frames(port)[0]
        assert frame.sequence == 42
        assert frame.yaw == pytest.approx(0.6)
        assert frame.pitch == pytest.approx(-0.3)
        assert frame.quaternion_is_unit
        # A later command uses a new acknowledgement, not the previous one.
        assert port.write(encode_aim_command(-0.2, 0.1), timeout=0.5) == 11
        assert read_frames(port)[0].sequence == 43
    finally:
        port.close()
    assert process.returncode == 0


def test_actual_windows_timing_frames_preserve_feedback(monkeypatch):
    monkeypatch.setattr('windows_duplex.helper_script', mock_helper_script)
    events = []
    port = WindowsSerialDuplex('COM3', 115200, trace_callback=events.append)
    try:
        assert read_frames(port)[0].sequence == 41
        rx = [event for event in events if event['kind'] == 'rx']
        assert rx
        assert rx[0]['windows_read_end_us'] >= rx[0]['windows_read_start_us'] > 0
        assert FeedbackParser().feed(rx[0]['data'])[0].sequence == 41
    finally:
        port.close()


def test_timing_probe_never_forwards_angle_commands(monkeypatch, tmp_path):
    import importlib.util
    import rclpy
    from auto_aim_interfaces.msg import AimCommand
    import windows_duplex

    repo = Path(__file__).resolve().parents[2]
    spec = importlib.util.spec_from_file_location('serial_probe', repo / 'tools/serial_pipeline_probe.py')
    probe = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(probe)
    monkeypatch.setenv('ROS_DOMAIN_ID', '194')
    monkeypatch.setenv('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')
    monkeypatch.setenv('CYCLONEDDS_URI', f'file://{repo}/tools/handeye_calibration/cyclonedds_wsl_local.xml')
    monkeypatch.setattr('windows_duplex.helper_script', mock_helper_script)
    original_transport = windows_duplex.WindowsSerialDuplex
    trace = probe.Trace()
    rclpy.init(args=['--ros-args', '-p', 'transport:=windows_duplex', '-p', 'port:=COM3'])
    node = None
    try:
        node = probe.make_bridge(trace)
        node.read_serial()
        deadline = time.monotonic() + 2
        while not trace.rows['published'] and time.monotonic() < deadline:
            node.read_serial()
            time.sleep(.002)
        assert trace.rows['published']
        node.command_callback(AimCommand(valid=True, yaw=.5, pitch=.2))
        for _ in range(5):
            node.write_command()
            node.read_serial()
        assert node.port.request_id == 0
        assert node.port.pending_write is None
        assert node.port.inflight_write is None
        assert len(trace.rows['ignored_command']) == 1
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
        windows_duplex.WindowsSerialDuplex = original_transport
        trace.save(tmp_path / 'probe.json', 'test', 0)


@pytest.mark.parametrize('mapping_error_ms', [-600, 600])
def test_idle_clock_mapping_is_refreshed_before_command(monkeypatch, mapping_error_ms):
    monkeypatch.setattr('windows_duplex.helper_script', mock_helper_script)
    port = WindowsSerialDuplex('COM3', 115200)
    try:
        read_frames(port)
        # Model a connection idle for 20 minutes with 500 ppm clock drift.
        port.sync_local_at -= 1200
        port.sync_windows_ms -= 1200000 - mapping_error_ms
        assert port.write(encode_aim_command(0.25, -0.125), timeout=0.1) == 11
        frame = read_frames(port)[0]
        assert frame.sequence == 42
        assert frame.yaw == pytest.approx(0.25)
        assert time.monotonic() - port.sync_local_at < 0.5
    finally:
        port.close()


@pytest.mark.parametrize('clock_offset', [-3600.0, -2.0, 2.0, 3600.0])
def test_windows_100ms_write_ignores_wall_clock_offset(monkeypatch, clock_offset):
    monkeypatch.setattr('windows_duplex.helper_script', mock_helper_script)
    port = WindowsSerialDuplex('COM3', 115200)
    try:
        read_frames(port)
        real_time = time.time
        with monkeypatch.context() as clock:
            clock.setattr('windows_duplex.time.time', lambda: real_time() + clock_offset)
            assert port.write(encode_aim_command(0.25, -0.125), timeout=0.1) == 11
            assert read_frames(port)[0].sequence == 42
        assert port.write(encode_aim_command(0.2, -0.1), timeout=0.1) == 11
        assert read_frames(port)[0].sequence == 43
    finally:
        port.close()


def test_delayed_command_is_rejected_without_mock_device_write(monkeypatch):
    monkeypatch.setattr('windows_duplex.helper_script', mock_helper_script)
    port = WindowsSerialDuplex('COM3', 115200)
    try:
        assert read_frames(port)[0].sequence == 41
        original_send = port._send_line

        def delay_envelope(command, deadline):
            if not command.startswith(b'SYNC '):
                time.sleep(0.15)
                # Deliberately bypass the client deadline to exercise the
                # actual Windows expiry check on an already stale envelope.
                return original_send(command, time.monotonic() + 0.5)
            return original_send(command, deadline)

        with monkeypatch.context() as delayed:
            delayed.setattr(port, '_send_line', delay_envelope)
            with pytest.raises(OSError, match='timed out'):
                port.write(encode_aim_command(1.0, 1.0), timeout=0.1)
        # No mock Write occurred for the expired command: next sequence is 42.
        assert port.write(encode_aim_command(0.25, -0.125), timeout=0.1) == 11
        assert read_frames(port)[0].sequence == 42
    finally:
        port.close()


def test_windows_write_failure_is_not_an_ack(monkeypatch):
    monkeypatch.setattr('windows_duplex.helper_script',
                        lambda port, baud: mock_helper_script(port, baud, fail_write=True))
    port = WindowsSerialDuplex('COM3', 115200)
    try:
        read_frames(port)
        with pytest.raises(OSError):
            port.write(encode_aim_command(0.1, 0.2), timeout=0.5)
    finally:
        port.close()


def test_ros_windows_duplex_mock_feedback_commands_and_reconnect(monkeypatch):
    os.environ['ROS_DOMAIN_ID'] = '192'
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    os.environ['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'
    repo = Path(__file__).resolve().parents[2]
    os.environ['CYCLONEDDS_URI'] = (
        f"file://{repo / 'tools/handeye_calibration/cyclonedds_wsl_local.xml'}"
    )
    import rclpy
    from auto_aim_interfaces.msg import AimCommand, GimbalFeedback
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    from serial_bridge_node import SerialBridgeNode

    monkeypatch.setattr('windows_duplex.helper_script', mock_helper_script)
    rclpy.init(args=['--ros-args', '-p', 'transport:=windows_duplex', '-p', 'port:=COM3',
                     '-p', 'command_timeout:=0.5', '-p', 'command_rate:=10.0'])
    bridge = SerialBridgeNode()
    observer = rclpy.create_node('windows_duplex_mock_observer')
    executor = SingleThreadedExecutor()
    executor.add_node(bridge)
    executor.add_node(observer)
    feedback = []
    observer.create_subscription(GimbalFeedback, '/serial_bridge/feedback',
                                 feedback.append, qos_profile_sensor_data)
    commands = observer.create_publisher(AimCommand, '/serial_bridge/aim_command',
                                        qos_profile_sensor_data)

    def spin(seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.01)

    try:
        spin(2.0)
        assert bridge.connected_state
        assert not bridge.command_active_state
        for _ in range(4):
            commands.publish(AimCommand(valid=True, yaw=0.6, pitch=-0.3))
            spin(0.1)
        assert bridge.command_active_state
        assert any(frame.sequence > 41 and abs(frame.yaw - 0.6) < 1e-5
                   and abs(frame.pitch + 0.3) < 1e-5 for frame in feedback)
        spin(0.8)
        assert not bridge.command_active_state
        count = len(feedback)
        spin(0.2)
        assert len(feedback) == count

        commands.publish(AimCommand(valid=True, yaw=0.1, pitch=0.2))
        spin(0.15)
        commands.publish(AimCommand(valid=False))
        spin(0.2)
        assert not bridge.command_active_state

        # Fail while a command is still fresh, then exercise automatic reopen.
        bridge.command_callback(AimCommand(valid=True, yaw=0.8, pitch=0.1))
        bridge.port.process.stdin.write(b'stop\n')
        bridge.port.process.wait(timeout=5.0)
        bridge.read_serial()
        assert bridge.port is None
        assert not bridge.feedback_alive_state
        bridge.command_callback(AimCommand(valid=True, yaw=1.0, pitch=0.1))
        spin(2.0)
        assert bridge.connected_state
        assert not bridge.command_active_state
        assert bridge.command_gate.packet(time.monotonic(), 0.5) is None
        assert feedback[-1].sequence == 41
        commands.publish(AimCommand(valid=True, yaw=-0.2, pitch=0.1))
        spin(0.3)
        assert any(frame.sequence > 41 and abs(frame.yaw + 0.2) < 1e-5 for frame in feedback)
    finally:
        executor.shutdown()
        bridge.destroy_node()
        observer.destroy_node()
        rclpy.shutdown()
