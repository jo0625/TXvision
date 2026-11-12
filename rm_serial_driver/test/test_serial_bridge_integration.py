import os
from pathlib import Path
import pty
import select
import signal
import struct
import subprocess
import sys
import time

PACKAGE = Path(__file__).resolve().parents[1]
REPOSITORY = PACKAGE.parent
os.environ["ROS_DOMAIN_ID"] = "89"
os.environ["ROS_LOCALHOST_ONLY"] = "1"
os.environ["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"
os.environ["CYCLONEDDS_URI"] = (
    f"file://{REPOSITORY / 'tools/handeye_calibration/cyclonedds_wsl_local.xml'}"
)

import pytest
import rclpy
from auto_aim_interfaces.msg import AimCommand, GimbalFeedback
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool

from protocol import crc16
@pytest.mark.parametrize("receive_only", [False, True])
def test_serial_bridge_feedback_command_and_timeouts(tmp_path, receive_only):
    package = PACKAGE
    master, slave = pty.openpty()
    slave_name = os.ttyname(slave)
    os.set_blocking(master, False)
    log_path = tmp_path / "serial_bridge.log"
    with log_path.open("w") as log:
        process = subprocess.Popen(
            [
                sys.executable,
                str(package / "scripts/serial_bridge_node.py"),
                "--ros-args",
                "--params-file",
                str(package / "config/serial_bridge.yaml"),
                "-p",
                f"port:={slave_name}",
                "-p",
                "feedback_timeout:=0.15",
                "-p",
                "command_timeout:=0.10",
                "-p",
                "command_rate:=100.0",
                "-p",
                f"receive_only:={str(receive_only).lower()}",
            ],
            env=os.environ.copy(),
            stdin=subprocess.DEVNULL,
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )

    rclpy.init()
    node = rclpy.create_node("serial_bridge_pty_test")
    feedback = []
    imus = []
    connected = []
    alive = []
    active = []
    node.create_subscription(
        GimbalFeedback,
        "/serial_bridge/feedback",
        feedback.append,
        qos_profile_sensor_data,
    )
    node.create_subscription(Imu, "/serial_bridge/imu", imus.append, qos_profile_sensor_data)
    node.create_subscription(
        Bool, "/serial_bridge/connected", lambda message: connected.append(message.data), 10
    )
    node.create_subscription(
        Bool, "/serial_bridge/feedback_alive", lambda message: alive.append(message.data), 10
    )
    node.create_subscription(
        Bool, "/serial_bridge/command_active", lambda message: active.append(message.data), 10
    )
    publisher = node.create_publisher(
        AimCommand, "/serial_bridge/aim_command", qos_profile_sensor_data
    )

    def spin_for(seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.02)

    def read_master(seconds):
        output = bytearray()
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            ready, _, _ = select.select([master], [], [], 0.01)
            if ready:
                try:
                    output.extend(os.read(master, 4096))
                except BlockingIOError:
                    pass
            rclpy.spin_once(node, timeout_sec=0.0)
        return bytes(output)

    try:
        # Allow DDS discovery and at least two periodic status publications.
        spin_for(3.0)
        assert process.poll() is None
        assert True in connected

        body = b"\xb5" + struct.pack(
            "<II6fB", 42, 123456, 1.0, 0.0, 0.0, 0.0, 0.3, -0.2, 4
        )
        os.write(master, body + struct.pack("<H", crc16(body)) + b"\x5b\x5a")
        spin_for(0.12)
        assert feedback and imus
        assert feedback[-1].sequence == 42
        assert feedback[-1].mcu_timestamp_ms == 123456
        assert abs(feedback[-1].yaw - 0.3) < 1e-5
        assert abs(feedback[-1].pitch + 0.2) < 1e-5
        assert True in alive

        command = AimCommand(valid=True, yaw=0.25, pitch=-0.125)
        for _ in range(8):
            publisher.publish(command)
            spin_for(0.03)
        wire = read_master(0.08)
        starts = [
            index
            for index in range(len(wire) - 10)
            if wire[index] == 0xA5 and wire[index + 9:index + 11] == b"\x5a\x5b"
        ]
        if receive_only:
            assert wire == b""
            assert True not in active
            assert publisher.get_subscription_count() == 0
        else:
            assert starts
            yaw, pitch = struct.unpack("<ff", wire[starts[0] + 1:starts[0] + 9])
            assert abs(yaw - 0.25) < 1e-6
            assert abs(pitch + 0.125) < 1e-6
            assert True in active

        publisher.publish(AimCommand(valid=False))
        spin_for(0.05)
        read_master(0.05)
        assert read_master(0.18) == b""
        assert active[-1] is False
        assert alive[-1] is False
    finally:
        node.destroy_node()
        rclpy.shutdown()
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        os.close(master)
        os.close(slave)
