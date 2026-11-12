import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

PACKAGE = Path(__file__).resolve().parents[1]
REPOSITORY = PACKAGE.parent
os.environ["ROS_DOMAIN_ID"] = "90"
os.environ["ROS_LOCALHOST_ONLY"] = "1"
os.environ["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"
os.environ["CYCLONEDDS_URI"] = (
    f"file://{REPOSITORY / 'tools/handeye_calibration/cyclonedds_wsl_local.xml'}"
)

import pytest
import rclpy
from auto_aim_interfaces.msg import GimbalFeedback
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from tf2_msgs.msg import TFMessage

def test_gimbal_feedback_builds_measured_tf_chain(tmp_path):
    package = PACKAGE
    log_path = tmp_path / "gimbal_tf.log"
    env = os.environ.copy()
    env["PYTHONPATH"] = str(package / "scripts") + os.pathsep + env.get(
        "PYTHONPATH", ""
    )
    with log_path.open("w") as log:
        process = subprocess.Popen(
            [
                sys.executable,
                str(package / "scripts/gimbal_tf_broadcaster.py"),
                "--ros-args",
                "--params-file",
                str(package / "config/gimbal_tf.yaml"),
                "-p", "muzzle_frame:=muzzle",
                "-p", "pitch_to_muzzle_x:=0.2",
            ],
            env=env,
            stdin=subprocess.DEVNULL,
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )

    rclpy.init()
    node = rclpy.create_node("gimbal_tf_integration_test")
    dynamic = {}
    static = {}
    node.create_subscription(
        TFMessage,
        "/tf",
        lambda message: dynamic.update(
            {transform.child_frame_id: transform for transform in message.transforms}
        ),
        100,
    )
    node.create_subscription(
        TFMessage,
        "/tf_static",
        lambda message: static.update(
            {transform.child_frame_id: transform for transform in message.transforms}
        ),
        QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        ),
    )
    publisher = node.create_publisher(
        GimbalFeedback, "/serial_bridge/feedback", qos_profile_sensor_data
    )

    def spin_for(seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.02)

    try:
        spin_for(2.0)
        assert process.poll() is None
        assert publisher.get_subscription_count() == 1

        message = GimbalFeedback()
        message.yaw = math.pi / 2.0
        message.pitch = 0.319451511 - 1.023925824751 * math.radians(10.0)
        for _ in range(20):
            message.header.stamp = node.get_clock().now().to_msg()
            publisher.publish(message)
            spin_for(0.03)

        assert set(dynamic) >= {"yaw_frame", "pitch_joint"}
        assert set(static) >= {"camera_link", "camera_optical_frame", "muzzle"}

        yaw = dynamic["yaw_frame"]
        assert yaw.header.frame_id == "odom"
        assert yaw.transform.rotation.z == pytest.approx(math.sqrt(0.5))
        assert yaw.transform.rotation.w == pytest.approx(math.sqrt(0.5))

        pitch = dynamic["pitch_joint"]
        assert pitch.header.frame_id == "yaw_frame"
        assert pitch.transform.translation.x == pytest.approx(0.0)
        assert pitch.transform.translation.y == pytest.approx(0.0)
        assert pitch.transform.translation.z == pytest.approx(0.178)
        assert pitch.transform.rotation.y == pytest.approx(math.sin(math.radians(-5.0)))
        assert pitch.transform.rotation.w == pytest.approx(math.cos(math.radians(5.0)))

        camera = static["camera_link"]
        assert camera.header.frame_id == "pitch_joint"
        assert camera.transform.translation.x == pytest.approx(0.10423)
        assert camera.transform.translation.y == pytest.approx(0.0)
        assert camera.transform.translation.z == pytest.approx(0.0228)

        muzzle = static["muzzle"]
        assert muzzle.header.frame_id == "pitch_joint"
        assert muzzle.transform.translation.x == pytest.approx(0.2)
        assert muzzle.transform.translation.y == pytest.approx(0.0)
        assert muzzle.transform.translation.z == pytest.approx(0.0)

        optical = static["camera_optical_frame"]
        assert optical.header.frame_id == "camera_link"
        assert optical.transform.rotation.x == pytest.approx(-0.5)
        assert optical.transform.rotation.y == pytest.approx(0.5)
        assert optical.transform.rotation.z == pytest.approx(-0.5)
        assert optical.transform.rotation.w == pytest.approx(0.5)
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
