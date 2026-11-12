#!/usr/bin/env python3
"""Publish the measured yaw, pitch and camera kinematic TF chain."""

from __future__ import annotations

import math

import rclpy
from auto_aim_interfaces.msg import GimbalFeedback
from geometry_msgs.msg import TransformStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from gimbal_kinematics import calibrated_pitch, quaternion_from_rpy


class GimbalTfBroadcaster(Node):
    def __init__(self) -> None:
        super().__init__("gimbal_tf")

        self.feedback_topic = str(
            self.declare_parameter("feedback_topic", "/serial_bridge/feedback").value
        )
        self.world_frame = str(self.declare_parameter("world_frame", "odom").value)
        self.yaw_frame = str(self.declare_parameter("yaw_frame", "yaw_frame").value)
        self.pitch_frame = str(
            self.declare_parameter("pitch_frame", "pitch_joint").value
        )
        self.camera_link_frame = str(
            self.declare_parameter("camera_link_frame", "camera_link").value
        )
        self.camera_optical_frame = str(
            self.declare_parameter(
                "camera_optical_frame", "camera_optical_frame"
            ).value
        )
        self.muzzle_frame = str(self.declare_parameter("muzzle_frame", "").value)

        self.yaw_raw_offset = float(
            self.declare_parameter("yaw_raw_offset_rad", 0.0).value
        )
        self.pitch_raw_offset = float(
            self.declare_parameter("pitch_raw_offset_rad", 0.319451511).value
        )
        self.pitch_raw_scale = float(
            self.declare_parameter("pitch_raw_scale", 1.023925824751).value
        )
        self.yaw_to_pitch = self._vector_parameter(
            "yaw_to_pitch", (0.0, 0.0, 0.178)
        )
        self.pitch_to_camera = self._vector_parameter(
            "pitch_to_camera", (0.10423, 0.0, 0.0228)
        )
        self.pitch_to_muzzle = self._vector_parameter(
            "pitch_to_muzzle", (0.0, 0.0, 0.0)
        )
        self.camera_mount_rpy = self._vector_parameter(
            "camera_mount_rpy", (0.0, 0.0, 0.0)
        )

        frames = (
            self.world_frame,
            self.yaw_frame,
            self.pitch_frame,
            self.camera_link_frame,
            self.camera_optical_frame,
        )
        if any(not frame for frame in frames) or len(set(frames)) != len(frames):
            raise ValueError("TF frame names must be non-empty and unique")
        if self.muzzle_frame and self.muzzle_frame in frames:
            raise ValueError("muzzle_frame must be distinct from the existing TF frames")
        if not math.isfinite(self.yaw_raw_offset):
            raise ValueError("yaw_raw_offset_rad must be finite")
        calibrated_pitch(
            self.pitch_raw_offset, self.pitch_raw_offset, self.pitch_raw_scale
        )

        self.dynamic_broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self._publish_static_transforms()
        self.create_subscription(
            GimbalFeedback,
            self.feedback_topic,
            self._feedback_callback,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            f"Publishing {self.world_frame} -> {self.yaw_frame} -> "
            f"{self.pitch_frame} -> {self.camera_link_frame} -> "
            f"{self.camera_optical_frame} from {self.feedback_topic}"
        )

    def _vector_parameter(self, prefix: str, defaults) -> tuple[float, float, float]:
        values = tuple(
            float(self.declare_parameter(f"{prefix}_{axis}", default).value)
            for axis, default in zip(("x", "y", "z"), defaults)
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError(f"{prefix} values must be finite")
        return values

    @staticmethod
    def _make_transform(
        stamp,
        parent: str,
        child: str,
        translation: tuple[float, float, float],
        rotation: tuple[float, float, float, float],
    ) -> TransformStamped:
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = parent
        transform.child_frame_id = child
        transform.transform.translation.x = translation[0]
        transform.transform.translation.y = translation[1]
        transform.transform.translation.z = translation[2]
        transform.transform.rotation.x = rotation[0]
        transform.transform.rotation.y = rotation[1]
        transform.transform.rotation.z = rotation[2]
        transform.transform.rotation.w = rotation[3]
        return transform

    def _publish_static_transforms(self) -> None:
        stamp = self.get_clock().now().to_msg()
        pitch_to_camera = self._make_transform(
            stamp,
            self.pitch_frame,
            self.camera_link_frame,
            self.pitch_to_camera,
            quaternion_from_rpy(*self.camera_mount_rpy),
        )
        camera_to_optical = self._make_transform(
            stamp,
            self.camera_link_frame,
            self.camera_optical_frame,
            (0.0, 0.0, 0.0),
            quaternion_from_rpy(-math.pi / 2.0, 0.0, -math.pi / 2.0),
        )
        static_transforms = [pitch_to_camera, camera_to_optical]
        if self.muzzle_frame:
            static_transforms.append(self._make_transform(
                stamp,
                self.pitch_frame,
                self.muzzle_frame,
                self.pitch_to_muzzle,
                (0.0, 0.0, 0.0, 1.0),
            ))
        self.static_broadcaster.sendTransform(static_transforms)

    def _feedback_callback(self, message: GimbalFeedback) -> None:
        try:
            yaw = float(message.yaw) - self.yaw_raw_offset
            if not math.isfinite(yaw):
                raise ValueError("yaw is not finite")
            pitch = calibrated_pitch(
                float(message.pitch), self.pitch_raw_offset, self.pitch_raw_scale
            )
        except ValueError as exc:
            self.get_logger().warning(f"Ignoring invalid gimbal feedback: {exc}")
            return

        yaw_transform = self._make_transform(
            message.header.stamp,
            self.world_frame,
            self.yaw_frame,
            (0.0, 0.0, 0.0),
            quaternion_from_rpy(0.0, 0.0, yaw),
        )
        pitch_transform = self._make_transform(
            message.header.stamp,
            self.yaw_frame,
            self.pitch_frame,
            self.yaw_to_pitch,
            quaternion_from_rpy(0.0, pitch, 0.0),
        )
        self.dynamic_broadcaster.sendTransform([yaw_transform, pitch_transform])


def main() -> None:
    rclpy.init()
    node = None
    try:
        node = GimbalTfBroadcaster()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except (KeyboardInterrupt, ExternalShutdownException):
            pass


if __name__ == "__main__":
    main()
