#!/usr/bin/env python3
"""Publish a finite synthetic-camera stream for the YOLO offline smoke test."""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


class TestImagePublisher(Node):
    def __init__(
        self,
        image_path: Path,
        frame_id: str,
        fx: float,
        fy: float,
        rate: float,
        count: int,
    ) -> None:
        super().__init__("yolo_test_image_publisher")
        image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
        if image is None:
            raise FileNotFoundError(f"cannot read image: {image_path}")
        self.image = image
        self.frame_id = frame_id
        self.fx = fx
        self.fy = fy
        self.remaining = count
        self.done = False
        self.image_pub = self.create_publisher(Image, "/image_raw", qos_profile_sensor_data)
        self.info_pub = self.create_publisher(
            CameraInfo, "/camera_info", qos_profile_sensor_data
        )
        self.timer = self.create_timer(1.0 / rate, self.publish_frame)
        self.get_logger().warning(
            f"Synthetic intrinsics are for offline testing only: "
            f"image={image_path}, fx={fx}, fy={fy}, count={count}"
        )

    def publish_frame(self) -> None:
        if self.remaining <= 0:
            self.timer.cancel()
            self.done = True
            return

        height, width = self.image.shape[:2]
        stamp = self.get_clock().now().to_msg()
        image_msg = Image()
        image_msg.header.stamp = stamp
        image_msg.header.frame_id = self.frame_id
        image_msg.height = height
        image_msg.width = width
        image_msg.encoding = "bgr8"
        image_msg.is_bigendian = 0
        image_msg.step = width * 3
        image_msg.data = self.image.tobytes()

        info_msg = CameraInfo()
        info_msg.header = image_msg.header
        info_msg.width = width
        info_msg.height = height
        cx, cy = width / 2.0, height / 2.0
        info_msg.distortion_model = "plumb_bob"
        info_msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info_msg.k = [self.fx, 0.0, cx, 0.0, self.fy, cy, 0.0, 0.0, 1.0]
        info_msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info_msg.p = [
            self.fx,
            0.0,
            cx,
            0.0,
            0.0,
            self.fy,
            cy,
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
        ]
        self.info_pub.publish(info_msg)
        self.image_pub.publish(image_msg)
        self.remaining -= 1


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Publish one repeated image with synthetic intrinsics. "
            "Do not use the resulting PnP values for real control."
        )
    )
    parser.add_argument("image", type=Path)
    parser.add_argument("--frame-id", default="camera_optical_frame")
    parser.add_argument("--fx", type=float, default=800.0)
    parser.add_argument("--fy", type=float, default=800.0)
    parser.add_argument("--rate", type=float, default=10.0)
    parser.add_argument("--count", type=int, default=30)
    args = parser.parse_args()
    if args.rate <= 0.0 or args.count <= 0:
        parser.error("--rate and --count must be positive")

    rclpy.init()
    node = TestImagePublisher(
        args.image, args.frame_id, args.fx, args.fy, args.rate, args.count
    )
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
