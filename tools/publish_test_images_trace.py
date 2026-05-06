#!/usr/bin/env python3
import argparse
from pathlib import Path

import cv2
import rclpy
import yaml
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String


class TestImagePublisher(Node):
    def __init__(self, image_dir: Path, camera_info_path: Path, period: float):
        super().__init__("test_image_publisher")
        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(Image, "/image_raw", 10)
        self.info_pub = self.create_publisher(CameraInfo, "/camera_info", 10)
        self.name_pub = self.create_publisher(String, "/test_image_filename", 10)

        self.image_paths = sorted([p for p in image_dir.iterdir() if p.is_file()])
        if not self.image_paths:
            raise RuntimeError(f"No images found in {image_dir}")

        self.images = []
        for path in self.image_paths:
            image = cv2.imread(str(path))
            if image is not None:
                self.images.append((path.name, image))

        if not self.images:
            raise RuntimeError("All images are unreadable")

        data = yaml.safe_load(camera_info_path.read_text())
        self.camera_info = CameraInfo()
        self.camera_info.width = data["image_width"]
        self.camera_info.height = data["image_height"]
        self.camera_info.distortion_model = data["distortion_model"]
        self.camera_info.d = data["distortion_coefficients"]["data"]
        self.camera_info.k = data["camera_matrix"]["data"]
        self.camera_info.r = data["rectification_matrix"]["data"]
        self.camera_info.p = data["projection_matrix"]["data"]

        self.index = 0
        self.timer = self.create_timer(period, self.publish_frame)
        self.get_logger().info(
            f"Loaded {len(self.images)} images from {image_dir}; period={period:.2f}s"
        )

    def publish_frame(self):
        name, image = self.images[self.index]
        stamp = self.get_clock().now().to_msg()
        stamp_text = f"{stamp.sec}.{stamp.nanosec:09d}"

        image_msg = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
        image_msg.header.stamp = stamp
        image_msg.header.frame_id = "camera_optical_frame"

        info_msg = CameraInfo()
        info_msg.header.stamp = stamp
        info_msg.header.frame_id = "camera_optical_frame"
        info_msg.width = self.camera_info.width
        info_msg.height = self.camera_info.height
        info_msg.distortion_model = self.camera_info.distortion_model
        info_msg.d = list(self.camera_info.d)
        info_msg.k = list(self.camera_info.k)
        info_msg.r = list(self.camera_info.r)
        info_msg.p = list(self.camera_info.p)

        name_msg = String()
        name_msg.data = f"{stamp_text} {name}"

        self.image_pub.publish(image_msg)
        self.info_pub.publish(info_msg)
        self.name_pub.publish(name_msg)

        self.get_logger().info(f"Publishing {stamp_text} {name}")
        self.index = (self.index + 1) % len(self.images)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--image-dir",
        type=Path,
        default=Path.home() / "ros_ws" / "test_data" / "images",
    )
    parser.add_argument(
        "--camera-info",
        type=Path,
        default=Path.home() / "ros_ws" / "test_data" / "camera_info" / "camera_info.yaml",
    )
    parser.add_argument("--period", type=float, default=3.0)
    args = parser.parse_args()

    rclpy.init()
    node = TestImagePublisher(args.image_dir, args.camera_info, args.period)
    rclpy.spin(node)


if __name__ == "__main__":
    main()
