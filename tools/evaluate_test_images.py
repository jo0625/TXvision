#!/usr/bin/env python3
import argparse
import csv
import time
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
import rclpy
import yaml
from auto_aim_interfaces.msg import Armors, DebugArmors, DebugLights
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image


def stamp_text(stamp):
    return f"{stamp.sec}.{stamp.nanosec:09d}"


def same_stamp(left, right):
    return left.sec == right.sec and left.nanosec == right.nanosec


@dataclass
class PendingResult:
    name: str
    stamp: object
    armors: object = None
    lights_count: int = 0
    candidates_count: int = 0
    result_image: object = None


class ImageEvaluator(Node):
    def __init__(
        self,
        image_dir: Path,
        camera_info_path: Path,
        output_dir: Path,
        timeout: float,
        repeats: int,
        repeat_interval: float,
    ):
        super().__init__("test_image_evaluator")
        self.bridge = CvBridge()
        self.output_dir = output_dir
        self.timeout = timeout
        self.repeats = repeats
        self.repeat_interval = repeat_interval
        self.pending = None

        sensor_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        reliable_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.image_pub = self.create_publisher(Image, "/image_raw", sensor_qos)
        self.info_pub = self.create_publisher(CameraInfo, "/camera_info", sensor_qos)
        self.armors_sub = self.create_subscription(
            Armors, "/detector/armors", self.on_armors, sensor_qos
        )
        self.lights_sub = self.create_subscription(
            DebugLights, "/detector/debug_lights", self.on_lights, reliable_qos
        )
        self.candidates_sub = self.create_subscription(
            DebugArmors, "/detector/debug_armors", self.on_candidates, reliable_qos
        )
        self.result_sub = self.create_subscription(
            Image, "/detector/result_img", self.on_result_image, reliable_qos
        )

        self.images = []
        for path in sorted([p for p in image_dir.iterdir() if p.is_file()]):
            image = cv2.imread(str(path))
            if image is not None:
                self.images.append((path.name, image))
        if not self.images:
            raise RuntimeError(f"No readable images found in {image_dir}")

        data = yaml.safe_load(camera_info_path.read_text())
        self.camera_info = CameraInfo()
        self.camera_info.width = data["image_width"]
        self.camera_info.height = data["image_height"]
        self.camera_info.distortion_model = data["distortion_model"]
        self.camera_info.d = data["distortion_coefficients"]["data"]
        self.camera_info.k = data["camera_matrix"]["data"]
        self.camera_info.r = data["rectification_matrix"]["data"]
        self.camera_info.p = data["projection_matrix"]["data"]

    def on_armors(self, msg):
        if self.pending and same_stamp(msg.header.stamp, self.pending.stamp):
            self.pending.armors = msg

    def on_lights(self, msg):
        # DebugLights has no header in this project, so it is best-effort context for the latest image.
        if self.pending:
            self.pending.lights_count = len(msg.data)

    def on_candidates(self, msg):
        # DebugArmors has no header in this project, so it is best-effort context for the latest image.
        if self.pending:
            self.pending.candidates_count = len(msg.data)

    def on_result_image(self, msg):
        if self.pending and same_stamp(msg.header.stamp, self.pending.stamp):
            self.pending.result_image = msg

    def publish_camera_info(self, stamp):
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
        self.info_pub.publish(info_msg)

    def publish_image(self, image, stamp):
        image_msg = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
        image_msg.header.stamp = stamp
        image_msg.header.frame_id = "camera_optical_frame"

        self.publish_camera_info(stamp)
        self.image_pub.publish(image_msg)

    def wait_for_result(self):
        deadline = time.monotonic() + self.timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.pending and (
                self.pending.armors is not None or self.pending.result_image is not None
            ):
                break
        # Give paired callbacks a chance to arrive after the first result.
        settle_deadline = time.monotonic() + 0.5
        while time.monotonic() < settle_deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
        return self.pending

    def save_result_image(self, result):
        if result.result_image is None:
            return ""
        image = self.bridge.imgmsg_to_cv2(result.result_image, desired_encoding="rgb8")
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        path = self.output_dir / "detector_result_images" / f"{Path(result.name).stem}.png"
        path.parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(path), image)
        return str(path)

    def save_original_image(self, name, image, status):
        path = self.output_dir / "original_images" / f"{Path(name).stem}.png"
        path.parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(path), image)

        if status in ("missed", "timeout"):
            missed_path = self.output_dir / "missed_original_images" / f"{Path(name).stem}.png"
            missed_path.parent.mkdir(parents=True, exist_ok=True)
            cv2.imwrite(str(missed_path), image)

        return str(path)

    def save_review_image(self, name, image, result, status):
        review = image.copy()
        height, width = review.shape[:2]
        base = max(height, width)
        thickness = max(4, base // 900)
        font_scale = max(1.0, base / 1800.0)
        text_thickness = max(2, thickness // 2)

        if result.armors is not None and result.armors.armors:
            for armor in result.armors.armors:
                points = [(int(point.x), int(point.y)) for point in armor.kpts]
                if len(points) >= 4:
                    polygon = np.array(points, dtype=np.int32).reshape((-1, 1, 2))
                    cv2.polylines(review, [polygon], True, (0, 255, 0), thickness)
                    for point in points:
                        cv2.circle(review, point, thickness * 2, (0, 255, 255), -1)
                    label_pos = (points[0][0], max(40, points[0][1] - 20))
                else:
                    label_pos = (20, 80)
                label = f"{armor.number} {armor.type}"
                cv2.putText(
                    review,
                    label,
                    label_pos,
                    cv2.FONT_HERSHEY_SIMPLEX,
                    font_scale,
                    (0, 255, 0),
                    text_thickness,
                    cv2.LINE_AA,
                )

        banner_color = (0, 170, 0) if status == "detected" else (0, 0, 255)
        if status == "timeout":
            banner_color = (0, 165, 255)
        banner_height = max(70, int(55 * font_scale))
        cv2.rectangle(review, (0, 0), (width, banner_height), banner_color, -1)
        if result.armors is None:
            armor_count = 0
        else:
            armor_count = len(result.armors.armors)
        title = (
            f"{status.upper()} | armors={armor_count} | lights={result.lights_count} "
            f"| candidates={result.candidates_count} | {name}"
        )
        cv2.putText(
            review,
            title,
            (20, int(banner_height * 0.68)),
            cv2.FONT_HERSHEY_SIMPLEX,
            font_scale,
            (255, 255, 255),
            text_thickness,
            cv2.LINE_AA,
        )

        path = self.output_dir / "review_images" / f"{Path(name).stem}_{status}.png"
        path.parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(path), review)

        if status in ("missed", "timeout"):
            missed_path = (
                self.output_dir / "missed_review_images" / f"{Path(name).stem}_{status}.png"
            )
            missed_path.parent.mkdir(parents=True, exist_ok=True)
            cv2.imwrite(str(missed_path), review)

        return str(path)

    def evaluate(self):
        self.output_dir.mkdir(parents=True, exist_ok=True)
        csv_path = self.output_dir / "detection_report.csv"

        # Let discovery settle, and make sure the detector receives camera info before the first image.
        discovery_deadline = time.monotonic() + 3.0
        while time.monotonic() < discovery_deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.image_pub.get_subscription_count() and self.info_pub.get_subscription_count():
                break

        for _ in range(10):
            self.publish_camera_info(self.get_clock().now().to_msg())
            rclpy.spin_once(self, timeout_sec=0.1)

        rows = []
        for name, image in self.images:
            stamp = self.get_clock().now().to_msg()
            self.pending = PendingResult(name=name, stamp=stamp)
            self.get_logger().info(f"Publishing {stamp_text(stamp)} {name}")
            for _ in range(self.repeats):
                self.publish_image(image, stamp)
                rclpy.spin_once(self, timeout_sec=self.repeat_interval)
            result = self.wait_for_result()

            if result.armors is None:
                status = "timeout"
                armor_count = 0
                numbers = ""
                types = ""
            else:
                armor_count = len(result.armors.armors)
                status = "detected" if armor_count else "missed"
                numbers = ",".join([armor.number for armor in result.armors.armors])
                types = ",".join([armor.type for armor in result.armors.armors])

            result_image = self.save_result_image(result)
            original_image = self.save_original_image(name, image, status)
            review_image = self.save_review_image(name, image, result, status)
            rows.append(
                {
                    "file": name,
                    "stamp": stamp_text(result.stamp),
                    "status": status,
                    "armor_count": armor_count,
                    "numbers": numbers,
                    "types": types,
                    "debug_lights": result.lights_count,
                    "debug_armor_candidates": result.candidates_count,
                    "original_image": original_image,
                    "result_image": result_image,
                    "review_image": review_image,
                }
            )
            self.get_logger().info(
                f"{status.upper():8s} {name} armors={armor_count} "
                f"lights={result.lights_count} candidates={result.candidates_count}"
            )

        with csv_path.open("w", newline="") as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=list(rows[0].keys()))
            writer.writeheader()
            writer.writerows(rows)

        detected = sum(1 for row in rows if row["status"] == "detected")
        missed = sum(1 for row in rows if row["status"] == "missed")
        timeout = sum(1 for row in rows if row["status"] == "timeout")
        self.get_logger().info(
            f"Report: {csv_path} detected={detected} missed={missed} timeout={timeout}"
        )


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
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path.home() / "ros_ws" / "test_results" / "image_eval",
    )
    parser.add_argument("--timeout", type=float, default=3.0)
    parser.add_argument("--repeats", type=int, default=5)
    parser.add_argument("--repeat-interval", type=float, default=0.1)
    args = parser.parse_args()

    rclpy.init()
    node = ImageEvaluator(
        args.image_dir,
        args.camera_info,
        args.output_dir,
        args.timeout,
        args.repeats,
        args.repeat_interval,
    )
    node.evaluate()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
