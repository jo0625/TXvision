#!/usr/bin/env python3
"""Save low-rate JPEG samples from a ROS 2 Image topic.

This tool intentionally avoids cv_bridge so it can run in the same Python
environment as the YOLO detector. It is intended for collecting deployment
background and hard-negative images without recording a large raw rosbag.
"""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


def image_msg_to_bgr(msg: Image) -> np.ndarray:
    encoding = msg.encoding.lower()
    if encoding in {"mono8", "8uc1"}:
        channels = 1
    elif encoding in {"bgr8", "rgb8", "8uc3"}:
        channels = 3
    elif encoding in {"bgra8", "rgba8"}:
        channels = 4
    else:
        raise ValueError(f"unsupported image encoding: {msg.encoding}")

    row_bytes = msg.width * channels
    step = msg.step or row_bytes
    if step < row_bytes:
        raise ValueError(f"image step too small: {step} < {row_bytes}")

    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    required = msg.height * step
    if raw.size < required:
        raise ValueError(f"image buffer too short: {raw.size} < {required}")
    rows = raw[:required].reshape(msg.height, step)[:, :row_bytes]

    if channels == 1:
        return cv2.cvtColor(rows.reshape(msg.height, msg.width), cv2.COLOR_GRAY2BGR)

    image = rows.reshape(msg.height, msg.width, channels)
    if encoding == "rgb8":
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    if encoding == "rgba8":
        return cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)
    if encoding == "bgra8":
        return cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
    return image.copy()


def sensor_qos() -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


class ImageSampler(Node):
    def __init__(
        self,
        topic: str,
        output_dir: Path,
        interval: float,
        limit: int,
        jpeg_quality: int,
        prefix: str,
    ) -> None:
        super().__init__("image_sampler")
        self.output_dir = output_dir
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.interval = interval
        self.limit = limit
        self.jpeg_quality = jpeg_quality
        self.prefix = prefix
        self.saved = 0
        self.last_saved = float("-inf")
        self.finished = False
        self.subscription = self.create_subscription(
            Image, topic, self.image_callback, sensor_qos()
        )
        self.get_logger().info(
            f"Sampling {topic} every {interval:.3f}s into {output_dir}; limit={limit}"
        )

    def image_callback(self, msg: Image) -> None:
        if self.finished:
            return
        now = time.monotonic()
        if now - self.last_saved < self.interval:
            return

        try:
            image = image_msg_to_bgr(msg)
        except (ValueError, cv2.error) as exc:
            self.get_logger().error(f"Cannot decode image: {exc}")
            return

        self.saved += 1
        path = self.output_dir / f"{self.prefix}_{self.saved:06d}.jpg"
        ok = cv2.imwrite(
            str(path), image, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
        )
        if not ok:
            self.saved -= 1
            self.get_logger().error(f"Failed to save {path}")
            return

        self.last_saved = now
        if self.saved == 1 or self.saved % 10 == 0:
            self.get_logger().info(f"Saved {self.saved}/{self.limit}: {path.name}")

        if self.saved >= self.limit:
            self.finished = True
            self.get_logger().info(f"Finished: saved {self.saved} images")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/image_raw")
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--interval", type=float, default=0.5)
    parser.add_argument("--limit", type=int, default=200)
    parser.add_argument("--jpeg-quality", type=int, default=90)
    parser.add_argument("--prefix", default="negative")
    args, ros_args = parser.parse_known_args()

    if args.interval <= 0.0:
        parser.error("--interval must be greater than zero")
    if args.limit <= 0:
        parser.error("--limit must be greater than zero")
    if not 1 <= args.jpeg_quality <= 100:
        parser.error("--jpeg-quality must be in [1, 100]")

    rclpy.init(args=ros_args)
    node = ImageSampler(
        topic=args.topic,
        output_dir=args.output_dir,
        interval=args.interval,
        limit=args.limit,
        jpeg_quality=args.jpeg_quality,
        prefix=args.prefix,
    )
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
