#!/usr/bin/env python3
"""Record a time-bounded MJPEG video from a ROS 2 Image topic."""

from __future__ import annotations

import argparse
import json
import time
from datetime import datetime
from pathlib import Path

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from capture_ros_images import image_msg_to_bgr, sensor_qos


class VideoRecorder(Node):
    def __init__(self, topic: str, output: Path, duration: float, fps: float) -> None:
        super().__init__("real_blank_video_recorder")
        self.output = output
        self.duration = duration
        self.fps = fps
        self.topic = topic
        self.writer: cv2.VideoWriter | None = None
        self.started_at: float | None = None
        self.ended_at: float | None = None
        self.last_frame_at: float | None = None
        self.callbacks_received = 0
        self.frames_written = 0
        self.finished = False
        self.failed = False
        self.subscription = self.create_subscription(
            Image, topic, self.image_callback, sensor_qos()
        )
        self.get_logger().info(
            f"Recording {topic} to {output} for {duration:.1f}s at {fps:.1f} FPS"
        )

    def image_callback(self, msg: Image) -> None:
        if self.finished:
            return

        now = time.monotonic()
        if self.started_at is None:
            self.started_at = now
        elapsed = now - self.started_at
        if elapsed >= self.duration:
            self.ended_at = now
            self.finished = True
            return
        self.callbacks_received += 1

        try:
            image = image_msg_to_bgr(msg)
        except (ValueError, cv2.error) as exc:
            self.get_logger().error(f"Cannot decode image: {exc}")
            self.failed = True
            self.finished = True
            return

        if self.writer is None:
            height, width = image.shape[:2]
            self.writer = cv2.VideoWriter(
                str(self.output),
                cv2.VideoWriter_fourcc(*"MJPG"),
                self.fps,
                (width, height),
            )
            if not self.writer.isOpened():
                self.get_logger().error(f"Cannot open video writer: {self.output}")
                self.failed = True
                self.finished = True
                return

        self.writer.write(image)
        self.frames_written += 1
        self.last_frame_at = now

    def close(self) -> None:
        if self.started_at is not None and self.ended_at is None:
            self.ended_at = time.monotonic()
        if self.writer is not None:
            self.writer.release()
            self.writer = None

    def metrics(self) -> dict:
        actual_duration = (
            self.ended_at - self.started_at
            if self.started_at is not None and self.ended_at is not None
            else 0.0
        )
        callback_fps = self.frames_written / actual_duration if actual_duration > 0 else 0.0
        container_duration = self.frames_written / self.fps
        playback_rate = self.fps / callback_fps if callback_fps > 0 else 0.0
        return {
            "schema_version": 1,
            "recorded_at_utc": datetime.now().astimezone().isoformat(),
            "topic": self.topic,
            "output": str(self.output.resolve()),
            "requested_real_duration_seconds": self.duration,
            "capture_started_monotonic_seconds": self.started_at,
            "capture_ended_monotonic_seconds": self.ended_at,
            "actual_capture_duration_seconds": actual_duration,
            "callbacks_received": self.callbacks_received,
            "valid_frames_written": self.frames_written,
            "actual_callback_fps": callback_fps,
            "container_fps": self.fps,
            "container_playback_duration_seconds": container_duration,
            "playback_rate_relative_to_real_time": playback_rate,
            "playback_accelerated": playback_rate > 1.0,
            "all_valid_callbacks_written": self.callbacks_received == self.frames_written,
        }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/image_raw")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--duration", type=float, required=True)
    parser.add_argument("--fps", type=float, default=20.0)
    args, ros_args = parser.parse_known_args()

    if args.duration <= 0.0:
        parser.error("--duration must be greater than zero")
    if args.fps <= 0.0:
        parser.error("--fps must be greater than zero")
    if args.output.exists():
        parser.error(f"output already exists: {args.output}")
    metadata_output = args.output.with_suffix(args.output.suffix + ".json")
    if metadata_output.exists():
        parser.error(f"metadata output already exists: {metadata_output}")
    args.output.parent.mkdir(parents=True, exist_ok=True)

    rclpy.init(args=ros_args)
    node = VideoRecorder(args.topic, args.output, args.duration, args.fps)
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        node.failed = True
    finally:
        node.close()
        metrics = node.metrics()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    with metadata_output.open("x", encoding="utf-8") as stream:
        json.dump(metrics, stream, indent=2, ensure_ascii=False)
        stream.write("\n")
    print(f"frames_written={node.frames_written}")
    print(f"output={args.output}")
    print(f"metadata={metadata_output}")
    print(json.dumps(metrics, ensure_ascii=False))
    return 1 if node.failed or node.frames_written == 0 else 0


if __name__ == "__main__":
    raise SystemExit(main())
