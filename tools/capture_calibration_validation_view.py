#!/usr/bin/env python3
"""Save the first complete 9x6 chessboard view from a ROS image topic."""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


PATTERN_SIZE = (9, 6)


def image_to_bgr(message: Image) -> np.ndarray:
    if message.encoding != "bgr8" or message.step != message.width * 3:
        raise RuntimeError(
            f"expected tightly packed bgr8, got encoding={message.encoding} step={message.step}"
        )
    expected = int(message.height * message.step)
    data = np.frombuffer(message.data, dtype=np.uint8)
    if data.size < expected:
        raise RuntimeError(f"short image payload: {data.size} < {expected}")
    return data[:expected].reshape(message.height, message.width, 3).copy()


def estimate_outer_board(corners: np.ndarray) -> np.ndarray:
    grid = corners.reshape(PATTERN_SIZE[1], PATTERN_SIZE[0], 2)
    top_left = grid[0, 0]
    top_right = grid[0, -1]
    bottom_left = grid[-1, 0]
    bottom_right = grid[-1, -1]
    horizontal = ((top_right - top_left) + (bottom_right - bottom_left)) / 16.0
    vertical = ((bottom_left - top_left) + (bottom_right - top_right)) / 10.0
    return np.asarray(
        [
            top_left - horizontal - vertical,
            top_right + horizontal - vertical,
            bottom_right + horizontal + vertical,
            bottom_left - horizontal + vertical,
        ],
        dtype=np.float32,
    )


class FirstCompleteChessboard(Node):
    def __init__(self, topic: str, output: Path) -> None:
        super().__init__("capture_calibration_validation_view")
        self.output = output
        self.saved = False
        self.seen_frames = 0
        self.detection_frames = 0
        self.result: dict | None = None
        self.create_subscription(Image, topic, self.callback, qos_profile_sensor_data)

    def callback(self, message: Image) -> None:
        if self.saved:
            return
        self.seen_frames += 1
        image = image_to_bgr(message)
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCornersSB(
            gray,
            PATTERN_SIZE,
            flags=cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE,
        )
        if not found or corners is None or len(corners) != 54:
            return
        self.detection_frames += 1
        corners = corners.reshape(-1, 2).astype(np.float32)
        outer = estimate_outer_board(corners)
        margin = 3.0
        fully_in_frame = bool(
            np.all(outer[:, 0] >= margin)
            and np.all(outer[:, 0] < message.width - margin)
            and np.all(outer[:, 1] >= margin)
            and np.all(outer[:, 1] < message.height - margin)
        )
        if not fully_in_frame:
            return

        mask = np.zeros_like(gray)
        cv2.fillConvexPoly(mask, np.round(outer).astype(np.int32), 255)
        laplacian = cv2.Laplacian(gray, cv2.CV_64F)
        sharpness = float(laplacian[mask > 0].var())
        if self.output.exists():
            raise RuntimeError(f"refusing to overwrite {self.output}")
        if not cv2.imwrite(str(self.output), image, [cv2.IMWRITE_PNG_COMPRESSION, 3]):
            raise RuntimeError(f"failed to save {self.output}")
        self.result = {
            "output": str(self.output),
            "width": int(message.width),
            "height": int(message.height),
            "encoding": message.encoding,
            "frame_id": message.header.frame_id,
            "timestamp": {
                "sec": int(message.header.stamp.sec),
                "nanosec": int(message.header.stamp.nanosec),
            },
            "seen_frames_before_save": self.seen_frames,
            "complete_detection_frames_before_save": self.detection_frames,
            "detected_inner_corners": int(len(corners)),
            "estimated_outer_board_xy": outer.astype(float).tolist(),
            "estimated_outer_board_fully_in_frame": fully_in_frame,
            "board_laplacian_variance": sharpness,
        }
        self.saved = True


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/image_raw")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--timeout", type=float, default=60.0)
    args = parser.parse_args()
    if args.timeout <= 0:
        parser.error("--timeout must be positive")
    if args.output.exists():
        parser.error(f"output already exists: {args.output}")
    args.output.parent.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    node = FirstCompleteChessboard(args.topic, args.output)
    deadline = time.monotonic() + args.timeout
    try:
        while rclpy.ok() and not node.saved and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    if not node.saved:
        print(
            json.dumps(
                {
                    "status": "timeout",
                    "seen_frames": node.seen_frames,
                    "complete_detection_frames": node.detection_frames,
                }
            )
        )
        return 2
    print(json.dumps({"status": "saved", **(node.result or {})}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
