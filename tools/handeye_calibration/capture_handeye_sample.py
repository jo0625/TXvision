#!/usr/bin/env python3
"""Capture one fixed-board image with a stable paired gimbal pose."""

from __future__ import annotations

import argparse
import json
import math
import re
import time
from collections import deque
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path

import cv2
import numpy as np
import rclpy
from auto_aim_interfaces.msg import GimbalFeedback
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from handeye_math import (
    PATTERN_SIZE,
    board_fully_in_frame,
    detect_chessboard,
    load_camera_calibration,
    solve_board_pose,
    transform_payload,
)


@dataclass(frozen=True)
class FeedbackSample:
    arrival_ns: int
    header_ns: int
    sequence: int
    mcu_timestamp_ms: int
    raw_yaw: float
    raw_pitch: float
    yaw: float
    pitch: float


def image_to_bgr(message: Image) -> np.ndarray:
    if message.encoding != "bgr8" or message.step != message.width * 3:
        raise RuntimeError(
            f"expected tightly packed bgr8, got {message.encoding=} {message.step=}"
        )
    expected = int(message.height * message.step)
    data = np.frombuffer(message.data, dtype=np.uint8)
    if data.size < expected:
        raise RuntimeError(f"short image payload: {data.size} < {expected}")
    return data[:expected].reshape(message.height, message.width, 3).copy()


def stamp_ns(message) -> int:
    return int(message.header.stamp.sec) * 1_000_000_000 + int(
        message.header.stamp.nanosec
    )


def angle_span_and_median(values: list[float]) -> tuple[float, float]:
    unwrapped = np.unwrap(np.asarray(values, dtype=np.float64))
    return float(np.ptp(unwrapped)), float(np.median(unwrapped))


class HandEyeSampleCollector(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("capture_handeye_sample")
        self.args = args
        self.camera_matrix, self.distortion, self.calibration = load_camera_calibration(
            args.camera_calibration
        )
        self.feedback: deque[FeedbackSample] = deque(maxlen=256)
        self.saved = False
        self.result: dict | None = None
        self.frames_seen = 0
        self.complete_boards = 0
        self.unstable_frames = 0
        self.out_of_frame_boards = 0
        self.create_subscription(
            GimbalFeedback,
            args.feedback_topic,
            self.feedback_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image, args.image_topic, self.image_callback, qos_profile_sensor_data
        )

    def feedback_callback(self, message: GimbalFeedback) -> None:
        values = (float(message.yaw), float(message.pitch))
        if not all(math.isfinite(value) for value in values):
            return
        now_ns = time.monotonic_ns()
        self.feedback.append(
            FeedbackSample(
                arrival_ns=now_ns,
                header_ns=stamp_ns(message),
                sequence=int(message.sequence),
                mcu_timestamp_ms=int(message.mcu_timestamp_ms),
                raw_yaw=values[0],
                raw_pitch=values[1],
                yaw=values[0] - self.args.yaw_offset_rad,
                pitch=(values[1] - self.args.pitch_offset_rad)
                / self.args.pitch_scale,
            )
        )

    def stable_feedback(self) -> tuple[list[FeedbackSample], float, float] | None:
        if not self.feedback:
            return None
        now_ns = time.monotonic_ns()
        if (now_ns - self.feedback[-1].arrival_ns) / 1e9 > self.args.feedback_max_age:
            return None
        cutoff = now_ns - int(self.args.stability_window * 1e9)
        samples = [sample for sample in self.feedback if sample.arrival_ns >= cutoff]
        if len(samples) < self.args.minimum_feedback:
            return None
        yaw_span, yaw_median = angle_span_and_median([sample.yaw for sample in samples])
        pitch_span, pitch_median = angle_span_and_median(
            [sample.pitch for sample in samples]
        )
        limit = math.radians(self.args.stability_limit_deg)
        if yaw_span > limit or pitch_span > limit:
            return None
        return samples, yaw_median, pitch_median

    def image_callback(self, message: Image) -> None:
        if self.saved:
            return
        self.frames_seen += 1
        stable = self.stable_feedback()
        if stable is None:
            self.unstable_frames += 1
            return
        samples, yaw_median, pitch_median = stable
        image = image_to_bgr(message)
        if image.shape[:2] != (
            self.calibration["image_height"],
            self.calibration["image_width"],
        ):
            raise RuntimeError(
                f"image shape {image.shape[:2]} does not match calibration "
                f"{self.calibration['image_height']}x{self.calibration['image_width']}"
            )
        corners = detect_chessboard(image)
        if corners is None:
            return
        self.complete_boards += 1
        fully_visible, outer = board_fully_in_frame(
            corners, message.width, message.height, self.args.margin_px
        )
        if not fully_visible:
            self.out_of_frame_boards += 1
            return

        pnp = solve_board_pose(
            corners,
            self.camera_matrix,
            self.distortion,
            self.args.square_size_m,
        )
        image_stamp = stamp_ns(message)
        nearest = min(samples, key=lambda sample: abs(sample.header_ns - image_stamp))
        yaw_span, _ = angle_span_and_median([sample.yaw for sample in samples])
        pitch_span, _ = angle_span_and_median([sample.pitch for sample in samples])

        mask = np.zeros(image.shape[:2], dtype=np.uint8)
        cv2.fillConvexPoly(mask, np.round(outer).astype(np.int32), 255)
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        sharpness = float(cv2.Laplacian(gray, cv2.CV_64F)[mask > 0].var())
        overlay = image.copy()
        cv2.drawChessboardCorners(
            overlay, PATTERN_SIZE, corners.reshape(-1, 1, 2), True
        )
        cv2.polylines(
            overlay,
            [np.round(outer).astype(np.int32)],
            True,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )
        label = (
            f"{self.args.sample_id} yaw={math.degrees(yaw_median):+.2f}deg "
            f"pitch={math.degrees(pitch_median):+.2f}deg "
            f"pnp={pnp['mean_reprojection_error_px']:.3f}px"
        )
        cv2.putText(
            overlay,
            label,
            (15, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            (0, 0, 0),
            5,
            cv2.LINE_AA,
        )
        cv2.putText(
            overlay,
            label,
            (15, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )

        sample_dir = self.args.output_dir / self.args.sample_id
        sample_dir.mkdir(parents=True, exist_ok=False)
        image_path = sample_dir / "image.png"
        overlay_path = sample_dir / "overlay.png"
        if not cv2.imwrite(str(image_path), image, [cv2.IMWRITE_PNG_COMPRESSION, 3]):
            raise RuntimeError(f"failed to save {image_path}")
        if not cv2.imwrite(
            str(overlay_path), overlay, [cv2.IMWRITE_PNG_COMPRESSION, 3]
        ):
            raise RuntimeError(f"failed to save {overlay_path}")

        self.result = {
            "schema_version": 1,
            "sample_id": self.args.sample_id,
            "role": self.args.role,
            "saved_at_utc": datetime.now(timezone.utc).isoformat(),
            "files": {"image": "image.png", "overlay": "overlay.png"},
            "image": {
                "width": int(message.width),
                "height": int(message.height),
                "encoding": message.encoding,
                "frame_id": message.header.frame_id,
                "header_stamp_ns": image_stamp,
            },
            "board": {
                "inner_corners": list(PATTERN_SIZE),
                "square_size_m": self.args.square_size_m,
                "detected_corners_xy": corners.astype(float).tolist(),
                "estimated_outer_board_xy": outer.astype(float).tolist(),
            },
            "camera_calibration": self.calibration,
            "gimbal": {
                "feedback_topic": self.args.feedback_topic,
                "window_sample_count": len(samples),
                "window_duration_s": self.args.stability_window,
                "yaw_span_deg": math.degrees(yaw_span),
                "pitch_span_deg": math.degrees(pitch_span),
                "yaw_rad": yaw_median,
                "yaw_deg": math.degrees(yaw_median),
                "pitch_rad": pitch_median,
                "pitch_deg": math.degrees(pitch_median),
                "yaw_raw_rad": float(np.median([sample.raw_yaw for sample in samples])),
                "pitch_raw_rad": float(
                    np.median([sample.raw_pitch for sample in samples])
                ),
                "yaw_offset_rad": self.args.yaw_offset_rad,
                "pitch_offset_rad": self.args.pitch_offset_rad,
                "pitch_scale": self.args.pitch_scale,
                "nearest_feedback": {
                    "header_stamp_ns": nearest.header_ns,
                    "image_delta_ms": (nearest.header_ns - image_stamp) / 1e6,
                    "sequence": nearest.sequence,
                    "mcu_timestamp_ms": nearest.mcu_timestamp_ms,
                },
            },
            "pnp": {
                "frame_convention": "camera_optical_from_board",
                **transform_payload(pnp["transform"]),
                "rvec": pnp["rvec"].tolist(),
                "tvec_m": pnp["tvec"].tolist(),
                "mean_reprojection_error_px": pnp[
                    "mean_reprojection_error_px"
                ],
                "rms_reprojection_error_px": pnp["rms_reprojection_error_px"],
                "max_reprojection_error_px": pnp["max_reprojection_error_px"],
            },
            "quality": {
                "board_fully_in_frame": True,
                "board_laplacian_variance": sharpness,
                "frames_seen_before_save": self.frames_seen,
                "complete_boards_before_save": self.complete_boards,
            },
        }
        metadata_path = sample_dir / "sample.json"
        metadata_path.write_text(
            json.dumps(self.result, indent=2, ensure_ascii=True) + "\n",
            encoding="utf-8",
        )
        self.saved = True


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--sample-id", required=True)
    parser.add_argument("--role", choices=("calibration", "holdout"), default="calibration")
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--image-topic", default="/image_raw")
    parser.add_argument("--feedback-topic", default="/serial_bridge/feedback")
    parser.add_argument(
        "--camera-calibration",
        type=Path,
        default=Path(__file__).resolve().parents[2]
        / "hik_camera/config/hikrobot_mv_cs016_10uc.yaml",
    )
    parser.add_argument("--square-size-m", type=float, default=0.024)
    parser.add_argument("--timeout", type=float, default=60.0)
    parser.add_argument("--stability-window", type=float, default=0.6)
    parser.add_argument("--stability-limit-deg", type=float, default=0.15)
    parser.add_argument("--minimum-feedback", type=int, default=8)
    parser.add_argument("--feedback-max-age", type=float, default=0.3)
    parser.add_argument("--margin-px", type=float, default=3.0)
    parser.add_argument("--yaw-offset-rad", type=float, default=0.0)
    parser.add_argument("--pitch-offset-rad", type=float, default=0.319451511)
    parser.add_argument("--pitch-scale", type=float, default=1.023925824751)
    args = parser.parse_args()
    if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.-]*", args.sample_id):
        parser.error("--sample-id must contain only letters, numbers, dot, underscore or dash")
    if args.timeout <= 0 or args.stability_window <= 0:
        parser.error("timeout and stability window must be positive")
    if args.minimum_feedback < 2 or args.stability_limit_deg <= 0:
        parser.error("minimum feedback must be >=2 and stability limit must be positive")
    if args.pitch_scale == 0 or not all(
        math.isfinite(value)
        for value in (
            args.square_size_m,
            args.yaw_offset_rad,
            args.pitch_offset_rad,
            args.pitch_scale,
        )
    ):
        parser.error("calibration values must be finite and pitch scale non-zero")
    sample_dir = args.output_dir / args.sample_id
    if sample_dir.exists():
        parser.error(f"sample already exists: {sample_dir}")
    return args


def main() -> int:
    args = parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = HandEyeSampleCollector(args)
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
                    "frames_seen": node.frames_seen,
                    "complete_boards": node.complete_boards,
                    "unstable_frames": node.unstable_frames,
                    "out_of_frame_boards": node.out_of_frame_boards,
                    "feedback_samples": len(node.feedback),
                }
            )
        )
        return 2
    print(json.dumps({"status": "saved", **(node.result or {})}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
