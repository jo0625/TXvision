#!/usr/bin/env python3
"""YOLO-Pose ROS 2 detector for the V0.1 auto-aim pipeline.

The node intentionally converts ``sensor_msgs/Image`` without ``cv_bridge``.
This lets it run in the existing Ultralytics virtual environment even when
that environment's NumPy ABI differs from the system cv_bridge extension.
"""

from __future__ import annotations

import array
import math
import os
import time
from collections import Counter
from pathlib import Path
from typing import Optional

import cv2
import numpy as np

import rclpy
from auto_aim_interfaces.msg import Armor, Armors
from geometry_msgs.msg import Point
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from raw_track_latch import RawTrackLatchProcessor, solve_normalized_dual_pnp_type
from track_diagnostics import serialize_track_diagnostics


TRACK_LATCH_STRATEGIES = {
    "raw_track_latch": "raw_ratio",
    "normalized_dual_pnp_latch": "normalized_dual_pnp",
}


def image_msg_to_bgr(msg: Image) -> np.ndarray:
    """Convert common sensor_msgs/Image encodings to a BGR OpenCV image."""
    encoding = msg.encoding.lower()
    if encoding in {"mono8", "8uc1"}:
        channels = 1
    elif encoding in {"bgra8", "rgba8"}:
        channels = 4
    elif encoding in {"bgr8", "rgb8", "8uc3"}:
        channels = 3
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
        gray = rows.reshape(msg.height, msg.width)
        return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    image = rows.reshape(msg.height, msg.width, channels)
    if encoding == "rgb8":
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    if encoding == "rgba8":
        return cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)
    if encoding == "bgra8":
        return cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
    return image.copy()


def bgr_to_image_msg(image: np.ndarray, header) -> Image:
    image = np.ascontiguousarray(image)
    output = Image()
    output.header = header
    output.height, output.width = image.shape[:2]
    output.encoding = "bgr8"
    output.is_bigendian = 0
    output.step = int(image.strides[0])
    output.data = array.array("B", image.tobytes())
    return output


def rotation_matrix_to_quaternion(rotation: np.ndarray) -> tuple[float, float, float, float]:
    """Convert a 3x3 rotation matrix to an x,y,z,w quaternion."""
    trace = float(np.trace(rotation))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (rotation[2, 1] - rotation[1, 2]) / scale
        y = (rotation[0, 2] - rotation[2, 0]) / scale
        z = (rotation[1, 0] - rotation[0, 1]) / scale
    elif rotation[0, 0] > rotation[1, 1] and rotation[0, 0] > rotation[2, 2]:
        scale = math.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
        w = (rotation[2, 1] - rotation[1, 2]) / scale
        x = 0.25 * scale
        y = (rotation[0, 1] + rotation[1, 0]) / scale
        z = (rotation[0, 2] + rotation[2, 0]) / scale
    elif rotation[1, 1] > rotation[2, 2]:
        scale = math.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
        w = (rotation[0, 2] - rotation[2, 0]) / scale
        x = (rotation[0, 1] + rotation[1, 0]) / scale
        y = 0.25 * scale
        z = (rotation[1, 2] + rotation[2, 1]) / scale
    else:
        scale = math.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
        w = (rotation[1, 0] - rotation[0, 1]) / scale
        x = (rotation[0, 2] + rotation[2, 0]) / scale
        y = (rotation[1, 2] + rotation[2, 1]) / scale
        z = 0.25 * scale
    quaternion = np.asarray([x, y, z, w], dtype=np.float64)
    norm = float(np.linalg.norm(quaternion))
    if norm > 0.0:
        quaternion /= norm
    return tuple(float(value) for value in quaternion)


def sensor_qos(depth: int) -> QoSProfile:
    """Best-effort keep-last QoS suited to low-latency camera streams."""
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=max(1, depth),
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


def resolve_model_path(model_path: str) -> Path:
    """Resolve a portable environment-backed model reference."""
    expanded = os.path.expandvars(model_path)
    if expanded.startswith("$"):
        raise ValueError(f"model environment variable is not set: {model_path}")
    return Path(expanded).expanduser()


class YoloArmorNode(Node):
    def __init__(self) -> None:
        super().__init__("yolo_armor_detector")
        default_model = (
            Path.home()
            / "my-project/my_first_autoaim_yolo/runs/pose/armor_pose_v2_with04/weights/best.pt"
        )

        self.model_path = str(self.declare_parameter("model", str(default_model)).value)
        self.image_topic = str(self.declare_parameter("image_topic", "/image_raw").value)
        self.camera_info_topic = str(
            self.declare_parameter("camera_info_topic", "/camera_info").value
        )
        self.debug_topic = str(
            self.declare_parameter("debug_topic", "/detector/result_img").value
        )
        self.armors_topic = str(
            self.declare_parameter("armors_topic", "/detector/armors").value
        )
        self.imgsz = int(self.declare_parameter("imgsz", 640).value)
        self.conf = float(self.declare_parameter("conf", 0.55).value)
        self.nms_iou = float(self.declare_parameter("nms_iou", 0.7).value)
        self.max_det = int(self.declare_parameter("max_det", 300).value)
        self.agnostic_nms = bool(self.declare_parameter("agnostic_nms", False).value)
        self.device = str(self.declare_parameter("device", "").value)
        self.warmup = bool(self.declare_parameter("warmup", True).value)
        self.publish_debug = bool(self.declare_parameter("publish_debug", True).value)
        self.publish_track_diagnostics = bool(
            self.declare_parameter("publish_track_diagnostics", False).value
        )
        self.track_diagnostics_topic = str(
            self.declare_parameter(
                "track_diagnostics_topic", "/detector/track_diagnostics"
            ).value
        )
        self.image_qos_depth = int(self.declare_parameter("image_qos_depth", 1).value)
        self.min_quad_area = float(self.declare_parameter("min_quad_area", 25.0).value)
        self.min_light_length = float(
            self.declare_parameter("min_light_length", 4.0).value
        )
        self.min_keypoint_conf = float(
            self.declare_parameter("min_keypoint_conf", 0.15).value
        )
        self.armor_type = str(self.declare_parameter("armor_type", "small").value)
        self.type_strategy = str(
            self.declare_parameter("type_strategy", "fixed").value
        )
        self.armor_number = str(self.declare_parameter("armor_number", "armor").value)
        self.detect_color = int(self.declare_parameter("detect_color", -1).value)
        self.color_margin = float(self.declare_parameter("color_margin", 8.0).value)
        self.max_reprojection_error = float(
            self.declare_parameter("max_reprojection_error", 12.0).value
        )
        self.min_pnp_distance = float(
            self.declare_parameter("min_pnp_distance", 0.1).value
        )
        self.max_pnp_distance = float(
            self.declare_parameter("max_pnp_distance", 10.0).value
        )
        self.diagnostics_period = int(
            self.declare_parameter("diagnostics_period", 30).value
        )
        self.gray_threshold = self.declare_parameter(
            "gray_threshold", 160
        ).value

        if self.armor_type not in {"small", "large"}:
            raise ValueError("armor_type must be 'small' or 'large'")
        if self.type_strategy not in {"fixed", *TRACK_LATCH_STRATEGIES}:
            raise ValueError(
                "type_strategy must be 'fixed', 'raw_track_latch', or "
                "'normalized_dual_pnp_latch'"
            )
        if self.imgsz <= 0 or not 0.0 < self.conf <= 1.0:
            raise ValueError("imgsz must be positive and conf must be in (0, 1]")
        if not 0.0 < self.nms_iou <= 1.0 or self.max_det <= 0:
            raise ValueError("nms_iou must be in (0, 1] and max_det must be positive")

        model_file = resolve_model_path(self.model_path)
        if not model_file.is_file():
            raise FileNotFoundError(f"YOLO model does not exist: {model_file}")
        try:
            from ultralytics import YOLO
        except ImportError as exc:
            raise RuntimeError(
                "ultralytics is unavailable; activate the project's .venv before launching"
            ) from exc

        self.get_logger().info(f"Loading YOLO model: {model_file}")
        self.model = YOLO(str(model_file))
        self.get_logger().info(
            f"YOLO ready: imgsz={self.imgsz}, conf={self.conf}, "
            f"nms_iou={self.nms_iou}, max_det={self.max_det}, "
            f"device={self.device or 'auto'}, image_qos_depth={max(1, self.image_qos_depth)}"
        )
        if self.warmup:
            warmup_kwargs = {
                "source": np.zeros((384, 640, 3), dtype=np.uint8),
                "imgsz": self.imgsz,
                "conf": self.conf,
                "iou": self.nms_iou,
                "max_det": self.max_det,
                "agnostic_nms": self.agnostic_nms,
                "verbose": False,
            }
            if self.device:
                warmup_kwargs["device"] = self.device
            try:
                self.model.predict(**warmup_kwargs)
                self.get_logger().info("YOLO warmup complete")
            except Exception as exc:  # noqa: BLE001 - inference backend errors vary
                self.get_logger().warning(f"YOLO warmup failed; continuing: {exc}")

        self.camera_matrix: Optional[np.ndarray] = None
        self.dist_coeffs: Optional[np.ndarray] = None
        self.camera_width = 0
        self.camera_height = 0
        self.camera_frame_id = ""
        self.frame_count = 0
        self.total_inference_ms = 0.0
        self.last_camera_warning = 0.0
        self.last_image_mismatch_warning = 0.0
        self.last_ownership_overload_warning = 0.0
        self.last_track_diagnostics_warning = 0.0
        self.raw_track_latch = (
            RawTrackLatchProcessor(
                TRACK_LATCH_STRATEGIES[self.type_strategy],
                gray_threshold=self.gray_threshold,
            )
            if self.type_strategy in TRACK_LATCH_STRATEGIES
            else None
        )
        self.get_logger().info(
            f"Traditional light extraction gray threshold: {self.gray_threshold}"
        )

        self.armors_pub = self.create_publisher(Armors, self.armors_topic, sensor_qos(1))
        self.debug_pub = (
            self.create_publisher(Image, self.debug_topic, sensor_qos(1))
            if self.publish_debug
            else None
        )
        self.track_diagnostics_pub = (
            self.create_publisher(
                String, self.track_diagnostics_topic, sensor_qos(10)
            )
            if self.publish_track_diagnostics
            else None
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            sensor_qos(1),
        )
        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            sensor_qos(self.image_qos_depth),
        )

    def camera_info_callback(self, msg: CameraInfo) -> None:
        matrix = np.asarray(msg.k, dtype=np.float64).reshape(3, 3)
        if not np.isfinite(matrix).all() or matrix[0, 0] <= 0.0 or matrix[1, 1] <= 0.0:
            self.get_logger().warning("Ignoring CameraInfo with invalid focal length/K")
            return
        distortion = np.asarray(msg.d, dtype=np.float64).reshape(-1, 1)
        if distortion.size == 0:
            distortion = np.zeros((5, 1), dtype=np.float64)
        changed = (
            self.camera_matrix is None
            or msg.width != self.camera_width
            or msg.height != self.camera_height
            or msg.header.frame_id != self.camera_frame_id
            or not np.allclose(matrix, self.camera_matrix)
        )
        self.camera_matrix = matrix
        self.dist_coeffs = distortion
        self.camera_width = int(msg.width)
        self.camera_height = int(msg.height)
        self.camera_frame_id = msg.header.frame_id
        if changed:
            self.get_logger().info(
                "CameraInfo: "
                f"{msg.width}x{msg.height}, frame_id='{msg.header.frame_id}', "
                f"fx={matrix[0, 0]:.3f}, fy={matrix[1, 1]:.3f}, "
                f"cx={matrix[0, 2]:.3f}, cy={matrix[1, 2]:.3f}, D={msg.d}"
            )

    def color_is_valid(self, image: np.ndarray, points: np.ndarray) -> bool:
        """Optionally check red/blue intensity along the two light bars."""
        if self.detect_color not in (0, 1):
            return True
        samples = []
        for start, end in ((points[0], points[1]), (points[3], points[2])):
            for ratio in np.linspace(0.15, 0.85, 8):
                x, y = np.round(start * (1.0 - ratio) + end * ratio).astype(int)
                radius = 2
                x0, x1 = max(0, x - radius), min(image.shape[1], x + radius + 1)
                y0, y1 = max(0, y - radius), min(image.shape[0], y + radius + 1)
                if x0 < x1 and y0 < y1:
                    samples.append(image[y0:y1, x0:x1].reshape(-1, 3))
        if not samples:
            return False
        pixels = np.concatenate(samples, axis=0).astype(np.float32)
        blue_minus_red = float(np.mean(pixels[:, 0] - pixels[:, 2]))
        score = blue_minus_red if self.detect_color == 1 else -blue_minus_red
        return score >= self.color_margin

    def valid_keypoints(self, image: np.ndarray, points: np.ndarray) -> bool:
        if points.shape != (4, 2) or not np.isfinite(points).all():
            return False
        area = abs(float(cv2.contourArea(points.astype(np.float32))))
        if area < self.min_quad_area:
            return False
        left_length = float(np.linalg.norm(points[1] - points[0]))
        right_length = float(np.linalg.norm(points[2] - points[3]))
        if min(left_length, right_length) < self.min_light_length:
            return False
        length_ratio = max(left_length, right_length) / max(
            min(left_length, right_length), 1e-6
        )
        if length_ratio > 2.5:
            return False
        return self.color_is_valid(image, points)

    def solve_armor(self, points: np.ndarray) -> Optional[Armor]:
        """Solve one fixed-size armor using rm_vision's point convention."""
        if self.camera_matrix is None or self.dist_coeffs is None:
            return None

        # YOLO: LT, LB, RB, RT. rm_vision PnP: LB, LT, RT, RB.
        image_points = np.asarray(
            [points[1], points[0], points[3], points[2]], dtype=np.float32
        )
        if self.armor_type == "large":
            armor_width, armor_height = 0.223, 0.057
        else:
            armor_width, armor_height = 0.132, 0.057
        half_y, half_z = armor_width / 2.0, armor_height / 2.0
        object_points = np.asarray(
            [
                [0.0, half_y, -half_z],
                [0.0, half_y, half_z],
                [0.0, -half_y, half_z],
                [0.0, -half_y, -half_z],
            ],
            dtype=np.float32,
        )
        try:
            success, rvec, tvec = cv2.solvePnP(
                object_points,
                image_points,
                self.camera_matrix,
                self.dist_coeffs,
                flags=cv2.SOLVEPNP_IPPE,
            )
        except cv2.error as exc:
            self.get_logger().debug(f"solvePnP failed: {exc}")
            return None
        if not success:
            return None

        position = tvec.reshape(3).astype(float)
        distance = float(np.linalg.norm(position))
        if (
            not np.isfinite(position).all()
            or position[2] <= 0.0
            or distance < self.min_pnp_distance
            or distance > self.max_pnp_distance
        ):
            return None
        projected, _ = cv2.projectPoints(
            object_points, rvec, tvec, self.camera_matrix, self.dist_coeffs
        )
        reprojection_error = float(
            np.mean(np.linalg.norm(projected.reshape(4, 2) - image_points, axis=1))
        )
        if reprojection_error > self.max_reprojection_error:
            return None

        rotation, _ = cv2.Rodrigues(rvec)
        qx, qy, qz, qw = rotation_matrix_to_quaternion(rotation)
        armor = Armor()
        armor.number = self.armor_number
        armor.type = self.armor_type
        armor.pose.position.x = float(position[0])
        armor.pose.position.y = float(position[1])
        armor.pose.position.z = float(position[2])
        armor.pose.orientation.x = qx
        armor.pose.orientation.y = qy
        armor.pose.orientation.z = qz
        armor.pose.orientation.w = qw
        center = np.mean(points, axis=0)
        principal_point = np.asarray(
            [self.camera_matrix[0, 2], self.camera_matrix[1, 2]], dtype=np.float32
        )
        armor.distance_to_image_center = float(np.linalg.norm(center - principal_point))
        armor.kpts = [
            Point(x=float(point[0]), y=float(point[1]), z=0.0) for point in points
        ]
        return armor

    def armor_from_raw_result(self, result: dict) -> Optional[Armor]:
        if not result["output_valid"] or self.camera_matrix is None:
            return None
        pnp = result["pnp"]
        points = np.asarray(result["pair"]["points_xy"], dtype=np.float32)
        position = np.asarray(pnp["tvec"], dtype=float)
        rotation, _ = cv2.Rodrigues(np.asarray(pnp["rvec"], dtype=np.float64))
        qx, qy, qz, qw = rotation_matrix_to_quaternion(rotation)

        armor = Armor()
        armor.number = self.armor_number
        armor.type = result["latched_type"]
        armor.pose.position.x = float(position[0])
        armor.pose.position.y = float(position[1])
        armor.pose.position.z = float(position[2])
        armor.pose.orientation.x = qx
        armor.pose.orientation.y = qy
        armor.pose.orientation.z = qz
        armor.pose.orientation.w = qw
        center = np.mean(points, axis=0)
        principal_point = np.asarray(
            [self.camera_matrix[0, 2], self.camera_matrix[1, 2]], dtype=np.float32
        )
        armor.distance_to_image_center = float(np.linalg.norm(center - principal_point))
        armor.kpts = [
            Point(x=float(point[0]), y=float(point[1]), z=0.0) for point in points
        ]
        return armor

    def draw_raw_result(
        self,
        image: np.ndarray,
        result: dict,
        confidence: float,
        armor: Optional[Armor],
    ) -> None:
        color = (0, 255, 0) if armor is not None else (0, 165, 255)
        x1, y1, x2, y2 = np.round(result["box_xyxy"]).astype(int)
        cv2.rectangle(image, (x1, y1), (x2, y2), color, 1)
        pair = result["pair"]
        if pair is not None:
            points = np.round(pair["points_xy"]).astype(np.int32)
            cv2.polylines(image, [points.reshape(-1, 1, 2)], True, color, 2)
            for point in points:
                cv2.circle(image, tuple(point), 3, (255, 0, 255), -1)

        ratio = "-" if result["raw_ratio"] is None else f"{result['raw_ratio']:.3f}"
        normalized_score = (
            "-"
            if result["normalized_dual_pnp_score"] is None
            else f"{result['normalized_dual_pnp_score']:.4f}"
        )
        latched_type = result["latched_type"] or "unknown"
        initialization = (
            "-"
            if result["initialization_frame"] is None
            else str(result["initialization_frame"])
        )
        failure = result["failure_reason"] or "ok"
        lines = [
            f"armor {confidence:.2f} trk={result['track_id']}",
            f"type={latched_type} init={initialization} raw={ratio} D={normalized_score}",
            f"status={failure}",
        ]
        text_x = max(0, x1)
        text_y = max(18, y1 - 42)
        for line_index, line in enumerate(lines):
            cv2.putText(
                image,
                line,
                (text_x, text_y + line_index * 17),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                color,
                1,
                cv2.LINE_AA,
            )
        if armor is not None:
            cv2.putText(
                image,
                f"x={armor.pose.position.x:.2f}m "
                f"y={armor.pose.position.y:.2f}m "
                f"z={armor.pose.position.z:.2f}m",
                (max(0, x1), min(image.shape[0] - 5, y2 + 18)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (0, 255, 0),
                1,
                cv2.LINE_AA,
            )

    def publish_empty(self, header) -> None:
        message = Armors()
        message.header = header
        self.armors_pub.publish(message)

    def publish_track_diagnostics_message(
        self,
        header,
        frame_index: int,
        raw_results: list[dict],
        confidences: np.ndarray,
        ended_track_ids: list[int],
    ) -> None:
        if self.track_diagnostics_pub is None:
            return
        try:
            dual_pnp_results = []
            for raw_result in raw_results:
                dual_pnp = raw_result.get("normalized_dual_pnp")
                pair = raw_result.get("pair")
                if (
                    dual_pnp is None
                    and pair is not None
                    and self.camera_matrix is not None
                    and self.dist_coeffs is not None
                ):
                    dual_pnp = solve_normalized_dual_pnp_type(
                        pair["points_xy"], self.camera_matrix, self.dist_coeffs
                    )
                dual_pnp_results.append(dual_pnp)
            output = String()
            output.data = serialize_track_diagnostics(
                header,
                frame_index,
                raw_results,
                confidences,
                ended_track_ids,
                dual_pnp_results,
            )
            self.track_diagnostics_pub.publish(output)
        except Exception as exc:  # noqa: BLE001 - diagnostics must not stop detection
            now = time.monotonic()
            if now - self.last_track_diagnostics_warning > 5.0:
                self.get_logger().warning(f"Cannot publish track diagnostics: {exc}")
                self.last_track_diagnostics_warning = now

    def image_callback(self, msg: Image) -> None:
        if self.camera_width and self.camera_height:
            mismatch = msg.width != self.camera_width or msg.height != self.camera_height
            if mismatch and time.monotonic() - self.last_image_mismatch_warning > 5.0:
                self.get_logger().warning(
                    "Image/CameraInfo resolution mismatch: "
                    f"image={msg.width}x{msg.height}, "
                    f"camera_info={self.camera_width}x{self.camera_height}"
                )
                self.last_image_mismatch_warning = time.monotonic()

        try:
            image = image_msg_to_bgr(msg)
        except (ValueError, cv2.error) as exc:
            self.get_logger().error(f"Cannot decode image ({msg.encoding}): {exc}")
            if self.raw_track_latch is not None:
                self.raw_track_latch.terminate_active()
            self.publish_empty(msg.header)
            return

        started = time.perf_counter()
        kwargs = {
            "source": image,
            "imgsz": self.imgsz,
            "conf": self.conf,
            "iou": self.nms_iou,
            "max_det": self.max_det,
            "agnostic_nms": self.agnostic_nms,
            "verbose": False,
        }
        if self.device:
            kwargs["device"] = self.device
        try:
            result = self.model.predict(**kwargs)[0]
        except Exception as exc:  # noqa: BLE001 - inference backend errors vary
            self.get_logger().error(f"YOLO inference failed: {exc}")
            if self.raw_track_latch is not None:
                self.raw_track_latch.terminate_active()
            self.publish_empty(msg.header)
            return

        inference_ms = (time.perf_counter() - started) * 1000.0
        self.frame_count += 1
        self.total_inference_ms += inference_ms
        annotated = image.copy() if self.publish_debug else None
        armors_msg = Armors()
        armors_msg.header = msg.header
        detection_count = 0
        valid_count = 0
        failure_counts = Counter()
        raw_results = []
        raw_confidences = np.empty((0,), dtype=np.float64)
        diagnostic_previous_track_ids = (
            set(self.raw_track_latch.tracker.active)
            if self.track_diagnostics_pub is not None
            and self.raw_track_latch is not None
            else set()
        )

        if result.boxes is not None:
            boxes = result.boxes.xyxy.cpu().numpy()
            confidences = result.boxes.conf.cpu().numpy()
            detection_count = len(boxes)

            if self.type_strategy in TRACK_LATCH_STRATEGIES:
                raw_results = self.raw_track_latch.process_frame(
                    image,
                    boxes,
                    self.frame_count - 1,
                    self.camera_matrix,
                    self.dist_coeffs,
                    confidences,
                )
                raw_confidences = confidences
                ownership = self.raw_track_latch.last_ownership_result
                if ownership and ownership["overload_component_count"]:
                    now = time.monotonic()
                    if now - self.last_ownership_overload_warning > 5.0:
                        self.get_logger().warning(
                            "Light ownership overload passthrough: "
                            f"components={ownership['overload_component_count']}, "
                            f"sizes={ownership['overload_component_sizes']}"
                        )
                        self.last_ownership_overload_warning = now
                for confidence, raw_result in zip(confidences, raw_results):
                    valid_count += int(raw_result["pair"] is not None)
                    armor = self.armor_from_raw_result(raw_result)
                    if armor is not None:
                        armors_msg.armors.append(armor)
                    else:
                        failure_counts[raw_result["failure_reason"]] += 1
                    if annotated is not None:
                        self.draw_raw_result(
                            annotated, raw_result, float(confidence), armor
                        )
            elif result.keypoints is not None:
                keypoints = result.keypoints.xy.cpu().numpy()
                keypoint_conf = (
                    result.keypoints.conf.cpu().numpy()
                    if result.keypoints.conf is not None
                    else None
                )
                for index, (box, confidence, points) in enumerate(
                    zip(boxes, confidences, keypoints)
                ):
                    points = np.asarray(points, dtype=np.float32)
                    valid = self.valid_keypoints(image, points)
                    if (
                        keypoint_conf is not None
                        and float(np.min(keypoint_conf[index])) < self.min_keypoint_conf
                    ):
                        valid = False
                    valid_count += int(valid)

                    armor = self.solve_armor(points) if valid else None
                    if armor is not None:
                        armors_msg.armors.append(armor)

                    if annotated is not None:
                        color = (0, 255, 0) if armor is not None else (0, 165, 255)
                        polygon = np.round(points).astype(np.int32).reshape(-1, 1, 2)
                        cv2.polylines(annotated, [polygon], True, color, 2)
                        x1, y1, x2, y2 = np.round(box).astype(int)
                        cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 1)
                        cv2.putText(
                            annotated,
                            f"armor {confidence:.2f}",
                            (max(0, x1), max(18, y1 - 5)),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.55,
                            color,
                            2,
                            cv2.LINE_AA,
                        )
                        for point_index, (x, y) in enumerate(points):
                            px, py = int(round(x)), int(round(y))
                            cv2.circle(annotated, (px, py), 3, (255, 0, 255), -1)
                            cv2.putText(
                                annotated,
                                str(point_index),
                                (px + 4, py - 4),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                0.45,
                                (255, 0, 255),
                                1,
                                cv2.LINE_AA,
                            )
                        if armor is not None:
                            cv2.putText(
                                annotated,
                                f"x={armor.pose.position.x:.2f}m "
                                f"y={armor.pose.position.y:.2f}m "
                                f"z={armor.pose.position.z:.2f}m",
                                (max(0, x1), min(annotated.shape[0] - 5, y2 + 18)),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                0.45,
                                (0, 255, 0),
                                1,
                                cv2.LINE_AA,
                            )
        elif self.type_strategy in TRACK_LATCH_STRATEGIES:
            self.raw_track_latch.process_frame(
                image,
                np.empty((0, 4), dtype=np.float32),
                self.frame_count - 1,
                self.camera_matrix,
                self.dist_coeffs,
                np.empty((0,), dtype=np.float32),
            )

        if self.track_diagnostics_pub is not None:
            current_track_ids = (
                set(self.raw_track_latch.tracker.active)
                if self.raw_track_latch is not None
                else set()
            )
            ended_track_ids = sorted(
                diagnostic_previous_track_ids - current_track_ids
            )
            self.publish_track_diagnostics_message(
                msg.header,
                self.frame_count - 1,
                raw_results,
                raw_confidences,
                ended_track_ids,
            )

        if self.camera_matrix is None:
            now = time.monotonic()
            if now - self.last_camera_warning > 5.0:
                self.get_logger().warning(
                    f"Waiting for {self.camera_info_topic}; PnP/Armors output is empty"
                )
                self.last_camera_warning = now

        self.armors_pub.publish(armors_msg)
        if self.debug_pub is not None and annotated is not None:
            self.debug_pub.publish(bgr_to_image_msg(annotated, msg.header))

        if self.diagnostics_period > 0 and self.frame_count % self.diagnostics_period == 0:
            failure_summary = (
                f", failures={dict(sorted(failure_counts.items()))}"
                if self.type_strategy in TRACK_LATCH_STRATEGIES
                else ""
            )
            self.get_logger().info(
                f"frames={self.frame_count}, detections={detection_count}, "
                f"valid_quads={valid_count}, armors={len(armors_msg.armors)}, "
                f"avg_inference_ms={self.total_inference_ms / self.frame_count:.1f}"
                f"{failure_summary}"
            )


def main() -> None:
    rclpy.init()
    node = None
    try:
        node = YoloArmorNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
