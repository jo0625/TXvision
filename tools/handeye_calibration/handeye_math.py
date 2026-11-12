#!/usr/bin/env python3
"""Pure geometry helpers for fixed-board gimbal hand-eye calibration."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Iterable

import cv2
import numpy as np
import yaml
from scipy.spatial.transform import Rotation


PATTERN_SIZE = (9, 6)
SQUARE_SIZE_M = 0.024


def load_camera_calibration(path: Path) -> tuple[np.ndarray, np.ndarray, dict]:
    payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    camera_matrix = np.asarray(
        payload["camera_matrix"]["data"], dtype=np.float64
    ).reshape(3, 3)
    distortion = np.asarray(
        payload["distortion_coefficients"]["data"], dtype=np.float64
    ).reshape(-1, 1)
    if not np.all(np.isfinite(camera_matrix)) or not np.all(np.isfinite(distortion)):
        raise ValueError("camera calibration contains non-finite values")
    return camera_matrix, distortion, {
        "path": str(path.resolve()),
        "image_width": int(payload["image_width"]),
        "image_height": int(payload["image_height"]),
        "camera_matrix": camera_matrix.tolist(),
        "distortion_coefficients": distortion.reshape(-1).tolist(),
        "distortion_model": str(payload.get("distortion_model", "plumb_bob")),
    }


def chessboard_object_points(
    pattern_size: tuple[int, int] = PATTERN_SIZE,
    square_size_m: float = SQUARE_SIZE_M,
) -> np.ndarray:
    if square_size_m <= 0 or not math.isfinite(square_size_m):
        raise ValueError("square_size_m must be finite and positive")
    points = np.zeros((pattern_size[0] * pattern_size[1], 3), dtype=np.float32)
    points[:, :2] = np.mgrid[
        0 : pattern_size[0], 0 : pattern_size[1]
    ].T.reshape(-1, 2)
    points[:, :2] *= square_size_m
    return points


def detect_chessboard(
    image_bgr: np.ndarray,
    pattern_size: tuple[int, int] = PATTERN_SIZE,
) -> np.ndarray | None:
    gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
    found, corners = cv2.findChessboardCornersSB(
        gray,
        pattern_size,
        flags=cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE,
    )
    expected = pattern_size[0] * pattern_size[1]
    if not found or corners is None or len(corners) != expected:
        return None
    corners = corners.astype(np.float32)
    cv2.cornerSubPix(
        gray,
        corners,
        (5, 5),
        (-1, -1),
        (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 40, 0.001),
    )
    return corners.reshape(-1, 2)


def estimate_outer_board(
    corners: np.ndarray,
    pattern_size: tuple[int, int] = PATTERN_SIZE,
) -> np.ndarray:
    columns, rows = pattern_size
    grid = np.asarray(corners, dtype=np.float64).reshape(rows, columns, 2)
    top_left = grid[0, 0]
    top_right = grid[0, -1]
    bottom_left = grid[-1, 0]
    bottom_right = grid[-1, -1]
    horizontal = (
        (top_right - top_left) + (bottom_right - bottom_left)
    ) / (2.0 * (columns - 1))
    vertical = (
        (bottom_left - top_left) + (bottom_right - top_right)
    ) / (2.0 * (rows - 1))
    return np.asarray(
        [
            top_left - horizontal - vertical,
            top_right + horizontal - vertical,
            bottom_right + horizontal + vertical,
            bottom_left - horizontal + vertical,
        ],
        dtype=np.float64,
    )


def board_fully_in_frame(
    corners: np.ndarray,
    width: int,
    height: int,
    margin_px: float = 3.0,
) -> tuple[bool, np.ndarray]:
    outer = estimate_outer_board(corners)
    inside = bool(
        np.all(outer[:, 0] >= margin_px)
        and np.all(outer[:, 0] < width - margin_px)
        and np.all(outer[:, 1] >= margin_px)
        and np.all(outer[:, 1] < height - margin_px)
    )
    return inside, outer


def solve_board_pose(
    corners: np.ndarray,
    camera_matrix: np.ndarray,
    distortion: np.ndarray,
    square_size_m: float = SQUARE_SIZE_M,
) -> dict:
    object_points = chessboard_object_points(square_size_m=square_size_m)
    success, rvec, tvec = cv2.solvePnP(
        object_points,
        np.asarray(corners, dtype=np.float32),
        camera_matrix,
        distortion,
        flags=cv2.SOLVEPNP_ITERATIVE,
    )
    if not success:
        raise RuntimeError("solvePnP failed")
    rotation, _ = cv2.Rodrigues(rvec)
    projected, _ = cv2.projectPoints(
        object_points, rvec, tvec, camera_matrix, distortion
    )
    errors = np.linalg.norm(
        np.asarray(corners).reshape(-1, 2) - projected.reshape(-1, 2), axis=1
    )
    return {
        "transform": make_transform(rotation, tvec.reshape(3)),
        "rotation_matrix": rotation,
        "rvec": rvec.reshape(3),
        "tvec": tvec.reshape(3),
        "mean_reprojection_error_px": float(np.mean(errors)),
        "rms_reprojection_error_px": float(np.sqrt(np.mean(np.square(errors)))),
        "max_reprojection_error_px": float(np.max(errors)),
        "projected_points": projected.reshape(-1, 2),
    }


def make_transform(rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = np.asarray(rotation, dtype=np.float64).reshape(3, 3)
    transform[:3, 3] = np.asarray(translation, dtype=np.float64).reshape(3)
    return transform


def invert_transform(transform: np.ndarray) -> np.ndarray:
    transform = np.asarray(transform, dtype=np.float64).reshape(4, 4)
    rotation = transform[:3, :3]
    translation = transform[:3, 3]
    return make_transform(rotation.T, -rotation.T @ translation)


def world_from_pitch(
    yaw_rad: float,
    pitch_rad: float,
    yaw_to_pitch_m: Iterable[float] = (0.0, 0.0, 0.178),
) -> np.ndarray:
    if not math.isfinite(yaw_rad) or not math.isfinite(pitch_rad):
        raise ValueError("yaw and pitch must be finite")
    world_from_yaw_rotation = Rotation.from_euler("z", yaw_rad).as_matrix()
    yaw_from_pitch_rotation = Rotation.from_euler("y", pitch_rad).as_matrix()
    yaw_to_pitch = np.asarray(tuple(yaw_to_pitch_m), dtype=np.float64).reshape(3)
    return make_transform(
        world_from_yaw_rotation @ yaw_from_pitch_rotation,
        world_from_yaw_rotation @ yaw_to_pitch,
    )


def camera_link_from_optical() -> np.ndarray:
    return make_transform(
        Rotation.from_euler("xyz", (-math.pi / 2.0, 0.0, -math.pi / 2.0)).as_matrix(),
        np.zeros(3),
    )


def mechanical_pitch_from_optical(
    pitch_to_camera_m: Iterable[float] = (0.10423, 0.0, 0.0228),
) -> np.ndarray:
    pitch_from_link = make_transform(np.eye(3), np.asarray(tuple(pitch_to_camera_m)))
    return pitch_from_link @ camera_link_from_optical()


HAND_EYE_METHODS = {
    "TSAI": cv2.CALIB_HAND_EYE_TSAI,
    "PARK": cv2.CALIB_HAND_EYE_PARK,
    "HORAUD": cv2.CALIB_HAND_EYE_HORAUD,
    "ANDREFF": cv2.CALIB_HAND_EYE_ANDREFF,
    "DANIILIDIS": cv2.CALIB_HAND_EYE_DANIILIDIS,
}


def solve_handeye(
    world_from_pitch_poses: list[np.ndarray],
    camera_from_board_poses: list[np.ndarray],
    method: str = "PARK",
) -> np.ndarray:
    if len(world_from_pitch_poses) != len(camera_from_board_poses):
        raise ValueError("robot and board pose counts differ")
    if len(world_from_pitch_poses) < 3:
        raise ValueError("at least three paired poses are required")
    method_name = method.upper()
    if method_name not in HAND_EYE_METHODS:
        raise ValueError(f"unknown hand-eye method: {method}")
    robot = [np.asarray(pose, dtype=np.float64).reshape(4, 4) for pose in world_from_pitch_poses]
    target = [np.asarray(pose, dtype=np.float64).reshape(4, 4) for pose in camera_from_board_poses]
    rotation, translation = cv2.calibrateHandEye(
        [pose[:3, :3] for pose in robot],
        [pose[:3, 3].reshape(3, 1) for pose in robot],
        [pose[:3, :3] for pose in target],
        [pose[:3, 3].reshape(3, 1) for pose in target],
        method=HAND_EYE_METHODS[method_name],
    )
    result = make_transform(rotation, translation)
    if not np.all(np.isfinite(result)):
        raise RuntimeError(f"{method_name} produced non-finite output")
    return result


def rotation_error_deg(first: np.ndarray, second: np.ndarray) -> float:
    delta = np.asarray(first).reshape(3, 3).T @ np.asarray(second).reshape(3, 3)
    return float(np.degrees(Rotation.from_matrix(delta).magnitude()))


def mean_pose(poses: list[np.ndarray]) -> np.ndarray:
    if not poses:
        raise ValueError("cannot average an empty pose list")
    rotations = np.asarray([pose[:3, :3] for pose in poses])
    translations = np.asarray([pose[:3, 3] for pose in poses])
    return make_transform(
        Rotation.from_matrix(rotations).mean().as_matrix(),
        np.mean(translations, axis=0),
    )


def fixed_board_residuals(
    world_from_pitch_poses: list[np.ndarray],
    pitch_from_camera: np.ndarray,
    camera_from_board_poses: list[np.ndarray],
    reference_world_from_board: np.ndarray | None = None,
) -> tuple[np.ndarray, list[dict]]:
    world_from_boards = [
        world_from_pitch @ pitch_from_camera @ camera_from_board
        for world_from_pitch, camera_from_board in zip(
            world_from_pitch_poses, camera_from_board_poses
        )
    ]
    reference = (
        mean_pose(world_from_boards)
        if reference_world_from_board is None
        else np.asarray(reference_world_from_board, dtype=np.float64).reshape(4, 4)
    )
    residuals = [
        {
            "translation_mm": float(
                np.linalg.norm(pose[:3, 3] - reference[:3, 3]) * 1000.0
            ),
            "rotation_deg": rotation_error_deg(reference[:3, :3], pose[:3, :3]),
        }
        for pose in world_from_boards
    ]
    return reference, residuals


def transform_payload(transform: np.ndarray) -> dict:
    transform = np.asarray(transform, dtype=np.float64).reshape(4, 4)
    quaternion = Rotation.from_matrix(transform[:3, :3]).as_quat()
    return {
        "matrix": transform.tolist(),
        "translation_m": transform[:3, 3].tolist(),
        "rotation_matrix": transform[:3, :3].tolist(),
        "quaternion_xyzw": quaternion.tolist(),
    }
