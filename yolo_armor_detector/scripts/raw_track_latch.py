#!/usr/bin/env python3
"""Traditional light extraction and causal track type latching."""

from __future__ import annotations

import itertools
import math
from collections import Counter, defaultdict
from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np


ROI_EXPANSION_FRACTION_PER_SIDE = 0.10
GRAY_THRESHOLD = 160
MIN_CONTOUR_POINTS = 5
MIN_WIDTH_LENGTH_RATIO = 0.06
MAX_WIDTH_LENGTH_RATIO = 0.40
MAX_TILT_DEG = 35.0
MIN_FILL_RATIO = 0.80
MIN_PAIR_LIGHT_LENGTH_RATIO = 0.50
MAX_PAIR_CENTER_ANGLE_DEG = 45.0
MIN_NORMALIZED_CENTER_DISTANCE = 0.80
MAX_NORMALIZED_CENTER_DISTANCE = 5.50
TYPE_BOUNDARY = 3.20
NORMALIZED_DUAL_PNP_TYPE_BOUNDARY = 0.0326
TRACK_IOU_THRESHOLD = 0.30
MIN_PNP_DISTANCE_M = 0.10
MAX_PNP_DISTANCE_M = 10.0
MAX_MEAN_REPROJECTION_ERROR_PX = 12.0
LIGHT_OWNERSHIP_ENDPOINT_DISTANCE_PX = 6.0
LIGHT_OWNERSHIP_DIRECTION_DIFFERENCE_DEG = 5.0
MAX_EXACT_OWNERSHIP_CONFLICT_COMPONENT_SIZE = 12
LIGHT_GEOMETRY_M = {
    "small": (0.132, 0.056),
    "large": (0.228, 0.053),
}


@dataclass
class TrackState:
    track_id: int
    start_frame: int
    last_frame: int
    last_box: np.ndarray
    latched_type: Optional[str] = None
    initialization_frame: Optional[int] = None
    initialization_ratio: Optional[float] = None
    initialization_metric: Optional[str] = None
    initialization_value: Optional[float] = None


def box_iou(first: np.ndarray, second: np.ndarray) -> float:
    ax1, ay1, ax2, ay2 = map(float, first)
    bx1, by1, bx2, by2 = map(float, second)
    width = max(0.0, min(ax2, bx2) - max(ax1, bx1))
    height = max(0.0, min(ay2, by2) - max(ay1, by1))
    intersection = width * height
    first_area = max(0.0, ax2 - ax1) * max(0.0, ay2 - ay1)
    second_area = max(0.0, bx2 - bx1) * max(0.0, by2 - by1)
    union = first_area + second_area - intersection
    return intersection / union if union > 0.0 else 0.0


def expanded_roi(
    box: np.ndarray, image_width: int, image_height: int
) -> tuple[int, int, int, int]:
    x1, y1, x2, y2 = map(float, box)
    box_width = max(x2 - x1, 1.0)
    box_height = max(y2 - y1, 1.0)
    return (
        max(0, int(math.floor(x1 - ROI_EXPANSION_FRACTION_PER_SIDE * box_width))),
        max(0, int(math.floor(y1 - ROI_EXPANSION_FRACTION_PER_SIDE * box_height))),
        min(
            image_width,
            int(math.ceil(x2 + ROI_EXPANSION_FRACTION_PER_SIDE * box_width)),
        ),
        min(
            image_height,
            int(math.ceil(y2 + ROI_EXPANSION_FRACTION_PER_SIDE * box_height)),
        ),
    )


def _dominant_reason(rejections: Counter) -> str:
    if not rejections:
        return "none"
    return sorted(rejections.items(), key=lambda item: (-item[1], item[0]))[0][0]


def light_from_contour(
    contour: np.ndarray, roi_x: int, roi_y: int
) -> tuple[Optional[dict], str]:
    if len(contour) < MIN_CONTOUR_POINTS:
        return None, "contour_points"
    x, y, width, height = cv2.boundingRect(contour)
    rotated = cv2.minAreaRect(contour)
    rotated_area = float(rotated[1][0] * rotated[1][1])
    if rotated_area <= 0.0:
        return None, "rotated_area"

    mask = np.zeros((height, width), dtype=np.uint8)
    shifted = contour.reshape(-1, 2) - np.asarray([x, y], dtype=np.int32)
    cv2.fillPoly(mask, [shifted.reshape(-1, 1, 2)], 255)
    nonzero = cv2.findNonZero(mask)
    if nonzero is None or len(nonzero) < 2:
        return None, "filled_points"
    filled_points = nonzero.reshape(-1, 2).astype(np.float32)
    fill_ratio = float(len(filled_points) / rotated_area)
    if not fill_ratio > MIN_FILL_RATIO:
        return None, "fill_ratio"

    vx, vy, fit_x, fit_y = cv2.fitLine(
        filled_points, cv2.DIST_L2, 0, 0.01, 0.01
    ).reshape(4)
    tilt_deg = float(abs(math.degrees(math.atan2(float(vx), float(vy)))))
    if tilt_deg > 90.0:
        tilt_deg = 180.0 - tilt_deg

    if abs(float(vy)) < 1e-9:
        center_x = float(x + width / 2.0)
        top_local = np.asarray([center_x, float(y)], dtype=np.float32)
        bottom_local = np.asarray([center_x, float(y + height)], dtype=np.float32)
    else:
        top_x = float(fit_x + (0.0 - fit_y) * vx / vy)
        bottom_x = float(fit_x + (float(height) - fit_y) * vx / vy)
        top_local = np.asarray([x + top_x, float(y)], dtype=np.float32)
        bottom_local = np.asarray(
            [x + bottom_x, float(y + height)], dtype=np.float32
        )

    offset = np.asarray([roi_x, roi_y], dtype=np.float32)
    top = top_local + offset
    bottom = bottom_local + offset
    length = float(np.linalg.norm(top - bottom))
    if length <= 0.0:
        return None, "light_length"
    light_width = float(len(filled_points) / length)
    width_length_ratio = light_width / length
    if not MIN_WIDTH_LENGTH_RATIO <= width_length_ratio <= MAX_WIDTH_LENGTH_RATIO:
        return None, "width_length_ratio"
    if not tilt_deg < MAX_TILT_DEG:
        return None, "tilt"

    return {
        "top": top,
        "bottom": bottom,
        "center": (top + bottom) / 2.0,
        "length_px": length,
        "width_px": light_width,
        "width_length_ratio": width_length_ratio,
        "tilt_deg": tilt_deg,
        "fill_ratio": fill_ratio,
        "contour_pixel_count": int(len(filled_points)),
    }, ""


def validate_gray_threshold(gray_threshold: int) -> int:
    if (
        isinstance(gray_threshold, bool)
        or not isinstance(gray_threshold, int)
        or not 0 <= gray_threshold <= 255
    ):
        raise ValueError("gray_threshold must be an integer in [0, 255]")
    return gray_threshold


def find_lights(
    image: np.ndarray,
    roi: tuple[int, int, int, int],
    gray_threshold: int = GRAY_THRESHOLD,
) -> tuple[list[dict], int, Counter]:
    gray_threshold = validate_gray_threshold(gray_threshold)
    x1, y1, x2, y2 = roi
    crop = image[y1:y2, x1:x2]
    if crop.size == 0:
        return [], 0, Counter({"empty_roi": 1})
    gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
    _, binary = cv2.threshold(gray, gray_threshold, 255, cv2.THRESH_BINARY)
    contours, _ = cv2.findContours(
        binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    lights = []
    rejections = Counter()
    for contour in contours:
        light, reason = light_from_contour(contour, x1, y1)
        if light is None:
            rejections[reason] += 1
        else:
            lights.append(light)
    lights.sort(
        key=lambda item: (float(item["center"][0]), float(item["center"][1]))
    )
    return lights, len(contours), rejections


def pair_lights(lights: list[dict]) -> tuple[Optional[dict], int, Counter]:
    pairs = []
    rejections = Counter()
    for left_index in range(len(lights)):
        for right_index in range(left_index + 1, len(lights)):
            first, second = lights[left_index], lights[right_index]
            left, right = (
                (first, second)
                if first["center"][0] <= second["center"][0]
                else (second, first)
            )
            length_ratio = min(left["length_px"], right["length_px"]) / max(
                left["length_px"], right["length_px"]
            )
            if not length_ratio > MIN_PAIR_LIGHT_LENGTH_RATIO:
                rejections["light_length_ratio"] += 1
                continue
            delta = np.asarray(right["center"]) - np.asarray(left["center"])
            center_angle = float(abs(math.degrees(math.atan2(delta[1], delta[0]))))
            if not center_angle < MAX_PAIR_CENTER_ANGLE_DEG:
                rejections["center_line_angle"] += 1
                continue
            average_length = (left["length_px"] + right["length_px"]) / 2.0
            normalized_distance = float(np.linalg.norm(delta) / average_length)
            if not (
                MIN_NORMALIZED_CENTER_DISTANCE
                <= normalized_distance
                < MAX_NORMALIZED_CENTER_DISTANCE
            ):
                rejections["normalized_center_distance"] += 1
                continue
            raw_type = "small" if normalized_distance < TYPE_BOUNDARY else "large"
            points = np.asarray(
                [left["top"], left["bottom"], right["bottom"], right["top"]],
                dtype=np.float32,
            )
            pairs.append(
                {
                    "left": left,
                    "right": right,
                    "points_xy": points,
                    "light_length_sum_px": left["length_px"]
                    + right["length_px"],
                    "light_length_ratio": length_ratio,
                    "center_line_angle_deg": center_angle,
                    "normalized_center_distance": normalized_distance,
                    "raw_type": raw_type,
                }
            )
    if not pairs:
        return None, 0, rejections
    pairs.sort(
        key=lambda pair: (
            -pair["light_length_sum_px"],
            float(pair["left"]["center"][0]),
            float(pair["right"]["center"][0]),
        )
    )
    return pairs[0], len(pairs), rejections


def extract_light_pair(
    image: np.ndarray,
    box: np.ndarray,
    gray_threshold: int = GRAY_THRESHOLD,
) -> dict:
    roi = expanded_roi(box, image.shape[1], image.shape[0])
    lights, contour_count, contour_rejections = find_lights(
        image, roi, gray_threshold
    )
    pair, pair_count, pair_rejections = pair_lights(lights)
    if pair is not None:
        failure_reason = ""
    elif not lights:
        failure_reason = f"no_valid_light:{_dominant_reason(contour_rejections)}"
    elif len(lights) < 2:
        failure_reason = "unable_to_pair:insufficient_lights"
    else:
        failure_reason = f"unable_to_pair:{_dominant_reason(pair_rejections)}"
    return {
        "roi_xyxy": roi,
        "raw_contour_count": contour_count,
        "valid_light_count": len(lights),
        "valid_pair_count": pair_count,
        "contour_rejections": dict(sorted(contour_rejections.items())),
        "pair_rejections": dict(sorted(pair_rejections.items())),
        "pair": pair,
        "failure_reason": failure_reason,
    }


def points_in_image_bounds(
    points_xy: np.ndarray, image_width: int, image_height: int
) -> bool:
    points = np.asarray(points_xy, dtype=np.float64)
    return bool(
        points.shape == (4, 2)
        and np.isfinite(points).all()
        and (points[:, 0] >= 0.0).all()
        and (points[:, 0] < image_width).all()
        and (points[:, 1] >= 0.0).all()
        and (points[:, 1] < image_height).all()
    )


def _object_points(armor_type: str) -> np.ndarray:
    width, height = LIGHT_GEOMETRY_M[armor_type]
    half_y, half_z = width / 2.0, height / 2.0
    return np.asarray(
        [
            [0.0, half_y, -half_z],
            [0.0, half_y, half_z],
            [0.0, -half_y, half_z],
            [0.0, -half_y, -half_z],
        ],
        dtype=np.float32,
    )


def solve_light_pnp(
    points: np.ndarray,
    armor_type: str,
    camera_matrix: np.ndarray,
    distortion: np.ndarray,
) -> dict:
    semantic_points = np.asarray(points, dtype=np.float32)
    image_points = np.asarray(
        [semantic_points[1], semantic_points[0], semantic_points[3], semantic_points[2]],
        dtype=np.float32,
    )
    object_points = _object_points(armor_type)
    try:
        success, rvec, tvec = cv2.solvePnP(
            object_points,
            image_points,
            camera_matrix,
            distortion,
            flags=cv2.SOLVEPNP_IPPE,
        )
    except cv2.error as exc:
        return {"solved": False, "valid": False, "failure_reason": "pnp_failed", "error": str(exc)}
    if not success:
        return {"solved": False, "valid": False, "failure_reason": "pnp_failed"}

    position = tvec.reshape(3).astype(float)
    distance = float(np.linalg.norm(position))
    projected, _ = cv2.projectPoints(
        object_points, rvec, tvec, camera_matrix, distortion
    )
    projected_pnp = projected.reshape(4, 2)
    projected_semantic = np.asarray(
        [projected_pnp[1], projected_pnp[0], projected_pnp[3], projected_pnp[2]],
        dtype=np.float32,
    )
    point_errors = np.linalg.norm(projected_semantic - semantic_points, axis=1)
    mean_error = float(np.mean(point_errors))
    max_error = float(np.max(point_errors))

    failure_reason = ""
    if not np.isfinite(position).all():
        failure_reason = "nonfinite_position"
    elif position[2] <= 0.0:
        failure_reason = "nonpositive_depth"
    elif distance < MIN_PNP_DISTANCE_M:
        failure_reason = "distance_below_min"
    elif distance > MAX_PNP_DISTANCE_M:
        failure_reason = "distance_above_max"
    elif mean_error > MAX_MEAN_REPROJECTION_ERROR_PX:
        failure_reason = "reprojection_error"
    return {
        "solved": True,
        "valid": not failure_reason,
        "failure_reason": failure_reason,
        "rvec": rvec.reshape(3).astype(float),
        "tvec": position,
        "distance_m": distance,
        "projected_points": projected_semantic,
        "point_errors_px": point_errors.astype(float),
        "mean_reprojection_error_px": mean_error,
        "max_reprojection_error_px": max_error,
    }


def normalized_dual_pnp_decision_from_results(
    points: np.ndarray,
    small_pnp: dict,
    large_pnp: dict,
) -> dict:
    semantic_points = np.asarray(points, dtype=np.float64)
    left_length = float(np.linalg.norm(semantic_points[1] - semantic_points[0]))
    right_length = float(np.linalg.norm(semantic_points[2] - semantic_points[3]))
    mean_light_length = (left_length + right_length) / 2.0

    def solved_result_is_finite(result: dict) -> bool:
        if not result.get("solved"):
            return False
        required_scalars = (
            result.get("mean_reprojection_error_px"),
            result.get("max_reprojection_error_px"),
        )
        if not all(
            value is not None and math.isfinite(float(value))
            for value in required_scalars
        ):
            return False
        return all(
            value is not None
            and np.isfinite(np.asarray(value, dtype=np.float64)).all()
            for value in (result.get("rvec"), result.get("tvec"))
        )

    failure_reason = ""
    if not small_pnp.get("solved"):
        failure_reason = "small_pnp_unsolved"
    elif not large_pnp.get("solved"):
        failure_reason = "large_pnp_unsolved"
    elif not solved_result_is_finite(small_pnp):
        failure_reason = "small_pnp_nonfinite"
    elif not solved_result_is_finite(large_pnp):
        failure_reason = "large_pnp_nonfinite"
    elif not math.isfinite(mean_light_length) or mean_light_length <= 0.0:
        failure_reason = "invalid_mean_light_length"

    score = None
    armor_type = None
    if not failure_reason:
        score = abs(
            float(small_pnp["mean_reprojection_error_px"])
            - float(large_pnp["mean_reprojection_error_px"])
        ) / mean_light_length
        armor_type = (
            "small" if score < NORMALIZED_DUAL_PNP_TYPE_BOUNDARY else "large"
        )
    return {
        "evaluable": not failure_reason,
        "failure_reason": failure_reason,
        "score": score,
        "type": armor_type,
        "mean_light_length_px": mean_light_length,
        "left_light_length_px": left_length,
        "right_light_length_px": right_length,
        "small_pnp": small_pnp,
        "large_pnp": large_pnp,
    }


def solve_normalized_dual_pnp_type(
    points: np.ndarray,
    camera_matrix: np.ndarray,
    distortion: np.ndarray,
) -> dict:
    small_pnp = solve_light_pnp(points, "small", camera_matrix, distortion)
    large_pnp = solve_light_pnp(points, "large", camera_matrix, distortion)
    return normalized_dual_pnp_decision_from_results(
        points, small_pnp, large_pnp
    )


def light_direction_difference_deg(first: dict, second: dict) -> float:
    """Return the undirected angle between two fitted light axes."""
    first_vector = np.asarray(first["bottom"], dtype=np.float64) - np.asarray(
        first["top"], dtype=np.float64
    )
    second_vector = np.asarray(second["bottom"], dtype=np.float64) - np.asarray(
        second["top"], dtype=np.float64
    )
    denominator = float(np.linalg.norm(first_vector) * np.linalg.norm(second_vector))
    if denominator <= 0.0:
        return float("inf")
    cross = abs(
        float(
            first_vector[0] * second_vector[1]
            - first_vector[1] * second_vector[0]
        )
    )
    dot = abs(float(np.dot(first_vector, second_vector)))
    return float(math.degrees(math.atan2(cross, dot)))


def same_physical_light(first: dict, second: dict) -> bool:
    return (
        float(np.linalg.norm(np.asarray(first["top"]) - np.asarray(second["top"])))
        <= LIGHT_OWNERSHIP_ENDPOINT_DISTANCE_PX
        and float(
            np.linalg.norm(np.asarray(first["bottom"]) - np.asarray(second["bottom"]))
        )
        <= LIGHT_OWNERSHIP_ENDPOINT_DISTANCE_PX
        and light_direction_difference_deg(first, second)
        <= LIGHT_OWNERSHIP_DIRECTION_DIFFERENCE_DEG
    )


class _UnionFind:
    def __init__(self, size: int) -> None:
        self.parent = list(range(size))

    def find(self, index: int) -> int:
        while self.parent[index] != index:
            self.parent[index] = self.parent[self.parent[index]]
            index = self.parent[index]
        return index

    def union(self, first: int, second: int) -> None:
        first_root = self.find(first)
        second_root = self.find(second)
        if first_root != second_root:
            self.parent[max(first_root, second_root)] = min(first_root, second_root)


def select_light_ownership(candidates: list[dict]) -> dict:
    """Resolve ownership exactly within bounded independent conflict components."""
    valid = [
        candidate
        for candidate in candidates
        if candidate["output_valid"] and candidate["pair"] is not None
    ]
    observations = []
    observation_index = {}
    for candidate in valid:
        for side in ("left", "right"):
            observation_index[(candidate["candidate_index"], side)] = len(observations)
            observations.append(
                {
                    "candidate_index": candidate["candidate_index"],
                    "side": side,
                    "light": candidate["pair"][side],
                }
            )

    union_find = _UnionFind(len(observations))
    for first_index, second_index in itertools.combinations(
        range(len(observations)), 2
    ):
        if same_physical_light(
            observations[first_index]["light"], observations[second_index]["light"]
        ):
            union_find.union(first_index, second_index)

    roots = sorted({union_find.find(index) for index in range(len(observations))})
    root_cluster_ids = {root: index for index, root in enumerate(roots)}
    clusters = []
    for root in roots:
        cluster_id = root_cluster_ids[root]
        members = []
        for index, observation in enumerate(observations):
            if union_find.find(index) != root:
                continue
            members.append(
                {
                    "candidate_index": observation["candidate_index"],
                    "side": observation["side"],
                }
            )
        clusters.append({"cluster_id": cluster_id, "members": members})

    candidate_clusters = {}
    for candidate in valid:
        candidate_clusters[candidate["candidate_index"]] = tuple(
            root_cluster_ids[
                union_find.find(observation_index[(candidate["candidate_index"], side)])
            ]
            for side in ("left", "right")
        )

    conflicts = []
    conflict_pairs = set()
    conflict_adjacency = {
        candidate["candidate_index"]: set() for candidate in valid
    }
    for first, second in itertools.combinations(valid, 2):
        first_index = first["candidate_index"]
        second_index = second["candidate_index"]
        shared = sorted(
            set(candidate_clusters[first_index]) & set(candidate_clusters[second_index])
        )
        if not shared:
            continue
        conflict_pairs.add(frozenset((first_index, second_index)))
        conflict_adjacency[first_index].add(second_index)
        conflict_adjacency[second_index].add(first_index)
        conflicts.append(
            {
                "first_candidate_index": first_index,
                "second_candidate_index": second_index,
                "shared_light_cluster_ids": shared,
            }
        )

    candidate_by_index = {
        candidate["candidate_index"]: candidate for candidate in valid
    }
    selected_indices = set()
    resolution_modes = {}
    component_sizes = {}
    conflict_components = []
    visited = set()
    largest_conflict_component_size = 0
    exact_component_count = 0
    overload_component_count = 0
    exact_subsets_evaluated = 0
    overload_component_sizes = []

    for start_index in sorted(conflict_adjacency):
        if start_index in visited:
            continue
        pending = [start_index]
        component = []
        visited.add(start_index)
        while pending:
            candidate_index = pending.pop()
            component.append(candidate_index)
            for neighbor in sorted(conflict_adjacency[candidate_index], reverse=True):
                if neighbor not in visited:
                    visited.add(neighbor)
                    pending.append(neighbor)
        component = sorted(component)
        component_size = len(component)
        for candidate_index in component:
            component_sizes[candidate_index] = component_size

        if component_size == 1 and not conflict_adjacency[component[0]]:
            selected_indices.add(component[0])
            resolution_modes[component[0]] = "no_conflict"
            continue

        largest_conflict_component_size = max(
            largest_conflict_component_size, component_size
        )
        if component_size > MAX_EXACT_OWNERSHIP_CONFLICT_COMPONENT_SIZE:
            selected_indices.update(component)
            overload_component_count += 1
            overload_component_sizes.append(component_size)
            mode = "overload_passthrough"
            component_selected = tuple(component)
            subsets_evaluated = 0
        else:
            exact_component_count += 1
            mode = "exact"
            ordered_component = [candidate_by_index[index] for index in component]
            best_indices: Optional[tuple[int, ...]] = None
            best_confidence = float("-inf")
            subsets_evaluated = 1 << component_size
            exact_subsets_evaluated += subsets_evaluated
            for mask in range(subsets_evaluated):
                indices = tuple(
                    ordered_component[index]["candidate_index"]
                    for index in range(component_size)
                    if mask & (1 << index)
                )
                if any(
                    frozenset(pair) in conflict_pairs
                    for pair in itertools.combinations(indices, 2)
                ):
                    continue
                confidence = sum(
                    candidate_by_index[index]["yolo_confidence"]
                    for index in indices
                )
                if (
                    best_indices is None
                    or len(indices) > len(best_indices)
                    or (
                        len(indices) == len(best_indices)
                        and confidence > best_confidence
                    )
                    or (
                        len(indices) == len(best_indices)
                        and confidence == best_confidence
                        and indices < best_indices
                    )
                ):
                    best_indices = indices
                    best_confidence = confidence
            if best_indices is None:
                raise RuntimeError(
                    "light ownership component found no valid subset"
                )
            component_selected = best_indices
            selected_indices.update(best_indices)

        for candidate_index in component:
            resolution_modes[candidate_index] = mode
        conflict_components.append(
            {
                "candidate_indices": component,
                "size": component_size,
                "resolution_mode": mode,
                "selected_candidate_indices": list(component_selected),
                "subsets_evaluated": subsets_evaluated,
            }
        )

    selected_candidate_indices = tuple(sorted(selected_indices))
    return {
        "selected_candidate_indices": selected_candidate_indices,
        "selected_yolo_confidence_sum": sum(
            candidate_by_index[index]["yolo_confidence"]
            for index in selected_candidate_indices
        ),
        "candidate_light_cluster_ids": candidate_clusters,
        "candidate_resolution_modes": resolution_modes,
        "candidate_conflict_component_sizes": component_sizes,
        "physical_light_clusters": clusters,
        "conflicts": conflicts,
        "conflict_components": conflict_components,
        "largest_conflict_component_size": largest_conflict_component_size,
        "exact_component_count": exact_component_count,
        "overload_component_count": overload_component_count,
        "overload_component_sizes": overload_component_sizes,
        "exact_subsets_evaluated": exact_subsets_evaluated,
    }


def apply_light_ownership(results: list[dict], confidences: np.ndarray) -> dict:
    confidence_values = np.asarray(confidences, dtype=np.float64).reshape(-1)
    if len(confidence_values) != len(results):
        raise ValueError(
            "confidence/result count mismatch: "
            f"{len(confidence_values)} != {len(results)}"
        )
    ownership = select_light_ownership(
        [
            {
                "candidate_index": result["candidate_index"],
                "yolo_confidence": float(confidence),
                "pair": result["pair"],
                "output_valid": bool(result["output_valid"]),
            }
            for result, confidence in zip(results, confidence_values)
        ]
    )
    selected = set(ownership["selected_candidate_indices"])
    conflict_map = defaultdict(list)
    for conflict in ownership["conflicts"]:
        first = conflict["first_candidate_index"]
        second = conflict["second_candidate_index"]
        conflict_map[first].append(second)
        conflict_map[second].append(first)

    for result, confidence in zip(results, confidence_values):
        candidate_index = result["candidate_index"]
        valid_before = bool(result["output_valid"])
        result["yolo_confidence"] = float(confidence)
        result["output_valid_before_ownership"] = valid_before
        result["ownership_selected"] = bool(
            valid_before and candidate_index in selected
        )
        result["ownership_light_cluster_ids"] = ownership[
            "candidate_light_cluster_ids"
        ].get(candidate_index)
        result["ownership_conflicting_candidate_indices"] = sorted(
            conflict_map[candidate_index]
        )
        result["ownership_resolution_mode"] = ownership[
            "candidate_resolution_modes"
        ].get(candidate_index, "not_applicable")
        result["ownership_overload_passthrough"] = (
            result["ownership_resolution_mode"] == "overload_passthrough"
        )
        result["ownership_conflict_component_size"] = ownership[
            "candidate_conflict_component_sizes"
        ].get(candidate_index, 0)
        if valid_before and candidate_index not in selected:
            result["output_valid"] = False
            result["failure_reason"] = "light_ownership_conflict"
    return ownership


class CausalTypeLatch:
    """Associate only with the previous frame and lock type once initialized."""

    def __init__(self) -> None:
        self.active: dict[int, TrackState] = {}
        self.next_track_id = 1

    def reset(self) -> None:
        self.active = {}
        self.next_track_id = 1

    def terminate_active(self) -> None:
        self.active = {}

    def assign(
        self, boxes: list[np.ndarray], frame_index: int
    ) -> tuple[list[tuple[TrackState, Optional[float], bool]], list[int]]:
        possible = []
        for track_id, track in self.active.items():
            for candidate_index, box in enumerate(boxes):
                iou = box_iou(track.last_box, box)
                if iou >= TRACK_IOU_THRESHOLD:
                    possible.append((-iou, track_id, candidate_index))
        possible.sort()
        candidate_matches = {}
        used_tracks = set()
        for negative_iou, track_id, candidate_index in possible:
            if track_id in used_tracks or candidate_index in candidate_matches:
                continue
            candidate_matches[candidate_index] = (
                self.active[track_id],
                -negative_iou,
            )
            used_tracks.add(track_id)

        previous_track_ids = set(self.active)
        next_active = {}
        assignments = []
        for candidate_index, box in enumerate(boxes):
            if candidate_index in candidate_matches:
                track, match_iou = candidate_matches[candidate_index]
                created = False
            else:
                track = TrackState(
                    track_id=self.next_track_id,
                    start_frame=frame_index,
                    last_frame=frame_index,
                    last_box=np.asarray(box, dtype=np.float64),
                )
                self.next_track_id += 1
                match_iou = None
                created = True
            track.last_box = np.asarray(box, dtype=np.float64)
            track.last_frame = frame_index
            next_active[track.track_id] = track
            assignments.append((track, match_iou, created))
        ended_track_ids = sorted(previous_track_ids - set(next_active))
        self.active = next_active
        return assignments, ended_track_ids

    @staticmethod
    def initialize(track: TrackState, ratio: float, frame_index: int) -> bool:
        initialized = CausalTypeLatch.initialize_type(
            track,
            "small" if ratio < TYPE_BOUNDARY else "large",
            frame_index,
            "raw_ratio",
            ratio,
        )
        if initialized:
            track.initialization_ratio = float(ratio)
        return initialized

    @staticmethod
    def initialize_type(
        track: TrackState,
        armor_type: str,
        frame_index: int,
        metric: str,
        value: float,
    ) -> bool:
        if track.latched_type is not None:
            return False
        if armor_type not in LIGHT_GEOMETRY_M:
            raise ValueError(f"unsupported armor type: {armor_type}")
        track.latched_type = armor_type
        track.initialization_frame = frame_index
        track.initialization_metric = metric
        track.initialization_value = float(value)
        return True


class RawTrackLatchProcessor:
    def __init__(
        self,
        type_initialization_strategy: str = "raw_ratio",
        gray_threshold: int = GRAY_THRESHOLD,
    ) -> None:
        if type_initialization_strategy not in {"raw_ratio", "normalized_dual_pnp"}:
            raise ValueError(
                "type_initialization_strategy must be 'raw_ratio' or "
                "'normalized_dual_pnp'"
            )
        self.type_initialization_strategy = type_initialization_strategy
        self.gray_threshold = validate_gray_threshold(gray_threshold)
        self.tracker = CausalTypeLatch()
        self.last_ownership_result: Optional[dict] = None

    def reset(self) -> None:
        self.tracker.reset()
        self.last_ownership_result = None

    def terminate_active(self) -> None:
        self.tracker.terminate_active()

    def process_frame(
        self,
        image: np.ndarray,
        boxes: np.ndarray,
        frame_index: int,
        camera_matrix: Optional[np.ndarray],
        distortion: Optional[np.ndarray],
        confidences: Optional[np.ndarray] = None,
    ) -> list[dict]:
        normalized_boxes = [
            np.asarray(box, dtype=np.float64) for box in np.asarray(boxes).reshape(-1, 4)
        ]
        assignments, ended_track_ids = self.tracker.assign(
            normalized_boxes, frame_index
        )
        results = []
        for candidate_index, (box, assignment) in enumerate(
            zip(normalized_boxes, assignments)
        ):
            track, match_iou, created = assignment
            extraction = extract_light_pair(image, box, self.gray_threshold)
            pair = extraction["pair"]
            raw_ratio = (
                float(pair["normalized_center_distance"]) if pair is not None else None
            )
            raw_type = pair["raw_type"] if pair is not None else None
            initialized = False
            normalized_dual_pnp = None
            pnp = None
            failure_reason = extraction["failure_reason"]
            if pair is not None:
                if (
                    track.latched_type is None
                    and self.type_initialization_strategy == "raw_ratio"
                ):
                    initialized = self.tracker.initialize(
                        track, raw_ratio, frame_index
                    )
                elif (
                    track.latched_type is None
                    and self.type_initialization_strategy == "normalized_dual_pnp"
                ):
                    image_height, image_width = image.shape[:2]
                    if not points_in_image_bounds(
                        pair["points_xy"], image_width, image_height
                    ):
                        failure_reason = (
                            "type_initialization_points_out_of_bounds"
                        )
                    elif camera_matrix is None or distortion is None:
                        failure_reason = "camera_info_unavailable"
                    else:
                        normalized_dual_pnp = solve_normalized_dual_pnp_type(
                            pair["points_xy"], camera_matrix, distortion
                        )
                        if normalized_dual_pnp["evaluable"]:
                            initialized = self.tracker.initialize_type(
                                track,
                                normalized_dual_pnp["type"],
                                frame_index,
                                "normalized_dual_pnp",
                                normalized_dual_pnp["score"],
                            )
                            if initialized:
                                track.initialization_ratio = raw_ratio
                            failure_reason = ""
                        else:
                            failure_reason = (
                                "normalized_dual_pnp_unavailable:"
                                f"{normalized_dual_pnp['failure_reason']}"
                            )

                if track.latched_type is not None:
                    if camera_matrix is None or distortion is None:
                        failure_reason = "camera_info_unavailable"
                    else:
                        if normalized_dual_pnp is not None and initialized:
                            pnp = normalized_dual_pnp[
                                f"{track.latched_type}_pnp"
                            ]
                        else:
                            pnp = solve_light_pnp(
                                pair["points_xy"],
                                track.latched_type,
                                camera_matrix,
                                distortion,
                            )
                        if not pnp["valid"]:
                            failure_reason = pnp["failure_reason"]
                        else:
                            failure_reason = ""
                elif not failure_reason:
                    failure_reason = "type_uninitialized"
            results.append(
                {
                    "candidate_index": candidate_index,
                    "box_xyxy": box,
                    "track_id": track.track_id,
                    "track_created": created,
                    "track_match_iou": match_iou,
                    "ended_track_ids": ended_track_ids,
                    "latched_type": track.latched_type,
                    "initialization_frame": track.initialization_frame,
                    "initialization_ratio": track.initialization_ratio,
                    "initialization_metric": track.initialization_metric,
                    "initialization_value": track.initialization_value,
                    "initialized_this_frame": initialized,
                    "raw_ratio": raw_ratio,
                    "raw_type": raw_type,
                    "normalized_dual_pnp": normalized_dual_pnp,
                    "normalized_dual_pnp_score": (
                        normalized_dual_pnp["score"]
                        if normalized_dual_pnp is not None
                        else None
                    ),
                    "extraction": extraction,
                    "pair": pair,
                    "pnp": pnp,
                    "output_valid": bool(pnp and pnp["valid"]),
                    "failure_reason": failure_reason,
                }
            )
        if confidences is None:
            confidences = np.zeros(len(results), dtype=np.float64)
        self.last_ownership_result = apply_light_ownership(results, confidences)
        return results
