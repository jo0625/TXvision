#!/usr/bin/env python3
"""Build the fixed visual audit for parity-v2 unmatched candidates.

This script only joins saved tabular results and decodes preregistered video
frames. It does not run YOLO, light extraction, or PnP.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
from collections import Counter, defaultdict
from pathlib import Path

import cv2
import numpy as np


ROOT = Path("/home/jo/my-project")
RM_AUTO_AIM = ROOT / "rm_vision/rm_auto_aim"
DATA = ROOT / "my_first_autoaim_yolo/data/team_exchange/real_blank_near_20260812"
CURRENT = DATA / "type_switch_validation_02"
VIDEO = CURRENT / "type_switch_validation_02.avi"
UNMATCHED = (
    RM_AUTO_AIM
    / "real_armor_diagnostics/raw_track_latch_runtime_parity_v2/unmatched_candidates.csv"
)
YOLO_REPLAY = (
    CURRENT / "yolo_outer_projective_type_v1/current_yolo_outer_replay.csv"
)
TRUTH = CURRENT / "temporal_type_eval_v1/visibility_truth.csv"
# This existing result is used only because it already contains the selected
# traditional endpoints. Recomputing those endpoints is prohibited here.
SAVED_ENDPOINTS = CURRENT / "temporal_type_eval_v1/frame_results.csv"
OUTPUT = (
    RM_AUTO_AIM
    / "real_armor_diagnostics/raw_track_latch_unmatched_audit_v1"
)

CLASSES = {
    "SAME_TARGET_DUPLICATE",
    "VALID_UNLABELED_ARMOR",
    "NON_ARMOR_FALSE_POSITIVE",
    "AMBIGUOUS",
}

COLORS = {
    "candidate": (0, 140, 255),
    "keypoint": (0, 255, 255),
    "light": (0, 255, 0),
    "small": (255, 160, 0),
    "large": (255, 0, 255),
}


def read_csv(path: Path) -> list[dict]:
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def write_csv(path: Path, fields: list[str], rows: list[dict]) -> None:
    with path.open("x", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def write_json(path: Path, value: object) -> None:
    with path.open("x", encoding="utf-8") as stream:
        json.dump(value, stream, indent=2, ensure_ascii=True)
        stream.write("\n")


def percentile(values: list[float], quantile: float) -> float | None:
    if not values:
        return None
    return float(np.percentile(np.asarray(values, dtype=np.float64), quantile))


def distribution(values: list[float]) -> dict:
    return {
        "count": len(values),
        "min": min(values) if values else None,
        "p10": percentile(values, 10),
        "median": statistics.median(values) if values else None,
        "p90": percentile(values, 90),
        "max": max(values) if values else None,
    }


def decode_json(value: str):
    return json.loads(value) if value else None


def candidate_center(box: list[float]) -> tuple[float, float]:
    return ((box[0] + box[2]) / 2.0, (box[1] + box[3]) / 2.0)


def euclidean(first: tuple[float, float], second: tuple[float, float]) -> float:
    return math.hypot(first[0] - second[0], first[1] - second[1])


def load_inputs() -> tuple[list[dict], dict, dict, dict]:
    unmatched_rows = read_csv(UNMATCHED)
    replay_rows = read_csv(YOLO_REPLAY)
    truth_rows = read_csv(TRUTH)
    temporal_rows = read_csv(SAVED_ENDPOINTS)

    replay = {
        (int(row["frame_index"]), int(row["new_candidate_index"])): row
        for row in replay_rows
    }
    truth = {int(row["frame_index"]): row for row in truth_rows}
    endpoint_details = {}
    for row in temporal_rows:
        frame_index = int(row["frame_index"])
        for detail in json.loads(row["candidate_details_json"]):
            endpoint_details[(frame_index, int(detail["candidate_index"]))] = detail

    normalized = []
    missing_replay = []
    missing_endpoints = []
    ratio_differences = []
    for source in unmatched_rows:
        row = dict(source)
        row["frame_index"] = int(row["frame_index"])
        row["candidate_index"] = int(row["candidate_index"])
        row["track_id"] = int(row["track_id"])
        key = (row["frame_index"], row["candidate_index"])
        yolo = replay.get(key)
        endpoint = endpoint_details.get(key)
        if yolo is None:
            missing_replay.append(key)
            continue
        if endpoint is None or endpoint.get("selected_pair") is None:
            missing_endpoints.append(key)
            continue
        row["box"] = decode_json(row["box_xyxy_json"])
        row["yolo_confidence"] = float(yolo["new_yolo_confidence"])
        row["keypoints"] = decode_json(yolo["keypoints_xy_json"])
        row["keypoint_confidences"] = decode_json(yolo["keypoint_confidence_json"])
        row["selected_pair"] = endpoint["selected_pair"]
        ratio_differences.append(
            abs(
                float(row["raw_ratio"])
                - float(endpoint["selected_pair"]["normalized_center_distance"])
            )
        )
        normalized.append(row)

    consistency = {
        "unmatched_candidate_count": len(unmatched_rows),
        "joined_candidate_count": len(normalized),
        "missing_yolo_replay_keys": missing_replay,
        "missing_saved_endpoint_keys": missing_endpoints,
        "saved_endpoint_source": str(SAVED_ENDPOINTS),
        "endpoint_recomputed": False,
        "pnp_recomputed": False,
        "yolo_recomputed": False,
        "raw_ratio_absolute_difference": distribution(ratio_differences),
        "raw_ratio_difference_over_1e_5_count": sum(
            value > 1e-5 for value in ratio_differences
        ),
    }
    if missing_replay or missing_endpoints or len(normalized) != 192:
        raise RuntimeError(f"input join mismatch: {consistency}")
    return normalized, replay, truth, consistency


def manifest_rows(rows: list[dict]) -> list[dict]:
    by_track = defaultdict(list)
    for row in rows:
        by_track[row["track_id"]].append(row)

    manifest = []
    for track_id, track_rows in sorted(by_track.items()):
        ordered = sorted(
            track_rows, key=lambda row: (row["frame_index"], row["candidate_index"])
        )
        selections = defaultdict(list)
        highest = sorted(
            ordered,
            key=lambda row: (
                -row["yolo_confidence"],
                row["frame_index"],
                row["candidate_index"],
            ),
        )[0]
        selections[(highest["frame_index"], highest["candidate_index"])].append(
            "highest_confidence"
        )
        if len(ordered) >= 5:
            for role, row in (
                ("first", ordered[0]),
                ("middle", ordered[(len(ordered) - 1) // 2]),
                ("last", ordered[-1]),
            ):
                selections[(row["frame_index"], row["candidate_index"])].append(role)
        indexed = {
            (row["frame_index"], row["candidate_index"]): row for row in ordered
        }
        for (frame_index, candidate_index), roles in sorted(selections.items()):
            row = indexed[(frame_index, candidate_index)]
            stem = (
                f"track_{track_id:03d}_frame_{frame_index:04d}_"
                f"candidate_{candidate_index:02d}"
            )
            manifest.append(
                {
                    "track_id": track_id,
                    "frame_index": frame_index,
                    "candidate_index": candidate_index,
                    "selection_roles": "+".join(roles),
                    "track_observation_count": len(ordered),
                    "yolo_confidence": row["yolo_confidence"],
                    "raw_full_frame": f"representative_frames/raw_full/{stem}.png",
                    "overlay_full_frame": f"representative_frames/overlay_full/{stem}.png",
                    "raw_local_crop": f"representative_frames/raw_crops/{stem}.png",
                    "overlay_local_crop": f"representative_frames/overlay_crops/{stem}.png",
                }
            )
    return manifest


def truth_box_rows(replay: dict, frame_index: int) -> list[dict]:
    result = []
    for (candidate_frame, _), row in replay.items():
        if candidate_frame != frame_index or not row["old_truth_type"]:
            continue
        result.append(
            {
                "truth_type": row["old_truth_type"],
                "box": decode_json(row["new_box_xyxy_json"]),
            }
        )
    return result


def draw_box(image: np.ndarray, box: list[float], color: tuple, thickness: int) -> None:
    x1, y1, x2, y2 = (int(round(value)) for value in box)
    cv2.rectangle(image, (x1, y1), (x2, y2), color, thickness, cv2.LINE_AA)


def draw_cross(
    image: np.ndarray, point: tuple[float, float], color: tuple, radius: int = 9
) -> None:
    x, y = (int(round(value)) for value in point)
    cv2.line(image, (x - radius, y), (x + radius, y), color, 2, cv2.LINE_AA)
    cv2.line(image, (x, y - radius), (x, y + radius), color, 2, cv2.LINE_AA)


def annotate(
    image: np.ndarray, row: dict, replay: dict, truth: dict
) -> np.ndarray:
    annotated = image.copy()
    draw_box(annotated, row["box"], COLORS["candidate"], 4)
    for index, point in enumerate(row["keypoints"]):
        center = tuple(int(round(value)) for value in point)
        cv2.circle(annotated, center, 6, COLORS["keypoint"], -1, cv2.LINE_AA)
        cv2.putText(
            annotated,
            f"K{index}",
            (center[0] + 7, center[1] - 7),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            COLORS["keypoint"],
            2,
            cv2.LINE_AA,
        )
    light_points = row["selected_pair"]["points_xy"]
    for index, point in enumerate(light_points):
        center = tuple(int(round(value)) for value in point)
        cv2.circle(annotated, center, 6, COLORS["light"], -1, cv2.LINE_AA)
        cv2.putText(
            annotated,
            f"L{index}",
            (center[0] + 7, center[1] + 17),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            COLORS["light"],
            2,
            cv2.LINE_AA,
        )
    for first, second in ((0, 1), (3, 2)):
        p1 = tuple(int(round(value)) for value in light_points[first])
        p2 = tuple(int(round(value)) for value in light_points[second])
        cv2.line(annotated, p1, p2, COLORS["light"], 2, cv2.LINE_AA)

    for item in truth_box_rows(replay, row["frame_index"]):
        color = COLORS[item["truth_type"]]
        draw_box(annotated, item["box"], color, 2)
        x1, y1 = (int(round(value)) for value in item["box"][:2])
        cv2.putText(
            annotated,
            f"truth {item['truth_type']}",
            (x1, max(22, y1 - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            color,
            2,
            cv2.LINE_AA,
        )
    truth_row = truth[row["frame_index"]]
    for truth_type in ("small", "large"):
        x_value = truth_row[f"{truth_type}_center_x"]
        y_value = truth_row[f"{truth_type}_center_y"]
        if x_value and y_value:
            draw_cross(
                annotated,
                (float(x_value), float(y_value)),
                COLORS[truth_type],
            )

    lines = [
        (
            f"track {row['track_id']} frame {row['frame_index']} "
            f"cand {row['candidate_index']} conf {row['yolo_confidence']:.3f}"
        ),
        (
            f"raw {float(row['raw_ratio']):.3f} latch {row['latched_type']} "
            f"dist {float(row['pnp_distance_m']):.3f}m "
            f"reproj {float(row['pnp_mean_reprojection_error_px']):.3f}px"
        ),
        "orange candidate | yellow YOLO corners | green light endpoints",
    ]
    for line_index, text in enumerate(lines):
        y = 32 + line_index * 30
        cv2.putText(
            annotated,
            text,
            (20, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 0, 0),
            4,
            cv2.LINE_AA,
        )
        cv2.putText(
            annotated,
            text,
            (20, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )
    return annotated


def crop_box(image: np.ndarray, box: list[float], margin_fraction: float = 0.05):
    height, width = image.shape[:2]
    x1, y1, x2, y2 = box
    margin_x = (x2 - x1) * margin_fraction
    margin_y = (y2 - y1) * margin_fraction
    left = max(0, int(math.floor(x1 - margin_x)))
    top = max(0, int(math.floor(y1 - margin_y)))
    right = min(width, int(math.ceil(x2 + margin_x)))
    bottom = min(height, int(math.ceil(y2 + margin_y)))
    return image[top:bottom, left:right].copy()


def fit_tile(image: np.ndarray, width: int, height: int, label: str) -> np.ndarray:
    canvas = np.full((height, width, 3), 32, dtype=np.uint8)
    content_height = height - 34
    scale = min(width / image.shape[1], content_height / image.shape[0])
    resized = cv2.resize(
        image,
        (
            max(1, int(round(image.shape[1] * scale))),
            max(1, int(round(image.shape[0] * scale))),
        ),
        interpolation=cv2.INTER_AREA,
    )
    x = (width - resized.shape[1]) // 2
    y = 34 + (content_height - resized.shape[0]) // 2
    canvas[y : y + resized.shape[0], x : x + resized.shape[1]] = resized
    cv2.putText(
        canvas,
        label,
        (8, 23),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (255, 255, 255),
        1,
        cv2.LINE_AA,
    )
    return canvas


def write_contact_pages(
    entries: list[tuple[dict, np.ndarray]], prefix: str, columns: int, rows: int
) -> list[str]:
    page_size = columns * rows
    names = []
    for page_index in range(0, len(entries), page_size):
        page_entries = entries[page_index : page_index + page_size]
        tiles = []
        for manifest, image in page_entries:
            label = (
                f"T{int(manifest['track_id']):03d} F{int(manifest['frame_index']):04d} "
                f"{manifest['selection_roles']}"
            )
            tiles.append(fit_tile(image, 440, 330, label))
        blank = np.full_like(tiles[0], 32)
        while len(tiles) < page_size:
            tiles.append(blank.copy())
        page_rows = [
            np.hstack(tiles[index : index + columns])
            for index in range(0, page_size, columns)
        ]
        page = np.vstack(page_rows)
        name = f"contact_sheets/{prefix}_{page_index // page_size + 1:02d}.png"
        cv2.imwrite(str(OUTPUT / name), page)
        names.append(name)
    return names


def build_track_statistics(rows: list[dict], truth: dict) -> list[dict]:
    by_track = defaultdict(list)
    all_centers = [candidate_center(row["box"]) for row in rows]
    global_center = (
        statistics.median(point[0] for point in all_centers),
        statistics.median(point[1] for point in all_centers),
    )
    for row in rows:
        by_track[row["track_id"]].append(row)

    result = []
    for track_id, track_rows in sorted(by_track.items()):
        ordered = sorted(track_rows, key=lambda row: row["frame_index"])
        frames = [row["frame_index"] for row in ordered]
        gaps = [
            {"after_frame": first, "before_frame": second, "missing_frames": second - first - 1}
            for first, second in zip(frames, frames[1:])
            if second > first + 1
        ]
        centers = [candidate_center(row["box"]) for row in ordered]
        widths = [row["box"][2] - row["box"][0] for row in ordered]
        heights = [row["box"][3] - row["box"][1] for row in ordered]
        small_distances = []
        large_distances = []
        for row, center in zip(ordered, centers):
            truth_row = truth[row["frame_index"]]
            for truth_type, values in (
                ("small", small_distances),
                ("large", large_distances),
            ):
                x_value = truth_row[f"{truth_type}_center_x"]
                y_value = truth_row[f"{truth_type}_center_y"]
                if x_value and y_value:
                    values.append(
                        euclidean(center, (float(x_value), float(y_value)))
                    )
        median_center = (
            statistics.median(point[0] for point in centers),
            statistics.median(point[1] for point in centers),
        )
        keypoint_confidences = [
            value for row in ordered for value in row["keypoint_confidences"]
        ]
        result.append(
            {
                "track_id": track_id,
                "start_frame": frames[0],
                "end_frame": frames[-1],
                "observation_count": len(ordered),
                "span_frames": frames[-1] - frames[0] + 1,
                "internal_gap_count": len(gaps),
                "internal_gaps_json": json.dumps(gaps, separators=(",", ":")),
                "center_x_min": min(point[0] for point in centers),
                "center_x_median": median_center[0],
                "center_x_max": max(point[0] for point in centers),
                "center_y_min": min(point[1] for point in centers),
                "center_y_median": median_center[1],
                "center_y_max": max(point[1] for point in centers),
                "box_width_min": min(widths),
                "box_width_median": statistics.median(widths),
                "box_width_max": max(widths),
                "box_height_min": min(heights),
                "box_height_median": statistics.median(heights),
                "box_height_max": max(heights),
                "yolo_confidence_min": min(row["yolo_confidence"] for row in ordered),
                "yolo_confidence_median": statistics.median(
                    row["yolo_confidence"] for row in ordered
                ),
                "yolo_confidence_max": max(row["yolo_confidence"] for row in ordered),
                "keypoint_confidence_min": min(keypoint_confidences),
                "keypoint_confidence_median": statistics.median(keypoint_confidences),
                "keypoint_confidence_max": max(keypoint_confidences),
                "raw_ratio_min": min(float(row["raw_ratio"]) for row in ordered),
                "raw_ratio_median": statistics.median(
                    float(row["raw_ratio"]) for row in ordered
                ),
                "raw_ratio_max": max(float(row["raw_ratio"]) for row in ordered),
                "latched_types": "+".join(
                    sorted({row["latched_type"] for row in ordered})
                ),
                "pnp_distance_m_min": min(
                    float(row["pnp_distance_m"]) for row in ordered
                ),
                "pnp_distance_m_median": statistics.median(
                    float(row["pnp_distance_m"]) for row in ordered
                ),
                "pnp_distance_m_max": max(
                    float(row["pnp_distance_m"]) for row in ordered
                ),
                "reprojection_error_px_min": min(
                    float(row["pnp_mean_reprojection_error_px"]) for row in ordered
                ),
                "reprojection_error_px_median": statistics.median(
                    float(row["pnp_mean_reprojection_error_px"]) for row in ordered
                ),
                "reprojection_error_px_max": max(
                    float(row["pnp_mean_reprojection_error_px"]) for row in ordered
                ),
                "small_truth_center_distance_px_min": min(small_distances),
                "small_truth_center_distance_px_median": statistics.median(
                    small_distances
                ),
                "large_truth_center_distance_px_min": min(large_distances),
                "large_truth_center_distance_px_median": statistics.median(
                    large_distances
                ),
                "distance_to_global_unmatched_center_px": euclidean(
                    median_center, global_center
                ),
                "same_dominant_spatial_cluster_50px": euclidean(
                    median_center, global_center
                )
                <= 50.0,
            }
        )
    return result


def prepare() -> None:
    OUTPUT.mkdir(parents=True, exist_ok=False)
    for relative in (
        "representative_frames/raw_full",
        "representative_frames/overlay_full",
        "representative_frames/raw_crops",
        "representative_frames/overlay_crops",
        "contact_sheets",
    ):
        (OUTPUT / relative).mkdir(parents=True)

    rows, replay, truth, consistency = load_inputs()
    manifest = manifest_rows(rows)
    write_csv(OUTPUT / "manifest.csv", list(manifest[0]), manifest)
    write_json(OUTPUT / "source_consistency.json", consistency)

    row_index = {
        (row["track_id"], row["frame_index"], row["candidate_index"]): row
        for row in rows
    }
    selected_frames = {int(row["frame_index"]) for row in manifest}
    decoded = {}
    capture = cv2.VideoCapture(str(VIDEO))
    if not capture.isOpened():
        raise RuntimeError(f"cannot open {VIDEO}")
    frame_index = 0
    while selected_frames:
        ok, image = capture.read()
        if not ok or image is None:
            break
        if frame_index in selected_frames:
            decoded[frame_index] = image.copy()
            selected_frames.remove(frame_index)
        frame_index += 1
    capture.release()
    if selected_frames:
        raise RuntimeError(f"failed to decode frames: {sorted(selected_frames)}")

    full_entries = []
    crop_entries = []
    sample_rows = []
    for item in manifest:
        key = (
            int(item["track_id"]),
            int(item["frame_index"]),
            int(item["candidate_index"]),
        )
        row = row_index[key]
        raw = decoded[row["frame_index"]]
        overlay = annotate(raw, row, replay, truth)
        raw_crop = crop_box(raw, row["box"])
        overlay_crop = crop_box(overlay, row["box"])
        cv2.imwrite(str(OUTPUT / item["raw_full_frame"]), raw)
        cv2.imwrite(str(OUTPUT / item["overlay_full_frame"]), overlay)
        cv2.imwrite(str(OUTPUT / item["raw_local_crop"]), raw_crop)
        cv2.imwrite(str(OUTPUT / item["overlay_local_crop"]), overlay_crop)
        full_entries.append((item, overlay))
        crop_entries.append((item, overlay_crop))

        center = candidate_center(row["box"])
        truth_row = truth[row["frame_index"]]
        sample_rows.append(
            {
                **{key: item[key] for key in item},
                "box_xyxy_json": json.dumps(row["box"], separators=(",", ":")),
                "keypoints_xy_json": json.dumps(
                    row["keypoints"], separators=(",", ":")
                ),
                "keypoint_confidences_json": json.dumps(
                    row["keypoint_confidences"], separators=(",", ":")
                ),
                "traditional_light_points_xy_json": json.dumps(
                    row["selected_pair"]["points_xy"], separators=(",", ":")
                ),
                "candidate_center_x": center[0],
                "candidate_center_y": center[1],
                "small_truth_center_distance_px": euclidean(
                    center,
                    (
                        float(truth_row["small_center_x"]),
                        float(truth_row["small_center_y"]),
                    ),
                ),
                "large_truth_center_distance_px": euclidean(
                    center,
                    (
                        float(truth_row["large_center_x"]),
                        float(truth_row["large_center_y"]),
                    ),
                ),
                "raw_ratio": row["raw_ratio"],
                "latched_type": row["latched_type"],
                "pnp_distance_m": row["pnp_distance_m"],
                "pnp_mean_reprojection_error_px": row[
                    "pnp_mean_reprojection_error_px"
                ],
            }
        )

    write_csv(OUTPUT / "sample_metrics.csv", list(sample_rows[0]), sample_rows)
    full_pages = write_contact_pages(full_entries, "full", columns=4, rows=3)
    crop_pages = write_contact_pages(crop_entries, "crops", columns=4, rows=3)

    highest_crop_entries = [
        entry
        for entry in crop_entries
        if "highest_confidence" in entry[0]["selection_roles"]
    ]
    overview_pages = write_contact_pages(
        highest_crop_entries, "track_overview", columns=6, rows=4
    )

    track_statistics = build_track_statistics(rows, truth)
    write_csv(
        OUTPUT / "track_statistics.csv",
        list(track_statistics[0]),
        track_statistics,
    )
    write_json(
        OUTPUT / "prepare_summary.json",
        {
            "experiment": "raw_track_latch_unmatched_audit_v1",
            "selection_frozen": True,
            "selection_rule": {
                "all_tracks": "highest YOLO confidence; ties use earliest frame and candidate index",
                "tracks_with_at_least_5_observations": "also first, middle, and last observation",
                "duplicate_selected_observations": "roles merged; frame not duplicated",
            },
            "track_count": len(track_statistics),
            "candidate_count": len(rows),
            "manifest_sample_count": len(manifest),
            "long_track_count": sum(
                int(row["observation_count"]) >= 5 for row in track_statistics
            ),
            "contact_sheets": {
                "full": full_pages,
                "crops": crop_pages,
                "highest_confidence_track_overview": overview_pages,
            },
            "source_consistency": consistency,
        },
    )
    print(
        json.dumps(
            {
                "track_count": len(track_statistics),
                "candidate_count": len(rows),
                "manifest_sample_count": len(manifest),
                "full_contact_pages": len(full_pages),
                "crop_contact_pages": len(crop_pages),
                "overview_pages": len(overview_pages),
            },
            indent=2,
        )
    )


def fmt(value: float | None, digits: int = 3) -> str:
    return "n/a" if value is None else f"{value:.{digits}f}"


def finalize() -> None:
    classification_path = OUTPUT / "track_classifications.csv"
    classifications = read_csv(classification_path)
    track_statistics = read_csv(OUTPUT / "track_statistics.csv")
    sample_metrics = read_csv(OUTPUT / "sample_metrics.csv")
    prepare_summary = json.loads(
        (OUTPUT / "prepare_summary.json").read_text(encoding="utf-8")
    )
    source_consistency = json.loads(
        (OUTPUT / "source_consistency.json").read_text(encoding="utf-8")
    )
    by_track = {int(row["track_id"]): row for row in classifications}
    expected_tracks = {int(row["track_id"]) for row in track_statistics}
    if set(by_track) != expected_tracks:
        raise RuntimeError(
            f"classification track mismatch: got {sorted(by_track)}, "
            f"expected {sorted(expected_tracks)}"
        )
    invalid = {
        row["classification"] for row in classifications
        if row["classification"] not in CLASSES
    }
    if invalid:
        raise RuntimeError(f"invalid classifications: {sorted(invalid)}")

    counts = Counter(row["classification"] for row in classifications)
    if len(counts) == 1 and not counts.get("AMBIGUOUS"):
        only = next(iter(counts))
        decision = {
            "SAME_TARGET_DUPLICATE": "UNMATCHED_OUTPUTS_ARE_DUPLICATE_TARGET_DETECTIONS",
            "VALID_UNLABELED_ARMOR": "UNMATCHED_OUTPUTS_ARE_VALID_UNLABELED_ARMOR",
            "NON_ARMOR_FALSE_POSITIVE": "UNMATCHED_OUTPUTS_ARE_NON_ARMOR_FALSE_POSITIVES",
        }[only]
    else:
        decision = "UNMATCHED_OUTPUTS_MIXED_OR_INCONCLUSIVE"

    centers_x = [float(row["candidate_center_x"]) for row in sample_metrics]
    centers_y = [float(row["candidate_center_y"]) for row in sample_metrics]
    all_candidate_rows = read_csv(UNMATCHED)
    all_boxes = [decode_json(row["box_xyxy_json"]) for row in all_candidate_rows]
    all_centers = [candidate_center(box) for box in all_boxes]
    yolo_confidences = []
    keypoint_confidences = []
    replay = {
        (int(row["frame_index"]), int(row["new_candidate_index"])): row
        for row in read_csv(YOLO_REPLAY)
    }
    for row in all_candidate_rows:
        yolo = replay[(int(row["frame_index"]), int(row["candidate_index"]))]
        yolo_confidences.append(float(yolo["new_yolo_confidence"]))
        keypoint_confidences.extend(decode_json(yolo["keypoint_confidence_json"]))

    joined_rows, _, truth, _ = load_inputs()
    large_right_endpoint_errors = []
    small_left_endpoint_errors = []
    for row in joined_rows:
        points = row["selected_pair"]["points_xy"]
        truth_row = truth[row["frame_index"]]
        large_points = decode_json(truth_row["large_light_points_json"])
        small_points = decode_json(truth_row["small_light_points_json"])
        large_right_endpoint_errors.extend(
            (euclidean(points[0], large_points[3]), euclidean(points[1], large_points[2]))
        )
        small_left_endpoint_errors.extend(
            (euclidean(points[3], small_points[0]), euclidean(points[2], small_points[1]))
        )

    ordered_tracks = sorted(track_statistics, key=lambda row: int(row["start_frame"]))
    inter_track_gaps = [
        int(current["start_frame"]) - int(previous["end_frame"]) - 1
        for previous, current in zip(ordered_tracks, ordered_tracks[1:])
    ]

    summary = {
        "experiment": "raw_track_latch_unmatched_audit_v1",
        "classification_is_visual_audit_not_precision_or_recall": True,
        "algorithm_reexecution": {
            "yolo": False,
            "light_extraction": False,
            "pnp": False,
        },
        "coverage": {
            "candidate_count": len(all_candidate_rows),
            "track_count": len(track_statistics),
            "classified_track_count": len(classifications),
            "manifest_sample_count": len(sample_metrics),
        },
        "classification_counts": dict(sorted(counts.items())),
        "decision_code": decision,
        "visual_mechanism": {
            "finding": "cross_target_composite",
            "description": (
                "The candidate left light is the large target right light and the "
                "candidate right light is the small target left light; no third armor "
                "body is present inside the candidate."
            ),
            "large_right_to_candidate_left_endpoint_error_px": distribution(
                large_right_endpoint_errors
            ),
            "small_left_to_candidate_right_endpoint_error_px": distribution(
                small_left_endpoint_errors
            ),
        },
        "track_fragmentation": {
            "first_observation_frame": min(
                int(row["start_frame"]) for row in track_statistics
            ),
            "last_observation_frame": max(
                int(row["end_frame"]) for row in track_statistics
            ),
            "tracks_with_internal_gaps": sum(
                int(row["internal_gap_count"]) > 0 for row in track_statistics
            ),
            "inter_track_gap_frames": distribution(inter_track_gaps),
            "longest_track": max(
                (
                    {
                        "track_id": int(row["track_id"]),
                        "start_frame": int(row["start_frame"]),
                        "end_frame": int(row["end_frame"]),
                        "observation_count": int(row["observation_count"]),
                    }
                    for row in track_statistics
                ),
                key=lambda row: row["observation_count"],
            ),
        },
        "spatial_distribution": {
            "all_candidate_center_x": distribution(
                [point[0] for point in all_centers]
            ),
            "all_candidate_center_y": distribution(
                [point[1] for point in all_centers]
            ),
            "representative_center_x": distribution(centers_x),
            "representative_center_y": distribution(centers_y),
            "tracks_in_50px_dominant_cluster": sum(
                row["same_dominant_spatial_cluster_50px"] == "True"
                for row in track_statistics
            ),
        },
        "confidence_distribution": {
            "yolo": distribution(yolo_confidences),
            "keypoint": distribution(keypoint_confidences),
        },
        "raw_ratio": distribution(
            [float(row["raw_ratio"]) for row in all_candidate_rows]
        ),
        "pnp_distance_m": distribution(
            [float(row["pnp_distance_m"]) for row in all_candidate_rows]
        ),
        "pnp_mean_reprojection_error_px": distribution(
            [
                float(row["pnp_mean_reprojection_error_px"])
                for row in all_candidate_rows
            ]
        ),
        "source_consistency": source_consistency,
        "contact_sheets": prepare_summary["contact_sheets"],
    }
    write_json(OUTPUT / "summary.json", summary)

    rationale_groups = defaultdict(list)
    for row in classifications:
        rationale_groups[row["classification"]].append(row["rationale"])
    report = [
        "# Raw Track Latch Unmatched Visual Audit V1",
        "",
        "This is a fixed visual audit of saved unmatched outputs. It is not a Precision or Recall estimate.",
        "YOLO, traditional light extraction, and PnP were not rerun.",
        "",
        "## Frozen Selection",
        "",
        f"- Candidates/tracks: {len(all_candidate_rows)}/{len(track_statistics)}.",
        f"- Fixed representative samples: {len(sample_metrics)}.",
        "- Every track contributes its highest-confidence frame; tracks with at least five observations also contribute first, middle, and last observations.",
        "- Saved endpoint join: 192/192; maximum raw-ratio difference from parity v2: "
        f"{source_consistency['raw_ratio_absolute_difference']['max']:.9f}.",
        "",
        "## Classification",
        "",
        "| Category | Tracks |",
        "|---|---:|",
    ]
    for category in sorted(CLASSES):
        report.append(f"| `{category}` | {counts.get(category, 0)} |")
    report.extend(
        [
            "",
            "## Visual Mechanism",
            "",
            "- All 42 tracks bridge the two real targets: candidate left light = large right light; candidate right light = small left light.",
            "- No third armor body is visible inside the orange candidate box.",
            (
                "- Large-right to candidate-left endpoint error px min/median/max: "
                f"{fmt(summary['visual_mechanism']['large_right_to_candidate_left_endpoint_error_px']['min'])}/"
                f"{fmt(summary['visual_mechanism']['large_right_to_candidate_left_endpoint_error_px']['median'])}/"
                f"{fmt(summary['visual_mechanism']['large_right_to_candidate_left_endpoint_error_px']['max'])}."
            ),
            (
                "- Small-left to candidate-right endpoint error px min/median/max: "
                f"{fmt(summary['visual_mechanism']['small_left_to_candidate_right_endpoint_error_px']['min'])}/"
                f"{fmt(summary['visual_mechanism']['small_left_to_candidate_right_endpoint_error_px']['median'])}/"
                f"{fmt(summary['visual_mechanism']['small_left_to_candidate_right_endpoint_error_px']['max'])}."
            ),
            "",
            "## Track Fragmentation",
            "",
            (
                f"- Observation extent: frames {summary['track_fragmentation']['first_observation_frame']}-"
                f"{summary['track_fragmentation']['last_observation_frame']}; tracks with internal gaps: "
                f"{summary['track_fragmentation']['tracks_with_internal_gaps']}."
            ),
            (
                "- Between-track interruption min/median/max frames: "
                f"{fmt(summary['track_fragmentation']['inter_track_gap_frames']['min'], 0)}/"
                f"{fmt(summary['track_fragmentation']['inter_track_gap_frames']['median'], 0)}/"
                f"{fmt(summary['track_fragmentation']['inter_track_gap_frames']['max'], 0)}."
            ),
            "",
            "## Spatial Pattern",
            "",
            (
                "- Candidate center x min/median/max: "
                f"{fmt(summary['spatial_distribution']['all_candidate_center_x']['min'])}/"
                f"{fmt(summary['spatial_distribution']['all_candidate_center_x']['median'])}/"
                f"{fmt(summary['spatial_distribution']['all_candidate_center_x']['max'])}."
            ),
            (
                "- Candidate center y min/median/max: "
                f"{fmt(summary['spatial_distribution']['all_candidate_center_y']['min'])}/"
                f"{fmt(summary['spatial_distribution']['all_candidate_center_y']['median'])}/"
                f"{fmt(summary['spatial_distribution']['all_candidate_center_y']['max'])}."
            ),
            (
                "- Track medians inside the dominant 50 px center cluster: "
                f"{summary['spatial_distribution']['tracks_in_50px_dominant_cluster']}/"
                f"{len(track_statistics)}."
            ),
            "",
            "## Saved Output Metrics",
            "",
            (
                "- YOLO confidence min/median/max: "
                f"{fmt(summary['confidence_distribution']['yolo']['min'])}/"
                f"{fmt(summary['confidence_distribution']['yolo']['median'])}/"
                f"{fmt(summary['confidence_distribution']['yolo']['max'])}."
            ),
            (
                "- Keypoint confidence min/median/max: "
                f"{fmt(summary['confidence_distribution']['keypoint']['min'])}/"
                f"{fmt(summary['confidence_distribution']['keypoint']['median'])}/"
                f"{fmt(summary['confidence_distribution']['keypoint']['max'])}."
            ),
            (
                "- Raw ratio min/median/max: "
                f"{fmt(summary['raw_ratio']['min'])}/"
                f"{fmt(summary['raw_ratio']['median'])}/"
                f"{fmt(summary['raw_ratio']['max'])}."
            ),
            (
                "- PnP distance m min/median/max: "
                f"{fmt(summary['pnp_distance_m']['min'])}/"
                f"{fmt(summary['pnp_distance_m']['median'])}/"
                f"{fmt(summary['pnp_distance_m']['max'])}."
            ),
            (
                "- Mean reprojection error px min/median/max: "
                f"{fmt(summary['pnp_mean_reprojection_error_px']['min'])}/"
                f"{fmt(summary['pnp_mean_reprojection_error_px']['median'])}/"
                f"{fmt(summary['pnp_mean_reprojection_error_px']['max'])}."
            ),
            "",
            "## Visual Evidence",
            "",
        ]
    )
    for key, paths in prepare_summary["contact_sheets"].items():
        report.append(f"- {key}: " + ", ".join(f"`{path}`" for path in paths))
    report.extend(
        [
            "",
            "Per-track ranges, gaps, geometry, confidences, PnP metrics, and truth-center distances are in `track_statistics.csv`.",
            "Per-track visual decisions and reasons are in `track_classifications.csv`.",
            "",
            "## Decision",
            "",
            f"**{decision}**",
            "",
        ]
    )
    with (OUTPUT / "report.md").open("x", encoding="utf-8") as stream:
        stream.write("\n".join(report))
    print(json.dumps({"classification_counts": counts, "decision": decision}, indent=2))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("stage", choices=("prepare", "finalize"))
    args = parser.parse_args()
    if args.stage == "prepare":
        prepare()
    else:
        finalize()


if __name__ == "__main__":
    main()
