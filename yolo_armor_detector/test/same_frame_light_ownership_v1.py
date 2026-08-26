#!/usr/bin/env python3
"""Evaluate preregistered same-frame ownership on saved YOLO candidates."""

from __future__ import annotations

import csv
import itertools
import json
import math
import sys
from collections import Counter, defaultdict
from pathlib import Path

import cv2
import numpy as np
import yaml


PACKAGE = Path(__file__).resolve().parents[1]
RM_AUTO_AIM = PACKAGE.parent
sys.path.insert(0, str(PACKAGE / "scripts"))

import raw_track_latch as runtime  # noqa: E402


DATA = Path(
    "/home/jo/my-project/my_first_autoaim_yolo/data/team_exchange/"
    "real_blank_near_20260812"
)
SWITCH = DATA / "type_switch_validation_02"
PARITY = RM_AUTO_AIM / "real_armor_diagnostics/raw_track_latch_runtime_parity_v2"
AUDIT = RM_AUTO_AIM / "real_armor_diagnostics/raw_track_latch_unmatched_audit_v1"
OUTPUT = RM_AUTO_AIM / "real_armor_diagnostics/same_frame_light_ownership_v1"
CURRENT_INPUT = SWITCH / "temporal_type_eval_v1/track_results.csv"
PRIOR_INPUT = DATA / "model_diagnostic_real_blank_v1/frame_results.csv"
CALIBRATION = RM_AUTO_AIM / "hik_camera/config/hikrobot_mv_cs016_10uc.yaml"
CURRENT_VIDEO = "type_switch_validation_02.avi"

VIDEO_COUNTS = {
    CURRENT_VIDEO: 2093,
    "near_050m_front.avi": 205,
    "near_070m_front.avi": 375,
    "near_090m_front.avi": 402,
    "near_angle_sweep.avi": 387,
    "near_distance_sweep.avi": 430,
    "background_only.avi": 386,
}
KNOWN_TYPE = {
    "near_050m_front.avi": "small",
    "near_070m_front.avi": "small",
    "near_090m_front.avi": "large",
    "near_angle_sweep.avi": "large",
    "near_distance_sweep.avi": "large",
}
EXPECTED_PASSES = {
    (CURRENT_VIDEO, "small"): 2093,
    (CURRENT_VIDEO, "large"): 2074,
    ("near_050m_front.avi", "small"): 205,
    ("near_070m_front.avi", "small"): 375,
    ("near_090m_front.avi", "large"): 402,
    ("near_angle_sweep.avi", "large"): 386,
    ("near_distance_sweep.avi", "large"): 430,
}
EXPECTED_FAILURE_FRAMES = {
    (CURRENT_VIDEO, "small"): [],
    (CURRENT_VIDEO, "large"): list(range(242, 261)),
    ("near_050m_front.avi", "small"): [],
    ("near_070m_front.avi", "small"): [],
    ("near_090m_front.avi", "large"): [],
    ("near_angle_sweep.avi", "large"): [212],
    ("near_distance_sweep.avi", "large"): [],
}

ENDPOINT_DISTANCE_PX = 6.0
DIRECTION_DIFFERENCE_DEG = 5.0

CANDIDATE_FIELDS = [
    "video",
    "frame_index",
    "candidate_index",
    "truth_type",
    "yolo_confidence",
    "track_id",
    "latched_type",
    "pair_extracted",
    "output_valid_before",
    "selected_by_ownership",
    "output_valid_after",
    "ownership_failure_reason",
    "left_light_cluster_id",
    "right_light_cluster_id",
    "conflicting_candidate_indices_json",
    "raw_ratio",
    "pnp_distance_m",
    "pnp_mean_reprojection_error_px",
    "box_xyxy_json",
    "traditional_light_points_xy_json",
]
FRAME_FIELDS = [
    "video",
    "frame_index",
    "truth_type",
    "candidate_count",
    "valid_output_count_before",
    "valid_output_count_after",
    "conflict_present",
    "conflict_pair_count",
    "selected_candidate_indices_json",
    "filtered_candidate_indices_json",
    "truth_candidate_count",
    "correct_type_valid_output_count_before",
    "correct_type_valid_output_count_after",
    "frame_pass_before",
    "frame_pass_after",
    "new_failure",
    "unmatched_valid_output_count_before",
    "unmatched_valid_output_count_after",
]
CONFLICT_FIELDS = [
    "video",
    "frame_index",
    "valid_output_count_before",
    "valid_output_count_after",
    "valid_candidate_indices_json",
    "selected_candidate_indices_json",
    "filtered_candidate_indices_json",
    "selected_yolo_confidence_sum",
    "physical_light_cluster_count",
    "physical_light_clusters_json",
    "conflicts_json",
]


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


def load_calibration() -> tuple[np.ndarray, np.ndarray]:
    data = yaml.safe_load(CALIBRATION.read_text(encoding="utf-8"))
    camera_matrix = np.asarray(
        data["camera_matrix"]["data"], dtype=np.float64
    ).reshape(3, 3)
    distortion = np.asarray(
        data["distortion_coefficients"]["data"], dtype=np.float64
    ).reshape(-1, 1)
    return camera_matrix, distortion


def video_path(video: str) -> Path:
    return SWITCH / video if video == CURRENT_VIDEO else DATA / video


def expected_truth_types(video: str) -> list[str]:
    if video == CURRENT_VIDEO:
        return ["small", "large"]
    if video in KNOWN_TYPE:
        return [KNOWN_TYPE[video]]
    return [""]


def load_candidate_universe() -> dict[str, dict[int, list[dict]]]:
    videos = {video: defaultdict(list) for video in VIDEO_COUNTS}
    for row in read_csv(CURRENT_INPUT):
        videos[CURRENT_VIDEO][int(row["frame_index"])].append(
            {
                "candidate_index": int(row["candidate_index"]),
                "truth_type": row["truth_type"],
                "box_xyxy": json.loads(row["box_xyxy_json"]),
                "yolo_confidence": float(row["yolo_confidence"]),
            }
        )
    for row in read_csv(PRIOR_INPUT):
        video = row["video"]
        if video not in videos:
            continue
        frame_index = int(row["frame_index"])
        for candidate_index, detail in enumerate(json.loads(row["candidate_details_json"])):
            videos[video][frame_index].append(
                {
                    "candidate_index": candidate_index,
                    "truth_type": KNOWN_TYPE.get(video, ""),
                    "box_xyxy": detail["box_xyxy"],
                    "yolo_confidence": float(detail["yolo_confidence"]),
                }
            )
    for frame_candidates in videos.values():
        for candidates in frame_candidates.values():
            candidates.sort(key=lambda item: item["candidate_index"])
    count = sum(
        len(candidates)
        for frames in videos.values()
        for candidates in frames.values()
    )
    if count != 6168:
        raise RuntimeError(f"saved candidate universe mismatch: {count} != 6168")
    return videos


def parity_index() -> dict[tuple[str, int, int], dict]:
    index = {}
    for row in read_csv(PARITY / "candidate_results.csv"):
        key = (row["video"], int(row["frame_index"]), int(row["candidate_index"]))
        index[key] = row
    if len(index) != 6168:
        raise RuntimeError(f"parity candidate count mismatch: {len(index)}")
    return index


def audited_unmatched_keys() -> set[tuple[str, int, int]]:
    keys = {
        (row["video"], int(row["frame_index"]), int(row["candidate_index"]))
        for row in read_csv(PARITY / "unmatched_candidates.csv")
    }
    if len(keys) != 192:
        raise RuntimeError(f"audited unmatched key count mismatch: {len(keys)}")
    classifications = read_csv(AUDIT / "track_classifications.csv")
    if len(classifications) != 42 or {
        row["classification"] for row in classifications
    } != {"NON_ARMOR_FALSE_POSITIVE"}:
        raise RuntimeError("unmatched audit decision is not the frozen 42-track result")
    return keys


def bool_value(value: str) -> bool:
    return value == "True"


def undirected_direction_difference(first: dict, second: dict) -> float:
    first_vector = np.asarray(first["bottom"], dtype=np.float64) - np.asarray(
        first["top"], dtype=np.float64
    )
    second_vector = np.asarray(second["bottom"], dtype=np.float64) - np.asarray(
        second["top"], dtype=np.float64
    )
    denominator = float(np.linalg.norm(first_vector) * np.linalg.norm(second_vector))
    if denominator <= 0.0:
        return float("inf")
    cosine = abs(float(np.dot(first_vector, second_vector) / denominator))
    return float(math.degrees(math.acos(np.clip(cosine, -1.0, 1.0))))


def same_physical_light(first: dict, second: dict) -> bool:
    return (
        float(np.linalg.norm(np.asarray(first["top"]) - np.asarray(second["top"])))
        <= ENDPOINT_DISTANCE_PX
        and float(
            np.linalg.norm(np.asarray(first["bottom"]) - np.asarray(second["bottom"]))
        )
        <= ENDPOINT_DISTANCE_PX
        and undirected_direction_difference(first, second) <= DIRECTION_DIFFERENCE_DEG
    )


class UnionFind:
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


def build_conflicts(valid: list[dict]) -> tuple[dict[int, tuple[int, int]], list[dict], list[dict]]:
    light_observations = []
    for item in valid:
        for side in ("left", "right"):
            light_observations.append(
                {
                    "candidate_index": item["candidate_index"],
                    "side": side,
                    "light": item["result"]["pair"][side],
                }
            )
    union_find = UnionFind(len(light_observations))
    for first_index, second_index in itertools.combinations(
        range(len(light_observations)), 2
    ):
        if same_physical_light(
            light_observations[first_index]["light"],
            light_observations[second_index]["light"],
        ):
            union_find.union(first_index, second_index)

    roots = sorted({union_find.find(index) for index in range(len(light_observations))})
    cluster_id = {root: index for index, root in enumerate(roots)}
    candidate_clusters = {}
    clusters = defaultdict(list)
    for observation_index, observation in enumerate(light_observations):
        identifier = cluster_id[union_find.find(observation_index)]
        clusters[identifier].append(
            {
                "candidate_index": observation["candidate_index"],
                "side": observation["side"],
                "top": np.asarray(observation["light"]["top"], dtype=float).tolist(),
                "bottom": np.asarray(
                    observation["light"]["bottom"], dtype=float
                ).tolist(),
            }
        )
    for item in valid:
        identifiers = []
        for side in ("left", "right"):
            observation_index = next(
                index
                for index, observation in enumerate(light_observations)
                if observation["candidate_index"] == item["candidate_index"]
                and observation["side"] == side
            )
            identifiers.append(cluster_id[union_find.find(observation_index)])
        candidate_clusters[item["candidate_index"]] = tuple(identifiers)

    conflicts = []
    for first, second in itertools.combinations(valid, 2):
        first_clusters = set(candidate_clusters[first["candidate_index"]])
        second_clusters = set(candidate_clusters[second["candidate_index"]])
        shared = sorted(first_clusters & second_clusters)
        if shared:
            conflicts.append(
                {
                    "first_candidate_index": first["candidate_index"],
                    "second_candidate_index": second["candidate_index"],
                    "shared_light_cluster_ids": shared,
                }
            )
    cluster_rows = [
        {"cluster_id": identifier, "members": members}
        for identifier, members in sorted(clusters.items())
    ]
    return candidate_clusters, cluster_rows, conflicts


def select_without_truth(valid: list[dict]) -> dict:
    """Select the frozen objective using geometry, confidence, and index only."""
    candidate_clusters, clusters, conflicts = build_conflicts(valid)
    conflict_pairs = {
        frozenset((row["first_candidate_index"], row["second_candidate_index"]))
        for row in conflicts
    }
    best_indices: tuple[int, ...] | None = None
    best_confidence = float("-inf")
    ordered = sorted(valid, key=lambda item: item["candidate_index"])
    for mask in range(1 << len(ordered)):
        subset = [ordered[index] for index in range(len(ordered)) if mask & (1 << index)]
        indices = tuple(item["candidate_index"] for item in subset)
        if any(frozenset(pair) in conflict_pairs for pair in itertools.combinations(indices, 2)):
            continue
        confidence = sum(item["yolo_confidence"] for item in subset)
        if (
            best_indices is None
            or len(indices) > len(best_indices)
            or (len(indices) == len(best_indices) and confidence > best_confidence)
            or (
                len(indices) == len(best_indices)
                and confidence == best_confidence
                and indices < best_indices
            )
        ):
            best_indices = indices
            best_confidence = confidence
    if best_indices is None:
        raise RuntimeError("ownership selection found no subset")
    return {
        "selected_indices": best_indices,
        "selected_confidence_sum": best_confidence,
        "candidate_clusters": candidate_clusters,
        "clusters": clusters,
        "conflicts": conflicts,
    }


def compact_frames(frames: list[int]) -> str:
    if not frames:
        return "none"
    intervals = []
    start = previous = frames[0]
    for frame_index in frames[1:]:
        if frame_index != previous + 1:
            intervals.append((start, previous))
            start = frame_index
        previous = frame_index
    intervals.append((start, previous))
    return ",".join(
        str(start) if start == end else f"{start}-{end}" for start, end in intervals
    )


def failure_runs(frames: list[int]) -> list[dict]:
    if not frames:
        return []
    runs = []
    start = previous = frames[0]
    for frame_index in frames[1:]:
        if frame_index != previous + 1:
            runs.append(
                {"start_frame": start, "end_frame": previous, "length_frames": previous - start + 1}
            )
            start = frame_index
        previous = frame_index
    runs.append(
        {"start_frame": start, "end_frame": previous, "length_frames": previous - start + 1}
    )
    return runs


def serialize_points(pair: dict | None) -> str:
    if pair is None:
        return ""
    return json.dumps(
        np.asarray(pair["points_xy"], dtype=float).tolist(), separators=(",", ":")
    )


def baseline_mismatches(
    video: str,
    frame_index: int,
    candidate: dict,
    result: dict,
    baseline: dict,
) -> list[dict]:
    mismatches = []
    comparisons = {
        "truth_type": (candidate["truth_type"], baseline["truth_type"]),
        "track_id": (str(result["track_id"]), baseline["track_id"]),
        "pair_extracted": (str(result["pair"] is not None), baseline["pair_extracted"]),
        "latched_type": (result["latched_type"] or "", baseline["latched_type"]),
        "output_valid": (str(result["output_valid"]), baseline["output_valid"]),
        "failure_reason": (result["failure_reason"] or "", baseline["failure_reason"]),
    }
    for field, (actual, expected) in comparisons.items():
        if actual != expected:
            mismatches.append(
                {
                    "video": video,
                    "frame_index": frame_index,
                    "candidate_index": candidate["candidate_index"],
                    "field": field,
                    "expected": expected,
                    "actual": actual,
                }
            )
    return mismatches


def process_all() -> tuple[list[dict], list[dict], list[dict], dict, dict]:
    universe = load_candidate_universe()
    baseline = parity_index()
    audited_keys = audited_unmatched_keys()
    camera_matrix, distortion = load_calibration()
    candidate_rows = []
    frame_rows = []
    conflict_rows = []
    mismatches = []
    runtime_results_by_frame = {}
    type_jumps = 0

    for video, frame_count in VIDEO_COUNTS.items():
        capture = cv2.VideoCapture(str(video_path(video)))
        if not capture.isOpened():
            raise RuntimeError(f"cannot open {video_path(video)}")
        processor = runtime.RawTrackLatchProcessor()
        previous_track_types = {}
        for frame_index in range(frame_count):
            ok, image = capture.read()
            if not ok or image is None:
                raise RuntimeError(f"cannot decode {video} frame {frame_index}")
            candidates = universe[video].get(frame_index, [])
            boxes = np.asarray(
                [candidate["box_xyxy"] for candidate in candidates], dtype=np.float32
            ).reshape(-1, 4)
            results = processor.process_frame(
                image, boxes, frame_index, camera_matrix, distortion
            )
            if len(results) != len(candidates):
                raise RuntimeError(f"candidate/result mismatch {video}:{frame_index}")
            paired = list(zip(candidates, results))
            valid = [
                {
                    "candidate_index": candidate["candidate_index"],
                    "yolo_confidence": candidate["yolo_confidence"],
                    "result": result,
                }
                for candidate, result in paired
                if result["output_valid"]
            ]
            ownership = select_without_truth(valid)
            selected = set(ownership["selected_indices"])
            conflict_map = defaultdict(list)
            for conflict in ownership["conflicts"]:
                first = conflict["first_candidate_index"]
                second = conflict["second_candidate_index"]
                conflict_map[first].append(second)
                conflict_map[second].append(first)

            if ownership["conflicts"]:
                filtered = sorted(
                    item["candidate_index"] for item in valid
                    if item["candidate_index"] not in selected
                )
                conflict_rows.append(
                    {
                        "video": video,
                        "frame_index": frame_index,
                        "valid_output_count_before": len(valid),
                        "valid_output_count_after": len(selected),
                        "valid_candidate_indices_json": json.dumps(
                            sorted(item["candidate_index"] for item in valid),
                            separators=(",", ":"),
                        ),
                        "selected_candidate_indices_json": json.dumps(
                            sorted(selected), separators=(",", ":")
                        ),
                        "filtered_candidate_indices_json": json.dumps(
                            filtered, separators=(",", ":")
                        ),
                        "selected_yolo_confidence_sum": ownership[
                            "selected_confidence_sum"
                        ],
                        "physical_light_cluster_count": len(ownership["clusters"]),
                        "physical_light_clusters_json": json.dumps(
                            ownership["clusters"], separators=(",", ":")
                        ),
                        "conflicts_json": json.dumps(
                            ownership["conflicts"], separators=(",", ":")
                        ),
                    }
                )

            enriched = []
            for candidate, result in paired:
                key = (video, frame_index, candidate["candidate_index"])
                mismatches.extend(
                    baseline_mismatches(
                        video, frame_index, candidate, result, baseline[key]
                    )
                )
                if result["latched_type"] is not None:
                    previous = previous_track_types.get(result["track_id"])
                    if previous is not None and previous != result["latched_type"]:
                        type_jumps += 1
                    previous_track_types[result["track_id"]] = result["latched_type"]
                after = bool(
                    result["output_valid"]
                    and candidate["candidate_index"] in selected
                )
                clusters = ownership["candidate_clusters"].get(
                    candidate["candidate_index"], (None, None)
                )
                pnp = result["pnp"]
                row = {
                    "video": video,
                    "frame_index": frame_index,
                    "candidate_index": candidate["candidate_index"],
                    "truth_type": candidate["truth_type"],
                    "yolo_confidence": candidate["yolo_confidence"],
                    "track_id": result["track_id"],
                    "latched_type": result["latched_type"],
                    "pair_extracted": result["pair"] is not None,
                    "output_valid_before": result["output_valid"],
                    "selected_by_ownership": (
                        candidate["candidate_index"] in selected
                        if result["output_valid"]
                        else False
                    ),
                    "output_valid_after": after,
                    "ownership_failure_reason": (
                        "same_frame_light_conflict"
                        if result["output_valid"] and not after
                        else result["failure_reason"]
                    ),
                    "left_light_cluster_id": clusters[0],
                    "right_light_cluster_id": clusters[1],
                    "conflicting_candidate_indices_json": json.dumps(
                        sorted(conflict_map[candidate["candidate_index"]]),
                        separators=(",", ":"),
                    ),
                    "raw_ratio": result["raw_ratio"],
                    "pnp_distance_m": pnp["distance_m"] if pnp else None,
                    "pnp_mean_reprojection_error_px": (
                        pnp["mean_reprojection_error_px"] if pnp else None
                    ),
                    "box_xyxy_json": json.dumps(
                        np.asarray(candidate["box_xyxy"], dtype=float).tolist(),
                        separators=(",", ":"),
                    ),
                    "traditional_light_points_xy_json": serialize_points(
                        result["pair"]
                    ),
                    "is_audited_cross_target": key in audited_keys,
                }
                enriched.append(row)
                candidate_rows.append(
                    {field: row[field] for field in CANDIDATE_FIELDS}
                )
            runtime_results_by_frame[(video, frame_index)] = enriched

            for truth_type in expected_truth_types(video):
                relevant = [row for row in enriched if row["truth_type"] == truth_type]
                correct_before = sum(
                    row["output_valid_before"] and row["latched_type"] == truth_type
                    for row in relevant
                ) if truth_type else 0
                correct_after = sum(
                    row["output_valid_after"] and row["latched_type"] == truth_type
                    for row in relevant
                ) if truth_type else 0
                pass_before = bool(correct_before) if truth_type else True
                pass_after = bool(correct_after) if truth_type else True
                unmatched_before = sum(
                    row["output_valid_before"] and not row["truth_type"]
                    for row in enriched
                )
                unmatched_after = sum(
                    row["output_valid_after"] and not row["truth_type"]
                    for row in enriched
                )
                frame_rows.append(
                    {
                        "video": video,
                        "frame_index": frame_index,
                        "truth_type": truth_type,
                        "candidate_count": len(candidates),
                        "valid_output_count_before": len(valid),
                        "valid_output_count_after": len(selected),
                        "conflict_present": bool(ownership["conflicts"]),
                        "conflict_pair_count": len(ownership["conflicts"]),
                        "selected_candidate_indices_json": json.dumps(
                            sorted(selected), separators=(",", ":")
                        ),
                        "filtered_candidate_indices_json": json.dumps(
                            sorted(
                                item["candidate_index"] for item in valid
                                if item["candidate_index"] not in selected
                            ),
                            separators=(",", ":"),
                        ),
                        "truth_candidate_count": len(relevant) if truth_type else 0,
                        "correct_type_valid_output_count_before": correct_before,
                        "correct_type_valid_output_count_after": correct_after,
                        "frame_pass_before": pass_before,
                        "frame_pass_after": pass_after,
                        "new_failure": pass_before and not pass_after,
                        "unmatched_valid_output_count_before": unmatched_before,
                        "unmatched_valid_output_count_after": unmatched_after,
                    }
                )
        extra_ok, _ = capture.read()
        capture.release()
        if extra_ok:
            raise RuntimeError(f"{video} contains more than {frame_count} frames")
    return (
        candidate_rows,
        frame_rows,
        conflict_rows,
        {
            "count": len(mismatches),
            "details": mismatches[:100],
        },
        {
            "type_jump_count": type_jumps,
            "runtime_results_by_frame": runtime_results_by_frame,
            "audited_keys": audited_keys,
        },
    )


def choose_representative_conflicts(conflict_rows: list[dict]) -> list[dict]:
    current = [row for row in conflict_rows if row["video"] == CURRENT_VIDEO]
    if not current:
        return []
    ordered = sorted(current, key=lambda row: int(row["frame_index"]))
    indices = {
        0,
        (len(ordered) - 1) // 4,
        (len(ordered) - 1) // 2,
        3 * (len(ordered) - 1) // 4,
        len(ordered) - 1,
    }
    max_filtered = max(
        range(len(ordered)),
        key=lambda index: (
            len(json.loads(ordered[index]["filtered_candidate_indices_json"])),
            -int(ordered[index]["frame_index"]),
        ),
    )
    indices.add(max_filtered)
    representatives = [ordered[index] for index in sorted(indices)]
    for video in sorted({row["video"] for row in conflict_rows} - {CURRENT_VIDEO}):
        representatives.append(
            min(
                (row for row in conflict_rows if row["video"] == video),
                key=lambda row: int(row["frame_index"]),
            )
        )
    return representatives


def draw_representatives(
    representatives: list[dict], runtime_results_by_frame: dict
) -> list[str]:
    directory = OUTPUT / "representative_images"
    directory.mkdir(exist_ok=True)
    images = {}
    for video in sorted({row["video"] for row in representatives}):
        wanted = {
            int(row["frame_index"])
            for row in representatives if row["video"] == video
        }
        capture = cv2.VideoCapture(str(video_path(video)))
        frame_index = 0
        while wanted:
            ok, image = capture.read()
            if not ok or image is None:
                break
            if frame_index in wanted:
                images[(video, frame_index)] = image.copy()
                wanted.remove(frame_index)
            frame_index += 1
        capture.release()
        if wanted:
            raise RuntimeError(
                f"representative decode failed for {video}: {sorted(wanted)}"
            )

    palette = [
        (255, 255, 0),
        (255, 0, 255),
        (0, 255, 255),
        (255, 128, 0),
        (0, 128, 255),
        (128, 255, 0),
    ]
    paths = []
    for conflict in representatives:
        video = conflict["video"]
        frame = int(conflict["frame_index"])
        image = images[(video, frame)]
        rows = runtime_results_by_frame[(video, frame)]
        selected = set(json.loads(conflict["selected_candidate_indices_json"]))
        filtered = set(json.loads(conflict["filtered_candidate_indices_json"]))
        for row in rows:
            if not row["output_valid_before"]:
                color = (128, 128, 128)
            elif row["candidate_index"] in selected:
                color = (0, 220, 0)
            else:
                color = (0, 0, 255)
            box = json.loads(row["box_xyxy_json"])
            x1, y1, x2, y2 = (int(round(value)) for value in box)
            cv2.rectangle(image, (x1, y1), (x2, y2), color, 3, cv2.LINE_AA)
            cv2.putText(
                image,
                f"C{row['candidate_index']} {'KEEP' if row['candidate_index'] in selected else 'DROP'}",
                (x1, max(25, y1 - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                color,
                2,
                cv2.LINE_AA,
            )
            points_text = row["traditional_light_points_xy_json"]
            if points_text:
                points = json.loads(points_text)
                for side_index, point_indices in enumerate(((0, 1), (3, 2))):
                    cluster_key = (
                        "left_light_cluster_id" if side_index == 0
                        else "right_light_cluster_id"
                    )
                    cluster_id = int(row[cluster_key])
                    light_color = palette[cluster_id % len(palette)]
                    first = tuple(int(round(value)) for value in points[point_indices[0]])
                    second = tuple(int(round(value)) for value in points[point_indices[1]])
                    cv2.line(image, first, second, light_color, 5, cv2.LINE_AA)
                    cv2.putText(
                        image,
                        f"L{cluster_id}",
                        (first[0] + 4, first[1] - 5),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        light_color,
                        2,
                        cv2.LINE_AA,
                    )
        title = (
            f"frame {frame}: valid {conflict['valid_output_count_before']} -> "
            f"{conflict['valid_output_count_after']} | green keep red drop"
        )
        cv2.putText(
            image,
            title,
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.85,
            (0, 0, 0),
            5,
            cv2.LINE_AA,
        )
        cv2.putText(
            image,
            title,
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.85,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        if video == CURRENT_VIDEO:
            relative = f"representative_images/conflict_frame_{frame:04d}.png"
        else:
            relative = (
                f"representative_images/{Path(video).stem}_conflict_frame_"
                f"{frame:04d}.png"
            )
        cv2.imwrite(str(OUTPUT / relative), image)
        paths.append(relative)
    return paths


def summarize(
    candidate_rows: list[dict],
    frame_rows: list[dict],
    conflict_rows: list[dict],
    baseline_consistency: dict,
    runtime_state: dict,
) -> dict:
    metrics = []
    all_matched_exact = True
    no_new_failures = True
    video_summaries = {}
    for video in VIDEO_COUNTS:
        video_candidates = [row for row in candidate_rows if row["video"] == video]
        video_frames = [row for row in frame_rows if row["video"] == video]
        truth_metrics = []
        for truth_type in expected_truth_types(video):
            if not truth_type:
                continue
            relevant = [row for row in video_frames if row["truth_type"] == truth_type]
            before_frames = sorted(
                int(row["frame_index"]) for row in relevant if row["frame_pass_before"]
            )
            after_frames = sorted(
                int(row["frame_index"]) for row in relevant if row["frame_pass_after"]
            )
            failure_before = sorted(
                int(row["frame_index"]) for row in relevant if not row["frame_pass_before"]
            )
            failure_after = sorted(
                int(row["frame_index"]) for row in relevant if not row["frame_pass_after"]
            )
            new_failures = sorted(set(failure_after) - set(failure_before))
            expected_count = EXPECTED_PASSES[(video, truth_type)]
            expected_failures = EXPECTED_FAILURE_FRAMES[(video, truth_type)]
            exact = len(after_frames) == expected_count and failure_after == expected_failures
            all_matched_exact = all_matched_exact and exact
            no_new_failures = no_new_failures and not new_failures
            metric = {
                "video": video,
                "truth_type": truth_type,
                "baseline_pass_count": len(before_frames),
                "after_pass_count": len(after_frames),
                "expected_pass_count": expected_count,
                "baseline_failure_frames": failure_before,
                "after_failure_frames": failure_after,
                "after_failure_runs": failure_runs(failure_after),
                "new_failure_frames": new_failures,
                "exact_parity": exact,
            }
            metrics.append(metric)
            truth_metrics.append(metric)
        conflicts = [row for row in conflict_rows if row["video"] == video]
        video_summaries[video] = {
            "frame_count": VIDEO_COUNTS[video],
            "candidate_count": len(video_candidates),
            "output_count_before": sum(
                bool(row["output_valid_before"]) for row in video_candidates
            ),
            "output_count_after": sum(
                bool(row["output_valid_after"]) for row in video_candidates
            ),
            "conflict_frame_count": len(conflicts),
            "filtered_output_count": sum(
                bool(row["output_valid_before"]) and not bool(row["output_valid_after"])
                for row in video_candidates
            ),
            "truth_metrics": truth_metrics,
        }

    audited_keys = runtime_state["audited_keys"]
    audited_rows = [
        row for row in candidate_rows
        if (row["video"], int(row["frame_index"]), int(row["candidate_index"]))
        in audited_keys
    ]
    remaining_audited = sum(bool(row["output_valid_after"]) for row in audited_rows)
    unmatched_rows = [row for row in candidate_rows if not row["truth_type"]]
    new_unmatched = sum(
        not bool(row["output_valid_before"]) and bool(row["output_valid_after"])
        for row in unmatched_rows
    )
    remaining_unmatched = sum(bool(row["output_valid_after"]) for row in unmatched_rows)
    background = video_summaries["background_only.avi"]

    conditions = {
        "baseline_replay_matches_parity_v2": baseline_consistency["count"] == 0,
        "matched_truth_parity_exact": all_matched_exact,
        "audited_cross_target_outputs_zero": remaining_audited == 0,
        "no_new_matched_failure_frames_or_runs": no_new_failures,
        "background_zero_candidates_outputs": (
            background["candidate_count"] == 0
            and background["output_count_after"] == 0
        ),
        "type_jumps_zero": runtime_state["type_jump_count"] == 0,
        "no_new_unmatched_valid_outputs": new_unmatched == 0,
    }
    decision = (
        "SAME_FRAME_LIGHT_OWNERSHIP_PASS"
        if all(conditions.values())
        else "SAME_FRAME_LIGHT_OWNERSHIP_FAIL"
    )
    failure_details = []
    if decision.endswith("FAIL"):
        for metric in metrics:
            if not metric["exact_parity"]:
                failure_details.append(metric)
        for row in candidate_rows:
            key = (row["video"], int(row["frame_index"]), int(row["candidate_index"]))
            if key in audited_keys and row["output_valid_after"]:
                failure_details.append(
                    {
                        "failure": "audited_cross_target_retained",
                        "video": row["video"],
                        "frame_index": row["frame_index"],
                        "candidate_index": row["candidate_index"],
                        "conflicts": row["conflicting_candidate_indices_json"],
                    }
                )
            if row["truth_type"] and row["output_valid_before"] and not row["output_valid_after"]:
                failure_details.append(
                    {
                        "failure": "matched_candidate_filtered",
                        "video": row["video"],
                        "frame_index": row["frame_index"],
                        "candidate_index": row["candidate_index"],
                        "truth_type": row["truth_type"],
                        "conflicts": row["conflicting_candidate_indices_json"],
                    }
                )
    conflict_breakdown = {}
    for video in VIDEO_COUNTS:
        rows = [row for row in conflict_rows if row["video"] == video]
        if not rows:
            continue
        transitions = Counter(
            (
                int(row["valid_output_count_before"]),
                int(row["valid_output_count_after"]),
            )
            for row in rows
        )
        conflict_breakdown[video] = {
            "frame_indices": [int(row["frame_index"]) for row in rows],
            "before_after_counts": [
                {"before": before, "after": after, "frame_count": count}
                for (before, after), count in sorted(transitions.items())
            ],
        }
    return {
        "experiment": "same_frame_light_ownership_v1",
        "algorithm": {
            "endpoint_distance_px_inclusive": ENDPOINT_DISTANCE_PX,
            "direction_difference_deg_inclusive": DIRECTION_DIFFERENCE_DEG,
            "selection_priority": [
                "maximum_candidate_count",
                "maximum_yolo_confidence_sum",
                "lexicographically_smallest_candidate_index_tuple",
            ],
            "truth_used_during_selection": False,
            "yolo_rerun": False,
        },
        "baseline_consistency": baseline_consistency,
        "metrics": metrics,
        "video_summaries": video_summaries,
        "conflict_frame_count": len(conflict_rows),
        "conflict_breakdown": conflict_breakdown,
        "audited_cross_target": {
            "baseline_valid_output_count": len(audited_rows),
            "remaining_valid_output_count": remaining_audited,
        },
        "unmatched_outputs": {
            "baseline_valid_output_count": sum(
                bool(row["output_valid_before"]) for row in unmatched_rows
            ),
            "remaining_valid_output_count": remaining_unmatched,
            "new_valid_output_count": new_unmatched,
        },
        "type_jump_count": runtime_state["type_jump_count"],
        "background": background,
        "conditions": conditions,
        "failure_details": failure_details,
        "decision_code": decision,
    }


def write_report(summary: dict, representative_paths: list[str]) -> None:
    lines = [
        "# Same-Frame Light Ownership V1",
        "",
        "The frozen saved YOLO candidate universe was replayed. YOLO was not run, and truth was not available to the ownership selector.",
        "",
        "## Matched-Truth Parity",
        "",
        "| Video | Truth | Baseline pass | After pass | Failure frames after | New failures |",
        "|---|---|---:|---:|---|---|",
    ]
    for metric in summary["metrics"]:
        lines.append(
            f"| `{metric['video']}` | {metric['truth_type']} | "
            f"{metric['baseline_pass_count']} | {metric['after_pass_count']} | "
            f"{compact_frames(metric['after_failure_frames'])} | "
            f"{compact_frames(metric['new_failure_frames'])} |"
        )
    lines.extend(
        [
            "",
            "## Ownership Results",
            "",
            f"- Conflict frames: {summary['conflict_frame_count']}.",
            (
                "- Audited cross-target valid outputs before/after: "
                f"{summary['audited_cross_target']['baseline_valid_output_count']}/"
                f"{summary['audited_cross_target']['remaining_valid_output_count']}."
            ),
            (
                "- All unmatched valid outputs before/after/new: "
                f"{summary['unmatched_outputs']['baseline_valid_output_count']}/"
                f"{summary['unmatched_outputs']['remaining_valid_output_count']}/"
                f"{summary['unmatched_outputs']['new_valid_output_count']}."
            ),
            f"- Latched-type jumps: {summary['type_jump_count']}.",
            (
                "- Background frames/candidates/outputs: "
                f"{summary['background']['frame_count']}/"
                f"{summary['background']['candidate_count']}/"
                f"{summary['background']['output_count_after']}."
            ),
            "",
            "## Per-Video Output Counts",
            "",
            "| Video | Candidates | Outputs before | Outputs after | Conflict frames | Filtered outputs |",
            "|---|---:|---:|---:|---:|---:|",
        ]
    )
    for video, values in summary["video_summaries"].items():
        lines.append(
            f"| `{video}` | {values['candidate_count']} | "
            f"{values['output_count_before']} | {values['output_count_after']} | "
            f"{values['conflict_frame_count']} | {values['filtered_output_count']} |"
        )
    lines.extend(["", "## Conditions", ""])
    for condition, passed in summary["conditions"].items():
        lines.append(f"- `{condition}`: **{'PASS' if passed else 'FAIL'}**")
    lines.extend(["", "## Representative Images", ""])
    for path in representative_paths:
        lines.append(f"- `{path}`")
    if summary["failure_details"]:
        lines.extend(["", "## Failure Details", "", "See `summary.json` and `candidate_results.csv` for exact candidates and conflict relations."])
    lines.extend(
        [
            "",
            "## Decision",
            "",
            f"**{summary['decision_code']}**",
            "",
        ]
    )
    with (OUTPUT / "report.md").open("x", encoding="utf-8") as stream:
        stream.write("\n".join(lines))


def main() -> None:
    preregistration = OUTPUT / "preregistration.md"
    if not preregistration.is_file():
        raise RuntimeError("preregistration.md must exist before execution")
    prohibited_outputs = [
        OUTPUT / name
        for name in (
            "frame_results.csv",
            "candidate_results.csv",
            "conflict_results.csv",
            "summary.json",
            "report.md",
        )
    ]
    if any(path.exists() for path in prohibited_outputs):
        raise RuntimeError("experiment outputs already exist")

    candidate_rows, frame_rows, conflict_rows, consistency, runtime_state = process_all()
    write_csv(OUTPUT / "candidate_results.csv", CANDIDATE_FIELDS, candidate_rows)
    write_csv(OUTPUT / "frame_results.csv", FRAME_FIELDS, frame_rows)
    write_csv(OUTPUT / "conflict_results.csv", CONFLICT_FIELDS, conflict_rows)
    summary = summarize(
        candidate_rows, frame_rows, conflict_rows, consistency, runtime_state
    )
    representatives = choose_representative_conflicts(conflict_rows)
    representative_paths = draw_representatives(
        representatives, runtime_state["runtime_results_by_frame"]
    )
    summary["representative_images"] = representative_paths
    write_json(OUTPUT / "summary.json", summary)
    write_report(summary, representative_paths)
    print(
        json.dumps(
            {
                "decision": summary["decision_code"],
                "conflict_frames": summary["conflict_frame_count"],
                "audited_remaining": summary["audited_cross_target"][
                    "remaining_valid_output_count"
                ],
                "matched_exact": summary["conditions"][
                    "matched_truth_parity_exact"
                ],
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
