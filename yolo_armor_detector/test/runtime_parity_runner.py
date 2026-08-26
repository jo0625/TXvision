#!/usr/bin/env python3
"""Replay frozen candidates through the formal raw-track-latch runtime path."""

from __future__ import annotations

import csv
import json
import sys
import time
from collections import Counter, defaultdict
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import yaml


PACKAGE = Path(__file__).resolve().parents[1]
RM_AUTO_AIM = PACKAGE.parents[0]
sys.path.insert(0, str(PACKAGE / "scripts"))

import raw_track_latch as runtime  # noqa: E402


DATA = Path(
    "/home/jo/my-project/my_first_autoaim_yolo/data/team_exchange/"
    "real_blank_near_20260812"
)
SWITCH = DATA / "type_switch_validation_02"
CURRENT_INPUT = SWITCH / "temporal_type_eval_v1/track_results.csv"
PRIOR_INPUT = DATA / "model_diagnostic_real_blank_v1/frame_results.csv"
CALIBRATION = RM_AUTO_AIM / "hik_camera/config/hikrobot_mv_cs016_10uc.yaml"
PARITY_V2 = RM_AUTO_AIM / "real_armor_diagnostics/raw_track_latch_runtime_parity_v2"
PARITY_V3 = RM_AUTO_AIM / "real_armor_diagnostics/raw_track_latch_runtime_parity_v3"
OUTPUT = RM_AUTO_AIM / "real_armor_diagnostics/raw_track_latch_runtime_parity_v4"
GENERAL_PARAMETER_FILE = (
    RM_AUTO_AIM / "auto_aim_bringup/params/yolo_detector_params.yaml"
)
CANDIDATE_PARAMETER_FILE = (
    RM_AUTO_AIM
    / "auto_aim_bringup/params/yolo_detector_params_v8_frozen_candidate.yaml"
)
CURRENT_FROZEN_MODEL = Path(
    "/home/jo/my-project/my_first_autoaim_yolo/models/frozen/"
    "armor_pose_real_blank_v1_candidate_20260812/model.pt"
)
NODE_SOURCE = PACKAGE / "scripts/yolo_armor_detector_node.py"

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
KNOWN_DUPLICATE_FILTERS = {
    ("near_angle_sweep.avi", 84, 1),
    ("near_angle_sweep.avi", 86, 1),
    ("near_distance_sweep.avi", 26, 1),
    ("near_distance_sweep.avi", 51, 1),
}

CANDIDATE_FIELDS = [
    "video",
    "frame_index",
    "candidate_index",
    "truth_type",
    "yolo_confidence",
    "track_id",
    "latched_type",
    "pair_extracted",
    "pnp_valid",
    "output_valid_before_ownership",
    "ownership_selected",
    "output_valid",
    "failure_reason",
    "raw_ratio",
    "pnp_mean_reprojection_error_px",
    "pnp_distance_m",
    "ownership_light_cluster_ids_json",
    "ownership_conflicting_candidate_indices_json",
    "ownership_resolution_mode",
    "ownership_overload_passthrough",
    "ownership_conflict_component_size",
    "v3_track_id",
    "v3_latched_type",
    "v3_pnp_valid",
    "v3_output_valid_before_ownership",
    "v3_ownership_selected",
    "v3_output_valid",
    "runtime_reference_match",
    "mismatch_fields_json",
    "box_xyxy_json",
]
FRAME_FIELDS = [
    "video",
    "frame_index",
    "truth_type",
    "candidate_count",
    "truth_candidate_count",
    "valid_output_count_before_ownership",
    "valid_output_count",
    "correct_type_valid_output_count",
    "frame_pass",
    "failure_reason",
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
    "conflicts_json",
    "largest_conflict_component_size",
    "exact_component_count",
    "overload_component_count",
    "exact_subsets_evaluated",
    "reference_match",
]
RUN_FIELDS = [
    "video",
    "truth_type",
    "start_frame",
    "end_frame",
    "length_frames",
    "failure_reasons_json",
]


def parse_bool(value: str) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes"}


def candidate_key(video: str, frame_index: int, candidate_index: int) -> tuple:
    return video, int(frame_index), int(candidate_index)


def load_csv_index(path: Path) -> dict[tuple, dict]:
    rows = {}
    with path.open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            key = candidate_key(
                row["video"], row["frame_index"], row["candidate_index"]
            )
            if key in rows:
                raise RuntimeError(f"duplicate candidate key in {path}: {key}")
            rows[key] = row
    return rows


def load_conflict_reference() -> dict[tuple[str, int], dict]:
    rows = {}
    path = PARITY_V3 / "conflict_results.csv"
    with path.open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            key = row["video"], int(row["frame_index"])
            rows[key] = row
    return rows


def load_calibration() -> tuple[np.ndarray, np.ndarray]:
    data = yaml.safe_load(CALIBRATION.read_text(encoding="utf-8"))
    camera_matrix = np.asarray(
        data["camera_matrix"]["data"], dtype=np.float64
    ).reshape(3, 3)
    distortion = np.asarray(
        data["distortion_coefficients"]["data"], dtype=np.float64
    ).reshape(-1, 1)
    return camera_matrix, distortion


def candidate_promotion_contract() -> dict:
    general = yaml.safe_load(GENERAL_PARAMETER_FILE.read_text(encoding="utf-8"))[
        "yolo_armor_detector"
    ]["ros__parameters"]
    candidate = yaml.safe_load(
        CANDIDATE_PARAMETER_FILE.read_text(encoding="utf-8")
    )["yolo_armor_detector"]["ros__parameters"]
    general_valid = (
        general.get("type_strategy") == "fixed"
        and general.get("armor_type") == "small"
    )
    candidate_valid = (
        candidate.get("type_strategy") == "normalized_dual_pnp_latch"
        and candidate.get("armor_type") == "small"
        and candidate.get("model") == str(CURRENT_FROZEN_MODEL)
    )
    node_text = NODE_SOURCE.read_text(encoding="utf-8")
    node_default_valid = 'declare_parameter("type_strategy", "fixed")' in node_text
    return {
        "valid": general_valid and candidate_valid and node_default_valid,
        "general_parameter_file": {
            "path": str(GENERAL_PARAMETER_FILE),
            "type_strategy": general.get("type_strategy"),
            "armor_type": general.get("armor_type"),
            "valid": general_valid,
        },
        "candidate_parameter_file": {
            "path": str(CANDIDATE_PARAMETER_FILE),
            "type_strategy": candidate.get("type_strategy"),
            "armor_type": candidate.get("armor_type"),
            "model": candidate.get("model"),
            "valid": candidate_valid,
        },
        "node_default_fixed": node_default_valid,
    }


def load_current() -> dict[int, list[dict]]:
    frames = defaultdict(list)
    with CURRENT_INPUT.open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            frame_index = int(row["frame_index"])
            frames[frame_index].append(
                {
                    "candidate_index": int(row["candidate_index"]),
                    "truth_type": row["truth_type"],
                    "box_xyxy": json.loads(row["box_xyxy_json"]),
                    "yolo_confidence": float(row["yolo_confidence"]),
                }
            )
    for candidates in frames.values():
        candidates.sort(key=lambda item: item["candidate_index"])
    if sum(map(len, frames.values())) != 4365:
        raise RuntimeError("current saved candidate universe is not 4365")
    return frames


def load_prior() -> dict[str, dict[int, list[dict]]]:
    frames = {
        video: defaultdict(list)
        for video in VIDEO_COUNTS
        if video != CURRENT_VIDEO
    }
    with PRIOR_INPUT.open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            video = row["video"]
            if video not in frames:
                continue
            frame_index = int(row["frame_index"])
            candidates = json.loads(row["candidate_details_json"])
            for candidate_index, candidate in enumerate(candidates):
                frames[video][frame_index].append(
                    {
                        "candidate_index": candidate_index,
                        "truth_type": KNOWN_TYPE.get(video, ""),
                        "box_xyxy": candidate["box_xyxy"],
                        "yolo_confidence": float(candidate["yolo_confidence"]),
                    }
                )
    return frames


def video_path(video: str) -> Path:
    return SWITCH / video if video == CURRENT_VIDEO else DATA / video


def expected_truth_types(video: str) -> list[str]:
    if video == CURRENT_VIDEO:
        return ["small", "large"]
    if video in KNOWN_TYPE:
        return [KNOWN_TYPE[video]]
    return []


def frame_failure_reason(relevant: list[tuple[dict, dict]]) -> str:
    if not relevant:
        return "no_truth_matched_yolo_candidate"
    reasons = Counter(result["failure_reason"] or "wrong_type" for _, result in relevant)
    return sorted(reasons.items(), key=lambda item: (-item[1], item[0]))[0][0]


def compare_candidate(
    video: str,
    frame_index: int,
    candidate: dict,
    result: dict,
    v3: dict,
) -> dict:
    pnp_valid = bool(result["pnp"] and result["pnp"]["valid"])
    expected_pnp = parse_bool(v3["pnp_valid"])
    expected_before = parse_bool(v3["output_valid_before_ownership"])
    expected_selected = parse_bool(v3["ownership_selected"])
    expected_after = parse_bool(v3["output_valid"])
    mismatches = []
    checks = {
        "track_id": int(result["track_id"]) == int(v3["track_id"]),
        "latched_type": (result["latched_type"] or "") == v3["latched_type"],
        "pnp_valid": pnp_valid == expected_pnp,
        "output_valid_before_ownership": (
            bool(result["output_valid_before_ownership"]) == expected_before
        ),
        "ownership_selected": (
            bool(result["ownership_selected"]) == expected_selected
        ),
        "output_valid": bool(result["output_valid"]) == expected_after,
    }
    for field, matched in checks.items():
        if not matched:
            mismatches.append(field)

    if result["output_valid_before_ownership"] and not result["ownership_selected"]:
        if result["failure_reason"] != "light_ownership_conflict":
            mismatches.append("filtered_failure_reason")
    elif not result["output_valid_before_ownership"]:
        if result["failure_reason"] != v3["failure_reason"]:
            mismatches.append("original_failure_reason")

    pnp = result["pnp"]
    return {
        "video": video,
        "frame_index": frame_index,
        "candidate_index": candidate["candidate_index"],
        "truth_type": candidate["truth_type"],
        "yolo_confidence": candidate["yolo_confidence"],
        "track_id": result["track_id"],
        "latched_type": result["latched_type"],
        "pair_extracted": result["pair"] is not None,
        "pnp_valid": pnp_valid,
        "output_valid_before_ownership": result["output_valid_before_ownership"],
        "ownership_selected": result["ownership_selected"],
        "output_valid": result["output_valid"],
        "failure_reason": result["failure_reason"],
        "raw_ratio": result["raw_ratio"],
        "pnp_mean_reprojection_error_px": (
            pnp["mean_reprojection_error_px"] if pnp else None
        ),
        "pnp_distance_m": pnp["distance_m"] if pnp else None,
        "ownership_light_cluster_ids_json": json.dumps(
            result["ownership_light_cluster_ids"], separators=(",", ":")
        ),
        "ownership_conflicting_candidate_indices_json": json.dumps(
            result["ownership_conflicting_candidate_indices"], separators=(",", ":")
        ),
        "ownership_resolution_mode": result["ownership_resolution_mode"],
        "ownership_overload_passthrough": result[
            "ownership_overload_passthrough"
        ],
        "ownership_conflict_component_size": result[
            "ownership_conflict_component_size"
        ],
        "v3_track_id": int(v3["track_id"]),
        "v3_latched_type": v3["latched_type"],
        "v3_pnp_valid": expected_pnp,
        "v3_output_valid_before_ownership": expected_before,
        "v3_ownership_selected": expected_selected,
        "v3_output_valid": expected_after,
        "runtime_reference_match": not mismatches,
        "mismatch_fields_json": json.dumps(mismatches, separators=(",", ":")),
        "box_xyxy_json": json.dumps(
            np.asarray(result["box_xyxy"], dtype=float).tolist(),
            separators=(",", ":"),
        ),
    }


def build_conflict_row(
    video: str,
    frame_index: int,
    candidates: list[dict],
    results: list[dict],
    ownership: dict,
    reference: dict | None,
) -> dict | None:
    valid_indices = [
        result["candidate_index"]
        for result in results
        if result["output_valid_before_ownership"]
    ]
    selected_indices = [
        result["candidate_index"]
        for result in results
        if result["output_valid_before_ownership"] and result["ownership_selected"]
    ]
    filtered_indices = sorted(set(valid_indices) - set(selected_indices))
    if not filtered_indices:
        return None

    cluster_ids = set()
    conflict_pairs = set()
    for result in results:
        if result["ownership_light_cluster_ids"] is not None:
            cluster_ids.update(result["ownership_light_cluster_ids"])
        for other in result["ownership_conflicting_candidate_indices"]:
            conflict_pairs.add(tuple(sorted((result["candidate_index"], other))))
    conflicts = [
        {"first_candidate_index": first, "second_candidate_index": second}
        for first, second in sorted(conflict_pairs)
    ]
    confidence_by_index = {
        candidate["candidate_index"]: candidate["yolo_confidence"]
        for candidate in candidates
    }
    reference_match = bool(
        reference
        and int(reference["valid_output_count_before"]) == len(valid_indices)
        and int(reference["valid_output_count_after"]) == len(selected_indices)
        and json.loads(reference["valid_candidate_indices_json"]) == valid_indices
        and json.loads(reference["selected_candidate_indices_json"])
        == selected_indices
        and json.loads(reference["filtered_candidate_indices_json"])
        == filtered_indices
    )
    return {
        "video": video,
        "frame_index": frame_index,
        "valid_output_count_before": len(valid_indices),
        "valid_output_count_after": len(selected_indices),
        "valid_candidate_indices_json": json.dumps(valid_indices, separators=(",", ":")),
        "selected_candidate_indices_json": json.dumps(
            selected_indices, separators=(",", ":")
        ),
        "filtered_candidate_indices_json": json.dumps(
            filtered_indices, separators=(",", ":")
        ),
        "selected_yolo_confidence_sum": sum(
            confidence_by_index[index] for index in selected_indices
        ),
        "physical_light_cluster_count": len(cluster_ids),
        "conflicts_json": json.dumps(conflicts, separators=(",", ":")),
        "largest_conflict_component_size": ownership[
            "largest_conflict_component_size"
        ],
        "exact_component_count": ownership["exact_component_count"],
        "overload_component_count": ownership["overload_component_count"],
        "exact_subsets_evaluated": ownership["exact_subsets_evaluated"],
        "reference_match": reference_match,
    }


def process_video(
    video: str,
    frame_candidates: dict[int, list[dict]],
    camera_matrix: np.ndarray,
    distortion: np.ndarray,
    v3_rows: dict[tuple, dict],
    conflict_reference: dict[tuple[str, int], dict],
) -> tuple[list[dict], list[dict], list[dict], dict]:
    capture = cv2.VideoCapture(str(video_path(video)))
    if not capture.isOpened():
        raise RuntimeError(f"cannot open {video_path(video)}")
    processor = runtime.RawTrackLatchProcessor()
    candidate_rows = []
    frame_rows = []
    conflict_rows = []
    last_track_types = {}
    type_jumps = 0
    largest_conflict_component_size = 0
    exact_component_count = 0
    overload_component_count = 0
    exact_subsets_evaluated = 0

    for frame_index in range(VIDEO_COUNTS[video]):
        ok, image = capture.read()
        if not ok or image is None:
            raise RuntimeError(f"cannot decode {video} frame {frame_index}")
        candidates = frame_candidates.get(frame_index, [])
        boxes = np.asarray(
            [candidate["box_xyxy"] for candidate in candidates], dtype=np.float32
        ).reshape(-1, 4)
        confidences = np.asarray(
            [candidate["yolo_confidence"] for candidate in candidates],
            dtype=np.float64,
        )
        results = processor.process_frame(
            image,
            boxes,
            frame_index,
            camera_matrix,
            distortion,
            confidences,
        )
        if len(results) != len(candidates):
            raise RuntimeError(f"candidate/result mismatch at {video}:{frame_index}")
        ownership = processor.last_ownership_result
        if ownership is None:
            raise RuntimeError(f"missing ownership result at {video}:{frame_index}")
        largest_conflict_component_size = max(
            largest_conflict_component_size,
            ownership["largest_conflict_component_size"],
        )
        exact_component_count += ownership["exact_component_count"]
        overload_component_count += ownership["overload_component_count"]
        exact_subsets_evaluated += ownership["exact_subsets_evaluated"]
        paired = list(zip(candidates, results))

        conflict_row = build_conflict_row(
            video,
            frame_index,
            candidates,
            results,
            ownership,
            conflict_reference.get((video, frame_index)),
        )
        if conflict_row is not None:
            conflict_rows.append(conflict_row)

        for candidate, result in paired:
            key = candidate_key(video, frame_index, candidate["candidate_index"])
            if key not in v3_rows:
                raise RuntimeError(f"candidate missing from parity references: {key}")
            candidate_rows.append(
                compare_candidate(
                    video,
                    frame_index,
                    candidate,
                    result,
                    v3_rows[key],
                )
            )
            latched = result["latched_type"]
            if latched is not None:
                previous = last_track_types.get(result["track_id"])
                if previous is not None and previous != latched:
                    type_jumps += 1
                last_track_types[result["track_id"]] = latched

        for truth_type in expected_truth_types(video):
            relevant = [item for item in paired if item[0]["truth_type"] == truth_type]
            correct = [
                item
                for item in relevant
                if item[1]["output_valid"] and item[1]["latched_type"] == truth_type
            ]
            frame_rows.append(
                {
                    "video": video,
                    "frame_index": frame_index,
                    "truth_type": truth_type,
                    "candidate_count": len(candidates),
                    "truth_candidate_count": len(relevant),
                    "valid_output_count_before_ownership": sum(
                        result["output_valid_before_ownership"]
                        for _, result in relevant
                    ),
                    "valid_output_count": sum(
                        result["output_valid"] for _, result in relevant
                    ),
                    "correct_type_valid_output_count": len(correct),
                    "frame_pass": bool(correct),
                    "failure_reason": (
                        "" if correct else frame_failure_reason(relevant)
                    ),
                }
            )

    extra_ok, _ = capture.read()
    capture.release()
    if extra_ok:
        raise RuntimeError(f"{video} has more than {VIDEO_COUNTS[video]} frames")

    return candidate_rows, frame_rows, conflict_rows, {
        "frames": VIDEO_COUNTS[video],
        "candidates": len(candidate_rows),
        "outputs_before_ownership": sum(
            parse_bool(row["output_valid_before_ownership"])
            for row in candidate_rows
        ),
        "outputs": sum(parse_bool(row["output_valid"]) for row in candidate_rows),
        "filtered_outputs": sum(
            parse_bool(row["output_valid_before_ownership"])
            and not parse_bool(row["output_valid"])
            for row in candidate_rows
        ),
        "conflict_frames": len(conflict_rows),
        "type_jumps": type_jumps,
        "tracks_created": processor.tracker.next_track_id - 1,
        "largest_conflict_component_size": largest_conflict_component_size,
        "exact_component_count": exact_component_count,
        "overload_component_count": overload_component_count,
        "exact_subsets_evaluated": exact_subsets_evaluated,
    }


def build_failure_runs(frame_rows: list[dict]) -> list[dict]:
    grouped = defaultdict(list)
    for row in frame_rows:
        if not row["frame_pass"]:
            grouped[(row["video"], row["truth_type"])].append(row)
    runs = []
    for (video, truth_type), rows in sorted(grouped.items()):
        current = []
        for row in rows:
            if current and row["frame_index"] != current[-1]["frame_index"] + 1:
                runs.append(current)
                current = []
            current.append(row)
        if current:
            runs.append(current)
    return [
        {
            "video": run[0]["video"],
            "truth_type": run[0]["truth_type"],
            "start_frame": run[0]["frame_index"],
            "end_frame": run[-1]["frame_index"],
            "length_frames": len(run),
            "failure_reasons_json": json.dumps(
                dict(sorted(Counter(row["failure_reason"] for row in run).items())),
                separators=(",", ":"),
            ),
        }
        for run in runs
    ]


def write_csv(path: Path, fields: list[str], rows: list[dict]) -> None:
    with path.open("x", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


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
    return ", ".join(
        str(start) if start == end else f"{start}-{end}"
        for start, end in intervals
    )


def synthetic_light(x: float) -> dict:
    top = np.asarray([x, 20.0], dtype=np.float64)
    bottom = np.asarray([x, 60.0], dtype=np.float64)
    return {"top": top, "bottom": bottom}


def synthetic_candidate(index: int, left: dict, right: dict) -> dict:
    return {
        "candidate_index": index,
        "yolo_confidence": 0.5 + index * 1e-6,
        "pair": {"left": left, "right": right},
        "output_valid": True,
    }


def run_performance_probes() -> dict:
    nonconflicting = [
        synthetic_candidate(
            index,
            synthetic_light(index * 1000.0),
            synthetic_light(index * 1000.0 + 100.0),
        )
        for index in range(300)
    ]
    started = time.perf_counter()
    nonconflicting_result = runtime.select_light_ownership(nonconflicting)
    nonconflicting_elapsed_ms = (time.perf_counter() - started) * 1000.0

    shared_left = synthetic_light(100.0)
    shared_right = synthetic_light(300.0)
    shared = [
        synthetic_candidate(index, shared_left, shared_right)
        for index in range(50)
    ]
    started = time.perf_counter()
    shared_result = runtime.select_light_ownership(shared)
    shared_elapsed_ms = (time.perf_counter() - started) * 1000.0

    return {
        "nonconflicting_300": {
            "candidate_count": 300,
            "elapsed_ms": nonconflicting_elapsed_ms,
            "selected_count": len(
                nonconflicting_result["selected_candidate_indices"]
            ),
            "largest_conflict_component_size": nonconflicting_result[
                "largest_conflict_component_size"
            ],
            "exact_component_count": nonconflicting_result[
                "exact_component_count"
            ],
            "overload_component_count": nonconflicting_result[
                "overload_component_count"
            ],
            "exact_subsets_evaluated": nonconflicting_result[
                "exact_subsets_evaluated"
            ],
        },
        "shared_light_50": {
            "candidate_count": 50,
            "elapsed_ms": shared_elapsed_ms,
            "selected_count": len(shared_result["selected_candidate_indices"]),
            "largest_conflict_component_size": shared_result[
                "largest_conflict_component_size"
            ],
            "exact_component_count": shared_result["exact_component_count"],
            "overload_component_count": shared_result[
                "overload_component_count"
            ],
            "overload_component_sizes": shared_result[
                "overload_component_sizes"
            ],
            "exact_subsets_evaluated": shared_result[
                "exact_subsets_evaluated"
            ],
        },
    }


def build_report(summary: dict) -> str:
    lines = [
        "# Raw Track Latch Ownership Runtime Parity V4",
        "",
        "The formal runtime processor decoded the frozen videos and replayed the same 6168 saved YOLO candidates. YOLO inference was not run.",
        "",
        "## Runtime Contract",
        "",
        f"- Ownership endpoint distance: <= {runtime.LIGHT_OWNERSHIP_ENDPOINT_DISTANCE_PX} px.",
        f"- Ownership undirected direction difference: <= {runtime.LIGHT_OWNERSHIP_DIRECTION_DIFFERENCE_DEG} deg.",
        f"- Exact selection is limited to conflict components of at most {runtime.MAX_EXACT_OWNERSHIP_CONFLICT_COMPONENT_SIZE} candidates.",
        "- Isolated candidates bypass enumeration; oversized components fail open.",
        "- Exact priority: maximum candidate count, maximum confidence sum, then lexicographic candidate index.",
        "- Ownership executes after tracking, type latching, and PnP.",
        "- The runner calls the formal runtime processor; it does not implement a second selector.",
        "",
        "## Frame Parity",
        "",
        "| Video | Truth | Expected | Actual | Difference | Failure frames |",
        "|---|---|---:|---:|---:|---|",
    ]
    for item in summary["metrics"]:
        lines.append(
            f"| `{item['video']}` | {item['truth_type']} | "
            f"{item['expected_pass_frames']} | {item['actual_pass_frames']} | "
            f"{item['difference']} | {compact_frames(item['failure_frames'])} |"
        )
    lines.extend(
        [
            "",
            "## Ownership Parity",
            "",
            f"- Candidates compared: {summary['candidate_count']}.",
            f"- Candidate field mismatches: {summary['candidate_mismatch_count']}.",
            f"- Conflict frames: {summary['conflict_frame_count']}.",
            f"- Largest frozen conflict component: {summary['frozen_largest_conflict_component_size']}.",
            f"- Frozen exact components/subsets evaluated: {summary['frozen_exact_component_count']}/{summary['frozen_exact_subsets_evaluated']}.",
            f"- Frozen-video overload components: {summary['frozen_overload_component_count']}.",
            f"- Filtered cross-target outputs: {summary['cross_target_filtered_count']}/192.",
            f"- Known angle/distance duplicate filters present: {summary['known_duplicate_filters_match']}.",
            f"- New matched failure frames: {summary['new_failure_frames']}.",
            f"- Type jumps: {summary['type_jump_count']}.",
            f"- Background candidates/outputs: {summary['background']['candidates']}/{summary['background']['outputs']}.",
            f"- Candidate promotion contract: {summary['candidate_promotion_contract']['valid']}.",
            "",
            "## Performance Probes",
            "",
            f"- 300 nonconflicting candidates: {summary['performance_probes']['nonconflicting_300']}.",
            f"- 50 shared-light candidates: {summary['performance_probes']['shared_light_50']}.",
            "",
            "## Conditions",
            "",
        ]
    )
    for name, passed in summary["conditions"].items():
        lines.append(f"- `{name}`: **{'PASS' if passed else 'FAIL'}**")
    lines.extend(
        [
            "",
            "## Decision",
            "",
            f"**{summary['decision_code']}**",
            "",
        ]
    )
    return "\n".join(lines)


def main() -> None:
    if OUTPUT.exists():
        raise RuntimeError(f"refusing to overwrite {OUTPUT}")

    v2_rows = load_csv_index(PARITY_V2 / "candidate_results.csv")
    v3_rows = load_csv_index(PARITY_V3 / "candidate_results.csv")
    conflict_reference = load_conflict_reference()
    if len(v2_rows) != 6168 or set(v2_rows) != set(v3_rows):
        raise RuntimeError("frozen parity reference candidate universe mismatch")

    camera_matrix, distortion = load_calibration()
    current_frames = load_current()
    prior_frames = load_prior()
    all_candidate_rows = []
    all_frame_rows = []
    all_conflict_rows = []
    video_summaries = {}

    for video in VIDEO_COUNTS:
        frames = current_frames if video == CURRENT_VIDEO else prior_frames[video]
        candidate_rows, frame_rows, conflict_rows, video_summary = process_video(
            video,
            frames,
            camera_matrix,
            distortion,
            v3_rows,
            conflict_reference,
        )
        all_candidate_rows.extend(candidate_rows)
        all_frame_rows.extend(frame_rows)
        all_conflict_rows.extend(conflict_rows)
        video_summaries[video] = video_summary

    runtime_keys = {
        candidate_key(row["video"], row["frame_index"], row["candidate_index"])
        for row in all_candidate_rows
    }
    if runtime_keys != set(v3_rows):
        raise RuntimeError("runtime candidate universe differs from frozen references")

    failure_runs = build_failure_runs(all_frame_rows)
    metrics = []
    failure_sets = {}
    for key, expected in EXPECTED_PASSES.items():
        video, truth_type = key
        rows = [
            row
            for row in all_frame_rows
            if row["video"] == video and row["truth_type"] == truth_type
        ]
        actual = sum(row["frame_pass"] for row in rows)
        failures = [row["frame_index"] for row in rows if not row["frame_pass"]]
        failure_sets[key] = failures
        metrics.append(
            {
                "video": video,
                "truth_type": truth_type,
                "expected_pass_frames": expected,
                "actual_pass_frames": actual,
                "difference": actual - expected,
                "failure_frames": failures,
            }
        )

    frame_count_conditions = {
        f"{Path(video).stem}_{truth_type}_{expected}": (
            next(
                item["actual_pass_frames"]
                for item in metrics
                if item["video"] == video and item["truth_type"] == truth_type
            )
            == expected
        )
        for (video, truth_type), expected in EXPECTED_PASSES.items()
    }
    mismatch_rows = [
        row for row in all_candidate_rows if not row["runtime_reference_match"]
    ]
    runtime_filtered = {
        candidate_key(row["video"], row["frame_index"], row["candidate_index"])
        for row in all_candidate_rows
        if parse_bool(row["output_valid_before_ownership"])
        and not parse_bool(row["output_valid"])
    }
    reference_filtered = {
        key
        for key, row in v3_rows.items()
        if parse_bool(row["output_valid_before_ownership"])
        and not parse_bool(row["output_valid"])
    }
    current_unmatched_baseline = {
        key
        for key, row in v2_rows.items()
        if key[0] == CURRENT_VIDEO
        and not row["truth_type"]
        and parse_bool(row["output_valid"])
    }
    current_unmatched_after = {
        candidate_key(row["video"], row["frame_index"], row["candidate_index"])
        for row in all_candidate_rows
        if row["video"] == CURRENT_VIDEO
        and not row["truth_type"]
        and parse_bool(row["output_valid"])
    }
    new_unmatched_outputs = {
        candidate_key(row["video"], row["frame_index"], row["candidate_index"])
        for row in all_candidate_rows
        if not row["truth_type"]
        and not parse_bool(v2_rows[candidate_key(
            row["video"], row["frame_index"], row["candidate_index"]
        )]["output_valid"])
        and parse_bool(row["output_valid"])
    }
    conflict_keys = {
        (row["video"], int(row["frame_index"])) for row in all_conflict_rows
    }
    type_jump_count = sum(item["type_jumps"] for item in video_summaries.values())
    intervals_match = all(
        failure_sets[key] == expected
        for key, expected in EXPECTED_FAILURE_FRAMES.items()
    )
    new_failure_frames = sum(
        len(set(failure_sets[key]) - set(expected))
        for key, expected in EXPECTED_FAILURE_FRAMES.items()
    )
    background = video_summaries["background_only.avi"]
    promotion_contract = candidate_promotion_contract()
    frozen_overload_component_count = sum(
        item["overload_component_count"] for item in video_summaries.values()
    )
    frozen_largest_conflict_component_size = max(
        item["largest_conflict_component_size"]
        for item in video_summaries.values()
    )
    frozen_exact_component_count = sum(
        item["exact_component_count"] for item in video_summaries.values()
    )
    frozen_exact_subsets_evaluated = sum(
        item["exact_subsets_evaluated"] for item in video_summaries.values()
    )
    performance_probes = run_performance_probes()
    nonconflicting_probe = performance_probes["nonconflicting_300"]
    overload_probe = performance_probes["shared_light_50"]
    conflict_rows_match = (
        conflict_keys == set(conflict_reference)
        and all(parse_bool(row["reference_match"]) for row in all_conflict_rows)
    )
    conditions = {
        **frame_count_conditions,
        "candidate_universe_6168": len(all_candidate_rows) == 6168,
        "all_candidate_runtime_fields_match_parity_v3": (
            not mismatch_rows
        ),
        "final_output_exactly_matches_parity_v3": (
            runtime_filtered == reference_filtered
        ),
        "conflict_frames_exactly_match_reference_196": (
            len(all_conflict_rows) == 196 and conflict_rows_match
        ),
        "cross_target_outputs_filtered_192_of_192": (
            len(current_unmatched_baseline) == 192
            and current_unmatched_baseline <= runtime_filtered
            and not current_unmatched_after
        ),
        "known_angle_distance_duplicates_filtered": (
            KNOWN_DUPLICATE_FILTERS <= runtime_filtered
        ),
        "no_new_truth_failure_frames_or_intervals": (
            intervals_match and new_failure_frames == 0
        ),
        "background_zero_candidates_outputs": (
            background["candidates"] == 0 and background["outputs"] == 0
        ),
        "locked_type_jump_count_zero": type_jump_count == 0,
        "no_new_unmatched_valid_outputs": not new_unmatched_outputs,
        "frozen_video_overload_component_count_zero": (
            frozen_overload_component_count == 0
        ),
        "performance_probe_300_nonconflicting_bypasses_enumeration": (
            nonconflicting_probe["selected_count"] == 300
            and nonconflicting_probe["exact_component_count"] == 0
            and nonconflicting_probe["overload_component_count"] == 0
            and nonconflicting_probe["exact_subsets_evaluated"] == 0
        ),
        "performance_probe_50_shared_lights_fails_open": (
            overload_probe["selected_count"] == 50
            and overload_probe["largest_conflict_component_size"] == 50
            and overload_probe["overload_component_count"] == 1
            and overload_probe["exact_subsets_evaluated"] == 0
        ),
        "candidate_promotion_contract_valid": promotion_contract["valid"],
    }
    decision_code = (
        "RAW_TRACK_LATCH_OWNERSHIP_BOUNDED_RUNTIME_PASS"
        if all(conditions.values())
        else "RAW_TRACK_LATCH_OWNERSHIP_BOUNDED_RUNTIME_FAIL"
    )
    summary = {
        "schema_version": 1,
        "generated_at": datetime.now().astimezone().isoformat(),
        "runtime_module": str(PACKAGE / "scripts/raw_track_latch.py"),
        "inputs": {
            "current_saved_candidates": str(CURRENT_INPUT),
            "prior_saved_candidates": str(PRIOR_INPUT),
            "parity_v2": str(PARITY_V2 / "candidate_results.csv"),
            "parity_v3": str(PARITY_V3 / "candidate_results.csv"),
            "calibration": str(CALIBRATION),
            "yolo_rerun": False,
        },
        "runtime_parameters": {
            "ownership_endpoint_distance_px_inclusive": (
                runtime.LIGHT_OWNERSHIP_ENDPOINT_DISTANCE_PX
            ),
            "ownership_direction_difference_deg_inclusive": (
                runtime.LIGHT_OWNERSHIP_DIRECTION_DIFFERENCE_DEG
            ),
            "maximum_exact_conflict_component_size": (
                runtime.MAX_EXACT_OWNERSHIP_CONFLICT_COMPONENT_SIZE
            ),
            "overload_behavior": "fail_open",
            "roi_expansion_fraction_per_side": (
                runtime.ROI_EXPANSION_FRACTION_PER_SIDE
            ),
            "width_length_range": [
                runtime.MIN_WIDTH_LENGTH_RATIO,
                runtime.MAX_WIDTH_LENGTH_RATIO,
            ],
            "type_boundary": runtime.TYPE_BOUNDARY,
            "track_iou_threshold": runtime.TRACK_IOU_THRESHOLD,
        },
        "candidate_count": len(all_candidate_rows),
        "candidate_mismatch_count": len(mismatch_rows),
        "candidate_mismatch_keys": [
            [row["video"], row["frame_index"], row["candidate_index"]]
            for row in mismatch_rows
        ],
        "conflict_frame_count": len(all_conflict_rows),
        "conflict_frames_match_reference": conflict_rows_match,
        "frozen_largest_conflict_component_size": (
            frozen_largest_conflict_component_size
        ),
        "frozen_exact_component_count": frozen_exact_component_count,
        "frozen_exact_subsets_evaluated": frozen_exact_subsets_evaluated,
        "frozen_overload_component_count": frozen_overload_component_count,
        "filtered_output_count": len(runtime_filtered),
        "cross_target_filtered_count": len(
            current_unmatched_baseline & runtime_filtered
        ),
        "current_unmatched_outputs_after": len(current_unmatched_after),
        "new_unmatched_output_count": len(new_unmatched_outputs),
        "known_duplicate_filters_match": (
            KNOWN_DUPLICATE_FILTERS <= runtime_filtered
        ),
        "known_duplicate_filter_keys": [list(key) for key in sorted(
            KNOWN_DUPLICATE_FILTERS & runtime_filtered
        )],
        "metrics": metrics,
        "video_summaries": video_summaries,
        "failure_runs": failure_runs,
        "failure_intervals_match_reference": intervals_match,
        "new_failure_frames": new_failure_frames,
        "type_jump_count": type_jump_count,
        "background": background,
        "candidate_promotion_contract": promotion_contract,
        "performance_probes": performance_probes,
        "conditions": conditions,
        "decision_code": decision_code,
    }

    OUTPUT.mkdir(parents=True)
    write_csv(OUTPUT / "candidate_results.csv", CANDIDATE_FIELDS, all_candidate_rows)
    write_csv(OUTPUT / "frame_results.csv", FRAME_FIELDS, all_frame_rows)
    write_csv(OUTPUT / "conflict_results.csv", CONFLICT_FIELDS, all_conflict_rows)
    write_csv(OUTPUT / "failure_runs.csv", RUN_FIELDS, failure_runs)
    with (OUTPUT / "summary.json").open("x", encoding="utf-8") as stream:
        json.dump(summary, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
    (OUTPUT / "report.md").write_text(build_report(summary), encoding="utf-8")
    print(
        json.dumps(
            {
                "candidate_count": summary["candidate_count"],
                "candidate_mismatch_count": summary["candidate_mismatch_count"],
                "conflict_frame_count": summary["conflict_frame_count"],
                "filtered_output_count": summary["filtered_output_count"],
                "cross_target_filtered_count": summary[
                    "cross_target_filtered_count"
                ],
                "metrics": metrics,
                "conditions": conditions,
                "decision_code": decision_code,
                "output": str(OUTPUT),
            },
            ensure_ascii=False,
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
