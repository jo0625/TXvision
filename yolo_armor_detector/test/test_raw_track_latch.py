#!/usr/bin/env python3

from __future__ import annotations

import ast
import sys
from pathlib import Path

import cv2
import numpy as np
import pytest


PACKAGE = Path(__file__).resolve().parents[1]
SCRIPTS = PACKAGE / "scripts"
sys.path.insert(0, str(SCRIPTS))

import raw_track_latch as latch  # noqa: E402


def make_light(center_x: float, length: float = 40.0) -> dict:
    top = np.asarray([center_x, 20.0], dtype=np.float32)
    bottom = np.asarray([center_x, 20.0 + length], dtype=np.float32)
    return {
        "top": top,
        "bottom": bottom,
        "center": (top + bottom) / 2.0,
        "length_px": length,
        "width_px": 5.0,
        "width_length_ratio": 0.125,
        "tilt_deg": 0.0,
        "fill_ratio": 1.0,
        "contour_pixel_count": 200,
    }


def make_oriented_light(
    top_x: float,
    top_y: float = 20.0,
    length: float = 40.0,
    angle_from_vertical_deg: float = 0.0,
) -> dict:
    angle = np.deg2rad(angle_from_vertical_deg)
    top = np.asarray([top_x, top_y], dtype=np.float64)
    bottom = top + np.asarray(
        [np.sin(angle) * length, np.cos(angle) * length], dtype=np.float64
    )
    return {
        "top": top,
        "bottom": bottom,
        "center": (top + bottom) / 2.0,
        "length_px": length,
    }


def make_ownership_pair(left: dict, right: dict) -> dict:
    return {
        "left": left,
        "right": right,
        "points_xy": np.asarray(
            [left["top"], left["bottom"], right["bottom"], right["top"]],
            dtype=np.float32,
        ),
    }


def ownership_candidate(
    candidate_index: int,
    confidence: float,
    left: dict,
    right: dict,
    output_valid: bool = True,
) -> dict:
    return {
        "candidate_index": candidate_index,
        "yolo_confidence": confidence,
        "pair": make_ownership_pair(left, right),
        "output_valid": output_valid,
    }


def make_processor_pair(
    left_x: float = 100.0,
    right_x: float = 300.0,
    length: float = 100.0,
    raw_ratio: float = 2.0,
) -> dict:
    pair = make_ownership_pair(
        make_oriented_light(left_x, length=length),
        make_oriented_light(right_x, length=length),
    )
    pair.update(
        {
            "normalized_center_distance": raw_ratio,
            "raw_type": "small" if raw_ratio < latch.TYPE_BOUNDARY else "large",
        }
    )
    return pair


def fake_pnp(mean_error: float, solved: bool = True, valid: bool = True) -> dict:
    return {
        "solved": solved,
        "valid": valid if solved else False,
        "failure_reason": "" if solved and valid else "reprojection_error",
        "mean_reprojection_error_px": mean_error if solved else None,
        "max_reprojection_error_px": mean_error if solved else None,
        "distance_m": 1.0 if solved else None,
        "rvec": np.zeros(3, dtype=np.float64) if solved else None,
        "tvec": np.asarray([0.0, 0.0, 1.0]) if solved else None,
    }


def test_fixed_runtime_constants() -> None:
    assert latch.ROI_EXPANSION_FRACTION_PER_SIDE == 0.10
    assert latch.GRAY_THRESHOLD == 160
    assert latch.MIN_WIDTH_LENGTH_RATIO == 0.06
    assert latch.MAX_WIDTH_LENGTH_RATIO == 0.40
    assert latch.MAX_TILT_DEG == 35.0
    assert latch.MIN_FILL_RATIO == 0.80
    assert latch.MIN_PAIR_LIGHT_LENGTH_RATIO == 0.50
    assert latch.MAX_PAIR_CENTER_ANGLE_DEG == 45.0
    assert latch.MIN_NORMALIZED_CENTER_DISTANCE == 0.80
    assert latch.MAX_NORMALIZED_CENTER_DISTANCE == 5.50
    assert latch.TYPE_BOUNDARY == 3.20
    assert latch.NORMALIZED_DUAL_PNP_TYPE_BOUNDARY == 0.0326
    assert latch.TRACK_IOU_THRESHOLD == 0.30
    assert latch.MIN_PNP_DISTANCE_M == 0.10
    assert latch.MAX_PNP_DISTANCE_M == 10.0
    assert latch.MAX_MEAN_REPROJECTION_ERROR_PX == 12.0
    assert latch.LIGHT_OWNERSHIP_ENDPOINT_DISTANCE_PX == 6.0
    assert latch.LIGHT_OWNERSHIP_DIRECTION_DIFFERENCE_DEG == 5.0
    assert latch.MAX_EXACT_OWNERSHIP_CONFLICT_COMPONENT_SIZE == 12
    assert latch.LIGHT_GEOMETRY_M == {
        "small": (0.132, 0.056),
        "large": (0.228, 0.053),
    }


def test_gray_threshold_defaults_to_160_and_is_instance_scoped(monkeypatch) -> None:
    observed = []

    def fake_extract(_image, _box, gray_threshold):
        observed.append(gray_threshold)
        return {"pair": None, "failure_reason": "unable_to_pair:insufficient_lights"}

    monkeypatch.setattr(latch, "extract_light_pair", fake_extract)
    default = latch.RawTrackLatchProcessor()
    candidate = latch.RawTrackLatchProcessor(gray_threshold=140)
    image = np.zeros((20, 20, 3), dtype=np.uint8)
    boxes = np.asarray([[0.0, 0.0, 10.0, 10.0]])

    default.process_frame(image, boxes, 0, None, None)
    candidate.process_frame(image, boxes, 0, None, None)
    default.process_frame(image, boxes, 1, None, None)

    assert default.gray_threshold == 160
    assert candidate.gray_threshold == 140
    assert observed == [160, 140, 160]
    assert latch.GRAY_THRESHOLD == 160


@pytest.mark.parametrize("value", [-1, 256, 140.0, "140", True])
def test_gray_threshold_rejects_non_integer_or_out_of_range_values(value) -> None:
    with pytest.raises(ValueError, match=r"integer in \[0, 255\]"):
        latch.RawTrackLatchProcessor(gray_threshold=value)


@pytest.mark.parametrize("value", [0, 255])
def test_gray_threshold_accepts_inclusive_bounds(value) -> None:
    assert latch.RawTrackLatchProcessor(gray_threshold=value).gray_threshold == value


def test_expanded_roi_uses_ten_percent_per_side() -> None:
    roi = latch.expanded_roi(np.asarray([100, 100, 200, 200]), 640, 480)
    assert roi == (90, 90, 210, 210)


def test_pair_selection_and_exact_type_boundary() -> None:
    left = make_light(100.0)
    right = make_light(228.0)
    pair, pair_count, rejections = latch.pair_lights([left, right])
    assert not rejections
    assert pair_count == 1
    assert pair is not None
    assert np.isclose(pair["normalized_center_distance"], 3.2)
    assert pair["raw_type"] == "large"


def test_traditional_extraction_finds_two_lights() -> None:
    image = np.zeros((200, 400, 3), dtype=np.uint8)
    for center_x in (100, 300):
        polygon = np.asarray(
            [
                [center_x - 4, 60],
                [center_x + 4, 60],
                [center_x + 6, 62],
                [center_x + 6, 138],
                [center_x + 4, 140],
                [center_x - 4, 140],
                [center_x - 6, 138],
                [center_x - 6, 62],
            ],
            dtype=np.int32,
        )
        cv2.fillPoly(image, [polygon], (255, 255, 255))
    result = latch.extract_light_pair(
        image, np.asarray([80.0, 40.0, 320.0, 160.0])
    )
    assert result["failure_reason"] == ""
    assert result["valid_light_count"] == 2
    assert result["valid_pair_count"] == 1
    assert result["pair"]["raw_type"] == "small"


def test_gray_160_keeps_contiguous_dim_tails_without_background_pair() -> None:
    image = np.zeros((220, 360, 3), dtype=np.uint8)
    for center_x in (100, 260):
        outer = np.asarray(
            [
                [center_x - 4, 40],
                [center_x + 4, 40],
                [center_x + 6, 42],
                [center_x + 6, 138],
                [center_x + 4, 140],
                [center_x - 4, 140],
                [center_x - 6, 138],
                [center_x - 6, 42],
            ],
            dtype=np.int32,
        )
        cv2.fillPoly(image, [outer], (175, 175, 175))
        cv2.rectangle(
            image,
            (center_x - 5, 60),
            (center_x + 5, 120),
            (255, 255, 255),
            -1,
        )

    # This separate dim background contour must be rejected as a light.
    cv2.rectangle(image, (175, 85), (190, 100), (175, 175, 175), -1)

    result = latch.extract_light_pair(
        image, np.asarray([70.0, 25.0, 290.0, 155.0])
    )

    assert result["failure_reason"] == ""
    assert result["raw_contour_count"] == 3
    assert result["valid_light_count"] == 2
    assert result["valid_pair_count"] == 1
    pair = result["pair"]
    assert pair is not None
    assert pair["raw_type"] == "small"
    assert pair["normalized_center_distance"] < latch.TYPE_BOUNDARY
    for light in (pair["left"], pair["right"]):
        assert np.isclose(light["top"][1], 40.0)
        assert np.isclose(light["bottom"][1], 141.0)
        assert light["length_px"] > 100.0


def test_single_valid_light_has_explicit_pair_failure() -> None:
    image = np.zeros((200, 200, 3), dtype=np.uint8)
    polygon = np.asarray(
        [[96, 60], [104, 60], [106, 62], [106, 138], [104, 140],
         [96, 140], [94, 138], [94, 62]],
        dtype=np.int32,
    )
    cv2.fillPoly(image, [polygon], (255, 255, 255))
    result = latch.extract_light_pair(
        image, np.asarray([80.0, 40.0, 120.0, 160.0])
    )
    assert result["valid_light_count"] == 1
    assert result["failure_reason"] == "unable_to_pair:insufficient_lights"


def test_causal_greedy_tracking_latches_and_does_not_transfer_history() -> None:
    tracker = latch.CausalTypeLatch()
    box_a = np.asarray([0.0, 0.0, 100.0, 100.0])
    box_b = np.asarray([200.0, 0.0, 300.0, 100.0])
    assignments, ended = tracker.assign([box_a, box_b], 0)
    assert not ended
    assert [item[0].track_id for item in assignments] == [1, 2]
    assert tracker.initialize(assignments[0][0], 2.5, 0)
    assert tracker.initialize(assignments[1][0], 4.0, 0)

    assignments, ended = tracker.assign(
        [box_b + np.asarray([2.0, 0.0, 2.0, 0.0]), box_a + 1.0], 1
    )
    assert not ended
    assert [item[0].track_id for item in assignments] == [2, 1]
    assert not tracker.initialize(assignments[0][0], 2.0, 1)
    assert not tracker.initialize(assignments[1][0], 4.0, 1)
    assert assignments[0][0].latched_type == "large"
    assert assignments[1][0].latched_type == "small"

    assignments, ended = tracker.assign([box_a + 2.0], 2)
    assert [item[0].track_id for item in assignments] == [1]
    assert ended == [2]
    assignments, ended = tracker.assign([box_b], 3)
    assert [item[0].track_id for item in assignments] == [3]
    assert assignments[0][0].latched_type is None
    assert ended == [1]
    assert tracker.initialize(assignments[0][0], 4.2, 3)
    assert assignments[0][0].latched_type == "large"


def test_light_pnp_recovers_valid_small_pose(monkeypatch) -> None:
    camera_matrix = np.asarray(
        [[1740.0, 0.0, 696.0], [0.0, 1740.0, 548.0], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )
    distortion = np.zeros((5, 1), dtype=np.float64)
    object_points = latch._object_points("small")
    rvec = np.asarray([0.08, -0.12, 0.03], dtype=np.float64)
    tvec = np.asarray([0.02, -0.01, 1.2], dtype=np.float64)
    projected, _ = cv2.projectPoints(
        object_points, rvec, tvec, camera_matrix, distortion
    )
    pnp_points = projected.reshape(4, 2)
    semantic_points = np.asarray(
        [pnp_points[1], pnp_points[0], pnp_points[3], pnp_points[2]],
        dtype=np.float32,
    )
    result = latch.solve_light_pnp(
        semantic_points, "small", camera_matrix, distortion
    )
    assert result["solved"]
    assert result["valid"]
    assert result["mean_reprojection_error_px"] < 1e-3
    assert np.isclose(result["distance_m"], np.linalg.norm(tvec), atol=1e-3)
    timing_observations = []
    monkeypatch.setattr(latch, "PNP_TIMING_SINK", timing_observations.append)
    observed = latch.solve_light_pnp(semantic_points, "small", camera_matrix, distortion)
    assert timing_observations == [True]
    np.testing.assert_equal(observed, result)


def test_normalized_dual_pnp_uses_fixed_boundary_and_ignores_runtime_validity() -> None:
    points = make_processor_pair(length=100.0)["points_xy"]
    below = latch.normalized_dual_pnp_decision_from_results(
        points,
        fake_pnp(1.0, valid=False),
        fake_pnp(1.0 + (latch.NORMALIZED_DUAL_PNP_TYPE_BOUNDARY - 1e-6) * 100.0),
    )
    above = latch.normalized_dual_pnp_decision_from_results(
        points,
        fake_pnp(1.0),
        fake_pnp(1.0 + (latch.NORMALIZED_DUAL_PNP_TYPE_BOUNDARY + 1e-6) * 100.0),
    )

    assert below["evaluable"]
    assert below["type"] == "small"
    assert not below["small_pnp"]["valid"]
    assert above["evaluable"]
    assert above["type"] == "large"


def test_normalized_dual_pnp_unsolved_is_undecided_without_raw_fallback(
    monkeypatch,
) -> None:
    points = make_processor_pair(raw_ratio=2.0)["points_xy"]
    decision = latch.normalized_dual_pnp_decision_from_results(
        points,
        fake_pnp(0.0, solved=False),
        fake_pnp(4.0),
    )
    assert not decision["evaluable"]
    assert decision["type"] is None
    assert decision["score"] is None
    assert decision["failure_reason"] == "small_pnp_unsolved"

    pair = make_processor_pair(raw_ratio=2.0)
    monkeypatch.setattr(
        latch,
        "extract_light_pair",
        lambda _image, _box, _gray_threshold: {"pair": pair, "failure_reason": ""},
    )
    monkeypatch.setattr(
        latch,
        "solve_normalized_dual_pnp_type",
        lambda _points, _matrix, _distortion: decision,
    )
    processor = latch.RawTrackLatchProcessor("normalized_dual_pnp")
    result = processor.process_frame(
        np.zeros((200, 400, 3), dtype=np.uint8),
        np.asarray([[0.0, 0.0, 10.0, 10.0]]),
        0,
        np.eye(3),
        np.zeros((5, 1)),
        np.asarray([0.9]),
    )[0]

    assert result["raw_type"] == "small"
    assert result["latched_type"] is None
    assert result["pnp"] is None
    assert not result["output_valid"]
    assert result["failure_reason"] == (
        "normalized_dual_pnp_unavailable:small_pnp_unsolved"
    )


def test_default_processor_still_initializes_from_raw_ratio(monkeypatch) -> None:
    pair = make_processor_pair(raw_ratio=4.5)
    monkeypatch.setattr(
        latch,
        "extract_light_pair",
        lambda _image, _box, _gray_threshold: {"pair": pair, "failure_reason": ""},
    )
    monkeypatch.setattr(
        latch,
        "solve_light_pnp",
        lambda _points, _armor_type, _matrix, _distortion: fake_pnp(1.0),
    )
    processor = latch.RawTrackLatchProcessor()
    result = processor.process_frame(
        np.zeros((20, 20, 3), dtype=np.uint8),
        np.asarray([[0.0, 0.0, 10.0, 10.0]]),
        0,
        np.eye(3),
        np.zeros((5, 1)),
        np.asarray([0.9]),
    )[0]

    assert processor.type_initialization_strategy == "raw_ratio"
    assert result["latched_type"] == "large"
    assert result["initialization_metric"] == "raw_ratio"
    assert np.isclose(result["initialization_value"], 4.5)


def test_normalized_dual_pnp_type_is_latched_once(monkeypatch) -> None:
    pair = make_processor_pair(raw_ratio=4.5)
    first_decision = {
        "evaluable": True,
        "failure_reason": "",
        "score": 0.02,
        "type": "small",
        "mean_light_length_px": 100.0,
        "left_light_length_px": 100.0,
        "right_light_length_px": 100.0,
        "small_pnp": fake_pnp(1.0),
        "large_pnp": fake_pnp(3.0),
    }
    decision_calls = []
    monkeypatch.setattr(
        latch,
        "extract_light_pair",
        lambda _image, _box, _gray_threshold: {"pair": pair, "failure_reason": ""},
    )

    def decide(*_args):
        decision_calls.append(True)
        return first_decision

    monkeypatch.setattr(latch, "solve_normalized_dual_pnp_type", decide)
    monkeypatch.setattr(
        latch,
        "solve_light_pnp",
        lambda _points, _armor_type, _matrix, _distortion: fake_pnp(9.0),
    )
    processor = latch.RawTrackLatchProcessor("normalized_dual_pnp")
    box = np.asarray([[0.0, 0.0, 10.0, 10.0]])
    first = processor.process_frame(
        np.zeros((200, 400, 3), dtype=np.uint8),
        box,
        0,
        np.eye(3),
        np.zeros((5, 1)),
        np.asarray([0.9]),
    )[0]
    second = processor.process_frame(
        np.zeros((200, 400, 3), dtype=np.uint8),
        box,
        1,
        np.eye(3),
        np.zeros((5, 1)),
        np.asarray([0.9]),
    )[0]

    assert first["raw_type"] == "large"
    assert first["latched_type"] == "small"
    assert first["initialized_this_frame"]
    assert first["initialization_metric"] == "normalized_dual_pnp"
    assert np.isclose(first["initialization_value"], 0.02)
    assert second["latched_type"] == "small"
    assert not second["initialized_this_frame"]
    assert second["normalized_dual_pnp_score"] is None
    assert len(decision_calls) == 1


def test_normalized_dual_pnp_pnp_precedes_ownership_filter(monkeypatch) -> None:
    pair = make_processor_pair(raw_ratio=2.0)
    decision = {
        "evaluable": True,
        "failure_reason": "",
        "score": 0.01,
        "type": "small",
        "mean_light_length_px": 100.0,
        "left_light_length_px": 100.0,
        "right_light_length_px": 100.0,
        "small_pnp": fake_pnp(1.0),
        "large_pnp": fake_pnp(2.0),
    }
    monkeypatch.setattr(
        latch,
        "extract_light_pair",
        lambda _image, _box, _gray_threshold: {"pair": pair, "failure_reason": ""},
    )
    monkeypatch.setattr(
        latch,
        "solve_normalized_dual_pnp_type",
        lambda *_args: decision,
    )
    processor = latch.RawTrackLatchProcessor("normalized_dual_pnp")
    results = processor.process_frame(
        np.zeros((200, 400, 3), dtype=np.uint8),
        np.asarray([[0.0, 0.0, 10.0, 10.0], [15.0, 0.0, 25.0, 10.0]]),
        0,
        np.eye(3),
        np.zeros((5, 1)),
        np.asarray([0.4, 0.9]),
    )

    assert all(result["latched_type"] == "small" for result in results)
    assert all(result["pnp"] is decision["small_pnp"] for result in results)
    assert all(result["output_valid_before_ownership"] for result in results)
    assert not results[0]["ownership_selected"]
    assert results[0]["failure_reason"] == "light_ownership_conflict"
    assert results[0]["pnp"] is decision["small_pnp"]
    assert results[1]["ownership_selected"]
    assert results[1]["output_valid"]


def test_normalized_dual_pnp_initialization_rejects_each_image_boundary(
    monkeypatch,
) -> None:
    image = np.zeros((100, 200, 3), dtype=np.uint8)
    base = np.asarray(
        [[20.0, 10.0], [20.0, 90.0], [180.0, 90.0], [180.0, 10.0]],
        dtype=np.float32,
    )
    invalid_points = []
    for point_index, coordinate_index, value in (
        (0, 0, -0.001),
        (2, 0, 200.0),
        (0, 1, -0.001),
        (1, 1, 100.0),
    ):
        points = base.copy()
        points[point_index, coordinate_index] = value
        invalid_points.append(points)

    solve_calls = []
    monkeypatch.setattr(
        latch,
        "solve_normalized_dual_pnp_type",
        lambda *_args: solve_calls.append(True),
    )
    for points in invalid_points:
        pair = make_processor_pair(raw_ratio=2.0)
        pair["points_xy"] = points
        monkeypatch.setattr(
            latch,
            "extract_light_pair",
            lambda _image, _box, _gray_threshold, current=pair: {
                "pair": current,
                "failure_reason": "",
            },
        )
        result = latch.RawTrackLatchProcessor(
            "normalized_dual_pnp"
        ).process_frame(
            image,
            np.asarray([[0.0, 0.0, 10.0, 10.0]]),
            0,
            np.eye(3),
            np.zeros((5, 1)),
            np.asarray([0.9]),
        )[0]
        assert result["latched_type"] is None
        assert result["pnp"] is None
        assert not result["output_valid"]
        assert result["failure_reason"] == (
            "type_initialization_points_out_of_bounds"
        )
    assert not solve_calls


def test_normalized_dual_pnp_initialization_accepts_exact_lower_bounds(
    monkeypatch,
) -> None:
    image = np.zeros((100, 200, 3), dtype=np.uint8)
    pair = make_processor_pair(raw_ratio=4.0)
    pair["points_xy"] = np.asarray(
        [[0.0, 0.0], [0.0, 99.999], [199.999, 99.999], [199.999, 0.0]],
        dtype=np.float32,
    )
    decision = {
        "evaluable": True,
        "failure_reason": "",
        "score": 0.01,
        "type": "small",
        "small_pnp": fake_pnp(1.0),
        "large_pnp": fake_pnp(2.0),
    }
    solve_calls = []
    monkeypatch.setattr(
        latch,
        "extract_light_pair",
        lambda _image, _box, _gray_threshold: {"pair": pair, "failure_reason": ""},
    )
    monkeypatch.setattr(
        latch,
        "solve_normalized_dual_pnp_type",
        lambda *_args: solve_calls.append(True) or decision,
    )
    result = latch.RawTrackLatchProcessor("normalized_dual_pnp").process_frame(
        image,
        np.asarray([[0.0, 0.0, 10.0, 10.0]]),
        7,
        np.eye(3),
        np.zeros((5, 1)),
        np.asarray([0.9]),
    )[0]
    assert latch.points_in_image_bounds(pair["points_xy"], 200, 100)
    assert len(solve_calls) == 1
    assert result["latched_type"] == "small"
    assert result["initialization_frame"] == 7
    assert result["output_valid"]


def test_normalized_dual_pnp_waits_for_next_in_bounds_frame(monkeypatch) -> None:
    image = np.zeros((100, 200, 3), dtype=np.uint8)
    outside = make_processor_pair(raw_ratio=4.0)
    outside["points_xy"] = np.asarray(
        [[20.0, 10.0], [20.0, 100.0], [180.0, 100.0], [180.0, 10.0]],
        dtype=np.float32,
    )
    inside = make_processor_pair(raw_ratio=4.0)
    inside["points_xy"] = np.asarray(
        [[20.0, 10.0], [20.0, 99.0], [180.0, 99.0], [180.0, 10.0]],
        dtype=np.float32,
    )
    pairs = iter((outside, inside))
    solve_calls = []
    decision = {
        "evaluable": True,
        "failure_reason": "",
        "score": 0.0013596693880084721,
        "type": "small",
        "small_pnp": fake_pnp(1.0),
        "large_pnp": fake_pnp(2.0),
    }
    monkeypatch.setattr(
        latch,
        "extract_light_pair",
        lambda _image, _box, _gray_threshold: {
            "pair": next(pairs),
            "failure_reason": "",
        },
    )
    monkeypatch.setattr(
        latch,
        "solve_normalized_dual_pnp_type",
        lambda *_args: solve_calls.append(True) or decision,
    )
    processor = latch.RawTrackLatchProcessor("normalized_dual_pnp")
    box = np.asarray([[0.0, 0.0, 10.0, 10.0]])
    blocked = processor.process_frame(
        image, box, 19985, np.eye(3), np.zeros((5, 1)), np.asarray([0.9])
    )[0]
    initialized = processor.process_frame(
        image, box, 19988, np.eye(3), np.zeros((5, 1)), np.asarray([0.9])
    )[0]
    assert blocked["latched_type"] is None
    assert not blocked["output_valid"]
    assert blocked["failure_reason"] == (
        "type_initialization_points_out_of_bounds"
    )
    assert len(solve_calls) == 1
    assert initialized["latched_type"] == "small"
    assert initialized["initialized_this_frame"]
    assert initialized["initialization_frame"] == 19988


def test_raw_ratio_initialization_ignores_image_bounds_gate(monkeypatch) -> None:
    pair = make_processor_pair(raw_ratio=4.5)
    assert not latch.points_in_image_bounds(pair["points_xy"], 20, 20)
    monkeypatch.setattr(
        latch,
        "extract_light_pair",
        lambda _image, _box, _gray_threshold: {"pair": pair, "failure_reason": ""},
    )
    monkeypatch.setattr(
        latch,
        "solve_light_pnp",
        lambda *_args: fake_pnp(1.0),
    )
    result = latch.RawTrackLatchProcessor("raw_ratio").process_frame(
        np.zeros((20, 20, 3), dtype=np.uint8),
        np.asarray([[0.0, 0.0, 10.0, 10.0]]),
        0,
        np.eye(3),
        np.zeros((5, 1)),
        np.asarray([0.9]),
    )[0]
    assert result["latched_type"] == "large"
    assert result["initialization_metric"] == "raw_ratio"
    assert result["output_valid"]


def test_latched_normalized_track_ignores_later_out_of_bounds_points(
    monkeypatch,
) -> None:
    image = np.zeros((100, 200, 3), dtype=np.uint8)
    inside = make_processor_pair(raw_ratio=2.0)
    inside["points_xy"] = np.asarray(
        [[20.0, 10.0], [20.0, 90.0], [180.0, 90.0], [180.0, 10.0]],
        dtype=np.float32,
    )
    outside = make_processor_pair(raw_ratio=4.5)
    outside["points_xy"] = inside["points_xy"].copy()
    outside["points_xy"][2, 0] = 200.0
    pairs = iter((inside, outside))
    decision_calls = []
    selected_pnp_calls = []
    decision = {
        "evaluable": True,
        "failure_reason": "",
        "score": 0.01,
        "type": "small",
        "small_pnp": fake_pnp(1.0),
        "large_pnp": fake_pnp(2.0),
    }
    monkeypatch.setattr(
        latch,
        "extract_light_pair",
        lambda _image, _box, _gray_threshold: {
            "pair": next(pairs),
            "failure_reason": "",
        },
    )
    monkeypatch.setattr(
        latch,
        "solve_normalized_dual_pnp_type",
        lambda *_args: decision_calls.append(True) or decision,
    )
    monkeypatch.setattr(
        latch,
        "solve_light_pnp",
        lambda *_args: selected_pnp_calls.append(True) or fake_pnp(1.0),
    )
    processor = latch.RawTrackLatchProcessor("normalized_dual_pnp")
    box = np.asarray([[0.0, 0.0, 10.0, 10.0]])
    first = processor.process_frame(
        image, box, 0, np.eye(3), np.zeros((5, 1)), np.asarray([0.9])
    )[0]
    second = processor.process_frame(
        image, box, 1, np.eye(3), np.zeros((5, 1)), np.asarray([0.9])
    )[0]
    assert first["latched_type"] == "small"
    assert second["latched_type"] == "small"
    assert second["initialization_frame"] == 0
    assert second["output_valid"]
    assert len(decision_calls) == 1
    assert len(selected_pnp_calls) == 1


def test_light_ownership_six_pixel_and_five_degree_boundaries() -> None:
    base = make_oriented_light(100.0)
    exactly_six = make_oriented_light(106.0)
    over_six = make_oriented_light(106.0001)
    exactly_five = make_oriented_light(100.0, angle_from_vertical_deg=5.0)
    over_five = make_oriented_light(100.0, angle_from_vertical_deg=5.0001)

    assert latch.same_physical_light(base, exactly_six)
    assert not latch.same_physical_light(base, over_six)
    assert np.isclose(latch.light_direction_difference_deg(base, exactly_five), 5.0)
    assert latch.same_physical_light(base, exactly_five)
    assert not latch.same_physical_light(base, over_five)


def test_light_ownership_chain_keeps_two_outer_candidates() -> None:
    lights = [make_oriented_light(float(x)) for x in (0, 100, 200, 300)]
    selection = latch.select_light_ownership(
        [
            ownership_candidate(0, 0.60, lights[0], lights[1]),
            ownership_candidate(1, 0.99, lights[1], lights[2]),
            ownership_candidate(2, 0.55, lights[2], lights[3]),
        ]
    )
    assert selection["selected_candidate_indices"] == (0, 2)
    assert len(selection["conflicts"]) == 2
    assert selection["largest_conflict_component_size"] == 3
    assert selection["exact_component_count"] == 1
    assert selection["overload_component_count"] == 0
    assert selection["exact_subsets_evaluated"] == 8


def test_light_ownership_shared_pair_keeps_higher_confidence() -> None:
    left = make_oriented_light(100.0)
    right = make_oriented_light(300.0)
    selection = latch.select_light_ownership(
        [
            ownership_candidate(0, 0.50, left, right),
            ownership_candidate(1, 0.80, left, right),
        ]
    )
    assert selection["selected_candidate_indices"] == (1,)


def test_light_ownership_confidence_tie_keeps_lower_candidate_index() -> None:
    left = make_oriented_light(100.0)
    right = make_oriented_light(300.0)
    selection = latch.select_light_ownership(
        [
            ownership_candidate(3, 0.75, left, right),
            ownership_candidate(1, 0.75, left, right),
        ]
    )
    assert selection["selected_candidate_indices"] == (1,)


def test_light_ownership_keeps_all_nonconflicting_candidates() -> None:
    candidates = [
        ownership_candidate(
            index,
            0.50 + index * 0.1,
            make_oriented_light(index * 300.0),
            make_oriented_light(index * 300.0 + 100.0),
        )
        for index in range(3)
    ]
    selection = latch.select_light_ownership(candidates)
    assert selection["selected_candidate_indices"] == (0, 1, 2)
    assert not selection["conflicts"]
    assert selection["largest_conflict_component_size"] == 0
    assert selection["exact_component_count"] == 0
    assert selection["overload_component_count"] == 0
    assert selection["exact_subsets_evaluated"] == 0
    assert set(selection["candidate_resolution_modes"].values()) == {
        "no_conflict"
    }


def test_light_ownership_solves_two_independent_conflict_components() -> None:
    first_left = make_oriented_light(100.0)
    first_right = make_oriented_light(300.0)
    second_left = make_oriented_light(1000.0)
    second_right = make_oriented_light(1200.0)
    selection = latch.select_light_ownership(
        [
            ownership_candidate(0, 0.30, first_left, first_right),
            ownership_candidate(1, 0.80, first_left, first_right),
            ownership_candidate(2, 0.90, second_left, second_right),
            ownership_candidate(3, 0.40, second_left, second_right),
        ]
    )

    assert selection["selected_candidate_indices"] == (1, 2)
    assert selection["largest_conflict_component_size"] == 2
    assert selection["exact_component_count"] == 2
    assert selection["overload_component_count"] == 0
    assert selection["exact_subsets_evaluated"] == 8
    assert set(selection["candidate_resolution_modes"].values()) == {"exact"}


def test_light_ownership_exact_subset_count_is_component_bounded() -> None:
    first_lights = [make_oriented_light(index * 100.0) for index in range(13)]
    second_lights = [
        make_oriented_light(10000.0 + index * 100.0) for index in range(4)
    ]
    candidates = [
        ownership_candidate(
            index, 0.50, first_lights[index], first_lights[index + 1]
        )
        for index in range(12)
    ]
    candidates.extend(
        ownership_candidate(
            20 + index,
            0.50,
            second_lights[index],
            second_lights[index + 1],
        )
        for index in range(3)
    )
    selection = latch.select_light_ownership(candidates)

    exact_components = [
        component
        for component in selection["conflict_components"]
        if component["resolution_mode"] == "exact"
    ]
    expected = sum(1 << component["size"] for component in exact_components)
    assert [component["size"] for component in exact_components] == [12, 3]
    assert selection["exact_subsets_evaluated"] == expected == 4104
    assert selection["exact_subsets_evaluated"] <= (
        selection["exact_component_count"]
        * (1 << latch.MAX_EXACT_OWNERSHIP_CONFLICT_COMPONENT_SIZE)
    )


def test_light_ownership_300_nonconflicting_candidates_do_not_enumerate() -> None:
    candidates = [
        ownership_candidate(
            index,
            0.50,
            make_oriented_light(index * 1000.0),
            make_oriented_light(index * 1000.0 + 100.0),
        )
        for index in range(300)
    ]
    selection = latch.select_light_ownership(candidates)

    assert selection["selected_candidate_indices"] == tuple(range(300))
    assert selection["exact_component_count"] == 0
    assert selection["overload_component_count"] == 0
    assert selection["exact_subsets_evaluated"] == 0
    assert set(selection["candidate_resolution_modes"].values()) == {
        "no_conflict"
    }


def test_light_ownership_50_shared_candidates_fail_open() -> None:
    left = make_oriented_light(100.0)
    right = make_oriented_light(300.0)
    results = [
        {
            "candidate_index": index,
            "pair": make_ownership_pair(left, right),
            "pnp": {"valid": True, "candidate_index": index},
            "output_valid": True,
            "failure_reason": "",
        }
        for index in range(50)
    ]
    ownership = latch.apply_light_ownership(
        results, np.linspace(0.40, 0.90, len(results))
    )

    assert ownership["selected_candidate_indices"] == tuple(range(50))
    assert ownership["largest_conflict_component_size"] == 50
    assert ownership["exact_component_count"] == 0
    assert ownership["overload_component_count"] == 1
    assert ownership["overload_component_sizes"] == [50]
    assert ownership["exact_subsets_evaluated"] == 0
    assert all(result["output_valid"] for result in results)
    assert all(result["ownership_selected"] for result in results)
    assert all(
        result["ownership_resolution_mode"] == "overload_passthrough"
        for result in results
    )
    assert all(result["ownership_overload_passthrough"] for result in results)
    assert all(result["ownership_conflict_component_size"] == 50 for result in results)


def test_invalid_candidate_does_not_compete_for_light_ownership() -> None:
    left = make_oriented_light(100.0)
    right = make_oriented_light(300.0)
    selection = latch.select_light_ownership(
        [
            ownership_candidate(0, 0.20, left, right),
            ownership_candidate(1, 0.99, left, right, output_valid=False),
        ]
    )
    assert selection["selected_candidate_indices"] == (0,)
    assert not selection["conflicts"]


def test_filtered_candidate_keeps_pnp_and_sets_failure_reason() -> None:
    left = make_oriented_light(100.0)
    right = make_oriented_light(300.0)
    first_pnp = {"valid": True, "sentinel": "first"}
    second_pnp = {"valid": True, "sentinel": "second"}
    results = [
        {
            "candidate_index": 0,
            "pair": make_ownership_pair(left, right),
            "pnp": first_pnp,
            "output_valid": True,
            "failure_reason": "",
        },
        {
            "candidate_index": 1,
            "pair": make_ownership_pair(left, right),
            "pnp": second_pnp,
            "output_valid": True,
            "failure_reason": "",
        },
    ]
    latch.apply_light_ownership(results, np.asarray([0.90, 0.40]))

    assert results[0]["output_valid_before_ownership"]
    assert results[0]["ownership_selected"]
    assert results[0]["output_valid"]
    assert results[0]["pnp"] is first_pnp
    assert results[0]["ownership_resolution_mode"] == "exact"
    assert not results[0]["ownership_overload_passthrough"]
    assert results[1]["output_valid_before_ownership"]
    assert not results[1]["ownership_selected"]
    assert not results[1]["output_valid"]
    assert results[1]["failure_reason"] == "light_ownership_conflict"
    assert results[1]["pnp"] is second_pnp
    assert results[1]["ownership_resolution_mode"] == "exact"
    assert not results[1]["ownership_overload_passthrough"]


def test_invalid_candidate_preserves_original_failure_reason() -> None:
    result = {
        "candidate_index": 0,
        "pair": None,
        "pnp": None,
        "output_valid": False,
        "failure_reason": "unable_to_pair:insufficient_lights",
    }
    latch.apply_light_ownership([result], np.asarray([0.99]))
    assert not result["output_valid_before_ownership"]
    assert not result["ownership_selected"]
    assert not result["output_valid"]
    assert result["failure_reason"] == "unable_to_pair:insufficient_lights"
    assert result["ownership_resolution_mode"] == "not_applicable"
    assert not result["ownership_overload_passthrough"]
    assert result["ownership_conflict_component_size"] == 0


def test_raw_node_passes_yolo_confidences_to_runtime() -> None:
    source = (SCRIPTS / "yolo_armor_detector_node.py").read_text(encoding="utf-8")
    tree = ast.parse(source)
    confidence_calls = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call) or len(node.args) < 6:
            continue
        function = node.func
        if not isinstance(function, ast.Attribute) or function.attr != "process_frame":
            continue
        confidence_calls.append(node.args[5])
    assert any(
        isinstance(argument, ast.Name) and argument.id == "confidences"
        for argument in confidence_calls
    )
    assert "last_ownership_overload_warning" in source
    assert "Light ownership overload passthrough" in source
    assert "now - self.last_ownership_overload_warning > 5.0" in source


def test_fixed_strategy_default_and_fixed_solver_contract() -> None:
    source = (SCRIPTS / "yolo_armor_detector_node.py").read_text(encoding="utf-8")
    tree = ast.parse(source)
    defaults = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call) or len(node.args) < 2:
            continue
        function = node.func
        if isinstance(function, ast.Attribute) and function.attr == "declare_parameter":
            first, second = node.args[:2]
            if isinstance(first, ast.Constant) and first.value == "type_strategy":
                defaults.append(second.value if isinstance(second, ast.Constant) else None)
    assert defaults == ["fixed"]
    assert '"normalized_dual_pnp_latch": "normalized_dual_pnp"' in source

    import yolo_armor_detector_node as node_module

    class Logger:
        @staticmethod
        def debug(_message: str) -> None:
            return None

    class FixedProxy:
        camera_matrix = np.asarray(
            [[1740.0, 0.0, 696.0], [0.0, 1740.0, 548.0], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        )
        dist_coeffs = np.zeros((5, 1), dtype=np.float64)
        armor_type = "small"
        armor_number = "armor"
        min_pnp_distance = 0.1
        max_pnp_distance = 10.0
        max_reprojection_error = 12.0

        @staticmethod
        def get_logger() -> Logger:
            return Logger()

    width, height = 0.132, 0.057
    half_y, half_z = width / 2.0, height / 2.0
    object_points = np.asarray(
        [
            [0.0, half_y, -half_z],
            [0.0, half_y, half_z],
            [0.0, -half_y, half_z],
            [0.0, -half_y, -half_z],
        ],
        dtype=np.float32,
    )
    projected, _ = cv2.projectPoints(
        object_points,
        np.asarray([0.03, -0.08, 0.01]),
        np.asarray([0.01, 0.02, 1.0]),
        FixedProxy.camera_matrix,
        FixedProxy.dist_coeffs,
    )
    pnp_points = projected.reshape(4, 2)
    semantic_points = np.asarray(
        [pnp_points[1], pnp_points[0], pnp_points[3], pnp_points[2]],
        dtype=np.float32,
    )
    armor = node_module.YoloArmorNode.solve_armor(FixedProxy(), semantic_points)
    assert armor is not None
    assert armor.type == "small"
    assert armor.number == "armor"
    assert len(armor.kpts) == 4
    assert np.allclose(
        [[point.x, point.y] for point in armor.kpts], semantic_points, atol=1e-6
    )
