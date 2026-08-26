#!/usr/bin/env python3

from __future__ import annotations

import ast
import json
import sys
from pathlib import Path

import numpy as np
from std_msgs.msg import Header
import yaml


PACKAGE = Path(__file__).resolve().parents[1]
SCRIPTS = PACKAGE / "scripts"
sys.path.insert(0, str(SCRIPTS))

import track_diagnostics as diagnostics  # noqa: E402


def declared_parameter_default(name: str):
    source = (SCRIPTS / "yolo_armor_detector_node.py").read_text(encoding="utf-8")
    tree = ast.parse(source)
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call) or len(node.args) < 2:
            continue
        function = node.func
        if not isinstance(function, ast.Attribute) or function.attr != "declare_parameter":
            continue
        parameter_name, default = node.args[:2]
        if isinstance(parameter_name, ast.Constant) and parameter_name.value == name:
            return default.value if isinstance(default, ast.Constant) else None
    raise AssertionError(f"parameter not declared: {name}")


def pnp(error: float, scale: float) -> dict:
    return {
        "solved": np.bool_(True),
        "valid": np.bool_(True),
        "failure_reason": "",
        "rvec": np.asarray([0.1, 0.2, 0.3], dtype=np.float64),
        "tvec": np.asarray([0.0, 0.0, scale], dtype=np.float64),
        "distance_m": np.float64(scale),
        "mean_reprojection_error_px": np.float64(error),
        "max_reprojection_error_px": np.float64(error + 0.5),
    }


def test_track_diagnostic_strict_json_and_required_fields() -> None:
    header = Header()
    header.stamp.sec = 123
    header.stamp.nanosec = 456
    header.frame_id = "camera_optical_frame"
    small_pnp = pnp(1.0, 1.5)
    large_pnp = pnp(3.0, 2.5)
    dual_pnp = {
        "evaluable": True,
        "failure_reason": "",
        "score": np.float64(0.02),
        "type": "small",
        "mean_light_length_px": np.float64(100.0),
        "small_pnp": small_pnp,
        "large_pnp": large_pnp,
    }
    result = {
        "candidate_index": np.int64(0),
        "box_xyxy": np.asarray([10.0, 20.0, 210.0, 120.0]),
        "track_id": np.int64(7),
        "track_created": True,
        "track_match_iou": np.float64(float("nan")),
        "ended_track_ids": [np.int64(3)],
        "latched_type": "small",
        "initialization_frame": np.int64(42),
        "initialization_metric": "normalized_dual_pnp",
        "initialization_value": np.float64(0.02),
        "initialized_this_frame": True,
        "raw_ratio": np.float64(float("inf")),
        "raw_type": "large",
        "failure_reason": "",
        "output_valid": True,
        "output_valid_before_ownership": True,
        "ownership_selected": True,
        "pair": {
            "points_xy": np.asarray(
                [[20.0, 30.0], [20.0, 90.0], [180.0, 90.0], [180.0, 30.0]]
            )
        },
        "pnp": small_pnp,
    }

    encoded = diagnostics.serialize_track_diagnostics(
        header,
        42,
        [result],
        np.asarray([np.float32(0.91)]),
        [3],
        [dual_pnp],
    )
    assert "NaN" not in encoded
    assert "Infinity" not in encoded
    payload = json.loads(encoded, parse_constant=lambda value: 1 / 0)
    assert payload["header_stamp"] == {"sec": 123, "nanosec": 456}
    assert payload["frame_index"] == 42
    assert payload["ended_track_ids"] == [3]
    candidate = payload["candidates"][0]
    assert candidate["track_id"] == 7
    assert candidate["track_match_iou"] is None
    assert candidate["raw_ratio"] is None
    assert candidate["points_xy"][0] == [20.0, 30.0]
    assert candidate["initialization_dual_pnp"]["D"] == 0.02
    assert candidate["dual_pnp"]["E_small_px"] == 1.0
    assert candidate["dual_pnp"]["E_large_px"] == 3.0
    assert candidate["dual_pnp"]["L_px"] == 100.0
    assert candidate["D"] == 0.02
    assert candidate["E_small_px"] == 1.0
    assert candidate["E_large_px"] == 3.0
    assert candidate["L_px"] == 100.0
    assert candidate["small_pnp"]["tvec"] == [0.0, 0.0, 1.5]
    assert candidate["large_pnp"]["tvec"] == [0.0, 0.0, 2.5]
    assert candidate["selected_pnp_type"] == "small"
    assert candidate["selected_pnp"]["mean_reprojection_error_px"] == 1.0


def test_track_diagnostic_empty_frame() -> None:
    header = Header()
    encoded = diagnostics.serialize_track_diagnostics(
        header, 8, [], np.empty((0,)), [4, 5], []
    )
    payload = json.loads(encoded)
    assert payload["frame_index"] == 8
    assert payload["ended_track_ids"] == [4, 5]
    assert payload["candidates"] == []


def test_track_diagnostics_default_off_and_formal_yamls_unchanged() -> None:
    assert declared_parameter_default("publish_track_diagnostics") is False
    assert (
        declared_parameter_default("track_diagnostics_topic")
        == "/detector/track_diagnostics"
    )
    for name in (
        "yolo_detector_params.yaml",
        "yolo_detector_params_v8_frozen_candidate.yaml",
    ):
        path = PACKAGE.parent / "auto_aim_bringup" / "params" / name
        params = yaml.safe_load(path.read_text(encoding="utf-8"))[
            "yolo_armor_detector"
        ]["ros__parameters"]
        assert "publish_track_diagnostics" not in params
        assert "track_diagnostics_topic" not in params
