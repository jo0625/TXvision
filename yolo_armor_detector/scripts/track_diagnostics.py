#!/usr/bin/env python3
"""Strict-JSON projection of raw track-latch detector results."""

from __future__ import annotations

import json
import math
from typing import Any, Optional

import numpy as np


def json_safe(value: Any) -> Any:
    """Convert NumPy containers and non-finite numbers to strict JSON values."""
    if isinstance(value, np.ndarray):
        return [json_safe(item) for item in value.tolist()]
    if isinstance(value, np.generic):
        return json_safe(value.item())
    if isinstance(value, dict):
        return {str(key): json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(item) for item in value]
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if value is None or isinstance(value, (bool, int, str)):
        return value
    raise TypeError(f"unsupported diagnostic JSON value: {type(value).__name__}")


def pnp_projection(pnp: Optional[dict]) -> Optional[dict]:
    if pnp is None:
        return None
    return {
        "solved": bool(pnp.get("solved", False)),
        "valid": bool(pnp.get("valid", False)),
        "failure_reason": pnp.get("failure_reason") or "",
        "tvec": pnp.get("tvec"),
        "rvec": pnp.get("rvec"),
        "distance_m": pnp.get("distance_m"),
        "mean_reprojection_error_px": pnp.get("mean_reprojection_error_px"),
        "max_reprojection_error_px": pnp.get("max_reprojection_error_px"),
    }


def dual_pnp_projection(dual_pnp: Optional[dict]) -> Optional[dict]:
    if dual_pnp is None:
        return None
    small_pnp = pnp_projection(dual_pnp.get("small_pnp"))
    large_pnp = pnp_projection(dual_pnp.get("large_pnp"))
    return {
        "evaluable": bool(dual_pnp.get("evaluable", False)),
        "failure_reason": dual_pnp.get("failure_reason") or "",
        "D": dual_pnp.get("score"),
        "candidate_type": dual_pnp.get("type"),
        "E_small_px": (
            small_pnp["mean_reprojection_error_px"] if small_pnp else None
        ),
        "E_large_px": (
            large_pnp["mean_reprojection_error_px"] if large_pnp else None
        ),
        "L_px": dual_pnp.get("mean_light_length_px"),
        "small_pnp": small_pnp,
        "large_pnp": large_pnp,
    }


def candidate_projection(
    result: dict, confidence: float, dual_pnp: Optional[dict]
) -> dict:
    pair = result.get("pair")
    projected_dual_pnp = dual_pnp_projection(dual_pnp)
    selected_pnp = pnp_projection(result.get("pnp"))
    initialized_this_frame = bool(result.get("initialized_this_frame", False))
    small_pnp = (
        projected_dual_pnp["small_pnp"] if projected_dual_pnp else None
    )
    large_pnp = (
        projected_dual_pnp["large_pnp"] if projected_dual_pnp else None
    )
    return {
        "candidate_index": result.get("candidate_index"),
        "confidence": confidence,
        "box_xyxy": result.get("box_xyxy"),
        "track_id": result.get("track_id"),
        "track_created": bool(result.get("track_created", False)),
        "track_match_iou": result.get("track_match_iou"),
        "ended_track_ids": result.get("ended_track_ids", []),
        "latched_type": result.get("latched_type"),
        "initialized_this_frame": initialized_this_frame,
        "initialization_frame": result.get("initialization_frame"),
        "initialization_metric": result.get("initialization_metric"),
        "initialization_value": result.get("initialization_value"),
        "raw_ratio": result.get("raw_ratio"),
        "raw_type": result.get("raw_type"),
        "failure_reason": result.get("failure_reason") or "",
        "output_valid": bool(result.get("output_valid", False)),
        "output_valid_before_ownership": bool(
            result.get("output_valid_before_ownership", False)
        ),
        "ownership_selected": bool(result.get("ownership_selected", False)),
        "points_xy": pair.get("points_xy") if pair is not None else None,
        "D": projected_dual_pnp["D"] if projected_dual_pnp else None,
        "E_small_px": (
            projected_dual_pnp["E_small_px"] if projected_dual_pnp else None
        ),
        "E_large_px": (
            projected_dual_pnp["E_large_px"] if projected_dual_pnp else None
        ),
        "L_px": projected_dual_pnp["L_px"] if projected_dual_pnp else None,
        "small_pnp": small_pnp,
        "large_pnp": large_pnp,
        "dual_pnp": projected_dual_pnp,
        "initialization_dual_pnp": (
            projected_dual_pnp if initialized_this_frame else None
        ),
        "selected_pnp_type": (
            result.get("latched_type") if selected_pnp is not None else None
        ),
        "selected_pnp": selected_pnp,
    }


def serialize_track_diagnostics(
    header,
    frame_index: int,
    results: list[dict],
    confidences: np.ndarray,
    ended_track_ids: list[int],
    dual_pnp_results: list[Optional[dict]],
) -> str:
    if len(results) != len(confidences) or len(results) != len(dual_pnp_results):
        raise ValueError("track diagnostic candidate field counts do not match")
    payload = {
        "header_stamp": {
            "sec": header.stamp.sec,
            "nanosec": header.stamp.nanosec,
        },
        "frame_id": header.frame_id,
        "frame_index": frame_index,
        "ended_track_ids": ended_track_ids,
        "candidates": [
            candidate_projection(result, confidence, dual_pnp)
            for result, confidence, dual_pnp in zip(
                results, confidences, dual_pnp_results
            )
        ],
    }
    return json.dumps(
        json_safe(payload), allow_nan=False, separators=(",", ":"), sort_keys=True
    )
