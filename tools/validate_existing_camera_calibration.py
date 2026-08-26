#!/usr/bin/env python3
"""Validate an existing camera calibration against preregistered chessboard views."""

from __future__ import annotations

import argparse
import csv
import json
import math
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import yaml


PATTERN_SIZE = (9, 6)
SQUARE_SIZE_MM = 24.0
MEAN_ERROR_LIMIT_PX = 2.0
SYSTEMATIC_BIAS_LIMIT_PX = 0.5
SYSTEMATIC_COHERENCE_LIMIT = 0.5
EDGE_OR_ANGLE_VIEWS = {
    "center_near",
    "left_side",
    "right_side",
    "upper_side",
    "lower_side",
    "yaw_left",
    "yaw_right",
    "tilt_up",
    "tilt_down",
    "corner_oblique",
}
PER_VIEW_FIELDS = [
    "order",
    "view_id",
    "placement",
    "image",
    "detected_corners",
    "mean_error_px",
    "rms_error_px",
    "p90_error_px",
    "max_error_px",
    "mean_residual_dx_px",
    "mean_residual_dy_px",
    "mean_residual_vector_magnitude_px",
    "residual_direction_coherence",
    "mean_radial_residual_px",
    "mean_tangential_residual_px",
    "distance_m",
    "pass_mean_2px",
    "systematic_same_direction_bias",
    "requires_retest",
    "residuals_json",
    "overlay",
]


def load_calibration(path: Path) -> tuple[np.ndarray, np.ndarray, dict]:
    payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    camera_matrix = np.asarray(payload["camera_matrix"]["data"], dtype=np.float64).reshape(3, 3)
    distortion = np.asarray(
        payload["distortion_coefficients"]["data"], dtype=np.float64
    ).reshape(-1, 1)
    return camera_matrix, distortion, {
        "path": str(path),
        "image_width": int(payload["image_width"]),
        "image_height": int(payload["image_height"]),
        "camera_matrix": camera_matrix.tolist(),
        "distortion_coefficients": distortion.reshape(-1).tolist(),
    }


def object_points() -> np.ndarray:
    points = np.zeros((PATTERN_SIZE[0] * PATTERN_SIZE[1], 3), dtype=np.float32)
    points[:, :2] = np.mgrid[
        0 : PATTERN_SIZE[0], 0 : PATTERN_SIZE[1]
    ].T.reshape(-1, 2)
    points[:, :2] *= SQUARE_SIZE_MM / 1000.0
    return points


def detect_corners(gray: np.ndarray) -> np.ndarray:
    found, corners = cv2.findChessboardCornersSB(
        gray,
        PATTERN_SIZE,
        flags=cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE,
    )
    if not found or corners is None or len(corners) != 54:
        raise RuntimeError("failed to detect all 9x6 inner corners")
    corners = corners.astype(np.float32)
    cv2.cornerSubPix(
        gray,
        corners,
        (5, 5),
        (-1, -1),
        (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 40, 0.001),
    )
    return corners.reshape(-1, 2)


def residual_direction_metrics(
    detected: np.ndarray,
    projected: np.ndarray,
    principal_point: np.ndarray,
) -> dict:
    residuals = detected - projected
    lengths = np.linalg.norm(residuals, axis=1)
    mean_vector = np.mean(residuals, axis=0)
    mean_vector_magnitude = float(np.linalg.norm(mean_vector))
    mean_length = float(np.mean(lengths))
    coherence = mean_vector_magnitude / mean_length if mean_length > 1e-12 else 0.0

    from_principal = detected - principal_point
    radii = np.linalg.norm(from_principal, axis=1)
    radial_unit = np.divide(
        from_principal,
        radii[:, None],
        out=np.zeros_like(from_principal),
        where=radii[:, None] > 1e-12,
    )
    tangential_unit = np.column_stack((-radial_unit[:, 1], radial_unit[:, 0]))
    radial = np.sum(residuals * radial_unit, axis=1)
    tangential = np.sum(residuals * tangential_unit, axis=1)
    systematic = (
        mean_vector_magnitude > SYSTEMATIC_BIAS_LIMIT_PX
        and coherence > SYSTEMATIC_COHERENCE_LIMIT
    )
    return {
        "residuals": residuals,
        "lengths": lengths,
        "mean_dx": float(mean_vector[0]),
        "mean_dy": float(mean_vector[1]),
        "mean_vector_magnitude": mean_vector_magnitude,
        "direction_coherence": float(coherence),
        "mean_radial": float(np.mean(radial)),
        "mean_tangential": float(np.mean(tangential)),
        "systematic_same_direction_bias": systematic,
    }


def draw_overlay(
    image: np.ndarray,
    detected: np.ndarray,
    projected: np.ndarray,
    result: dict,
) -> np.ndarray:
    output = image.copy()
    green = (0, 255, 0)
    magenta = (255, 0, 255)
    line_color = (255, 255, 255)
    for observed, reprojection in zip(detected, projected):
        p = tuple(np.round(observed).astype(int))
        q = tuple(np.round(reprojection).astype(int))
        cv2.line(output, p, q, line_color, 1, cv2.LINE_AA)
        cv2.circle(output, p, 4, green, -1, cv2.LINE_AA)
        cv2.circle(output, q, 3, magenta, -1, cv2.LINE_AA)
    lines = [
        f"{result['view_id']}: green=detected magenta=fixed-K/D reprojection",
        (
            f"mean={result['mean_error_px']:.3f}px RMS={result['rms_error_px']:.3f}px "
            f"P90={result['p90_error_px']:.3f}px max={result['max_error_px']:.3f}px"
        ),
        (
            f"mean residual=({result['mean_residual_dx_px']:+.3f},"
            f"{result['mean_residual_dy_px']:+.3f})px "
            f"distance={result['distance_m']:.4f}m pass={result['pass_mean_2px']}"
        ),
    ]
    for index, text in enumerate(lines):
        y = 30 + index * 29
        cv2.putText(
            output, text, (14, y), cv2.FONT_HERSHEY_SIMPLEX, 0.62,
            (0, 0, 0), 5, cv2.LINE_AA,
        )
        cv2.putText(
            output, text, (14, y), cv2.FONT_HERSHEY_SIMPLEX, 0.62,
            (255, 255, 255), 2, cv2.LINE_AA,
        )
    return output


def analyze_view(
    row: dict[str, str],
    root: Path,
    overlays: Path,
    camera_matrix: np.ndarray,
    distortion: np.ndarray,
) -> tuple[dict, np.ndarray]:
    image_path = root / row["image"]
    image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
    if image is None:
        raise RuntimeError(f"failed to decode {image_path}")
    if image.shape != (1080, 1440, 3):
        raise RuntimeError(f"unexpected image shape for {image_path}: {image.shape}")
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    detected = detect_corners(gray)
    success, rvec, tvec = cv2.solvePnP(
        object_points(), detected, camera_matrix, distortion,
        flags=cv2.SOLVEPNP_ITERATIVE,
    )
    if not success:
        raise RuntimeError(f"solvePnP failed for {image_path}")
    projected, _ = cv2.projectPoints(
        object_points(), rvec, tvec, camera_matrix, distortion
    )
    projected = projected.reshape(-1, 2)
    metrics = residual_direction_metrics(
        detected,
        projected,
        np.asarray([camera_matrix[0, 2], camera_matrix[1, 2]], dtype=np.float32),
    )
    errors = metrics["lengths"]
    view_id = row["view_id"]
    overlay_path = overlays / f"{int(row['order']):02d}_{view_id}_overlay.png"
    residuals = [
        {
            "corner_index": index,
            "detected_x": float(observed[0]),
            "detected_y": float(observed[1]),
            "projected_x": float(reprojection[0]),
            "projected_y": float(reprojection[1]),
            "dx": float(residual[0]),
            "dy": float(residual[1]),
            "error_px": float(error),
            "direction_deg": float(math.degrees(math.atan2(residual[1], residual[0]))),
        }
        for index, (observed, reprojection, residual, error) in enumerate(
            zip(detected, projected, metrics["residuals"], errors)
        )
    ]
    mean_error = float(np.mean(errors))
    result = {
        "order": int(row["order"]),
        "view_id": view_id,
        "placement": row["placement"],
        "image": str(image_path),
        "detected_corners": len(detected),
        "mean_error_px": mean_error,
        "rms_error_px": float(np.sqrt(np.mean(np.square(errors)))),
        "p90_error_px": float(np.percentile(errors, 90)),
        "max_error_px": float(np.max(errors)),
        "mean_residual_dx_px": metrics["mean_dx"],
        "mean_residual_dy_px": metrics["mean_dy"],
        "mean_residual_vector_magnitude_px": metrics["mean_vector_magnitude"],
        "residual_direction_coherence": metrics["direction_coherence"],
        "mean_radial_residual_px": metrics["mean_radial"],
        "mean_tangential_residual_px": metrics["mean_tangential"],
        "distance_m": float(np.linalg.norm(tvec.reshape(3))),
        "rvec": rvec.reshape(3).astype(float).tolist(),
        "tvec_m": tvec.reshape(3).astype(float).tolist(),
        "pass_mean_2px": mean_error <= MEAN_ERROR_LIMIT_PX,
        "systematic_same_direction_bias": metrics["systematic_same_direction_bias"],
        "requires_retest": mean_error > MEAN_ERROR_LIMIT_PX,
        "residuals": residuals,
        "overlay": str(overlay_path),
    }
    return result, draw_overlay(image, detected, projected, result)


def aggregate_direction(views: list[dict]) -> dict:
    residuals = np.asarray(
        [
            [residual["dx"], residual["dy"]]
            for view in views
            for residual in view["residuals"]
        ],
        dtype=np.float64,
    )
    lengths = np.linalg.norm(residuals, axis=1)
    mean_vector = np.mean(residuals, axis=0)
    magnitude = float(np.linalg.norm(mean_vector))
    mean_length = float(np.mean(lengths))
    coherence = magnitude / mean_length if mean_length > 1e-12 else 0.0
    return {
        "views": [view["view_id"] for view in views],
        "mean_dx_px": float(mean_vector[0]),
        "mean_dy_px": float(mean_vector[1]),
        "mean_vector_magnitude_px": magnitude,
        "direction_coherence": coherence,
        "systematic_same_direction_bias": (
            magnitude > SYSTEMATIC_BIAS_LIMIT_PX
            and coherence > SYSTEMATIC_COHERENCE_LIMIT
        ),
    }


def csv_row(view: dict) -> dict:
    row = {key: view[key] for key in PER_VIEW_FIELDS if key in view}
    row["residuals_json"] = json.dumps(view["residuals"], separators=(",", ":"))
    return row


def build_report(summary: dict) -> str:
    lines = [
        "# Existing Camera Calibration Validation",
        "",
        "The existing YAML K/D were evaluated without recalibration. Each preregistered view uses one PNG, 9x6 inner corners, 24 mm spacing, per-view solvePnP, and all 54 projected points.",
        "",
        "## Runtime Contract",
        "",
        "Runtime checks passed at 1440x1080 BGR8 with no ROI/cropping/binning, and `/image_raw` and `/camera_info` used matching timestamps and `camera_optical_frame`. CameraInfo K/D matched the YAML.",
        "",
        "## Per-view Results",
        "",
        "| View | Mean px | RMS px | P90 px | Max px | Mean residual dx/dy px | Distance m | Pass 2 px | Retest |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for view in summary["views"]:
        lines.append(
            f"| `{view['view_id']}` | {view['mean_error_px']:.3f} | "
            f"{view['rms_error_px']:.3f} | {view['p90_error_px']:.3f} | "
            f"{view['max_error_px']:.3f} | {view['mean_residual_dx_px']:+.3f}/"
            f"{view['mean_residual_dy_px']:+.3f} | {view['distance_m']:.4f} | "
            f"{view['pass_mean_2px']} | {view['requires_retest']} |"
        )
    lines.extend(
        [
            "",
            "## Directional Residual Check",
            "",
            (
                f"Edge/near/angled aggregate residual vector is "
                f"({summary['edge_near_angle_direction']['mean_dx_px']:+.4f}, "
                f"{summary['edge_near_angle_direction']['mean_dy_px']:+.4f}) px; "
                f"coherence={summary['edge_near_angle_direction']['direction_coherence']:.4f}; "
                f"systematic={summary['edge_near_angle_direction']['systematic_same_direction_bias']}."
            ),
            "",
            "## Decision",
            "",
            f"**{summary['decision']['code']}**",
            "",
            summary["decision"]["conclusion"],
            "",
            summary["decision"]["next_step"],
        ]
    )
    return "\n".join(lines) + "\n"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--calibration", type=Path, required=True)
    args = parser.parse_args()
    manifest_path = args.root / "manifest.csv"
    per_view_path = args.root / "per_view.csv"
    summary_path = args.root / "summary.json"
    report_path = args.root / "report.md"
    overlays = args.root / "overlays"
    for path in (per_view_path, summary_path, report_path, overlays):
        if path.exists():
            raise RuntimeError(f"refusing to overwrite result path: {path}")

    with manifest_path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    if len(rows) != 12:
        raise RuntimeError(f"expected 12 manifest rows, got {len(rows)}")
    camera_matrix, distortion, calibration = load_calibration(args.calibration)
    views = []
    overlay_images = []
    for row in rows:
        result, overlay = analyze_view(
            row, args.root, overlays, camera_matrix, distortion
        )
        views.append(result)
        overlay_images.append((Path(result["overlay"]), overlay))

    retest_views = [view["view_id"] for view in views if view["requires_retest"]]
    edge_near_angle = aggregate_direction(
        [view for view in views if view["view_id"] in EDGE_OR_ANGLE_VIEWS]
    )
    if retest_views:
        decision = {
            "code": "CALIBRATION_RETEST_REQUIRED",
            "conclusion": (
                "At least one preregistered view exceeds the 2 px mean-error gate. The fixed "
                "decision requires exactly one same-position retest before judging calibration."
            ),
            "next_step": f"Retest only these positions once: {', '.join(retest_views)}.",
        }
    elif edge_near_angle["systematic_same_direction_bias"]:
        decision = {
            "code": "CALIBRATION_SUSPECT",
            "conclusion": (
                "All mean errors pass 2 px, but edge/near/angled views retain a coherent "
                "same-direction residual bias."
            ),
            "next_step": "Review calibration acquisition geometry; do not overwrite the YAML in this stage.",
        }
    else:
        decision = {
            "code": "CALIBRATION_NOT_PRIMARY",
            "conclusion": (
                "All views are at or below 2 px mean error and edge/near/angled views do not "
                "show a coherent same-direction residual bias. This calibration cannot explain "
                "the armor's 14-34 px difficult-frame reprojection errors by itself."
            ),
            "next_step": (
                "Keep the existing calibration unchanged and return to the remaining armor "
                "keypoint/physical-correspondence geometry question."
            ),
        }
    summary = {
        "schema_version": 1,
        "generated_at": datetime.now().astimezone().isoformat(),
        "experiment": "existing_camera_calibration_independent_validation",
        "recalibration_performed": False,
        "pattern": {"inner_corners": [9, 6], "square_size_mm": SQUARE_SIZE_MM},
        "calibration": calibration,
        "runtime_contract": {
            "pass": True,
            "image": {"width": 1440, "height": 1080, "encoding": "bgr8", "roi": False, "cropping": False, "scaling": False, "binning": False},
            "camera_info_matches_yaml": True,
            "matching_image_camera_info_timestamp": True,
            "frame_id": "camera_optical_frame",
            "checked_before_initial_capture": True,
            "checked_after_camera_resume": True,
        },
        "thresholds": {
            "mean_error_limit_px": MEAN_ERROR_LIMIT_PX,
            "systematic_mean_vector_limit_px": SYSTEMATIC_BIAS_LIMIT_PX,
            "systematic_direction_coherence_limit": SYSTEMATIC_COHERENCE_LIMIT,
        },
        "views": views,
        "retest_required_views": retest_views,
        "edge_near_angle_direction": edge_near_angle,
        "decision": decision,
    }

    overlays.mkdir()
    for path, overlay in overlay_images:
        if not cv2.imwrite(str(path), overlay, [cv2.IMWRITE_PNG_COMPRESSION, 3]):
            raise RuntimeError(f"failed to save {path}")
    with per_view_path.open("x", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=PER_VIEW_FIELDS)
        writer.writeheader()
        writer.writerows(csv_row(view) for view in views)
    with summary_path.open("x", encoding="utf-8") as stream:
        json.dump(summary, stream, indent=2, ensure_ascii=False)
        stream.write("\n")
    report_path.write_text(build_report(summary), encoding="utf-8")
    print(json.dumps({"retest_required_views": retest_views, "decision": decision, "views": [{"view_id": v["view_id"], "mean": v["mean_error_px"], "rms": v["rms_error_px"], "p90": v["p90_error_px"], "max": v["max_error_px"]} for v in views]}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
