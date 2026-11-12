#!/usr/bin/env python3
"""Solve pitch-joint to camera extrinsics from captured fixed-board samples."""

from __future__ import annotations

import argparse
import json
import math
from datetime import datetime, timezone
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

from handeye_math import (
    HAND_EYE_METHODS,
    camera_link_from_optical,
    fixed_board_residuals,
    invert_transform,
    mechanical_pitch_from_optical,
    rotation_error_deg,
    solve_handeye,
    transform_payload,
    world_from_pitch,
)


def load_samples(root: Path) -> list[dict]:
    samples = []
    for path in sorted(root.glob("*/sample.json")):
        payload = json.loads(path.read_text(encoding="utf-8"))
        payload["metadata_path"] = str(path.resolve())
        payload["world_from_pitch"] = world_from_pitch(
            float(payload["gimbal"]["yaw_rad"]),
            float(payload["gimbal"]["pitch_rad"]),
        )
        payload["camera_from_board"] = np.asarray(
            payload["pnp"]["matrix"], dtype=np.float64
        ).reshape(4, 4)
        samples.append(payload)
    return samples


def residual_summary(residuals: list[dict]) -> dict:
    if not residuals:
        return {"count": 0}
    translations = np.asarray([item["translation_mm"] for item in residuals])
    rotations = np.asarray([item["rotation_deg"] for item in residuals])
    return {
        "count": len(residuals),
        "translation_rms_mm": float(np.sqrt(np.mean(np.square(translations)))),
        "translation_max_mm": float(np.max(translations)),
        "rotation_rms_deg": float(np.sqrt(np.mean(np.square(rotations)))),
        "rotation_max_deg": float(np.max(rotations)),
    }


def add_sample_ids(samples: list[dict], residuals: list[dict]) -> list[dict]:
    return [
        {"sample_id": sample["sample_id"], **residual}
        for sample, residual in zip(samples, residuals)
    ]


def build_report(result: dict) -> str:
    chosen = result["chosen_solution"]
    mechanical = result["mechanical_comparison"]
    calibration = chosen["calibration_residual_summary"]
    holdout = chosen["holdout_residual_summary"]
    recommendation = result["recommended_gimbal_tf_parameters"]
    lines = [
        "# Gimbal Hand-Eye Calibration",
        "",
        f"Generated: {result['generated_at_utc']}",
        "",
        "## Dataset",
        "",
        f"- Calibration samples: {result['calibration_sample_count']}",
        f"- Holdout samples: {result['holdout_sample_count']}",
        f"- Chosen OpenCV method: {result['chosen_method']}",
        "",
        "## Fixed-board consistency",
        "",
        f"- Calibration translation RMS: {calibration['translation_rms_mm']:.3f} mm",
        f"- Calibration rotation RMS: {calibration['rotation_rms_deg']:.4f} deg",
    ]
    if holdout["count"]:
        lines.extend(
            [
                f"- Holdout translation RMS: {holdout['translation_rms_mm']:.3f} mm",
                f"- Holdout rotation RMS: {holdout['rotation_rms_deg']:.4f} deg",
            ]
        )
    lines.extend(
        [
            "",
            "## Mechanical comparison",
            "",
            f"- Translation difference: {mechanical['translation_difference_mm']:.3f} mm",
            f"- Rotation difference: {mechanical['rotation_difference_deg']:.4f} deg",
            "",
            "## Candidate gimbal_tf values",
            "",
            "These are solver output for review; this command does not edit the deployment YAML.",
            "",
            "```yaml",
            f"pitch_to_camera_x: {recommendation['pitch_to_camera_x']:.9f}",
            f"pitch_to_camera_y: {recommendation['pitch_to_camera_y']:.9f}",
            f"pitch_to_camera_z: {recommendation['pitch_to_camera_z']:.9f}",
            f"camera_mount_rpy_x: {recommendation['camera_mount_rpy_x']:.9f}",
            f"camera_mount_rpy_y: {recommendation['camera_mount_rpy_y']:.9f}",
            f"camera_mount_rpy_z: {recommendation['camera_mount_rpy_z']:.9f}",
            "```",
            "",
        ]
    )
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--session", type=Path, required=True)
    parser.add_argument("--method", choices=tuple(HAND_EYE_METHODS), default="PARK")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    samples = load_samples(args.session)
    calibration_samples = [
        sample for sample in samples if sample.get("role", "calibration") == "calibration"
    ]
    holdout_samples = [sample for sample in samples if sample.get("role") == "holdout"]
    if len(calibration_samples) < 3:
        parser.error("at least three calibration samples are required")

    robot = [sample["world_from_pitch"] for sample in calibration_samples]
    target = [sample["camera_from_board"] for sample in calibration_samples]
    solutions = {}
    for method in HAND_EYE_METHODS:
        try:
            pitch_from_camera = solve_handeye(robot, target, method)
            world_from_board, calibration_residuals = fixed_board_residuals(
                robot, pitch_from_camera, target
            )
            holdout_robot = [sample["world_from_pitch"] for sample in holdout_samples]
            holdout_target = [sample["camera_from_board"] for sample in holdout_samples]
            _, holdout_residuals = fixed_board_residuals(
                holdout_robot,
                pitch_from_camera,
                holdout_target,
                world_from_board,
            ) if holdout_samples else (world_from_board, [])
            solutions[method] = {
                "pitch_joint_from_camera_optical": transform_payload(pitch_from_camera),
                "world_from_board": transform_payload(world_from_board),
                "calibration_residual_summary": residual_summary(calibration_residuals),
                "calibration_residuals": add_sample_ids(
                    calibration_samples, calibration_residuals
                ),
                "holdout_residual_summary": residual_summary(holdout_residuals),
                "holdout_residuals": add_sample_ids(holdout_samples, holdout_residuals),
            }
        except (RuntimeError, ValueError, cv2.error) as exc:
            solutions[method] = {"error": str(exc)}

    chosen = solutions[args.method]
    if "error" in chosen:
        raise RuntimeError(f"chosen method {args.method} failed: {chosen['error']}")
    pitch_from_optical = np.asarray(
        chosen["pitch_joint_from_camera_optical"]["matrix"], dtype=np.float64
    )
    mechanical = mechanical_pitch_from_optical()
    pitch_from_link = pitch_from_optical @ invert_transform(camera_link_from_optical())
    mount_rpy = Rotation.from_matrix(pitch_from_link[:3, :3]).as_euler("xyz")

    result = {
        "schema_version": 1,
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "session": str(args.session.resolve()),
        "calibration_sample_count": len(calibration_samples),
        "holdout_sample_count": len(holdout_samples),
        "sample_ids": [sample["sample_id"] for sample in samples],
        "chosen_method": args.method,
        "chosen_solution": chosen,
        "all_solutions": solutions,
        "mechanical_initial_pitch_joint_from_camera_optical": transform_payload(
            mechanical
        ),
        "mechanical_comparison": {
            "translation_difference_mm": float(
                np.linalg.norm(pitch_from_optical[:3, 3] - mechanical[:3, 3])
                * 1000.0
            ),
            "rotation_difference_deg": rotation_error_deg(
                mechanical[:3, :3], pitch_from_optical[:3, :3]
            ),
        },
        "recommended_gimbal_tf_parameters": {
            "pitch_to_camera_x": float(pitch_from_link[0, 3]),
            "pitch_to_camera_y": float(pitch_from_link[1, 3]),
            "pitch_to_camera_z": float(pitch_from_link[2, 3]),
            "camera_mount_rpy_x": float(mount_rpy[0]),
            "camera_mount_rpy_y": float(mount_rpy[1]),
            "camera_mount_rpy_z": float(mount_rpy[2]),
        },
    }
    output = args.output or args.session / "handeye_result.json"
    output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    report = output.with_suffix(".md")
    report.write_text(build_report(result), encoding="utf-8")
    print(json.dumps({"status": "solved", "result": str(output), "report": str(report), **result}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
