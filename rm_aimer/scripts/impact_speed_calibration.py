#!/usr/bin/env python3
"""Fit a gravity-only effective speed from independently measured impacts."""

from __future__ import annotations

import argparse
import csv
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import hashlib
import io
import json
import math
from pathlib import Path


MODEL = "gravity_only_impact_fit"
COLUMNS = ("shot_id", "gear", "horizontal_distance_m", "launch_pitch_deg", "impact_height_m")


@dataclass(frozen=True)
class ImpactMeasurement:
    shot_id: str
    gear: str
    horizontal_distance_m: float
    launch_pitch_deg: float
    impact_height_m: float


def impact_terms(shot: ImpactMeasurement, gravity: float):
    """Return gravity coefficient and observed drop below the launch ray."""
    distance, pitch, height = (
        shot.horizontal_distance_m, shot.launch_pitch_deg, shot.impact_height_m)
    if not shot.shot_id.strip() or not shot.gear.strip():
        raise ValueError("each shot needs a shot_id and gear")
    if not all(math.isfinite(value) for value in (distance, pitch, height, gravity)):
        raise ValueError("measurements and gravity must be finite")
    if distance <= 0 or gravity <= 0 or not -90 < pitch < 90:
        raise ValueError("distance/gravity must be positive and elevation within (-90, 90) degrees")
    angle = math.radians(pitch)
    cosine = math.cos(angle)
    if cosine < 1e-6:
        raise ValueError("near-vertical measurements are ill-conditioned")
    drop = distance * math.tan(angle) - height
    if drop <= 1e-9 * max(1.0, distance, abs(height)):
        raise ValueError("impact must lie measurably below the actual launch ray")
    coefficient = gravity * distance**2 / (2 * cosine**2)
    if not math.isfinite(coefficient):
        raise ValueError("measurement magnitude is too large")
    return coefficient, drop


def fit_impact_speed(shots: list[ImpactMeasurement], gravity: float = 9.81) -> dict:
    """Least-squares fit of observed drop = coefficient / speed**2.

    This estimates an effective model parameter, not a chronograph measurement.
    All shots must share a gear, mounting configuration and physical angle reference.
    Residuals describe fit consistency and are not an absolute-accuracy guarantee.
    """
    if len(shots) < 3:
        raise ValueError("at least three independently measured impacts are required")
    if len({shot.shot_id for shot in shots}) != len(shots):
        raise ValueError("shot_id values must be unique")
    if len({shot.gear for shot in shots}) != 1:
        raise ValueError("do not mix friction-wheel gears in one speed calibration")
    terms = [impact_terms(shot, gravity) for shot in shots]
    scale = max(coefficient for coefficient, _ in terms)
    scaled = [(coefficient / scale, drop) for coefficient, drop in terms]
    slope = math.fsum(x * drop for x, drop in scaled) / math.fsum(x * x for x, _ in scaled)
    speed = math.sqrt(scale / slope)
    if not math.isfinite(speed) or speed <= 0:
        raise ValueError("fitted speed must be finite and positive")
    residuals = [drop - slope * x for x, drop in scaled]
    individual = [math.sqrt(coefficient / drop) for coefficient, drop in terms]
    spread = (max(individual) - min(individual)) / speed
    warnings = ["effective_speed_only", "mount_and_physical_angle_reference_must_remain_fixed"]
    distances = [shot.horizontal_distance_m for shot in shots]
    if max(distances) - min(distances) < 1e-6:
        warnings.append("single_distance_does_not_check_range_dependence")
    if spread > 0.10:
        warnings.append("individual_speed_spread_exceeds_10_percent")
    return {
        "speed_mps": speed,
        "gear": shots[0].gear,
        "sample_count": len(shots),
        "height_residual_rms_m": math.sqrt(math.fsum(r*r for r in residuals) / len(shots)),
        "height_residuals_m": residuals,
        "individual_speed_mps": individual,
        "individual_speed_relative_spread": spread,
        "observed_drop_m": [drop for _, drop in terms],
        "horizontal_distance_min_m": min(distances),
        "horizontal_distance_max_m": max(distances),
        "warnings": warnings,
    }


def read_measurements(path: Path) -> tuple[list[ImpactMeasurement], bytes]:
    raw = path.read_bytes()
    reader = csv.DictReader(io.StringIO(raw.decode("utf-8-sig")))
    if not reader.fieldnames or set(reader.fieldnames) != set(COLUMNS):
        raise ValueError("CSV columns must be: " + ",".join(COLUMNS))
    shots = []
    for line, row in enumerate(reader, start=2):
        try:
            shots.append(ImpactMeasurement(
                shot_id=row["shot_id"].strip(), gear=row["gear"].strip(),
                horizontal_distance_m=float(row["horizontal_distance_m"]),
                launch_pitch_deg=float(row["launch_pitch_deg"]),
                impact_height_m=float(row["impact_height_m"])))
        except (AttributeError, TypeError, ValueError) as exc:
            raise ValueError(f"CSV line {line}: incomplete or invalid measurement") from exc
    return shots, raw


def build_calibration(shots, raw, gravity=9.81):
    fit = fit_impact_speed(shots, gravity)
    return {
        "schema": 1, "model": MODEL, "source": "measured_impacts",
        "fit_created_at_utc": datetime.now(timezone.utc).isoformat(),
        "measurements_sha256": hashlib.sha256(raw).hexdigest(),
        "gravity_mps2": gravity,
        "measurements": [asdict(shot) for shot in shots],
        "fit": fit,
    }


def load_speed_calibration(path: Path, gravity: float) -> dict:
    """Validate the saved inputs and recompute the fit before using its speed."""
    try:
        data = json.loads(Path(path).read_text())
        if (data["schema"] != 1 or data["model"] != MODEL
                or data["source"] != "measured_impacts"):
            raise ValueError("unsupported speed calibration format")
        if not math.isfinite(gravity) or not math.isclose(
                data["gravity_mps2"], gravity, rel_tol=0, abs_tol=1e-9):
            raise ValueError("calibration gravity differs from aimer gravity")
        digest = data["measurements_sha256"]
        if not isinstance(digest, str) or len(digest) != 64 or any(c not in "0123456789abcdef" for c in digest):
            raise ValueError("missing measurement provenance hash")
        shots = [ImpactMeasurement(**row) for row in data["measurements"]]
        recomputed = fit_impact_speed(shots, gravity)
        if not math.isclose(data["fit"]["speed_mps"], recomputed["speed_mps"], rel_tol=1e-9):
            raise ValueError("saved speed does not match the measured inputs")
        if data["fit"]["gear"] != recomputed["gear"]:
            raise ValueError("saved gear does not match the measured inputs")
        return dict(recomputed, calibration_id=digest)
    except (OSError, KeyError, TypeError, AttributeError, OverflowError, ValueError) as exc:
        raise ValueError(f"Cannot load speed calibration {path}: {exc}") from exc


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--template", type=Path, help="Create an empty measurement CSV and exit.")
    parser.add_argument("--input", type=Path, help="CSV containing actual impact measurements.")
    parser.add_argument("--output", type=Path, help="New calibration JSON; existing files are preserved.")
    parser.add_argument("--gravity", type=float, default=9.81)
    args = parser.parse_args(argv)
    try:
        if args.template:
            if args.input or args.output:
                parser.error("--template cannot be combined with --input/--output")
            args.template.parent.mkdir(parents=True, exist_ok=True)
            with args.template.open("x", newline="") as stream:
                csv.writer(stream).writerow(COLUMNS)
            print(f"Empty measurement template: {args.template}")
            return
        if args.input is None or args.output is None:
            parser.error("provide --input and --output, or --template")
        shots, raw = read_measurements(args.input)
        result = build_calibration(shots, raw, args.gravity)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open("x") as stream:
            json.dump(result, stream, indent=2, allow_nan=False)
            stream.write("\n")
        fit = result["fit"]
        print(f"Effective speed: {fit['speed_mps']:.3f} m/s; gear={fit['gear']}; shots={len(shots)}")
        print(f"Impact-height fit RMS: {fit['height_residual_rms_m']*1000:.2f} mm")
        print(f"Calibration: {args.output}; estimates are conditional on measured angle and mounting.")
    except (OSError, ValueError, OverflowError) as exc:
        parser.exit(2, f"error: {exc}\n")


if __name__ == "__main__":
    main()
