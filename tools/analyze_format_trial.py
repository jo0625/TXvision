#!/usr/bin/env python3
"""Analyze one effective-target BGR8/BayerRG8 comparison window."""

import argparse
import json
import re
from pathlib import Path

import numpy as np


DIAGNOSTIC_RE = re.compile(
    r"received=(\d+) published=(\d+) frame_gaps=(\d+) counter_anomalies=(\d+) "
    r"image_build_ms=([0-9.]+)/([0-9.]+) publish_ms=([0-9.]+)/([0-9.]+)"
)
FPS_RE = re.compile(r"Publishing ([0-9.]+) FPS")


def stats(values):
    if not values:
        return None
    values = np.asarray(values, dtype=float)
    return {
        "n": int(values.size),
        "mean": float(values.mean()),
        "std": float(values.std()),
        "p50": float(np.percentile(values, 50)),
        "p95": float(np.percentile(values, 95)),
        "p99": float(np.percentile(values, 99)),
        "max": float(values.max()),
    }


def topic_summary(rows, name, duration, expected_fps):
    samples = [row for row in rows if row["topic"] == name]
    arrival_gaps = [
        (right["mono_ns"] - left["mono_ns"]) / 1e6
        for left, right in zip(samples, samples[1:])
    ]
    stamp_gaps = [
        (right["stamp_ns"] - left["stamp_ns"]) / 1e6
        for left, right in zip(samples, samples[1:])
    ]
    threshold_1_5 = 1500.0 / expected_fps
    threshold_2 = 2000.0 / expected_fps
    def long_gaps(values):
        return {
            "threshold_1_5x_ms": threshold_1_5,
            "count_1_5x": sum(value > threshold_1_5 for value in values),
            "threshold_2x_ms": threshold_2,
            "count_2x": sum(value > threshold_2 for value in values),
        }

    return {
        "count": len(samples),
        "fps": len(samples) / duration,
        "arrival_interval_ms": stats(arrival_gaps),
        "source_stamp_interval_ms": stats(stamp_gaps),
        "long_arrival_gaps": long_gaps(arrival_gaps),
        "long_source_stamp_gaps": long_gaps(stamp_gaps),
    }


def ages(rows):
    return stats([(row["wall_ns"] - row["stamp_ns"]) / 1e6 for row in rows])


def camera_summary(text):
    diagnostics = [match.groups() for match in DIAGNOSTIC_RE.finditer(text)]
    rates = [float(value) for value in FPS_RE.findall(text)]
    return {
        "diagnostic_windows": len(diagnostics),
        "received": sum(int(row[0]) for row in diagnostics),
        "published": sum(int(row[1]) for row in diagnostics),
        "frame_gaps": sum(int(row[2]) for row in diagnostics),
        "counter_anomalies": sum(int(row[3]) for row in diagnostics),
        "reported_fps": stats(rates),
        "image_build_mean_ms": stats([float(row[4]) for row in diagnostics]),
        "image_build_max_ms": max((float(row[5]) for row in diagnostics), default=None),
        "publish_mean_ms": stats([float(row[6]) for row in diagnostics]),
        "publish_max_ms": max((float(row[7]) for row in diagnostics), default=None),
    }


def process_resource(samples, role, key):
    values = []
    for sample in samples:
        process = sample.get("processes", {}).get(role)
        if process is not None:
            values.append(process[key])
    return stats(values)


def resource_summary(samples):
    gpu_samples = [sample["gpu"] for sample in samples if sample.get("gpu")]
    result = {
        "samples": len(samples),
        "system_cpu_percent": stats([sample["system_cpu_percent"] for sample in samples]),
        "system_memory_used_mib": stats(
            [sample["system_memory_used_mib"] for sample in samples]
        ),
    }
    for role in ["camera", "detector", "tracker", "rviz"]:
        result[role] = {
            "cpu_percent": process_resource(samples, role, "cpu_percent"),
            "rss_mib": process_resource(samples, role, "rss_mib"),
        }
    result["gpu"] = {
        "util_percent": stats([sample["util_percent"] for sample in gpu_samples]),
        "memory_mib": stats([sample["memory_mib"] for sample in gpu_samples]),
        "power_w": stats([sample["power_w"] for sample in gpu_samples]),
        "temperature_c": stats([sample["temperature_c"] for sample in gpu_samples]),
    }
    return result


def primary_armor(row):
    armors = row.get("armors", [])
    if not armors:
        return None
    return min(armors, key=lambda armor: armor["distance_to_image_center"])


def quaternion_jitter(quaternions):
    if not quaternions:
        return None
    values = np.asarray(quaternions, dtype=float)
    norms = np.linalg.norm(values, axis=1)
    values = values[norms > 1e-9]
    if not len(values):
        return None
    values /= np.linalg.norm(values, axis=1)[:, None]
    reference = values[0]
    values[np.dot(values, reference) < 0] *= -1
    mean = values.mean(axis=0)
    mean /= np.linalg.norm(mean)
    angles = 2.0 * np.arccos(np.clip(np.abs(values @ mean), 0.0, 1.0))
    return {
        "mean_quaternion_xyzw": mean.tolist(),
        "angular_deviation_deg": stats(np.degrees(angles).tolist()),
    }


def binned_path_length(samples, positions, period_ns=100_000_000):
    if len(samples) < 2:
        return 0.0
    bins = {}
    start = samples[0]["wall_ns"]
    for row, position in zip(samples, positions):
        key = (row["wall_ns"] - start) // period_ns
        bins.setdefault(key, []).append(position)
    points = np.asarray([np.median(bins[key], axis=0) for key in sorted(bins)])
    if len(points) < 2:
        return 0.0
    return float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum())


def position_summary(samples, positions):
    if not positions:
        return None
    values = np.asarray(positions, dtype=float)
    median = np.median(values, axis=0)
    deviations = np.linalg.norm(values - median, axis=1)
    ranges = np.linalg.norm(values, axis=1)
    return {
        "n": len(values),
        "mean_xyz_m": values.mean(axis=0).tolist(),
        "median_xyz_m": median.tolist(),
        "std_xyz_m": values.std(axis=0).tolist(),
        "peak_to_peak_xyz_m": np.ptp(values, axis=0).tolist(),
        "deviation_from_median_m": stats(deviations.tolist()),
        "range_m": stats(ranges.tolist()),
        "range_span_m": float(np.ptp(ranges)),
        "position_span_m": float(np.linalg.norm(np.ptp(values, axis=0))),
        "path_length_10hz_m": binned_path_length(samples, values),
    }


def armor_pose_summary(rows):
    pairs = [(row, primary_armor(row)) for row in rows]
    pairs = [(row, armor) for row, armor in pairs if armor is not None]
    samples = [row for row, _ in pairs]
    armors = [armor for _, armor in pairs]
    result = position_summary(samples, [armor["position"] for armor in armors])
    if result is None:
        return None
    result["orientation"] = quaternion_jitter(
        [armor["orientation"] for armor in armors]
    )
    numbers, number_counts = np.unique(
        [armor["number"] for armor in armors], return_counts=True
    )
    types, type_counts = np.unique(
        [armor["type"] for armor in armors], return_counts=True
    )
    result["number_counts"] = {
        str(name): int(count) for name, count in zip(numbers, number_counts)
    }
    result["type_counts"] = {
        str(name): int(count) for name, count in zip(types, type_counts)
    }
    return result


def tracker_pose_summary(rows):
    tracking = [row for row in rows if row.get("tracking")]
    result = position_summary(tracking, [row["position"] for row in tracking])
    if result is None:
        return None
    result["yaw_rad"] = stats([row["yaw"] for row in tracking])
    result["speed_mps"] = stats(
        [float(np.linalg.norm(row["velocity"])) for row in tracking]
    )
    return result


def lock_episodes(rows):
    if not rows:
        return {"drop_count": 0, "relock_ms": None, "open_drop": False}
    first_locked = next((index for index, row in enumerate(rows) if row.get("tracking")), None)
    if first_locked is None:
        return {"drop_count": 0, "relock_ms": None, "open_drop": True}
    drops = []
    drop_start = None
    previous = True
    for row in rows[first_locked + 1:]:
        locked = bool(row.get("tracking"))
        if previous and not locked:
            drop_start = row["wall_ns"]
        elif not previous and locked and drop_start is not None:
            drops.append((row["wall_ns"] - drop_start) / 1e6)
            drop_start = None
        previous = locked
    return {
        "drop_count": len(drops) + (1 if drop_start is not None else 0),
        "relock_ms": stats(drops),
        "open_drop": drop_start is not None,
    }


def empty_runs(armors, tracker, minimum_ms):
    runs = []
    start = None
    for index, row in enumerate(armors):
        if row.get("count", 0) == 0 and start is None:
            start = index
        if row.get("count", 0) > 0 and start is not None:
            first = armors[start]
            duration_ms = (row["wall_ns"] - first["wall_ns"]) / 1e6
            if duration_ms >= minimum_ms:
                relock = next(
                    (
                        (target["wall_ns"] - row["wall_ns"]) / 1e6
                        for target in tracker
                        if target["wall_ns"] >= row["wall_ns"] and target.get("tracking")
                    ),
                    None,
                )
                runs.append(
                    {
                        "empty_start_wall_ns": first["wall_ns"],
                        "first_nonempty_wall_ns": row["wall_ns"],
                        "empty_duration_ms": duration_ms,
                        "relock_after_first_nonempty_ms": relock,
                    }
                )
            start = None
    return runs


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--prefix", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--format", required=True)
    parser.add_argument("--mode", choices=["static", "dynamic"], required=True)
    parser.add_argument("--expected-fps", type=float, required=True)
    parser.add_argument("--minimum-empty-ms", type=float, default=100.0)
    args = parser.parse_args()

    prefix = Path(args.prefix)
    rows = json.loads(prefix.with_name(prefix.name + "_observations.json").read_text())
    resources = json.loads(prefix.with_name(prefix.name + "_resources.json").read_text())
    camera_log = prefix.with_name(prefix.name + "_camera.log").read_text()
    vision_log = prefix.with_name(prefix.name + "_vision.log").read_text()
    visual_path = prefix.with_name(prefix.name + "_visual.json")
    duration = (rows[-1]["wall_ns"] - rows[0]["wall_ns"]) / 1e9
    armors = [row for row in rows if row["topic"] == "armors"]
    tracker = [row for row in rows if row["topic"] == "tracker"]
    nonempty = [row for row in armors if row.get("count", 0) > 0]
    empty = [row for row in armors if row.get("count", 0) == 0]
    locked = [row for row in tracker if row.get("tracking")]
    unlocked = [row for row in tracker if not row.get("tracking")]

    result = {
        "format": args.format,
        "mode": args.mode,
        "expected_fps": args.expected_fps,
        "duration_s": duration,
        "first_wall_ns": rows[0]["wall_ns"],
        "last_wall_ns": rows[-1]["wall_ns"],
        "detector": {
            **topic_summary(rows, "armors", duration, args.expected_fps),
            "nonempty_count": len(nonempty),
            "empty_count": len(empty),
            "effective_rate": len(nonempty) / len(armors) if armors else 0.0,
            "valid_age_ms": ages(nonempty),
            "empty_age_ms": ages(empty),
        },
        "tracker": {
            **topic_summary(rows, "tracker", duration, args.expected_fps),
            "locked_count": len(locked),
            "unlocked_count": len(unlocked),
            "lock_rate": len(locked) / len(tracker) if tracker else 0.0,
            "valid_age_ms": ages(locked),
            "unlocked_age_ms": ages(unlocked),
            **lock_episodes(tracker),
        },
        "preview": topic_summary(rows, "result_img", duration, 15.0),
        "camera": camera_summary(camera_log),
        "pnp_pose": armor_pose_summary(nonempty),
        "tracker_pose": tracker_pose_summary(tracker),
        "empty_runs": empty_runs(armors, tracker, args.minimum_empty_ms),
        "resources": resource_summary(resources),
        "tracker_reset_warnings": vision_log.count("Invalid Armors dt="),
        "detector_errors": vision_log.count("[ERROR]"),
        "visual": json.loads(visual_path.read_text()) if visual_path.exists() else None,
    }
    Path(args.output).write_text(json.dumps(result, indent=2) + "\n")


if __name__ == "__main__":
    main()
