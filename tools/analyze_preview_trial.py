#!/usr/bin/env python3
"""Summarize one controlled camera/detector/tracker preview trial."""

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
        "p50": float(np.percentile(values, 50)),
        "p95": float(np.percentile(values, 95)),
        "p99": float(np.percentile(values, 99)),
        "max": float(values.max()),
    }


def topic_summary(rows, name, duration):
    samples = [row for row in rows if row["topic"] == name]
    arrival_gaps = [
        (right["mono_ns"] - left["mono_ns"]) / 1e6
        for left, right in zip(samples, samples[1:])
    ]
    stamp_gaps = [
        (right["stamp_ns"] - left["stamp_ns"]) / 1e6
        for left, right in zip(samples, samples[1:])
    ]
    ages = [(row["wall_ns"] - row["stamp_ns"]) / 1e6 for row in samples]
    start_ns = samples[0]["wall_ns"] if samples else 0
    end_ns = samples[-1]["wall_ns"] if samples else 0
    first_ages = [
        (row["wall_ns"] - row["stamp_ns"]) / 1e6
        for row in samples
        if row["wall_ns"] < start_ns + 10_000_000_000
    ]
    last_ages = [
        (row["wall_ns"] - row["stamp_ns"]) / 1e6
        for row in samples
        if row["wall_ns"] > end_ns - 10_000_000_000
    ]
    return {
        "count": len(samples),
        "fps": len(samples) / duration,
        "arrival_interval_ms": stats(arrival_gaps),
        "source_stamp_interval_ms": stats(stamp_gaps),
        "age_ms": stats(ages),
        "first_10s_age_ms": stats(first_ages),
        "last_10s_age_ms": stats(last_ages),
    }


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
        "system_memory_used_mib": stats([sample["system_memory_used_mib"] for sample in samples]),
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
    }
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--prefix", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()

    prefix = Path(args.prefix)
    rows = json.loads(prefix.with_name(prefix.name + "_observations.json").read_text())
    resources = json.loads(prefix.with_name(prefix.name + "_resources.json").read_text())
    camera_log = prefix.with_name(prefix.name + "_camera.log").read_text()
    vision_log = prefix.with_name(prefix.name + "_vision.log").read_text()
    duration = (rows[-1]["wall_ns"] - rows[0]["wall_ns"]) / 1e9
    armors = [row for row in rows if row["topic"] == "armors"]
    tracker = [row for row in rows if row["topic"] == "tracker"]

    result = {
        "duration_s": duration,
        "first_wall_ns": rows[0]["wall_ns"],
        "last_wall_ns": rows[-1]["wall_ns"],
        "nonempty_armors": sum(row.get("count", 0) > 0 for row in armors),
        "nonempty_rate": (
            sum(row.get("count", 0) > 0 for row in armors) / len(armors) if armors else 0
        ),
        "tracking_messages": sum(bool(row.get("tracking")) for row in tracker),
        "tracking_rate": (
            sum(bool(row.get("tracking")) for row in tracker) / len(tracker) if tracker else 0
        ),
        "tracker_reset_warnings": vision_log.count("Invalid Armors dt="),
        "detector_errors": vision_log.count("[ERROR]"),
        "camera": camera_summary(camera_log),
        "armors": topic_summary(rows, "armors", duration),
        "tracker": topic_summary(rows, "tracker", duration),
        "result_img": topic_summary(rows, "result_img", duration),
        "resources": resource_summary(resources),
    }
    Path(args.output).write_text(json.dumps(result, indent=2) + "\n")


if __name__ == "__main__":
    main()
