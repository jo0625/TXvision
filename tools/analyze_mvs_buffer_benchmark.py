#!/usr/bin/env python3
"""Analyze raw MVS buffer benchmark data without third-party dependencies."""

from __future__ import annotations

import argparse
import csv
import json
import statistics
from collections import defaultdict
from pathlib import Path
from typing import Any, Iterable


INTEGER_COLUMNS = {
    "node_count", "pause_ms", "repeat", "sequence", "phase_elapsed_ns",
    "get_started_ns", "app_get_ns", "app_realtime_ns", "frame_number",
    "device_timestamp_raw", "sdk_host_timestamp_raw", "frame_bytes", "width",
    "height", "pixel_type", "sdk_queue_after_get", "queue_query_code", "frame_gap_before",
    "duplicate_or_reordered", "process_ns", "process_code", "free_code", "rss_bytes",
    "checksum",
}


def percentile(values: Iterable[float], fraction: float) -> float | None:
    ordered = sorted(values)
    if not ordered:
        return None
    position = (len(ordered) - 1) * fraction
    lower = int(position)
    upper = min(lower + 1, len(ordered) - 1)
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (position - lower)


def distribution(values: Iterable[float]) -> dict[str, float | int | None]:
    data = list(values)
    if not data:
        return {"n": 0, "p50": None, "p95": None, "p99": None, "max": None, "min": None}
    return {
        "n": len(data),
        "p50": percentile(data, 0.50),
        "p95": percentile(data, 0.95),
        "p99": percentile(data, 0.99),
        "max": max(data),
        "min": min(data),
    }


def read_config(path: Path) -> dict[str, str]:
    with path.open(newline="") as stream:
        return {row["key"]: row["value"] for row in csv.DictReader(stream)}


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="") as stream:
        return list(csv.DictReader(stream))


def read_frames(directory: Path) -> list[dict[str, Any]]:
    frames: list[dict[str, Any]] = []
    for path in sorted(directory.glob("raw_frames_nodes_*.csv")):
        for row in read_csv(path):
            parsed: dict[str, Any] = dict(row)
            for key in INTEGER_COLUMNS:
                parsed[key] = int(row[key])
            frames.append(parsed)
    if not frames:
        raise RuntimeError(f"no raw_frames_nodes_*.csv files found in {directory}")
    return frames


def best_calibration(rows: list[dict[str, str]], nodes: int, position: str) -> dict[str, float]:
    candidates = []
    for row in rows:
        if int(row["node_count"]) != nodes or row["position"] != position:
            continue
        before = int(row["host_before_ns"])
        after = int(row["host_after_ns"])
        increment = int(row["timestamp_increment_ns"])
        # Keep raw latch units: this firmware's XML unit labels are unreliable.
        device_ns = int(row["device_timestamp_ns"])
        midpoint = (before + after) / 2.0
        candidates.append({
            "host_ns": midpoint,
            "device_raw": device_ns,
            "offset_ns": midpoint - device_ns,
            "half_rtt_ns": (after - before) / 2.0,
            "increment_ns": float(increment),
        })
    if not candidates:
        raise RuntimeError(f"missing {position} clock calibration for {nodes} nodes")
    return min(candidates, key=lambda item: item["half_rtt_ns"])


def apply_clock_calibration(
    frames: list[dict[str, Any]], calibration_rows: list[dict[str, str]]
) -> dict[int, dict[str, float]]:
    calibration: dict[int, dict[str, float]] = {}
    for nodes in sorted({row["node_count"] for row in frames}):
        before = best_calibration(calibration_rows, nodes, "before")
        after = best_calibration(calibration_rows, nodes, "after")
        span_ns = after["host_ns"] - before["host_ns"]
        device_span = after["device_raw"] - before["device_raw"]
        if span_ns <= 0 or device_span <= 0:
            raise RuntimeError(f"non-monotonic clock calibration for {nodes} nodes")
        scale = span_ns / device_span
        # Firmware may report 100 MHz ticks despite the XML's ns label.
        # Estimate scale from actual bracketing latches, never assume XML units.
        residuals = []
        for sample in calibration_rows:
            if int(sample["node_count"]) != nodes:
                continue
            mid = (int(sample["host_before_ns"]) + int(sample["host_after_ns"])) / 2
            mapped = before["host_ns"] + (int(sample["device_timestamp_ns"]) - before["device_raw"]) * scale
            residuals.append(abs(mid - mapped))
        error_ns = max(residuals, default=0) + max(before["half_rtt_ns"], after["half_rtt_ns"])
        calibration[nodes] = {
            "before_host_ns": before["host_ns"],
            "after_host_ns": after["host_ns"],
            "before_offset_ns": before["offset_ns"],
            "after_offset_ns": after["offset_ns"],
            "best_half_rtt_ms": max(before["half_rtt_ns"], after["half_rtt_ns"]) / 1e6,
            "scale_ns_per_raw_unit": scale,
            "frequency_hz": 1e9 / scale,
            "calibration_error_ms": error_ns / 1e6,
            "timestamp_increment_ns": before["increment_ns"],
        }
        for frame in (item for item in frames if item["node_count"] == nodes):
            mapped_camera_ns = before["host_ns"] + (frame["device_timestamp_raw"] - before["device_raw"]) * scale
            frame["camera_mapped_steady_ns"] = int(round(mapped_camera_ns))
            frame["camera_to_app_ms"] = (frame["app_get_ns"] - mapped_camera_ns) / 1e6
            frame["clock_error_ms"] = error_ns / 1e6
            # Host timestamps are wall-clock milliseconds on this SDK/firmware.
            # Preserve quantization; fractional results do not imply sub-ms precision.
            host_raw = frame.get("sdk_host_timestamp_raw", 0)
            realtime_ns = frame.get("app_realtime_ns", 0)
            frame["sdk_host_to_app_ms"] = (
                (realtime_ns - host_raw * 1_000_000) / 1e6
                if host_raw > 0 and realtime_ns > 0 else None)
    return calibration


def event_index(rows: list[dict[str, str]]) -> dict[tuple[int, str], list[dict[str, Any]]]:
    result: dict[tuple[int, str], list[dict[str, Any]]] = defaultdict(list)
    for raw in rows:
        row: dict[str, Any] = dict(raw)
        for key in ("node_count", "pause_ms", "repeat", "steady_ns", "realtime_ns", "code"):
            row[key] = int(raw[key])
        result[(row["node_count"], row["phase"])].append(row)
    return result


def median_window(rows: list[dict[str, Any]], key: str, first: bool) -> float | None:
    if not rows:
        return None
    edge = min(row["app_get_ns"] for row in rows) if first else max(row["app_get_ns"] for row in rows)
    if first:
        values = [row[key] for row in rows if row["app_get_ns"] <= edge + 10_000_000_000]
    else:
        values = [row[key] for row in rows if row["app_get_ns"] >= edge - 10_000_000_000]
    return statistics.median(values) if values else None


def summarize_stage(
    rows: list[dict[str, Any]], events: list[dict[str, Any]], phase: str
) -> dict[str, Any]:
    if not rows:
        return {"phase": phase, "frames": 0}
    ordered = sorted(rows, key=lambda row: row["app_get_ns"])
    span_s = (ordered[-1]["app_get_ns"] - ordered[0]["app_get_ns"]) / 1e9
    queue = [row["sdk_queue_after_get"] for row in ordered if row["queue_query_code"] == 0]
    latency = [row["camera_to_app_ms"] for row in ordered]
    rss = [row["rss_bytes"] for row in ordered if row["rss_bytes"]]
    first_queue = median_window(ordered, "sdk_queue_after_get", True)
    last_queue = median_window(ordered, "sdk_queue_after_get", False)
    first_latency = median_window(ordered, "camera_to_app_ms", True)
    last_latency = median_window(ordered, "camera_to_app_ms", False)
    bins: dict[int, list[float]] = defaultdict(list)
    for row in ordered:
        bins[(row["app_get_ns"] - ordered[0]["app_get_ns"]) // 10_000_000_000].append(
            row["camera_to_app_ms"])
    # A tiny trailing bin is not comparable to a full ten-second window.
    bin_medians = [statistics.median(values) for values in bins.values()
                   if len(values) >= max(2, len(ordered) / max(span_s, 1) * 5)]
    errors = [event for event in events if event["event"].endswith("error")]
    timeouts = [event for event in events if event["event"] == "get_timeout"]
    phase_start = next((event for event in events if event["event"] == "phase_start"), None)
    phase_end = next((event for event in events if event["event"] == "phase_end"), None)
    return {
        "phase": phase,
        "frames": len(ordered),
        "span_s": span_s,
        "wall_duration_s": (phase_end["steady_ns"] - phase_start["steady_ns"]) / 1e9
        if phase_start and phase_end else None,
        "fps": (len(ordered) - 1) / span_s if span_s > 0 else None,
        "throughput_mib_s": (sum(row["frame_bytes"] for row in ordered) / span_s / 2**20)
        if span_s > 0 else None,
        "queue": distribution(queue),
        "queue_first_10s_p50": first_queue,
        "queue_last_10s_p50": last_queue,
        "queue_growth_frames": (last_queue - first_queue)
        if first_queue is not None and last_queue is not None else None,
        "queue_sustained_growth": bool(
            first_queue is not None and last_queue is not None and last_queue > first_queue + 1.0
        ),
        "frame_gaps": sum(row["frame_gap_before"] for row in ordered),
        "duplicate_or_reordered": sum(row["duplicate_or_reordered"] for row in ordered),
        "timeouts": len(timeouts),
        "interface_errors": len(errors),
        "latency_ms": distribution(latency),
        "latency_first_10s_p50_ms": first_latency,
        "latency_last_10s_p50_ms": last_latency,
        "latency_p50_drift_ms": (last_latency - first_latency)
        if first_latency is not None and last_latency is not None else None,
        "latency_10s_bin_medians_ms": bin_medians,
        "latency_10s_median_range_ms": max(bin_medians) - min(bin_medians)
        if bin_medians else None,
        "sdk_host_to_app_ms": distribution(
            row["sdk_host_to_app_ms"] for row in ordered
            if row.get("sdk_host_to_app_ms") is not None),
        "process_ms": distribution(row["process_ns"] / 1e6 for row in ordered),
        "rss_bytes": distribution(rss),
        "clock_error_ms_max": max(row["clock_error_ms"] for row in ordered),
    }


def recovery_time_ms(
    rows: list[dict[str, Any]], resume_ns: int, threshold_ms: float, consecutive: int = 5
) -> float | None:
    run = 0
    for row in sorted((row for row in rows if row["app_get_ns"] >= resume_ns),
                      key=lambda row: row["app_get_ns"]):
        if row["camera_to_app_ms"] <= threshold_ms:
            run += 1
            if run >= consecutive:
                return (row["app_get_ns"] - resume_ns) / 1e6
        else:
            run = 0
    return None


def summarize_pause_repetition(
    rows: list[dict[str, Any]], events: list[dict[str, Any]], normal_p95_ms: float,
    measurement_floor_ms: float,
) -> dict[str, Any]:
    pause_end = next((event for event in events if event["event"] == "pause_end"), None)
    pause_start = next((event for event in events if event["event"] == "pause_start"), None)
    if not rows or pause_end is None or pause_start is None:
        return {"complete": False}
    ordered = sorted(rows, key=lambda row: row["app_get_ns"])
    resume_ns = pause_end["steady_ns"]
    after_resume = [row for row in ordered if row["app_get_ns"] >= resume_ns]
    if not after_resume:
        return {"complete": False}
    first = after_resume[0]
    drain = next((row for row in after_resume if row["sdk_queue_after_get"] == 0), None)
    errors = [event for event in events if event["event"].endswith("error")]
    timeouts = [event for event in events if event["event"] == "get_timeout"]
    gaps = sum(row["frame_gap_before"] for row in after_resume)
    duplicates = sum(row["duplicate_or_reordered"] for row in after_resume)
    threshold = normal_p95_ms + measurement_floor_ms
    first_normal = next((row for row in after_resume
                         if row["camera_to_app_ms"] <= threshold), None)
    return {
        "complete": True,
        "requested_pause_ms": first["pause_ms"],
        "actual_pause_ms": (pause_end["steady_ns"] - pause_start["steady_ns"]) / 1e6,
        "first_frame_age_ms": first["camera_to_app_ms"],
        "first_queue_after_get": first["sdk_queue_after_get"],
        "max_queue_after_get": max(row["sdk_queue_after_get"] for row in after_resume),
        "backlog_drain_ms": (drain["app_get_ns"] - resume_ns) / 1e6 if drain else None,
        "local_normal_p95_ms": normal_p95_ms,
        "normal_latency_threshold_ms": threshold,
        "first_normal_latency_ms": (first_normal["app_get_ns"] - resume_ns) / 1e6
        if first_normal else None,
        "normal_latency_recovery_ms": recovery_time_ms(
            after_resume, resume_ns, normal_p95_ms + measurement_floor_ms),
        "frame_gaps": gaps,
        "duplicate_or_reordered": duplicates,
        "timeouts": len(timeouts),
        "interface_errors": len(errors),
        "lossless": gaps == 0 and duplicates == 0 and not timeouts and not errors and drain is not None
        and recovery_time_ms(after_resume, resume_ns, normal_p95_ms + measurement_floor_ms) is not None,
    }


def usb_session_counts(runtime: dict[str, str], rows: list[dict[str, Any]]) -> dict[str, Any]:
    """Validate session scope against all recorded frames, including warmup.

    Before was sampled before StartGrabbing and belongs to the previous session.
    Never subtract it from the after counter: StartGrabbing resets these counters.
    """
    expected = len(rows) + sum(row["frame_gap_before"] for row in rows)
    after = int(runtime["usb_received_after"])
    verified = after == expected and not any(row["duplicate_or_reordered"] for row in rows)
    return {
        "usb_received_before_start_raw": int(runtime["usb_received_before"]),
        "usb_received_after_raw": after,
        "usb_received_frames": after if verified else None,
        "usb_error_frames": int(runtime["usb_errors_after"]) if verified else None,
        "usb_counter_scope": "start_grabbing_session_verified_against_frames_plus_gaps"
        if verified else "unverified_raw_counters_only",
        "usb_expected_received_frames": expected,
    }


def lower_latency_evidence(gain: float | None, floor: float, temporal_range: float) -> bool:
    # Observed temporal variation is a stability diagnostic, not a clock error bound.
    return gain is not None and temporal_range <= floor and gain > floor


def flatten_distribution(prefix: str, values: dict[str, Any], output: dict[str, Any]) -> None:
    for key in ("p50", "p95", "p99", "max"):
        output[f"{prefix}_{key}"] = values.get(key)


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    if not rows:
        return
    fields: list[str] = []
    for row in rows:
        for key in row:
            if key not in fields:
                fields.append(key)
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def fmt(value: Any, digits: int = 3, missing: str = "-") -> str:
    if value is None:
        return missing
    if isinstance(value, bool):
        return "是" if value else "否"
    if isinstance(value, (int, float)):
        return f"{value:.{digits}f}"
    return str(value)


def mib(value: float | int | None) -> str:
    return fmt(value / 2**20 if value is not None else None, 2)


def protocol_checks(
    config: dict[str, str], nodes: list[int], pause_rows: list[dict[str, Any]],
    stage_results: dict[tuple[int, str], dict[str, Any]],
) -> list[str]:
    failures = []
    if float(config["continuous_seconds"]) < 60:
        failures.append("连续快速取帧不足 60 秒")
    if float(config["load_seconds"]) < 60:
        failures.append("负载阶段不足 60 秒")
    if nodes != [2, 4, 8, 16, 32, 64]:
        failures.append(f"节点集合为 {nodes}，不是 2/4/8/16/32/64")
    for nodes_value in nodes:
        for phase in ("continuous", "simulated_load"):
            duration = stage_results.get((nodes_value, phase), {}).get("wall_duration_s")
            if duration is None or duration < 60.0:
                failures.append(
                    f"{nodes_value} 节点的 {phase} 实际计时时长为 {duration} 秒，不足 60 秒")
        for pause_ms in (20, 50, 100, 250):
            complete = [row for row in pause_rows if row.get("node_count") == nodes_value
                        and row.get("pause_ms") == pause_ms and row.get("complete")]
            if len(complete) < 3:
                failures.append(f"{nodes_value} 节点、{pause_ms} ms 暂停只有 {len(complete)} 次完整重复")
    return failures


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("run_dir", type=Path)
    args = parser.parse_args()
    directory = args.run_dir.resolve()

    config = read_config(directory / "configuration.csv")
    frames = read_frames(directory)
    calibration_rows = read_csv(directory / "clock_calibration.csv")
    events_raw = read_csv(directory / "events.csv")
    runtime_rows = read_csv(directory / "runtime_memory_usb.csv")
    calibrations = apply_clock_calibration(frames, calibration_rows)
    events = event_index(events_raw)

    formal_frames = [row for row in frames if not row["phase"].endswith("_excluded")]
    write_csv(directory / "per_frame.csv", formal_frames)

    nodes_values = sorted({row["node_count"] for row in formal_frames})
    stage_results: dict[tuple[int, str], dict[str, Any]] = {}
    stage_csv: list[dict[str, Any]] = []
    for nodes in nodes_values:
        phases = sorted({row["phase"] for row in formal_frames if row["node_count"] == nodes})
        for phase in phases:
            selected = [row for row in formal_frames
                        if row["node_count"] == nodes and row["phase"] == phase]
            result = summarize_stage(selected, events.get((nodes, phase), []), phase)
            stage_results[(nodes, phase)] = result
            flat = {
                "node_count": nodes,
                "phase": phase,
                "frames": result.get("frames"),
                "span_s": result.get("span_s"),
                "wall_duration_s": result.get("wall_duration_s"),
                "fps": result.get("fps"),
                "throughput_mib_s": result.get("throughput_mib_s"),
                "frame_gaps": result.get("frame_gaps"),
                "duplicate_or_reordered": result.get("duplicate_or_reordered"),
                "timeouts": result.get("timeouts"),
                "interface_errors": result.get("interface_errors"),
                "queue_first_10s_p50": result.get("queue_first_10s_p50"),
                "queue_last_10s_p50": result.get("queue_last_10s_p50"),
                "queue_growth_frames": result.get("queue_growth_frames"),
                "queue_sustained_growth": result.get("queue_sustained_growth"),
                "latency_first_10s_p50_ms": result.get("latency_first_10s_p50_ms"),
                "latency_last_10s_p50_ms": result.get("latency_last_10s_p50_ms"),
                "latency_p50_drift_ms": result.get("latency_p50_drift_ms"),
                "latency_10s_median_range_ms": result.get("latency_10s_median_range_ms"),
                "clock_error_ms_max": result.get("clock_error_ms_max"),
            }
            flatten_distribution("queue", result.get("queue", {}), flat)
            flatten_distribution("latency_ms", result.get("latency_ms", {}), flat)
            flatten_distribution("sdk_host_to_app_ms", result.get("sdk_host_to_app_ms", {}), flat)
            flatten_distribution("process_ms", result.get("process_ms", {}), flat)
            flatten_distribution("rss_bytes", result.get("rss_bytes", {}), flat)
            stage_csv.append(flat)
    write_csv(directory / "stage_summary.csv", stage_csv)

    pause_results: list[dict[str, Any]] = []
    for nodes in nodes_values:
        continuous = stage_results[(nodes, "continuous")]
        normal_p95 = continuous["latency_ms"]["p95"]
        measurement_floor = max(1.0, continuous["clock_error_ms_max"])
        pause_phases = sorted(
            {row["phase"] for row in formal_frames
             if row["node_count"] == nodes and row["phase"].startswith("pause_")}
        )
        for phase in pause_phases:
            selected = [row for row in formal_frames
                        if row["node_count"] == nodes and row["phase"] == phase]
            phase_events = events.get((nodes, phase), [])
            start = next((event["steady_ns"] for event in phase_events
                          if event["event"] == "pause_start"), None)
            local = sorted((row for row in frames if row["node_count"] == nodes
                            and start is not None and row["app_get_ns"] < start
                            and row["queue_query_code"] == 0 and row["sdk_queue_after_get"] == 0),
                           key=lambda row: row["app_get_ns"])[-100:]
            baseline = percentile((row["camera_to_app_ms"] for row in local), .95)
            result = summarize_pause_repetition(
                selected, phase_events, baseline if baseline is not None else normal_p95,
                measurement_floor)
            result["local_baseline_frame_count"] = len(local)
            result.update({
                "node_count": nodes,
                "phase": phase,
                "pause_ms": selected[0]["pause_ms"] if selected else None,
                "repeat": selected[0]["repeat"] if selected else None,
            })
            pause_results.append(result)
    write_csv(directory / "pause_repetitions.csv", pause_results)

    payload_size = int(config["payload_size_bytes"])
    runtime_by_node = {int(row["node_count"]): row for row in runtime_rows}
    comparison: list[dict[str, Any]] = []
    for nodes in nodes_values:
        continuous = stage_results[(nodes, "continuous")]
        load = stage_results[(nodes, "simulated_load")]
        node_frames = [row for row in formal_frames if row["node_count"] == nodes]
        rss = distribution(row["rss_bytes"] for row in node_frames if row["rss_bytes"])
        row: dict[str, Any] = {
            "node_count": nodes,
            "theoretical_payload_memory_bytes": nodes * payload_size,
            "actual_rss_p50_bytes": rss["p50"],
            "actual_rss_max_bytes": rss["max"],
            "continuous_fps": continuous["fps"],
            "continuous_throughput_mib_s": continuous["throughput_mib_s"],
            "continuous_queue_p50": continuous["queue"]["p50"],
            "continuous_queue_p95": continuous["queue"]["p95"],
            "continuous_queue_max": continuous["queue"]["max"],
            "continuous_queue_growth": continuous["queue_growth_frames"],
            "continuous_frame_gaps": continuous["frame_gaps"],
            "continuous_duplicate_or_reordered": continuous["duplicate_or_reordered"],
            "continuous_timeouts": continuous["timeouts"],
            "continuous_errors": continuous["interface_errors"],
            "continuous_latency_p50_ms": continuous["latency_ms"]["p50"],
            "continuous_latency_p95_ms": continuous["latency_ms"]["p95"],
            "continuous_latency_p99_ms": continuous["latency_ms"]["p99"],
            "continuous_latency_max_ms": continuous["latency_ms"]["max"],
            "load_fps": load["fps"],
            "load_throughput_mib_s": load["throughput_mib_s"],
            "load_queue_p50": load["queue"]["p50"],
            "load_queue_p95": load["queue"]["p95"],
            "load_queue_max": load["queue"]["max"],
            "load_queue_growth": load["queue_growth_frames"],
            "load_frame_gaps": load["frame_gaps"],
            "load_duplicate_or_reordered": load["duplicate_or_reordered"],
            "load_timeouts": load["timeouts"],
            "load_errors": load["interface_errors"],
            "load_latency_p50_ms": load["latency_ms"]["p50"],
            "load_latency_p95_ms": load["latency_ms"]["p95"],
            "load_latency_p99_ms": load["latency_ms"]["p99"],
            "load_latency_max_ms": load["latency_ms"]["max"],
            "clock_calibration_error_ms": calibrations[nodes]["calibration_error_ms"],
            "clock_frequency_hz": calibrations[nodes]["frequency_hz"],
        }
        for label, stage in (("continuous", continuous), ("load", load)):
            flatten_distribution(f"{label}_sdk_host_to_app_ms", stage["sdk_host_to_app_ms"], row)
            row[f"{label}_latency_10s_median_range_ms"] = stage["latency_10s_median_range_ms"]
        tested_lossless = []
        for pause_ms in (20, 50, 100, 250):
            reps = [item for item in pause_results
                    if item.get("node_count") == nodes and item.get("pause_ms") == pause_ms]
            complete = len(reps) >= 3 and all(item.get("complete") for item in reps)
            lossless = complete and all(item.get("lossless") for item in reps)
            row[f"pause_{pause_ms}_lossless"] = lossless
            row[f"pause_{pause_ms}_frame_gaps"] = sum(item.get("frame_gaps", 0) for item in reps)
            row[f"pause_{pause_ms}_first_age_max_ms"] = max(
                (item["first_frame_age_ms"] for item in reps if item.get("complete")), default=None)
            row[f"pause_{pause_ms}_queue_max"] = max(
                (item["max_queue_after_get"] for item in reps if item.get("complete")), default=None)
            row[f"pause_{pause_ms}_drain_max_ms"] = max(
                (item["backlog_drain_ms"] for item in reps
                 if item.get("backlog_drain_ms") is not None), default=None)
            row[f"pause_{pause_ms}_latency_recovery_max_ms"] = max(
                (item["normal_latency_recovery_ms"] for item in reps
                 if item.get("normal_latency_recovery_ms") is not None), default=None)
            if lossless:
                tested_lossless.append(pause_ms)
        row["max_tested_lossless_pause_ms"] = max(tested_lossless, default=0)
        runtime = runtime_by_node.get(nodes)
        if runtime:
            row.update(usb_session_counts(runtime, [item for item in frames
                                                   if item["node_count"] == nodes]))
            row["rss_before_start_bytes"] = int(runtime["rss_before_start_bytes"])
            row["rss_after_start_bytes"] = int(runtime["rss_after_start_bytes"])
            row["rss_after_stop_bytes"] = int(runtime["rss_after_stop_bytes"])
        comparison.append(row)
    write_csv(directory / "configuration_comparison.csv", comparison)

    failures = protocol_checks(config, nodes_values, pause_results, stage_results)
    restoration_path = directory / "restoration_status.txt"
    if not restoration_path.exists() or restoration_path.read_text().strip() != "restoration_verified":
        failures.append("相机设置恢复未通过读回验证，请查看 restoration_status.txt")
    valid_protocol = not failures
    by_node = {row["node_count"]: row for row in comparison}
    measurement_floor_ms = max(
        1.0, max(row["clock_calibration_error_ms"] for row in comparison))
    normal_latency_values = [row["continuous_latency_p50_ms"] for row in comparison] + [
        row["load_latency_p50_ms"] for row in comparison]
    normal_latency_spread = max(normal_latency_values) - min(normal_latency_values)
    all_normal_empty = all(
        row["continuous_queue_max"] == 0 and row["load_queue_max"] == 0
        and row["continuous_frame_gaps"] == 0 and row["load_frame_gaps"] == 0
        and row["continuous_duplicate_or_reordered"] == 0 and row["load_duplicate_or_reordered"] == 0
        and row["continuous_timeouts"] == 0 and row["load_timeouts"] == 0
        and row["continuous_errors"] == 0 and row["load_errors"] == 0
        for row in comparison
    )
    normal_stages = [stage_results[(nodes, phase)] for nodes in nodes_values
                     for phase in ("continuous", "simulated_load")]
    temporal_range = max((stage["latency_10s_median_range_ms"] or 0) for stage in normal_stages)
    camera_latency_stable = temporal_range <= measurement_floor_ms
    host_medians = [stage["sdk_host_to_app_ms"]["p50"] for stage in normal_stages
                    if stage["sdk_host_to_app_ms"]["p50"] is not None]
    host_spread = max(host_medians) - min(host_medians) if host_medians else None
    no_latency_reason = all_normal_empty and (
        (camera_latency_stable and normal_latency_spread <= measurement_floor_ms)
        or (not camera_latency_stable and host_spread is not None
            and host_spread <= measurement_floor_ms))
    sixteen = by_node.get(16)
    sixteen_enough = bool(sixteen and
        sixteen["continuous_frame_gaps"] == 0 and sixteen["load_frame_gaps"] == 0 and
        sixteen["continuous_timeouts"] == 0 and sixteen["load_timeouts"] == 0 and
        sixteen["continuous_errors"] == 0 and sixteen["load_errors"] == 0 and
        sixteen["continuous_duplicate_or_reordered"] == 0 and sixteen["load_duplicate_or_reordered"] == 0 and
        sixteen["continuous_fps"] >= .98 * float(config["actual_frame_rate"]) and
        sixteen["load_fps"] >= .98 * float(config["actual_frame_rate"]) and
        sixteen["continuous_queue_p95"] <= 4 and sixteen["load_queue_p95"] <= 4 and
        not stage_results[(16, "continuous")]["queue_sustained_growth"] and
        not stage_results[(16, "simulated_load")]["queue_sustained_growth"])

    lower = [row for row in comparison if row["node_count"] < 16]
    lower_latency_gain = None
    if lower and sixteen:
        best_lower = min(row["load_latency_p50_ms"] for row in lower)
        lower_latency_gain = sixteen["load_latency_p50_ms"] - best_lower
    higher = [row for row in comparison if row["node_count"] > 16]
    higher_reduces_loss = bool(sixteen and higher and any(
        row["max_tested_lossless_pause_ms"] > sixteen["max_tested_lossless_pause_ms"]
        or sum(row[f"pause_{pause}_frame_gaps"] for pause in (20, 50, 100, 250)) <
        sum(sixteen[f"pause_{pause}_frame_gaps"] for pause in (20, 50, 100, 250))
        for row in higher
    ))
    higher_old_age = bool(sixteen and higher and any(
        (row[f"pause_{pause}_first_age_max_ms"] or 0) >
        (sixteen[f"pause_{pause}_first_age_max_ms"] or 0) + measurement_floor_ms
        or (row[f"pause_{pause}_queue_max"] or 0) > (sixteen[f"pause_{pause}_queue_max"] or 0)
        for row in higher for pause in (20, 50, 100, 250)
    ))

    conclusions = {
        "protocol_valid": valid_protocol,
        "protocol_failures": failures,
        "measurement_floor_ms": measurement_floor_ms,
        "normal_latency_p50_spread_ms": normal_latency_spread,
        "normal_latency_max_10s_median_range_ms": temporal_range,
        "camera_latency_stable_for_node_ranking": camera_latency_stable,
        "sdk_host_to_app_p50_spread_ms": host_spread,
        "all_configs_normal_queue_empty_and_clean": all_normal_empty,
        "no_normal_latency_reason_to_adjust_nodes": no_latency_reason,
        "A_16_nodes_enough_current_load": sixteen_enough,
        "B_best_lower_node_latency_gain_ms": lower_latency_gain,
        "B_lower_nodes_materially_reduce_latency": lower_latency_evidence(
            lower_latency_gain, measurement_floor_ms, temporal_range),
        "C_higher_nodes_reduce_pause_loss": higher_reduces_loss,
        "C_higher_nodes_accumulate_older_frames": higher_old_age,
    }

    details = {
        "configuration": config,
        "clock_calibration": calibrations,
        "stage_summary": [
            {"node_count": nodes, **result} for (nodes, _), result in stage_results.items()
        ],
        "pause_repetitions": pause_results,
        "configuration_comparison": comparison,
        "conclusions": conclusions,
    }
    (directory / "summary.json").write_text(
        json.dumps(details, ensure_ascii=False, indent=2, allow_nan=False) + "\n")

    report = [
        "# MVS SDK 缓冲节点对比报告",
        "",
        "## 测试配置",
        "",
        f"- 相机：{config['model']}，SN {config['serial']}。",
        f"- 固定配置：{config['actual_width']}×{config['actual_height']}，Offset "
        f"({config['actual_offset_x']}, {config['actual_offset_y']})，BayerRG8 "
        f"(0x{int(config['actual_pixel_format']):08x})，{float(config['actual_frame_rate']):.3f} FPS，"
        f"曝光 {float(config['actual_exposure_us']):.3f} µs，增益 {float(config['actual_gain_db']):.3f} dB。",
        f"- 取帧策略：OneByOne；每组连续阶段 {config['continuous_seconds']} s，负载阶段 "
        f"{config['load_seconds']} s。",
        "- 负载性质：模拟负载。同步执行与现有相机节点一致的 BayerRG8→BGR8 转换和一次消息缓冲拷贝；"
        "未运行 YOLO 推理、DDS 发布或显示。",
        "- SDK 队列深度定义：每次 GetImageBuffer 成功后立刻调用 GetValidImageNum 得到的剩余有效图像数，"
        "不含当前借出的这一帧。暂停期间未调用 ClearImageBuffer。",
        "- 延迟定义：相机帧时间戳映射到 CLOCK_MONOTONIC 后至 GetImageBuffer 返回的时间，包含相机时间戳位置、"
        "USB 传输、SDK 缓冲和主机调度，不能称为纯 USB 时间。每组前后各做 20 次 DeviceTimestampLatch，"
        "选前后最小往返样本拟合时间比例及偏移；表中误差估计为全部锁存残差最大值加最佳往返时间的一半，"
        "不包含未观测到的非线性时钟漂移。"
        "SDK 主机时间戳原值保存在逐帧 CSV，但其精度仅为毫秒，不解释小于 1 ms 的差异。",
        "- 时钟稳定性检查：逐阶段按 10 秒分箱检查延迟中位数变化。该变化是稳定性诊断，"
        "不是严格的时钟误差界限；前后锁存的残差不能约束阶段内未观测到的非线性漂移。"
        "本轮单次顺序测试中，相机到应用的绝对延迟估计不宜直接用来给节点数排名。",
        "- SDK 主机标记至应用返回：app_realtime_ns − sdk_host_timestamp_raw × 10^6；"
        "仅在 SDK 返回正的主机毫秒时间戳时计算。这是辅助指标，不是完整的相机到应用延迟，"
        "也不能分离纯队列等待或 USB 耗时。",
        "- 暂停恢复以暂停前最近 100 个队列为空的帧的延迟 p95，加 max(1 ms, 锁存误差) 为阈值；"
        "分别记录首个正常帧、队列首次为空、连续 5 帧正常的确认时间。后者包含等待 5 帧的时间。",
        "- SDK USB 计数在 StartGrabbing 后按本次取流会话重新计数；旧程序在 StartGrabbing 前读到的值"
        "属于前一会话，不能相减。仅在 after 与全部原始行数（含预热）加缺帧数一致时报告本会话计数。"
        "USB 错误帧计数仅有整组统计，不能分配到各阶段。",
        "- 理论内存按 SDK PayloadSize × 节点数计算，不含 SDK 元数据、对齐和转换缓冲；RSS 为进程实测。",
        "- RSS 包含采样记录本身与分配器保留的内存，须结合各配置取流前/后值阅读，不能把全部差值归因于图像节点。",
        "- 每帧成功取出后释放缓冲，模拟处理同步完成后才释放；无异步悬空访问。计时循环不显示图像、"
        "打印或写盘，原始逐帧数据在各配置停止取流后保存。阶段初始化没有主动清队列；预热单独标为 excluded。",
        "- 测试前相机设置已恢复并通过读回验证。" if valid_protocol else
        "- 设置恢复状态详见 restoration_status.txt。",
        "",
        "## 配置对比",
        "",
        "| 节点 | FPS / MiB/s（连续｜负载） | 队列 p50,p95,max（连续｜负载） | "
        "队列首尾增长（连续｜负载） | 延迟估计 p50,p95,p99,max ms（连续｜负载，勿排名） | "
        "缺帧,重复,超时,错误（连续｜负载） | 无丢帧暂停 | 理论缓存 MiB | RSS 峰值 MiB |",
        "|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for row in comparison:
        queue_text = (
            f"{fmt(row['continuous_queue_p50'], 1)},{fmt(row['continuous_queue_p95'], 1)},"
            f"{fmt(row['continuous_queue_max'], 0)} / {fmt(row['load_queue_p50'], 1)},"
            f"{fmt(row['load_queue_p95'], 1)},{fmt(row['load_queue_max'], 0)}"
        )
        latency_text = (
            f"{fmt(row['continuous_latency_p50_ms'])},{fmt(row['continuous_latency_p95_ms'])},"
            f"{fmt(row['continuous_latency_p99_ms'])},{fmt(row['continuous_latency_max_ms'])} / "
            f"{fmt(row['load_latency_p50_ms'])},{fmt(row['load_latency_p95_ms'])},"
            f"{fmt(row['load_latency_p99_ms'])},{fmt(row['load_latency_max_ms'])}"
        )
        lossless = row["max_tested_lossless_pause_ms"]
        lossless_text = f"≥{lossless} ms" if lossless == 250 else (
            f"{lossless} ms" if lossless else "<20 ms")
        report.append(
            f"| {row['node_count']} | {fmt(row['continuous_fps'])} / "
            f"{fmt(row['continuous_throughput_mib_s'])} ｜ {fmt(row['load_fps'])} / "
            f"{fmt(row['load_throughput_mib_s'])} | {queue_text} | "
            f"{fmt(row['continuous_queue_growth'], 1)} ｜ {fmt(row['load_queue_growth'], 1)} | "
            f"{latency_text} | {row['continuous_frame_gaps']},"
            f"{row['continuous_duplicate_or_reordered']},{row['continuous_timeouts']},"
            f"{row['continuous_errors']} ｜ {row['load_frame_gaps']},"
            f"{row['load_duplicate_or_reordered']},{row['load_timeouts']},{row['load_errors']} | "
            f"{lossless_text} | "
            f"{mib(row['theoretical_payload_memory_bytes'])} | {mib(row['actual_rss_max_bytes'])} |"
        )

    report += ["", "## 时钟稳定性及 SDK 主机辅助指标", "",
        "| 节点 | 锁存残差+半 RTT ms | 10 秒中位数极差 ms（连续｜负载） | SDK 主机至应用 p50,p95,p99,max ms（连续｜负载） | USB 会话接收/错误帧 |",
        "|---:|---:|---:|---:|---:|",
    ]
    for row in comparison:
        host = [",".join(fmt(row[f"{phase}_sdk_host_to_app_ms_{stat}"])
                         for stat in ("p50", "p95", "p99", "max"))
                for phase in ("continuous", "load")]
        report.append(
            f"| {row['node_count']} | {fmt(row['clock_calibration_error_ms'])} | "
            f"{fmt(row['continuous_latency_10s_median_range_ms'])} ｜ "
            f"{fmt(row['load_latency_10s_median_range_ms'])} | {host[0]} ｜ {host[1]} | "
            f"{row.get('usb_received_frames', '-')} / {row.get('usb_error_frames', '-')} |")

    report += ["", "## 暂停恢复", "",
        "第一帧年龄仍使用相机时钟拟合估计，含前述漂移限制；短时间恢复用局部基线降低影响。"
        "排空指高速取帧下第一次观测到剩余队列为 0，不代表业务处理也能以该速度排空。"
        "暂停测试使用快速取帧恢复，未叠加模拟处理负载。", "",
        "| 节点 | 暂停 | 3 次均无丢帧 | 缺帧合计 | 第一帧年龄最大 ms | 队列最大 | 排空最长 ms | 连续 5 帧正常确认最长 ms |",
        "|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for row in comparison:
        for pause_ms in (20, 50, 100, 250):
            report.append(
                f"| {row['node_count']} | {pause_ms} ms | "
                f"{'是' if row[f'pause_{pause_ms}_lossless'] else '否'} | "
                f"{row[f'pause_{pause_ms}_frame_gaps']} | "
                f"{fmt(row[f'pause_{pause_ms}_first_age_max_ms'])} | "
                f"{fmt(row[f'pause_{pause_ms}_queue_max'], 0)} | "
                f"{fmt(row[f'pause_{pause_ms}_drain_max_ms'])} | "
                f"{fmt(row[f'pause_{pause_ms}_latency_recovery_max_ms'])} |"
            )

    report += ["", "## 结论", ""]
    if not valid_protocol:
        report.append("本轮不满足正式测试协议，以下结论无效：" + "；".join(failures) + "。")
    if no_latency_reason and valid_protocol:
        if camera_latency_stable:
            report.append("正常运行时所有配置都没有积压，且延迟差异处于测量分辨能力内：无需因正常延迟调整节点数。")
        else:
            report.append(
                "无需因正常延迟调整节点数。正常运行时所有配置均无 SDK 队列积压，SDK 主机至应用 "
                f"p50 的跨阶段极差仅 {fmt(host_spread)} ms，低于毫秒时间戳的分辨能力。"
                f"但相机到应用延迟估计的单阶段 10 秒中位数极差可达 {fmt(temporal_range)} ms，"
                "明显超过边界锁存残差；因此不能声称所有真实端到端延迟都相同，"
                "也不能把顺序测试的数毫秒差异归因于节点数量。")
    if sixteen and valid_protocol:
        report.append("")
        report.append(
            "A. 当前模拟负载下，16 节点" + ("已有足够余量" if sixteen_enough else "尚不能证明有足够余量") +
            f"；连续/负载队列 p95 为 {fmt(sixteen['continuous_queue_p95'], 1)} / "
            f"{fmt(sixteen['load_queue_p95'], 1)}，实测无丢帧暂停上限为 "
            f"{sixteen['max_tested_lossless_pause_ms']} ms（仅为已测试档位，非精确极限）。"
            "这验证了相机转换/拷贝模拟负载，尚未验证完整装甲板识别业务的最坏处理时间。"
        )
        report.append("")
        report.append(
            "B. 减少节点" + ("观测到超过分辨下限且通过稳定性检查的延迟下降；仍需重复交错测试确认因果"
                if conclusions["B_lower_nodes_materially_reduce_latency"] else
                "没有证明能降低正常延迟；已确定的是降低内存和抗暂停能力") +
            f"。负载 p50 最佳表观改善 {fmt(lower_latency_gain)} ms，"
            f"分辨下限 {measurement_floor_ms:.3f} ms；"
            "判定同时检查阶段内变化，不能仅凭表观改善超过 1 ms 就宣称收益。"
        )
        report.append("")
        report.append(
            "C. 增加到 32/64 节点" + ("减少了暂停相关丢帧" if higher_reduces_loss else "未观察到进一步减少丢帧") +
            "，并且" + ("产生了更多旧帧积压" if higher_old_age else "未观察到超出误差的更多旧帧积压") + "。"
        )
        if (sixteen_enough and sixteen["max_tested_lossless_pause_ms"] == 100
                and by_node.get(32, {}).get("max_tested_lossless_pause_ms") == 250
                and by_node.get(64, {}).get("max_tested_lossless_pause_ms") == 250):
            report.append("")
            report.append(
                "建议保留默认 16 节点，前提是业务可接受本轮验证的 100 ms 暂停容忍度。"
                "若要求 250 ms 暂停后保留全部帧，选 32 节点；本轮 64 节点没有额外的抗暂停收益，"
                "理论图像内存约为 32 节点的两倍。更长暂停和真实业务负载尚未验证。")
    report += [
        "",
        "完整逐帧数据见 `per_frame.csv`，逐阶段统计见 `stage_summary.csv`，配置主表见 "
        "`configuration_comparison.csv`，各次暂停见 `pause_repetitions.csv`。",
    ]
    (directory / "report.md").write_text("\n".join(report) + "\n")
    print(json.dumps({
        "run_dir": str(directory),
        "frames": len(formal_frames),
        "protocol_valid": valid_protocol,
        "report": str(directory / "report.md"),
        "conclusions": conclusions,
    }, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
