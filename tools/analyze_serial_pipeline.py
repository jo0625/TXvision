#!/usr/bin/env python3
"""Compare timestamps from serial_pipeline_probe bridge and observer processes."""
import argparse
import json
from pathlib import Path

import numpy as np


def describe(values):
    values = np.asarray(values, dtype=float)
    if not len(values):
        return dict(count=0)
    return dict(count=len(values), mean=float(np.mean(values)),
                median=float(np.median(values)), p95=float(np.percentile(values, 95)),
                max=float(np.max(values)))


def analyze(bridge_path, observer_path):
    bridge = json.loads(bridge_path.read_text())
    observer = json.loads(observer_path.read_text())
    br, ob = bridge["rows"], observer["rows"]
    has_feedback = bool(br.get("published"))
    start = (br["published"][0][0] if has_feedback else bridge["started_ns"]) + 1_000_000_000
    end = br["published"][-1][0] if has_feedback else br["host_clock"][-1][0]

    def within(rows):
        return [row for row in rows if start <= row[0] <= end]

    stages = {
        "windows_frame": (within(br.get("windows_frame", [])), 2, 1000),
        "wsl_decode": (within(br.get("windows_frame", [])), 0, 1e6),
        "ros_publish": (within(br.get("published", [])), 0, 1e6),
        "ros_observer": (within(ob.get("feedback", [])), 0, 1e6),
        "windows_clock": (within(br.get("windows_clock", [])), 1, 1),
        "bridge_heartbeat": (within(br.get("host_clock", [])), 0, 1e6),
        "observer_heartbeat": (within(ob.get("host_clock", [])), 0, 1e6),
        "detector": (within(ob.get("detector", [])), 0, 1e6),
    }
    report = dict(bridge_source=str(bridge_path), observer_source=str(observer_path),
                  status="feedback_recorded" if has_feedback else "no_mcu_feedback",
                  start_monotonic_ns=start, end_monotonic_ns=end,
                  analysis_duration_s=(end - start) / 1e9,
                  ignored_command_count=len(br.get("ignored_command", [])), stages={})
    for name, (rows, col, divisor) in stages.items():
        deltas = np.diff([row[col] for row in rows]) / divisor
        summary = describe(deltas)
        summary["gaps_over_150ms"] = [dict(
            at_s=(rows[i + 1][0] - start) / 1e9, gap_ms=float(deltas[i]))
            for i in np.flatnonzero(deltas > 150)]
        report["stages"][name] = summary

    windows = {row[3]: row for row in within(br.get("windows_frame", []))}
    published = {row[2]: row for row in within(br.get("published", []))}
    observed = {row[1]: row for row in within(ob.get("feedback", []))}
    common = sorted(windows.keys() & published.keys() & observed.keys())
    report["matched_frames"] = len(common)
    report["stage_duration_ms"] = {
        "windows_read": describe([(row[2] - row[1]) / 1000 for row in within(br.get("windows_chunk", []))]),
        "wsl_decode_to_publish_begin": describe([(published[key][0] - windows[key][0]) / 1e6 for key in common]),
        "ros_publish_call": describe([(row[1] - row[0]) / 1e6 for row in within(br.get("published", []))]),
        "ros_publish_begin_to_observer": describe([(observed[key][0] - published[key][0]) / 1e6 for key in common]),
    }
    report["active_slow_calls"] = [dict(at_s=(row[0] - start) / 1e9,
                                         duration_ms=(row[1] - row[0]) / 1e6, function=row[2])
                                   for row in within(br.get("slow_call", []))]
    report["read_timer_gaps"] = [dict(at_s=(row[0] - start) / 1e9,
                                       gap_ms=(row[1] - row[0]) / 1e6)
                                 for row in within(br.get("read_timer_gap", []))]
    for key, data in (("bridge", br), ("observer", ob)):
        clock = within(data.get("host_clock", []))
        report[key + "_realtime_steps"] = []
        for before, after in zip(clock, clock[1:]):
            monotonic_ms = (after[0] - before[0]) / 1e6
            realtime_ms = (after[2] - before[2]) / 1e6
            if abs(realtime_ms - monotonic_ms) > 200:
                report[key + "_realtime_steps"].append(dict(
                    at_s=(after[0] - start) / 1e9, monotonic_interval_ms=monotonic_ms,
                    realtime_interval_ms=realtime_ms,
                    realtime_minus_monotonic_ms=realtime_ms - monotonic_ms))
        if len(clock) >= 2:
            report[key + "_clock_span_s"] = dict(
                monotonic=(clock[-1][0] - clock[0][0]) / 1e9,
                raw=(clock[-1][1] - clock[0][1]) / 1e9,
                realtime=(clock[-1][2] - clock[0][2]) / 1e9)
    report["callback_ms_including_startup"] = bridge.get("callback_ms", {})
    report["clock_note"] = (
        "Windows timestamps and Linux timestamps have different epochs. "
        "No raw subtraction is used to claim absolute Windows-to-WSL latency.")
    return report


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bridge", type=Path)
    parser.add_argument("observer", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    report = analyze(args.bridge, args.observer)
    args.output.write_text(json.dumps(report, indent=2))
    print(json.dumps({key: value for key, value in report.items()
                      if key not in ("read_timer_gaps", "active_slow_calls")}, indent=2))
