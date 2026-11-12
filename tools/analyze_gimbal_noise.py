#!/usr/bin/env python3
"""Audit recorded command discontinuities and feedback without opening ROS or USB."""

import argparse
from bisect import bisect_right
import json
import math
from pathlib import Path
import statistics


def read_rows(path):
    return json.loads(path.read_text())["rows"]


def zero_command_audit(rows):
    feedback = rows["feedback"]
    times = [row[0] for row in feedback]
    start = rows["forwarded"][0][0]
    candidates = []
    for preview in rows["preview"]:
        t, yaw, pitch, status = preview[:4]
        if t < start or (yaw, pitch) != (0.0, 0.0) or status != "candidate_unverified":
            continue
        index = bisect_right(times, t) - 1
        if index < 0:
            continue
        last = feedback[index]
        yaw_error = abs(math.remainder(-last[3], 2 * math.pi))
        pitch_error = abs(last[4])
        candidates.append({
            "t_s": t,
            "yaw_error_deg": math.degrees(yaw_error),
            "pitch_error_deg": math.degrees(pitch_error),
            "feedback_age_ms": (t - last[0]) * 1000,
            "passes_angle_gate": yaw_error <= 0.6 and pitch_error <= 0.6,
        })
    return {
        "stale_status_zero_candidates": len(candidates),
        "passing_angle_gate": sum(row["passes_angle_gate"] for row in candidates),
        "forwarded_zero_commands": sum(
            row[1] == 0 and row[2] == 0 for row in rows["forwarded"]
        ),
        "candidates": candidates,
    }


def pitch_window(rows, start, end):
    samples = [row for row in rows["feedback"] if start <= row[0] < end]
    values = [math.degrees(row[4]) for row in samples]
    steps = [b - a for a, b in zip(values, values[1:])]
    return {
        "window_s": [start, end],
        "samples": len(values),
        "min_deg": min(values),
        "max_deg": max(values),
        "std_deg": statistics.pstdev(values),
        "median_abs_step_deg": statistics.median(abs(step) for step in steps),
        "reversals_above_0_2_deg": sum(
            a * b < 0 and abs(a) >= 0.2 and abs(b) >= 0.2
            for a, b in zip(steps, steps[1:])
        ),
        "mcu_interval_ms_range": [
            min(b[2] - a[2] for a, b in zip(samples, samples[1:])),
            max(b[2] - a[2] for a, b in zip(samples, samples[1:])),
        ],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("previous", type=Path)
    parser.add_argument("incident", type=Path)
    parser.add_argument("--output-prefix", type=Path, required=True)
    args = parser.parse_args()
    previous = read_rows(args.previous)
    incident = read_rows(args.incident)
    last_command = incident["forwarded"][-1]
    mode_changes = [
        {"t_s": b[0], "from": a[5], "to": b[5]}
        for a, b in zip(incident["feedback"], incident["feedback"][1:])
        if a[5] != b[5]
    ]
    result = {
        "previous_file": str(args.previous),
        "incident_file": str(args.incident),
        "previous": zero_command_audit(previous),
        "incident": zero_command_audit(incident),
        "last_forwarded_command": last_command,
        "command_activity_after_last_command": [
            row for row in incident["command_active"] if row[0] >= last_command[0]
        ],
        "mode_changes": mode_changes,
        "post_command_pitch": pitch_window(incident, 13, 17),
        "disabled_pitch": pitch_window(incident, 19.5, 29.9),
        "limits": "ROS forwarding and 20 Hz feedback; no MCU setpoint, current or audio trace.",
    }
    args.output_prefix.parent.mkdir(parents=True, exist_ok=True)
    args.output_prefix.with_suffix(".json").write_text(json.dumps(result, indent=2) + "\n")

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(2, 1, figsize=(11, 6), constrained_layout=True)
    feedback = incident["feedback"]
    times = [r[0] for r in feedback]
    pitch = [math.degrees(r[4]) for r in feedback]
    line_times, line_pitch = [], []
    for index, (t, value) in enumerate(zip(times, pitch)):
        if index and t - times[index - 1] > 0.15:
            line_times.append(math.nan)
            line_pitch.append(math.nan)
        line_times.append(t)
        line_pitch.append(value)
    axes[0].plot(line_times, line_pitch, ".-", color="#207561", linewidth=0.8,
                 markersize=3, label="Pitch feedback")
    axes[0].plot([r[0] for r in incident["forwarded"]],
                 [math.degrees(r[2]) for r in incident["forwarded"]],
                 ".", color="#b45a28", markersize=4, label="Forwarded ROS pitch target")
    axes[0].axvline(last_command[0], color="#b45a28", linestyle="--",
                    label="Last forwarded command")
    for change in mode_changes:
        if change["to"] == 0:
            axes[0].axvline(change["t_s"], color="#555555", linestyle="--",
                            label="Feedback mode becomes 0")
    axes[0].set(xlim=(10.5, 20.5), ylim=(-2, 8), ylabel="Pitch (degrees)",
                title="Pitch continues to vary after ROS commands stop")
    axes[0].legend(loc="upper right", fontsize=8)
    axes[1].plot(times, pitch, ".-", color="#207561", linewidth=1, markersize=4)
    axes[1].axhline(0, color="#888888", linewidth=0.7)
    axes[1].set(xlim=(13, 14), ylim=(-1.3, 1.3), ylabel="Pitch (degrees)",
                xlabel="Recorded arrival time from test start (seconds)",
                title="Detail: no new forwarded commands; feedback mode remains 6")
    for axis in axes:
        axis.grid(alpha=0.2)
    fig.savefig(args.output_prefix.with_suffix(".png"), dpi=160)
    plt.close(fig)
    print(json.dumps({
        "previous_zero_commands": result["previous"]["forwarded_zero_commands"],
        "incident_zero_commands": result["incident"]["forwarded_zero_commands"],
        "post_command_pitch": result["post_command_pitch"],
        "disabled_pitch": result["disabled_pitch"],
    }, indent=2))


if __name__ == "__main__":
    main()
