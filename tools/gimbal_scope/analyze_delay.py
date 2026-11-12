#!/usr/bin/env python3
"""Estimate curve lag from a recorded target/feedback pair, excluding arrival gaps."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def interpolate(times, rows, column, minimum_gap=0.0):
    source_times = rows[:, 0]
    indices = np.clip(np.searchsorted(source_times, times, side="right") - 1, 0, len(rows) - 2)
    intervals = source_times[indices + 1] - source_times[indices]
    valid = ((times >= source_times[0]) & (times < source_times[-1]) &
             (intervals >= minimum_gap) & (intervals <= 0.15))
    angles = np.unwrap(rows[:, column])
    return np.interp(times, source_times, angles), valid


def estimate(feedback, commands, axis, start, end):
    command_col, feedback_col = (1, 3) if axis == "yaw" else (2, 4)
    times = np.arange(start, end, 0.01)
    lags = np.arange(0, 0.401, 0.005)
    actual, valid = interpolate(times, feedback, feedback_col, minimum_gap=0.015)
    candidates = []
    # Every lag uses the same samples, preventing gaps from biasing the best lag.
    for lag in lags:
        target, available = interpolate(times - lag, commands, command_col)
        candidates.append(target)
        valid &= available
    if valid.sum() < 30:
        return None
    errors = np.array([np.degrees(np.sqrt(np.mean((actual[valid] - target[valid]) ** 2)))
                       for target in candidates])
    best = int(np.argmin(errors))
    return dict(start_s=start, end_s=end, lag_ms=float(lags[best] * 1000),
                aligned_rmse_deg=float(errors[best]), zero_lag_rmse_deg=float(errors[0]),
                correlation=float(np.corrcoef(actual[valid], candidates[best][valid])[0, 1]),
                usable_grid_points=int(valid.sum()), grid_step_s=0.01,
                # These interpolated grid points are not independent measurements.
                usable_grid_duration_s=float(valid.sum() * 0.01))


def disconnected_line(rows, column):
    xs, ys = [], []
    for index, row in enumerate(rows):
        if index and rows[index, 0] - rows[index - 1, 0] > 0.15:
            xs.append(float("nan"))
            ys.append(float("nan"))
        xs.append(row[0])
        ys.append(float(np.degrees(row[column])))
    return xs, ys


def analyze(source, output_prefix):
    record = json.loads(source.read_text())
    feedback = np.array(record["rows"]["feedback"], dtype=float)
    commands = np.array(record["rows"]["forwarded"], dtype=float)
    if len(feedback) < 2 or len(commands) < 2:
        raise ValueError("At least two feedback and target samples are required")
    start = max(3.2, commands[0, 0])
    end = min(29.5, feedback[-1, 0], commands[-1, 0])
    result = dict(source=str(source), feedback_samples=len(feedback), target_samples=len(commands),
                  method="Minimize angle RMSE between feedback(t) and target(t-lag); common valid sample set",
                  time_base="Host monotonic receive time, not MCU command receipt time",
                  interpolation="Linear interpolation; 10 ms grid is not 10 ms measurement accuracy",
                  feedback_gap_limit_s=0.15, feedback_burst_min_interval_s=0.015,
                  lag_search_ms=[0, 400, 5], axes={}, arrival_gaps=[])
    intervals = np.diff(feedback[:, 0])
    for index in np.flatnonzero(intervals > 0.15):
        result["arrival_gaps"].append(dict(
            start_s=float(feedback[index, 0]), end_s=float(feedback[index + 1, 0]),
            host_gap_ms=float(intervals[index] * 1000),
            mcu_step_ms=float(feedback[index + 1, 2] - feedback[index, 2]),
            sequence_step=int(feedback[index + 1, 1] - feedback[index, 1])))
    result["burst_intervals_under_15ms"] = int(np.sum(intervals < 0.015))
    result["mcu_interval_median_ms"] = float(np.median(np.diff(feedback[:, 2])))
    for axis in ("yaw", "pitch"):
        windows = []
        for begin in np.arange(3.2, 29.0, 4):
            window = estimate(feedback, commands, axis, float(begin), min(float(begin + 4), 29.9))
            if window is not None and window["usable_grid_duration_s"] >= 0.8:
                windows.append(window)
        result["axes"][axis] = dict(overall=estimate(feedback, commands, axis, start, end), windows=windows)

    output_prefix.parent.mkdir(parents=True, exist_ok=True)
    output_prefix.with_suffix(".json").write_text(json.dumps(result, indent=2, ensure_ascii=False))
    fig, axes = plt.subplots(3, 2, figsize=(14, 9), constrained_layout=True)
    for col, axis in enumerate(("yaw", "pitch")):
        ci, fi = (1, 3) if axis == "yaw" else (2, 4)
        for row_index, bounds in enumerate(((3, 30), (23.2, 26.5), (23.2, 26.5))):
            panel = axes[row_index, col]
            tx, ty = disconnected_line(commands, ci)
            fx, fy = disconnected_line(feedback, fi)
            panel.plot(tx, ty, color="#bf6b27", linewidth=1.2, label="Forwarded target")
            if row_index == 2:
                estimate_window = next(w for w in result["axes"][axis]["windows"] if abs(w["start_s"] - 23.2) < 0.01)
                shift = estimate_window["lag_ms"] / 1000
                fx = np.asarray(fx) - shift
                label = f"Feedback shifted earlier by {shift * 1000:.0f} ms"
            else:
                label = "Received feedback"
            panel.plot(fx, fy, color="#117c73", linewidth=1.1, label=label)
            if row_index == 0:
                for gap in result["arrival_gaps"]:
                    panel.axvspan(gap["start_s"], gap["end_s"], color="#c04c54", alpha=0.12)
            panel.set(xlim=bounds, ylabel=f"{axis.capitalize()} (deg)", xlabel="Record time (s)")
            panel.grid(alpha=0.18)
            panel.legend(fontsize=8, loc="best")
    axes[0, 0].set_title("Full recording; shaded area = feedback arrival gap")
    axes[0, 1].set_title("Full recording")
    axes[1, 0].set_title("Detail before time alignment")
    axes[1, 1].set_title("Detail before time alignment")
    axes[2, 0].set_title("Same detail after time alignment")
    axes[2, 1].set_title("Same detail after time alignment")
    fig.suptitle("2026-09-23 live follow: target to feedback curve lag", fontsize=15)
    fig.savefig(output_prefix.with_suffix(".png"), dpi=150)
    plt.close(fig)
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("--output-prefix", type=Path, required=True)
    args = parser.parse_args()
    result = analyze(args.source, args.output_prefix)
    print(json.dumps(result, indent=2))
