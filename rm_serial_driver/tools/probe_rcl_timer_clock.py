#!/usr/bin/env python3
"""Compare rclpy steady timers with Linux monotonic and realtime clocks."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import threading
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node


class Probe(Node):
    def __init__(self):
        super().__init__("rcl_timer_clock_probe", enable_rosout=False,
                         start_parameter_services=False)
        self.steady = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer_rows = []
        self.create_timer(0.01, self.on_timer, clock=self.steady)

    def on_timer(self):
        self.timer_rows.append([
            time.monotonic_ns(),
            time.clock_gettime_ns(time.CLOCK_MONOTONIC_RAW),
            time.time_ns(),
            self.steady.now().nanoseconds,
        ])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=120.0)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    rclpy.init()
    node = Probe()
    heartbeat_rows = []
    stop = threading.Event()

    def heartbeat():
        while not stop.wait(0.1):
            heartbeat_rows.append([
                time.monotonic_ns(),
                time.clock_gettime_ns(time.CLOCK_MONOTONIC_RAW),
                time.time_ns(),
                node.steady.now().nanoseconds,
            ])

    worker = threading.Thread(target=heartbeat, daemon=True)
    worker.start()
    started = time.monotonic()
    try:
        while time.monotonic() - started < args.duration:
            rclpy.spin_once(node, timeout_sec=0.02)
    finally:
        stop.set()
        worker.join(timeout=1)
        node.destroy_node()
        rclpy.shutdown()

    def gaps(rows, column):
        return [(b[column] - a[column]) / 1e6 for a, b in zip(rows, rows[1:])]

    def realtime_steps(rows):
        events = []
        for before, after in zip(rows, rows[1:]):
            monotonic_ms = (after[0] - before[0]) / 1e6
            realtime_ms = (after[2] - before[2]) / 1e6
            difference_ms = realtime_ms - monotonic_ms
            if abs(difference_ms) > 200:
                events.append({
                    "at_s": (after[0] - rows[0][0]) / 1e9,
                    "realtime_minus_monotonic_ms": difference_ms,
                })
        return events

    timer_monotonic_gaps = gaps(node.timer_rows, 0)
    timer_steady_gaps = gaps(node.timer_rows, 3)
    heartbeat_monotonic_gaps = gaps(heartbeat_rows, 0)
    result = {
        "requested_duration_s": args.duration,
        "timer_callbacks": len(node.timer_rows),
        "heartbeat_samples": len(heartbeat_rows),
        "timer_monotonic_max_gap_ms": max(timer_monotonic_gaps, default=0),
        "timer_steady_max_gap_ms": max(timer_steady_gaps, default=0),
        "heartbeat_monotonic_max_gap_ms": max(heartbeat_monotonic_gaps, default=0),
        "timer_gaps_over_100ms": [
            {"at_s": (node.timer_rows[index + 1][0] - node.timer_rows[0][0]) / 1e9,
             "gap_ms": value}
            for index, value in enumerate(timer_monotonic_gaps) if value > 100
        ],
        "heartbeat_realtime_steps": realtime_steps(heartbeat_rows),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps({"summary": result,
                                       "timer": node.timer_rows,
                                       "heartbeat": heartbeat_rows}, indent=2) + "\n")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
