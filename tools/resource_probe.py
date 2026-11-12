#!/usr/bin/env python3
"""Sample system and rm_vision process resources without changing ROS state."""

import argparse
import json
import subprocess
import time
from pathlib import Path

import psutil


PROCESS_MARKERS = {
    "camera": "hik_camera_node",
    "detector": "yolo_armor_detector_node.py",
    "tracker": "armor_tracker_node",
    "rviz": "rviz2",
}


def gpu_sample():
    command = [
        "nvidia-smi",
        "--query-gpu=utilization.gpu,memory.used,power.draw,temperature.gpu",
        "--format=csv,noheader,nounits",
    ]
    try:
        values = subprocess.check_output(command, text=True, timeout=2).strip().split(", ")
        return {
            "util_percent": float(values[0]),
            "memory_mib": float(values[1]),
            "power_w": float(values[2]),
            "temperature_c": float(values[3]),
        }
    except (OSError, subprocess.SubprocessError, ValueError, IndexError):
        return None


def matching_processes():
    matches = {}
    scores = {}
    for process in psutil.process_iter(["pid", "cmdline"]):
        try:
            command_line = process.info["cmdline"] or []
            command = " ".join(command_line)
            executable = Path(command_line[0]).name if command_line else ""
            for role, marker in PROCESS_MARKERS.items():
                if marker not in command:
                    continue
                score = 2 if executable == marker else 1
                if score > scores.get(role, 0):
                    process.cpu_percent(None)
                    matches[role] = process
                    scores[role] = score
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            continue
    return matches


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    parser.add_argument("--duration", type=float, required=True)
    parser.add_argument("--interval", type=float, default=1.0)
    args = parser.parse_args()

    processes = matching_processes()
    psutil.cpu_percent(None)
    samples = []
    deadline = time.monotonic() + args.duration
    while time.monotonic() < deadline:
        time.sleep(args.interval)
        memory = psutil.virtual_memory()
        process_data = {}
        for role, process in list(processes.items()):
            try:
                info = process.memory_info()
                process_data[role] = {
                    "pid": process.pid,
                    "cpu_percent": process.cpu_percent(None),
                    "rss_mib": info.rss / 1024 / 1024,
                }
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                process_data[role] = None
        samples.append(
            {
                "wall_ns": time.time_ns(),
                "system_cpu_percent": psutil.cpu_percent(None),
                "system_memory_used_mib": memory.used / 1024 / 1024,
                "system_memory_available_mib": memory.available / 1024 / 1024,
                "processes": process_data,
                "gpu": gpu_sample(),
            }
        )

    Path(args.output).write_text(json.dumps(samples, indent=2) + "\n")


if __name__ == "__main__":
    main()
