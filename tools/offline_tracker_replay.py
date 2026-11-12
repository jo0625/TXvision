#!/usr/bin/env python3
"""Replay recorded tracker inputs in a separate ROS domain, without actuators."""

import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--domain-id", type=int, default=194)
    args = parser.parse_args()
    if args.domain_id == int(os.environ.get("ROS_DOMAIN_ID", "0")):
        parser.error("Replay must use a different ROS domain from the live system")
    os.environ["ROS_DOMAIN_ID"] = str(args.domain_id)

    import rclpy
    from auto_aim_interfaces.msg import Armors, Target, TrackerInfo
    from rcl_interfaces.msg import Log
    from rclpy.qos import qos_profile_sensor_data
    from rosidl_runtime_py.convert import message_to_ordereddict

    root = Path(__file__).resolve().parents[1]
    args.output.parent.mkdir(parents=True, exist_ok=True)
    rows = {key: [] for key in ("detector", "target", "info", "logs")}
    rclpy.init()
    node = rclpy.create_node("offline_tracker_recorder")

    def record(key, message):
        if key == "logs" and message.name != "armor_tracker":
            return
        rows[key].append(message_to_ordereddict(message))

    for cls, topic, key in (
        (Armors, "/detector/armors", "detector"),
        (Target, "/tracker/target", "target"),
        (TrackerInfo, "/tracker/info", "info"),
        (Log, "/rosout", "logs"),
    ):
        node.create_subscription(
            cls, topic, lambda msg, key=key: record(key, msg), qos_profile_sensor_data
        )

    processes = []
    try:
        with args.output.with_suffix(".log").open("w") as log:
            tracker = subprocess.Popen(
                [str(root / "install/armor_tracker/lib/armor_tracker/armor_tracker_node"),
                 "--ros-args", "--params-file",
                 str(root / "auto_aim_bringup/params/tracker_params.yaml"),
                 "-p", "use_sim_time:=true"],
                stdout=log, stderr=subprocess.STDOUT,
            )
            processes.append(tracker)
            deadline = time.monotonic() + 10
            while node.count_publishers("/tracker/target") == 0:
                if tracker.poll() is not None or time.monotonic() > deadline:
                    raise RuntimeError("Offline tracker did not become ready")
                rclpy.spin_once(node, timeout_sec=0.1)
            player = subprocess.Popen(
                ["ros2", "bag", "play", str(args.bag.resolve()),
                 "--topics", "/detector/armors", "/tf", "/tf_static",
                 "--clock", "100", "--delay", "1", "--disable-keyboard-controls"],
                stdout=log, stderr=subprocess.STDOUT,
            )
            processes.append(player)
            deadline = time.monotonic() + 120
            while player.poll() is None:
                if time.monotonic() > deadline or tracker.poll() is not None:
                    raise RuntimeError("Offline replay timed out or tracker exited")
                rclpy.spin_once(node, timeout_sec=0.1)
            if player.returncode:
                raise RuntimeError(f"Bag playback failed: {player.returncode}")
            deadline = time.monotonic() + 0.5
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        for process in reversed(processes):
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
        node.destroy_node()
        rclpy.shutdown()

    payload = {"bag": str(args.bag), "domain_id": args.domain_id, "rows": rows}
    args.output.write_text(json.dumps(payload, indent=2) + "\n")
    print(json.dumps({
        "detector_frames": len(rows["detector"]),
        "target_frames": len(rows["target"]),
        "tracking_frames": sum(m["tracking"] for m in rows["target"]),
        "output": str(args.output),
    }), flush=True)


if __name__ == "__main__":
    main()
