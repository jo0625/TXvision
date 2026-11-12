#!/usr/bin/env python3
"""Read-only acceptance check for the live auto-aim pipeline."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import time

import rclpy
from auto_aim_interfaces.msg import AimCommand, Armors, GimbalFeedback, Target
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool, String
from tf2_msgs.msg import TFMessage


def message_rate(rows):
    if len(rows) < 2:
        return 0.0
    return (len(rows) - 1) / (rows[-1][0] - rows[0][0])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--require-target", action="store_true")
    parser.add_argument("--require-readonly", action="store_true")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if not math.isfinite(args.duration) or args.duration <= 0:
        parser.error("--duration must be positive")

    rclpy.init()
    node = rclpy.create_node("complete_system_test")
    started = time.monotonic()
    rows = {key: [] for key in (
        "armors", "target", "aim", "preview", "feedback", "tf")}
    states = {}
    statuses = []

    def elapsed():
        return time.monotonic() - started

    subscriptions = [
        node.create_subscription(
            Armors, "/detector/armors",
            lambda msg: rows["armors"].append((elapsed(), len(msg.armors))),
            qos_profile_sensor_data),
        node.create_subscription(
            Target, "/tracker/target",
            lambda msg: rows["target"].append((elapsed(), bool(msg.tracking))),
            qos_profile_sensor_data),
        node.create_subscription(
            AimCommand, "/aimer/aim_command",
            lambda msg: rows["aim"].append(
                (elapsed(), bool(msg.valid), msg.yaw, msg.pitch)),
            qos_profile_sensor_data),
        node.create_subscription(
            AimCommand, "/controller_command_preview/command",
            lambda msg: rows["preview"].append(
                (elapsed(), bool(msg.valid), msg.yaw, msg.pitch)),
            qos_profile_sensor_data),
        node.create_subscription(
            GimbalFeedback, "/serial_bridge/feedback",
            lambda msg: rows["feedback"].append((
                elapsed(), msg.sequence, msg.mcu_timestamp_ms, msg.mode,
                msg.yaw, msg.pitch)),
            qos_profile_sensor_data),
        node.create_subscription(
            TFMessage, "/tf",
            lambda msg: rows["tf"].append((elapsed(), len(msg.transforms))),
            qos_profile_sensor_data),
        node.create_subscription(
            String, "/controller_command_preview/status",
            lambda msg: statuses.append((elapsed(), msg.data)), 10),
    ]
    for key in ("connected", "feedback_alive", "command_active"):
        subscriptions.append(node.create_subscription(
            Bool, "/serial_bridge/" + key,
            lambda msg, key=key: states.update({key: bool(msg.data)}), 1))

    deadline = started + args.duration
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    counts = {key: len(value) for key, value in rows.items()}
    rates = {key: message_rate(value) for key, value in rows.items()}
    nonempty = sum(item[1] > 0 for item in rows["armors"])
    tracking = sum(item[1] for item in rows["target"])
    valid_aim = sum(item[1] for item in rows["aim"])
    valid_preview = sum(item[1] for item in rows["preview"])
    result = {
        "duration_s": elapsed(),
        "counts": counts,
        "rates_hz": rates,
        "nonempty_armor_messages": nonempty,
        "tracking_true_messages": tracking,
        "valid_aim_messages": valid_aim,
        "valid_preview_messages": valid_preview,
        "serial_states": states,
        "feedback_first": rows["feedback"][0][1:] if rows["feedback"] else None,
        "feedback_last": rows["feedback"][-1][1:] if rows["feedback"] else None,
        "preview_last": rows["preview"][-1][1:] if rows["preview"] else None,
        "latest_preview_status": statuses[-1][1] if statuses else None,
    }
    if rows["preview"] and rows["feedback"]:
        result["preview_feedback_error_rad"] = {
            "yaw": math.remainder(
                rows["preview"][-1][2] - rows["feedback"][-1][4],
                2 * math.pi),
            "pitch": rows["preview"][-1][3] - rows["feedback"][-1][5],
        }

    failures = []
    minimum_samples = max(1, int(args.duration * 10))
    if counts["armors"] < minimum_samples or counts["target"] < minimum_samples:
        failures.append("vision topics below 10 Hz")
    if counts["feedback"] < minimum_samples or counts["tf"] < minimum_samples:
        failures.append("feedback or TF below 10 Hz")
    if states.get("connected") is not True:
        failures.append("serial bridge is not connected")
    if states.get("feedback_alive") is not True:
        failures.append("feedback is not alive")
    if args.require_readonly and states.get("command_active") is not False:
        failures.append("command channel is not confirmed inactive")
    if args.require_target:
        required = max(1, int(counts["armors"] * 0.8))
        if nonempty < required:
            failures.append("armor detection is not stable")
        if tracking < max(1, int(counts["target"] * 0.8)):
            failures.append("tracker is not stable")
        if valid_aim < max(1, int(counts["aim"] * 0.8)):
            failures.append("aimer output is not stable")
        if valid_preview < max(1, int(counts["preview"] * 0.8)):
            failures.append("controller preview is not stable")

    result["passed"] = not failures
    result["failures"] = failures
    payload = json.dumps(result, indent=2) + "\n"
    print(payload, end="")
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(payload)

    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(0 if not failures else 2)


if __name__ == "__main__":
    main()
