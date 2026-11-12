#!/usr/bin/env python3
"""Read-only live timing probe; never subscribes to or sends actuator commands."""

import argparse
import json
from pathlib import Path
import signal
import subprocess
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from auto_aim_interfaces.msg import Armors, GimbalFeedback, Target
from sensor_msgs.msg import CameraInfo, Image


ROOT = Path(__file__).resolve().parents[1]


def stamp_ns(msg):
    return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec


def stats(values):
    if not values:
        return None
    a = np.asarray(values, dtype=float)
    return {"n": len(a), "mean": float(a.mean()),
            "p95": float(np.percentile(a, 95)), "max": float(a.max())}


def save(path, data):
    Path(path).write_text(json.dumps(data, indent=2) + "\n")


def observe(args):
    rclpy.init()
    node = Node("latency_observer")
    rows = []
    running = True

    def stop(*_):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    def receive(topic, msg):
        row = {"topic": topic, "wall_ns": time.time_ns(),
               "mono_ns": time.monotonic_ns(), "stamp_ns": stamp_ns(msg)}
        if topic == "armors":
            row["count"] = len(msg.armors)
            row["armors"] = [
                {
                    "number": armor.number,
                    "type": armor.type,
                    "distance_to_image_center": armor.distance_to_image_center,
                    "position": [
                        armor.pose.position.x,
                        armor.pose.position.y,
                        armor.pose.position.z,
                    ],
                    "orientation": [
                        armor.pose.orientation.x,
                        armor.pose.orientation.y,
                        armor.pose.orientation.z,
                        armor.pose.orientation.w,
                    ],
                    "kpts": [[point.x, point.y, point.z] for point in armor.kpts],
                }
                for armor in msg.armors
            ]
        elif topic == "tracker":
            row["tracking"] = bool(msg.tracking)
            row["id"] = msg.id
            row["armors_num"] = msg.armors_num
            row["position"] = [msg.position.x, msg.position.y, msg.position.z]
            row["velocity"] = [msg.velocity.x, msg.velocity.y, msg.velocity.z]
            row["yaw"] = msg.yaw
            row["v_yaw"] = msg.v_yaw
            row["radius_1"] = msg.radius_1
            row["radius_2"] = msg.radius_2
            row["dz"] = msg.dz
        elif topic == "feedback":
            row.update(sequence=msg.sequence, mcu_ms=msg.mcu_timestamp_ms,
                       yaw=msg.yaw, pitch=msg.pitch, mode=msg.mode)
        rows.append(row)

    subscriptions = []
    for topic, name, msg_type in [
        ("/camera_info", "camera_info", CameraInfo),
        ("/detector/result_img", "result_img", Image),
        ("/detector/armors", "armors", Armors),
        ("/tracker/raw_target", "tracker", Target),
        ("/serial_bridge/feedback", "feedback", GimbalFeedback),
    ]:
        subscriptions.append(node.create_subscription(
            msg_type, topic, lambda msg, n=name: receive(n, msg), qos_profile_sensor_data))
    try:
        while running and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        save(args.output, rows)
        node.destroy_node()
        rclpy.shutdown()


def summarize(data, observations):
    start, end = data["start_ns"], data["end_ns"]
    frames = [r for r in data["frames"] if start <= r["wall_ns"] < end]
    result = {"duration_s": (end - start) / 1e9, "frames": len(frames),
              "frames_with_candidates": sum(r.get("candidates", 0) > 0 for r in frames),
              "frames_with_armors": sum(r.get("armors", 0) > 0 for r in frames)}
    result["callback_ms"] = {
        key: stats([r.get(key, 0) for r in frames]) for key in
        ["decode_ms", "predict_ms", "geometry_ms", "draw_ms", "debug_convert_ms",
         "debug_publish_ms", "callback_ms", "input_age_ms"]}
    for name in ["camera_info", "result_img", "armors", "tracker", "feedback"]:
        samples = [r for r in observations if r["topic"] == name and start <= r["wall_ns"] < end]
        gaps = [(b["mono_ns"] - a["mono_ns"]) / 1e6 for a, b in zip(samples, samples[1:])]
        entry = {"n": len(samples), "fps": len(samples) / result["duration_s"],
                 "interval_ms": stats(gaps),
                 "age_ms": stats([(r["wall_ns"] - r["stamp_ns"]) / 1e6 for r in samples])}
        if name == "tracker":
            entry["tracking_frames"] = sum(r["tracking"] for r in samples)
        if name == "feedback":
            entry["yaw_span_rad"] = max((r["yaw"] for r in samples), default=0) - min((r["yaw"] for r in samples), default=0)
            entry["pitch_span_rad"] = max((r["pitch"] for r in samples), default=0) - min((r["pitch"] for r in samples), default=0)
            entry["mcu_interval_ms"] = stats([b["mcu_ms"] - a["mcu_ms"] for a, b in zip(samples, samples[1:])])
        result[name] = entry
    # Matched camera stamps avoid subtracting unrelated, separately averaged samples.
    arrivals = {r["stamp_ns"]: r["mono_ns"] for r in observations if r["topic"] == "armors"}
    result["tracker_after_detector_ms"] = stats([
        (r["mono_ns"] - arrivals[r["stamp_ns"]]) / 1e6 for r in observations
        if r["topic"] == "tracker" and start <= r["wall_ns"] < end and r["stamp_ns"] in arrivals])
    return result


def detector_run(args, ros_args):
    sys.path.insert(0, str(ROOT / "yolo_armor_detector/scripts"))
    import yolo_armor_detector_node as detector

    if args.debug_width and not hasattr(detector, "AsyncDebugPublisher"):
        original_convert = detector.bgr_to_image_msg

        def display_convert(image, header):
            if image.shape[1] > args.debug_width:
                height = round(image.shape[0] * args.debug_width / image.shape[1])
                image = detector.cv2.resize(image, (args.debug_width, height),
                                            interpolation=detector.cv2.INTER_AREA)
            return original_convert(image, header)

        detector.bgr_to_image_msg = display_convert

    rclpy.init(args=ros_args)
    data = {"phase": args.phase, "frames": [], "start_ns": None, "end_ns": None}

    class Probe(detector.YoloArmorNode):
        def __init__(self):
            self.current = None
            self.first_mono = None
            self.done = False
            super().__init__()
            self.wrap(detector, "image_msg_to_bgr", "decode_ms")
            self.wrap(detector, "bgr_to_image_msg", "debug_convert_ms")
            self.wrap(self.model, "predict", "predict_ms")
            if self.raw_track_latch is not None:
                self.wrap(self.raw_track_latch, "process_frame", "geometry_ms")
            else:
                self.wrap(self, "solve_armor", "geometry_ms")
            self.wrap(self, "draw_raw_result", "draw_ms")
            if self.debug_pub:
                self.wrap(self.debug_pub, "publish", "debug_publish_ms")
            self.wrap(self.armors_pub, "publish", "armors_publish_ms")
            self.create_timer(0.05, self.check_end)

        def wrap(self, owner, name, key):
            original = getattr(owner, name)

            def timed(*a, **kw):
                begin = time.perf_counter_ns()
                try:
                    result = original(*a, **kw)
                    if self.current is not None:
                        if key == "predict_ms":
                            self.current["candidates"] = len(result[0].boxes) if result[0].boxes is not None else 0
                        if key == "armors_publish_ms":
                            self.current["armors"] = len(a[0].armors)
                    return result
                finally:
                    if self.current is not None:
                        self.current[key] = self.current.get(key, 0) + (time.perf_counter_ns() - begin) / 1e6

            setattr(owner, name, timed)

        def image_callback(self, msg):
            mono, wall = time.monotonic(), time.time_ns()
            if self.first_mono is None:
                self.first_mono = mono
                data["start_ns"] = wall + int(args.warmup * 1e9)
                data["end_ns"] = data["start_ns"] + int(args.duration * 1e9)
            if self.done:
                return
            self.current = {"wall_ns": wall, "stamp_ns": stamp_ns(msg),
                            "input_age_ms": (wall - stamp_ns(msg)) / 1e6}
            begin = time.perf_counter_ns()
            try:
                super().image_callback(msg)
            finally:
                self.current["callback_ms"] = (time.perf_counter_ns() - begin) / 1e6
                data["frames"].append(self.current)
                self.current = None

        def check_end(self):
            if self.first_mono is not None and time.monotonic() - self.first_mono >= args.warmup + args.duration:
                self.done = True

    observer_path = str(Path(args.output).with_suffix(".observations.json"))
    observer = subprocess.Popen([sys.executable, __file__, "observe", "--output", observer_path])
    node = None
    try:
        node = Probe()
        data["model_path"] = str(detector.resolve_model_path(node.model_path))
        data["backend"] = str(node.model.predictor.device)
        data["fp16"] = bool(node.model.predictor.model.fp16)
        data["debug"] = node.publish_debug
        data["debug_width"] = args.debug_width
        data["async_debug"] = hasattr(detector, "AsyncDebugPublisher")
        data["worker_max_fps"] = getattr(node, "debug_max_fps", None)
        data["worker_max_width"] = getattr(node, "debug_max_width", None)
        deadline = time.monotonic() + args.warmup + args.duration + 30
        while rclpy.ok() and not node.done and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        data["completed"] = node.done
    finally:
        observer.send_signal(signal.SIGINT)
        observer.wait(timeout=10)
        if node:
            node.destroy_node()
        rclpy.shutdown()
        if data["start_ns"] is not None:
            data["summary"] = summarize(data, json.loads(Path(observer_path).read_text()))
        save(args.output, data)
    print(json.dumps({k: v for k, v in data.items() if k != "frames"}, indent=2), flush=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("mode", choices=["observe", "detector"])
    parser.add_argument("--output", required=True)
    parser.add_argument("--phase", default="unspecified")
    parser.add_argument("--duration", type=float, default=30)
    parser.add_argument("--warmup", type=float, default=5)
    parser.add_argument("--debug-width", type=int, default=0)
    args, ros_args = parser.parse_known_args()
    if args.mode == "observe":
        observe(args)
    else:
        detector_run(args, ros_args)


if __name__ == "__main__":
    main()
