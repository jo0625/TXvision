#!/usr/bin/env python3
"""Compare raw, annotated, and screen output outside performance windows."""

from __future__ import annotations

import argparse
from collections import Counter, OrderedDict
from datetime import datetime
import json
from pathlib import Path
import signal
import subprocess
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


BAYER_CONVERSIONS = {
    "bayer_rggb8": cv2.COLOR_BayerBG2BGR,
    "bayer_bggr8": cv2.COLOR_BayerRG2BGR,
    "bayer_gbrg8": cv2.COLOR_BayerGR2BGR,
    "bayer_grbg8": cv2.COLOR_BayerGB2BGR,
}


def iso_now():
    return datetime.now().astimezone().isoformat(timespec="milliseconds")


def stamp_ns(msg):
    return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec


def decode_image(encoding, height, width, step, payload):
    encoding = encoding.lower()
    if encoding in {"mono8", "8uc1", *BAYER_CONVERSIONS}:
        channels = 1
    elif encoding in {"bgr8", "rgb8", "8uc3"}:
        channels = 3
    elif encoding in {"bgra8", "rgba8"}:
        channels = 4
    else:
        raise ValueError(f"unsupported encoding: {encoding}")

    row_bytes = width * channels
    step = step or row_bytes
    if height <= 0 or width <= 0 or step < row_bytes:
        raise ValueError(
            f"invalid image geometry: height={height} width={width} step={step}"
        )
    raw = np.frombuffer(payload, dtype=np.uint8)
    required = height * step
    if raw.size < required:
        raise ValueError(f"image buffer too short: {raw.size} < {required}")
    rows = raw[:required].reshape(height, step)[:, :row_bytes]

    if channels == 1:
        plane = rows.reshape(height, width)
        if encoding in BAYER_CONVERSIONS:
            return cv2.cvtColor(plane, BAYER_CONVERSIONS[encoding])
        return cv2.cvtColor(plane, cv2.COLOR_GRAY2BGR)
    image = rows.reshape(height, width, channels)
    if encoding == "rgb8":
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    if encoding == "rgba8":
        return cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)
    if encoding == "bgra8":
        return cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
    return image[:, :, :3].copy()


def image_record(msg):
    return {
        "encoding": msg.encoding,
        "height": int(msg.height),
        "width": int(msg.width),
        "step": int(msg.step),
        "payload": bytes(msg.data),
    }


def image_stats(image):
    pixels = image.reshape(-1, 3).astype(np.float32)
    luminance = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    return {
        "mean_bgr": pixels.mean(axis=0).tolist(),
        "std_bgr": pixels.std(axis=0).tolist(),
        "mean_luma": float(luminance.mean()),
        "std_luma": float(luminance.std()),
        "black_fraction": float(np.mean(luminance <= 3)),
    }


def analyze_pair(raw, result, analysis_width=320):
    height = max(1, round(raw.shape[0] * analysis_width / raw.shape[1]))
    raw_small = cv2.resize(raw, (analysis_width, height), interpolation=cv2.INTER_AREA)
    result_small = cv2.resize(
        result, (analysis_width, height), interpolation=cv2.INTER_AREA
    )
    difference = np.abs(raw_small.astype(np.int16) - result_small.astype(np.int16))
    bgr_mae = float(difference.mean())
    swapped_mae = float(
        np.abs(raw_small[:, :, ::-1].astype(np.int16) - result_small.astype(np.int16)).mean()
    )
    raw_summary = image_stats(raw_small)
    result_summary = image_stats(result_small)
    raw_luma = raw_summary["mean_luma"]
    result_luma = result_summary["mean_luma"]
    flags = []
    if result_luma <= 3.0 or (
        raw_luma > 5.0
        and result_luma < raw_luma * 0.25
        and result_summary["black_fraction"] > raw_summary["black_fraction"] + 0.25
    ):
        flags.append("black_result")
    if swapped_mae + 8.0 < bgr_mae:
        flags.append("possible_red_blue_swap")
    if float(np.median(difference)) > 20.0:
        flags.append("content_mismatch")
    return {
        "raw": raw_summary,
        "result": result_summary,
        "difference": {
            "mean_abs_bgr": bgr_mae,
            "median_abs_bgr": float(np.median(difference)),
            "p95_abs_bgr": float(np.percentile(difference, 95)),
            "red_blue_swapped_mean_abs": swapped_mae,
        },
        "flags": flags,
    }


def endpoint_snapshot(node, topic):
    endpoints = []
    for info in node.get_publishers_info_by_topic(topic):
        endpoints.append(
            {
                "node_namespace": info.node_namespace,
                "node_name": info.node_name,
                "endpoint_gid": list(info.endpoint_gid),
                "qos": str(info.qos_profile),
            }
        )
    return endpoints


def start_screen_recording(args, output):
    if args.no_screen:
        return None, None, None
    video = output.with_name(output.name + "_screen.mkv")
    log_path = output.with_name(output.name + "_screen_gstreamer.log")
    log_handle = log_path.open("wb")
    frame_count = max(1, round(args.duration * args.screen_fps))
    rate_numerator = max(1, round(args.screen_fps * 1000))
    command = [
        "gst-launch-1.0",
        "-e",
        "ximagesrc",
        f"display-name={args.display}",
        "use-damage=0",
        "show-pointer=true",
        f"num-buffers={frame_count}",
        "!",
        f"video/x-raw,framerate={rate_numerator}/1000",
        "!",
        "videoconvert",
        "!",
        "vp8enc",
        "deadline=1",
        "cpu-used=8",
        "threads=4",
        "!",
        "matroskamux",
        "!",
        "filesink",
        f"location={video}",
    ]
    process = subprocess.Popen(command, stdout=log_handle, stderr=subprocess.STDOUT)
    return process, log_handle, command


def save_pair(output, index, stamp, raw, result, label):
    result_resized = cv2.resize(
        result, (raw.shape[1], raw.shape[0]), interpolation=cv2.INTER_AREA
    )
    montage = np.concatenate((raw, result_resized), axis=1)
    path = output.with_name(
        f"{output.name}_{label}_{index:04d}_{stamp}.png"
    )
    cv2.imwrite(str(path), montage)
    return str(path)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--duration", type=float, default=60.0)
    parser.add_argument("--raw-topic", default="/image_raw")
    parser.add_argument("--result-topic", default="/detector/result_img")
    parser.add_argument("--raw-buffer", type=int, default=50)
    parser.add_argument("--analysis-width", type=int, default=320)
    parser.add_argument("--periodic-seconds", type=float, default=10.0)
    parser.add_argument("--max-anomaly-images", type=int, default=20)
    parser.add_argument("--display", default=":1")
    parser.add_argument("--screen-fps", type=float, default=15.0)
    parser.add_argument("--no-screen", action="store_true")
    args = parser.parse_args()
    if args.duration <= 0 or args.raw_buffer <= 0 or args.analysis_width <= 0:
        parser.error("duration, raw-buffer, and analysis-width must be positive")

    args.output.parent.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = Node("visual_integrity_probe")
    raw_frames = OrderedDict()
    pairs = []
    saved_images = []
    errors = []
    graph = []
    counts = Counter()
    running = True
    started_mono = time.monotonic()
    started_wall = iso_now()
    next_graph = started_mono
    next_periodic = started_mono

    def stop(*_):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    def raw_callback(msg):
        counts["raw"] += 1
        raw_frames[stamp_ns(msg)] = image_record(msg)
        while len(raw_frames) > args.raw_buffer:
            raw_frames.popitem(last=False)

    def result_callback(msg):
        nonlocal next_periodic
        counts["result"] += 1
        stamp = stamp_ns(msg)
        raw_record = raw_frames.pop(stamp, None)
        if raw_record is None:
            counts["unmatched_result"] += 1
            return
        try:
            raw = decode_image(**raw_record)
            result = decode_image(
                msg.encoding, msg.height, msg.width, msg.step, bytes(msg.data)
            )
            metrics = analyze_pair(raw, result, args.analysis_width)
        except Exception as exc:  # noqa: BLE001 - retain malformed-frame evidence
            counts["decode_error"] += 1
            errors.append({"stamp_ns": stamp, "error": repr(exc)})
            return

        metrics.update(
            {
                "stamp_ns": stamp,
                "received_at": iso_now(),
                "raw_encoding": raw_record["encoding"],
                "raw_size": [raw_record["width"], raw_record["height"]],
                "result_encoding": msg.encoding,
                "result_size": [int(msg.width), int(msg.height)],
            }
        )
        pairs.append(metrics)
        counts["matched_pair"] += 1
        counts.update(metrics["flags"])
        now = time.monotonic()
        label = None
        if metrics["flags"] and counts["saved_anomaly"] < args.max_anomaly_images:
            label = "anomaly"
            counts["saved_anomaly"] += 1
        elif now >= next_periodic:
            label = "periodic"
            next_periodic = now + args.periodic_seconds
        if label:
            saved_images.append(
                save_pair(args.output, len(saved_images), stamp, raw, result, label)
            )

    subscriptions = [
        node.create_subscription(Image, args.raw_topic, raw_callback, qos_profile_sensor_data),
        node.create_subscription(
            Image, args.result_topic, result_callback, qos_profile_sensor_data
        ),
    ]
    screen_process, screen_log, screen_command = start_screen_recording(args, args.output)
    screen_started_wall = iso_now() if screen_process else None
    try:
        deadline = started_mono + args.duration
        while running and rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            now = time.monotonic()
            if now >= next_graph:
                graph.append(
                    {
                        "at": iso_now(),
                        "raw_publishers": endpoint_snapshot(node, args.raw_topic),
                        "result_publishers": endpoint_snapshot(node, args.result_topic),
                    }
                )
                next_graph = now + 1.0
    finally:
        ended_wall = iso_now()
        elapsed = time.monotonic() - started_mono
        if screen_process is not None:
            try:
                screen_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                screen_process.send_signal(signal.SIGINT)
                screen_process.wait(timeout=5)
            screen_returncode = screen_process.returncode
            screen_log.close()
        else:
            screen_returncode = None
        node.destroy_node()
        rclpy.shutdown()

        result = {
            "started_at": started_wall,
            "ended_at": ended_wall,
            "duration_s": elapsed,
            "topics": {"raw": args.raw_topic, "result": args.result_topic},
            "counts": dict(counts),
            "rates_hz": {
                "raw": counts["raw"] / elapsed,
                "result": counts["result"] / elapsed,
                "matched_pair": counts["matched_pair"] / elapsed,
            },
            "publisher_graph": graph,
            "pairs": pairs,
            "errors": errors,
            "saved_images": saved_images,
            "screen": {
                "started_at": screen_started_wall,
                "command": screen_command,
                "returncode": screen_returncode,
                "video": None
                if args.no_screen
                else str(args.output.with_name(args.output.name + "_screen.mkv")),
            },
        }
        args.output.with_suffix(".json").write_text(
            json.dumps(result, indent=2) + "\n", encoding="utf-8"
        )


if __name__ == "__main__":
    main()
