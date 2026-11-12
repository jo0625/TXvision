#!/usr/bin/env python3
"""Capture one stamp-matched raw, annotated, and nonempty armor sample."""

import argparse
import json
import sys
import time
from collections import OrderedDict
from pathlib import Path

import cv2
import numpy as np
import rclpy
from auto_aim_interfaces.msg import Armors
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "yolo_armor_detector" / "scripts"))
from yolo_armor_detector_node import image_msg_to_bgr  # noqa: E402


def stamp_ns(msg):
    return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec


def channel_stats(image):
    pixels = image.reshape(-1, 3).astype(np.int16)
    blue, green, red = pixels.T
    return {
        "mean_bgr": [float(value) for value in pixels.mean(axis=0)],
        "std_bgr": [float(value) for value in pixels.std(axis=0)],
        "red_dominant_fraction": float(
            np.mean((red > blue + 20) & (red > green + 20))
        ),
        "blue_dominant_fraction": float(
            np.mean((blue > red + 20) & (blue > green + 20))
        ),
    }


def image_rows(msg, channels):
    row_bytes = msg.width * channels
    step = msg.step or row_bytes
    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    return raw[: msg.height * step].reshape(msg.height, step)[:, :row_bytes]


def armor_data(armor):
    return {
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


class CaptureNode(Node):
    def __init__(self, output_prefix, cache_size, max_raw_offset_ms):
        super().__init__("format_visual_capture")
        self.output_prefix = Path(output_prefix)
        self.cache_size = cache_size
        self.max_raw_offset_ns = int(max_raw_offset_ms * 1e6)
        self.raw = OrderedDict()
        self.result = OrderedDict()
        self.armors = OrderedDict()
        self.saved = False
        self.create_subscription(
            Image, "/image_raw", self.on_raw, qos_profile_sensor_data
        )
        self.create_subscription(
            Image, "/detector/result_img", self.on_result, qos_profile_sensor_data
        )
        self.create_subscription(
            Armors, "/detector/armors", self.on_armors, qos_profile_sensor_data
        )

    def put(self, cache, key, value):
        cache[key] = value
        cache.move_to_end(key)
        while len(cache) > self.cache_size:
            cache.popitem(last=False)

    def on_raw(self, msg):
        key = stamp_ns(msg)
        self.put(self.raw, key, msg)
        self.try_save(key)

    def on_result(self, msg):
        key = stamp_ns(msg)
        self.put(self.result, key, msg)
        self.try_save(key)

    def on_armors(self, msg):
        if not msg.armors:
            return
        key = stamp_ns(msg)
        self.put(self.armors, key, msg)
        self.try_save(key)

    def try_save(self, key):
        if self.saved or key not in self.result or key not in self.armors or not self.raw:
            return
        raw_key = min(self.raw, key=lambda candidate: abs(candidate - key))
        if abs(raw_key - key) > self.max_raw_offset_ns:
            return
        raw_msg = self.raw[raw_key]
        result_msg = self.result[key]
        armors_msg = self.armors[key]
        raw = image_msg_to_bgr(raw_msg)
        result = image_msg_to_bgr(result_msg)
        primary = min(
            armors_msg.armors, key=lambda armor: armor.distance_to_image_center
        )
        points = np.asarray([[point.x, point.y] for point in primary.kpts])
        x0 = max(0, int(np.floor(points[:, 0].min())) - 20)
        y0 = max(0, int(np.floor(points[:, 1].min())) - 20)
        x1 = min(raw.shape[1], int(np.ceil(points[:, 0].max())) + 21)
        y1 = min(raw.shape[0], int(np.ceil(points[:, 1].max())) + 21)
        roi = raw[y0:y1, x0:x1]

        self.output_prefix.parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(self.output_prefix) + "_raw.png", raw)
        cv2.imwrite(str(self.output_prefix) + "_result.png", result)
        cv2.imwrite(str(self.output_prefix) + "_armor_roi.png", roi)
        metadata = {
            "captured_wall_ns": time.time_ns(),
            "stamp_ns": key,
            "raw_stamp_ns": raw_key,
            "raw_result_stamp_delta_ms": (raw_key - key) / 1e6,
            "raw_encoding": raw_msg.encoding,
            "result_encoding": result_msg.encoding,
            "size": [raw_msg.width, raw_msg.height],
            "raw": channel_stats(raw),
            "armor_roi": channel_stats(roi),
            "roi_xyxy": [x0, y0, x1, y1],
            "armors": [armor_data(armor) for armor in armors_msg.armors],
        }
        if raw_msg.encoding.lower().startswith("bayer_"):
            mosaic = image_rows(raw_msg, 1).reshape(raw_msg.height, raw_msg.width)
            cv2.imwrite(str(self.output_prefix) + "_mosaic.png", mosaic)
            variants = {
                "RG": cv2.COLOR_BayerRG2BGR,
                "BG": cv2.COLOR_BayerBG2BGR,
                "GR": cv2.COLOR_BayerGR2BGR,
                "GB": cv2.COLOR_BayerGB2BGR,
            }
            metadata["bayer_variants"] = {}
            for name, conversion in variants.items():
                image = cv2.cvtColor(mosaic, conversion)
                image_roi = image[y0:y1, x0:x1]
                cv2.imwrite(
                    str(self.output_prefix) + f"_bayer_{name.lower()}.png", image
                )
                metadata["bayer_variants"][name] = {
                    "raw": channel_stats(image),
                    "armor_roi": channel_stats(image_roi),
                }
        Path(str(self.output_prefix) + "_visual.json").write_text(
            json.dumps(metadata, indent=2) + "\n"
        )
        self.saved = True


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-prefix", required=True)
    parser.add_argument("--timeout", type=float, default=20.0)
    parser.add_argument("--cache-size", type=int, default=40)
    parser.add_argument("--max-raw-offset-ms", type=float, default=15.0)
    args = parser.parse_args()

    rclpy.init()
    node = CaptureNode(
        args.output_prefix, args.cache_size, args.max_raw_offset_ms
    )
    deadline = time.monotonic() + args.timeout
    try:
        while rclpy.ok() and not node.saved and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    if not node.saved:
        raise SystemExit("no stamp-matched nonempty raw/result/armors sample captured")


if __name__ == "__main__":
    main()
