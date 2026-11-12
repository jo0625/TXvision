from pathlib import Path
import sys

import cv2
import numpy as np
from sensor_msgs.msg import Image


TOOLS = Path(__file__).resolve().parents[2] / "tools"
sys.path.insert(0, str(TOOLS))

from visual_integrity_probe import analyze_pair, decode_image, image_record  # noqa: E402


def test_decode_bayer_rggb8_and_padded_bgr():
    bayer = np.zeros((8, 10), dtype=np.uint8)
    bayer[0::2, 0::2] = 255
    decoded = decode_image("bayer_rggb8", 8, 10, 10, bayer.tobytes())
    assert decoded[:, :, 2].mean() == 255
    assert decoded[:, :, 0].mean() == 0

    bgr = np.arange(8 * 10 * 3, dtype=np.uint8).reshape(8, 10, 3)
    padded = np.pad(bgr, ((0, 0), (0, 2), (0, 0)), constant_values=99)
    restored = decode_image("bgr8", 8, 10, padded.strides[0], padded.tobytes())
    np.testing.assert_array_equal(restored, bgr)


def test_image_record_can_be_passed_to_decoder():
    image = np.arange(8 * 10 * 3, dtype=np.uint8).reshape(8, 10, 3)
    message = Image()
    message.encoding = "bgr8"
    message.height, message.width = image.shape[:2]
    message.step = message.width * 3
    message.data = image.reshape(-1).tolist()
    np.testing.assert_array_equal(decode_image(**image_record(message)), image)


def test_pair_analysis_accepts_matching_annotated_image():
    raw = np.full((240, 320, 3), (20, 40, 80), dtype=np.uint8)
    result = raw.copy()
    cv2.rectangle(result, (80, 80), (180, 160), (0, 255, 0), 2)
    analysis = analyze_pair(raw, result)
    assert analysis["flags"] == []


def test_pair_analysis_detects_black_and_red_blue_swap():
    raw = np.full((240, 320, 3), (10, 40, 180), dtype=np.uint8)
    black = analyze_pair(raw, np.zeros_like(raw))
    assert "black_result" in black["flags"]

    swapped = analyze_pair(raw, raw[:, :, ::-1].copy())
    assert "possible_red_blue_swap" in swapped["flags"]
