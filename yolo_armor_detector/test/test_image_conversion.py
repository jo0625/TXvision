#!/usr/bin/env python3

from __future__ import annotations

import array
import sys
from pathlib import Path

import numpy as np
import pytest
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import Image
from std_msgs.msg import Header


PACKAGE = Path(__file__).resolve().parents[1]
SCRIPTS = PACKAGE / "scripts"
sys.path.insert(0, str(SCRIPTS))

import yolo_armor_detector_node as detector  # noqa: E402


def make_header() -> Header:
    header = Header()
    header.frame_id = "camera_optical_frame"
    header.stamp.sec = 123
    header.stamp.nanosec = 456_789
    return header


def assert_message_matches(message: Image, image: np.ndarray, header: Header) -> None:
    contiguous = np.ascontiguousarray(image)
    assert message.header == header
    assert message.height == contiguous.shape[0]
    assert message.width == contiguous.shape[1]
    assert message.encoding == "bgr8"
    assert message.is_bigendian == 0
    assert message.step == contiguous.shape[1] * 3
    assert isinstance(message.data, array.array)
    assert message.data.typecode == "B"
    assert bytes(message.data) == contiguous.tobytes()


def test_bgr_to_image_message_fields_and_bytes() -> None:
    image = np.arange(6 * 8 * 3, dtype=np.uint8).reshape(6, 8, 3)
    header = make_header()
    message = detector.bgr_to_image_msg(image, header)
    assert_message_matches(message, image, header)


def test_bgr_to_image_message_serialization_round_trip() -> None:
    image = np.arange(7 * 9 * 3, dtype=np.uint8).reshape(7, 9, 3)
    header = make_header()
    message = detector.bgr_to_image_msg(image, header)
    restored = deserialize_message(serialize_message(message), Image)
    assert_message_matches(restored, image, header)
    assert restored == message


def test_bgr_to_image_message_non_contiguous_input() -> None:
    source = np.arange(10 * 16 * 3, dtype=np.uint8).reshape(10, 16, 3)
    image = source[:, ::2, :]
    assert not image.flags.c_contiguous
    header = make_header()
    message = detector.bgr_to_image_msg(image, header)
    assert_message_matches(message, image, header)


def test_model_path_expands_environment_variable(tmp_path, monkeypatch) -> None:
    model = tmp_path / "model.pt"
    model.touch()
    monkeypatch.setenv("YOLO_ARMOR_MODEL", str(model))

    assert detector.resolve_model_path("${YOLO_ARMOR_MODEL}") == model


def test_model_path_rejects_unset_environment_variable(monkeypatch) -> None:
    monkeypatch.delenv("YOLO_ARMOR_MODEL", raising=False)

    with pytest.raises(ValueError, match="YOLO_ARMOR_MODEL"):
        detector.resolve_model_path("${YOLO_ARMOR_MODEL}")
