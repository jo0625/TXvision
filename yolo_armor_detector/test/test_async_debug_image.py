import multiprocessing as mp
from pathlib import Path
import sys
import time
import uuid

import numpy as np
import pytest
from std_msgs.msg import Header

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from async_debug_image import AsyncDebugPublisher, LatestImage, image_message


def header(sequence=1):
    msg = Header()
    msg.stamp.sec = sequence
    msg.stamp.nanosec = 12345
    msg.frame_id = "camera_optical_frame"
    return msg


def test_latest_image_replaces_old_frame_and_preserves_header():
    slot = LatestImage(mp.get_context("spawn"), 12 * 16 * 3)
    for value in range(1, 8):
        assert slot.put(np.full((12, 16, 3), value, np.uint8), header(value))
    sequence, image, sec, nano, frame = slot.take(0)
    assert sequence == sec == 7
    assert nano == 12345 and frame == "camera_optical_frame"
    assert np.all(image == 7)
    assert slot.take(sequence) is None


def hold_slot(slot, entered, release):
    with slot.lock:
        entered.set()
        release.wait(10)


def test_busy_consumer_does_not_block_detector():
    context = mp.get_context("spawn")
    slot = LatestImage(context, 12)
    entered, release = context.Event(), context.Event()
    process = context.Process(target=hold_slot, args=(slot, entered, release))
    process.start()
    try:
        assert entered.wait(10)
        started = time.monotonic()
        assert slot.put(np.zeros((2, 2, 3), np.uint8), header()) is False
        assert time.monotonic() - started < 0.5
    finally:
        release.set()
        process.join(10)
        if process.is_alive():
            process.terminate()
            process.join()
    assert process.exitcode == 0


def test_conversion_resize_is_display_only_and_preserves_stamp():
    image = np.arange(8 * 12 * 3, dtype=np.uint8).reshape(8, 12, 3)
    original = image.copy()
    message = image_message(image, 42, 765, "optical", 6)
    assert (message.width, message.height, message.step) == (6, 4, 18)
    assert message.header.stamp.sec == 42 and message.header.stamp.nanosec == 765
    assert message.header.frame_id == "optical" and message.encoding == "bgr8"
    assert len(message.data) == 72
    np.testing.assert_array_equal(original, image)


def test_noncontiguous_input_and_consistent_snapshot():
    slot = LatestImage(mp.get_context("spawn"), 8 * 12 * 3)
    source = np.full((8, 24, 3), 19, np.uint8)[:, ::2]
    assert slot.put(source, header())
    source[:] = 88
    assert np.all(slot.take(0)[1] == 19)


@pytest.mark.parametrize("rate,width", [(0, 960), (float("nan"), 960), (15, -1)])
def test_invalid_limits(rate, width):
    with pytest.raises(ValueError):
        AsyncDebugPublisher("/unused", "unused", rate, width)


def test_worker_publishes_image_and_stops():
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image

    rclpy.init(args=[])
    name = "debug_test_" + uuid.uuid4().hex[:8]
    node = rclpy.create_node(name)
    received = []
    subscription = node.create_subscription(Image, "/" + name, received.append,
                                            qos_profile_sensor_data)
    publisher = AsyncDebugPublisher("/" + name, name + "_worker", 15, 6)
    process = None
    try:
        publisher.publish(np.full((8, 12, 3), 31, np.uint8), header(9))
        process = publisher.process
        deadline = time.monotonic() + 10
        while not received and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        assert received
        assert (received[0].width, received[0].height) == (6, 4)
        assert received[0].header == header(9)
        assert set(received[0].data) == {31}
    finally:
        publisher.close()
        publisher.close()
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()
    assert publisher.process is None
