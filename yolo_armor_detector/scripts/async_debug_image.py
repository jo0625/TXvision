"""Bounded latest-image handoff to a separate ROS publisher process."""

import array
import math
import multiprocessing as mp
import time

import numpy as np


class LatestImage:
    def __init__(self, context, capacity):
        self.pixels = context.RawArray("B", capacity)
        self.frame_id = context.RawArray("B", 1024)
        # sequence, height, width, stamp seconds, stamp nanoseconds, frame-id length
        self.metadata = context.RawArray("q", 6)
        self.lock = context.Lock()

    def put(self, image, header):
        frame_id = header.frame_id.encode("utf-8")
        if image.dtype != np.uint8 or image.ndim != 3 or image.shape[2] != 3:
            raise ValueError("debug image must be uint8 BGR")
        if image.size > len(self.pixels) or len(frame_id) > len(self.frame_id):
            raise ValueError("debug image/header exceeds shared slot capacity")
        if not self.lock.acquire(False):
            return False
        try:
            target = np.frombuffer(self.pixels, dtype=np.uint8, count=image.size)
            np.copyto(target.reshape(image.shape), image)
            self.frame_id[:len(frame_id)] = frame_id
            self.metadata[1:] = [image.shape[0], image.shape[1], header.stamp.sec,
                                 header.stamp.nanosec, len(frame_id)]
            self.metadata[0] += 1
        finally:
            self.lock.release()
        return True

    def take(self, previous_sequence):
        with self.lock:
            sequence, height, width, sec, nanosec, length = self.metadata[:]
            if sequence == previous_sequence:
                return None
            image = np.frombuffer(self.pixels, dtype=np.uint8, count=height * width * 3)
            image = image.reshape(height, width, 3).copy()
            frame_id = bytes(self.frame_id[:length]).decode("utf-8")
        return sequence, image, sec, nanosec, frame_id


def image_message(image, sec, nanosec, frame_id, max_width):
    import cv2
    from sensor_msgs.msg import Image

    if max_width and image.shape[1] > max_width:
        height = max(1, round(image.shape[0] * max_width / image.shape[1]))
        image = cv2.resize(image, (max_width, height), interpolation=cv2.INTER_AREA)
    image = np.ascontiguousarray(image)
    msg = Image()
    msg.header.stamp.sec = sec
    msg.header.stamp.nanosec = nanosec
    msg.header.frame_id = frame_id
    msg.height, msg.width = image.shape[:2]
    msg.encoding = "bgr8"
    msg.step = msg.width * 3
    msg.data = array.array("B", image.tobytes())
    return msg


def publisher_process(slot, stop, topic, node_name, rate, max_width):
    import cv2
    import rclpy
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Image

    # Spawn avoids inheriting CUDA state. ROS arguments belong to the detector,
    # so the worker receives only its resolved topic and a distinct node name.
    cv2.setNumThreads(1)
    rclpy.init(args=[])
    node = rclpy.create_node(node_name)
    publisher = node.create_publisher(Image, topic, QoSProfile(
        depth=1, history=HistoryPolicy.KEEP_LAST,
        reliability=ReliabilityPolicy.BEST_EFFORT))
    sequence = 0
    next_publish = 0.0
    report_start = time.monotonic()
    count = 0
    convert_ms = publish_ms = 0.0
    try:
        while rclpy.ok() and not stop.is_set():
            rclpy.spin_once(node, timeout_sec=0)
            now = time.monotonic()
            if now < next_publish or publisher.get_subscription_count() == 0:
                stop.wait(0.005)
                continue
            frame = slot.take(sequence)
            if frame is None:
                stop.wait(0.005)
                continue
            sequence, image, sec, nanosec, frame_id = frame
            begin = time.perf_counter()
            message = image_message(image, sec, nanosec, frame_id, max_width)
            converted = time.perf_counter()
            publisher.publish(message)
            finished = time.perf_counter()
            convert_ms += (converted - begin) * 1000
            publish_ms += (finished - converted) * 1000
            count += 1
            next_publish = now + 1.0 / rate
            elapsed = time.monotonic() - report_start
            if elapsed >= 5.0:
                node.get_logger().info(
                    f"Debug worker: {count / elapsed:.1f} FPS, "
                    f"convert={convert_ms / count:.2f} ms, "
                    f"publish={publish_ms / count:.2f} ms, "
                    f"size={message.width}x{message.height}")
                report_start = time.monotonic()
                count = 0
                convert_ms = publish_ms = 0.0
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


class AsyncDebugPublisher:
    def __init__(self, topic, node_name, rate=30.0, max_width=960):
        if not math.isfinite(rate) or rate <= 0 or max_width < 0:
            raise ValueError("debug rate must be positive; width must be nonnegative")
        self.topic, self.node_name = topic, node_name
        self.rate, self.max_width = rate, max_width
        self.context = mp.get_context("spawn")
        self.slot = None
        self.process = None
        self.stop = None

    def publish(self, image, header):
        if self.slot is None or image.size > len(self.slot.pixels):
            self.close()
            self.slot = LatestImage(self.context, image.size)
            self.stop = self.context.Event()
            self.process = self.context.Process(
                target=publisher_process,
                args=(self.slot, self.stop, self.topic, self.node_name,
                      self.rate, self.max_width), daemon=True)
            self.process.start()
        if not self.process.is_alive():
            raise RuntimeError("debug publisher process exited")
        return self.slot.put(image, header)

    def close(self):
        if self.process is not None:
            self.stop.set()
            self.process.join(timeout=1.0)
            if self.process.is_alive():
                self.process.terminate()
                self.process.join(timeout=1.0)
            self.process.close()
        self.process = None
        self.slot = None
