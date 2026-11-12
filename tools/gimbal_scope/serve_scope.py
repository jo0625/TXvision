#!/usr/bin/env python3
"""Local curve viewer. ROS subscriptions only; no command or device access."""
import argparse
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import threading
import time
from urllib.parse import parse_qs, urlsplit
import uuid

from build_scope import build

WALL_TIME_INDICES = {"feedback": 6, "forwarded": 3, "command_active": 2}


class LiveBuffer:
    def __init__(self):
        self.lock = threading.Lock()
        self.events = deque(maxlen=30000)
        self.started = time.monotonic()
        self.session = uuid.uuid4().hex
        self.sequence = 0
        self.last_feedback = None
        self.error = None
        self.ros_ready = False

    def append(self, kind, values):
        if not all(isinstance(value, (int, float, bool)) and math.isfinite(value)
                   for value in values):
            return
        now = time.monotonic()
        wall_ms = time.time_ns() // 1_000_000
        with self.lock:
            self.sequence += 1
            self.events.append([self.sequence, kind, [now - self.started, *values, wall_ms]])
            if kind == "feedback":
                self.last_feedback = now
            while self.events and now - self.started - self.events[0][2][0] > 120:
                self.events.popleft()

    def snapshot(self, after):
        with self.lock:
            now = time.monotonic()
            recent = [event for event in self.events if event[2][0] >= now - self.started - 120]
            return dict(session=self.session, cursor=self.sequence, elapsed=now - self.started,
                        wall_time_indices=WALL_TIME_INDICES,
                        wall_time_source="observer_host_receive_unix_ms",
                        events=[event for event in recent if event[0] > after],
                        truncated=bool(recent and after and after < recent[0][0] - 1),
                        feedback_age=None if self.last_feedback is None else now - self.last_feedback,
                        ros_ready=self.ros_ready, error=self.error)


def receive_ros(buffer, stop):
    node = None
    context = None
    executor = None
    try:
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from auto_aim_interfaces.msg import AimCommand, GimbalFeedback
        from std_msgs.msg import Bool

        context = Context()
        rclpy.init(context=context)
        node = Node("gimbal_scope_observer", context=context, enable_rosout=False,
                    start_parameter_services=False)
        node.create_subscription(
            GimbalFeedback, "/serial_bridge/feedback",
            lambda msg: buffer.append("feedback", [msg.sequence, msg.mcu_timestamp_ms,
                                                    msg.yaw, msg.pitch, msg.mode]),
            qos_profile_sensor_data)
        node.create_subscription(
            AimCommand, "/serial_bridge/aim_command",
            lambda msg: buffer.append("forwarded", [msg.yaw, msg.pitch]) if msg.valid else None,
            qos_profile_sensor_data)
        node.create_subscription(
            Bool, "/serial_bridge/command_active",
            lambda msg: buffer.append("command_active", [msg.data]), qos_profile_sensor_data)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        buffer.ros_ready = True
        while not stop.is_set():
            executor.spin_once(timeout_sec=0.1)
    except Exception as exc:
        buffer.error = str(exc)
    finally:
        buffer.ros_ready = False
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        if context is not None and context.ok():
            context.shutdown()


def make_handler(html, buffer):
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            path = urlsplit(self.path)
            if path.path == "/api/live":
                try:
                    after = max(0, int(parse_qs(path.query).get("after", ["0"])[0]))
                except ValueError:
                    self.send_error(400, "Invalid cursor")
                    return
                body = json.dumps(buffer.snapshot(after), allow_nan=False).encode()
                content_type = "application/json"
            elif path.path in ("/", "/index.html"):
                body, content_type = html, "text/html; charset=utf-8"
            elif path.path == "/favicon.ico":
                self.send_response(204)
                self.end_headers()
                return
            else:
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            try:
                self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def log_message(self, *_args):
            pass

    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=8765)
    args = parser.parse_args()
    html = build().read_bytes()
    buffer = LiveBuffer()
    stop = threading.Event()
    # Bind before starting ROS so an occupied port leaves no orphan observer.
    server = ThreadingHTTPServer(("127.0.0.1", args.port), make_handler(html, buffer))
    worker = threading.Thread(target=receive_ros, args=(buffer, stop), daemon=True)
    worker.start()
    print(f"Gimbal scope: http://localhost:{server.server_port} (subscriptions only)", flush=True)
    try:
        server.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        server.server_close()
        worker.join(timeout=2)


if __name__ == "__main__":
    main()
