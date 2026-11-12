#!/usr/bin/env python3
"""Time the existing Windows duplex receive path without forwarding any commands."""
import argparse
from collections import defaultdict
import json
from pathlib import Path
import sys
import threading
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "rm_serial_driver/scripts"))


class Trace:
    def __init__(self):
        self.started_ns = time.monotonic_ns()
        self.rows = defaultdict(list)
        self.durations = defaultdict(list)
        self.stop = threading.Event()
        self.worker = threading.Thread(target=self.heartbeat, daemon=True)
        self.worker.start()

    def heartbeat(self):
        while not self.stop.wait(0.1):
            self.rows["host_clock"].append([
                time.monotonic_ns(), time.clock_gettime_ns(time.CLOCK_MONOTONIC_RAW),
                time.time_ns()])

    def measure(self, name, callback, *args):
        start = time.monotonic_ns()
        try:
            return callback(*args)
        finally:
            end = time.monotonic_ns()
            elapsed_ms = (end - start) / 1e6
            self.durations[name].append(elapsed_ms)
            if elapsed_ms >= 10:
                self.rows["slow_call"].append([start, end, name])

    def save(self, path, role, seconds):
        self.stop.set()
        self.worker.join(timeout=1)
        summary = {}
        for name, durations in self.durations.items():
            ordered = sorted(durations)
            summary[name] = dict(count=len(ordered), mean_ms=sum(ordered) / len(ordered),
                                 p95_ms=ordered[int((len(ordered) - 1) * .95)], max_ms=ordered[-1])
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(dict(role=role, duration_s=seconds,
                                       started_ns=self.started_ns,
                                       callback_ms=summary, rows=dict(self.rows)), separators=(",", ":")))
        print(f"PROBE_SAVED {path}", flush=True)


def make_bridge(trace):
    import windows_duplex
    from protocol import FeedbackParser
    from serial_bridge_node import SerialBridgeNode

    parser = FeedbackParser()

    def receive_trace(event):
        now = time.monotonic_ns()
        if event["kind"] == "clock":
            trace.rows["windows_clock"].append([now, event["windows_ms"]])
            return
        frames = parser.feed(event["data"])
        start, end = event["windows_read_start_us"], event["windows_read_end_us"]
        trace.rows["windows_chunk"].append([now, start, end, len(event["data"]), len(frames)])
        for frame in frames:
            trace.rows["windows_frame"].append([now, start, end, frame.sequence, frame.mcu_timestamp_ms])

    class TimedTransport(windows_duplex.WindowsSerialDuplex):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, trace_callback=receive_trace, **kwargs)

        def _pump(self):
            return trace.measure("pipe_pump", super()._pump)

        def _send_line(self, *args):
            return trace.measure("pipe_send", super()._send_line, *args)

    windows_duplex.WindowsSerialDuplex = TimedTransport

    class ProbeBridge(SerialBridgeNode):
        def __init__(self):
            self.last_read_ns = None
            super().__init__()

        def read_serial(self):
            now = time.monotonic_ns()
            if self.last_read_ns is not None and now - self.last_read_ns >= 20_000_000:
                trace.rows["read_timer_gap"].append([self.last_read_ns, now])
            self.last_read_ns = now
            return trace.measure("read_serial", super().read_serial)

        def service_duplex_writes(self):
            return trace.measure("service_duplex", super().service_duplex_writes)

        def publish_feedback(self, frame):
            before = time.monotonic_ns()
            result = trace.measure("publish_feedback", super().publish_feedback, frame)
            trace.rows["published"].append([before, time.monotonic_ns(), frame.sequence,
                                            frame.mcu_timestamp_ms])
            return result

        def republish_states(self):
            return trace.measure("republish_states", super().republish_states)

        def update_health(self):
            return trace.measure("update_health", super().update_health)

        def publish_state(self, *args):
            return trace.measure("publish_state", super().publish_state, *args)

        def command_callback(self, message):
            # Keep the real duplex receive path, but never admit an angle command.
            trace.rows["ignored_command"].append([time.monotonic_ns(), message.valid])

        def write_command(self):
            return

    return ProbeBridge()


def make_observer(trace):
    from auto_aim_interfaces.msg import Armors, GimbalFeedback, Target
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from std_msgs.msg import Bool

    node = Node("serial_pipeline_observer")
    node.create_subscription(
        GimbalFeedback, "/serial_bridge/feedback",
        lambda msg: trace.rows["feedback"].append([
            time.monotonic_ns(), msg.sequence, msg.mcu_timestamp_ms, msg.yaw, msg.pitch, msg.mode]),
        qos_profile_sensor_data)
    node.create_subscription(
        Bool, "/serial_bridge/command_active",
        lambda msg: trace.rows["command_active"].append([time.monotonic_ns(), msg.data]),
        qos_profile_sensor_data)
    node.create_subscription(
        Armors, "/detector/armors",
        lambda msg: trace.rows["detector"].append([time.monotonic_ns(), len(msg.armors)]),
        qos_profile_sensor_data)
    node.create_subscription(
        Target, "/tracker/target",
        lambda msg: trace.rows["tracker"].append([time.monotonic_ns(), msg.tracking]),
        qos_profile_sensor_data)
    return node


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("role", choices=["bridge", "observer"])
    parser.add_argument("--duration", type=float, default=120)
    parser.add_argument("--output", type=Path, required=True)
    args, ros_args = parser.parse_known_args()
    import rclpy
    from rclpy.executors import SingleThreadedExecutor

    trace = Trace()
    rclpy.init(args=ros_args)
    node = None
    executor = SingleThreadedExecutor()
    try:
        node = make_bridge(trace) if args.role == "bridge" else make_observer(trace)
        executor.add_node(node)
        start = time.monotonic()
        print(f"PROBE_STARTED role={args.role} duration={args.duration}s commands=blocked", flush=True)
        while time.monotonic() - start < args.duration:
            executor.spin_once(timeout_sec=.02)
            if (args.role == "bridge" and time.monotonic() - start > 15
                    and not trace.rows.get("windows_frame")):
                print("PROBE_NO_FEEDBACK: no MCU frame within 15 seconds; stopping empty capture", flush=True)
                break
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        trace.save(args.output, args.role, args.duration)


if __name__ == "__main__":
    main()
