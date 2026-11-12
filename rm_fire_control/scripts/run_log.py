"""Bounded JSONL capture for a separate, passive ROS observer."""
from collections import Counter
from datetime import datetime
import json
import math
from pathlib import Path
import time
from uuid import uuid4
from zoneinfo import ZoneInfo


def json_safe(value):
    if isinstance(value, float) and not math.isfinite(value):
        return str(value)
    if isinstance(value, dict):
        return {str(k): json_safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(v) for v in value]
    if hasattr(value, "tolist"):
        return json_safe(value.tolist())
    return value


class RunLog:
    def __init__(self, root, max_bytes=512 * 1024 * 1024):
        if max_bytes <= 0:
            raise ValueError("max_bytes must be positive")
        name = datetime.now(ZoneInfo("Asia/Shanghai")).strftime("%Y%m%d_%H%M%S_%f")
        self.directory = Path(root).expanduser() / f"{name}_{uuid4().hex[:8]}"
        self.directory.mkdir(parents=True, exist_ok=False)
        self.stream = (self.directory / "events.jsonl").open("x", encoding="utf-8")
        self.max_bytes = max_bytes
        self.bytes_written = 0
        self.received = Counter()
        self.written = Counter()
        self.valid = {}
        self.decisions = Counter()
        self.source_sequences = {}
        self.source_gaps = Counter()
        self.parameters = {}
        self.expected_nodes = []
        self.topic_types = {}
        self.error = None
        self.closed = False
        self.started_ns = time.time_ns()
        self.started_monotonic_ns = time.monotonic_ns()
        self.checkpoint()

    def record(self, topic, message, ros_ns, host_ns=None):
        self.received[topic] += 1
        if self.error:
            return False
        data = json_safe(message)
        row = dict(schema=1, topic=topic, received_ros_ns=ros_ns,
                   received_monotonic_ns=time.monotonic_ns() if host_ns is None else host_ns,
                   message=data)
        line = json.dumps(row, ensure_ascii=False, allow_nan=False, separators=(",", ":")) + "\n"
        size = len(line.encode("utf-8"))
        if self.bytes_written + size > self.max_bytes:
            self.error = "log_size_limit_reached"
            return False
        try:
            self.stream.write(line)
        except OSError as exc:
            self.error = f"log_write_failed: {exc}"
            return False
        self.bytes_written += size
        self.written[topic] += 1
        if isinstance(data, dict):
            if type(data.get("valid")) is bool:
                self.valid.setdefault(topic, Counter())[str(data["valid"]).lower()] += 1
            if topic == "/fire_control/decision":
                self.decisions[str(data.get("reason", "missing_reason"))] += 1
            sequence = data.get("event_sequence")
            if type(sequence) is int:
                previous = self.source_sequences.get(topic, 0)
                if sequence > previous:
                    self.source_gaps[topic] += sequence - previous - 1
                self.source_sequences[topic] = sequence
        return True

    def checkpoint(self):
        self.stream.flush()
        summary = dict(
            schema=1, started_unix_ns=self.started_ns,
            started_monotonic_ns=self.started_monotonic_ns,
            checkpoint_unix_ns=time.time_ns(), closed=self.closed,
            log_error=self.error, bytes_written=self.bytes_written,
            max_bytes=self.max_bytes, received_counts=dict(self.received),
            written_counts=dict(self.written),
            dropped_counts={k: n - self.written[k] for k, n in self.received.items()
                            if n != self.written[k]},
            valid_counts={k: dict(v) for k, v in self.valid.items()},
            fire_decision_counts=dict(self.decisions),
            diagnostic_sequence_gaps=dict(self.source_gaps),
            effective_parameters=json_safe(self.parameters),
            missing_parameter_nodes=[n for n in self.expected_nodes if n not in self.parameters],
            topic_types=self.topic_types,
            missing_topics=[t for t in self.topic_types if not self.received[t]],
            notes=[
                "ROS source stamps remain inside each message; receive timestamps are separate.",
                "MCU timestamps are not synchronized to ROS time.",
                "host_write_returned is host-side evidence, not an MCU acknowledgement.",
                "Sensor subscriptions use best effort; counts describe received observations.",
                "Nonfinite numeric values are strings (nan, inf, -inf).",
            ])
        pending = self.directory / "summary.json.tmp"
        pending.write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n",
                           encoding="utf-8")
        pending.replace(self.directory / "summary.json")

    def close(self):
        if self.closed:
            return
        self.closed = True
        try:
            self.checkpoint()
        finally:
            self.stream.close()
