"""Bounded, opt-in host timing. No file I/O in receive/send callbacks."""
from __future__ import annotations

from collections import Counter, OrderedDict
import csv
import json
import os
from pathlib import Path
import time
import uuid

from control_latency_protocol import APPLIED, AckParser, DiagnosticStream, WireIds


class ControlLatencyTrace:
    def __init__(self, path: str, *, capacity: int = 200000, ack_timeout_s: float = 1.0,
                 protocol: str = "legacy", context: dict | None = None):
        if capacity <= 0 or ack_timeout_s <= 0 or protocol not in ("legacy", "diagnostic_v1"):
            raise ValueError("invalid latency tracing configuration")
        self.path = Path(path)
        if self.path.exists() or self.path.with_suffix(".metadata.json").exists():
            raise FileExistsError("use a new latency trace path")
        self.capacity = capacity
        self.ack_timeout_ns = int(ack_timeout_s * 1e9)
        self.protocol = protocol
        self.rows: list[dict] = []
        self.counts: Counter = Counter()
        self.trace_session = uuid.uuid4().hex
        self.ids = WireIds(uuid.uuid4().int & 0xFFFFFFFFFFFFFFFF)
        self.acks = AckParser()
        self.demux = DiagnosticStream()
        self.pending: OrderedDict[tuple[int, int], dict] = OrderedDict()
        self.source = None
        self.source_id = 0
        self.send_id = 0
        self.requests: dict[int, int] = {}
        self.closed = False
        self.started_ns = time.monotonic_ns()
        self.context = context or {}
        self.boot_id = Path("/proc/sys/kernel/random/boot_id").read_text().strip()

    def emit(self, event: str, *, host_ns: int | None = None, **fields) -> None:
        self.counts[event] += 1
        if len(self.rows) >= self.capacity:
            self.counts["log_records_dropped"] += 1
            return
        self.rows.append(dict(event=event, host_ns=host_ns if host_ns is not None
                              else time.monotonic_ns(), **fields))

    def new_source(self, message) -> None:
        if self.source is not None and not self.source["submitted"]:
            self.emit("source_overwritten_before_send", source_id=self.source["source_id"])
        self.source_id += 1
        self.source = dict(source_id=self.source_id,
                           source_stamp_ns=message.header.stamp.sec * 1_000_000_000
                           + message.header.stamp.nanosec, submitted=False,
                           frame_id=getattr(message.header, 'frame_id', ''),
                           yaw=message.yaw, pitch=message.pitch,
                           source_received_ns=time.monotonic_ns())
        self.emit("source_received", source_id=self.source_id,
                  source_stamp_ns=self.source["source_stamp_ns"], valid=bool(message.valid),
                  yaw=message.yaw, pitch=message.pitch)

    def packed(self, begin_ns: int, end_ns: int) -> None:
        if self.source is not None:
            self.source.update(pack_start_ns=begin_ns, pack_end_ns=end_ns)

    def source_result(self, accepted: bool, reason: str = "") -> None:
        if self.source is not None:
            self.source["queue_ready_ns"] = time.monotonic_ns()
            self.emit("source_packed" if accepted else "source_rejected", **self.source,
                      reason=reason)
            if not accepted:
                self.source = None

    def retire_source(self, reason: str) -> None:
        if self.source is not None:
            self.emit(reason, **self.source)
            self.source = None

    def begin_send(self) -> dict:
        self.send_id += 1
        fields = {"send_id": self.send_id}
        if self.source:
            fields.update(self.source)
            self.source["submitted"] = True
        if self.protocol == "diagnostic_v1":
            fields["wire_session"], fields["wire_sequence"] = self.ids.next()
        return fields

    def send_returned(self, fields: dict, status: str, *, submitted: bool) -> None:
        self.emit("send", **fields, status=status, ack_supported=self.protocol == "diagnostic_v1")
        if submitted and self.protocol == "diagnostic_v1":
            if len(self.pending) >= 4096:
                _, oldest = self.pending.popitem(last=False)
                self.emit("ack_tracking_overflow_unconfirmed", **oldest)
            self.pending[(fields["wire_session"], fields["wire_sequence"])] = fields

    def feed_ack(self, data: bytes, received_ns: int) -> None:
        if self.protocol != "diagnostic_v1":
            return
        for ack in self.acks.feed(data):
            key = ack["wire_session"], ack["wire_sequence"]
            sent = self.pending.get(key)
            # A late acknowledgement must still be associated before timeout expiry.
            if sent is None:
                self.emit("unmatched_or_duplicate_mcu_ack", host_ns=received_ns, **ack)
                continue
            del self.pending[key]
            # The receive callback may run before the periodic expiry callback.
            # Preserve a missed deadline even when the late ACK is correlated.
            if received_ns - sent["call_before_ns"] >= self.ack_timeout_ns:
                self.emit("mcu_ack_timeout_adoption_unknown", host_ns=received_ns, **sent)
            event = "mcu_applied_ack" if ack["mcu_status"] == APPLIED else "mcu_nonadoption_ack"
            self.emit(event, host_ns=received_ns, send_id=sent["send_id"],
                      call_before_ns=sent["call_before_ns"], **ack)

    def receive(self, data: bytes, received_ns: int) -> bytes:
        if self.protocol == "legacy":
            return data
        previous = self.demux.invalid_frames
        previous_echoes = self.demux.command_echoes
        telemetry, acknowledgements = self.demux.feed(data)
        if self.demux.invalid_frames != previous:
            self.emit("diagnostic_stream_invalid_frames", host_ns=received_ns,
                      invalid_frames=self.demux.invalid_frames - previous)
        if self.demux.command_echoes != previous_echoes:
            self.emit("diagnostic_command_echo_observed", host_ns=received_ns,
                      echo_frames=self.demux.command_echoes - previous_echoes)
        self.feed_ack(acknowledgements, received_ns)
        return telemetry

    def expire(self, now_ns: int) -> None:
        while self.pending:
            key, sent = next(iter(self.pending.items()))
            if now_ns - sent["call_before_ns"] < self.ack_timeout_ns:
                break
            del self.pending[key]
            self.emit("mcu_ack_timeout_adoption_unknown", host_ns=now_ns, **sent)

    def helper_result(self, result) -> None:
        self.emit("windows_host_write_result", request_id=result.request_id,
                  send_id=self.requests.pop(result.request_id, 0),
                  written=result.written, error="" if result.error is None else str(result.error))

    def helper_drop(self, request, reason: str) -> None:
        if request is not None:
            self.emit(reason, request_id=request.request_id,
                      send_id=self.requests.pop(request.request_id, 0))

    def disconnected(self) -> None:
        self.retire_source("source_disconnected")
        for sent in self.pending.values():
            self.emit("disconnect_adoption_unknown", **sent)
        self.pending.clear()
        for request_id, send_id in self.requests.items():
            self.emit("windows_disconnect_unconfirmed", request_id=request_id, send_id=send_id)
        self.requests.clear()

    def close(self) -> None:
        if self.closed:
            return
        self.closed = True
        self.expire(time.monotonic_ns())
        self.retire_source("source_at_capture_end")
        for sent in self.pending.values():
            self.emit("capture_end_adoption_unknown", **sent)
        for request_id, send_id in self.requests.items():
            self.emit("windows_capture_end_unconfirmed", request_id=request_id, send_id=send_id)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        fields = list(dict.fromkeys(key for row in self.rows for key in row)) or ["event", "host_ns"]
        with self.path.open("x", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            writer.writerows(self.rows)
        metadata = dict(schema=1, component="serial_bridge", trace_session=self.trace_session,
                        boot_id=self.boot_id, clock="time.monotonic_ns", unit="ns",
                        time_namespace=os.readlink("/proc/self/ns/time"),
                        clock_resolution_s=time.get_clock_info("monotonic").resolution,
                        started_ns=self.started_ns, stopped_ns=time.monotonic_ns(),
                        wire_protocol=self.protocol, counts=dict(self.counts), context=self.context,
                        ack_parser_invalid_frames=self.acks.invalid_frames,
                        diagnostic_stream_invalid_frames=self.demux.invalid_frames,
                        diagnostic_command_echo_frames=self.demux.command_echoes,
                        warning="Host write completion is not MCU arrival or adoption")
        with self.path.with_suffix(".metadata.json").open("x") as stream:
            json.dump(metadata, stream, ensure_ascii=False, indent=2)
