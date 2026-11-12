"""Opt-in Windows serial transport with acknowledged, expiring writes."""

from __future__ import annotations

import base64
from collections import deque
from dataclasses import dataclass
import os
from pathlib import Path
import select
import struct
import time

from windows_serial import WindowsSerialReader


HEADER = struct.Struct("<BI")
ACK = struct.Struct("<IB")
SYNC = struct.Struct("<Iq")
CLOCK = struct.Struct("<q")
RX_TIMING = struct.Struct("<qq")
# Keep a short, bounded window for late replies. A discarded reply can never
# complete a newer request, even after its individual bookkeeping is retired.
EXPIRED_ACK_TTL = 2.0
MAX_EXPIRED_ACKS = 256


@dataclass(frozen=True)
class WriteResult:
    request_id: int
    written: int
    error: Exception | None = None


@dataclass(frozen=True)
class _QueuedWrite:
    request_id: int
    data: bytes
    deadline: float


@dataclass(frozen=True)
class _InFlightWrite:
    request_id: int
    sequence: int
    size: int
    deadline: float


def helper_script(port: str, baud: int, *, trace: bool = False) -> str:
    source = Path(__file__).with_name("windows_duplex_helper.cs").read_text()
    return (
        '$ErrorActionPreference = "Stop"\n'
        "Add-Type -TypeDefinition @'\n" + source + "\n'@\n"
        f'[DuplexPipe]::Run([ComEndpoint]::new("{port}", {baud})'
        + (", $true)\n" if trace else ")\n")
    )


class WindowsSerialDuplex(WindowsSerialReader):
    """Duplex pipe with a latest-only asynchronous write queue."""

    def __init__(self, port: str, baud: int, trace_callback=None, **kwargs):
        self.trace_callback = trace_callback
        self.frames = bytearray()
        self.acks = {}
        self.sequence = 0
        self.request_id = 0
        self.issued_sequences = set()
        self.expired_sequences = {}
        self.retired_sequence = 0
        self.pending_write = None
        self.inflight_write = None
        self.write_results = deque()
        self.sync_reply = None
        self.sync_sequence = None
        self.sync_windows_ms = None
        self.sync_local_at = None
        startup_timeout = float(kwargs.get("startup_timeout", 10.0))
        factory = helper_script if trace_callback is None else (
            lambda name, rate: helper_script(name, rate, trace=True))
        super().__init__(port, baud, script_factory=factory, **kwargs)
        try:
            self._synchronize(min(startup_timeout, 1.0))
        except Exception:
            self.close()
            raise

    def _pump(self) -> None:
        # The common opener consumes READY; subsequent stdout is framed binary.
        if not getattr(self, "ready", False):
            return super()._pump()
        pending = self.buffer
        self.buffer = bytearray()
        try:
            super()._pump()
            self.frames.extend(self.buffer)
        finally:
            self.buffer = pending
        self._decode_frames()

    def _decode_frames(self) -> None:
        while len(self.frames) >= HEADER.size:
            kind, size = HEADER.unpack_from(self.frames)
            if size > 65536:
                raise OSError("Invalid Windows duplex frame length")
            end = HEADER.size + size
            if len(self.frames) < end:
                return
            payload = bytes(self.frames[HEADER.size:end])
            del self.frames[:end]
            if kind == 1:
                self.buffer.extend(payload)
            elif kind == 5 and size > RX_TIMING.size:
                start_us, end_us = RX_TIMING.unpack_from(payload)
                data = payload[RX_TIMING.size:]
                if self.trace_callback is not None:
                    self.trace_callback(dict(kind="rx", windows_read_start_us=start_us,
                                             windows_read_end_us=end_us, data=data))
                self.buffer.extend(data)
            elif kind == 2 and size == ACK.size:
                sequence, status = ACK.unpack(payload)
                if status not in (0, 1, 2, 3):
                    raise OSError("Invalid Windows write acknowledgement status")
                if sequence not in self.issued_sequences and 1 < sequence <= self.retired_sequence:
                    continue
                if sequence not in self.issued_sequences or sequence in self.acks:
                    raise OSError("Unexpected Windows write acknowledgement")
                self.acks[sequence] = status
            elif kind == 3 and size == SYNC.size:
                sequence, timestamp_ms = SYNC.unpack(payload)
                if sequence != self.sync_sequence or self.sync_reply is not None:
                    raise OSError("Unexpected Windows clock handshake")
                self.sync_reply = (timestamp_ms, time.monotonic())
            elif kind == 4 and size == CLOCK.size:
                self.sync_windows_ms = CLOCK.unpack(payload)[0]
                self.sync_local_at = time.monotonic()
                if self.trace_callback is not None:
                    self.trace_callback(dict(kind="clock", windows_ms=self.sync_windows_ms))
            else:
                raise OSError("Invalid Windows duplex frame")

    def _after_ready(self) -> None:
        self.frames.extend(self.buffer)
        self.buffer.clear()
        self.ready = True
        self._decode_frames()

    def _send_line(self, command: bytes, deadline: float) -> None:
        fd = self.process.stdin.fileno()
        os.set_blocking(fd, False)
        sent = 0
        while sent < len(command):
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise OSError("Windows command pipe timed out")
            _, writable, _ = select.select([], [fd], [], remaining)
            if writable:
                try:
                    sent += os.write(fd, command[sent:])
                except BlockingIOError:
                    pass

    def _wait_for(self, predicate, deadline: float, description: str) -> None:
        while not predicate():
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise OSError(f"Windows {description} timed out")
            readable, _, _ = select.select([self.process.stdout], [], [], remaining)
            if readable:
                self._pump()

    def _synchronize(self, timeout: float) -> None:
        self.sequence = (self.sequence + 1) & 0xFFFFFFFF
        self.sync_sequence = self.sequence
        self.sync_reply = None
        deadline = time.monotonic() + timeout
        self._send_line(f"SYNC {self.sync_sequence}\n".encode("ascii"), deadline)
        self._wait_for(lambda: self.sync_reply is not None, deadline, "clock handshake")
        self.sync_windows_ms, self.sync_local_at = self.sync_reply
        self.sync_sequence = None

    @staticmethod
    def _validate_write(data: bytes, timeout: float) -> None:
        if len(data) > 1024 or not 0 < timeout <= 1.0:
            raise ValueError("Windows writes require <=1024 bytes and 0<timeout<=1 second")

    def queue_write(self, data: bytes, timeout: float = 0.1) -> int:
        """Replace any unsent command and return without waiting for its ack."""
        if not data:
            return 0
        self._validate_write(data, timeout)
        if self.process is None:
            raise OSError("Windows duplex transport is closed")
        self.request_id += 1
        request = _QueuedWrite(
            request_id=self.request_id,
            data=bytes(data),
            deadline=time.monotonic() + timeout,
        )
        self.pending_write = request
        self.service_writes()
        return request.request_id

    def cancel_pending_write(self) -> None:
        self.pending_write = None

    def _complete_inflight(self, error: Exception | None = None) -> None:
        request = self.inflight_write
        if request is None:
            return
        self.write_results.append(
            WriteResult(request.request_id, request.size if error is None else 0, error)
        )
        self.issued_sequences.discard(request.sequence)
        self.inflight_write = None

    def _consume_acknowledgements(self) -> None:
        for sequence, status in list(self.acks.items()):
            request = self.inflight_write
            if request is not None and sequence == request.sequence:
                del self.acks[sequence]
                if status == 0:
                    self._complete_inflight()
                elif status == 1:
                    self._complete_inflight(TimeoutError("Windows rejected expired command"))
                elif status == 2:
                    self._complete_inflight(TimeoutError("Windows rejected future deadline"))
                elif status == 3:
                    self._complete_inflight(TimeoutError("Windows superseded queued command"))
                else:
                    raise OSError("Invalid Windows write acknowledgement status")
            elif sequence in self.expired_sequences:
                del self.acks[sequence]
                del self.expired_sequences[sequence]
                self.issued_sequences.discard(sequence)
            else:
                raise OSError("Unexpected Windows write acknowledgement")

    def _retire_expired_acknowledgements(self, now: float) -> None:
        while self.expired_sequences:
            sequence = next(iter(self.expired_sequences))
            if (self.expired_sequences[sequence] > now
                    and len(self.expired_sequences) <= MAX_EXPIRED_ACKS):
                break
            del self.expired_sequences[sequence]
            self.issued_sequences.discard(sequence)
            self.retired_sequence = max(self.retired_sequence, sequence)

    def service_writes(self) -> None:
        """Pump pipe traffic, finish acks, and dispatch at most one current write."""
        if self.process is None:
            raise OSError("Windows duplex transport is closed")
        self._pump()
        self._consume_acknowledgements()

        now = time.monotonic()
        if self.inflight_write is not None and now >= self.inflight_write.deadline:
            sequence = self.inflight_write.sequence
            self.expired_sequences[sequence] = now + EXPIRED_ACK_TTL
            self.write_results.append(WriteResult(
                self.inflight_write.request_id,
                0,
                TimeoutError("Windows serial write acknowledgement timed out"),
            ))
            self.inflight_write = None

        self._retire_expired_acknowledgements(now)

        if self.inflight_write is not None or self.pending_write is None:
            return

        request = self.pending_write
        now = time.monotonic()
        if now >= request.deadline:
            self.pending_write = None
            self.write_results.append(WriteResult(
                request.request_id, 0, TimeoutError("Command expired before Windows write")
            ))
            return

        if now - self.sync_local_at > 0.1:
            return
        self.pending_write = None
        # Use the latest Windows clock sample as an EARLIER anchor. Never
        # extrapolate it with WSL uptime: the two clock rates can diverge.
        # Sample transit/age shortens the budget rather than renewing it.
        expires_ms = self.sync_windows_ms + int((request.deadline - now) * 1000) - 2
        # Sequence IDs must not be reused on a connection: an ancient reply
        # must never match a new request after wrapping the uint32 counter.
        if self.sequence >= 0xFFFFFFFF:
            raise OSError("Windows request sequence exhausted; reconnect required")
        self.sequence += 1
        command = (
            f"{self.sequence} {expires_ms} "
            + base64.b64encode(request.data).decode("ascii")
            + "\n"
        ).encode("ascii")
        # A command line is tiny and there is only one in flight. Bound pipe
        # submission separately so a wedged helper cannot occupy the ROS loop.
        pipe_deadline = min(request.deadline, now + 0.005)
        self._send_line(command, pipe_deadline)
        self.issued_sequences.add(self.sequence)
        self.inflight_write = _InFlightWrite(
            request.request_id, self.sequence, len(request.data), request.deadline
        )

    def take_write_results(self) -> list[WriteResult]:
        results = list(self.write_results)
        self.write_results.clear()
        return results

    @property
    def in_waiting(self) -> int:
        self.service_writes()
        return len(self.buffer)

    def write(self, data: bytes, timeout: float = 0.1) -> int:
        """Synchronous compatibility API; the ROS node uses queue_write()."""
        if not data:
            return 0
        if self.pending_write is not None or self.inflight_write is not None:
            raise OSError("Another Windows write is already pending")
        request_id = self.queue_write(data, timeout)
        deadline = time.monotonic() + timeout
        while True:
            self.service_writes()
            for index, result in enumerate(self.write_results):
                if result.request_id != request_id:
                    continue
                del self.write_results[index]
                if result.error is not None:
                    raise result.error
                return result.written
            if time.monotonic() >= deadline:
                self.service_writes()
                continue
            time.sleep(0.001)
