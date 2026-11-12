#!/usr/bin/env python3
"""Passively inspect the supplied 38-byte STM32 USB CDC feedback format."""

from __future__ import annotations

import argparse
from datetime import datetime
import json
import math
from pathlib import Path
import struct
import time


FRAME_SIZE = 38
PAYLOAD = struct.Struct("<II6fB")


def crc16(data: bytes) -> int:
    """RoboMaster CRC16: initial 0xffff, reflected 0x1021, no final XOR."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ (0x8408 if crc & 1 else 0)
    return crc


class FeedbackParser:
    def __init__(self) -> None:
        self.buffer = bytearray()
        self.crc_errors = 0
        self.tail_errors = 0
        self.discarded_bytes = 0

    def feed(self, data: bytes) -> list[dict]:
        self.buffer.extend(data)
        frames = []
        while self.buffer:
            start = self.buffer.find(b"\xb5")
            if start < 0:
                self.discarded_bytes += len(self.buffer)
                self.buffer.clear()
                break
            self.discarded_bytes += start
            del self.buffer[:start]
            if len(self.buffer) < FRAME_SIZE:
                break
            frame = bytes(self.buffer[:FRAME_SIZE])
            if frame[36:] != b"\x5b\x5a":
                self.tail_errors += 1
            elif crc16(frame[:34]) != struct.unpack_from("<H", frame, 34)[0]:
                self.crc_errors += 1
            else:
                seq, stamp_ms, qw, qx, qy, qz, yaw, pitch, mode = PAYLOAD.unpack(frame[1:34])
                quaternion = [qw, qx, qy, qz]
                norm = math.sqrt(sum(value * value for value in quaternion))
                warnings = []
                if not all(math.isfinite(value) for value in quaternion + [yaw, pitch]):
                    warnings.append("nonfinite_pose")
                if not math.isfinite(norm) or abs(norm - 1.0) > 0.05:
                    warnings.append("quaternion_not_unit")
                frames.append({
                    "sequence": seq,
                    "mcu_timestamp_ms": stamp_ms,
                    "quaternion_wxyz": [value if math.isfinite(value) else None
                                        for value in quaternion],
                    "quaternion_norm": norm if math.isfinite(norm) else None,
                    "yaw_rad": yaw if math.isfinite(yaw) else None,
                    "pitch_rad": pitch if math.isfinite(pitch) else None,
                    "yaw_deg": math.degrees(yaw) if math.isfinite(yaw) else None,
                    "pitch_deg": math.degrees(pitch) if math.isfinite(pitch) else None,
                    "mode": mode,
                    "warnings": warnings,
                })
                del self.buffer[:FRAME_SIZE]
                continue
            self.discarded_bytes += 1
            del self.buffer[:1]
        return frames


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--duration", type=float, default=10.0, help="Observation time in seconds")
    parser.add_argument("--output-prefix", type=Path,
                        default=Path("/tmp") / f"rm_handeye_feedback_{datetime.now():%Y%m%d_%H%M%S}")
    args = parser.parse_args()
    if not math.isfinite(args.duration) or args.duration <= 0 or args.baud <= 0:
        parser.error("duration and baud must be positive and finite")

    import serial

    decoder = FeedbackParser()
    received_bytes = valid_frames = usable_frames = 0
    first_frame_ns = last_frame_ns = None
    last_frame = None
    raw_path = Path(str(args.output_prefix) + ".bin")
    json_path = Path(str(args.output_prefix) + ".jsonl")
    args.output_prefix.parent.mkdir(parents=True, exist_ok=True)
    print(f"Receive only: {args.port}, {args.baud}, {args.duration:g}s", flush=True)
    try:
        with serial.Serial(args.port, baudrate=args.baud, timeout=0.1, exclusive=True) as port:
            with raw_path.open("xb") as raw, json_path.open("x") as log:
                started = time.monotonic()
                deadline = started + args.duration
                while time.monotonic() < deadline:
                    data = port.read(min(max(port.in_waiting, 1), 4096))
                    if not data:
                        continue
                    host_monotonic_ns = time.monotonic_ns()
                    host_unix_ns = time.time_ns()
                    raw.write(data)
                    received_bytes += len(data)
                    for frame in decoder.feed(data):
                        frame.update(host_receive_unix_ns=host_unix_ns,
                                     host_receive_monotonic_ns=host_monotonic_ns)
                        valid_frames += 1
                        usable_frames += not frame["warnings"]
                        if first_frame_ns is None:
                            first_frame_ns = host_monotonic_ns
                        last_frame_ns = host_monotonic_ns
                        last_frame = frame
                        log.write(json.dumps(frame, allow_nan=False) + "\n")
                        if valid_frames <= 3:
                            print(json.dumps(frame, allow_nan=False), flush=True)
                elapsed = time.monotonic() - started
    except (serial.SerialException, OSError) as exc:
        print(json.dumps({"status": "serial_error", "error": str(exc)}))
        return 2

    rate = None
    if first_frame_ns is not None and last_frame_ns > first_frame_ns:
        rate = (valid_frames - 1) * 1e9 / (last_frame_ns - first_frame_ns)
    print(json.dumps({
        "status": "feedback_received" if valid_frames else "no_valid_feedback",
        "observation_seconds": elapsed,
        "received_bytes": received_bytes,
        "valid_crc_frames": valid_frames,
        "finite_unit_quaternion_frames": usable_frames,
        "host_arrival_rate_hz": rate,
        "crc_errors": decoder.crc_errors,
        "tail_errors": decoder.tail_errors,
        "discarded_bytes": decoder.discarded_bytes,
        "pending_bytes": len(decoder.buffer),
        "last_frame": last_frame,
        "raw_log": str(raw_path),
        "decoded_log": str(json_path),
        "transmitted_protocol_bytes": 0,
    }, indent=2, allow_nan=False))
    return 0 if usable_frames else 1


if __name__ == "__main__":
    raise SystemExit(main())
