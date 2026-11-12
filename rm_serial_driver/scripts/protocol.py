"""Wire codec for the current vision-controller protocol."""

from __future__ import annotations

from dataclasses import dataclass
import math
import struct
import time


FEEDBACK_SIZE = 38
FEEDBACK_PAYLOAD = struct.Struct("<II6fB")
AIM_COMMAND = struct.Struct("<BffBB")


def crc16(data: bytes) -> int:
    """RoboMaster CRC16 with initial 0xffff and reflected polynomial 0x8408."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ (0x8408 if crc & 1 else 0)
    return crc


@dataclass(frozen=True)
class FeedbackFrame:
    sequence: int
    mcu_timestamp_ms: int
    quaternion_wxyz: tuple[float, float, float, float]
    yaw: float
    pitch: float
    mode: int

    @property
    def pose_is_finite(self) -> bool:
        return all(
            math.isfinite(value)
            for value in (*self.quaternion_wxyz, self.yaw, self.pitch)
        )

    @property
    def quaternion_norm(self) -> float:
        return math.sqrt(sum(value * value for value in self.quaternion_wxyz))

    @property
    def quaternion_is_unit(self) -> bool:
        return self.pose_is_finite and abs(self.quaternion_norm - 1.0) <= 0.05


class FeedbackParser:
    def __init__(self) -> None:
        self.buffer = bytearray()
        self.crc_errors = 0
        self.tail_errors = 0
        self.discarded_bytes = 0

    def feed(self, data: bytes) -> list[FeedbackFrame]:
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
            if len(self.buffer) < FEEDBACK_SIZE:
                break

            wire = bytes(self.buffer[:FEEDBACK_SIZE])
            if wire[36:] != b"\x5b\x5a":
                self.tail_errors += 1
            elif crc16(wire[:34]) != struct.unpack_from("<H", wire, 34)[0]:
                self.crc_errors += 1
            else:
                values = FEEDBACK_PAYLOAD.unpack(wire[1:34])
                frames.append(
                    FeedbackFrame(
                        sequence=values[0],
                        mcu_timestamp_ms=values[1],
                        quaternion_wxyz=tuple(values[2:6]),
                        yaw=values[6],
                        pitch=values[7],
                        mode=values[8],
                    )
                )
                del self.buffer[:FEEDBACK_SIZE]
                continue

            self.discarded_bytes += 1
            del self.buffer[:1]
        return frames


def encode_aim_command(yaw: float, pitch: float, *, timing=None) -> bytes:
    if not math.isfinite(yaw) or not math.isfinite(pitch):
        raise ValueError("yaw and pitch must be finite")
    begin = time.monotonic_ns() if timing is not None else 0
    packet = AIM_COMMAND.pack(0xA5, yaw, pitch, 0x5A, 0x5B)
    if timing is not None:
        end = time.monotonic_ns()
        timing(begin, end)
    return packet


class CommandGate:
    """Keeps invalid or stale aim commands off the wire."""

    def __init__(self) -> None:
        self._packet: bytes | None = None
        self._updated_at = 0.0

    def update(self, valid: bool, yaw: float, pitch: float, now: float, *, timing=None) -> bool:
        if not valid:
            self.invalidate()
            return False
        try:
            packet = encode_aim_command(yaw, pitch, timing=timing)
        except ValueError:
            self.invalidate()
            return False
        self._packet = packet
        self._updated_at = now
        return True

    def invalidate(self) -> None:
        self._packet = None
        self._updated_at = 0.0

    def packet(self, now: float, timeout: float) -> bytes | None:
        if self._packet is None or now - self._updated_at > timeout:
            self.invalidate()
            return None
        return self._packet

    def remaining_time(self, now: float, timeout: float) -> float:
        if self.packet(now, timeout) is None:
            return 0.0
        return max(0.0, timeout - (now - self._updated_at))
