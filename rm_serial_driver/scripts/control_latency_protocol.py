"""Opt-in diagnostic envelope; the legacy 11-byte command stays unchanged.

This codec alone does not establish that a controller supports the envelope.
Enable only after integrating the matching MCU hooks into the flashed build.
"""
from __future__ import annotations

import struct

from protocol import crc16

MAGIC = b"\xd3\x91"
HEADER = struct.Struct("<2sBBH")
COMMAND = struct.Struct("<QI11s")
ACK = struct.Struct("<QIBIIIIII")
APPLIED, SUPERSEDED, REJECTED, NOT_ADOPTED = 1, 2, 3, 4


def envelope(kind: int, payload: bytes) -> bytes:
    body = HEADER.pack(MAGIC, 1, kind, len(payload)) + payload
    return body + struct.pack("<H", crc16(body))


def encode_command(session: int, sequence: int, legacy: bytes) -> bytes:
    if len(legacy) != 11 or legacy[0] != 0xA5 or legacy[-2:] != b"\x5a\x5b":
        raise ValueError("diagnostic command must contain one legacy 11-byte command")
    return envelope(1, COMMAND.pack(session, sequence, legacy))


def elapsed_us(later: int, earlier: int) -> int | None:
    """Unsigned MCU subtraction; reject ambiguous spans >= half the 32-bit range."""
    delta = (later - earlier) & 0xFFFFFFFF
    return delta if delta < 0x80000000 else None


class WireIds:
    def __init__(self, session: int):
        self.session, self.sequence = session, 0

    def next(self) -> tuple[int, int]:
        result = self.session, self.sequence
        if self.sequence == 0xFFFFFFFF:
            self.session = (self.session + 1) & 0xFFFFFFFFFFFFFFFF
            self.sequence = 0
        else:
            self.sequence += 1
        return result


class AckParser:
    def __init__(self):
        self.buffer = bytearray()
        self.invalid_frames = 0

    def feed(self, data: bytes) -> list[dict]:
        self.buffer.extend(data)
        output = []
        while len(self.buffer) >= HEADER.size:
            index = self.buffer.find(MAGIC)
            if index < 0:
                self.buffer[:] = self.buffer[-1:]
                break
            del self.buffer[:index]
            if len(self.buffer) < HEADER.size:
                break
            _, version, kind, size = HEADER.unpack_from(self.buffer)
            if version != 1 or kind != 2 or size != ACK.size:
                del self.buffer[:1]
                self.invalid_frames += 1
                continue
            end = HEADER.size + size + 2
            if len(self.buffer) < end:
                break
            wire = bytes(self.buffer[:end])
            if crc16(wire[:-2]) != struct.unpack_from("<H", wire, end - 2)[0]:
                del self.buffer[:1]
                self.invalid_frames += 1
                continue
            values = ACK.unpack(wire[HEADER.size:-2])
            if values[2] not in (APPLIED, SUPERSEDED, REJECTED, NOT_ADOPTED):
                self.invalid_frames += 1
                del self.buffer[:end]
                continue
            output.append(dict(zip(("wire_session", "wire_sequence", "mcu_status",
                                    "rx_complete_us", "parsed_us", "adopted_us",
                                    "control_cycle", "control_period_us", "control_tick_us"), values)))
            del self.buffer[:end]
        return output


class DiagnosticStream:
    """Demultiplex complete CRC-valid telemetry/ACK frames before either decoder.

    A diagnostic magic byte inside a valid telemetry payload is payload, and
    must never be mistaken for a second frame. Legacy mode does not use this.
    """
    def __init__(self):
        self.buffer = bytearray()
        self.invalid_frames = 0
        self.command_echoes = 0

    def feed(self, data: bytes) -> tuple[bytes, bytes]:
        self.buffer.extend(data)
        telemetry, acknowledgements = bytearray(), bytearray()
        while self.buffer:
            if self.buffer[0] == 0xB5:
                size, output = 38, telemetry
                if len(self.buffer) < size:
                    break
                valid = self.buffer[36:38] == b"\x5b\x5a" and crc16(self.buffer[:34]) == int.from_bytes(
                    self.buffer[34:36], "little")
            elif self.buffer[0] == MAGIC[0]:
                if len(self.buffer) < HEADER.size:
                    break
                magic, version, kind, length = HEADER.unpack_from(self.buffer)
                expected_size = {1: COMMAND.size, 2: ACK.size}.get(kind)
                if magic != MAGIC or version != 1 or length != expected_size:
                    del self.buffer[:1]
                    continue
                # The public USB task echoes incoming bytes. Consume a valid
                # command echo as one frame; it is never an adoption ACK, and
                # magic inside its session/target fields is not a new frame.
                size = HEADER.size + length + 2
                output = acknowledgements if kind == 2 else None
                if len(self.buffer) < size:
                    break
                valid = crc16(self.buffer[:size - 2]) == int.from_bytes(self.buffer[size - 2:size], "little")
            else:
                del self.buffer[:1]
                continue
            if not valid:
                self.invalid_frames += 1
                del self.buffer[:1]
                continue
            if output is None:
                self.command_echoes += 1
            else:
                output.extend(self.buffer[:size])
            del self.buffer[:size]
        return bytes(telemetry), bytes(acknowledgements)
