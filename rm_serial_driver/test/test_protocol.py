import math
import struct

import pytest

from protocol import CommandGate, FeedbackParser, crc16, encode_aim_command


def make_feedback(seq=7, stamp_ms=12345, quaternion=(1.0, 0.0, 0.0, 0.0),
                  yaw=0.25, pitch=-0.125, mode=2):
    body = b"\xb5" + struct.pack(
        "<II6fB", seq, stamp_ms, *quaternion, yaw, pitch, mode
    )
    return body + struct.pack("<H", crc16(body)) + b"\x5b\x5a"


def test_crc_known_vector():
    assert crc16(b"123456789") == 0x6F91


def test_feedback_layout_and_units():
    wire = make_feedback()
    assert len(wire) == 38
    frame = FeedbackParser().feed(wire)[0]
    assert frame.sequence == 7
    assert frame.mcu_timestamp_ms == 12345
    assert frame.quaternion_wxyz == (1.0, 0.0, 0.0, 0.0)
    assert frame.yaw == pytest.approx(0.25)
    assert frame.pitch == pytest.approx(-0.125)
    assert frame.mode == 2
    assert frame.quaternion_is_unit


@pytest.mark.parametrize("split", range(1, 38))
def test_feedback_all_split_points(split):
    wire = make_feedback()
    parser = FeedbackParser()
    assert parser.feed(wire[:split]) == []
    assert len(parser.feed(wire[split:])) == 1


def test_parser_resynchronizes_after_noise_crc_and_tail_errors():
    bad_crc = bytearray(make_feedback(seq=1))
    bad_crc[1] ^= 1
    bad_tail = make_feedback(seq=2)[:-1] + b"\x00"
    parser = FeedbackParser()
    frames = parser.feed(b"noise" + bad_crc + bad_tail + make_feedback(seq=3))
    assert [frame.sequence for frame in frames] == [3]
    assert parser.crc_errors == 1
    assert parser.tail_errors == 1
    assert parser.discarded_bytes >= 7


def test_invalid_quaternion_is_retained_but_not_usable_for_imu():
    frame = FeedbackParser().feed(make_feedback(quaternion=(0, 0, 0, 0)))[0]
    assert frame.quaternion_norm == 0
    assert not frame.quaternion_is_unit


def test_encode_aim_command_exact_layout():
    wire = encode_aim_command(0.25, -0.125)
    assert len(wire) == 11
    assert wire[0] == 0xA5
    assert wire[-2:] == b"\x5a\x5b"
    assert struct.unpack("<ff", wire[1:9]) == pytest.approx((0.25, -0.125))


@pytest.mark.parametrize("value", [math.nan, math.inf, -math.inf])
def test_encode_rejects_nonfinite_angles(value):
    with pytest.raises(ValueError):
        encode_aim_command(value, 0.0)
    with pytest.raises(ValueError):
        encode_aim_command(0.0, value)


def test_command_gate_stops_invalid_and_stale_commands():
    gate = CommandGate()
    assert gate.update(True, 0.2, -0.1, 10.0)
    assert gate.packet(10.05, 0.1) is not None
    assert gate.packet(10.11, 0.1) is None
    assert gate.update(True, 0.2, -0.1, 11.0)
    assert not gate.update(False, 0.2, -0.1, 11.01)
    assert gate.packet(11.01, 0.1) is None


def test_command_gate_rejects_nonfinite_valid_command():
    gate = CommandGate()
    assert not gate.update(True, math.nan, 0.0, 10.0)
    assert gate.packet(10.0, 0.1) is None


def test_remaining_time_does_not_renew_command_lifetime():
    gate = CommandGate()
    assert gate.remaining_time(10.0, 0.1) == 0.0
    gate.update(True, 0.2, -0.1, 10.0)
    assert gate.remaining_time(10.08, 0.1) == pytest.approx(0.02)
    assert gate.remaining_time(10.11, 0.1) == 0.0
    assert gate.packet(10.11, 0.1) is None
