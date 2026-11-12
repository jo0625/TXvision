"""Synthetic timing/protocol checks only; never opens hardware."""
import ctypes
import importlib.util
import json
from pathlib import Path
import subprocess
import sys

import pytest

from control_latency_protocol import ACK, AckParser, DiagnosticStream, WireIds, elapsed_us, encode_command, envelope
from control_latency_trace import ControlLatencyTrace
from protocol import CommandGate, encode_aim_command
from protocol import crc16
import struct

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location("control_analysis", ROOT / "tools/analyze_control_latency.py")
analysis = importlib.util.module_from_spec(spec)
spec.loader.exec_module(analysis)


def test_observing_pack_preserves_legacy_wire_and_gate_expiry():
    stamps = []
    gate = CommandGate()
    assert gate.update(True, .25, -.125, 10, timing=lambda a, b: stamps.append((a, b)))
    assert gate.packet(10.05, .1) == encode_aim_command(.25, -.125)
    assert len(gate.packet(10.05, .1)) == 11
    assert gate.packet(10.2, .1) is None
    assert len(stamps) == 1 and stamps[0][1] >= stamps[0][0]


def test_sequence_rollover_rotates_session_and_timer_wrap_is_unsigned():
    ids = WireIds(123)
    ids.sequence = 0xFFFFFFFF
    assert ids.next() == (123, 0xFFFFFFFF)
    assert ids.next() == (124, 0)
    assert elapsed_us(4, 0xFFFFFFFC) == 8
    assert elapsed_us(1, 100) is None


def test_mixed_stream_does_not_parse_diagnostic_magic_inside_telemetry():
    payload = b"\xb5" + struct.pack("<II6fB", 0x91D3, 1, 1., 0., 0., 0., .1, .2, 6)
    feedback = payload + struct.pack("<H", crc16(payload)) + b"\x5b\x5a"
    ack = envelope(2, ACK.pack(9, 12, 1, 10, 20, 100, 5, 1000, 90))
    for split in range(1, len(feedback + ack)):
        stream = DiagnosticStream()
        a, b = stream.feed((feedback + ack)[:split])
        c, d = stream.feed((feedback + ack)[split:])
        assert a + c == feedback and b + d == ack


def test_command_echo_with_embedded_headers_never_becomes_ack(tmp_path):
    payload = b"\xb5" + struct.pack("<II6fB", 1, 20, 1., 0., 0., 0., .1, .2, 6)
    feedback = payload + struct.pack("<H", crc16(payload)) + b"\x5b\x5a"
    echo = encode_command(0xB591D3, 1, encode_aim_command(.25, -.125))
    ack = envelope(2, ACK.pack(0xB591D3, 1, 1, 100, 120, 200, 1, 1000, 195))
    wire = echo + feedback + ack
    for split in range(1, len(wire)):
        stream = DiagnosticStream()
        a, b = stream.feed(wire[:split])
        c, d = stream.feed(wire[split:])
        assert a + c == feedback and b + d == ack
        assert stream.command_echoes == 1 and not stream.buffer
        assert stream.invalid_frames == 0
    trace = ControlLatencyTrace(str(tmp_path / "host.csv"), protocol="diagnostic_v1")
    trace.ids = WireIds(0xB591D3)
    trace.ids.sequence = 1
    marks = trace.begin_send()
    marks.update(call_before_ns=10, call_after_ns=20)
    trace.send_returned(marks, "host_write_returned", submitted=True)
    assert trace.receive(echo, 30) == b""
    assert len(trace.pending) == 1 and trace.counts["mcu_applied_ack"] == 0
    trace.receive(ack, 40)
    assert not trace.pending and trace.counts["mcu_applied_ack"] == 1
    trace.close()
    assert json.loads(trace.path.with_suffix(".metadata.json").read_text())[
        "diagnostic_command_echo_frames"] == 1


@pytest.mark.parametrize("split", range(1, 45))
def test_ack_crc_split_and_resynchronization(split):
    wire = envelope(2, ACK.pack(9, 12, 1, 0xFFFFFFF0, 0xFFFFFFF8, 0, 100, 1000, 0))
    bad = bytearray(wire)
    bad[-1] ^= 1
    parser = AckParser()
    assert parser.feed(b"telemetry" + bad + wire[:split]) == []
    decoded = parser.feed(wire[split:])
    assert len(decoded) == 1 and decoded[0]["adopted_us"] == 0
    assert parser.invalid_frames == 1


def test_logging_is_bounded_and_timeout_is_not_proven_nonadoption(tmp_path):
    trace = ControlLatencyTrace(str(tmp_path / "host.csv"), capacity=3, protocol="diagnostic_v1")
    marks = trace.begin_send()
    marks.update(call_before_ns=10, call_after_ns=20)
    trace.send_returned(marks, "host_write_returned", submitted=True)
    trace.expire(2_000_000_000)
    assert trace.counts["mcu_ack_timeout_adoption_unknown"] == 1
    ack = envelope(2, ACK.pack(marks["wire_session"], marks["wire_sequence"], 1, 100, 105,
                               200, 1, 1000, 195))
    trace.feed_ack(ack, 2_100_000_000)
    trace.emit("extra_record")
    assert trace.counts["log_records_dropped"] == 1
    assert not trace.path.exists()  # No callback wrote to disk.
    trace.close()
    assert json.loads(trace.path.with_suffix(".metadata.json").read_text())["counts"][
        "log_records_dropped"] == 1


def test_analyzer_correlates_100_synthetic_commands_without_cross_clock_subtraction():
    host, pnp = [], []
    for i in range(100):
        start = 10_000_000_000 + i * 50_000_000
        pnp.extend([dict(event="image_received", source_stamp_ns=str(i), host_ns=str(start)),
                    dict(event="pnp_return", source_stamp_ns=str(i), host_ns=str(start + 1_000_000)),
                    dict(event="frame_result", source_stamp_ns=str(i), host_ns=str(start + 2_000_000),
                         armors_count="1")])
        host.extend([dict(event="send", send_id=str(i), source_id=str(i), source_stamp_ns=str(i),
                          wire_session="9", wire_sequence=str(i), status="host_write_returned",
                          pack_start_ns=str(start + 3_000_000), pack_end_ns=str(start + 3_010_000),
                          call_before_ns=str(start + 4_000_000), call_after_ns=str(start + 4_020_000)),
                     dict(event="mcu_applied_ack", host_ns=str(start + 14_000_000),
                          wire_session="9", wire_sequence=str(i), mcu_status="1",
                          rx_complete_us=str(0xFFFFFFF0), parsed_us=str(0xFFFFFFF8), adopted_us="0",
                          control_cycle=str(i * 50), control_period_us="1000", control_tick_us="0")])
    meta = dict(boot_id="synthetic", time_namespace="synthetic-time-ns", clock="time.monotonic_ns")
    commands, summary = analysis.summarize(host, meta, pnp, meta)
    assert summary["fully_linked_applied_commands"] == 100
    assert summary["metrics"]["pnp_to_submit_ms"]["median_ms"] == 3
    assert summary["metrics"]["round_trip_ack_ms"]["median_ms"] == 10
    assert summary["metrics"]["mcu_rx_to_adopt_ms"]["median_ms"] == .016
    assert summary["actual_rates"]["host_send_interface_submissions_hz"] == 20
    assert len(commands) == 100
    _, separate_clocks = analysis.summarize(host, meta, pnp, dict(meta, boot_id="other-host"))
    assert separate_clocks["metrics"]["pnp_to_submit_ms"]["n"] == 0
    for missing in ("boot_id", "time_namespace"):
        incomplete_meta = {key: value for key, value in meta.items() if key != missing}
        _, missing_identity = analysis.summarize(host, incomplete_meta, pnp, incomplete_meta)
        assert not missing_identity["same_host_monotonic_clock_verified"]
        assert missing_identity["metrics"]["pnp_to_submit_ms"]["n"] == 0
    _, dropped = analysis.summarize(host, dict(meta, counts={"send": 100,
        "mcu_ack_timeout_adoption_unknown": 7, "log_records_dropped": 7}), pnp, meta)
    assert dropped["status"] == "at_least_100_linked_applied_with_quality_issues"
    assert dropped["event_counts"]["mcu_ack_timeout_adoption_unknown"] == 7
    assert dropped["stored_event_counts"].get("mcu_ack_timeout_adoption_unknown", 0) == 0


def test_late_ack_before_periodic_expiry_still_counts_timeout_once(tmp_path):
    trace = ControlLatencyTrace(str(tmp_path / "host.csv"), protocol="diagnostic_v1")
    marks = trace.begin_send()
    marks.update(call_before_ns=10, call_after_ns=20)
    trace.send_returned(marks, "host_write_returned", submitted=True)
    ack = envelope(2, ACK.pack(marks["wire_session"], marks["wire_sequence"], 1,
                               100, 105, 200, 1, 1000, 195))
    trace.feed_ack(ack, 1_000_000_010)
    trace.expire(2_000_000_000)
    assert trace.counts["mcu_ack_timeout_adoption_unknown"] == 1
    assert trace.counts["mcu_applied_ack"] == 1
    assert not trace.pending


@pytest.mark.parametrize("rx,parsed,adopted", [(100, 50, 200), (100, 200, 150),
                                              (0, 0x70000000, 0xE0000000)])
def test_invalid_mcu_event_order_is_preserved_but_not_a_valid_interval(rx, parsed, adopted):
    host = [dict(event="send", send_id="1", status="host_write_returned",
                 call_before_ns="1000", call_after_ns="2000", wire_session="3", wire_sequence="1"),
            dict(event="mcu_applied_ack", host_ns="10000", wire_session="3", wire_sequence="1",
                 mcu_status="1", rx_complete_us=str(rx), parsed_us=str(parsed),
                 adopted_us=str(adopted), control_cycle="1", control_period_us="1000",
                 control_tick_us="0")]
    commands, result = analysis.summarize(host, {})
    assert commands[0]["mcu_status"] == "1"  # Raw MCU assertion is retained.
    assert not commands[0]["fully_linked_applied"]
    assert result["metrics"]["mcu_rx_to_adopt_ms"]["n"] == 0
    assert result["metrics"]["mcu_parse_to_adopt_ms"]["n"] == 0
    assert result["analysis_errors"]["invalid_mcu_adoption_order_or_ambiguous_wrap"] == 1


def test_nonadoption_and_missing_status_are_not_excluded():
    host = [dict(event="send", send_id="1", status="host_write_returned", host_ns="10",
                 call_before_ns="10", call_after_ns="20", wire_session="3", wire_sequence="1"),
            dict(event="mcu_nonadoption_ack", host_ns="100", wire_session="3", wire_sequence="1",
                 mcu_status="2", rx_complete_us="0", parsed_us="10", adopted_us="0",
                 control_cycle="0", control_period_us="0", control_tick_us="0"),
            dict(event="send", send_id="2", status="host_write_returned", host_ns="200",
                 call_before_ns="200", call_after_ns="220")]
    commands, summary = analysis.summarize(host, {})
    assert len(commands) == 2 and summary["mcu_status_counts"] == {"2": 1}
    assert summary["commands_without_correlated_mcu_status"] == 1
    assert summary["metrics"]["mcu_rx_to_adopt_ms"]["n"] == 0


def test_rate_keeps_batch_delivered_samples_and_packing_counts_real_calls_once():
    assert analysis.rate([0, 0, 1_000_000_000]) == 2
    rows = [dict(event="source_packed", source_id=str(i), pack_start_ns=str(i * 100),
                 pack_end_ns=str(i * 100 + 10)) for i in (1, 2)]
    rows += [dict(event="send", source_id="1", status="host_write_returned",
                  call_before_ns=str(i * 1000), call_after_ns=str(i * 1000 + 10),
                  pack_start_ns="100", pack_end_ns="110") for i in (1, 2, 3)]
    _, result = analysis.summarize(rows, {})
    assert result["metrics"]["pack_ms"]["n"] == 2
    assert result["metrics"]["send_call_ms"]["n"] == 3


def test_c_mcu_codec_ring_and_python_codec_agree(tmp_path):
    library = tmp_path / "mcu_latency.so"
    subprocess.run(["cc", "-std=c99", "-Wall", "-Wextra", "-Werror", "-pedantic", "-fPIC", "-shared",
                    str(ROOT / "tools/control_latency/mcu_latency_hooks.c"), "-o", str(library)], check=True)
    lib = ctypes.CDLL(str(library))
    enter_type = ctypes.CFUNCTYPE(ctypes.c_uint32)
    leave_type = ctypes.CFUNCTYPE(None, ctypes.c_uint32)
    enter, leave = enter_type(lambda: 0), leave_type(lambda saved: None)
    lib.latency_init.argtypes = [enter_type, leave_type]
    assert lib.latency_init(enter, leave) == 1
    class Tag(ctypes.Structure):
        _fields_ = [("session", ctypes.c_uint64), ("sequence", ctypes.c_uint32),
                    ("rx_us", ctypes.c_uint32), ("parsed_us", ctypes.c_uint32)]
    lib.latency_decode_command.argtypes = [ctypes.c_void_p, ctypes.c_size_t, ctypes.c_uint32,
                                           ctypes.POINTER(Tag), ctypes.c_void_p]
    lib.latency_parsed.argtypes = [ctypes.POINTER(Tag), ctypes.c_uint32]
    lib.latency_record.argtypes = [ctypes.POINTER(Tag), ctypes.c_uint8, ctypes.c_uint32,
                                   ctypes.c_uint32, ctypes.c_uint32, ctypes.c_uint32]
    wire = encode_command(0x123456789ABCDEF0, 0xFFFFFFFF, encode_aim_command(.25, -.125))
    tag, legacy = Tag(), ctypes.create_string_buffer(11)
    assert lib.latency_decode_command(wire, len(wire), 0xFFFFFFF0, ctypes.byref(tag), legacy) == 1
    assert legacy.raw == encode_aim_command(.25, -.125)
    lib.latency_parsed(ctypes.byref(tag), 0xFFFFFFF8)
    for _ in range(66):
        lib.latency_record(ctypes.byref(tag), 1, 0, 25, 1000, 0xFFFFFFFC)
    assert lib.latency_dropped_records() == 2
    output = ctypes.create_string_buffer(45)
    assert lib.latency_peek_ack(output) == 1
    ack = AckParser().feed(output.raw)[0]
    assert ack["wire_session"] == tag.session
    assert ack["wire_sequence"] == 0xFFFFFFFF and ack["parsed_us"] == 0xFFFFFFF8
    assert ack["adopted_us"] == 0 and ack["control_tick_us"] == 0xFFFFFFFC
    for _ in range(64):
        assert lib.latency_peek_ack(output) == 1
        lib.latency_pop_ack()
    assert lib.latency_peek_ack(output) == 0
    assert lib.latency_decode_command(wire[:-1] + b"\0", len(wire), 0, ctypes.byref(tag), legacy) == 0
    assert lib.latency_invalid_commands() == 1
