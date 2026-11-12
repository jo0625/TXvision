"""Exercise real bridge callbacks against memory endpoints; no DDS or hardware."""
import json
from pathlib import Path
from types import SimpleNamespace
import sys

from auto_aim_interfaces.msg import AimCommand

from control_latency_protocol import ACK, COMMAND, AckParser, envelope
from control_latency_trace import ControlLatencyTrace
from protocol import CommandGate, FeedbackParser
from serial_bridge_node import SerialBridgeNode

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))
sys.path.insert(0, str(ROOT / "yolo_armor_detector/scripts"))
from analyze_control_latency import main as analyze_main, summarize
from pnp_latency_trace import PnpLatencyTrace


def memory_node(trace):
    class Endpoint:
        def __init__(self):
            self.received = bytearray()
            self.writes = []

        def write(self, packet):
            self.writes.append(packet)
            if len(packet) == 31:
                session, sequence, legacy = COMMAND.unpack(packet[6:-2])
                self.received.extend(envelope(2, ACK.pack(session, sequence, 1, 100, 120,
                                                         200, sequence * 20, 1000, 195)))
            return len(packet)

        @property
        def in_waiting(self):
            return len(self.received)

        def read(self, count):
            data = bytes(self.received[:count])
            del self.received[:count]
            return data

    return SimpleNamespace(port=Endpoint(), transport="serial", receive_only=False,
                           latency_trace=trace, command_gate=CommandGate(), command_timeout=.1,
                           parser=FeedbackParser(), publish_state=lambda *args: None,
                           service_duplex_writes=lambda: None, open_serial=lambda now: None)


def test_actual_callbacks_legacy_still_emit_11_bytes(tmp_path):
    trace = ControlLatencyTrace(str(tmp_path / "legacy.csv"))
    node = memory_node(trace)
    message = AimCommand(valid=True, yaw=0., pitch=0.)
    SerialBridgeNode.command_callback(node, message)
    SerialBridgeNode.write_command(node)
    assert len(node.port.writes) == 1 and len(node.port.writes[0]) == 11
    send = next(row for row in trace.rows if row["event"] == "send")
    assert send["pack_start_ns"] <= send["pack_end_ns"] <= send["call_before_ns"] <= send["call_after_ns"]
    assert not trace.pending
    SerialBridgeNode.command_callback(node, AimCommand(valid=False))
    SerialBridgeNode.write_command(node)
    assert len(node.port.writes) == 1
    trace.close()


def test_120_synthetic_commands_through_real_callbacks_and_csv_analyzer(tmp_path, monkeypatch):
    trace = ControlLatencyTrace(str(tmp_path / "host.csv"), protocol="diagnostic_v1",
                                context=dict(data_kind="synthetic_memory_endpoint"))
    pnp = PnpLatencyTrace(str(tmp_path / "pnp.csv"))
    node = memory_node(trace)
    for i in range(120):
        pnp.source_stamp_ns = i + 1
        pnp.record("image_received")
        pnp.pnp_return(True)
        pnp.record("frame_result", armors_count=1)
        message = AimCommand(valid=True, yaw=0., pitch=0.)
        message.header.stamp.nanosec = i + 1
        SerialBridgeNode.command_callback(node, message)
        SerialBridgeNode.write_command(node)
        SerialBridgeNode.read_serial(node)
    assert not trace.pending
    # A source replaced before send remains explicitly recorded.
    SerialBridgeNode.command_callback(node, AimCommand(valid=True))
    SerialBridgeNode.command_callback(node, AimCommand(valid=True))
    assert trace.counts["source_overwritten_before_send"] == 1
    trace.close()
    pnp.close()
    monkeypatch.setattr(sys, "argv", ["analyze_control_latency.py", "--host", str(trace.path),
                                     "--pnp", str(pnp.path), "--output", str(tmp_path / "analysis")])
    analyze_main()
    result = json.loads((tmp_path / "analysis/summary.json").read_text())
    assert result["fully_linked_applied_commands"] == 120
    assert result["metrics"]["send_call_ms"]["n"] == 120
    assert result["metrics"]["pack_ms"]["n"] == 122
    assert result["event_counts"]["source_overwritten_before_send"] == 1
    assert result["commands_without_correlated_mcu_status"] == 0


def test_diagnostics_timestamp_brackets_native_write_without_changing_packet(monkeypatch):
    import serial_bridge_node as module
    node = memory_node(None)
    events = []
    node.tx_event_pub = SimpleNamespace(publish=lambda msg: events.append(json.loads(msg.data)))
    node.tx_event_sequence = 0
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=99))
    ticks = iter([1794400000123456789, 1794400000124456789])
    monkeypatch.setattr(module.time, "time_ns", lambda: next(ticks))
    message = AimCommand(valid=True, yaw=0.12, pitch=-0.03)
    SerialBridgeNode.command_callback(node, message)
    SerialBridgeNode.write_command(node)
    assert len(node.port.writes) == 1 and len(node.port.writes[0]) == 11
    assert len(events) == 1
    assert events[0]["write_started_unix_ns"] == 1794400000123456789
    assert events[0]["write_completed_unix_ns"] == 1794400000124456789
    assert events[0]["ros_stamp_ns"] == 99  # Epoch timestamps do not use ROS simulation time.
    assert events[0]["wire_hex"] == node.port.writes[0].hex()
