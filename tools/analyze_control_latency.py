#!/usr/bin/env python3
"""Offline latency baseline; never opens a serial port or sends motion commands."""
from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import csv
import json
from pathlib import Path
import statistics
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "rm_serial_driver/scripts"))
from control_latency_protocol import APPLIED, elapsed_us  # noqa: E402


def read_csv(path):
    with Path(path).open(newline="") as stream:
        return list(csv.DictReader(stream))


def number(row, key):
    return int(row[key]) if row.get(key, "") != "" else None


def stats(values):
    values = sorted(values)
    if not values:
        return dict(n=0, median_ms=None, p95_ms=None, max_ms=None)
    position = (len(values) - 1) * .95
    index = int(position)
    p95 = values[index] + (values[min(index + 1, len(values) - 1)] - values[index]) * (position - index)
    return dict(n=len(values), median_ms=statistics.median(values), p95_ms=p95, max_ms=max(values))


def rate(times, scale=1e9):
    times = sorted(times)
    return (len(times) - 1) * scale / (times[-1] - times[0]) if len(times) > 1 and times[-1] > times[0] else None


def summarize(host_rows, host_meta, pnp_rows=None, pnp_meta=None):
    same_clock = bool(pnp_meta and host_meta.get("boot_id") and host_meta.get("time_namespace")
                      and pnp_meta.get("boot_id") == host_meta.get("boot_id")
                      and pnp_meta.get("time_namespace") == host_meta.get("time_namespace")
                      and pnp_meta.get("clock") == host_meta.get("clock") == "time.monotonic_ns")
    pnp = defaultdict(list)
    image_counts = Counter()
    nonempty = set()
    for row in pnp_rows or []:
        stamp = number(row, "source_stamp_ns")
        if row["event"] == "image_received":
            image_counts[stamp] += 1
        if row["event"] == "frame_result" and (number(row, "armors_count") or 0) > 0:
            nonempty.add(stamp)
        if row["event"] == "pnp_return":
            pnp[stamp].append(number(row, "host_ns"))
    ack_by_wire = defaultdict(list)
    for row in host_rows:
        if row["event"] in ("mcu_applied_ack", "mcu_nonadoption_ack", "unmatched_or_duplicate_mcu_ack"):
            ack_by_wire[(row.get("wire_session"), row.get("wire_sequence"))].append(row)
    metrics = defaultdict(list)
    commands = []
    errors = Counter()
    for row in host_rows:
        if row["event"] != "send":
            continue
        derived = {key: row.get(key, "") for key in (
            "send_id", "source_id", "source_stamp_ns", "wire_session", "wire_sequence",
            "pack_start_ns", "pack_end_ns", "call_before_ns", "call_after_ns", "status")}
        derived["repeated_source_submission"] = row.get("submitted") == "True"
        begin, end = number(row, "call_before_ns"), number(row, "call_after_ns")
        submitted = row["status"] in ("host_write_returned", "host_queued")
        def interval(name, later, earlier, divisor, aggregate=True):
            if later is None or earlier is None:
                return
            value = (later - earlier) / divisor
            if value < 0:
                errors["negative_" + name] += 1
                return
            derived[name] = value
            if aggregate:
                metrics[name].append(value)
        interval("pack_ms", number(row, "pack_end_ns"), number(row, "pack_start_ns"), 1e6, False)
        interval("wire_envelope_pack_ms", number(row, "wire_pack_end_ns"),
                 number(row, "wire_pack_start_ns"), 1e6)
        interval("send_call_ms", end, begin, 1e6)
        stamp = number(row, "source_stamp_ns")
        candidates = pnp.get(stamp, []) if same_clock else []
        if (image_counts[stamp] == 1 and stamp in nonempty and candidates
                and begin is not None and max(candidates) <= begin):
            derived["pnp_completed_ns"] = max(candidates)
            derived["pnp_association"] = "source_frame_last_solver_return"
            interval("pnp_to_submit_ms", begin, max(candidates), 1e6)
        else:
            derived["pnp_association"] = "missing_ambiguous_or_different_clock"
        acknowledgements = ack_by_wire.get((row.get("wire_session"), row.get("wire_sequence")), [])
        if acknowledgements and submitted:
            ack = acknowledgements[0]
            derived.update({key: ack[key] for key in (
                "mcu_status", "rx_complete_us", "parsed_us", "adopted_us",
                "control_cycle", "control_period_us", "control_tick_us")})
            derived["ack_received_ns"] = number(ack, "host_ns")
            interval("round_trip_ack_ms", number(ack, "host_ns"), begin, 1e6)
            delta = elapsed_us(number(ack, "parsed_us"), number(ack, "rx_complete_us"))
            if delta is not None:
                derived["mcu_rx_to_parse_ms"] = delta / 1000
                metrics["mcu_rx_to_parse_ms"].append(delta / 1000)
            else:
                errors["ambiguous_mcu_rx_to_parse_wrap_or_reset"] += 1
            if number(ack, "mcu_status") == APPLIED:
                rx_parse = delta
                parse_adopt = elapsed_us(number(ack, "adopted_us"), number(ack, "parsed_us"))
                rx_adopt = elapsed_us(number(ack, "adopted_us"), number(ack, "rx_complete_us"))
                # Three individually plausible deltas need not describe one
                # ordered interval across wrap/reset. Validate the whole chain.
                if (rx_parse is not None and parse_adopt is not None and rx_adopt is not None
                        and rx_parse + parse_adopt == rx_adopt):
                    for name, value in (("mcu_parse_to_adopt_ms", parse_adopt / 1000),
                                        ("mcu_rx_to_adopt_ms", rx_adopt / 1000)):
                        derived[name] = value
                        metrics[name].append(value)
                else:
                    errors["invalid_mcu_adoption_order_or_ambiguous_wrap"] += 1
        derived["fully_linked_applied"] = (submitted and "pnp_completed_ns" in derived
            and derived.get("mcu_status") == str(APPLIED)
            and all(name in derived for name in ("pack_ms", "send_call_ms", "round_trip_ack_ms",
                    "mcu_rx_to_parse_ms", "mcu_rx_to_adopt_ms", "mcu_parse_to_adopt_ms")))
        commands.append(derived)

    # Packing occurs on source updates, not on every timer-driven resubmission.
    # Include packed commands overwritten before send, and count each pack once.
    packs = [row for row in host_rows if row["event"] == "source_packed"]
    seen_packs = set()
    for row in packs or commands:
        begin, end = number(row, "pack_start_ns"), number(row, "pack_end_ns")
        key = row.get("source_id"), begin, end
        if begin is not None and end is not None and end >= begin and key not in seen_packs:
            metrics["pack_ms"].append((end - begin) / 1e6)
            seen_packs.add(key)

    metric_names = ("pnp_to_submit_ms", "pack_ms", "wire_envelope_pack_ms", "send_call_ms",
                    "round_trip_ack_ms", "mcu_rx_to_parse_ms", "mcu_parse_to_adopt_ms", "mcu_rx_to_adopt_ms")
    stored_counts = Counter(row["event"] for row in host_rows)
    # Metadata counters continue even after the bounded raw buffer fills.
    # Never silently report the truncated raw-event count as the full count.
    counts = Counter(host_meta.get("counts", {})) or stored_counts
    statuses = Counter(row.get("status") for row in host_rows if row["event"] == "send")
    mcu_statuses = Counter(str(row.get("mcu_status")) for row in commands if row.get("mcu_status"))
    submitted = [row for row in host_rows if row["event"] == "send"
                 and row["status"] in ("host_write_returned", "host_queued")]
    feedback = [row for row in host_rows if row["event"] == "feedback"]
    feedback_periods = []
    missing_feedback = 0
    for a, b in zip(feedback, feedback[1:]):
        delta = (number(b, "mcu_timestamp_ms") - number(a, "mcu_timestamp_ms")) & 0xFFFFFFFF
        gap = (number(b, "feedback_sequence") - number(a, "feedback_sequence")) & 0xFFFFFFFF
        if 0 < delta < 0x80000000 and 0 < gap < 0x80000000:
            feedback_periods.append(delta / gap)
            missing_feedback += gap - 1
        else:
            errors["feedback_wrap_reset_or_duplicate_ambiguous"] += 1
    control_periods = [int(row["control_period_us"]) / 1000 for row in commands
                       if row.get("mcu_status") == str(APPLIED) and int(row["control_period_us"]) > 0]
    applied = [row for row in commands if row.get("mcu_status") == str(APPLIED)]
    control_cycles, control_elapsed_us = 0, 0
    for a, b in zip(applied, applied[1:]):
        delta_us = elapsed_us(int(b["control_tick_us"]), int(a["control_tick_us"]))
        delta_cycles = (int(b["control_cycle"]) - int(a["control_cycle"])) & 0xFFFFFFFF
        if delta_us and 0 < delta_cycles < 0x80000000:
            control_cycles += delta_cycles
            control_elapsed_us += delta_us
    metric_results = {name: stats(metrics[name]) for name in metric_names}
    metric_results["sampled_control_period_ms"] = stats(control_periods)
    fully_linked = sum(row["fully_linked_applied"] for row in commands)
    quality_issues = []
    if host_meta.get("counts", {}).get("log_records_dropped", 0):
        quality_issues.append("host_log_records_dropped")
    if (pnp_meta or {}).get("counts", {}).get("log_records_dropped", 0):
        quality_issues.append("pnp_log_records_dropped")
    if errors:
        quality_issues.append("invalid_timing_or_feedback_records")
    status = "incomplete_baseline" if fully_linked < 100 else (
        "at_least_100_linked_applied_with_quality_issues" if quality_issues
        else "at_least_100_linked_applied")
    return commands, dict(
        schema=1, status=status, quality_issues=quality_issues,
        fully_linked_applied_commands=fully_linked,
        same_host_monotonic_clock_verified=same_clock, metrics=metric_results,
        event_counts=dict(counts), stored_event_counts=dict(stored_counts),
        event_count_scope="metadata_totals" if host_meta.get("counts") else "stored_rows_only",
        send_status_counts=dict(statuses), mcu_status_counts=dict(mcu_statuses),
        pack_metric_scope="all_source_packed_events" if packs else "unique_packs_on_sent_commands_only",
        commands_without_correlated_mcu_status=sum(not row.get("mcu_status") for row in commands),
        duplicate_mcu_ack_count=sum(max(0, len(rows) - 1) for rows in ack_by_wire.values()),
        feedback_missing_sequences=missing_feedback, analysis_errors=dict(errors),
        host_log_records_dropped=host_meta.get("counts", {}).get("log_records_dropped", 0),
        pnp_log_records_dropped=(pnp_meta or {}).get("counts", {}).get("log_records_dropped", 0),
        ack_parser_invalid_frames=host_meta.get("ack_parser_invalid_frames", 0),
        diagnostic_stream_invalid_frames=host_meta.get("diagnostic_stream_invalid_frames", 0),
        diagnostic_command_echo_frames=host_meta.get("diagnostic_command_echo_frames", 0),
        actual_rates=dict(
            host_send_interface_submissions_hz=rate([number(row, "call_before_ns") for row in submitted]),
            host_feedback_observed_hz=rate([number(row, "host_ns") for row in feedback]),
            mcu_feedback_timestamp_inferred_hz=1000 / statistics.median(feedback_periods)
            if feedback_periods else None,
            mcu_control_sampled_period_inferred_hz=1000 / statistics.median(control_periods)
            if control_periods else None,
            mcu_control_cycle_counter_hz=control_cycles * 1e6 / control_elapsed_us
            if control_elapsed_us else None),
        context=host_meta.get("context", {}),
        limitations=[
            "Send return means host write/queue completion, never MCU receipt/adoption.",
            "Round-trip acknowledgement includes feedback scheduling/transmission; never divide by two.",
            "USB command echoes are counted separately and never treated as MCU adoption acknowledgements.",
            "MCU and host clocks are never subtracted; MCU uint32 us intervals must be <2^31 us.",
            "PnP association is the last solvePnP return of the source frame, not individual object identity.",
            "Legacy telemetry has no command ACK; missing status means unknown adoption, not proven loss.",
            "Windows submissions may be overwritten before a physical write; see helper result/drop events.",
            "Control frequency uses MCU cycle counter/ticks between adoptions and sampled periods; full cycle distribution is not captured.",
            "Nonzero log-drop counts mean failure totals and percentiles may be incomplete.",
            "MCU ring drop/invalid counters must be exported separately; absent counters remain unknown.",
        ])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", type=Path, required=True)
    parser.add_argument("--pnp", type=Path)
    parser.add_argument("--mcu-counters", type=Path,
                        help="Optional debugger/export JSON of latency_dropped_records/invalid_commands")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    metadata = lambda path: json.loads(path.with_suffix(".metadata.json").read_text())
    commands, summary = summarize(read_csv(args.host), metadata(args.host),
                                  read_csv(args.pnp) if args.pnp else None,
                                  metadata(args.pnp) if args.pnp else None)
    summary["mcu_counters"] = json.loads(args.mcu_counters.read_text()) if args.mcu_counters else None
    if summary["mcu_counters"]:
        raw = summary["mcu_counters"]
        for key in ("dropped", "invalid"):
            before, after = raw.get(key + "_before"), raw.get(key + "_after")
            if before is not None and after is not None:
                raw[key + "_delta_uint32"] = (int(after) - int(before)) & 0xFFFFFFFF
    summary["raw_sources"] = dict(host=str(args.host.resolve()),
                                 pnp=str(args.pnp.resolve()) if args.pnp else None)
    args.output.mkdir(parents=True, exist_ok=True)
    fields = list(dict.fromkeys(key for row in commands for key in row)) or ["send_id"]
    with (args.output / "commands.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(commands)
    (args.output / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n")
    display = lambda value: "待实测/不可得" if value is None else f"{value:.6f}"
    report = ["# 双轴控制延迟基线", "", f"状态：{summary['status']}；完整关联且已采用命令 "
              f"{summary['fully_linked_applied_commands']} 条。", "",
              "| 指标（ms） | n | 中位数 | P95 | 最大值 |", "|---|---:|---:|---:|---:|"]
    for name, values in summary["metrics"].items():
        report.append(f"| {name} | {values['n']} | {display(values['median_ms'])} | "
                      f"{display(values['p95_ms'])} | {display(values['max_ms'])} |")
    report += ["", "实际频率：", ""]
    report += [f"- {name}: {display(value)} Hz" for name, value in summary["actual_rates"].items()]
    report += ["", "事件、发送结果和 MCU 状态数量（未采用/未知状态保留）：", "",
               "```json", json.dumps({key: summary[key] for key in (
                   "event_counts", "send_status_counts", "mcu_status_counts",
                   "stored_event_counts", "event_count_scope", "quality_issues",
                   "commands_without_correlated_mcu_status", "duplicate_mcu_ack_count",
                   "host_log_records_dropped", "pnp_log_records_dropped", "ack_parser_invalid_frames",
                   "diagnostic_stream_invalid_frames", "diagnostic_command_echo_frames",
                   "analysis_errors", "mcu_counters")},
                   ensure_ascii=False, indent=2), "```", "", "边界：", ""]
    report += [f"- {item}" for item in summary["limitations"]]
    (args.output / "report.md").write_text("\n".join(report) + "\n")
    print(json.dumps(dict(status=summary["status"], commands=len(commands),
                         fully_linked=summary["fully_linked_applied_commands"],
                         report=str(args.output / "report.md")), ensure_ascii=False))


if __name__ == "__main__":
    main()
