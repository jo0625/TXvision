#!/usr/bin/env python3

import unittest
import csv
import json
import subprocess
import sys
import tempfile
from pathlib import Path

from tools.analyze_mvs_buffer_benchmark import (
    apply_clock_calibration,
    percentile,
    summarize_pause_repetition,
    summarize_stage,
    lower_latency_evidence,
    usb_session_counts,
    INTEGER_COLUMNS,
)


class BufferBenchmarkAnalysisTest(unittest.TestCase):
    def test_end_to_end_csv_report_with_synthetic_clock_and_all_nodes(self):
        # Synthetic data verifies report generation only; it is never camera evidence.
        with tempfile.TemporaryDirectory(prefix="mvs-analysis-test-") as temp:
            directory = Path(temp)
            def save(name, rows, fields=None):
                with (directory / name).open("w", newline="") as stream:
                    writer = csv.DictWriter(stream, fieldnames=fields or list(rows[0]))
                    writer.writeheader()
                    writer.writerows(rows)
            config = dict(model="MV-CS016-10UC", serial="SYNTHETIC", actual_width=1440,
                          actual_height=1080, actual_offset_x=0, actual_offset_y=0,
                          actual_pixel_format=17301513, actual_frame_rate=100,
                          actual_exposure_us=3000, actual_gain_db=12, continuous_seconds=60,
                          load_seconds=60, payload_size_bytes=1555200)
            save("configuration.csv", [{"key": k, "value": v} for k, v in config.items()])
            events, calibrations = [], []
            for nodes in (2, 4, 8, 16, 32, 64):
                rows = []
                origin = nodes * 1_000_000_000_000
                def event(phase, kind, instant):
                    events.append(dict(node_count=nodes, phase=phase, pause_ms=0, repeat=0,
                                       event=kind, steady_ns=instant, realtime_ns=instant, code=0))
                def frame(phase, instant, pause=0, repeat=0):
                    row = dict.fromkeys(INTEGER_COLUMNS, 0)
                    row.update(node_count=nodes, phase=phase, app_get_ns=instant,
                               device_timestamp_raw=(instant - origin - 5_000_000) // 10,
                               frame_bytes=1555200, rss_bytes=50_000_000,
                               pause_ms=pause, repeat=repeat)
                    rows.append(row)
                for index, phase in enumerate(("continuous", "simulated_load")):
                    start = origin + (index * 70 + 10) * 1_000_000_000
                    event(phase, "phase_start", start)
                    event(phase, "phase_end", start + 60_000_000_001)
                    for sample in range(61):
                        frame(phase, start + sample * 1_000_000_000)
                instant = origin + 150_000_000_000
                for pause in (20, 50, 100, 250):
                    for repeat in range(1, 4):
                        phase = f"pause_{pause}ms_r{repeat}"
                        event(phase, "pause_start", instant - pause * 1_000_000)
                        event(phase, "pause_end", instant)
                        for sample in range(10):
                            frame(phase, instant + sample * 10_000_000, pause, repeat)
                        instant += 1_000_000_000
                for position, instant in (("before", origin + 1_000_000_000),
                                          ("after", origin + 200_000_000_000)):
                    calibrations.append(dict(node_count=nodes, position=position, sample=0,
                        host_before_ns=instant - 100, host_after_ns=instant + 100,
                        device_timestamp_ns=(instant - origin) // 10, timestamp_increment_ns=10))
                save(f"raw_frames_nodes_{nodes:02d}.csv", rows)
            save("events.csv", events)
            save("clock_calibration.csv", calibrations)
            save("runtime_memory_usb.csv", [], ["node_count"])
            (directory / "restoration_status.txt").write_text("restoration_verified\n")
            result = subprocess.run([sys.executable, str(Path(__file__).with_name(
                "analyze_mvs_buffer_benchmark.py")), str(directory)], capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            summary = json.loads((directory / "summary.json").read_text())
            self.assertTrue(summary["conclusions"]["protocol_valid"])
            self.assertIn("无需因正常延迟调整节点数", (directory / "report.md").read_text())
            self.assertEqual(len(summary["configuration_comparison"]), 6)

    def test_percentile_interpolates(self):
        self.assertEqual(percentile([0.0, 10.0], 0.5), 5.0)
        self.assertEqual(percentile([3.0], 0.99), 3.0)

    def test_latency_gain_is_not_claimed_during_temporal_instability(self):
        self.assertFalse(lower_latency_evidence(1.153, 1.0, 12.0))
        self.assertFalse(lower_latency_evidence(.15, 1.0, .2))
        self.assertTrue(lower_latency_evidence(2.0, 1.0, .2))

    def test_usb_counter_before_start_belongs_to_previous_session(self):
        rows = [{"frame_gap_before": gap, "duplicate_or_reordered": 0} for gap in (0, 2, 0)]
        result = usb_session_counts(dict(usb_received_before="13932", usb_received_after="5",
                                         usb_errors_after="0"), rows)
        self.assertEqual(result["usb_received_frames"], 5)
        self.assertEqual(result["usb_error_frames"], 0)
        self.assertEqual(result["usb_received_before_start_raw"], 13932)
        unverified = usb_session_counts(dict(usb_received_before="13932", usb_received_after="6",
                                             usb_errors_after="0"), rows)
        self.assertIsNone(unverified["usb_received_frames"])

    def test_sdk_host_time_is_milliseconds_without_camera_clock_mapping(self):
        samples = [
            dict(node_count="16", position=position, host_before_ns=str(instant - 100),
                 host_after_ns=str(instant + 100), device_timestamp_ns=str(instant // 10),
                 timestamp_increment_ns="100000000")
            for position, instant in (("before", 1_000_000), ("after", 11_000_000))]
        frames = [dict(node_count=16, app_get_ns=6_000_000, device_timestamp_raw=590_000,
                       app_realtime_ns=1_000_000_540_000, sdk_host_timestamp_raw=1_000_000)]
        apply_clock_calibration(frames, samples)
        self.assertAlmostEqual(frames[0]["sdk_host_to_app_ms"], .54)

    def test_clock_calibration_maps_device_timestamp(self):
        calibration = [
            {
                "node_count": "16", "position": "before", "sample": "0",
                "host_before_ns": "1000000", "host_after_ns": "1000200",
                "device_timestamp_ns": "100000", "timestamp_increment_ns": "10",
            },
            {
                "node_count": "16", "position": "after", "sample": "0",
                "host_before_ns": "11000000", "host_after_ns": "11000200",
                "device_timestamp_ns": "10100000", "timestamp_increment_ns": "10",
            },
        ]
        frames = [{"node_count": 16, "app_get_ns": 6000100, "device_timestamp_raw": 5000000}]
        result = apply_clock_calibration(frames, calibration)
        self.assertAlmostEqual(frames[0]["camera_to_app_ms"], 0.1, places=6)
        self.assertAlmostEqual(frames[0]["clock_error_ms"], 0.0001, places=9)
        self.assertAlmostEqual(result[16]["scale_ns_per_raw_unit"], 1.0, places=9)

    def test_clock_scale_is_measured_for_100mhz_firmware(self):
        samples = [
            {"node_count": "16", "position": "before", "host_before_ns": "1000000",
             "host_after_ns": "1000200", "device_timestamp_ns": "10000",
             "timestamp_increment_ns": "10"},
            {"node_count": "16", "position": "after", "host_before_ns": "11000000",
             "host_after_ns": "11000200", "device_timestamp_ns": "1010000",
             "timestamp_increment_ns": "10"},
        ]
        frames = [{"node_count": 16, "app_get_ns": 6000100, "device_timestamp_raw": 500000}]
        result = apply_clock_calibration(frames, samples)
        self.assertEqual(result[16]["scale_ns_per_raw_unit"], 10.0)
        self.assertAlmostEqual(frames[0]["camera_to_app_ms"], .1)

    def test_stage_summary_counts_queue_gaps_and_duration(self):
        rows = []
        for index, queue in enumerate((0, 1, 2)):
            rows.append({
                "app_get_ns": index * 1_000_000_000,
                "frame_bytes": 1024,
                "sdk_queue_after_get": queue,
                "queue_query_code": 0,
                "camera_to_app_ms": 2.0 + index,
                "rss_bytes": 1000 + index,
                "frame_gap_before": 1 if index == 2 else 0,
                "duplicate_or_reordered": 0,
                "process_ns": 1_000_000,
                "clock_error_ms": 0.2,
            })
        events = [
            {"event": "phase_start", "steady_ns": 0},
            {"event": "phase_end", "steady_ns": 60_000_000_001},
            {"event": "get_timeout", "steady_ns": 1},
        ]
        summary = summarize_stage(rows, events, "continuous")
        self.assertEqual(summary["frame_gaps"], 1)
        self.assertEqual(summary["timeouts"], 1)
        self.assertEqual(summary["queue"]["p50"], 1)
        self.assertGreaterEqual(summary["wall_duration_s"], 60.0)

    def test_pause_recovery_requires_consecutive_normal_frames(self):
        resume = 1_000_000_000
        rows = []
        for index, (age, queue) in enumerate(
            ((30.0, 2), (20.0, 1), (5.0, 0), (5.0, 0), (5.0, 0), (5.0, 0), (5.0, 0))
        ):
            rows.append({
                "app_get_ns": resume + index * 10_000_000,
                "camera_to_app_ms": age,
                "sdk_queue_after_get": queue,
                "frame_gap_before": 0,
                "duplicate_or_reordered": 0,
                "pause_ms": 20,
            })
        events = [
            {"event": "pause_start", "steady_ns": resume - 20_000_000},
            {"event": "pause_end", "steady_ns": resume},
        ]
        summary = summarize_pause_repetition(rows, events, normal_p95_ms=5.0,
                                             measurement_floor_ms=1.0)
        self.assertTrue(summary["lossless"])
        self.assertEqual(summary["first_frame_age_ms"], 30.0)
        self.assertEqual(summary["backlog_drain_ms"], 20.0)
        self.assertEqual(summary["normal_latency_recovery_ms"], 60.0)
        self.assertEqual(summary["first_normal_latency_ms"], 20.0)
        self.assertEqual(summary["normal_latency_threshold_ms"], 6.0)


if __name__ == "__main__":
    unittest.main()
