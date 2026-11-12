import json
import math
from run_log import RunLog


def test_live_checkpoint_preserves_timing_and_validity(tmp_path):
    log = RunLog(tmp_path)
    assert log.record("/aimer/solution", dict(
        valid=True, event_sequence=1, source_stamp_ns=123,
        gravity_compensation_rad=0.08, flight_time_s=0.25), 1000, 2000)
    assert log.record("/aimer/solution", dict(
        valid=False, event_sequence=3, source_stamp_ns=124, reason="not_tracking"), 1100, 2100)
    log.record("/fire_control/decision", dict(allowed=False, reason="not_aligned"), 1100)
    log.checkpoint()
    rows = [json.loads(line) for line in (log.directory / "events.jsonl").read_text().splitlines()]
    assert rows[0]["received_ros_ns"] == 1000
    assert rows[0]["received_monotonic_ns"] == 2000
    assert rows[0]["message"]["source_stamp_ns"] == 123
    summary = json.loads((log.directory / "summary.json").read_text())
    assert summary["valid_counts"]["/aimer/solution"] == {"true": 1, "false": 1}
    assert summary["diagnostic_sequence_gaps"]["/aimer/solution"] == 1
    assert summary["fire_decision_counts"]["not_aligned"] == 1
    assert not summary["closed"]
    log.close()
    assert json.loads((log.directory / "summary.json").read_text())["closed"]


def test_bounded_log_reports_loss_without_overwriting_prior_run(tmp_path):
    first = RunLog(tmp_path, max_bytes=200)
    first.record("/test", {"a": 1}, 1)
    first.record("/test", {"data": "x" * 201}, 2)
    first.record("/test", {"a": 2}, 3)
    first.close()
    summary = json.loads((first.directory / "summary.json").read_text())
    assert summary["log_error"] == "log_size_limit_reached"
    assert summary["dropped_counts"]["/test"] == 2
    second = RunLog(tmp_path)
    assert first.directory != second.directory
    second.record("/feedback", {"yaw": math.nan, "pitch": math.inf}, 1)
    second.close()
    row = json.loads((second.directory / "events.jsonl").read_text())
    assert row["message"] == {"yaw": "nan", "pitch": "inf"}
