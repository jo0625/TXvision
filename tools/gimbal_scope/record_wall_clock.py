#!/usr/bin/env python3
"""Record native serial write wall times and external motion observations."""
from __future__ import annotations
import argparse
from collections import Counter
import csv
from datetime import datetime, timezone
from decimal import Decimal
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import threading
import time
from zoneinfo import ZoneInfo

ZONE = ZoneInfo("Asia/Shanghai")
MANUAL_FIELDS = ["trial_id", "tx_id", "observed_start_iso", "observed_reached_iso",
                 "clock_offset_ms", "observation_uncertainty_ms", "notes"]
TX_FIELDS = ["tx_id", "trial_id", "change_id", "write_started_local", "write_completed_local",
             "write_started_unix_ns", "write_completed_unix_ns", "host_monotonic_ns",
             "yaw_deg", "pitch_deg", "wire_hex"]


def local_time(ns):
    seconds, remainder = divmod(int(ns), 1_000_000_000)
    return datetime.fromtimestamp(seconds, ZONE).strftime("%Y-%m-%d %H:%M:%S") + f".{remainder // 1_000_000:03d}+08:00"


def parse_time(value):
    dt = datetime.fromisoformat(value)
    if dt.tzinfo is None:
        raise ValueError("Observed times must include timezone, for example +08:00")
    delta = dt.astimezone(timezone.utc) - datetime(1970, 1, 1, tzinfo=timezone.utc)
    return (delta.days * 86400 + delta.seconds) * 1_000_000_000 + delta.microseconds * 1000


def match_observations(transmissions, observations):
    by_id = {row["tx_id"]: row for row in transmissions}
    result = []
    for observation in observations:
        if not observation["tx_id"]:
            continue
        tx = by_id[observation["tx_id"]]
        row = dict(observation, sent_local=tx["write_completed_local"],
                   start_delay_ms=None, reached_delay_ms=None, status="pending_observation")
        if not observation["observed_start_iso"]:
            result.append(row)
            continue
        if observation["clock_offset_ms"] == "":
            row["status"] = "clock_offset_unknown"
            result.append(row)
            continue
        offset = int(Decimal(observation["clock_offset_ms"]) * 1_000_000)
        for source, dest in (("observed_start_iso", "start_delay_ms"),
                             ("observed_reached_iso", "reached_delay_ms")):
            if observation[source]:
                row[dest] = (parse_time(observation[source]) - offset - int(tx["write_completed_unix_ns"])) / 1_000_000
        row["status"] = "matched"
        if row["start_delay_ms"] < 0 or (row["reached_delay_ms"] is not None and row["reached_delay_ms"] < row["start_delay_ms"]):
            row["status"] = "time_order_requires_review"
        result.append(row)
    return result


def summarize(directory, state="recording"):
    with (directory / "transmissions.csv").open(newline="") as stream:
        tx = list(csv.DictReader(stream))
    with (directory / "manual_observations.csv").open(newline="") as stream:
        observations = list(csv.DictReader(stream))
    matched = match_observations(tx, observations)
    meta = json.loads((directory / "session.json").read_text())
    complete = [r for r in matched if r["status"] == "matched"]
    observed = [r for r in matched if r["observed_start_iso"] or r["observed_reached_iso"]]
    payload = dict(state=state, transmissions=len(tx), observations=len(observed),
                   observation_template_rows=len(observations),
                   matched=len(complete), rows=matched, timezone="Asia/Shanghai")
    (directory / "summary.json").write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n")
    lines = ["# 云台运动时间对照总结", "", f"生成时间：{local_time(time.time_ns())}",
             f"记录状态：{state}；电脑发送事件 {len(tx)} 条；观察模板 {len(observations)} 行；已录入外部观察 {len(observed)} 条；已配对 {len(complete)} 条。", "",
             f"会话开始：{meta['started_local']}；ROS_DOMAIN_ID={meta['domain']}。",
             "时间采用北京时间（UTC+08:00），保存整数纳秒原值，显示到毫秒。", "",
             "发送基准为电脑原生串口 write 返回时刻，不等同于 MCU 采用时刻或云台机械起动时刻。",
             "手机时钟偏差定义为：手机时间 − 电脑时间；只有填写该偏差后才计算延迟。",
             "毫秒显示位数不代表毫秒测量准确度；录像帧率、屏幕刷新和校时误差应另记。", ""]
    if tx:
        lines += [f"首次发送：{tx[0]['write_completed_local']}；最后发送：{tx[-1]['write_completed_local']}。", ""]
    if not observed:
        lines += ["尚未录入可用于配对的外部观察时间，当前没有机械响应延迟结论。", ""]
    else:
        lines += ["|试次|发送编号|发送时间|开始移动延迟 ms|到位延迟 ms|状态|", "|---|---|---|---|---|---|"]
        for r in matched:
            shown = lambda key: "待确定" if r[key] is None else f"{r[key]:.3f}"
            lines.append(f"|{r['trial_id']}|{r['tx_id']}|{r['sent_local']}|{shown('start_delay_ms')}|{shown('reached_delay_ms')}|{r['status']}|")
    lines += ["", "原始文件：events.jsonl（所有事件）、transmissions.csv（每次发送）、manual_observations.csv（人工观察）、session.json（校时与时钟元数据）。",
              "人工时间示例格式：2026-11-11 20:30:00.123+08:00；该示例不是实测。", ""]
    review_notes = directory / "review_notes.md"
    if review_notes.exists():
        lines += ["", review_notes.read_text()]
    (directory / "summary.md").write_text("\n".join(lines))
    return payload


class Recording:
    def __init__(self, directory, domain):
        self.directory = directory
        directory.mkdir(parents=True, exist_ok=False)
        self.lock = threading.Lock()
        self.counts = Counter()
        self.tx_id = 0
        self.change_id = 0
        self.trial_id = 0
        self.last_wire = None
        self.latest_tx = None
        self.status = "recording"
        self.stop = threading.Event()
        self.started = time.monotonic()
        self.events = (directory / "events.jsonl").open("x")
        self.tx_stream = (directory / "transmissions.csv").open("x", newline="")
        self.tx_csv = csv.DictWriter(self.tx_stream, fieldnames=TX_FIELDS)
        self.tx_csv.writeheader()
        with (directory / "manual_observations.csv").open("x", newline="") as stream:
            csv.DictWriter(stream, fieldnames=MANUAL_FIELDS).writeheader()
        begin = time.monotonic_ns()
        wall = time.time_ns()
        end = time.monotonic_ns()
        clock_status = subprocess.check_output(["timedatectl", "show", "-p", "Timezone", "-p", "NTPSynchronized"], text=True)
        meta = dict(schema=1, domain=domain, timezone="Asia/Shanghai", started_local=local_time(wall),
                    started_unix_ns=wall, monotonic_bracket_ns=[begin, end], system_clock_status=clock_status,
                    boot_id=Path("/proc/sys/kernel/random/boot_id").read_text().strip(),
                    timestamp_source="serial_bridge native write begin/end time.time_ns", observation_source="phone or external recording")
        (directory / "session.json").write_text(json.dumps(meta, ensure_ascii=False, indent=2) + "\n")
        self.flush()
        summarize(directory)

    def append(self, event, **data):
        with self.lock:
            self.counts[event] += 1
            record = dict(event=event, received_unix_ns=time.time_ns(), received_monotonic_ns=time.monotonic_ns(),
                          trial_id=self.trial_id, **data)
            self.events.write(json.dumps(record, ensure_ascii=False, allow_nan=False) + "\n")

    def transmission(self, payload):
        # Required fields come from the instrumented native write call, not observer arrival.
        first, last = payload["write_started_unix_ns"], payload["write_completed_unix_ns"]
        with self.lock:
            self.tx_id += 1
            if payload["wire_hex"] != self.last_wire:
                self.change_id += 1
                self.last_wire = payload["wire_hex"]
            row = dict(tx_id=str(self.tx_id), trial_id=self.trial_id, change_id=self.change_id,
                       write_started_local=local_time(first), write_completed_local=local_time(last),
                       write_started_unix_ns=first, write_completed_unix_ns=last,
                       host_monotonic_ns=payload["host_monotonic_ns"],
                       yaw_deg=math.degrees(payload["yaw_rad"]), pitch_deg=math.degrees(payload["pitch_rad"]),
                       wire_hex=payload["wire_hex"])
            self.tx_csv.writerow(row)
            self.latest_tx = row
        self.append("serial_write", tx_id=self.tx_id, payload=payload)

    def marker(self):
        with self.lock:
            self.trial_id += 1
        now = time.time_ns()
        self.append("manual_trial_marker", marker_unix_ns=now, marker_local=local_time(now))
        self.flush()
        return dict(trial_id=self.trial_id, marker_local=local_time(now))

    def flush(self):
        with self.lock:
            self.events.flush()
            self.tx_stream.flush()

    def snapshot(self):
        with self.lock:
            return dict(state=self.status, trial_id=self.trial_id, counts=dict(self.counts),
                        tx_count=self.tx_id, latest_tx=self.latest_tx, computer_time=local_time(time.time_ns()))

    def close(self):
        self.append("recording_stopped")
        self.flush()
        self.events.close()
        self.tx_stream.close()
        self.status = "stopped"
        summarize(self.directory, "stopped_pending_external_observations")


def handler(recording):
    page = Path(__file__).with_name("wall_clock.html").read_bytes()
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            if self.path == "/":
                body, kind = page, "text/html; charset=utf-8"
            elif self.path == "/status":
                body, kind = json.dumps(recording.snapshot(), ensure_ascii=False).encode(), "application/json"
            else:
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header("Content-Type", kind)
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_POST(self):
            if self.path == "/marker":
                if recording.status != "recording":
                    self.send_error(409, "Recording has finished")
                    return
                body = json.dumps(recording.marker()).encode()
            elif self.path == "/stop":
                recording.stop.set()
                body = b'{"stopping":true}'
            else:
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, *_):
            pass
    return Handler


def run(args):
    os.environ.update(ROS_DOMAIN_ID=str(args.domain), ROS_LOCALHOST_ONLY="1", RMW_IMPLEMENTATION="rmw_fastrtps_cpp")
    os.environ.pop("CYCLONEDDS_URI", None)
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from auto_aim_interfaces.msg import AimCommand, GimbalFeedback
    from std_msgs.msg import String
    rclpy.init()
    recording = Recording(args.output, args.domain)
    server = ThreadingHTTPServer(("127.0.0.1", args.port), handler(recording))
    worker = threading.Thread(target=server.serve_forever, daemon=True)
    worker.start()
    node = rclpy.create_node("gimbal_wall_clock_recorder", enable_rosout=False, start_parameter_services=False)
    node.create_subscription(String, "/serial_bridge/tx_event", lambda msg: recording.transmission(json.loads(msg.data)), qos_profile_sensor_data)
    def feedback(msg):
        recording.append("feedback", sequence=msg.sequence, mcu_timestamp_ms=msg.mcu_timestamp_ms,
                         yaw_rad=msg.yaw, pitch_rad=msg.pitch, mode=msg.mode)
    node.create_subscription(GimbalFeedback, "/serial_bridge/feedback", feedback, qos_profile_sensor_data)
    node.create_subscription(AimCommand, "/serial_bridge/aim_command",
        lambda msg: recording.append("command_observed", valid=msg.valid, yaw_rad=msg.yaw, pitch_rad=msg.pitch,
                                      source_stamp_ns=msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec), qos_profile_sensor_data)
    node.create_timer(0.25, recording.flush)
    def clocks():
        a=time.monotonic_ns(); b=time.time_ns(); c=time.monotonic_ns()
        recording.append("clock_anchor", wall_unix_ns=b, monotonic_bracket_ns=[a,c])
    node.create_timer(1.0, clocks)
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: recording.stop.set())
    print(json.dumps(dict(event="RECORDER_READY", output=str(args.output), domain=args.domain,
                          url=f"http://127.0.0.1:{args.port}", time=local_time(time.time_ns())), ensure_ascii=False), flush=True)
    try:
        while not recording.stop.is_set() and time.monotonic()-recording.started < args.duration:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        recording.close()
        print("RECORDER_FINISHED " + str(args.output / "summary.md"), flush=True)
        if args.keep_page:
            recording.stop.clear()
            page_deadline = time.monotonic() + 600
            while time.monotonic() < page_deadline and not recording.stop.wait(1.0):
                pass
        server.shutdown()
        worker.join()
        server.server_close()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    sub=parser.add_subparsers(dest="mode", required=True)
    record=sub.add_parser("record")
    record.add_argument("--output", type=Path, required=True)
    record.add_argument("--domain", type=int, default=198)
    record.add_argument("--port", type=int, default=8766)
    record.add_argument("--duration", type=float, default=1800)
    record.add_argument("--keep-page", action="store_true", help="Keep the clock page available for 10 minutes after recording")
    report=sub.add_parser("summarize")
    report.add_argument("directory", type=Path)
    args=parser.parse_args()
    if args.mode == "record":
        run(args)
    else:
        print(json.dumps(summarize(args.directory, "external_observations_review"), ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
