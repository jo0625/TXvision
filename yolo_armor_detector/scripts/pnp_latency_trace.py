"""Bounded PnP-return markers; flush only when the detector stops."""
import csv
import json
import os
from pathlib import Path
import time
import uuid
import threading


class PnpLatencyTrace:
    def __init__(self, path, capacity=200000, async_flush=False):
        if capacity <= 0:
            raise ValueError("latency_trace_capacity must be positive")
        self.path = Path(path)
        if self.path.exists() or self.path.with_suffix(".metadata.json").exists():
            raise FileExistsError("use a new latency trace path")
        self.capacity = capacity
        self.rows = []
        self.dropped = 0
        self.source_stamp_ns = 0
        self.started_ns = time.monotonic_ns()
        self.closed = False
        self._stop = threading.Event()
        self._writer_error = None
        self._writer = None
        if async_flush:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            self._writer = threading.Thread(target=self._flush_loop, daemon=True)
            self._writer.start()

    def _flush_loop(self):
        try:
            with self.path.with_suffix(".events.jsonl").open("x") as out:
                cursor = 0
                while True:
                    end = len(self.rows)
                    for row in self.rows[cursor:end]:
                        out.write(json.dumps(row, ensure_ascii=False) + "\n")
                    cursor = end
                    out.flush()
                    if self._stop.is_set() and cursor == len(self.rows):
                        break
                    self._stop.wait(.25)
        except Exception as exc:
            self._writer_error = repr(exc)

    def pnp_return(self, success):
        # Called immediately after cv2.solvePnP returns, before post-validation.
        instant = time.monotonic_ns()
        self.record("pnp_return", instant, success=int(bool(success)))

    def record(self, event, instant=None, **fields):
        if len(self.rows) >= self.capacity:
            self.dropped += 1
            return
        self.rows.append(dict(event=event, host_ns=instant if instant is not None
                              else time.monotonic_ns(), source_stamp_ns=self.source_stamp_ns,
                              **fields))

    def close(self):
        if self.closed:
            return
        self.closed = True
        self._stop.set()
        if self._writer is not None:
            self._writer.join()
        self.path.parent.mkdir(parents=True, exist_ok=True)
        fields = list(dict.fromkeys(key for row in self.rows for key in row)) or ["event", "host_ns"]
        with self.path.open("x", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            writer.writerows(self.rows)
        metadata = dict(schema=1, component="detector", trace_session=uuid.uuid4().hex,
                        boot_id=Path("/proc/sys/kernel/random/boot_id").read_text().strip(),
                        time_namespace=os.readlink("/proc/self/ns/time"),
                        clock="time.monotonic_ns", unit="ns",
                        clock_resolution_s=time.get_clock_info("monotonic").resolution,
                        started_ns=self.started_ns, stopped_ns=time.monotonic_ns(),
                        counts=dict(log_records_dropped=self.dropped),
                        async_writer_error=self._writer_error,
                        definition="cv2.solvePnP return before geometric post-validation; "
                        "frame association uses last return in that source frame, not object ID")
        with self.path.with_suffix(".metadata.json").open("x") as stream:
            json.dump(metadata, stream, ensure_ascii=False, indent=2)
