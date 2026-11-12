"""Exercise the real Windows duplex helper against memory only, never COM3."""

import argparse
from collections import Counter
import json
import os
from pathlib import Path
import resource
import struct
import sys
import time
from unittest.mock import patch


PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE / "scripts"))
sys.path.insert(0, str(PACKAGE / "test"))

from windows_duplex import WindowsSerialDuplex
from windows_duplex_load_mock import load_helper_script


RECORD = struct.Struct("<2sIqIIqq")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=600)
    parser.add_argument("--fault", choices=["none", "slow_write", "write_timeout", "write_error", "read_error"], default="none")
    parser.add_argument("--delay-ms", type=int, default=0)
    parser.add_argument("--write-timeout", type=float, default=.1)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if not 1 <= args.duration <= 3600:
        parser.error("duration must be between 1 and 3600 seconds")
    if not 0 < args.write_timeout <= 1:
        parser.error("write-timeout must be between 0 and 1 seconds")
    if args.output.exists():
        parser.error("output already exists; use a new filename")

    records, chunks, clocks, results, samples = [], [], [], [], []
    pending_bytes = bytearray()
    submitted_at = {}
    maxima = Counter()
    error = None
    port = process = None
    stderr_copy = None
    started = time.monotonic()

    def trace(event):
        now = time.monotonic()
        if event["kind"] == "clock":
            clocks.append((now, event["windows_ms"]))
            return
        data = event["data"]
        chunks.append((now, event["windows_read_start_us"], event["windows_read_end_us"], len(data)))
        pending_bytes.extend(data)
        while len(pending_bytes) >= RECORD.size:
            magic, seq, generated_us, attempted, completed, begin_us, end_us = RECORD.unpack_from(pending_bytes)
            del pending_bytes[:RECORD.size]
            if magic != b"MB":
                raise ValueError("Corrupt synthetic memory record")
            records.append((now, seq, generated_us, attempted, completed, begin_us, end_us, event["windows_read_end_us"]))

    def factory(name, baud, **kwargs):
        return load_helper_script(name, baud, fault=args.fault, delay_ms=args.delay_ms, **kwargs)

    def collect():
        port.service_writes()
        port.read(len(port.buffer))
        now = time.monotonic()
        for result in port.take_write_results():
            submitted = submitted_at.pop(result.request_id, now)
            results.append((result.request_id, result.written,
                            str(result.error) if result.error else None,
                            (now - submitted) * 1000))
        for name in ("expired_sequences", "issued_sequences", "acks", "frames", "buffer", "write_results"):
            maxima[name] = max(maxima[name], len(getattr(port, name)))

    try:
        with patch("windows_duplex.helper_script", factory):
            port = WindowsSerialDuplex("COM999", 115200, trace_callback=trace)
        process = port.process
        stderr_copy = os.fdopen(os.dup(port.errors.fileno()), "rb")
        started = time.monotonic()
        next_send, next_report = started + .5, started + 30
        print("START memory_only=true physical_port_opened=false", flush=True)
        while time.monotonic() - started < args.duration:
            now = time.monotonic()
            if now >= next_send:
                # Opaque non-command bytes; only the memory endpoint sees them.
                request_id = port.queue_write(b"MEMORYTEST!", timeout=args.write_timeout)
                submitted_at[request_id] = now
                # The production queue silently supersedes unsent requests.
                active_ids = {r.request_id for r in port.write_results}
                active_ids.update(r.request_id for r in (port.pending_write, port.inflight_write) if r)
                for key in list(submitted_at):
                    if key not in active_ids:
                        del submitted_at[key]
                next_send = now + .02
            collect()
            if now >= next_report:
                sample = dict(elapsed_s=now-started, submitted=port.request_id,
                              completed=len(results), feedback=len(records),
                              expired=len(port.expired_sequences), issued=len(port.issued_sequences),
                              max_rss_kib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss)
                samples.append(sample)
                print("PROGRESS " + json.dumps(sample), flush=True)
                next_report = now + 30
            time.sleep(.001)
        drain_until = time.monotonic() + .2
        while time.monotonic() < drain_until:
            collect()
            time.sleep(.001)
    except Exception as exc:
        error = f"{type(exc).__name__}: {exc}"
    finally:
        elapsed = time.monotonic() - started
        state = None if port is None else dict(
            submitted=port.request_id, expired=len(port.expired_sequences),
            issued=len(port.issued_sequences), ack_pending=len(port.acks),
            pending=port.pending_write is not None, inflight=port.inflight_write is not None,
            helper_alive_before_close=process.poll() is None)
        if port is not None:
            port.close()
        stderr = ""
        if stderr_copy is not None:
            stderr_copy.seek(0)
            stderr = stderr_copy.read().decode("utf-8", errors="replace")
            stderr_copy.close()

    def gap_max(rows, column=0, scale=1000):
        return max(((b[column]-a[column])*scale for a, b in zip(rows, rows[1:])), default=0)

    summary = dict(
        memory_only=True, physical_port_opened=False, fault=args.fault, delay_ms=args.delay_ms,
        write_timeout_s=args.write_timeout,
        duration_s=elapsed, requested_duration_s=args.duration, error=error,
        feedback_records=len(records), sequence_gaps=sum(b[1]!=a[1]+1 for a,b in zip(records,records[1:])),
        max_rx_chunk_gap_ms=gap_max(chunks), max_clock_gap_host_ms=gap_max(clocks),
        max_clock_gap_windows_ms=gap_max(clocks, 1, 1),
        max_generated_to_windows_read_ms=max(((r[7]-r[2])/1000 for r in records),default=0),
        successful_acks=sum(r[2] is None for r in results),
        write_errors=dict(Counter(r[2] for r in results if r[2])),
        max_ack_latency_ms=max((r[3] for r in results), default=0),
        state=state, maxima=dict(maxima),
        helper_exit_code=None if process is None else process.returncode,
        samples=samples, endpoint_stderr=stderr,
    )
    artifact = dict(summary=summary, records=records, chunks=chunks, clocks=clocks, write_results=results)
    with args.output.open("x") as output:
        json.dump(artifact, output, indent=2)
        output.write("\n")
    print("SUMMARY " + json.dumps(summary, ensure_ascii=False), flush=True)
    return 1 if error else 0


if __name__ == "__main__":
    sys.exit(main())
