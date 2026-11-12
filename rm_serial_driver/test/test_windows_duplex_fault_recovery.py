"""Exercise real helper concurrency and recovery with memory endpoints only."""

from pathlib import Path
import base64
import struct
import time

import pytest

from windows_duplex import WindowsSerialDuplex
from windows_duplex_load_mock import load_helper_script
from windows_serial import POWERSHELL


pytestmark = pytest.mark.skipif(not Path(POWERSHELL).is_file(), reason="Windows interop required")
RECORD = struct.Struct("<2sIqIIqq")


@pytest.mark.parametrize("fault", ["write_timeout", "write_error", "read_error"])
def test_failed_helper_replacement_receives_before_new_write(monkeypatch, fault):
    monkeypatch.setattr("windows_duplex.helper_script",
                        lambda port, baud: load_helper_script(port, baud, fault=fault))
    failed = WindowsSerialDuplex("COM999", 115200)
    old_process = failed.process
    try:
        deadline = time.monotonic()+3
        next_write = time.monotonic()
        stage = 'read' if fault == 'read_error' else 'write'
        with pytest.raises(OSError, match=f"Windows serial {stage} failed"):
            while time.monotonic() < deadline:
                now = time.monotonic()
                if now >= next_write:
                    failed.queue_write(b"MEMORYTEST!", timeout=.1)
                    next_write = now+.02
                failed.service_writes()
                failed.take_write_results()
                failed.read(len(failed.buffer))
                time.sleep(.001)
    finally:
        failed.close()
    assert old_process.poll() is not None

    monkeypatch.setattr("windows_duplex.helper_script", load_helper_script)
    restored = WindowsSerialDuplex("COM999", 115200)
    process = restored.process
    try:
        # Incoming periodic records prove reception works without retransmitting
        # any request from the failed connection.
        data = bytearray()
        deadline = time.monotonic()+1
        while len(data) < RECORD.size and time.monotonic() < deadline:
            waiting = restored.in_waiting
            data.extend(restored.read(waiting))
            time.sleep(.001)
        assert len(data) >= RECORD.size
        magic, seq, stamp, attempts, completed, begin, end = RECORD.unpack_from(data)
        assert magic == b"MB"
        assert attempts == completed == 0
        assert restored.request_id == 0
        assert restored.write(b"MEMORYTEST!", timeout=.5) == 11
    finally:
        restored.close()
    assert process.returncode == 0


@pytest.mark.parametrize('stop_during_write', [False, True])
def test_helper_replies_to_replaced_requests_and_discards_queue_on_stop(monkeypatch, stop_during_write):
    monkeypatch.setattr('windows_duplex.helper_script',
                        lambda name, baud, **kw: load_helper_script(
                            name, baud, fault='slow_write', delay_ms=500, **kw))
    records = []
    pending = bytearray()

    def trace(event):
        if event['kind'] != 'rx':
            return
        pending.extend(event['data'])
        while len(pending) >= RECORD.size:
            records.append(RECORD.unpack_from(pending))
            del pending[:RECORD.size]

    port = WindowsSerialDuplex('COM999', 115200, trace_callback=trace)
    process = port.process
    try:
        for _ in range(4):
            assert port.write(b'MEMORYTEST!', timeout=.5) == 11

        # Submit directly to the helper to deterministically exercise its
        # one-slot queue independently of the client's own latest-only queue.
        def submit(sequence):
            port.issued_sequences.add(sequence)
            data = base64.b64encode(b'MEMORYTEST!').decode('ascii')
            port._send_line(f'{sequence} {port.sync_windows_ms + 900} {data}\n'.encode(),
                            time.monotonic()+.2)

        submit(6)
        limit = time.monotonic()+1
        while not any(r[3:5] == (5, 4) for r in records) and time.monotonic() < limit:
            port._pump()
            time.sleep(.001)
        assert any(r[3:5] == (5, 4) for r in records)
        for sequence in (7, 8, 9):
            submit(sequence)
        limit = time.monotonic()+.3
        while not {7, 8}.issubset(port.acks) and time.monotonic() < limit:
            port._pump()
            time.sleep(.001)
        assert port.acks[7] == port.acks[8] == 3
        if stop_during_write:
            port._send_line(b'stop\n', time.monotonic()+.2)
            process.wait(timeout=3)
            port.errors.seek(0)
            diagnostics = port.errors.read().decode(errors='replace')
            # Only the already-running fifth write may complete after stop;
            # the latest queued sixth write must never run.
            stop_line = next(line for line in diagnostics.splitlines() if 'phase=stop' in line)
            assert 'attempted=5 completed=5' in stop_line
        else:
            limit = time.monotonic()+1
            while not {6, 9}.issubset(port.acks) and time.monotonic() < limit:
                port._pump()
                time.sleep(.001)
            assert port.acks == {6: 0, 7: 3, 8: 3, 9: 0}
    finally:
        port.close()
    assert process.returncode == 0


def test_slow_write_does_not_stall_periodic_reads_or_clock(monkeypatch):
    def factory(name, baud, **kwargs):
        return load_helper_script(name, baud, fault='slow_write', delay_ms=500, **kwargs)

    monkeypatch.setattr('windows_duplex.helper_script', factory)
    clocks, records = [], []
    pending = bytearray()

    def trace(event):
        if event['kind'] == 'clock':
            clocks.append(event['windows_ms'])
        else:
            pending.extend(event['data'])
            while len(pending) >= RECORD.size:
                record = RECORD.unpack_from(pending)
                del pending[:RECORD.size]
                records.append((record, event['windows_read_end_us']))

    port = WindowsSerialDuplex('COM999', 115200, trace_callback=trace)
    process = port.process
    results = []
    try:
        for _ in range(4):
            assert port.write(b'MEMORYTEST!', timeout=.5) == 11
        port.queue_write(b'MEMORYTEST!', timeout=.1)
        end = time.monotonic()+.8
        while time.monotonic() < end:
            port.service_writes()
            port.read(len(port.buffer))
            results.extend(port.take_write_results())
            time.sleep(.001)
        during_write = [(r, read_us) for r, read_us in records if r[3:5] == (5, 4)]
        assert len(during_write) >= 4
        assert max((read_us-r[2])/1000 for r, read_us in during_write) < 150
        assert max(b-a for a, b in zip(clocks, clocks[1:])) < 150
        assert any(isinstance(r.error, TimeoutError) for r in results)
        assert process.poll() is None
        assert not port.expired_sequences
    finally:
        port.close()
    assert process.returncode == 0
