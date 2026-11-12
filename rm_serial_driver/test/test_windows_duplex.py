import time

import pytest

from protocol import encode_aim_command
from test_windows_serial import fake_windows_process
from windows_duplex import WindowsSerialDuplex


def test_traced_feedback_preserves_bytes_and_windows_timestamps(monkeypatch):
    processes = fake_windows_process(monkeypatch, '''
import struct, sys, time
print('READY', flush=True)
kind, seq = sys.stdin.readline().split()
out = sys.stdout.buffer
out.write(struct.pack('<BIIq', 3, 12, int(seq), int(time.monotonic() * 1000)))
out.flush()
payload = struct.pack('<qq', 1000000, 1000123) + b'feedback'
wire = struct.pack('<BI', 5, len(payload)) + payload
out.write(wire[:8]); out.flush(); time.sleep(0.01)
out.write(wire[8:]); out.flush()
assert sys.stdin.readline() == 'stop\\n'
''')
    events = []
    port = WindowsSerialDuplex('COM3', 115200, trace_callback=events.append)
    try:
        assert read_until(port, 8) == b'feedback'
        received = [event for event in events if event['kind'] == 'rx']
        assert len(received) == 1
        assert received[0]['windows_read_start_us'] == 1000000
        assert received[0]['windows_read_end_us'] == 1000123
        assert received[0]['data'] == b'feedback'
    finally:
        port.close()
    assert processes[0].returncode == 0


SYNC_REPLY = '''
request = sys.stdin.readline().split()
assert request[0] == 'SYNC'
sys.stdout.buffer.write(struct.pack('<BIIq', 3, 12, int(request[1]),
                                    int(time.monotonic() * 1000)))
sys.stdout.buffer.flush()
'''


def read_until(port, size):
    data = bytearray()
    deadline = time.monotonic() + 2.0
    while len(data) < size and time.monotonic() < deadline:
        if port.in_waiting:
            data.extend(port.read(size - len(data)))
        time.sleep(0.001)
    return bytes(data)


def test_fragmented_feedback_during_acknowledged_write(monkeypatch):
    packet = encode_aim_command(0.25, -0.125)
    processes = fake_windows_process(monkeypatch, f'''
import base64, struct, sys, time
out = sys.stdout.buffer
def send(data):
    out.write(data); out.flush()
send(b'READY\\n' + struct.pack('<BI', 1, 3) + b'\\x00\\xff\\xb5')
{SYNC_REPLY}
seq, expires, data = sys.stdin.readline().split()
assert base64.b64decode(data) == {packet!r}
assert 0 < int(expires) - time.monotonic() * 1000 <= 1000
feedback = struct.pack('<BI', 1, 3) + b'\\x0a\\x5b\\x5a'
ack = struct.pack('<BIIB', 2, 5, int(seq), 0)
wire = feedback + ack
send(wire[:4]); time.sleep(0.01); send(wire[4:])
assert sys.stdin.readline() == 'stop\\n'
''')
    port = WindowsSerialDuplex('COM3', 115200)
    try:
        assert port.write(packet, timeout=0.5) == len(packet)
        assert read_until(port, 6) == b'\x00\xff\xb5\x0a\x5b\x5a'
    finally:
        port.close()
    assert processes[0].returncode == 0


@pytest.mark.parametrize('reply, error', [
    ("struct.pack('<BIIB', 2, 5, int(seq), 1)", 'expired'),
    ("struct.pack('<BIIB', 2, 5, int(seq) + 1, 0)", 'acknowledgement'),
    ("struct.pack('<BIIB', 2, 5, int(seq), 4)", 'status'),
    ("struct.pack('<BIIB', 2, 5, int(seq), 0) * 2", 'acknowledgement'),
    ("struct.pack('<BI', 1, 65537)", 'frame length'),
    ("struct.pack('<BI', 3, 0)", 'frame'),
])
def test_rejected_and_corrupt_acknowledgements(monkeypatch, reply, error):
    processes = fake_windows_process(monkeypatch, f'''
import struct, sys, time
print('READY', flush=True)
{SYNC_REPLY}
seq, expires, data = sys.stdin.readline().split()
sys.stdout.buffer.write({reply}); sys.stdout.buffer.flush()
sys.stdin.readline()
''')
    port = WindowsSerialDuplex('COM3', 115200)
    try:
        with pytest.raises(OSError, match=error):
            port.write(b'command', timeout=0.5)
    finally:
        port.close()
    assert processes[0].poll() is not None


@pytest.mark.parametrize('disconnect', [False, True])
def test_missing_ack_is_not_reported_as_success(monkeypatch, disconnect):
    processes = fake_windows_process(monkeypatch, f'''
import struct, sys, time
print('READY', flush=True)
{SYNC_REPLY}
sys.stdin.readline()
if not {disconnect!r}: sys.stdin.readline()
''')
    port = WindowsSerialDuplex('COM3', 115200)
    try:
        start = time.monotonic()
        with pytest.raises(OSError, match='closed|timed out'):
            port.write(b'command', timeout=0.1)
        assert time.monotonic() - start < 1.0
    finally:
        port.close()
    assert processes[0].poll() is not None


def test_feedback_remains_readable_while_write_ack_is_missing(monkeypatch):
    processes = fake_windows_process(monkeypatch, f'''
import struct, sys, time
print('READY', flush=True)
{SYNC_REPLY}
seq, expires, data = sys.stdin.readline().split()
feedback = struct.pack('<BI', 1, 4) + b'live'
sys.stdout.buffer.write(feedback); sys.stdout.buffer.flush()
time.sleep(0.2)
assert sys.stdin.readline() == 'stop\\n'
''')
    port = WindowsSerialDuplex('COM3', 115200)
    try:
        port.queue_write(b'command', timeout=0.05)
        assert read_until(port, 4) == b'live'
        deadline = time.monotonic() + 0.5
        results = []
        while not results and time.monotonic() < deadline:
            port.service_writes()
            results = port.take_write_results()
            time.sleep(0.001)
        assert len(results) == 1
        assert isinstance(results[0].error, TimeoutError)
        assert port.process.poll() is None
    finally:
        port.close()
    assert processes[0].returncode == 0


def test_pending_write_is_replaced_by_latest_command(monkeypatch):
    first = encode_aim_command(0.1, 0.0)
    superseded = encode_aim_command(0.2, 0.0)
    latest = encode_aim_command(0.3, 0.0)
    processes = fake_windows_process(monkeypatch, f'''
import base64, struct, sys, time
print('READY', flush=True)
{SYNC_REPLY}
seq1, expires, data = sys.stdin.readline().split()
assert base64.b64decode(data) == {first!r}
time.sleep(0.03)
sys.stdout.buffer.write(struct.pack('<BIIB', 2, 5, int(seq1), 0))
sys.stdout.buffer.flush()
seq2, expires, data = sys.stdin.readline().split()
assert base64.b64decode(data) == {latest!r}
sys.stdout.buffer.write(struct.pack('<BIIB', 2, 5, int(seq2), 0))
sys.stdout.buffer.flush()
assert sys.stdin.readline() == 'stop\\n'
''')
    port = WindowsSerialDuplex('COM3', 115200)
    try:
        port.queue_write(first, timeout=0.5)
        port.queue_write(superseded, timeout=0.5)
        port.queue_write(latest, timeout=0.5)
        deadline = time.monotonic() + 1.0
        results = []
        while len(results) < 2 and time.monotonic() < deadline:
            port.service_writes()
            results.extend(port.take_write_results())
            time.sleep(0.001)
        assert [result.written for result in results] == [len(first), len(latest)]
        assert all(result.error is None for result in results)
    finally:
        port.close()
    assert processes[0].returncode == 0


def test_bad_com_name_never_starts_a_helper(monkeypatch):
    processes = fake_windows_process(monkeypatch, "raise AssertionError('must not run')")
    with pytest.raises(ValueError, match='Windows serial port'):
        WindowsSerialDuplex('COM3; Write-Host bad', 115200)
    assert not processes


@pytest.mark.parametrize('delay', [0.03, 0.07])
def test_startup_handshake_delay_does_not_renew_command_lifetime(monkeypatch, delay):
    processes = fake_windows_process(monkeypatch, f'''
import struct, sys, time
print('READY', flush=True)
request = sys.stdin.readline().split()
stamp = int(time.monotonic() * 1000)
time.sleep({delay})
sys.stdout.buffer.write(struct.pack('<BIIq', 3, 12, int(request[1]), stamp))
sys.stdout.buffer.flush()
seq, expires, data = sys.stdin.readline().split()
assert 0 < int(expires) - time.monotonic() * 1000 < 100 - {delay} * 1000
sys.stdout.buffer.write(struct.pack('<BIIB', 2, 5, int(seq), 1))
sys.stdout.buffer.flush()
sys.stdin.readline()
''')
    port = WindowsSerialDuplex('COM3', 115200)
    try:
        with pytest.raises(TimeoutError, match='expired'):
            port.write(b'command', timeout=0.1)
    finally:
        port.close()
    assert processes[0].returncode == 0


@pytest.mark.parametrize('reply', [
    "b''",
    "struct.pack('<BIIq', 3, 12, int(seq) + 1, 123)",
])
def test_bad_or_missing_sync_never_sends_a_command(monkeypatch, reply):
    processes = fake_windows_process(monkeypatch, f'''
import struct, sys
print('READY', flush=True)
kind, seq = sys.stdin.readline().split()
assert kind == 'SYNC'
sys.stdout.buffer.write({reply}); sys.stdout.buffer.flush()
assert sys.stdin.readline() == 'stop\\n'
''')
    with pytest.raises(OSError, match='handshake'):
        WindowsSerialDuplex('COM3', 115200, startup_timeout=0.1)
    assert processes[0].returncode == 0


def test_deadline_does_not_extrapolate_elapsed_wsl_time(monkeypatch):
    processes = fake_windows_process(monkeypatch, '''
import struct, sys, time
print('READY', flush=True)
kind, seq = sys.stdin.readline().split()
stamp = int(time.monotonic() * 1000)
sys.stdout.buffer.write(struct.pack('<BIIq', 3, 12, int(seq), stamp))
sys.stdout.buffer.flush()
seq, expires, data = sys.stdin.readline().split()
assert 0 < int(expires) - stamp <= 98
sys.stdout.buffer.write(struct.pack('<BIIB', 2, 5, int(seq), 0))
sys.stdout.buffer.flush()
assert sys.stdin.readline() == 'stop\\n'
''')
    port = WindowsSerialDuplex('COM3', 115200)
    try:
        time.sleep(0.04)
        assert port.write(b'command', timeout=0.1) == 7
    finally:
        port.close()
    assert processes[0].returncode == 0


def test_missing_clock_pulse_drops_command_but_keeps_feedback(monkeypatch):
    processes = fake_windows_process(monkeypatch, f'''
import struct, sys, time
print('READY', flush=True)
{SYNC_REPLY}
sys.stdout.buffer.write(struct.pack('<BI', 1, 4) + b'live')
sys.stdout.buffer.flush()
assert sys.stdin.readline() == 'stop\\n'
''')
    port = WindowsSerialDuplex('COM3', 115200)
    try:
        port.sync_local_at -= 10
        port.queue_write(b'command', timeout=0.05)
        assert read_until(port, 4) == b'live'
        end = time.monotonic() + 0.5
        results = []
        while not results and time.monotonic() < end:
            port.service_writes()
            results = port.take_write_results()
            time.sleep(0.001)
        assert len(results) == 1 and isinstance(results[0].error, TimeoutError)
        assert port.process.poll() is None
    finally:
        port.close()
    assert processes[0].returncode == 0


def test_superseded_request_is_a_failure_not_a_disconnect(monkeypatch):
    processes = fake_windows_process(monkeypatch, f'''
import struct, sys, time
print('READY', flush=True)
{SYNC_REPLY}
seq, expires, data = sys.stdin.readline().split()
sys.stdout.buffer.write(struct.pack('<BIIB', 2, 5, int(seq), 3))
sys.stdout.buffer.flush()
assert sys.stdin.readline() == 'stop\\n'
''')
    port = WindowsSerialDuplex('COM999', 115200)
    try:
        with pytest.raises(TimeoutError, match='superseded'):
            port.write(b'memory', timeout=.5)
        assert port.process.poll() is None
        assert not port.issued_sequences
    finally:
        port.close()
    assert processes[0].returncode == 0


@pytest.mark.parametrize('retirement', ['ttl', 'capacity'])
def test_lost_ack_is_retired_and_late_reply_cannot_complete_new_request(monkeypatch, retirement):
    count = 1 if retirement == 'ttl' else 4
    if retirement == 'capacity':
        monkeypatch.setattr('windows_duplex.MAX_EXPIRED_ACKS', 3)
        monkeypatch.setattr('windows_duplex.EXPIRED_ACK_TTL', 60)
    processes = fake_windows_process(monkeypatch, f'''
import struct, sys, time
print('READY', flush=True)
{SYNC_REPLY}
old, expires, data = sys.stdin.readline().split()
for _ in range({count - 1}):
    sys.stdin.readline()
assert sys.stdin.readline() == 'REPLY\\n'
out = sys.stdout.buffer
out.write(struct.pack('<BIq', 4, 8, int(time.monotonic()*1000)))
out.write(struct.pack('<BIIB', 2, 5, int(old), 0))
out.write(struct.pack('<BI', 1, 4) + b'live')
out.flush()
new, expires, data = sys.stdin.readline().split()
assert new != old
out.write(struct.pack('<BIIB', 2, 5, int(new), 1)); out.flush()
assert sys.stdin.readline() == 'stop\\n'
''')
    port = WindowsSerialDuplex('COM999', 115200)
    try:
        for _ in range(count):
            with pytest.raises(TimeoutError, match='timed out'):
                port.write(b'old', timeout=.02)
        if retirement == 'ttl':
            end = time.monotonic()+2.2
            while time.monotonic() < end:
                port.service_writes()
                time.sleep(.005)
        remaining = 0 if retirement == 'ttl' else 3
        assert len(port.expired_sequences) == remaining
        assert len(port.issued_sequences) == remaining
        assert 2 not in port.issued_sequences
        port._send_line(b'REPLY\n', time.monotonic()+.5)
        assert read_until(port, 4) == b'live'
        with pytest.raises(TimeoutError, match='expired'):
            port.write(b'new', timeout=.5)
    finally:
        port.close()
    assert processes[0].returncode == 0


def test_sequence_exhaustion_requires_reconnect_without_reusing_an_id(monkeypatch):
    processes = fake_windows_process(monkeypatch, f'''
import struct, sys, time
print('READY', flush=True)
{SYNC_REPLY}
assert sys.stdin.readline() == 'stop\\n'
''')
    port = WindowsSerialDuplex('COM999', 115200)
    try:
        port.sequence = 0xFFFFFFFF
        with pytest.raises(OSError, match='sequence exhausted'):
            port.write(b'memory', timeout=.5)
        assert not port.issued_sequences
    finally:
        port.close()
    assert processes[0].returncode == 0
