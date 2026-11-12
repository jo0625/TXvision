import subprocess
import sys
import time

import pytest

from windows_serial import WindowsSerialReader


def fake_windows_process(monkeypatch, script):
    popen = subprocess.Popen
    processes = []

    def start(_command, **kwargs):
        process = popen([sys.executable, "-u", "-c", script], **kwargs)
        processes.append(process)
        return process

    monkeypatch.setattr("windows_serial.subprocess.Popen", start)
    return processes


def test_fragmented_binary_feedback_and_receive_only(monkeypatch):
    processes = fake_windows_process(
        monkeypatch,
        "import sys,time\n"
        "sys.stdout.buffer.write(b'READY\\n\\xb5\\x00'); sys.stdout.buffer.flush()\n"
        "time.sleep(0.05)\n"
        "sys.stdout.buffer.write(b'\\xff\\x0a\\x5b\\x5a'); sys.stdout.buffer.flush()\n"
        "assert sys.stdin.readline() == 'stop\\n'\n",
    )
    reader = WindowsSerialReader("COM3", 115200)
    try:
        received = bytearray()
        deadline = time.monotonic() + 2.0
        while len(received) < 6 and time.monotonic() < deadline:
            if reader.in_waiting:
                received.extend(reader.read(2))
            time.sleep(0.005)
        assert bytes(received) == b"\xb5\x00\xff\x0a\x5b\x5a"
        with pytest.raises(OSError, match="receive-only"):
            reader.write(b"\xa5" + b"\x00" * 10)
    finally:
        reader.close()
    assert processes[0].poll() == 0


def test_open_failure_preserves_windows_error(monkeypatch):
    processes = fake_windows_process(
        monkeypatch, "import sys; sys.stderr.write('COM3 is busy'); sys.exit(2)"
    )
    with pytest.raises(OSError, match="COM3 is busy"):
        WindowsSerialReader("COM3", 115200)
    assert processes[0].poll() is not None


def test_closed_windows_pipe_is_a_disconnect(monkeypatch):
    processes = fake_windows_process(
        monkeypatch, "import sys; print('READY', flush=True)"
    )
    reader = WindowsSerialReader("COM3", 115200)
    try:
        processes[0].wait(timeout=2)
        with pytest.raises(OSError, match="closed"):
            _ = reader.in_waiting
    finally:
        reader.close()
