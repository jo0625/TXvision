"""Receive Windows USB CDC bytes over a local WSL process pipe."""

from __future__ import annotations

import base64
import os
import re
import select
import subprocess
import tempfile
import time


POWERSHELL = "/mnt/c/Windows/System32/WindowsPowerShell/v1.0/powershell.exe"

# Standard output carries unmodified serial bytes; standard input only stops
# the helper. No input from WSL is ever written to the Windows serial port.
RECEIVER_SCRIPT = r'''
$ErrorActionPreference = "Stop"
Add-Type -TypeDefinition @"
using System;
using System.Threading.Tasks;
public static class FeedbackStopInput {
    public static Task<string> Wait() {
        return Task.Run(() => Console.ReadLine());
    }
}
"@
$port = [System.IO.Ports.SerialPort]::new("__PORT__", __BAUD__,
    [System.IO.Ports.Parity]::None, 8, [System.IO.Ports.StopBits]::One)
$port.ReadTimeout = 100
$port.Handshake = [System.IO.Ports.Handshake]::None
$port.DtrEnable = $false
$port.RtsEnable = $false
try {
    $port.Open()
    $stop = [FeedbackStopInput]::Wait()
    $output = [Console]::OpenStandardOutput()
    $ready = [System.Text.Encoding]::ASCII.GetBytes("READY`n")
    $output.Write($ready, 0, $ready.Length)
    $output.Flush()
    $buffer = New-Object byte[] 4096
    while (-not $stop.IsCompleted) {
        try {
            $count = $port.Read($buffer, 0, $buffer.Length)
            $output.Write($buffer, 0, $count)
            $output.Flush()
        } catch [System.TimeoutException] {}
    }
} catch {
    [Console]::Error.WriteLine($_.Exception.Message)
    exit 2
} finally {
    if ($port.IsOpen) { $port.Close() }
    $port.Dispose()
}
'''


class WindowsSerialReader:
    """Small receive-only subset of the pyserial interface used by the node."""

    def __init__(self, port: str, baud: int, powershell: str = POWERSHELL,
                 startup_timeout: float = 10.0, script_factory=None):
        if re.fullmatch(r"COM[1-9][0-9]*", port, flags=re.IGNORECASE) is None:
            raise ValueError("Windows serial port must be COM followed by a positive number")
        if baud <= 0:
            raise ValueError("baud must be positive")
        script = RECEIVER_SCRIPT.replace("__PORT__", port.upper()).replace("__BAUD__", str(baud))
        if script_factory is not None:
            script = script_factory(port.upper(), baud)
        encoded = base64.b64encode(script.encode("utf-16-le")).decode("ascii")
        self.buffer = bytearray()
        self.process = None
        self.errors = tempfile.TemporaryFile()
        try:
            self.process = subprocess.Popen(
                [powershell, "-NoProfile", "-NonInteractive", "-EncodedCommand", encoded],
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=self.errors,
                bufsize=0,
            )
            os.set_blocking(self.process.stdout.fileno(), False)
            deadline = time.monotonic() + startup_timeout
            while b"\n" not in self.buffer:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise OSError("Windows serial open timed out")
                readable, _, _ = select.select([self.process.stdout], [], [], min(0.1, remaining))
                if readable:
                    self._pump()
            header, _, pending = self.buffer.partition(b"\n")
            if header != b"READY":
                raise OSError("Unexpected Windows serial startup response")
            self.buffer = bytearray(pending)
            self._after_ready()
        except Exception:
            self.close()
            raise

    def _after_ready(self) -> None:
        pass

    def _pump(self) -> None:
        try:
            data = os.read(self.process.stdout.fileno(), 4096)
        except BlockingIOError:
            return
        if not data:
            self.errors.seek(0)
            detail = self.errors.read(4096).decode("utf-8", errors="replace").strip()
            raise OSError(f"Windows serial pipe closed: {detail or 'helper exited'}")
        self.buffer.extend(data)

    @property
    def in_waiting(self) -> int:
        if not self.buffer:
            self._pump()
        return len(self.buffer)

    def read(self, size: int) -> bytes:
        data = bytes(self.buffer[:size])
        del self.buffer[:size]
        return data

    def write(self, _data: bytes) -> int:
        raise OSError("Windows feedback transport is receive-only")

    def close(self) -> None:
        if self.process is not None:
            try:
                if self.process.poll() is None:
                    try:
                        self.process.stdin.write(b"stop\n")
                        self.process.stdin.flush()
                    except (BrokenPipeError, OSError):
                        pass
                    try:
                        self.process.wait(timeout=3.0)
                    except subprocess.TimeoutExpired:
                        self.process.kill()
                        self.process.wait(timeout=3.0)
            finally:
                self.process.stdin.close()
                self.process.stdout.close()
                self.process = None
        self.errors.close()
