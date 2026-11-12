"""Probe client ACK bookkeeping with a synthetic pipe peer, without hardware.

This exercises the real Python client, not the Windows helper. Only this
diagnostic's child process is replaced; the robot receiver is untouched.
"""

import argparse
from collections import Counter
import json
from pathlib import Path
import subprocess
import sys
import time
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from windows_duplex import WindowsSerialDuplex


PEER = r'''
import select, struct, sys, time
mode = sys.argv[1]
output = sys.stdout.buffer
def frame(kind, data):
    output.write(struct.pack('<BI', kind, len(data)) + data)
    output.flush()
output.write(b'READY\n'); output.flush()
next_tick = 0
late = []
while True:
    now = time.monotonic()
    if now >= next_tick:
        frame(4, struct.pack('<q', int(now*1000)))
        frame(1, b'alive')
        next_tick = now + .02
    ready = [r for r in late if r[0] <= now]
    late = [r for r in late if r[0] > now]
    for due, seq in ready:
        frame(2, struct.pack('<IB', seq, 0))
    if select.select([sys.stdin], [], [], .001)[0]:
        line = sys.stdin.readline().strip()
        if not line or line == 'stop': break
        fields = line.split()
        if fields[0] == 'SYNC':
            frame(3, struct.pack('<Iq', int(fields[1]), int(time.monotonic()*1000)))
        elif mode == 'late':
            late.append((time.monotonic()+.15, int(fields[0])))
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=["lost", "late"], required=True)
    parser.add_argument("--count", type=int, default=20)
    parser.add_argument("--settle-seconds", type=float, default=2.2)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if not 1 <= args.count <= 1000 or args.output.exists():
        parser.error("count must be 1..1000 and output must be new")
    if not 0 <= args.settle_seconds <= 60:
        parser.error("settle-seconds must be between 0 and 60")
    popen = subprocess.Popen
    processes = []

    def peer_process(_command, **kwargs):
        proc = popen([sys.executable, "-u", "-c", PEER, args.mode], **kwargs)
        processes.append(proc)
        return proc

    feedback = 0
    outcomes, samples = [], []
    port = None
    try:
        with patch("windows_serial.subprocess.Popen", peer_process):
            port = WindowsSerialDuplex("COM999", 115200)
        for index in range(args.count):
            port.queue_write(b"memory", timeout=.04)
            limit = time.monotonic()+.5
            batch = []
            while not batch and time.monotonic() < limit:
                port.service_writes()
                feedback += len(port.read(len(port.buffer))) // 5
                batch = port.take_write_results()
                time.sleep(.001)
            if not batch:
                raise RuntimeError("No timeout or ACK result within 500 ms")
            outcomes.extend(str(r.error) if r.error else "success" for r in batch)
            samples.append(dict(completed=index+1, expired=len(port.expired_sequences),
                                issued=len(port.issued_sequences), feedback=feedback))
        limit = time.monotonic()+args.settle_seconds
        while time.monotonic() < limit:
            port.service_writes()
            feedback += len(port.read(len(port.buffer))) // 5
            time.sleep(.001)
        summary = dict(memory_only=True, real_python_client=True, real_windows_helper=False,
                       mode=args.mode, submitted=port.request_id, feedback=feedback,
                       settle_seconds=args.settle_seconds,
                       outcomes=dict(Counter(outcomes)), expired_after_idle=len(port.expired_sequences),
                       issued_after_idle=len(port.issued_sequences),
                       ack_pending=len(port.acks), inflight=port.inflight_write is not None,
                       pending=port.pending_write is not None, samples=samples)
    finally:
        if port is not None:
            port.close()
    summary["child_exit_codes"] = [p.returncode for p in processes]
    with args.output.open("x") as output:
        json.dump(summary, output, indent=2)
        output.write("\n")
    print(json.dumps(summary), flush=True)


if __name__ == "__main__":
    main()
