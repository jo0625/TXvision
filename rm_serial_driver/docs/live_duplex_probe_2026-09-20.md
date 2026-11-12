# Live Windows duplex probe, 2026-09-20

The user explicitly requested the live test after stating that motors remained
enabled and the controller was not in mode 6. The probe did not change modes.
This test does not establish that mode 3 is a verified motor interlock.

## Results

Two bounded probe runs used COM3 through the existing Windows duplex helper.
Each attempted one command, with no automatic retries. The current receive-only
node was stopped for each run and restored afterward. Camera PID 14294 remained
running throughout. No production source or configuration was changed.

| Run (local date/time label) | Validity | Windows result | Feedback afterward |
| --- | --- | --- | --- |
| 20260920-220227 | 100 ms | Helper rejected expiry before SerialPort.Write | 61 valid frames, mode 3 |
| 20260920-220343 | 500 ms | SerialPort.Write completed, 11 bytes | 61 valid frames, mode 3 |

Thus one hardware write completed across these two runs. The second run sent
`a5b212e9bc970d5fbd5a5b`: yaw -0.0284512974 rad and pitch -0.0544563197 rad,
copied from the newest feedback. Feedback and command coordinate equivalence
remains unverified; this is not proof that such copying is a hold command.

The successful-write run captured 3952 raw bytes, all accounted for by 104
CRC-valid 38-byte frames (43 baseline, 61 afterward). It found zero CRC errors,
tail errors, discarded/pending bytes, or separate 8-byte response candidates.
Post-write feedback sequences were 59102 through 59162. Maximum reported angle
deviation from the sent values was 0.0001317374 rad yaw and 0.0007669888 rad pitch.
These are reported sensor changes, not independent confirmation of motor state.

Windows write completion does not establish STM32 parsing. No command-specific
reply or MCU parser counter was observed. The lack of an 8-byte response does
not establish a missing receive callback in the flashed firmware.

## Host clock issue

The first helper returned its expired/unsynchronized status without calling
SerialPort.Write. Five subsequent read-only samples bracketed WSL's UTC timestamp
between Windows timestamps taken immediately before and after `wsl date`:

| Sample | WSL minus Windows lower bound (ms) | Upper bound (ms) |
| --- | --- | --- |
| 1 | -222 | -54 |
| 2 | -227 | -50 |
| 3 | -215 | -49 |
| 4 | -238 | -84 |
| 5 | -254 | -84 |

WSL was behind Windows in every sample. These intervals include process-launch
latency and are not precise offset estimates. They demonstrate clock skew that
can interfere with comparing a WSL-generated 100 ms UTC expiry against Windows
UTC. Scheduling latency may also contribute to expiry. The successful 500 ms
diagnostic does not fix this production timing problem; the configured 100 ms
command timeout was retained, and neither system clock was modified.

Before active aiming, resolve or compensate host clock skew while retaining a
bounded command age. Separately, verify STM32 parsing using the actual flashed
firmware's receive/parse counter or a documented command acknowledgement.

## Restoration and evidence

The restored node is receive-only (`transport:=windows`, `port:=COM3`,
`receive_only:=true`), PID 59679 at verification. It logged `Opened COM3` and
published feedback sequence 59736, MCU timestamp 2988941 ms, mode 3 after restore.

Raw capture directories (temporary, not durable across cleanup):

- `/tmp/rm-duplex-live-20260920-220227/`
- `/tmp/rm-duplex-live-20260920-220343/`

Each contains `summary.json`, timestamped `events.jsonl`, `received.bin`, and
`restored_receiver.log`. The one-off script is
`/tmp/rm_duplex_live_once_20260920.py`. Do not rerun it unattended or assume the
recorded PIDs still identify the same processes.

## Computer-side fix and 100 ms retest

After the preceding diagnostic, `windows_duplex.py` and its C# helper were
updated to remove cross-system UTC comparisons. Before each command a pipe-only
`SYNC sequence` exchange returns Windows monotonic milliseconds. The client
subtracts all elapsed handshake time from the original command budget, plus
2 ms for rounding, then anchors that remaining duration at the earlier Windows
timestamp. Windows checks this deadline using the same monotonic clock before
writing. The request must match the latest handshake sequence and the deadline
cannot exceed that handshake timestamp by more than one second. The handshake
itself never writes to COM3. A new handshake on every command avoids a stale
offset estimate; it does not provide hard real-time guarantees on WSL.

Focused validation passed 74 tests, zero skips, including actual Windows
helpers with memory-only COM endpoints. Tests verify UTC offsets of +/-2 seconds
and +/-1 hour at a 100 ms write budget, deducted handshake latency, missing and
corrupt handshakes, Windows rejecting delayed commands without a device write,
binary feedback, write failures, receive-only selection, and reconnect behavior.

The live retest directory is `/tmp/rm-duplex-live-20260920-221342/`:

- Original 100 ms budget, one write attempted, one Windows completion ACK.
- Command `a584b57abcd63162bd5a5b`, 11 bytes, copied from latest feedback.
- Logged call-to-completion duration approximately 18.7 ms, including handshake.
- 43 baseline feedback frames and 61 afterward; all 104 frames CRC-valid.
- Post-write sequence 71085 through 71145, mode 3 throughout, no extra response.
- No CRC/tail errors, discarded bytes, pending bytes, or 8-byte candidates.
- Maximum reported angle changes: yaw 0.0000703577 rad, pitch 0.0007669888 rad.
- Receive-only node restored as PID 61806; ROS feedback sequence 71879 observed
  afterward. Camera PID 14294 remained running.

The installed Python and C# helper paths resolve to the edited source files in
the current symlink installation. No normal rebuild or configuration change was
needed. The running receive-only transport is unchanged. Sustained 50 Hz live
command transmission was not tested.

The computer-side expiry problem is resolved for this test. Actual STM32 parsing
remains unconfirmed. Available firmware excerpts expose
`vision_aim_get_frame_cnt()`, but the live 38-byte feedback does not include this
counter. Windows device enumeration found COM3 and no named ST-Link, J-Link, or
CMSIS-DAP debugger. The current setup therefore has no identified way to read
that internal state. The next useful evidence requires the actual firmware's
diagnostic response or a debugger connection with the matching build/symbols;
repeated host writes cannot substitute for it.
