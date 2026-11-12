# Robot USB reception confirmed by buzzer and byte echo

## Latest result

On 2026-09-20 the user reported flashing firmware that sounds the buzzer on
receiving data, and explicitly requested no motor movement test. A single NUL
byte (`00`) was used instead of an 11-byte angle command. After the user asked
for repeated sending, the probe sent one NUL per second with a 100 ms write
budget. The user then confirmed: "可以听到声音" (the sound can be heard).

Sending was stopped immediately on receiving that confirmation. The original
Windows receive-only ROS node was restored and verified to publish feedback.
No angle command or mode-change command was sent in either buzzer probe.

This provides evidence that data sent through WSL -> Windows COM3 -> robot USB
reaches the robot and triggers its firmware. Together with the existing upward
telemetry, basic bidirectional data transport is now confirmed for this setup.
It does not confirm parsing of the separate 11-byte angle-command format,
coordinate conventions, motion control, or native USB/IP attachment to WSL.

## Measured evidence

| Probe | Writes | Windows completions | Raw telemetry | Additional received bytes |
| --- | --- | --- | --- | --- |
| 23:13:15 single probe | 1 byte `00` | 1 | 94 CRC-valid 38-byte frames | One `00`, immediately after baseline sequence 3995 |
| 23:15:18-23:15:40 repeated probe | 23 bytes `00` | 23 | 483 CRC-valid 38-byte frames | 23 bytes, all `00` |

The repeated probe collected 18377 bytes: 483 * 38 + 23. There were no CRC or
tail errors and no pending partial frame. The current telemetry parser counts
the echoed NUL bytes as discarded bytes because they are outside its 38-byte
format; raw capture preserves them. These bytes are consistent with an echo
triggered by received data, rather than an extension to periodic telemetry.
The Windows helper forwards actual SerialPort.Read output and does not locally
echo outgoing payloads. The user's buzzer observation independently supports
robot-side reception. The exact firmware callback or parse stage containing
the buzzer code remains unidentified without the flashed source.

The earlier post-flash passive captures did not transmit any test data. Their
absence of extra bytes therefore does not contradict this newly observed
receive-triggered behavior.

## State and artifacts

Final receive-only node PID: 75244 at verification, COM3 opened successfully.
Post-restoration ROS feedback: sequence 7814, MCU timestamp 392841 ms, mode 3.
Camera PID 14294 remained running throughout.

Temporary evidence directories:

- `/tmp/rm-buzzer-rx-20260920-231312/`
- `/tmp/rm-buzzer-rx-20260920-231515/`

Both contain raw `received.bin`, `summary.json`, and the receiver restoration
log. The automated summaries retain `buzzer_observation: requires user
observation`; the user's subsequent confirmation is recorded in this document.
The repeated run records `stopped_by_user: true`. Its requested 60-probe limit
was not reached; only the recorded 23 writes occurred.

The next distinct milestone is verifying the complete angle-command parser
without assuming that a raw-byte buzzer or echo proves angle parsing. A motor
movement test remains outside the user's current requested scope.
