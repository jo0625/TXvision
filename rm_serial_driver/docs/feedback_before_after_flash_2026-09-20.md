# Feedback comparison before and after reported flashing

## Evidence and scope

This comparison checks saved raw bytes, rather than only ROS messages. Three
pre-flash captures contain 311 frames. Two post-flash receive-only captures
contain 508 frames (103 from the earlier 5-second capture, plus 405 from a new
20-second capture). All 819 frames pass the existing 38-byte framing and CRC16
checks. Every captured byte is accounted for: zero extra bytes or partial tails.

The post-flash captures send no hardware commands. They can characterize
periodic telemetry but cannot test a response emitted only after a command.
No observed frame layout is a firmware version identifier.

## What changed

Representative values compare the first frame of the latest pre-flash capture
with the first post-flash capture. They were recorded at different times and
are not a controlled test with fixed physical pose.

| Field | Before (22:13 capture) | After (22:41 capture) | Interpretation |
| --- | --- | --- | --- |
| Sequence | 71024 | 3632 | Counter returned to a lower value, consistent with restart |
| MCU milliseconds | 3553341 | 183741 | Uptime returned to a lower value, consistent with restart |
| Yaw | -0.0152974 rad, about -0.876 degrees | 2.1137505 rad, about 121.109 degrees | Reported heading changed |
| Pitch | -0.0544563 rad, about -3.120 degrees | -0.0015340 rad, about -0.088 degrees | Reported encoder angle changed |
| Quaternion w | 0.9999462 | 0.4915929 | Orientation changed consistently with yaw |
| Quaternion z | -0.0076245 | 0.8708125 | Orientation changed consistently with yaw |
| Mode | 3 | 3 | No observed change |

Across the captures, quaternion norms remain within approximately 1.3e-7 of
one. Wrapped yaw from the quaternion agrees with the yaw field to within
1.2e-5 radians. There is no evidence here of a gross field-offset, endian,
radians/degrees, or quaternion-order decoding error.

Pitch values still lie on the same 8192-count-per-revolution grid. Before,
the observed values correspond to -72/-71 ticks; after, to -3/-2 ticks. Physical
repositioning, reference initialization, or calibration changes cannot be
distinguished from these samples. Do not automatically change the ROS pitch
offset or hand-eye calibration based on this difference.

## What did not change in the sampled telemetry

The same little-endian layout fits every captured byte:

| Byte offsets (zero based) | Field | Size |
| --- | --- | --- |
| 0 | B5 header | 1 |
| 1-4 | Sequence, uint32 | 4 |
| 5-8 | MCU milliseconds, uint32 | 4 |
| 9-24 | Quaternion w/x/y/z, float32 | 16 |
| 25-28 | Yaw, float32 | 4 |
| 29-32 | Pitch, float32 | 4 |
| 33 | Mode | 1 |
| 34-35 | CRC16 over bytes 0-33 | 2 |
| 36-37 | 5B 5A tail | 2 |

For every captured frame before and after flashing:

`mcu_timestamp_ms - 50 * sequence == 2141`

Each sequence step corresponds to exactly 50 MCU milliseconds, including the
initial sequence jumps. The counter progresses during receive-only sampling
without any computer command. Thus the observed sequence is a periodic
telemetry counter, not evidence of an increment caused by command parsing.

Each capture has one sequence jump between its first two frames. The same
pattern occurs before and after flashing, while the capture procedure closes
and reopens COM3. It is consistent with the handover gap and a queued first
frame, not evidence of a new packet format. Subsequent frames within each
capture are consecutive. The first post-flash jump skips 17 sequence values;
the extended capture skips 16.

No extra ASCII diagnostics, appended fields, or separate 8-byte responses occur
in these sampled streams. This does not exclude command-triggered responses,
rare diagnostics outside the observation window, or changes to internal RX
logic which leave periodic telemetry unchanged.

## Comparison with the old source excerpts

`E:/000workplace-E/rm/rm视觉/串口协议/vision_link(1).h` declares an 8-byte frame:
header, yaw, mode, tail. Compared with that excerpt, the live 38-byte layout
includes 30 additional bytes: sequence (4), timestamp (4), quaternion (16),
pitch (4), and CRC16 (2). All those additions were already present in the
pre-flash raw captures; they do not identify this particular flash's changes.

The six excerpts in that directory retain September 4 timestamps. A separate
local project was also found at
`/home/jo/my-project/20.standard_robot_firmware_only_8f18d3c/20.standard_robot_firmware_only`.
Its `Src/usbd_cdc_if.c` has an August 29 timestamp and implements a receive ring
buffer. Its `application/usb_task.c` consumes newline-delimited tuning commands
and invokes FireWater telemetry, with no vision parser in that file. It cannot
be assumed to be the newly flashed build. Do not infer that the live firmware
lacks a vision RX parser from this older local project.

## Reproduction and restored state

Offline comparison: `/usr/bin/python3 /tmp/compare_serial_captures_20260920.py`.
Machine-readable statistics: `/tmp/rm-feedback-comparison-20260920.json`.
These temporary artifacts are not guaranteed to survive system cleanup.

Raw data directories:

- `/tmp/rm-duplex-live-20260920-220227/`
- `/tmp/rm-duplex-live-20260920-220343/`
- `/tmp/rm-duplex-live-20260920-221342/`
- `/tmp/rm-after-flash-rx-20260920-224101/`
- `/tmp/rm-after-flash-rx-20260920-224612/`

After the extended capture the original receive-only configuration was
restored as PID 69790. Camera PID 14294 remained running. No production source,
parameters, MCU firmware, or motor commands were changed in this comparison.

The remaining unknown is the actual flashed firmware's change to reception or
command-specific feedback. Periodic telemetry alone does not reveal it.
