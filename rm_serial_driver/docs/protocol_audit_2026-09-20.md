# Protocol and Angle Audit, 2026-09-20

## Confirmed evidence

- The earlier live probe received acknowledgements from the Windows helper
  for three serial writes. These are host-side write completions, not MCU ACKs.
- It then collected 684 bytes: 18 CRC-valid 38-byte feedback frames, with no
  discarded bytes or detected 8-byte response. This proves continued upstream
  telemetry, not downstream command parsing.
- The available source directory is
  `/mnt/e/000workplace-E/rm/rm视觉/串口协议`. It contains vision-link, vision-aim,
  and gimbal-behaviour excerpts, but no complete USB receive task/callback or
  buildable controller project. It is not evidence of the exact flashed build.
- `vision_link(1).h` declares an 11-byte command (`A5`, two float32 values,
  `5A 5B`) and an 8-byte feedback frame (`B5`, float32 yaw, mode, `5B 5A`).
  The actual 38-byte CRC frame includes a sequence, MCU timestamp, quaternion,
  yaw, pitch and mode. The two feedback formats differ materially.
- `vision_link_rx_byte()` in the excerpt updates the target and calls
  `vision_link_send_feedback()` when a full command parses. There is no
  mode-6 requirement in this receive path. Mode 3 therefore does not explain
  missing responses from this particular source implementation.
- `CDC_Transmit_FS()` returns a status that this excerpt ignores. Even if
  parsing succeeds, this code does not guarantee the response was transmitted.

## Reproducible source-only check

From the repository root:

```bash
/usr/bin/python3 rm_serial_driver/tools/verify_firmware_excerpt.py \
  --source-dir '/mnt/e/000workplace-E/rm/rm视觉/串口协议'
```

The tool copies the supplied C/H files into a temporary directory, compiles
them against memory-only host stubs, and feeds the same 11-byte wire format.
It does not open USB, change external source files or flash firmware.

Observed result: mode 3, one parsed command, yaw 0.25, pitch -0.125, one
transmit attempt, and an 8-byte response. This validates the supplied parser
format in isolation. It says nothing about whether the actual firmware calls
this parser or implements that response.

## Remaining controller questions

1. Identify the actual flashed build and its 38-byte telemetry implementation.
   Check its USB receive entry point and how data reaches the command parser.
2. Observe a receive/parse counter, such as `vision_aim_get_frame_cnt()` if
   present in that build, using the debugger or an existing diagnostic. A
   counter increase is stronger evidence than expecting an undocumented ACK.
3. Establish whether an acknowledgement is implemented. If it is, confirm its
   exact format, timing, CDC busy handling and transmit-buffer lifetime.
4. Confirm that feedback yaw is continuous and has the same zero/sign as the
   target yaw; confirm pitch feedback and target use the same encoder origin,
   sign and scale. Check mode enumeration and command timeout in that build.

Do not infer a missing callback simply because it is absent from the excerpts.
Do not switch into vision mode merely to look for an ACK. The earlier practice
of copying feedback angles into a command is not a proven hold command until
the feedback/target coordinate contract is confirmed.

## Work completed without hardware writes

The basic aimer preserves the target timestamp used for TF lookup. A separate
controller-angle preview consumes `pitch_joint` sight angles and time-matched
feedback, removes the current pitch rotation, computes a continuous yaw target,
and applies the inverse provisional pitch calibration. Tests cover combined
yaw/pitch, wrap crossings, stale and mismatched feedback, and frame errors.

The preview always publishes `valid=false` on its own topic; candidate numbers
are for inspection only. It does not connect to the serial command topic or
make any change to the running camera/receive-only bridge. Firmware acceptance,
actuation, final calibration, travel limits and timing remain unverified.
