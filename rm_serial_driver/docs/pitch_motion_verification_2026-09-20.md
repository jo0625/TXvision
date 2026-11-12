# Supervised pitch motion verification, 2026-09-20

The user requested a direct, faster 10-degree pitch test without an intervening
2-degree test. The user had confirmed matching command/feedback angle units
and reference plus on-site manual-stop capability, and explicitly confirmed
at least 10 degrees of clear head-up mechanical travel for this test.

## Execution and outcome

One head-up-and-back cycle was executed in mode 6. The signed command change
was -10 degrees in the controller's pitch coordinates. Yaw was held at its
fresh baseline of 2.430135727 rad; baseline pitch was 0 rad. The command used
a 1-second outward ramp, 2-second hold, 1-second return ramp, and 2-second
baseline hold. A 1-second target ramp is not a claim that the mechanism reaches
the full angle within one second. The command freshness budget remained 100 ms.

Measured results:

- 84 command writes completed in Windows; all 84 echoed 11-byte commands
  matched the transmitted bytes in order.
- Maximum reported pitch change: -9.931640615 degrees, consistent with the
  requested -10-degree target. Nearest observed target error: 0.068359385 degrees.
- End-of-probe pitch residual: -0.087890627 degrees, measured during the short
  passive observation after command sending stopped.
- Maximum absolute yaw deviation: 0.019588983 degrees; final yaw deviation
  during the probe: -0.015531850 degrees.
- The user confirmed seeing the mechanism tilt up and return.
- A later receive-only sample, sequence 23243 and MCU time 1164291 ms,
  reported pitch 0 rad again, mode 6, yaw 2.430254459 rad. Thus the short
  -0.0879-degree residual was not a demonstrated permanent steady-state error.

Together, operator observation and the correlated pitch feedback confirm the
functional pitch command/response path for this firmware, direction, and test
range. Together with the preceding yaw test, both axes now have independently
observed basic motion response. This is not validation of automatic target
tracking, pointing accuracy across the full workspace, all speeds, or safety
interlocks. The previous yaw target/return errors remain a separate follow-up;
no motor gains or calibration values were changed here.

## Data integrity and safeguards

Raw capture contains 7384 bytes: 170 CRC-valid feedback frames (6460 bytes)
plus 84 exact command echoes (924 bytes). No bytes remain unclassified.
The telemetry parser's 924 discarded bytes are these known command echoes,
not corrupt feedback. CRC error count was zero.

Before writing, the probe required stationary valid mode-6 feedback. It was
configured to stop sending if feedback became stale beyond 150 ms, mode changed,
reported pitch excursion exceeded 10.5 degrees, reported yaw changed by more
than 0.5 degrees, or pitch moved over 0.25 degrees opposite the requested
direction. No abort condition triggered. These software checks do not replace
the operator's stop control; stopping transmission does not guarantee stopping
the mechanism. Ten focused, hardware-free checks passed for ramp/guard logic
before this live test.

## Restored state and evidence

Command sending has stopped. Original receive-only Windows COM3 configuration
was restored as PID 84893 and ROS feedback was verified. Camera PID 14294
remained running. No motion sender was left running and no production motor
parameters, firmware, or calibration were edited.

Temporary evidence: `/tmp/rm-small-motion-20260920-235441/`, containing
`summary.json`, `events.jsonl`, `received.bin`, `analysis.json`, and
`restored_receiver.log`. These temporary files may be removed by system cleanup.
The user's physical observation was: head-up motion and return were visible.
