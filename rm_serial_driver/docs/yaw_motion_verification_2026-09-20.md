# Supervised yaw motion verification, 2026-09-20

## Confirmed outcome

The user requested motion testing and confirmed that the flashed firmware's
command and feedback angles share units and reference, and that an on-site
operator can switch to manual control or stop the robot. Live feedback reported
mode 6. Three separate, bounded yaw out-and-back tests were then completed.
The increases from 0.5 to 2 degrees and then to 10 degrees followed explicit
user requests. Each test used its own newly measured stationary baseline.

For the final 10-degree test, the user confirmed seeing clear rotation and
return. The corresponding feedback follows the commanded positive yaw ramp
and return, while pitch remains unchanged. This confirms functional yaw
command delivery, reception/interpretation, and actuator response through
WSL -> Windows COM3 -> robot for the tested firmware and operating mode.

This is not validation of independent pitch control, tracking/detection-driven
aiming, ballistic compensation, calibrated pointing accuracy, all modes,
mechanical travel limits, or native USB/IP attachment.

## Measurements

Angles below are relative to each run's starting feedback yaw, not absolute
robot heading. Final residual is measured at the end of the short post-send
observation, approximately 0.7 seconds after the sending phase ended.

| Requested yaw offset | Largest reported yaw offset | Final yaw residual | Largest pitch change | Windows completed writes |
| --- | --- | --- | --- | --- |
| 0.5 degrees | 0.414470 degrees | +0.137765 degrees | 0 degrees | 30 / 30 |
| 2 degrees | 1.687480 degrees | +0.385168 degrees | 0 degrees | 99 / 99 |
| 10 degrees | 9.208324 degrees | +0.563600 degrees | 0 degrees | 191 / 191 |

The 10-degree run started at yaw 2.420809269 rad and pitch 0 rad. It used a
5-second outward ramp, 2-second target hold, 5-second return ramp, and a
2-second baseline hold, followed by passive observation. All commands retained
the measured baseline pitch. The command age budget remained 100 ms. No firing
or mode-change command exists in these transmitted 11-byte frames.

The 10-degree hold ended near +9.189828 degrees, and the largest measured
offset was +9.208324 degrees. During baseline hold it fell from about +1.023
to +0.583 degrees, then to +0.564 degrees during passive observation. Thus
movement and return are demonstrated, but exact target attainment and exact
return to the initial angle are not demonstrated. Do not infer a particular
PID, friction, scaling, or calibration fault from this test alone.

A later receive-only sample (sequence 10657, MCU timestamp 534991 ms) reported
yaw 2.427264690 rad, about +0.370 degrees relative to the final run's baseline.
It remained in mode 6; this does not isolate mechanical settling from reported
heading drift. The residuals above are time-specific measurements, not a claim
of permanent steady-state error.

## Raw transport evidence

The final run captured 14717 bytes:

- 332 CRC-valid 38-byte telemetry frames: 12616 bytes.
- 191 echoed 11-byte angle frames: 2101 bytes.
- Every echoed angle frame exactly matched the transmitted sequence of bytes.
- No unknown bytes remained after separating telemetry and echo.

The existing telemetry parser counted the echo as 2101 discarded bytes; these
were retained and identified in the raw capture, not CRC-corrupt telemetry.
The first 0.5-degree run likewise had 89 valid feedback frames and 30 exact
command echoes. Echo alone proves byte return, while the correlated yaw motion
and the operator's observation establish functional control response.

## Limits and restored state

The probe rejects a nonstationary or non-mode-6 baseline. During execution it
stops transmitting on stale feedback, unexpected mode, nonfinite angles,
opposite yaw movement over 0.25 degrees, pitch deviation over 0.5 degrees, or
yaw excursion greater than the requested offset plus 0.5 degrees. These checks
are software observations, not a hardware interlock: stopping transmission
cannot guarantee motor stop. The operator retained responsibility for manual
stop throughout. Nine focused tests checked ramp bounds and feedback guards
without opening hardware. No abort condition triggered in these three runs.

After the final test, receive-only configuration was restored as PID 82305,
COM3 was opened, and ROS feedback was verified. Camera PID 14294 remained
running. No continuing motion sender was left running. No production motor
parameters, calibration, or firmware were changed by these tests.

Temporary evidence directories:

- `/tmp/rm-small-motion-20260920-234047/` (0.5 degrees)
- `/tmp/rm-small-motion-20260920-234303/` (2 degrees)
- `/tmp/rm-small-motion-20260920-234414/` (10 degrees)

Each has `summary.json`, timestamped `events.jsonl`, `received.bin`, and
`restored_receiver.log`. The first and final runs also have `analysis.json`.
Temporary files may be removed by system cleanup. The user observation for
the final run was: clear rotation and return were visible.
