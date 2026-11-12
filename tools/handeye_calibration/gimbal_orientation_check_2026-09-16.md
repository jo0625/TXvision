# Gimbal Orientation Check, 2026-09-16

## Result

Yaw changed consistently with the operator-reported left turn. The independent
pitch field decreased during the operator-reported upward pitch movement, but
the transmitted quaternion barely changed. Do not yet use that quaternion as
the complete orientation of the camera/pitch assembly.

The operator subsequently confirmed that the IMU follows yaw but does not
pitch with the camera. The measurements are consistent with that mounting;
absence of pitch motion in this quaternion is therefore expected, not evidence
of a broken quaternion update. The quaternion represents the yaw-stage IMU.
This test does not establish a full camera-to-world transform or hand-eye
extrinsics. No dynamic TF was published and no control commands were sent.

## Method

The controller stayed on Windows COM3. The receive-only Windows-to-WSL pipe
decoded the existing 38-byte feedback format. Each pose was observed for about
six seconds; the first second was preserved in the raw log but excluded from
pose statistics because opening the device can release an older buffered frame.

The initial baseline was observed stable without a separate operator
stationarity confirmation. The operator confirmed arrival and holding for
each later pose before recording began. The chassis was requested to remain
fixed. Left is defined while standing behind the robot looking forward.

Quaternion layout is wxyz. Statistics use SciPy Rotation, normalized valid
quaternions, a rotation mean, and extrinsic xyz Euler angles (roll, pitch, yaw).
These mathematical angles do not by themselves establish physical frame names.
Independent yaw/pitch fields are converted from radians to degrees.

## Measured Poses

| Pose | Analyzed frames | Independent yaw (deg) | Quaternion yaw (deg) | Independent pitch (deg) | Quaternion pitch (deg) |
| --- | ---: | ---: | ---: | ---: | ---: |
| Baseline | 101 | 2.129202 | 2.130017 | 16.918945 | -0.889972 |
| Left yaw | 93 | 40.779521 | 40.777517 | 16.875000 | -0.990965 |
| Pitch up, yaw held | 102 | 40.741344 | 40.739761 | 1.142578 | -0.951759 |

All three captures had zero CRC and frame-tail errors. Within each analyzed
window, the sequence increment was one and the MCU timestamp increment was
50 ms (20 Hz). The maximum orientation deviation from the pose mean was
0.0213, 0.0145, and 0.0505 degrees respectively.

## Interpretation

- Left yaw relative to baseline: independent yaw +38.650319 degrees;
  quaternion yaw +38.647499 degrees. The measured movement exceeded the
  requested approximate 10-15 degrees but remains useful for the sign check.
  Left turn corresponds to increasing independent yaw and increasing direct
  quaternion-derived yaw under the stated math convention.
- Pitch up relative to the left-yaw pose: independent pitch -15.732422 degrees;
  quaternion pitch +0.039206 degrees; quaternion roll -0.153834 degrees;
  quaternion yaw -0.037755 degrees. The quaternion's total relative rotation,
  independent of Euler decomposition, is only 0.163781 degrees.
- Independent pitch therefore decreases for the reported upward motion.
  Its absolute zero and physical angle scale were not independently measured.
- The static pitch discrepancy is not established as a constant offset:
  a substantial independent pitch change was absent from the quaternion.
  Quaternion inversion or a fixed coordinate-basis change cannot turn a
  0.164-degree relative rotation into a 15.73-degree rotation.

The supplied protocol excerpt reads the quaternion from `get_INS_quat_point()`
and the separate pitch value from `pitch_motor->absolute_angle`. These can
represent different physical stages. The complete flashed firmware was not
available for inspection, so the exact source implementation remains unknown.

## Confirmed Mounting and Next Step

The operator confirmed: the IMU turns left/right but does not follow upward or
downward pitch. The raw quaternion is consequently not the camera/pitch pose.

Build the camera pose through the physical kinematic chain:

```text
world -> yaw-stage IMU -> yaw mechanical frame -> pitch joint -> camera
```

This needs the IMU-to-mechanical mounting transform, pitch axis and mechanical
zero/offset, and the camera's fixed mounting on the pitch assembly. Use the
measured independent pitch in that joint transform after verifying its exact
definition. Do not set the baseline value of 16.92 degrees to zero merely
because it was the first recorded pose, and do not add Euler angles directly.

With ROS x-forward/y-left/z-up axes, a head-up rotation about the left-pointing
y axis has a negative angle. The observed decreasing pitch is not by itself a
sign error. The current data establishes left-yaw and head-up signs, but not
all fixed axis alignments, the pitch zero, world-frame convention, or any
translation. Those are prerequisites for a correct camera transform and
paired hand-eye samples.

The subsequent inclinometer procedure produced a provisional pitch zero and
scale. See [pitch calibration](pitch_calibration_2026-09-16.md). The next input
needed for the kinematic chain is the fixed mechanical geometry from CAD or
measurement.

## Local Evidence

- `/tmp/rm_orientation_baseline_20260916_210041.{bin,json,jsonl}`
- `/tmp/rm_orientation_yaw_left_20260916_210226.{bin,json,jsonl}`
- `/tmp/rm_orientation_pitch_up_20260916_210404.{bin,json,jsonl}`

Each serial read ended and closed its COM3 handle. These are stationary-pose
diagnostics, not synchronized camera/IMU calibration samples.
