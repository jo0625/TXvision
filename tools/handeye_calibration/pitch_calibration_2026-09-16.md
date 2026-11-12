# Pitch Calibration, 2026-09-16

## Outcome

The controller pitch field is stable, decreases for head-up motion, and is
approximately proportional to the physical pitch angle. The first level pose
from the replacement inclinometer was inconsistent with later controlled
returns and is excluded from the provisional zero estimate.

The provisional ROS pitch-joint conversion is:

```text
pitch_ros_deg = (pitch_raw_deg - 18.303223) / 1.023926
pitch_ros_rad = (pitch_raw_rad - 0.319452) / 1.023926
```

This uses ROS axes x-forward, y-left, z-up. Head-up is a negative rotation
about +y, matching the negative result produced by the formula. For an
inclinometer convention where head-up is positive, negate `pitch_ros`.

These values initialize the kinematic model; they are not final deployment
constants. Refine or validate the zero with fixed-board camera observations
after the mechanical transforms are available.

## Replacement-Inclinometer Measurements

| Pose | Physical pitch | Raw controller pitch | Approach |
| --- | ---: | ---: | --- |
| Initial level | 0 deg | 17.446289 deg | Uncontrolled initial setup |
| Head up | +10 deg | 7.690430 deg | From level toward head up |
| Head down | -10 deg | 28.168947 deg | From +10 through level |
| Return level A | 0 deg | 18.237305 deg | From -10 toward head up |
| Return level B | 0 deg | 18.369141 deg | From +10 toward head down |

Each pose used 99-102 analyzed frames after excluding the first second. Every
analyzed sequence advanced by one at 50 ms MCU intervals (20 Hz). CRC and tail
errors were zero. The raw pitch span within each stationary capture was one
encoder step, about 0.043945 degrees.

## Derived Values

- Controlled-return zero: mean of 18.237305 and 18.369141 = 18.303223 deg
  (0.319451511 rad).
- Directional zero difference: 0.131836 deg. This bounds the observed
  direction-dependent return difference for this test, including inclinometer
  placement/readout uncertainty; it is not proven pure drivetrain backlash.
- Endpoint scale magnitude: (28.168947 - 7.690430) / 20 = 1.023926 raw degrees
  per physical degree.
- The +/-10 endpoint midpoint was 17.929688 deg, 0.373535 deg below the
  controlled-return zero. This is the current residual between endpoint and
  zero measurements.
- The initial level was 0.856934 deg below the controlled-return mean and is
  rejected because the two later direction-controlled returns agree much more
  closely with each other.

## Limits

The inclinometer's metrology, exact resolution, and alignment to the camera
optical axis were not independently calibrated. Operator readings were given
as 0 and +/-10 degrees. Therefore the many decimal places in the raw controller
values describe encoder repeatability, not absolute physical-angle accuracy.
Do not infer sub-tenth-degree absolute accuracy from this test.

The yaw-stage quaternion does not include pitch because its IMU does not move
with the pitch assembly. Camera orientation must compose the yaw-stage attitude,
the pitch joint above, and fixed mechanical/camera transforms.

## Evidence

- `/tmp/rm_pitch_cal2_level_20260916_214051.{bin,json,jsonl}`
- `/tmp/rm_pitch_cal2_up_20260916_214325.{bin,json,jsonl}`
- `/tmp/rm_pitch_cal2_down_20260916_214454.{bin,json,jsonl}`
- `/tmp/rm_pitch_cal2_return_level_20260916_214616.{bin,json,jsonl}`
- `/tmp/rm_pitch_cal2_return_from_up_20260916_214927.{bin,json,jsonl}`

All captures were receive-only. DTR and RTS were disabled, and no aim-command
bytes were transmitted.
