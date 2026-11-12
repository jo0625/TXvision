# STEP Gimbal Axis Extraction

Date: 2026-09-16

Source model (read only):

```text
/mnt/e/000workplace-E/rm/rm视觉/整车装配(1).STEP
```

All coordinates below are in the STEP model's global coordinate system and use
millimetres. They have not yet been remapped to ROS `x forward, y left, z up`.

## Extracted axes

Yaw axis:

```text
point     = (21.443411, any, 588.551920) mm
direction = (0, 1, 0) in STEP coordinates
line      = (21.443411, t, 588.551920)
```

Pitch joint axis:

```text
point     = (21.443411, 233.356421, any) mm
direction = (0, 0, 1) in STEP coordinates
line      = (21.443411, 233.356421, t)
```

The axes are perpendicular and intersect at:

```text
(21.443411, 233.356421, 588.551920) mm
```

Their shortest distance in the model is therefore zero within the exported
STEP precision.

Using the selected yaw joint origin, the operator-provided joint-origin
translation is:

```text
STEP coordinates: yaw_origin -> pitch_origin = (0, 178, 0) mm
```

This does not contradict the zero shortest distance between the two axis
lines. A rotation axis has no unique origin along itself; `178 mm` selects the
pitch origin's position along the yaw line. Because STEP Y is the robot's
vertical direction in this model, the corresponding ROS translation under
`x forward, y left, z up` is provisionally:

```text
ROS coordinates: yaw_frame -> pitch_joint = (0, 0, 0.178) m
```

The STEP-to-ROS axis signs still need the physical left/right check listed
below before this is promoted to deployment configuration.

## Provisional camera translation

The camera is absent from the STEP assembly. The operator-provided physical
measurement from the pitch-joint origin to the camera optical centre is:

```text
STEP coordinates: pitch_joint -> camera_optical_centre
(dx, dy, dz) = (104.23, 22.8, 0) mm
```

With STEP X mapped to robot forward, STEP Y mapped to robot up, and the zero
lateral component unaffected by the unresolved STEP-Z sign, this becomes:

```text
ROS x-forward/y-left/z-up translation:
(x, y, z) = (0.10423, 0, 0.0228) m
```

At mechanical zero, the combined origin displacement is therefore:

```text
STEP yaw_origin -> camera_optical_centre = (104.23, 200.8, 0) mm
ROS  yaw_origin -> camera optical origin = (0.10423, 0, 0.2008) m
```

The pitch-to-camera vector belongs to the moving pitch frame and rotates with
the pitch joint. The camera's fixed mounting rotation and the standard
`camera_link` to `camera_optical_frame` rotation are still required.

## Live TF direction acceptance

The implemented chain was exercised against receive-only COM3 feedback:

```text
odom -> yaw_frame -> pitch_joint -> camera_link -> camera_optical_frame
```

Feedback arrived at approximately 19.8 Hz. From the initial held pose to the
operator-confirmed left-yaw pose, raw yaw changed from `-0.382658 deg` to
`+7.737111 deg` (`+8.119769 deg`). The camera TF changed in the same horizontal
direction by approximately `+8.07 deg`; pitch stayed at `-1.266095 deg`.

From that pose to the operator-confirmed head-up pose, calibrated ROS pitch
changed from `-1.266095 deg` to `-10.450644 deg` (`-9.184549 deg`), while yaw
changed by less than `0.001 deg`. The optical forward axis gained the expected
positive-up component, and the camera-origin translation changed from about
`(0.103, 0.014, 0.203) m` to `(0.098, 0.013, 0.219) m` as the measured camera
offset rotated about the pitch axis.

These observations accept the live yaw/pitch signs, TF parent-child ordering,
and mechanical translations for hand-eye data collection. They do not replace
the fixed-board refinement of the camera mounting transform.

## Evidence

The yaw line is shared by the following independent parts:

- yaw column
- cross-roller bearing
- bearing seat and machined bearing carrier
- yaw GM6020 motor
- slip-ring mount
- motor mount and related plates

The pitch joint line is shared by:

- four F6704 flange-bearing instances, two on each side
- both gimbal side frames
- the moving linkage endpoint

The GM6020 mounted in the top gimbal has a separate parallel axis:

```text
(21.443411, 102.356421, t) mm
```

It is 131.000 mm from the F6704 joint axis in the STEP Y direction and drives
the pitch mechanism through the linkage. It is a drive-shaft axis, not the
pitch joint axis.

## Remaining physical checks

- Confirm which STEP lateral direction maps to ROS positive `y` (robot left).
- Confirm the desired positive pitch sign against the existing encoder data.
- Add the missing camera position and fixed mounting rotation.
- Use a fixed calibration target to refine the mechanically measured camera
  transform before treating it as a deployment calibration.
