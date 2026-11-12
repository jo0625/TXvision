# Hand-eye Capture Environment

This profile is for low-rate, static-pose calibration capture on the current
WSL computer. It is not a validated real-time deployment or throughput profile.
It requires `ros-humble-rmw-cyclonedds-cpp` and a socket receive-buffer limit of
at least 8388608 bytes (`sysctl net.core.rmem_max`).

Run from the repository root:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_LOCALHOST_ONLY=1
export CYCLONEDDS_URI="file://$PWD/tools/handeye_calibration/cyclonedds_wsl_local.xml"
```

Apply these environment variables to every ROS process used for capture.
They affect only that shell and its child processes. The localhost setting is
not suitable for ROS communication with another computer.

Camera-only preview, without starting a detector, tracker, recorder, or any
gimbal-command transmitter:

```bash
ros2 run hik_camera hik_camera_node --ros-args \
  --params-file hik_camera/config/camera_params.yaml -p frame_rate:=5.0
```

Use Ctrl+C to release the camera. The 5 FPS override is for static calibration
only; it does not edit the normal camera YAML (30 FPS). A later launch using
the normal YAML requests 30 FPS again. Exposure, gain, resolution, and existing
intrinsics are unchanged by the override.

## Verification on 2026-09-15

- Ordinary localhost UDP probes of 1000 and 1400 bytes arrived; 8000 and 60000
  bytes did not arrive in the same test. This observation does not identify
  the exact Windows/WSL network-filter cause.
- CycloneDDS passed the cross-process small-message test.
- With this profile, the camera subscriber received 43 frames at about
  5.05 FPS, all with matching CameraInfo timestamps, at 1440x1080 BGR8 in
  `camera_optical_frame`. CameraInfo K/D matched the existing calibration.
- The diagnostic image was nearly black; no chessboard was detected.
- No formal hand-eye samples or camera-to-gimbal extrinsics were produced.
- The controller appeared as STM32 USB CDC (`0483:5740`, COM3, BUSID `6-3`),
  but sharing it with WSL required Windows administrator privileges.

The existing calibration board is a 9x6 inner-corner chessboard with 24 mm
spacing. Keep the board fixed while moving the gimbal during hand-eye capture.
Do not reuse old intrinsic-calibration images as hand-eye samples: they do not
contain paired gimbal poses.

Before solving, verify the live feedback format (the latest supplied format is
38 bytes with sequence, MCU timestamp, wxyz quaternion, yaw, pitch, mode, CRC16,
and tails). The older files on E: describe an 8-byte format and are insufficient.
On 2026-09-16, the user confirmed that the quaternion represents the gimbal,
not a chassis-mounted IMU. Before publishing a dynamic transform, still verify
whether it maps gimbal-to-world or world-to-gimbal and confirm the physical
axis directions by observing controlled yaw and pitch motion.

## Follow-up Passive Preflight

Both USB devices were subsequently attached to WSL. The controller appeared
at `/dev/ttyACM0`, owned by `root:dialout`. User `jo` was added to `dialout`;
already-running shells may need `sg dialout` until a new login picks up the
group membership. Do not make the serial device world-writable.

A read-only serial probe opened the device at 115200 and received zero bytes
in a 3-second observation. No aim commands or other serial data were sent.
This does not prove a hardware fault: verify that the flashed firmware sends
feedback periodically, independently of receiving an aim command, and that
its header/constants match the supplied 38-byte implementation.

The camera probe received 28 frames at about 5.02 FPS. The diagnostic image
had a mean grayscale of 0.109/255 and a P95 of 1/255; no 9x6 chessboard was
found. Check board placement, lighting, lens cap, and iris before capture.
Longer exposure may be used for a stationary printed calibration board, but
that would be a calibration-only override, not an armor-detection deployment
change. The camera and serial port were closed after these checks. No formal
hand-eye samples have been collected.

## Temporary Brightness Preview

After physical adjustments, the 3000 us / 12 dB capture detected all 54 board
corners, but the image remained dark (mean grayscale 5.76/255). A camera-only
30000 us / 12 dB probe subsequently received 27 frames and showed a much
brighter scene. The board was absent from that image and some white background
surfaces were saturated. Therefore 30000 us is a provisional preview setting,
not an accepted calibration exposure or an armor-detection setting.

With the ROS environment above, load the temporary overrides after the normal
configuration:

```bash
ros2 run hik_camera hik_camera_node --ros-args \
  --params-file hik_camera/config/camera_params.yaml \
  --params-file tools/handeye_calibration/camera_preview_overrides.yaml
```

The normal YAML remains at 30 FPS, 3000 us, and 12 dB. Keep the lens focus,
resolution, and camera mounting unchanged. Flatten and fix the board in view
before choosing the final preview exposure. The brightness probe was stopped;
no controller was opened and no formal hand-eye samples were collected.

## Feedback Protocol Probe

`feedback_probe.py` is a receive-only command-line probe, not a ROS serial
bridge or a gimbal controller. It implements the supplied 38-byte feedback:
`B5`, uint32 sequence, uint32 MCU milliseconds, six float32 values (wxyz
quaternion, yaw radians, pitch radians), uint8 mode, little-endian CRC16, and
`5B 5A`. CRC covers bytes 0-33, using the conventional RoboMaster CRC16
(initial 0xffff, reflected polynomial 0x8408, no final XOR). The supplied
source calls `append_CRC16_check_sum` but does not include its implementation;
the exact flashed firmware's CRC implementation still needs live validation.

```bash
/usr/bin/python3 -m unittest discover -s tools/handeye_calibration \
  -p 'test_feedback_probe.py' -v
sg dialout -c '/usr/bin/python3 tools/handeye_calibration/feedback_probe.py --duration 10'
```

Use `sg dialout` only if the current shell has not picked up the new group.
The probe saves raw `.bin` and decoded `.jsonl` logs under `/tmp`, then closes
the port. Host receive timestamps are arrival times, not camera exposure times
or clock-synchronized MCU timestamps. The quaternion is confirmed to represent
the gimbal, but its transform direction and physical axis convention still
require live motion verification.

The 2026-09-15 run passed all nine parser tests. `/dev/ttyACM0` opened at
115200, but received zero bytes in 10.027 seconds; no protocol bytes were
transmitted. This leaves firmware feedback scheduling and the live transport
unverified, rather than indicating a CRC parsing failure. Ask the electrical
teammate to confirm periodic `vision_link_send_feedback()` calls independent
of receiving visual aim commands.

Immediately before the serial probe, the camera received 19 frames at the
temporary 30000 us / 12 dB settings. The board was mounted on a rigid backing
and visible, but its bottom extended outside the image, so a complete 9x6
pattern was not detected. Move it upward or farther away before checking
corner quality. Both probes ended; no formal hand-eye samples were collected.

## Windows Feedback Workaround (2026-09-16)

Real periodic feedback now works in Windows COM3. Both BUSID 2-1 and 6-2
subsequently failed usbipd attach with `Device in error state`; the controller
changed from `0483:5740` to a descriptor-failure device (`0000:0002`, Code 43).
The same board, cable, and firmware were recognized on the teammate's computer.
These observations narrow the trigger to USB handoff/reset/re-enumeration,
but do not prove whether the host stack or firmware reset handling is at fault.

Use the receive-only Windows backend after applying the ROS environment at the
top of this document. The controller stays owned by Windows; only the camera
needs USB/IP attachment to WSL:

```bash
ros2 run rm_serial_driver serial_bridge_node.py --ros-args \
  --params-file rm_serial_driver/config/serial_bridge.yaml \
  -p transport:=windows -p port:=COM3 -p receive_only:=true
```

The live pipe delivered 205 CRC-valid, finite, unit-quaternion frames in about
10 seconds with no CRC or tail errors. A separate ROS check received 210
feedback messages and 209 IMU callbacks, with the continuing sequence at
20 Hz. No control commands were sent, and the checks stopped cleanly.
See [serial bridge details](../../rm_serial_driver/README.md) for limitations.

An old packet can appear on opening the port. Wait for a fresh, continuous
MCU sequence/time progression before pairing stationary images and poses.
Arrival timestamps still include buffering and Windows/WSL latency; dynamic
time synchronization has not been validated. Next verify the quaternion's
transform direction and the yaw/pitch axes with deliberate single-axis motion.
No dynamic TF or formal hand-eye extrinsics have been produced yet.

Local diagnostic records from this session:

- `/tmp/rm_windows_feedback_20260916_203151_report.json`: native Windows read.
- `/tmp/rm_windows_pipe_20260916_204837.{bin,json,jsonl}`: raw-byte pipe check.
- `/tmp/rm_windows_ros_20260916_205028.{log,json,jsonl}`: cross-process ROS check.

## Physical Axis Check (2026-09-16)

The operator-assisted pose comparison found that left yaw increased both the
independent yaw and quaternion-derived yaw by about 38.65 degrees. Upward pitch
decreased the independent pitch by 15.73 degrees while the quaternion's total
relative rotation was only 0.164 degrees. The quaternion therefore has not been
validated as the full orientation of the pitch/camera assembly. The operator
then confirmed that the IMU follows yaw only and does not pitch. This explains
the observed difference. Combine the yaw-stage IMU attitude with the verified
pitch joint and mounting transforms before building camera TF or collecting
paired hand-eye samples. Pitch mechanical zero and fixed axis alignments remain
unverified.

See [pose measurements and interpretation](gimbal_orientation_check_2026-09-16.md).

The follow-up replacement-inclinometer run established a provisional
controller-to-ROS pitch conversion. The controlled-return zero was 18.303223
degrees (0.319451511 rad), the observed directional return difference was
0.131836 degrees, and the +/-10-degree endpoint scale was 1.023926 raw degrees
per physical degree. The first level reading was rejected as an initial setup
outlier. These are kinematic initialization values and still require
fixed-board validation; see [pitch calibration](pitch_calibration_2026-09-16.md).

## Fixed-board hand-eye capture

The camera-to-gimbal direction checks and mechanical initialization are now
complete. During formal capture, keep the chassis and 9x6, 24 mm chessboard
fixed and move only the yaw/pitch gimbal. The collector uses the actual
controller feedback, so requested pose angles are approximate targets rather
than values the operator must reach exactly.

Start the receive-only bridge and camera with the ROS environment from the top
of this document. For each stationary pose, save one sample:

```bash
/usr/bin/python3 tools/handeye_calibration/capture_handeye_sample.py \
  --sample-id 01_center \
  --role calibration \
  --output-dir handeye_calibration_runs/20260916_fixed_board
```

The command saves only after feedback is stable, all 54 corners are visible,
and solvePnP succeeds. Each sample directory contains the original image, an
overlay, camera intrinsics, raw and calibrated yaw/pitch, timestamps, corner
coordinates, and the board-to-camera pose. Captures are local generated data
and are ignored by git.

After collecting at least three calibration poses, and normally 8-10 diverse
calibration poses plus 2 holdout poses, solve without changing deployment
parameters:

```bash
/usr/bin/python3 tools/handeye_calibration/solve_handeye.py \
  --session handeye_calibration_runs/20260916_fixed_board \
  --method PARK
```

The solver compares the result with the mechanical initialization and reports
fixed-board consistency. It writes candidate `gimbal_tf` values for review but
does not edit `config/gimbal_tf.yaml`.
