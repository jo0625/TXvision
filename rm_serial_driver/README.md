# RM Serial Driver

This package bridges the current STM32 USB CDC vision protocol to ROS 2.

Opt-in control latency recording is described in
[the baseline audit and capture guide](../docs/control_latency_baseline_2026-10-15.md).
`latency_trace_path` enables bounded CSV recording; default `latency_protocol=legacy`
preserves the 11-byte wire command and does not infer MCU acknowledgements from
telemetry. `diagnostic_v1` requires integration into the actual MCU firmware first;
the supplied MCU hooks have not been flashed or measured on hardware.

## Topics

- `~/feedback` (`auto_aim_interfaces/GimbalFeedback`): CRC-valid raw feedback.
- `~/imu` (`sensor_msgs/Imu`): finite, unit-length feedback quaternions only.
- `~/aim_command` (`auto_aim_interfaces/AimCommand`): already-computed yaw and pitch.
- `~/connected` (`std_msgs/Bool`): the serial device is open.
- `~/feedback_alive` (`std_msgs/Bool`): valid feedback arrived within the timeout.
- `~/command_active` (`std_msgs/Bool`): a fresh valid command was transmitted.

Feedback headers use host receive time because the MCU and ROS clocks are not
synchronized. `mcu_timestamp_ms` preserves the original controller timestamp.
The user clarified on 2026-09-16 that the IMU is on the yaw stage: it follows
left/right motion but not pitch. The quaternion is that IMU's orientation,
not the complete camera/pitch-assembly orientation. A physical motion check
confirmed increasing yaw for left motion and decreasing independent pitch
for head-up motion. Pitch zero and scale now have provisional inclinometer
values; the fixed IMU/camera mounting alignment still needs visual refinement.
The frame ID remains configurable. See the
[orientation check](../tools/handeye_calibration/gimbal_orientation_check_2026-09-16.md).

The gimbal TF broadcaster can publish an optional `pitch_joint -> muzzle`
static transform. Set `muzzle_frame` and measured `pitch_to_muzzle_x/y/z` in
`config/gimbal_tf.yaml` to make the physical barrel exit available to the
ballistic aimer. The default empty `muzzle_frame` publishes no muzzle transform.

The current 11-byte command format contains only yaw and pitch. It has no CRC,
sequence, validity, control-mode, or shoot field. On an invalid or stale ROS
command, this node stops transmission instead of inventing a neutral angle.
The controller must independently time out stale vision commands before this
bridge is allowed to control real hardware.

Launching the package opens the serial device but transmits nothing until a
fresh `valid=true` aim command arrives:

```bash
ros2 launch rm_serial_driver serial_bridge.launch.py
```

## Windows Receive-Only Transport

For the current WSL USB/IP enumeration failure, keep the controller owned by
Windows. Do not attach the controller with usbipd. A `Shared` device can still
be opened by Windows as long as it is not `Attached`.

From the repository root, after sourcing ROS and this workspace:

```bash
ros2 run rm_serial_driver serial_bridge_node.py --ros-args \
  --params-file rm_serial_driver/config/serial_bridge.yaml \
  -p transport:=windows -p port:=COM3 -p receive_only:=true
```

The helper uses the built-in Windows PowerShell/.NET serial API and transfers
raw bytes to WSL through its process stdout. It needs neither a Windows Python
installation nor a TCP listener. WSL parses the same feedback protocol and
publishes the same topics. Set `port` to the current Windows COM name.

The `transport=windows` backend always disables the aim-command subscription and sender,
even if `receive_only` is false. Its helper contains no serial write path.
Ctrl+C stops the helper and releases the COM port. For a native Linux device,
the default `transport=serial` behavior is unchanged; `receive_only=true` can
also explicitly disable commands on that backend.

This is a temporary diagnostic/static-calibration transport. ROS headers are
WSL arrival times and include Windows/pipe latency; they are not synchronized
camera exposure or MCU timestamps. Startup captures contained an older first
packet followed by a large MCU timestamp jump and then regular 50 ms steps.
Do not pair the initial buffered packet with a camera image. Physical axis
conventions, transform direction, and dynamic timing still need verification.

## Opt-in Windows Duplex Transport

`transport=windows_duplex` adds WSL-to-Windows command delivery. Existing
`transport=windows` launches remain receive-only, including when given
`receive_only=false`. The default YAML and launch still select the native
Linux transport; they do not enable the duplex transport automatically.

After building this package, the explicit duplex selection is:

```bash
ros2 run rm_serial_driver serial_bridge_node.py --ros-args \
  --params-file rm_serial_driver/config/serial_bridge.yaml \
  -p transport:=windows_duplex -p port:=COM3 -p receive_only:=false
```

This command opens a real Windows serial port. Run it only for a planned
hardware session after verifying the controller's receive protocol and
timeout behavior. With `receive_only=true`, even `windows_duplex` uses the
original receiver helper, which has no serial write path.

One Windows helper owns the COM port for both directions. Commands travel as
base64 envelopes over stdin; feedback and write acknowledgements use framed
binary stdout. Envelope sequence numbers and expiry times are local pipe
metadata, not changes to the STM32 wire protocol. The existing wire command
is still 11 bytes: `A5`, little-endian float32 yaw/pitch, `5A 5B`.

The client tracks one current write, with one latest-only unsent slot. A newer command
replaces an older command that has not entered the pipe. Windows acknowledges
a command after `SerialPort.Write` returns, not when the controller accepts or
executes it. The ROS node pumps feedback and acknowledgements without waiting
synchronously for either one. An acknowledgement timeout drops that command
but keeps the COM connection and feedback path alive; a closed helper, actual
write error, or malformed pipe frame still closes the connection and retries
according to `reconnect_period`. Closing or opening a connection clears the
command gate, and commands delivered while disconnected are discarded. A new
command callback is required after reopen.

The Windows helper now runs serial writes on a dedicated worker while the main
loop receives feedback and emits clock frames. A slow write therefore does not
hold up these reads or clocks. All output frames share one lock so concurrent
feedback and ACK writers cannot interleave their bytes. The helper also has one
latest-only waiting slot; replacing that slot returns ACK status 3 (superseded,
not executed). Deadlines are checked again immediately before each serial write.
Stop discards waiting commands and allows an already-running write to finish;
unknown write results are never automatically replayed. Actual serial read/write
exceptions still fail the connection and now identify the failing stage.

The client retains timed-out request IDs for at most two seconds of continued
servicing, capped at 256 entries. Older replies are ignored and cannot complete
new requests. Sequence IDs are never reused on one connection; exhausting the
32-bit counter requires reconnecting. These limits prevent lost ACKs from
growing request bookkeeping indefinitely.

`command_timeout` is measured from the ROS callback's monotonic arrival time,
not the producer's header timestamp. A startup `SYNC` checks the pipe; the
Windows helper then publishes its monotonic clock every 20 ms (frame kind 4,
little-endian int64 milliseconds). This clock traffic never writes to COM.
Each queued command keeps its original local deadline. The client adds only
the remaining lifetime to the latest Windows sample, with a 2 ms rounding
margin. It never extrapolates Windows time from elapsed WSL uptime: their
clock rates can differ. Sample transit and age shorten the effective budget.
If no clock sample has arrived for 100 ms, pending commands wait for a new
sample and retain their original expiry. Windows checks the deadline before
`SerialPort.Write`; expired and excessively future deadlines have distinct
rejection statuses. System UTC time is not used at either endpoint. `command_timeout`
must be at most one second for this backend. Expired commands are dropped
without disconnecting; a missing or corrupt startup handshake is a transport
error. Both helper files must be deployed together. These checks do not
establish camera/MCU clock synchronization or a hard real-time guarantee.
Data already accepted by the USB driver cannot be retracted, so the MCU must
enforce its own command timeout.

### Hardware-free Verification

The tests below never open a real COM port. Host tests compile and run the
actual Windows helper with an in-memory endpoint that validates the 11-byte
command and returns CRC-valid feedback. The ROS test uses domain 192. Host
tests skip on machines without Windows PowerShell interop.

```bash
PYTHONDONTWRITEBYTECODE=1 \
PYTHONPATH="$PWD/rm_serial_driver/scripts:$PYTHONPATH" \
/usr/bin/python3 -m pytest -p no:cacheprovider \
  rm_serial_driver/test/test_protocol.py \
  rm_serial_driver/test/test_windows_serial.py \
  rm_serial_driver/test/test_windows_duplex.py \
  rm_serial_driver/test/test_windows_duplex_node.py \
  rm_serial_driver/test/test_windows_duplex_fault_recovery.py \
  rm_serial_driver/test/test_windows_duplex_host.py -q
```

The simulated checks cover binary fragmentation, feedback while an
acknowledgement is missing, latest-only replacement, 100 ms writes despite UTC
offsets of +/-1 hour, startup-handshake delay deducted from the mapped Windows
deadline, delayed commands rejected by Windows, missing/corrupt handshakes,
missing acknowledgements, write failure, ROS feedback/command flow, and
reopening without replaying the cached command.
Additional regressions cover an old clock mapping after a long idle period,
missing clock pulses while feedback continues, and deadlines that must not
accumulate elapsed WSL uptime.
An independent periodic memory endpoint additionally covers 500 ms write stalls,
superseded requests, stopping with a queued write, and replacement after real
helper read/write exceptions. See the
[2026-09-23 repair report](../docs/windows_duplex_stability_fix_2026-09-23.md)
for fault-injection and sustained-load results.
They do not validate the actual flashed firmware, motor execution, serial
line stability, or the unresolved USB/IP enumeration problem.

### First real Windows write probe

On 2026-09-20, the controller reported `mode=3`, interpreted as manual
absolute-angle mode according to the supplied firmware excerpt. The duplex
transport sent three 11-byte commands populated from the latest controller
feedback. Equivalence between those feedback fields and command coordinates
has not been confirmed; this is not a validated hold-position command.
Windows acknowledged all three serial writes. During the probe, 18 normal
38-byte periodic feedback frames remained valid with zero CRC or tail errors.

No 8-byte `B5 + yaw + mode + 5B 5A` controller response was observed. The
provided firmware excerpt defines `vision_link_rx_byte()`, but the available
files do not contain the USB receive-loop call that feeds bytes into it.
Therefore this first probe proved WSL-to-Windows COM delivery, but did not by
itself prove that the flashed STM32 firmware parsed the command. The bridge was
returned to `transport=windows`, `receive_only=true` after the probe.

### First actuated duplex test

Later on 2026-09-21, the controller reported vision mode (`mode=6`) and the
operator authorized a direct dynamic-target test. An explicit temporary relay
forwarded only fresh, finite controller-preview candidates with less than
0.6 rad error on each axis. In 25 seconds it published 59 valid commands; the
serial bridge reported an active command and the measured controller feedback
changed by 0.364 rad in yaw and 0.150 rad in pitch. This physical response
confirms that the flashed controller parsed and executed the 11-byte command,
even though the separate 8-byte response was not observed.

The target was tracked for only 65 of 750 messages because it was outside the
camera's valid view for most of the interval, so this run is not a sustained
dynamic-following acceptance test. After the relay stopped,
`command_active=false`; the bridge was returned to `transport=windows` with
`receive_only=true`. One command expired during the Windows clock handshake and
was dropped without reconnecting, as designed.

A subsequent 60-second run kept the same explicit relay and command limits.
The bridge transmitted 284 valid candidates and remained in controller mode 6,
but the detector saw an armor in only 268/1801 messages and the longest empty
interval was 1310 frames (about 43.7 seconds). The tracker reported tracking in
293/1783 messages. During fresh commands, mean absolute feedback error was
0.0383 rad in yaw and 0.0212 rad in pitch. The detector log contained mostly
`detections=0` during the long gap, with occasional `insufficient_lights` and
reprojection failures when the target re-entered. This run therefore did not
pass sustained-following acceptance.

The same run also produced one Windows write-acknowledgement timeout. Automatic
reconnection restored COM3, but the feedback/TF interruption filled the tracker
message-filter queue and was followed by several negative-dt resets. The bridge
was again returned to the receive-only Windows transport after the run.

The available excerpt sends 8-byte feedback, while live firmware sends 38-byte
feedback. Absence of the old response does not prove that the USB receive
callback is missing, and does not by itself justify reflashing. An offline
compile of the supplied excerpt successfully parsed the current 11-byte
command and attempted an 8-byte response in mode 3. See the
[protocol audit](docs/protocol_audit_2026-09-20.md) for evidence and the remaining
firmware checks.

On 2026-09-20, an isolated package build and all 81 tests passed, including
the real Windows-process tests with a memory-only endpoint (zero skips).
Build, install, and test logs are under `/tmp/rm-duplex-validation-20260920`;
the workspace's normal `build` and `install` directories were not rebuilt.
No real serial port was opened, no USB attachment was changed, and no
hardware command was sent for this validation.

The subsequent monotonic-handshake fix passed 74 focused tests with no skips.
A live single-frame probe then completed an 11-byte Windows write with the
original 100 ms budget, followed by 61 valid feedback frames in mode 3. The
receiver was restored to receive-only mode. This confirms the computer-side
expiry fix for that probe, not STM32 command parsing or sustained 50 Hz writes.
See the [live probe record](docs/live_duplex_probe_2026-09-20.md).

A later firmware update added a receive-triggered buzzer. A non-angle probe
sent 23 NUL bytes at one-second intervals, received 23 matching echoed bytes,
and the user confirmed audible feedback. Basic robot-side USB reception and
bidirectional transport through Windows are now confirmed for that test.
That buzzer-only test did not verify angle-command parsing or motor control. Sending stopped
and the receive-only bridge was restored. See the
[buzzer reception record](docs/buzzer_rx_confirmation_2026-09-20.md).

The subsequent operator-supervised yaw tests confirmed functional angle-command
control in mode 6. At the user's requested 10-degree offset, feedback reached
9.208 degrees and the operator saw rotation and return. All 191 command echoes
matched the transmitted frames. The final short observation retained a
0.564-degree yaw residual; pointing accuracy and independent pitch movement
are not yet validated. The bridge was restored to receive-only. See the
[yaw motion record](docs/yaw_motion_verification_2026-09-20.md).

Independent pitch motion was subsequently tested with the user's confirmed
10-degree head-up clearance. A -10-degree pitch target with 1-second ramps
produced -9.932 degrees of reported travel; the operator saw head-up movement
and return. Maximum yaw change was 0.020 degrees. The short post-send pitch
residual was -0.088 degrees, and a later receive-only sample returned to zero.
All 84 command echoes matched. Both axes now have basic motion verification;
full automatic aiming and pointing accuracy remain separate work. See the
[pitch motion record](docs/pitch_motion_verification_2026-09-20.md).

## Gimbal TF

`gimbal_tf_broadcaster.py` converts the receive-only feedback into this TF
chain:

```text
odom -> yaw_frame -> pitch_joint -> camera_link -> camera_optical_frame
```

It uses the independently verified yaw field and the provisional pitch
calibration. It does not use the yaw-stage IMU quaternion as a complete camera
orientation because that IMU does not follow pitch and its fixed mounting
rotation is not yet calibrated.

The measured translations in `config/gimbal_tf.yaml` are:

```text
yaw_frame -> pitch_joint: (0, 0, 0.178) m
pitch_joint -> camera_link: (0.10423, 0, 0.0228) m
```

The camera mount currently assumes an upright, forward-facing camera. The
standard REP-103 optical rotation is published separately from `camera_link`
to `camera_optical_frame`.

Start only the TF conversion with:

```bash
ros2 launch rm_serial_driver gimbal_tf.launch.py
```

This node subscribes to `/serial_bridge/feedback` and never opens the serial
port or publishes an aim command. The first configuration is suitable for a
level, stationary robot during direction and fixed-board checks. Chassis
attitude, the camera's small mounting-angle error, and final hand-eye refinement
remain outside this mechanical initialization.

## Verification

On 2026-09-16, `auto_aim_interfaces` and `rm_serial_driver` built successfully.
All 64 pytest cases passed: 47 protocol, three Windows-pipe, two
pseudo-terminal serial integration, 11 gimbal-kinematics, and one TF integration
case. They cover fragmented binary reception, open failure, disconnect,
feedback publication/timeouts, ordinary aim encoding, receive-only suppression,
calibrated joint signs, measured translations, and the complete TF frame chain.
Simulated command tests use pseudo-terminals, not the robot.

Real COM3 feedback passed both receive-only checks:

- Windows-to-WSL byte pipe: 205 valid frames in about 10 seconds, zero CRC or
  tail errors, all poses finite with unit quaternions. After one initial
  timestamp/sequence discontinuity, 204 frames were consecutive at 20 Hz.
- Cross-process ROS subscriber: 210 feedback and 209 IMU callbacks in about
  10 seconds. The final 209 feedback frames were consecutive at 50 ms MCU
  intervals. The subscriber stopped at its time boundary; the callback counts
  are not a matched-topic delivery guarantee. Connected and feedback-alive
  became true, command-active stayed false, and there was no aim subscription.

The controller remained in Windows throughout. No protocol commands were
transmitted. USB/IP reset/enumeration compatibility remains unresolved. This
does not validate synchronized image pairing, small camera mounting-angle
errors, final hand-eye accuracy, or the controller's stale-command timeout.
