# Basic Aimer

实测落点反推等效弹速并用于补偿：见 [落点弹速标定](docs/impact_speed_calibration_zh.md)。

实时画面叠加预测击打点：见 [相机弹道预览](docs/image_preview_zh.md)。

弹道接入、参数配置与启动方式见 [中文说明](docs/ballistics_zh.md)。
The standalone gravity solver is `scripts/ballistic_solver.py`. Launch
`ballistic_aimer.launch.py` with explicit `projectile_speed` and `muzzle_frame`
arguments to enable it; `basic_aimer.launch.py` retains the geometric default.

The node reconstructs armor centers from `auto_aim_interfaces/Target`, projects
the state by the configured fixed prediction time, transforms each center into
`aim_frame`, selects the closest forward armor, and publishes yaw and pitch as
`auto_aim_interfaces/AimCommand`.

The gimbal-frame convention is ROS REP-103: x forward, y left, z up. Therefore
positive yaw points left and positive pitch points up. Controller signs and
zero offsets are intentionally not applied until the real protocol is checked.

Optional `ballistics_enabled` replaces the geometric pitch with the low-arc
solution for constant gravity and configured `projectile_speed` (m/s). It reports
an invalid command if the target is unreachable. The basic launch default remains
geometric. The configured speed is now 14.7 m/s, a preliminary effective-speed
estimate accepted by the user from two 2 m impact groups; see the
[measurement record](docs/impact_speed_estimate_2026-11-11.md). Its source is
`impact_estimate`, and it is not a chronograph measurement. The historical gear
table estimates 19 m/s at 5980 rpm and 23 m/s at 6580 rpm; there is no automatic
gear feedback. The muzzle offset is based on the user's approximate level-pose
measurements.
To enable it, set `ballistics_enabled: true`, provide a positive configured speed,
and provide a `muzzle_frame` TF at the physical projectile exit. The
`gravity_frame` (default `odom`) must have z pointing upward relative to gravity.
The node solves in that frame and transforms the launch direction back to
`aim_frame`, so the existing pitched `pitch_joint` command frame can be used.
Configure the measured `pitch_to_muzzle_*` offset and `muzzle_frame` in
`rm_serial_driver/config/gimbal_tf.yaml`, then use the same `muzzle_frame` in
`rm_aimer/config/basic_aimer.yaml` with `aim_frame: pitch_joint` and the measured
`projectile_speed`. A missing muzzle TF produces `valid=false`.
The solver predicts target translation
and rotation over the computed flight time, repeating the low-arc solution up
to 10 times until flight time changes by less than 1 ms. It publishes
`valid=false` if this does not converge. `prediction_time` remains a fixed
additional lead; it is not inferred from the target timestamp. The solver does
not model drag or changing target velocity. TF is evaluated at the target
timestamp, so later movement of the muzzle is not predicted.

Missing transforms,
invalid tracker state, unsupported armor count, non-finite values, and stale
targets all produce `valid=false`.

```bash
ros2 launch rm_aimer basic_aimer.launch.py
```

Launching this package only publishes `/aimer/aim_command`. It does not connect
that topic to `/serial_bridge/aim_command` and cannot move the robot by itself.

## Controller Angle Preview

`controller_command_preview.py` addresses the different angle conventions in
the supplied controller excerpt. It reads geometric commands and feedback,
then publishes candidate continuous yaw and encoder-relative pitch on
`/controller_command_preview/command`. The message's `valid` flag indicates
whether that specific calculation succeeded. Invalid inputs and timeouts
publish `valid=false`; their default zero angles must never be sent as targets.
`/controller_command_preview/status` reports diagnostic text, including
`candidate_unverified` for usable numeric previews. It is a separate ROS topic
and must not be used as the validity flag for an independently delivered angle
message. The explicit test relay checks each message's `valid` flag before
forwarding it. This node never opens a serial port and has no publisher on
`/serial_bridge/aim_command`; starting it alone cannot move the robot.

The geometric command must be expressed in `pitch_joint`, the existing pitched
frame at the pitch-axis origin. Do not feed camera-optical or `gimbal_link`
angles into this conversion. For a later observation session, start the aimer
with its usual configuration plus `config/controller_preview.yaml`, and start
the preview node with that same override. Existing launch defaults are unchanged:

```bash
ros2 run rm_aimer basic_aimer_node.py --ros-args \
  --params-file rm_aimer/config/basic_aimer.yaml \
  --params-file rm_aimer/config/controller_preview.yaml
```

In another terminal with the same ROS environment:

```bash
ros2 run rm_aimer controller_command_preview.py --ros-args \
  --params-file rm_aimer/config/controller_preview.yaml
```

Avoid starting another aimer if one already owns `/aimer/aim_command`.
The tracker, current feedback, and `odom -> yaw_frame -> pitch_joint` TF must
already be available. The basic aimer now preserves the target header timestamp
used for its TF lookup instead of replacing it with computation time. The
preview matches feedback from its short history against that timestamp and
rejects wrong frames, missing/stale/future stamps, unmatched feedback, invalid
geometry, and input timeouts. This does not solve MCU/camera clock alignment;
feedback stamps are still host arrival times and this preview is for slow or
stationary observations only until timing is measured.

The conversion first reconstructs the ray from geometric yaw/elevation, then
rotates it by the current calibrated pitch into the yaw frame. It computes
the yaw correction from that ray and adds it to the current continuous yaw,
without wrapping the result to +/-pi. It converts the target pitch joint angle
back using the inverse of `pitch_ros = (pitch_raw - offset) / scale`. Positive
geometric elevation is up; positive ROS rotation about +y is down. Independent
addition of two angles is incorrect for combined off-axis yaw/pitch motion.

The YAML reuses provisional calibration values; they must match the active
gimbal TF. The preview assumes that feedback yaw really is the continuous
controller yaw (including turns), that pitch feedback and the command use the
same encoder coordinate, and that yaw has the configured sign. These assumptions
are not established by the live 38-byte feedback layout or the older 8-byte
firmware excerpt. No deployment command adapter, mechanical limit enforcement,
barrel offset or ballistic correction is enabled by this preview.

See [protocol findings](../rm_serial_driver/docs/protocol_audit_2026-09-20.md)
for what has and has not been established about the flashed firmware.

## Temporary Target Deadband

The supervised `tools/dynamic_follow_test.py` relay accepts
`--angle-deadband-deg 0.5` for the temporary 0.5-degree experiment. The default
is 0 (disabled); this option does not change detector, tracker, or deployment
parameters. It takes effect only when that motion-test relay is started.

Yaw and pitch are checked independently against their last forwarded values.
Changes strictly below the threshold keep that axis unchanged; changes at or
above it update that axis. Yaw differences use the shortest angular distance.
Small steps accumulate relative to the held target, so slow motion is not
discarded forever. The first valid target passes through unchanged.

For every fresh valid input, the relay still publishes a valid command with
the current input header, even when holding both angles. Invalid targets,
input/feedback timeouts, and a non-vision feedback mode clear the held values;
the original validity and feedback-angle checks still apply. Raw preview and
forwarded angles remain separate in the record, and its summary includes the
chosen threshold.

This experiment can suppress small target jitter but also introduces a
stepwise target and a per-axis difference below the selected threshold while
holding. It does not repair USB failures, lost tracking, or total system delay.
