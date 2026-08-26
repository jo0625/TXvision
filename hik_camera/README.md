# hik_camera

ROS 2 driver for the Hikrobot `MV-CS016-10UC`, using the MVS SDK installed at
`/opt/MVS`.

The first-stage goal is image publication only:

```text
MV-CS016-10UC (BayerRG8) -> MVS conversion -> /image_raw (bgr8)
```

`/camera_info` is disabled until a real calibration YAML is available. Do not
use synthetic intrinsics for real PnP or gimbal control.

## Build

Close MVS before starting the ROS node. The camera can only be opened by one
process at a time.

```bash
source /opt/ros/humble/setup.bash
cd /home/jo/my-project/rm_vision/rm_auto_aim
colcon build --symlink-install --packages-select hik_camera
source install/setup.bash
```

## Run

```bash
ros2 launch hik_camera hik_camera.launch.py
```

Verify:

```bash
ros2 topic info /image_raw -v
ros2 topic hz /image_raw
```

In RViz, add an `Image` display with:

```text
Topic: /image_raw
Reliability: Best Effort
```

## Initial parameters

```text
serial_number: DA8541046
frame_rate: 60 FPS
exposure_time_us: 5000
gain_db: 8
pixel format: BayerRG8 (converted to bgr8 before publication)
```

After camera calibration, set `publish_camera_info: true` and provide a
`camera_info_url` such as `file:///absolute/path/camera.yaml`.
