# hik_camera

ROS 2 driver for the Hikrobot `MV-CS016-10UC`, using the MVS SDK installed at
`/opt/MVS`.

The default path converts the camera's native Bayer image before publication:

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

## High-throughput YOLO path

The default configuration publishes `1440x1080` `bgr8` images so calibration
and capture tools retain their existing input contract. For the YOLO deployment
path, publish the camera's native Bayer data and let the detector convert it
after transport:

```bash
ros2 launch hik_camera hik_camera.launch.py publish_bayer:=true
```

Run the YOLO bringup headless with `publish_debug:=false`; leaving annotated
images enabled adds a full-size BGR image publication and lowers throughput.

```bash
ros2 launch auto_aim_bringup yolo_auto_aim.launch.py publish_debug:=false
```

This changes `/image_raw` to `bayer_rggb8` without changing its resolution,
field of view, timestamps, or camera intrinsics. The YOLO detector supports
this encoding. Tools that explicitly require tightly packed `bgr8`, including
the hand-eye sample collector, must continue using the default `false` value.

On the 2026-09-19 WSL test path, Bayer transport with debug-image publication
disabled and a 1300-byte CycloneDDS fragment produced 26-29 detector messages
per second. The latest 10.033-second stationary real-armor sample produced
292/292 valid messages at 29.00 FPS; average message age was 23.75 ms and the
maximum was 31.27 ms. This is a WSL diagnostic result, not a guarantee for the
final mini PC.
