# Native Ubuntu camera latency baseline (2026-10-12)

## Goal

Keep the restored WSL Ubuntu 22.04 functionality while reducing end-to-end
latency by running the camera and ROS 2 directly on Ubuntu. Removing the
Windows-to-WSL USB forwarding and virtualization path is a reason to expect
improvement, not proof of a specific gain. Compare measured latency under the
same camera settings and workload before claiming an improvement.

## Setup

- Native Ubuntu kernel `6.8.0-138-generic`; `systemd-detect-virt` reports `none`.
- Hikrobot `MV-CS016-10UC`, serial `DA8541046`, direct USB 3.0 at 5 Gbps.
- ROS 2 Humble, `hik_camera`, 1440x1080 BGR8, 3000 us exposure, 12 dB gain.
- `hik_camera/config/camera_params.yaml` was not edited. FPS changes below used
  command-line ROS parameter overrides only.
- RViz preview used Mesa software OpenGL because the default GLX provider
  failed to create a context. RViz display latency was not measured.
- No serial driver, controller, or motor was started.

## Measurements

| Test | Received | Arrival gaps | ROS stamp to subscriber callback |
| --- | --- | --- | --- |
| 10 FPS, 60 s, RViz active | 600 frames, 9.98 FPS | 0 over 200 ms; max 102.6 ms | Separate 150-frame probe: median 14.89 ms, P95 15.78 ms, max 16.72 ms |
| 30 FPS, 60 s, RViz active | 1771 frames, 29.51 FPS | 16 over 66 ms; max 100.5 ms | Median 11.78 ms, P95 13.58 ms, P99 13.99 ms, max 15.03 ms |
| 30 FPS, 30 s, RViz stopped | 895 frames, 29.81 FPS | 5 over 66 ms; max 67.8 ms | Median 13.10 ms, max 16.03 ms |

The camera node logged approximately 30.0 FPS during the 30 FPS tests. The
subscriber still observed frame intervals near two or three frame periods,
including with RViz stopped. The current `sensor_msgs/Image` does not carry the
MVS frame number, so these results do not locate the loss precisely.

The ROS image stamp is assigned **after** `MV_CC_GetImageBuffer` returns and
**before** Bayer-to-BGR conversion and publication. Stamp-to-callback age
therefore includes conversion, ROS transport, and subscriber scheduling, but
excludes exposure, sensor readout, USB transfer, and SDK buffering. It is not
camera-to-screen or camera-to-control latency. At 10 FPS, the 100 ms frame
period alone adds roughly 0-100 ms of event sampling wait; this was a stability
setting, not a production latency target.

## Next checks

1. Add opt-in diagnostics for MVS frame numbers, device/host timestamps, and
   per-stage timings without changing the normal image topic contract.
2. Repeat camera-only 30 FPS tests to distinguish acquisition gaps from DDS or
   subscriber drops; keep RViz separate from the control-path measurement.
3. Progress to 60 and 100 FPS only with bounded memory and CPU monitoring.
   At 1440x1080 BGR8, 100 FPS is about 467 MB/s before ROS copies.
4. Measure detection and tracking P50/P95/P99 latency and drops with the same
   source settings. Test the controller path separately, after camera and vision
   behavior are established.

The 10 FPS camera and RViz preview were restored after the 30 FPS tests.

## Frame-counter diagnosis

The camera node now has an opt-in `diagnostics_enabled` ROS parameter, default
`false`. Every five seconds while enabled, it reports MVS `nFrameNum` gaps,
received and published counts, image-build time, and `publish()` call time.
The camera YAML and `/image_raw` message contract were not changed.

The following comparison used 30 FPS, 1440x1080, the same exposure and gain,
and no RViz or lower controller. Each subscriber ran for 60 seconds.

| Output | SDK frames in logged windows | SDK counter gaps | Subscriber frames | Stamp-to-callback median / P95 / P99 | Stamp gaps over 50 ms |
| --- | --- | --- | --- | --- | --- |
| BGR8, 4,665,600 bytes/frame | 3454 received, 3454 published | 0 | 1797, 29.94 FPS | 11.23 / 13.20 / 13.71 ms | 2; max 66.7 ms |
| BayerRG8, 1,555,200 bytes/frame | 3302 received, 3302 published | 0 | 1799, 29.97 FPS | 3.56 / 3.86 / 4.06 ms | 1; max 66.7 ms |

The SDK totals cover the whole node uptime and are not the exact subscriber
window. They establish that the camera frame counter was continuous throughout
both runs and every received frame was published. The rare subscriber gaps are
therefore downstream of acquisition, but these measurements cannot distinguish
DDS transport, its best-effort depth-1 queue, and Python subscriber scheduling.
One run per format is not enough to claim a change in loss rate.

Typical node-side image-build and `publish()` call means were about 1.2-1.5
and 0.4-0.5 ms for BGR8, versus 0.09-0.11 and 0.2 ms for BayerRG8.
Publishing Bayer avoids the camera-node demosaic and cuts the ROS image payload
to one third. A downstream consumer may still need demosaic, so this is not yet
proof of the same improvement for the complete detector/tracker path.

After this diagnosis, the 10 FPS BGR8 camera and RViz preview were restored.

## YOLO comparison

The camera fed the same warmed `yolo_armor_detector` process on CUDA, with
`publish_debug=false`, no RViz, and a 4 GiB process memory limit. Only the
camera's runtime FPS and `publish_bayer` parameters changed; model, exposure,
gain, image size, and YAML files were unchanged. Each row is a 30-second
subscriber window on `/detector/armors` using best-effort QoS.

| Camera output | Detector results | Image stamp to result callback median / P95 / P99 | Result gaps |
| --- | --- | --- | --- |
| 10 FPS BGR8 | 301, 10.01 FPS | 16.56 / 22.28 / 24.72 ms | 0 over 200 ms |
| 10 FPS BayerRG8 | 301, 10.01 FPS | 9.29 / 10.22 / 10.95 ms | 0 over 200 ms |
| 30 FPS BayerRG8 | 897, 29.90 FPS | 8.36 / 9.91 / 10.56 ms | 2 over 66 ms |
| 30 FPS BGR8 | 901, 30.02 FPS | 13.28 / 21.14 / 22.71 ms | 0 over 66 ms |

All four windows produced **zero nonempty armor messages** because no valid
armor target was in view. The model still ran inference and published an empty
`Armors` message for each processed image, but these numbers exclude successful
target post-processing, PnP, tracker work, and any lower-controller response.
They also exclude exposure, USB transport, and MVS buffering because the image
stamp is assigned after `MV_CC_GetImageBuffer` returns. This single A/B pass
supports Bayer as a lower-latency camera-to-detector transport, not a finalized
production configuration or a measured Windows/WSL-to-Ubuntu improvement.

The YOLO run had no ROS error or warning logs; its memory use remained around
1.37 GiB. Existing Bayer/BGR image-conversion tests passed (7 tests). The YOLO
node was stopped and the 10 FPS BGR8 camera plus RViz preview were restored.
