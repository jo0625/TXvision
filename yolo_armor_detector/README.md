# YOLO armor detector debug display

Detection and PnP use the original camera image. Annotated images on
`/detector/result_img` are visualization output only.

Optional `latency_trace_path` records PnP-return timestamps in bounded memory and
writes CSV on clean shutdown. It defaults to disabled and covers both direct and
track-latch PnP paths. See the
[control latency baseline guide](../docs/control_latency_baseline_2026-10-15.md)
for clock/association limits and capture steps.

When `publish_debug` is true, a separate spawned process converts and publishes
debug images. The detector copies the latest annotated BGR image and its header
into one bounded shared-memory slot. If the consumer holds the slot lock, the
detector skips that display handoff instead of waiting. A newer frame replaces
an unconsumed older frame. Drawing and the initial image copy remain in the
detector; ROS message conversion and DDS publication do not.

The worker copies a consistent image/header snapshot, releases the slot, then
resizes, converts and publishes outside the lock. It does no conversion or
publication when there are no subscribers. It uses BEST_EFFORT, KEEP_LAST, depth
1. CUDA stays in the detector process; the worker is started with `spawn`, not
by forking initialized CUDA state. Detector shutdown also stops the worker.

| Parameter | Default | Meaning |
| --- | --- | --- |
| `publish_debug` | `true` | Enable debug rendering and the display worker |
| `debug_max_fps` | `30.0` | Positive upper limit on display publication rate, Hz |
| `debug_max_width` | `960` | Maximum display width in pixels; 0 keeps original size |

Aspect ratio, source timestamp and frame ID are retained. These settings do
not resize model input or change PnP camera intrinsics. Do not pair the resized
debug image with the original CameraInfo for geometric calculations. The image
must fit the shared slot; a larger input resolution recreates the worker with
a larger slot. Worker failures are logged and do not stop armor detection.

The image topic is resolved using the detector's namespace/remappings before
passing it to the worker. The worker node is named `<detector_name>_debug_image`.
Display limits are startup parameters; restart the detector to change them.

The worker removes synchronous publication from the detector callback; it does
not remove total CPU or memory-bandwidth costs. Remaining timing variation and
physical display/actuation latency must still be measured separately.

See [live comparison](../docs/async_debug_split_2026-09-22.md).
