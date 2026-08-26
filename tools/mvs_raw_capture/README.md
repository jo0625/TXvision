# MVS DirectRaw Capture Tools

This directory contains a standalone production capture CLI and an offline Bayer converter. It is
not part of the `hik_camera` ROS node and does not perform inference or real-time video encoding.

## Capture

```bash
build/mvs_raw_capture \
  --duration 60 \
  --output-dir /path/to/captures \
  --session-name armor_001
```

The camera target is fixed to `DA8541046 / MV-CS016-10UC`. Every camera open applies and reads back
the validated volatile baseline: 1440x1080 BayerRG8, frame-rate enable/250 FPS, manual 3000 us
exposure, manual 12.4932 dB gain, trigger off, and continuous acquisition. The gate also requires
payload 1555200, TransferSize 1048576, TransferWays 8, and usbfs memory 16. No UserSet or USB Set API
is used.

Two data budgets are reported before SDK initialization:

- Current-environment reference: `duration * 98,000,000 bytes/s`.
- 250 FPS safety bound: `duration * 1,555,200 bytes/frame * 250 frames/s`.

The startup disk gate uses the safety bound plus a 1 GiB reserve. All rate multiplication, duration
estimation, and reserve addition reject integer overflow. For five seconds, the reference is about
0.49 GB, the 250 FPS bound is 1.944 GB, and the required free space is about 3.02 GB after adding the
reserve. Actual file size depends on the frame rate received by the host. Existing session
directories are never reused.

Exit status is `0` only for a complete capture, `1` for a setup/runtime/incomplete-session failure,
and `2` for invalid CLI arguments. `--help` exits `0` without initializing the MVS SDK.

## Session format

Each session directory contains:

- `session.json`: atomically replaced state manifest; status is `incomplete` or `complete`, and it
  references `capture_result.json`.
- `capture_result.json`: durable capture metrics, error counts, disk budgets, release status, and
  final result.
- `initial_camera_parameters.json`: all getter results before volatile baseline application.
- `applied_camera_parameters.json`: all getter results after the 11 authorized Set calls.
- `frames.bayer`: fixed-size contiguous BayerRG8 frames with no per-frame header.
- `frames.csv`: one row per stored frame.
- `counter_events.csv`: sensor counter gap/reset/repeat diagnostics.

`frames.csv` fields:

```text
acquired_index,sensor_frame_id,sensor_counter_source,sensor_timestamp,
host_receive_timestamp_ns,file_byte_offset,payload_size,crc32,counter_event,
previous_sensor_frame_id,missing_count
```

`acquired_index` is the only ordering and naming key. It starts at zero and is contiguous. Sensor
counters are diagnostic only and may gap or reset. A full queue aborts the capture and leaves the
session incomplete instead of silently creating an acquired-index hole.

The initial manifest is durably written as incomplete before SDK initialization. SIGINT, parameter
gate failure, queue overflow, SDK failure, or write/sync/close failure leaves it incomplete. After
the Bayer, frame CSV, and event CSV files are fdatasynced and closed, the tool verifies file size and
CSV continuity, releases the camera, and atomically writes `capture_result.json`. Only after that
write succeeds and every completion gate passes may `session.json` change atomically to complete.
Failure to persist the result keeps the session incomplete.

## Offline conversion

```bash
build/mvs_bayer_convert \
  --session-dir /path/to/captures/armor_001 \
  --output-dir /path/to/exported_images
```

Use `--indices 0,120,240` to export selected acquired indices. Input order may be arbitrary, but the
converter rejects duplicates and exports in ascending `acquired_index` order. Output names are
`frame_<acquired_index>.png`; sensor IDs are never used for ordering or names. `metadata.csv`
preserves the full source row and counter event fields in the same ascending order. Incomplete
sessions are rejected by default; `--allow-incomplete` is an explicit forensic override.

The converter returns `0` for a completed export, `1` for an input/conversion failure, and `2` for
invalid CLI arguments. It never initializes the MVS SDK.

## Offline tests

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

The FPS_DIAG_26 spot check uses only three requested frame offsets:

```bash
build/test_diag26_decode \
  --bayer ../../frame_rate_diagnostics/FPS_DIAG_26_RAW_BAYER_60S_STABILITY/results/direct_raw_60s.bayer \
  --csv ../../frame_rate_diagnostics/FPS_DIAG_26_RAW_BAYER_60S_STABILITY/results/direct_raw_60s.csv \
  --indices 0,1901,3803
```
