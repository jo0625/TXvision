# YOLO v8 Gray140 Candidate Freeze

Date: 2026-08-26

## Status

`WSL_VALIDATED_CANDIDATE_FROZEN_FOR_PORTABLE_DEPLOYMENT`

This freeze establishes an immutable WSL baseline for later mini-PC parity and
full-chain testing. It does not claim production acceptance on the mini PC.

## Frozen Identity

| Component | Frozen value |
|---|---|
| Release | `yolo_v8_gray140_candidate_20260826` |
| Git tag | `yolo-v8-gray140-candidate-20260826` |
| Model identity | `armor_pose_real_blank_v1_candidate_20260812` |
| Model size | `6411992` bytes |
| Model SHA-256 | `9beedce9691e7a602b2912ae617aca991622c1ef3c99107e7a59e0120b9c3dc2` |
| Model location parameter | `${YOLO_ARMOR_MODEL}` |
| Confidence | `0.40` |
| Type strategy | `normalized_dual_pnp_latch` |
| Gray threshold | `140` |
| D boundary | `< 0.0326 small; otherwise large` |

The complete machine-readable definition is
`deployment/yolo_v8_gray140_candidate_20260826/manifest.yaml`. The model binary
is deliberately outside ordinary Git and must be transferred separately.

## Accepted Evidence

- `GRAY140_CANDIDATE_CONFIGURATION_IMPLEMENTED_OFFLINE_VERIFIED`
- `GRAY140_CANDIDATE_LIVE_YAW_SWEEP_PASS`
- `DETECTOR_OUTPUT_LATENCY_BASELINE_ESTABLISHED`
- Stable live endpoints: front 60/60, user-confirmed left45 57/57,
  user-confirmed right45 60/60, return front 50/50 for candidate, green, large,
  and selected PnP valid output.
- Offline output-age baseline: 5176 records, median 42.52 ms, P95 73.97 ms,
  maximum 504.32 ms, and no negative ages.

## Deployment Boundary

Frozen now:

- Detector, tracker time guard, target watchdog, launch and tests.
- Exact model identity and complete candidate YAML behavior.
- WSL acceptance evidence and latency baseline.

Deferred until mini-PC architecture is known:

- PyTorch/CUDA, CPU, OpenVINO, ROCm, TensorRT or other inference backend.
- Native Linux camera transport throughput.
- Clean-checkout parity, camera smoke and complete control-chain acceptance.

No new model training, exposure change, PnP threshold change or repeated armor
capture is authorized by this freeze.
