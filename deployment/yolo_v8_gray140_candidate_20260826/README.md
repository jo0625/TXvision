# YOLO v8 Gray140 Candidate 2026-08-26

This directory defines the portable boundary of the WSL-validated candidate.
The directory name records the model line (`yolo_v8`), the full candidate
post-processing profile (`gray140`), its pre-production status (`candidate`),
and the freeze date (`20260826`).

## Included In Git

- `manifest.yaml`: model identity, one model checksum, complete candidate
  configuration identity, and validated verdicts.
- The candidate parameter file under `auto_aim_bringup/params/`.
- Detector, tracker, watchdog, launch, and test source code.

## Kept Outside Git

`model.pt` is an external binary artifact. Place the validated file in this
directory after cloning, or keep it in another deployment-owned directory.
Set its path before launching:

```bash
export YOLO_ARMOR_MODEL=/absolute/path/to/model.pt
source install/setup.bash
source /absolute/path/to/runtime-venv/bin/activate
ros2 launch auto_aim_bringup yolo_auto_aim.launch.py \
  yolo_params:=$(ros2 pkg prefix auto_aim_bringup)/share/auto_aim_bringup/params/yolo_detector_params_v8_frozen_candidate.yaml
```

Changing `YOLO_ARMOR_MODEL` changes only the artifact location. The file must
still match the identity and SHA-256 recorded in `manifest.yaml`.

The mini-PC CPU/GPU backend is intentionally not frozen here. That environment
and its runtime venv are selected and validated after the target hardware
architecture is known.
