from pathlib import Path
import hashlib
import json
import yaml

B = Path(__file__).resolve().parent


def check_config(verbose=True):
    settings = json.loads((B / "runtime_paths.json").read_text())
    review = json.loads((B / "configuration_review.json").read_text())
    aimer = yaml.safe_load((B / settings["aimer_params"]).read_text())["aimer"]["ros__parameters"]
    preview_document = yaml.safe_load((B / settings["preview_params"]).read_text())
    preview = preview_document["controller_command_preview"]["ros__parameters"]
    aimer.update(preview_document["aimer"]["ros__parameters"])
    tf = yaml.safe_load(Path(settings["tf_params"]).read_text())["gimbal_tf"]["ros__parameters"]
    tracker = yaml.safe_load(Path(settings["tracker_params"]).read_text())["armor_tracker"]["ros__parameters"]
    r_parameters = {key: tracker["ekf"][key] for key in ("r_xyz_factor", "r_xyz_min", "r_yaw")}
    checks = {
        "pitch_offsets_equal": tf["pitch_raw_offset_rad"] == preview["pitch_raw_offset_rad"],
        "pitch_scales_equal": tf["pitch_raw_scale"] == preview["pitch_raw_scale"],
        "frames_equal": aimer["aim_frame"] == tf["pitch_frame"] == preview["input_frame"],
        "ballistics_enabled": aimer["ballistics_enabled"] is True,
        "prediction_time_zero": aimer["prediction_time"] == 0,
        "tf_timeout_unchanged": aimer["tf_timeout"] == .02,
        "muzzle_frame_equal": aimer["muzzle_frame"] == tf["muzzle_frame"],
        "candidate_parameters_equal": all(tf[key] == value for key, value in review["candidate_parameters"].items()),
    }
    failed = [name for name, passed in checks.items() if not passed]
    if failed:
        raise RuntimeError("Configuration consistency check failed: " + str(failed))
    lock = json.loads((Path(settings["dependency_root"]) / "outputs/tf_deadlock_fix/dependency_lock.json").read_text())
    for name, expected in lock["overlay_files"].items():
        if hashlib.sha256(Path(name).read_bytes()).hexdigest() != expected:
            raise RuntimeError("TF overlay changed: " + name)
    if verbose:
        print("整链启动前检查通过；未启动硬件。")
        print("外参文件：" + settings["tf_params"])
        print("有效瞄准坐标系：" + aimer["aim_frame"])
        print("配置弹速：" + str(aimer["projectile_speed"]) + " m/s")
        print("Tracker 参数文件：" + settings["tracker_params"])
        print("EKF R：" + json.dumps(r_parameters, ensure_ascii=False))
        print("流程：每阶段 7 秒预热、15 秒只接收预检、通过后 60 秒跟随。")
        print(json.dumps(review["candidate_parameters"], ensure_ascii=False, indent=2))
    return checks


if __name__ == "__main__":
    check_config()
