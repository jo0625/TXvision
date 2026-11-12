import json
import math
from dataclasses import replace

import pytest
import rclpy
from auto_aim_interfaces.msg import Target
from rclpy.parameter import Parameter

from basic_aimer_node import BasicAimerNode
from ballistic_solver import solve_ballistic_pitch
from aimer_geometry import Point3
from impact_speed_calibration import (
    ImpactMeasurement, build_calibration, fit_impact_speed, load_speed_calibration,
    main, read_measurements,
)


def synthetic_shots(speed=18.0):
    """Synthetic flight equations, not real calibration measurements."""
    shots = []
    for index, (distance, pitch_deg) in enumerate([(2.0, 0.0), (3.0, 4.0), (5.0, -2.0)]):
        pitch = math.radians(pitch_deg)
        flight = distance / (speed * math.cos(pitch))
        height = speed * math.sin(pitch) * flight - 0.5 * 9.81 * flight**2
        shots.append(ImpactMeasurement(str(index), "gear_0", distance, pitch_deg, height))
    return shots


def save_calibration(path, shots=None):
    result = build_calibration(shots or synthetic_shots(), b"synthetic test inputs")
    path.write_text(json.dumps(result))
    return result


def test_independent_motion_measurements_recover_speed():
    result = fit_impact_speed(synthetic_shots())
    assert result["speed_mps"] == pytest.approx(18.0)
    assert result["height_residual_rms_m"] == pytest.approx(0, abs=1e-12)
    assert result["individual_speed_mps"] == pytest.approx([18.0] * 3)


def test_lower_impacts_infer_lower_speed_and_more_pitch_compensation():
    shots = synthetic_shots()
    lower = [replace(s, impact_height_m=s.impact_height_m - 0.02) for s in shots]
    speed = fit_impact_speed(lower)["speed_mps"]
    assert speed < 18.0
    assert solve_ballistic_pitch(Point3(5, 0, 0), speed)[0] > solve_ballistic_pitch(Point3(5, 0, 0), 18)[0]


def test_fit_reports_inconsistency_without_hiding_samples():
    shots = synthetic_shots()
    shots[-1] = replace(shots[-1], impact_height_m=shots[-1].impact_height_m - 0.5)
    fit = fit_impact_speed(shots)
    assert fit["height_residual_rms_m"] > 0
    assert len(fit["height_residuals_m"]) == 3
    assert "individual_speed_spread_exceeds_10_percent" in fit["warnings"]


@pytest.mark.parametrize("field,value", [
    ("horizontal_distance_m", 0), ("horizontal_distance_m", -1),
    ("launch_pitch_deg", 90), ("launch_pitch_deg", -90),
    ("launch_pitch_deg", math.nan), ("impact_height_m", math.inf),
    ("impact_height_m", 0), ("impact_height_m", 0.1),
    ("shot_id", ""), ("gear", ""),
])
def test_invalid_measurements_are_rejected(field, value):
    shots = synthetic_shots()
    shots[0] = replace(shots[0], **{field: value})
    with pytest.raises(ValueError):
        fit_impact_speed(shots)


def test_no_fit_from_missing_duplicate_or_mixed_gear_measurements():
    shots = synthetic_shots()
    with pytest.raises(ValueError, match="three"):
        fit_impact_speed(shots[:2])
    with pytest.raises(ValueError, match="unique"):
        fit_impact_speed([shots[0], shots[0], shots[1]])
    with pytest.raises(ValueError, match="gears"):
        fit_impact_speed([shots[0], shots[1], replace(shots[2], gear="gear_1")])
    with pytest.raises(ValueError, match="finite"):
        fit_impact_speed(shots, gravity=math.nan)


def test_template_has_no_fabricated_measurements_and_does_not_overwrite(tmp_path):
    path = tmp_path / "measurements.csv"
    main(["--template", str(path)])
    shots, _ = read_measurements(path)
    assert shots == []
    with pytest.raises(SystemExit) as exc:
        main(["--template", str(path)])
    assert exc.value.code == 2
    with pytest.raises(SystemExit):
        main(["--input", str(path), "--output", str(tmp_path / "calibration.json")])
    assert not (tmp_path / "calibration.json").exists()


def test_calibration_loader_rechecks_gravity_and_recorded_speed(tmp_path):
    path = tmp_path / "calibration.json"
    report = save_calibration(path)
    assert load_speed_calibration(path, 9.81)["speed_mps"] == pytest.approx(18)
    with pytest.raises(ValueError, match="gravity"):
        load_speed_calibration(path, 9.7)
    report["fit"]["speed_mps"] = 23.0
    path.write_text(json.dumps(report))
    with pytest.raises(ValueError, match="saved speed"):
        load_speed_calibration(path, 9.81)
    with pytest.raises(ValueError, match="Cannot load"):
        load_speed_calibration(tmp_path / "missing.json", 9.81)


def test_aimer_uses_calibrated_speed_and_reports_its_provenance(tmp_path, monkeypatch):
    path = tmp_path / "calibration.json"
    report = save_calibration(path)
    rclpy.init()
    node = None
    try:
        node = BasicAimerNode(parameter_overrides=[
            Parameter("aim_frame", value="muzzle"),
            Parameter("gravity_frame", value="muzzle"),
            Parameter("muzzle_frame", value="muzzle"),
            Parameter("ballistics_enabled", value=True),
            Parameter("projectile_speed", value=19.0),
            Parameter("speed_calibration_file", value=str(path)),
            Parameter("publish_diagnostics", value=True),
        ])
        solutions, commands = [], []
        monkeypatch.setattr(node.solution_pub, "publish", lambda msg: solutions.append(json.loads(msg.data)))
        monkeypatch.setattr(node.publisher, "publish", commands.append)
        target = Target(tracking=True, id="3", armors_num=4, radius_1=0.2, radius_2=0.3)
        target.header.frame_id = "muzzle"
        target.header.stamp = node.get_clock().now().to_msg()
        target.position.x = 5.0
        node.target_callback(target)
        assert commands[-1].valid
        assert commands[-1].pitch == pytest.approx(solve_ballistic_pitch(Point3(4.8, 0, 0), 18)[0])
        assert node.get_parameter("projectile_speed").value == pytest.approx(18)
        data = solutions[-1]
        assert data["projectile_speed_source"] == "impact_fit"
        assert data["speed_calibration_id"] == report["measurements_sha256"]
        assert data["speed_calibration_gear"] == "gear_0"
        assert data["projectile_speed_mps"] == pytest.approx(18)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
