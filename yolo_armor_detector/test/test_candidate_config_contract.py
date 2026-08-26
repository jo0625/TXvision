#!/usr/bin/env python3

from __future__ import annotations

import ast
from pathlib import Path

import yaml


PACKAGE = Path(__file__).resolve().parents[1]
RM_AUTO_AIM = PACKAGE.parent
GENERAL_PARAMS = RM_AUTO_AIM / "auto_aim_bringup/params/yolo_detector_params.yaml"
CANDIDATE_PARAMS = (
    RM_AUTO_AIM
    / "auto_aim_bringup/params/yolo_detector_params_v8_frozen_candidate.yaml"
)
NODE_SOURCE = PACKAGE / "scripts/yolo_armor_detector_node.py"
BRINGUP_LAUNCH = RM_AUTO_AIM / "auto_aim_bringup/launch/yolo_auto_aim.launch.py"
CURRENT_FROZEN_MODEL = (
    "/home/jo/my-project/my_first_autoaim_yolo/models/frozen/"
    "armor_pose_real_blank_v1_candidate_20260812/model.pt"
)


def load_parameters(path: Path) -> dict:
    return yaml.safe_load(path.read_text(encoding="utf-8"))["yolo_armor_detector"][
        "ros__parameters"
    ]


def declared_parameter_default(parameter_name: str):
    tree = ast.parse(NODE_SOURCE.read_text(encoding="utf-8"))
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call) or len(node.args) < 2:
            continue
        function = node.func
        if not isinstance(function, ast.Attribute) or function.attr != "declare_parameter":
            continue
        name, default = node.args[:2]
        if isinstance(name, ast.Constant) and name.value == parameter_name:
            return default.value if isinstance(default, ast.Constant) else None
    return None


def test_candidate_promotion_configuration_contract() -> None:
    general = load_parameters(GENERAL_PARAMS)
    candidate = load_parameters(CANDIDATE_PARAMS)

    assert general["type_strategy"] == "fixed"
    assert candidate["type_strategy"] == "normalized_dual_pnp_latch"
    assert candidate["model"] == CURRENT_FROZEN_MODEL
    assert Path(candidate["model"]).is_file()
    assert general["armor_type"] == "small"
    assert candidate["armor_type"] == "small"
    assert candidate["publish_debug"] is True
    assert general["gray_threshold"] == 160
    assert candidate["gray_threshold"] == 140
    assert declared_parameter_default("type_strategy") == "fixed"
    assert declared_parameter_default("gray_threshold") == 160
    node_source = NODE_SOURCE.read_text(encoding="utf-8")
    assert "gray_threshold=self.gray_threshold" in node_source


def test_launch_default_remains_generic_fixed_configuration() -> None:
    source = BRINGUP_LAUNCH.read_text(encoding="utf-8")
    assert '[bringup_share, "params", "yolo_detector_params.yaml"]' in source
    assert "yolo_detector_params_v8_frozen_candidate.yaml" not in source
