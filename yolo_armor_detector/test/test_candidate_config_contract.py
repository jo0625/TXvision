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
DEPLOYMENT_MANIFEST = (
    RM_AUTO_AIM / "deployment/yolo_v8_gray140_candidate_20260826/manifest.yaml"
)
CANDIDATE_MODEL_REFERENCE = "${YOLO_ARMOR_MODEL}"


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
    deployment = yaml.safe_load(DEPLOYMENT_MANIFEST.read_text(encoding="utf-8"))

    assert general["type_strategy"] == "fixed"
    assert candidate["type_strategy"] == "normalized_dual_pnp_latch"
    assert candidate["model"] == CANDIDATE_MODEL_REFERENCE
    assert deployment["model"]["environment_variable"] == "YOLO_ARMOR_MODEL"
    assert deployment["model"]["filename"] == "model.pt"
    assert general["armor_type"] == "small"
    assert candidate["armor_type"] == "small"
    assert candidate["publish_debug"] is True
    assert general["gray_threshold"] == 160
    assert candidate["gray_threshold"] == 140
    assert declared_parameter_default("type_strategy") == "fixed"
    assert declared_parameter_default("gray_threshold") == 160
    node_source = NODE_SOURCE.read_text(encoding="utf-8")
    assert "gray_threshold=self.gray_threshold" in node_source
    assert "resolve_model_path(self.model_path)" in node_source


def test_launch_default_remains_generic_fixed_configuration() -> None:
    source = BRINGUP_LAUNCH.read_text(encoding="utf-8")
    assert '[bringup_share, "params", "yolo_detector_params.yaml"]' in source
    assert "yolo_detector_params_v8_frozen_candidate.yaml" not in source
