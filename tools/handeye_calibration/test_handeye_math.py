#!/usr/bin/env python3

import math
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

from handeye_math import (
    camera_link_from_optical,
    fixed_board_residuals,
    invert_transform,
    make_transform,
    rotation_error_deg,
    solve_handeye,
    world_from_pitch,
)


class HandEyeMathTest(unittest.TestCase):
    def test_transform_inverse(self):
        transform = make_transform(
            Rotation.from_euler("xyz", (0.2, -0.1, 0.3)).as_matrix(),
            np.asarray((0.1, -0.2, 0.3)),
        )
        np.testing.assert_allclose(transform @ invert_transform(transform), np.eye(4), atol=1e-12)

    def test_world_from_pitch_uses_rotated_joint_offset(self):
        transform = world_from_pitch(math.pi / 2.0, 0.0, (0.0, 0.0, 0.178))
        np.testing.assert_allclose(transform[:3, 3], (0.0, 0.0, 0.178), atol=1e-12)
        np.testing.assert_allclose(
            transform[:3, :3], Rotation.from_euler("z", math.pi / 2.0).as_matrix(), atol=1e-12
        )

    def test_synthetic_handeye_solution_and_fixed_board_consistency(self):
        pitch_from_camera = make_transform(
            Rotation.from_euler("xyz", (0.015, -0.025, 0.01)).as_matrix()
            @ camera_link_from_optical()[:3, :3],
            np.asarray((0.10423, -0.002, 0.0228)),
        )
        world_from_board = make_transform(
            Rotation.from_euler("xyz", (math.pi / 2.0, 0.1, -0.2)).as_matrix(),
            np.asarray((1.1, 0.05, 0.25)),
        )
        angles = [
            (-0.22, -0.10),
            (-0.12, 0.08),
            (0.0, 0.0),
            (0.13, -0.09),
            (0.24, 0.11),
            (-0.18, 0.06),
            (0.19, -0.04),
        ]
        robot = [world_from_pitch(yaw, pitch) for yaw, pitch in angles]
        target = [
            invert_transform(world_pose @ pitch_from_camera) @ world_from_board
            for world_pose in robot
        ]
        solved = solve_handeye(robot, target, "PARK")
        self.assertLess(
            np.linalg.norm(solved[:3, 3] - pitch_from_camera[:3, 3]), 1e-7
        )
        self.assertLess(
            rotation_error_deg(solved[:3, :3], pitch_from_camera[:3, :3]), 1e-6
        )
        _, residuals = fixed_board_residuals(robot, solved, target)
        self.assertLess(max(item["translation_mm"] for item in residuals), 1e-5)
        self.assertLess(max(item["rotation_deg"] for item in residuals), 1e-6)


if __name__ == "__main__":
    unittest.main()
