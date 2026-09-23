"""Synthetic check of frame directions, hand-eye solve, and holdout scoring."""

import unittest
from pathlib import Path

import cv2
import numpy as np

from handeye import (
    board_poses, check_motion_diversity, handeye, inverse,
    parameters_to_matrix, pose_from_fr5, read_board, refine, rotation_angle_degrees, score,
    tag_object_corners, transform, transform_mean,
)


class HandeyeSyntheticTest(unittest.TestCase):
    def test_fr5_pose_units_and_euler_order(self):
        pose = pose_from_fr5([1000, 2000, 3000, 90, 0, 90])
        np.testing.assert_allclose(pose[:3, 3], [1, 2, 3])
        np.testing.assert_allclose(pose[:3, :3],
                                   [[0, 0, 1], [1, 0, 0], [0, 1, 0]], atol=1e-12)

    def test_kalibr_grid_coordinates(self):
        board = read_board(Path(__file__).resolve().parents[1] / "target.example.yaml")
        pitch = 0.055 * 1.3
        np.testing.assert_allclose(tag_object_corners(0, board)[0], [0.055, 0, 0])
        np.testing.assert_allclose(tag_object_corners(1, board)[0], [pitch + 0.055, 0, 0])
        np.testing.assert_allclose(tag_object_corners(6, board)[0], [0.055, pitch, 0])

    def test_park_and_holdout_on_noisy_data(self):
        rng = np.random.default_rng(18)
        board = read_board(Path(__file__).resolve().parents[1] / "target.example.yaml")
        objects = np.concatenate([tag_object_corners(i, board) for i in range(36)])
        k = np.array([[1200, 0, 960], [0, 1200, 600], [0, 0, 1]], dtype=float)
        distortion = np.zeros(5)
        truth_flange_t_camera = parameters_to_matrix(np.array([0.14, -0.08, 0.11,
                                                                0.045, -0.025, 0.085]))
        truth_base_t_board = parameters_to_matrix(np.array([0.02, -0.03, 0.04,
                                                            -0.20, -0.20, 0.95]))
        observations = []
        for i in range(25):
            robot = parameters_to_matrix(np.r_[rng.uniform(-0.35, 0.35, 3),
                                                  rng.uniform(-0.08, 0.08, 3)])
            camera_t_board = inverse(robot @ truth_flange_t_camera) @ truth_base_t_board
            rvec, _ = cv2.Rodrigues(camera_t_board[:3, :3])
            pixels, _ = cv2.projectPoints(objects, rvec, camera_t_board[:3, 3],
                                          k, distortion)
            pixels = pixels.reshape(-1, 2) + rng.normal(0, 0.12, (len(objects), 2))
            ok, observed_rvec, observed_tvec = cv2.solvePnP(
                objects, pixels, k, distortion, flags=cv2.SOLVEPNP_ITERATIVE
            )
            self.assertTrue(ok)
            observed_rot, _ = cv2.Rodrigues(observed_rvec)
            observations.append({"index": i, "base_T_flange": robot,
                                 "camera_T_board": transform(observed_rot, observed_tvec),
                                 "object_points": objects, "image_points": pixels})
        train, holdout = observations[:20], observations[20:]
        check_motion_diversity(train)
        estimated = handeye(train, cv2.CALIB_HAND_EYE_PARK)
        position_error_mm = np.linalg.norm(estimated[:3, 3] - truth_flange_t_camera[:3, 3]) * 1000
        angle_error_deg = rotation_angle_degrees(estimated[:3, :3].T @ truth_flange_t_camera[:3, :3])
        self.assertLess(position_error_mm, 3)
        self.assertLess(angle_error_deg, 0.5)
        estimated_board = transform_mean(board_poses(train, estimated))
        heldout = score(holdout, estimated, estimated_board, k, distortion)
        self.assertLess(heldout["pixel_rmse"], 1.5)
        self.assertLess(heldout["board_position_rmse_mm"], 3)
        refined_camera, refined_board = refine(train, estimated, estimated_board, k, distortion)
        refined_holdout = score(holdout, refined_camera, refined_board, k, distortion)
        self.assertLess(refined_holdout["pixel_rmse"], 1.5)
        self.assertLess(np.linalg.norm(refined_camera[:3, 3] -
                                       truth_flange_t_camera[:3, 3]) * 1000, 3)


if __name__ == "__main__":
    unittest.main()
