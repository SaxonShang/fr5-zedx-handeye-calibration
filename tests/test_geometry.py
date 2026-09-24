"""Frame directions, the FR5 Euler convention, and hand-eye on projected (not rendered) points."""

import unittest
from pathlib import Path

import cv2
import numpy as np

from handeye.board import read_board, tag_object_corners
from handeye.geometry import fr5_from_matrix, inverse, parameters_to_matrix, pose_from_fr5, \
    rotation_angle_degrees, rotation_vector, transform, transform_mean
from handeye.solver import board_poses, check_motion_diversity, handeye, refine, score

TARGET = Path(__file__).resolve().parents[1] / "templates" / "session" / "target.yaml"


def axis_rotation(axis, degrees):
    rvec = np.zeros(3)
    rvec[axis] = np.radians(degrees)
    return cv2.Rodrigues(rvec)[0]


class GeometryTest(unittest.TestCase):
    def test_fr5_pose_units_and_euler_order(self):
        pose = pose_from_fr5([1000, 2000, 3000, 90, 0, 90])
        np.testing.assert_allclose(pose[:3, 3], [1, 2, 3])
        np.testing.assert_allclose(pose[:3, :3], [[0, 0, 1], [1, 0, 0], [0, 1, 0]], atol=1e-12)

    def test_all_three_angles_follow_rz_ry_rx(self):
        # With ry = 0 an Ry in the wrong place would go unnoticed; use three non-zero angles.
        expected = axis_rotation(2, 35) @ axis_rotation(1, -20) @ axis_rotation(0, 50)
        np.testing.assert_allclose(pose_from_fr5([0, 0, 0, 50, -20, 35])[:3, :3], expected, atol=1e-12)
        self.assertEqual(fr5_from_matrix(pose_from_fr5([12.5, -3, 400, 50, -20, 35])),
                         [12.5, -3.0, 400.0, 50.0, -20.0, 35.0])

    def test_rotation_angle_is_accurate_near_zero_and_180(self):
        self.assertAlmostEqual(rotation_angle_degrees(axis_rotation(0, 1e-5)), 1e-5, places=10)
        self.assertAlmostEqual(rotation_angle_degrees(axis_rotation(1, 179.9999)), 179.9999, places=6)

    def test_rotation_vector_below_the_cv2_rodrigues_floor(self):
        tiny = np.array([1e-7, -2e-7, 3e-7])  # cv2.Rodrigues(matrix) returns exactly 0 here
        np.testing.assert_allclose(rotation_vector(cv2.Rodrigues(tiny)[0]), tiny, rtol=1e-6, atol=1e-15)
        general = np.array([0.4, -1.1, 0.7])
        np.testing.assert_allclose(rotation_vector(cv2.Rodrigues(general)[0]), general, atol=1e-12)
        near_pi = np.array([0.0, 0.0, np.pi - 1e-6])  # axis from cv2.Rodrigues: ~1.6e-6 rad off there
        np.testing.assert_allclose(np.abs(rotation_vector(cv2.Rodrigues(near_pi)[0])), near_pi, atol=1e-5)

    def test_kalibr_grid_coordinates(self):
        board = read_board(TARGET)
        pitch = 0.055 * 1.3
        np.testing.assert_allclose(tag_object_corners(0, board)[0], [0.055, 0, 0])
        np.testing.assert_allclose(tag_object_corners(1, board)[0], [pitch + 0.055, 0, 0])
        np.testing.assert_allclose(tag_object_corners(6, board)[0], [0.055, pitch, 0])

    def test_park_and_refinement_on_noisy_projections(self):
        rng = np.random.default_rng(18)
        board = read_board(TARGET)
        objects = np.concatenate([tag_object_corners(i, board) for i in range(36)])
        k = np.array([[733, 0, 960], [0, 733, 600], [0, 0, 1]], dtype=float)  # ZED X 2.2 mm, HD1200
        distortion = np.zeros(5)
        truth_camera = parameters_to_matrix(np.array([0.14, -0.08, 0.11, 0.045, -0.025, 0.085]))
        truth_board = parameters_to_matrix(np.array([0.02, -0.03, 0.04, -0.20, -0.20, 0.60]))
        observations = []
        for _ in range(25):
            robot = parameters_to_matrix(np.r_[rng.uniform(-0.35, 0.35, 3), rng.uniform(-0.08, 0.08, 3)])
            camera_t_board = inverse(robot @ truth_camera) @ truth_board
            rvec, _ = cv2.Rodrigues(camera_t_board[:3, :3])
            pixels, _ = cv2.projectPoints(objects, rvec, camera_t_board[:3, 3], k, distortion)
            pixels = pixels.reshape(-1, 2) + rng.normal(0, 0.12, (len(objects), 2))
            inside = np.all((pixels >= 0) & (pixels < [1920, 1200]), axis=1)
            visible = np.repeat(inside.reshape(-1, 4).all(axis=1), 4)  # whole tags only, like the detector
            view_objects, pixels = objects[visible], pixels[visible]
            ok, observed_rvec, observed_tvec = cv2.solvePnP(view_objects, pixels, k, distortion,
                                                            flags=cv2.SOLVEPNP_ITERATIVE)
            self.assertTrue(ok)
            observations.append({"base_T_flange": robot,
                                 "camera_T_board": transform(cv2.Rodrigues(observed_rvec)[0], observed_tvec),
                                 "object_points": view_objects, "image_points": pixels})
        train, test = observations[:20], observations[20:]
        check_motion_diversity(train)
        estimated = handeye(train, cv2.CALIB_HAND_EYE_PARK)
        # Limits sit a little above the worst of 200 seeds of this setup, not 5-50x above.
        self.assertLess(np.linalg.norm(estimated[:3, 3] - truth_camera[:3, 3]) * 1000, 1.0)
        self.assertLess(rotation_angle_degrees(estimated[:3, :3].T @ truth_camera[:3, :3]), 0.1)
        estimated_board = transform_mean(board_poses(train, estimated))
        test_score = score(test, estimated, estimated_board, k, distortion)
        self.assertLess(test_score["pixel_rmse"], 0.4)
        self.assertLess(test_score["board_position_rmse_mm"], 0.5)
        refined_camera, refined_board = refine(train, estimated, estimated_board, k, distortion)
        self.assertLess(score(test, refined_camera, refined_board, k, distortion)["pixel_rmse"], 0.4)
        self.assertLess(np.linalg.norm(refined_camera[:3, 3] - truth_camera[:3, 3]) * 1000, 0.2)


if __name__ == "__main__":
    unittest.main()
