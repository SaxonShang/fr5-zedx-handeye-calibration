"""Regression cases for whole-tag PnP rejection and optional diagnostics."""

import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import cv2
import numpy as np

from handeye import diagnostics
from handeye.board import tag_object_corners
from handeye.dataset import Sample, Session
from handeye.observations import build_observations, fit_pose


BOARD = {"target_type": "aprilgrid", "tagRows": 6, "tagCols": 6, "tagSize": 0.055, "tagSpacing": 0.3}
K = np.array([[733.0, 0, 959.5], [0, 733.0, 599.5], [0, 0, 1.0]])
DISTORTION = np.zeros(5)


def points(ids):
    obj = np.concatenate([tag_object_corners(i, BOARD) for i in ids])
    rvec, tvec = np.array([0.15, -0.12, 0.05]), np.array([-0.1, -0.1, 0.5])
    img = cv2.projectPoints(obj, rvec, tvec, K, DISTORTION)[0].reshape(-1, 2)
    return obj, img, rvec, tvec


def session():
    sample = Sample(1, "images/one.png", [0.0] * 6, np.eye(4))
    return Session(Path("."), BOARD.copy(), K.copy(), DISTORTION.copy(), 1920, 1200, [sample], {})


class WholeTagReviewTest(unittest.TestCase):
    def test_four_tags_with_one_bad_corner_are_rejected(self):
        obj, img, rvec, tvec = points([0, 1, 6, 7])
        img[0] += [5.0, 5.0]
        with self.assertRaisesRegex(ValueError, "too few good complete tags"):
            fit_pose(obj, img, K, DISTORTION, rvec, tvec, 4)

    def test_bad_tag_is_removed_when_enough_good_tags_remain(self):
        obj, img, rvec, tvec = points([0, 1, 6, 7, 12])
        img[0] += [5.0, 5.0]
        _, fitted_t, kept, rmse = fit_pose(obj, img, K, DISTORTION, rvec.copy(), tvec.copy(), 4)
        self.assertEqual(kept.tolist(), list(range(4, 20)))
        self.assertLess(rmse, 1e-6)
        np.testing.assert_allclose(fitted_t, tvec, atol=1e-7)

    def test_fallback_cannot_reaccept_bad_complete_tag(self):
        ids = [0, 1, 6, 7]
        obj, img, rvec, tvec = points(ids)
        img[0] += [5.0, 5.0]
        # Even if RANSAC reports every corner as an inlier, whole-tag checking
        # must reject the corner that the initial fit already identified.
        with patch("handeye.observations.read_image", return_value=np.zeros((1200, 1920), np.uint8)), \
                patch("handeye.observations.detect_board", return_value=(obj, img, ids)), \
                patch("handeye.observations.cv2.solvePnPRansac",
                      return_value=(True, rvec, tvec, np.arange(16).reshape(-1, 1))):
            accepted, rejected = build_observations(session())
        self.assertEqual(accepted, [])
        self.assertEqual(len(rejected), 1)

    def test_fallback_requires_four_complete_tags(self):
        ids = [0, 1, 6, 7]
        obj, img, rvec, tvec = points(ids)
        with patch("handeye.observations.read_image", return_value=np.zeros((1200, 1920), np.uint8)), \
                patch("handeye.observations.detect_board", return_value=(obj, img, ids)), \
                patch("handeye.observations.cv2.solvePnP", return_value=(False, rvec, tvec)), \
                patch("handeye.observations.cv2.solvePnPRansac",
                      return_value=(True, rvec, tvec, np.arange(15).reshape(-1, 1))):
            accepted, rejected = build_observations(session())
        self.assertEqual(accepted, [])
        self.assertEqual(len(rejected), 1)


class DiagnosticsReviewTest(unittest.TestCase):
    def setUp(self):
        self.session = session()
        obj, img, _, _ = points([0, 1, 6, 7])
        self.observations = [{"object_points": obj, "image_points": img, "base_T_flange": np.eye(4)}] * 5
        self.best = {"flange_T_left_camera": np.eye(4), "base_T_board": np.eye(4)}
        self.options = SimpleNamespace(max_scale_error=0.005, max_focal_error=0.003,
                                       max_principal_point_px=2.0, max_distortion_px=1.0)

    def test_each_focal_axis_uses_its_own_uncertainty(self):
        # fx changes more but is inconclusive; fy changes less and is significant.
        # Swapping x/y must not change whether the diagnostic warns.
        for changes, deviations in (([0.02, 0.01], [0.02, 0.0001]),
                                    ([0.01, 0.02], [0.0001, 0.02])):
            k = K.copy()
            k[0, 0] *= 1 + changes[0]
            k[1, 1] *= 1 + changes[1]
            sd = np.zeros(18)
            sd[:2] = np.array(deviations) * 733.0
            result = (0.1, k, np.zeros(5), [], [], sd, [], [])
            with patch("handeye.diagnostics.cv2.calibrateCameraExtended", return_value=result):
                report = diagnostics.intrinsics_from_images(self.observations, self.session, self.options)
            self.assertTrue(any("focal length" in text for text in report["warnings"]), report)

    def test_nonfinite_intrinsic_estimate_does_not_report_ok(self):
        result = (float("nan"), K.copy(), np.zeros(5), [], [], np.zeros(18), [], [])
        with patch("handeye.diagnostics.cv2.calibrateCameraExtended", return_value=result):
            report = diagnostics.intrinsics_from_images(self.observations, self.session, self.options)
        self.assertEqual(report["status"], "failed")
        self.assertTrue(report["warnings"])

    def scale_fit(self, **overrides):
        values = {"success": True, "message": "converged", "x": np.zeros(13), "fun": np.array([3.0, 4.0, 0.0, 0.0])}
        values.update(overrides)
        with patch("scipy.optimize.least_squares", return_value=SimpleNamespace(**values)):
            return diagnostics.board_scale(self.observations, self.best, self.session, self.options)

    def test_scale_rmse_is_per_2d_corner(self):
        report = self.scale_fit()
        self.assertEqual(report["status"], "ok")
        self.assertAlmostEqual(report["pixel_rmse"], np.sqrt(25.0 / 2.0))

    def test_unconverged_scale_fit_is_inconclusive(self):
        report = self.scale_fit(success=False, message="max_nfev reached")
        self.assertEqual(report["status"], "inconclusive")
        self.assertNotIn("board_scale", report)
        self.assertTrue(report["warnings"])

    def test_nonfinite_scale_fit_is_failed(self):
        for bad in ({"x": np.full(13, np.nan)}, {"fun": np.array([np.inf, 0.0])}):
            report = self.scale_fit(**bad)
            self.assertEqual(report["status"], "failed")
            self.assertNotIn("board_scale", report)
            self.assertTrue(report["warnings"])

    def test_scale_optimizer_exception_stays_in_diagnostic(self):
        with patch("scipy.optimize.least_squares", side_effect=ValueError("non-finite starting residual")):
            report = diagnostics.board_scale(self.observations, self.best, self.session, self.options)
        self.assertEqual(report["status"], "failed")
        self.assertTrue(report["warnings"])

    def test_scale_warning_keeps_authoritative_board_dimensions(self):
        values = np.zeros(13)
        values[12] = np.log(1.01)
        report = self.scale_fit(x=values)
        self.assertAlmostEqual(report["board_scale"], 1.01)
        self.assertIn("fixed and accurate", " ".join(report["warnings"]))
        self.assertNotIn("measure the printed board", " ".join(report["warnings"]))
        self.assertEqual(self.session.board, BOARD)
        np.testing.assert_array_equal(self.best["flange_T_left_camera"], np.eye(4))


if __name__ == "__main__":
    unittest.main()
