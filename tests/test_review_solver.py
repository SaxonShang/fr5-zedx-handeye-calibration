"""Regression tests for selection isolation and output frame contracts."""
import math
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

from handeye.solver import Options, _serialise, calibrate


class SolverReviewTest(unittest.TestCase):
    def test_failed_test_rejects_cv_winner_without_selecting_runner_up(self):
        observations = [{"row": i + 1, "image": f"{i}.png", "tag_ids": [0, 1, 6, 7],
                         "dropped_tag_ids": [], "pnp_rmse_px": 0.1,
                         "image_points": np.zeros((16, 2))} for i in range(12)]
        session = SimpleNamespace(k=np.eye(3), distortion=np.zeros(5), path=Path("unused"),
                                  camera={}, board={}, samples=[SimpleNamespace(joints_deg=None)] * 12)
        park = np.eye(4)
        daniilidis = np.eye(4)
        daniilidis[0, 3] = 0.001
        candidates = [{"method": method, "flange_T_left_camera": x, "base_T_board": np.eye(4)}
                      for method, x in (("Park", park), ("Daniilidis", daniilidis))]

        def evaluate(views, x, _y, _k, _d):
            value = math.inf if len(views) < 9 and x[0, 3] == 0 else 0.1
            return {"pixel_rmse": value, "board_position_rmse_mm": value, "board_rotation_rmse_deg": value}

        with patch("handeye.solver.build_observations", return_value=(observations, [])), \
             patch("handeye.solver.check_motion_diversity"), \
             patch("handeye.solver.fit_candidates", return_value=candidates), \
             patch("handeye.solver.cross_validated_rmse", return_value={"Park": 0.1, "Daniilidis": 0.2}), \
             patch("handeye.solver.score", side_effect=evaluate), \
             patch("handeye.solver.diagnostics.coverage", return_value={"warnings": []}), \
             patch("handeye.solver.diagnostics.intrinsics_from_images", return_value={"warnings": []}), \
             patch("handeye.solver.diagnostics.bootstrap", return_value={}), \
             patch("handeye.solver.diagnostics.board_scale", return_value={"warnings": []}):
            result = calibrate(session, Options(bootstrap=0))
        self.assertEqual(result["chosen_method"], "Park")
        self.assertEqual(result["status"], "rejected")
        self.assertIn("non-finite held-out", " ".join(result["reasons"]))
        self.assertEqual(result["frames"]["pose_moving"], "flange")
        self.assertEqual(result["schema_version"], 2)

    def test_noncanonical_frames_never_export_false_aliases(self):
        item = {"method": "Park", "flange_T_left_camera": np.eye(4), "base_T_board": np.eye(4),
                "cv_pixel_rmse": 0.1, "train": {}, "test": {}}
        actual = _serialise(item, {"pose_moving": "reported_tcp", "pose_reference": "reported_work_frame"})
        self.assertNotIn("flange_T_left_camera", actual)
        self.assertNotIn("left_camera_T_flange", actual)
        self.assertNotIn("base_T_board", actual)
        self.assertIn("pose_moving_T_left_camera", actual)
        self.assertIn("pose_reference_T_board", actual)

    def test_nonfinite_thresholds_and_mount_values_are_rejected(self):
        for option in ("max_test_px", "max_view_px", "max_board_mm", "max_board_deg", "max_scale_error"):
            for invalid in (math.nan, math.inf, -1.0):
                with self.subTest(option=option, invalid=invalid), self.assertRaises(ValueError):
                    Options(**{option: invalid})
        with self.assertRaises(ValueError):
            Options(expected_translation_mm=(math.nan, 0, 0))
        with self.assertRaises(ValueError):
            Options(bootstrap=-1)


if __name__ == "__main__":
    unittest.main()
