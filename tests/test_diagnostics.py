"""Diagnostics (board scale, intrinsics, distortion, pose frames, typos) and result checks."""

import tempfile
import unittest
from pathlib import Path

import numpy as np

from handeye.board import board_point, read_board
from handeye.dataset import load_session
from handeye.geometry import parameters_to_matrix
from handeye.solver import Options, calibrate
from handeye.validation import compare_results, kabsch, read_touch_points, validate
from sim import synthetic

TARGET = Path(__file__).resolve().parents[1] / "templates" / "session" / "target.yaml"
QUIET_ON_GOOD_DATA = ("board scale", "focal", "principal point", "residual radial distortion",
                      "user/work frame", "active TCP")


def solve(tmp, name, options=None, seed=7, **overrides):
    session = synthetic.make_session(Path(tmp) / name, TARGET, poses=15, seed=seed, **overrides)
    return session, calibrate(load_session(session), options or Options(bootstrap=0))


def warned(result, *words):
    return any(word in text for text in result["warnings"] + result["reasons"] for word in words)


class DiagnosticsTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def test_good_data_raises_no_diagnostic_warning(self):
        for name, noise in (("clean", {}), ("noisy", {"robot_noise_mm": 0.2, "robot_noise_deg": 0.02})):
            _, result = solve(self.tmp.name, name, **noise)
            self.assertFalse(warned(result, *QUIET_ON_GOOD_DATA), result["warnings"])
        _, result = solve(self.tmp.name, "clean2", seed=8)
        self.assertAlmostEqual(result["board_scale_check"]["board_scale"], 1.0, delta=5e-4)
        self.assertLess(result["intrinsics_check"]["distortion_corner_shift_px"], 0.3)

    def test_residual_distortion_is_flagged(self):
        _, result = solve(self.tmp.name, "distorted", residual_k1=-0.0008)
        self.assertTrue(warned(result, "residual radial distortion"), result["warnings"])

    def test_poses_of_an_active_tcp_are_rejected_unless_allowed(self):
        _, result = solve(self.tmp.name, "tcp", tcp_offset_mm=120.0)
        self.assertEqual(result["status"], "rejected")
        self.assertIn("active TCP", " ".join(result["reasons"]))
        self.assertAlmostEqual(np.linalg.norm(result["pose_joint_consistency"]["flange_offset"]["translation_mm"]),
                               120.0, delta=0.5)
        _, allowed = solve(self.tmp.name, "tcp-allowed", Options(bootstrap=0, allow_tcp_offset=True),
                           tcp_offset_mm=120.0)
        self.assertEqual(allowed["status"], "accepted")
        self.assertTrue(warned(allowed, "active TCP"))

    def test_poses_in_a_work_frame_warn_but_keep_the_hand_eye_result(self):
        _, result = solve(self.tmp.name, "user", user_frame=True)
        self.assertEqual(result["status"], "accepted")
        self.assertTrue(warned(result, "user/work frame"), result["warnings"])
        x = np.asarray(result["chosen"]["flange_T_left_camera"])
        self.assertLess(np.linalg.norm(x[:3, 3] - synthetic.TRUTH_FLANGE_T_CAMERA[:3, 3]) * 1000, 0.5)

    def test_independent_sessions_agree_and_a_changed_board_does_not(self):
        noise = {"robot_noise_mm": 0.2, "robot_noise_deg": 0.02}
        results = [solve(self.tmp.name, f"repeat{seed}", Options(bootstrap=10), seed=seed, **noise)[1]
                   for seed in (21, 22)]
        _, scaled = solve(self.tmp.name, "repeat-scaled", Options(bootstrap=10), seed=23, board_scale=1.01, **noise)
        same, changed = compare_results(results, ["a", "b"])[0], compare_results([results[0], scaled], ["a", "c"])[0]
        self.assertLess(same["translation_mm"], 3 * same["expected_random_mm"] + 0.2)
        self.assertGreater(changed["translation_mm"], 2.0)

    def test_wrong_board_size_is_flagged_and_estimated(self):
        _, result = solve(self.tmp.name, "scale", board_scale=1.01)
        self.assertTrue(warned(result, "board scale"), result["warnings"])
        self.assertAlmostEqual(result["board_scale_check"]["board_scale"], 1.01, delta=1e-3)

    def test_wrong_intrinsics_are_flagged(self):
        _, result = solve(self.tmp.name, "k", k_error=(0.005, 3.0, -3.0))
        self.assertTrue(warned(result, "focal"), result["warnings"])
        self.assertTrue(warned(result, "principal point"), result["warnings"])

    def test_mistyped_pose_row_is_flagged(self):
        _, result = solve(self.tmp.name, "typo", pose_typo=3)
        self.assertEqual([row["row"] for row in result["pose_joint_consistency"]["suspect_rows"]], [4])

    def test_touch_points_agree_with_an_accepted_calibration(self):
        session, result = solve(self.tmp.name, "touch")
        report = validate(np.asarray(result["chosen"]["base_T_board"]), result["board"],
                          read_touch_points(session / "touch_points.csv"))
        self.assertLess(report["rms_mm"], 0.5)
        self.assertLess(report["board_pose_difference"]["translation_mm"], 0.5)


class ValidationTest(unittest.TestCase):
    def test_kabsch_recovers_a_rigid_transform(self):
        truth = parameters_to_matrix(np.array([0.1, -0.2, 0.3, 0.4, -0.1, 0.05]))
        model = np.random.default_rng(1).uniform(-0.2, 0.2, (6, 3))
        measured = (truth[:3, :3] @ model.T + truth[:3, 3:4]).T
        np.testing.assert_allclose(kabsch(model, measured), truth, atol=1e-12)

    def test_offset_touches_show_up_as_error(self):
        board = read_board(TARGET)
        base_t_board = parameters_to_matrix(np.array([0.0, 0.0, 0.1, 0.25, -0.19, 0.0]))
        points = []
        for row, (tag, corner) in enumerate(((0, "BL"), (5, "BR"), (30, "TL"), (35, "TR")), start=1):
            xyz = (base_t_board @ np.r_[board_point(tag, corner, board), 1.0])[:3] * 1000 + [2.0, 0.0, 0.0]
            points.append({"row": row, "tag_id": tag, "corner": corner, "base_mm": xyz.tolist()})
        report = validate(base_t_board, board, points)
        self.assertAlmostEqual(report["rms_mm"], 2.0, places=6)
        self.assertAlmostEqual(report["board_pose_difference"]["translation_mm"], 2.0, places=6)
        with self.assertRaisesRegex(ValueError, "tag 99"):
            validate(base_t_board, board, points + [{"row": 5, "tag_id": 99, "corner": "BL", "base_mm": [0, 0, 0]}])


if __name__ == "__main__":
    unittest.main()
