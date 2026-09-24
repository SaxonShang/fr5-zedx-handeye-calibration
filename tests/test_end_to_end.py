"""End to end on rendered Kalibr Aprilgrid sessions: detect -> PnP -> hand-eye -> gate."""

import contextlib
import io
import json
import tempfile
import unittest
from pathlib import Path

import cv2
import numpy as np

from handeye.__main__ import main as cli
from handeye.board import detect_board, detector, read_board, tag_object_corners
from handeye.dataset import Sample, Session, load_session
from handeye.geometry import fr5_from_matrix, pose_from_fr5, read_image, rotation_angle_degrees, write_image
from handeye.observations import build_observations
from handeye.solver import Options, calibrate
from sim import synthetic

TARGET = Path(__file__).resolve().parents[1] / "templates" / "session" / "target.yaml"
FAST = Options(bootstrap=8)


def error_to_truth(result):
    estimate = np.asarray(result["chosen"]["flange_T_left_camera"])
    truth = synthetic.TRUTH_FLANGE_T_CAMERA
    return (np.linalg.norm(estimate[:3, 3] - truth[:3, 3]) * 1000,
            rotation_angle_degrees(truth[:3, :3].T @ estimate[:3, :3]))


class EndToEndTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        cls.clean = synthetic.make_session(Path(cls.tmp.name) / "clean", TARGET, poses=15, seed=3)

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def make(self, name, **overrides):
        return synthetic.make_session(Path(self.tmp.name) / name, TARGET, poses=15, seed=3, **overrides)

    def test_detected_corners_match_board_model(self):
        # Kalibr's gap squares make every corner an X-junction; corners must still land on the model.
        board = read_board(TARGET)
        texture, tex_to_board = synthetic.board_texture(board)
        rng = np.random.default_rng(4)
        k = synthetic.K_ZEDX_2_2MM
        aruco = detector()
        for _, camera_t_board, _ in synthetic.sample_views(rng, 3, board, k, 1.0, False):
            image = synthetic.render_view(texture, tex_to_board, camera_t_board, k, rng, 0.6, 1.5)
            obj, pts, ids = detect_board(image, board, aruco)
            self.assertGreaterEqual(len(ids), synthetic.MIN_VISIBLE_TAGS)
            rvec, _ = cv2.Rodrigues(camera_t_board[:3, :3])
            projected, _ = cv2.projectPoints(obj, rvec, camera_t_board[:3, 3], k, None)
            self.assertLess(np.median(np.linalg.norm(projected.reshape(-1, 2) - pts, axis=1)), 0.1)

    def test_clean_session_is_accepted_and_recovers_truth(self):
        result = calibrate(load_session(self.clean), FAST)
        self.assertEqual(result["status"], "accepted", result["reasons"])
        mm, deg = error_to_truth(result)
        self.assertLess(mm, 0.5)
        self.assertLess(deg, 0.05)
        self.assertLess(result["uncertainty"]["translation_sd_norm_mm"], 0.2)
        self.assertEqual(result["pose_joint_consistency"]["suspect_rows"], [])

    def test_cli_writes_result_and_signals_rejection(self):
        with contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(cli(["solve", "--session", str(self.clean), "--bootstrap", "0"]), 0)
            wrong = self.make("euler", euler_order="xyz")
            self.assertEqual(cli(["solve", "--session", str(wrong), "--bootstrap", "0"]), 2)
        result = json.loads((wrong / "result.json").read_text(encoding="utf-8"))
        self.assertEqual(result["status"], "rejected")
        self.assertTrue(result["reasons"])

    def test_mount_cad_check_rejects_a_distant_result(self):
        truth = synthetic.TRUTH_FLANGE_T_CAMERA[:3, 3] * 1000
        options = Options(bootstrap=0, expected_translation_mm=tuple(truth + [30.0, 0.0, 0.0]))
        result = calibrate(load_session(self.clean), options)
        self.assertEqual(result["status"], "rejected")
        self.assertIn("mount estimate", " ".join(result["reasons"]))

    def test_single_rotation_axis_is_insufficient(self):
        result = calibrate(load_session(self.make("single", single_axis=True)), FAST)
        self.assertEqual(result["status"], "insufficient_data")
        self.assertIn("distinct axes", result["reasons"][0])

    def test_view_with_one_tag_row_is_rejected(self):
        board = read_board(TARGET)
        k = synthetic.K_ZEDX_2_2MM
        rng = np.random.default_rng(5)
        (base_t_flange, camera_t_board, _), = synthetic.sample_views(rng, 1, board, k, 1.0, False)
        texture, tex_to_board = synthetic.board_texture(board)
        image = synthetic.render_view(texture, tex_to_board, camera_t_board, k, rng, 0.6, 1.5)
        row = np.concatenate([tag_object_corners(i, board) for i in range(board["tagCols"])])
        rvec, _ = cv2.Rodrigues(camera_t_board[:3, :3])
        hull = cv2.convexHull(cv2.projectPoints(row, rvec, camera_t_board[:3, 3], k, None)[0]
                              .reshape(-1, 2).astype(np.int32))
        mask = np.zeros_like(image)
        cv2.fillConvexPoly(mask, hull, 255)
        mask = cv2.dilate(mask, np.ones((9, 9), np.uint8))
        image = np.where(mask > 0, image, synthetic.WHITE).astype(np.uint8)
        with tempfile.TemporaryDirectory() as tmp:
            self.assertTrue(write_image(Path(tmp) / "0.png", image))
            pose = fr5_from_matrix(base_t_flange)
            session = Session(Path(tmp), board, k, np.zeros(5), synthetic.WIDTH, synthetic.HEIGHT,
                              [Sample(1, "0.png", pose, pose_from_fr5(pose))], {})
            accepted, rejected = build_observations(session)
        self.assertEqual(accepted, [])
        self.assertIn("1 rows", rejected[0]["reason"])

    def test_session_in_non_ascii_directory(self):
        # cv2.imread/imwrite cannot open non-ASCII Windows paths such as this repository's.
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "标定" / "image.png"
            path.parent.mkdir()
            image = np.arange(64, dtype=np.uint8).reshape(8, 8)
            self.assertTrue(write_image(path, image))
            np.testing.assert_array_equal(read_image(path), image)


if __name__ == "__main__":
    unittest.main()
