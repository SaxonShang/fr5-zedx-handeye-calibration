"""End-to-end checks on rendered Kalibr Aprilgrid images: detect -> PnP -> solve."""

import argparse
import contextlib
import io
import tempfile
import unittest
from pathlib import Path

import cv2
import numpy as np

import handeye
from sim import synthetic

TARGET = Path(__file__).resolve().parents[1] / "target.example.yaml"


def solve_quietly(session: Path):
    args = argparse.Namespace(session=session, target=None, output=None,
                              min_tags=4, max_pnp_rmse=2.0, no_refine=False)
    with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
        handeye.solve(args)


class EndToEndTest(unittest.TestCase):
    def test_detected_corners_match_board_model(self):
        # Kalibr's gap squares make every corner an X-junction; the corners
        # must still land on the projected board model.
        board = handeye.read_board(TARGET)
        texture, tex_to_board = synthetic.board_texture(board)
        rng = np.random.default_rng(4)
        k = synthetic.K_ZEDX_2_2MM
        views = synthetic.sample_views(rng, 4, board, k, 1.0, False)
        aruco = handeye.detector()
        for _, camera_t_board in views:
            image = synthetic.render_view(texture, tex_to_board, camera_t_board, k, rng, 0.6, 1.5)
            obj, pts, ids = handeye.detect_board(image, board, aruco)
            self.assertGreaterEqual(len(ids), synthetic.MIN_VISIBLE_TAGS)
            rvec, _ = cv2.Rodrigues(camera_t_board[:3, :3])
            projected, _ = cv2.projectPoints(obj, rvec, camera_t_board[:3, 3], k, None)
            errors = np.linalg.norm(projected.reshape(-1, 2) - pts, axis=1)
            self.assertLess(np.median(errors), 0.1)

    def test_clean_session_recovers_truth(self):
        with tempfile.TemporaryDirectory() as tmp:
            session = synthetic.make_session(Path(tmp) / "session", TARGET, poses=15, seed=3)
            solve_quietly(session)
            chosen = synthetic.compare_session(session)["chosen"]
        self.assertLess(chosen["translation_error_mm"], 0.5)
        self.assertLess(chosen["rotation_error_deg"], 0.05)
        self.assertLess(chosen["holdout_pixel_rmse"], 0.2)

    def test_single_rotation_axis_is_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            session = synthetic.make_session(Path(tmp) / "session", TARGET, poses=15, seed=3,
                                             single_axis=True)
            with self.assertRaisesRegex(ValueError, "distinct axes"):
                solve_quietly(session)

    def test_session_in_non_ascii_directory(self):
        # The repository lives under a Chinese path; cv2.imread/imwrite fail there.
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "标定" / "image.png"
            path.parent.mkdir()
            image = np.arange(64, dtype=np.uint8).reshape(8, 8)
            self.assertTrue(handeye.write_image(path, image))
            np.testing.assert_array_equal(handeye.read_image(path), image)


if __name__ == "__main__":
    unittest.main()
