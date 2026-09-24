"""Reading the hand-written session files."""

import tempfile
import unittest
from pathlib import Path

from handeye.dataset import read_camera, read_poses


def write(folder, name, text):
    path = Path(folder) / name
    path.write_text(text, encoding="utf-8")
    return path


class DatasetTest(unittest.TestCase):
    def test_poses_with_comments_and_joints(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = write(tmp, "poses.csv",
                         "# flange in base frame, read from the FR5 WebApp\n"
                         "image,x_mm,y_mm,z_mm,rx_deg,ry_deg,rz_deg,j1_deg,j2_deg,j3_deg,j4_deg,j5_deg,j6_deg\n"
                         "images/a.png,400,0,500,180,0,0,0,-90,90,-90,-90,0\n"
                         "\n"
                         "images/b.png, 410 ,5,495,179.5,1,2,1,-91,89,-92,-90,3\n")
            samples = read_poses(path)
        self.assertEqual([(s.row, s.image) for s in samples], [(1, "images/a.png"), (2, "images/b.png")])
        self.assertEqual(samples[1].pose_mm_deg[0], 410.0)
        self.assertEqual(samples[0].joints_deg, [0, -90, 90, -90, -90, 0])
        self.assertAlmostEqual(samples[0].base_t_flange[2, 3], 0.5)

    def test_joints_are_optional_but_all_or_none(self):
        with tempfile.TemporaryDirectory() as tmp:
            no_joints = write(tmp, "a.csv", "image,x_mm,y_mm,z_mm,rx_deg,ry_deg,rz_deg\n0.png,1,2,3,4,5,6\n")
            self.assertIsNone(read_poses(no_joints)[0].joints_deg)
            partial = write(tmp, "b.csv", "image,x_mm,y_mm,z_mm,rx_deg,ry_deg,rz_deg,j1_deg\n0.png,1,2,3,4,5,6,7\n")
            with self.assertRaisesRegex(ValueError, "or none"):
                read_poses(partial)

    def test_bad_rows_name_the_row(self):
        with tempfile.TemporaryDirectory() as tmp:
            repeated = write(tmp, "a.csv", "image,x_mm,y_mm,z_mm,rx_deg,ry_deg,rz_deg\n"
                                           "0.png,1,2,3,4,5,6\n0.png,1,2,3,4,5,6\n")
            with self.assertRaisesRegex(ValueError, "row 2"):
                read_poses(repeated)
            typo = write(tmp, "b.csv", "image,x_mm,y_mm,z_mm,rx_deg,ry_deg,rz_deg\n0.png,1,2,3,4,5,six\n")
            with self.assertRaisesRegex(ValueError, "row 1"):
                read_poses(typo)

    def test_init_creates_a_fillable_session(self):
        import contextlib
        import io

        from handeye.__main__ import main as cli
        from handeye.dataset import load_session
        from handeye.validation import read_touch_points
        with tempfile.TemporaryDirectory() as tmp:
            session = Path(tmp) / "s001"
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(cli(["init", "--session", str(session), "--intrinsics", "735.2", "734.9",
                                      "961.3", "598.7"]), 0)
            self.assertTrue((session / "images").is_dir())
            self.assertEqual(read_camera(session / "camera.yaml")["cx"], 961.3)
            self.assertEqual(read_touch_points(session / "touch_points.csv"), [])
            with self.assertRaisesRegex(ValueError, "no pose rows"):
                load_session(session)  # header only until rows are added
            with (session / "poses.csv").open("a", encoding="utf-8") as handle:
                handle.write("images/0001.png,512.34,-35.12,610.45,-176.32,8.14,31.5,"
                             "-12.3,-85.4,92.1,-96.8,-88.5,40.2\n")
            loaded = load_session(session)
            self.assertEqual((loaded.k[0, 0], len(loaded.samples)), (735.2, 1))
            with contextlib.redirect_stderr(io.StringIO()):
                self.assertEqual(cli(["init", "--session", str(session)]), 1)  # never overwrites

    def test_camera_needs_intrinsics(self):
        with tempfile.TemporaryDirectory() as tmp:
            good = write(tmp, "a.yaml", "image_width: 1920\nimage_height: 1200\nfx: 733\nfy: 733\ncx: 960\ncy: 600\n")
            self.assertEqual(read_camera(good)["fx"], 733)
            with self.assertRaisesRegex(ValueError, "cy"):
                read_camera(write(tmp, "b.yaml", "image_width: 1920\nimage_height: 1200\nfx: 733\nfy: 733\ncx: 960\n"))
            with self.assertRaisesRegex(ValueError, "4, 5, 8, 12 or 14"):
                read_camera(write(tmp, "c.yaml", "image_width: 1920\nimage_height: 1200\nfx: 733\nfy: 733\n"
                                                 "cx: 960\ncy: 600\ndistortion: [0, 0, 0]\n"))


if __name__ == "__main__":
    unittest.main()
