"""Regression tests for independent validation and result provenance checks."""

import contextlib
import copy
import io
import json
import shutil
import tempfile
import unittest
from pathlib import Path

import numpy as np

from handeye.__main__ import main as cli, print_summary
from handeye.board import board_point, read_board
from handeye.dataset import load_session
from handeye.validation import compare_results, read_touch_points, validate, validate_result
from sim import synthetic

TARGET = Path(__file__).resolve().parents[1] / "templates" / "session" / "target.yaml"


class ReviewValidationTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        cls.root = Path(cls.tmp.name)
        cls.source = synthetic.make_session(cls.root / "source", TARGET, poses=3, seed=17)
        cls.independent = synthetic.make_session(cls.root / "independent", TARGET, poses=4, seed=18)
        cls.session = load_session(cls.independent)
        source = load_session(cls.source)
        cls.board = read_board(TARGET)
        cls.points = read_touch_points(cls.source / "touch_points.csv")
        cls.result = {
            "schema_version": 2, "status": "accepted", "session": str(cls.source),
            "board": cls.board, "camera": source.camera,
            "frames": {"pose_moving": "flange", "pose_reference": "base", "camera": "zed_left_rectified_optical",
                       "board": "kalibr_aprilgrid", "basis": "declared_with_joint_check"},
            "accepted_views": [{"image": sample.image} for sample in source.samples],
            "chosen": {"pose_moving_T_left_camera": synthetic.TRUTH_FLANGE_T_CAMERA.tolist(),
                       "pose_reference_T_board": synthetic.TRUTH_BASE_T_BOARD.tolist()}}
        cls.other = copy.deepcopy(cls.result)
        cls.other.update(session=str(cls.independent),
                         accepted_views=[{"image": sample.image} for sample in cls.session.samples])

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def save(self, name, result):
        path = self.root / name
        path.write_text(json.dumps(result), encoding="utf-8")
        return path

    def invoke(self, arguments):
        with contextlib.redirect_stdout(io.StringIO()) as output, contextlib.redirect_stderr(io.StringIO()):
            code = cli(arguments)
        return code, output.getvalue()

    def test_saved_board_check_cannot_validate_changed_x_but_new_chain_can(self):
        changed = copy.deepcopy(self.result)
        changed["chosen"]["pose_moving_T_left_camera"][0][3] += 0.025
        board_check = validate_result(changed, self.board, self.points)
        self.assertEqual(board_check["scope"], "board_pose_check")
        self.assertLess(board_check["rms_mm"], 0.5)
        chain = validate_result(changed, self.board, self.points, self.session)
        self.assertEqual(chain["scope"], "handeye_chain_check")
        self.assertGreater(chain["rms_mm"], 20)
        path = self.save("changed.json", changed)
        code, output = self.invoke(["validate", "--result", str(path), "--points", str(self.source / "touch_points.csv"),
                                    "--session", str(self.independent)])
        self.assertEqual(code, 2)
        self.assertIn("handeye_chain_check", output)

    def test_new_chain_uses_x_and_new_observations_without_saved_y(self):
        changed = copy.deepcopy(self.result)
        changed["chosen"]["pose_reference_T_board"][0][3] += 1.0
        report = validate_result(changed, self.board, self.points, self.session)
        self.assertLess(report["rms_mm"], 0.5)
        path = self.save("good.json", self.result)
        code, output = self.invoke(["validate", "--result", str(path), "--points", str(self.source / "touch_points.csv"),
                                    "--session", str(self.independent)])
        self.assertEqual(code, 0, output)

    def test_rejected_legacy_and_wrong_frames_never_validate(self):
        for key, value in (("status", "rejected"), ("schema_version", 1), ("frames", {})):
            result = copy.deepcopy(self.result)
            result[key] = value
            path = self.save("bad.json", result)
            code, _ = self.invoke(["validate", "--result", str(path), "--points", str(self.source / "touch_points.csv")])
            self.assertNotEqual(code, 0)
        for key, value in (("pose_reference", "reported_work_frame"), ("pose_moving", "reported_tcp")):
            result = copy.deepcopy(self.result)
            result["frames"][key] = value
            with self.assertRaises(ValueError):
                validate_result(result, self.board, self.points)

    def test_nonfinite_repeated_and_collinear_touches_are_rejected(self):
        y = synthetic.TRUTH_BASE_T_BOARD
        nonfinite = copy.deepcopy(self.points)
        nonfinite[0]["base_mm"][0] = float("nan")
        physical_repeat = copy.deepcopy(self.points)
        physical_repeat[1]["base_mm"] = physical_repeat[0]["base_mm"]
        collinear = []
        for tag in (0, 1, 2):
            xyz = (y @ np.r_[board_point(tag, "BL", self.board), 1])[:3] * 1000
            collinear.append({"row": tag + 1, "tag_id": tag, "corner": "BL", "base_mm": xyz.tolist()})
        for points in (nonfinite, self.points + [self.points[0]], physical_repeat, collinear, self.points[:2]):
            with self.assertRaises(ValueError):
                validate(y, self.board, points)
        path = self.root / "nan-points.csv"
        path.write_text("tag_id,corner,x_mm,y_mm,z_mm\n0,BL,nan,0,0\n", encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "finite"):
            read_touch_points(path)

    def test_validation_rejects_source_session_and_copied_pixels(self):
        with self.assertRaisesRegex(ValueError, "different"):
            validate_result(self.result, self.board, self.points, load_session(self.source))
        copied = self.root / "copy-session"
        shutil.copytree(self.source, copied)
        with self.assertRaisesRegex(ValueError, "reuses"):
            validate_result(self.result, self.board, self.points, load_session(copied))

    def test_comparison_requires_distinct_accepted_sessions_and_frames(self):
        self.assertEqual(len(compare_results([self.result, self.other], ["a.json", "b.json"])), 1)
        for results, names in (([self.result], ["a.json"]),
                               ([self.result, self.other], ["a.json", "./a.json"]),
                               ([self.result, self.result], ["a.json", "b.json"])):
            with self.assertRaises(ValueError):
                compare_results(results, names)
        for edit in (lambda r: r.update(status="rejected"),
                     lambda r: r.pop("frames"),
                     lambda r: r["frames"].update(pose_moving="reported_tcp"),
                     lambda r: r["camera"].update(fx=800),
                     lambda r: r["chosen"]["pose_moving_T_left_camera"][0].__setitem__(0, float("nan"))):
            other = copy.deepcopy(self.other)
            edit(other)
            with self.assertRaises(ValueError):
                compare_results([self.result, other], ["a.json", "b.json"])
        path = self.save("compare-one.json", self.result)
        for paths in ([str(path)], [str(path), str(path)]):
            code, _ = self.invoke(["compare", "--results", *paths])
            self.assertNotEqual(code, 0)

    def test_comparison_rejects_copied_source_under_another_path(self):
        copied = self.root / "compare-copy-session"
        shutil.copytree(self.source, copied)
        other = copy.deepcopy(self.result)
        other["session"] = str(copied)
        with self.assertRaisesRegex(ValueError, "reuse"):
            compare_results([self.result, other], ["a.json", "b.json"])

    def test_transform_aliases_cannot_disagree_with_generic_fields(self):
        for name, value in (("flange_T_left_camera", synthetic.TRUTH_FLANGE_T_CAMERA.copy()),
                            ("left_camera_T_flange", np.linalg.inv(synthetic.TRUTH_FLANGE_T_CAMERA)),
                            ("base_T_board", synthetic.TRUTH_BASE_T_BOARD.copy())):
            changed = copy.deepcopy(self.result)
            value[0, 3] += 0.025
            changed["chosen"][name] = value.tolist()
            with self.assertRaisesRegex(ValueError, "contradicts"):
                validate_result(changed, self.board, self.points)
            with self.assertRaisesRegex(ValueError, "contradicts"):
                compare_results([changed, self.other], ["a.json", "b.json"])

    def test_comparison_checks_recorded_camera_metadata_and_warns_when_missing(self):
        rows = compare_results([self.result, self.other], ["a.json", "b.json"])
        self.assertIn("unrecorded camera metadata", " ".join(rows[0]["warnings"]))
        for key, values in (("camera_serial", (100, 200)), ("zed_sdk", ("5.4", "5.5")),
                            ("camera_fps", (30, 60)), ("self_calibration", ("disabled", "enabled")),
                            ("image_view", ("LEFT", "RIGHT")), ("rectified", (True, False)),
                            ("calibration_source", ("calibration_parameters.left_cam", "raw"))):
            a, b = copy.deepcopy(self.result), copy.deepcopy(self.other)
            a["camera"][key], b["camera"][key] = values
            with self.assertRaises(ValueError, msg=key):
                compare_results([a, b], ["a.json", "b.json"])

    def test_summary_handles_failed_scale_and_prints_tcp_frame(self):
        result = copy.deepcopy(self.result)
        result.update(views=3, rejected_views=[], train_rows=[], test_rows=[], chosen_method="Park",
                      board_scale_check={"status": "failed"}, reasons=[], warnings=[])
        result["frames"]["pose_moving"] = "reported_tcp"
        with contextlib.redirect_stdout(io.StringIO()) as output:
            print_summary(result)
        self.assertIn("reported_tcp_T_left_camera", output.getvalue())
        self.assertIn("diagnostic: failed", output.getvalue())


if __name__ == "__main__":
    unittest.main()
