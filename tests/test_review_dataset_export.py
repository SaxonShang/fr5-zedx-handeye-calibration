"""Regression tests for finite inputs, image-frame evidence and safe SDK exports."""

import contextlib
import io
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

import yaml

from handeye.board import read_board
from handeye.dataset import JOINT_COLUMNS, POSE_COLUMNS, read_camera, read_poses
from tools import zed_camera_yaml


CAMERA = dict(image_width=1920, image_height=1200, fx=733., fy=733., cx=959.5, cy=599.5)
BOARD = dict(target_type="aprilgrid", tagCols=6, tagRows=6, tagSize=.055, tagSpacing=.3)


def write_yaml(directory, name, values):
    path = Path(directory) / name
    path.write_text(yaml.safe_dump(values), encoding="utf-8")
    return path


FAKE_SDK = '''
import os
from types import SimpleNamespace as NS
RESOLUTION = NS(HD1200="HD1200", HD1080="HD1080", SVGA="SVGA")
DEPTH_MODE = NS(NONE="NONE")
FLIP_MODE = NS(OFF="OFF")
ERROR_CODE = NS(SUCCESS=0)
class InitParameters:
    pass
class Camera:
    status = 0
    focal = 734.125678901
    closed = False
    settings = None
    def open(self, init):
        type(self).settings = init
        print("[SDK] startup message on stdout")
        os.write(1, b"[SDK native] startup on fd 1\\n")
        return self.status
    def get_init_parameters(self):
        return self.settings
    def get_camera_information(self):
        left = NS(fx=self.focal, fy=735., cx=959.5, cy=599.5, disto=[0.] * 5)
        config = NS(resolution=NS(width=1920, height=1200), fps=25.,
                    calibration_parameters=NS(left_cam=left))
        return NS(camera_configuration=config, serial_number=123456, camera_model="ZED X")
    def close(self):
        type(self).closed = True
    @staticmethod
    def get_sdk_version():
        return "5.5.0-test"
'''


class DatasetReviewTest(unittest.TestCase):
    def test_camera_nonfinite_fields_and_dimension_types(self):
        cases = [(key, value) for key in ("fx", "fy", "cx", "cy")
                 for value in (float("nan"), float("inf"), True)]
        cases += [(key, value) for key in ("image_width", "image_height")
                  for value in (1920.5, 1920., True, 0)]
        cases += [("distortion", [0., 0., value, 0., 0.])
                  for value in (float("nan"), float("-inf"), False)]
        with tempfile.TemporaryDirectory() as tmp:
            for key, value in cases:
                with self.subTest(key=key, value=value):
                    path = write_yaml(tmp, "camera.yaml", {**CAMERA, key: value})
                    with self.assertRaisesRegex(ValueError, key):
                        read_camera(path)

    def test_explicit_incompatible_image_metadata_is_rejected(self):
        cases = [("image_flip", "ON"), ("image_flip", "AUTO"), ("image_flip", True),
                 ("image_view", "RIGHT"), ("image_view", "LEFT_UNRECTIFIED"),
                 ("view", "VIEW.RIGHT"), ("rectified", False),
                 ("image_rectified", False), ("calibration_source", "calibration_parameters_raw.left_cam")]
        with tempfile.TemporaryDirectory() as tmp:
            for key, value in cases:
                with self.subTest(key=key, value=value):
                    with self.assertRaisesRegex(ValueError, key):
                        read_camera(write_yaml(tmp, "camera.yaml", {**CAMERA, key: value}))
            for extra in ({}, {"image_flip": False}, {"image_flip": "OFF", "image_view": "LEFT", "rectified": True}):
                read_camera(write_yaml(tmp, "camera.yaml", {**CAMERA, **extra}))

    def test_board_dimensions_must_be_finite_valid_numbers(self):
        cases = [(key, value) for key in ("tagSize", "tagSpacing")
                 for value in (float("nan"), float("inf"), True, 0.)]
        cases += [("tagCols", 6.), ("tagRows", True)]
        with tempfile.TemporaryDirectory() as tmp:
            for key, value in cases:
                with self.subTest(key=key, value=value):
                    with self.assertRaisesRegex(ValueError, key):
                        read_board(write_yaml(tmp, "target.yaml", {**BOARD, key: value}))
            self.assertEqual(read_board(write_yaml(tmp, "target.yaml", BOARD)), BOARD)

    def test_csv_errors_locate_numeric_field_and_physical_line(self):
        names = [*POSE_COLUMNS, *JOINT_COLUMNS]
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "poses.csv"
            for index, name in enumerate(names):
                values = ["0"] * len(names)
                values[index] = "nan" if index % 2 else "inf"
                path.write_text("# first comment\nimage," + ",".join(names) +
                                "\n\n# second comment\nimages/1.png," + ",".join(values) + "\n", encoding="utf-8")
                with self.subTest(field=name):
                    with self.assertRaisesRegex(ValueError, rf"row 1 \(line 5\).*{name}"):
                        read_poses(path)

    def test_csv_extra_fields_and_duplicate_headers_fail(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "poses.csv"
            header = "image," + ",".join(POSE_COLUMNS)
            path.write_text(header + "\nx.png,0,0,0,0,0,0,EXTRA\n", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "row 1.*more fields"):
                read_poses(path)
            path.write_text(header + ",x_mm\nx.png,0,0,0,0,0,0,1\n", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "repeated column"):
                read_poses(path)


class CameraExportReviewTest(unittest.TestCase):
    def fake_modules(self):
        package = types.ModuleType("pyzed")
        module = types.ModuleType("pyzed.sl")
        exec(FAKE_SDK.replace('os.write(1, b"[SDK native] startup on fd 1\\n")', 'pass'), module.__dict__)
        package.sl = module
        return {"pyzed": package, "pyzed.sl": module}, module

    def test_standalone_export_ignores_native_sdk_stdout(self):
        with tempfile.TemporaryDirectory() as tmp:
            package = Path(tmp) / "pyzed"
            package.mkdir()
            (package / "__init__.py").write_text("", encoding="utf-8")
            (package / "sl.py").write_text(FAKE_SDK, encoding="utf-8")
            # Match the deployment contract: only this script is copied to the Box.
            script = Path(tmp) / "zed_camera_yaml.py"
            script.write_text(Path(zed_camera_yaml.__file__).read_text(encoding="utf-8"), encoding="utf-8")
            for forbidden in ("handeye", "numpy", "cv2", "scipy"):
                (Path(tmp) / f"{forbidden}.py").write_text(
                    f"raise RuntimeError('standalone export imported {forbidden}')", encoding="utf-8")
            destination = Path(tmp) / "camera.yaml"
            env = dict(os.environ, PYTHONPATH=tmp, PYTHONDONTWRITEBYTECODE="1")
            run = subprocess.run([sys.executable, "-B", str(script), "--output", str(destination)],
                                 cwd=tmp, env=env, capture_output=True, text=True)
            self.assertEqual(run.returncode, 0, run.stderr)
            self.assertIn("[SDK native]", run.stdout)
            self.assertNotIn("[SDK", destination.read_text(encoding="utf-8"))
            data = read_camera(destination)
            self.assertEqual(data["image_view"], "LEFT")
            self.assertTrue(data["rectified"])
            self.assertEqual(data["camera_fps"], 25.)  # actual, not requested 30
            self.assertEqual(data["fx"], 734.125678901)  # no forced rounding
            self.assertEqual(data["self_calibration"], "disabled")
            self.assertEqual(data["camera_serial"], 123456)

    def test_failed_sdk_or_invalid_intrinsics_preserves_existing_file(self):
        for fault in ("open", "nan", "write"):
            with self.subTest(fault=fault), tempfile.TemporaryDirectory() as tmp:
                modules, sdk = self.fake_modules()
                path = Path(tmp) / "camera.yaml"
                path.write_text("existing calibration\n", encoding="utf-8")
                if fault == "open":
                    sdk.Camera.status = 42
                elif fault == "nan":
                    sdk.Camera.focal = float("nan")
                failure = patch.object(Path, "replace", side_effect=OSError("test replacement failure"))
                with patch.dict(sys.modules, modules), contextlib.redirect_stdout(io.StringIO()), \
                        contextlib.redirect_stderr(io.StringIO()), failure if fault == "write" else contextlib.nullcontext():
                    self.assertEqual(zed_camera_yaml.main(["--output", str(path)]), 1)
                self.assertEqual(path.read_text(encoding="utf-8"), "existing calibration\n")
                self.assertTrue(sdk.Camera.closed)
                self.assertEqual(list(Path(tmp).glob("*.tmp")), [])

    def test_self_calibration_scope_is_explicit(self):
        with tempfile.TemporaryDirectory() as tmp:
            modules, sdk = self.fake_modules()
            path = Path(tmp) / "camera.yaml"
            warning = io.StringIO()
            with patch.dict(sys.modules, modules), contextlib.redirect_stdout(io.StringIO()), \
                    contextlib.redirect_stderr(warning):
                self.assertEqual(zed_camera_yaml.main(["--output", str(path), "--self-calib"]), 0)
            data = read_camera(path)
            self.assertEqual(data["self_calibration"], "enabled")
            self.assertEqual(data["intrinsics_scope"], "standalone_export_instance")
            self.assertIn("SAME capture instance", data["image_source_requirement"])
            self.assertIn("same instance", warning.getvalue())
            self.assertEqual(sdk.Camera.settings.sdk_verbose, 0)
            self.assertFalse(sdk.Camera.settings.camera_disable_self_calib)


if __name__ == "__main__":
    unittest.main()
