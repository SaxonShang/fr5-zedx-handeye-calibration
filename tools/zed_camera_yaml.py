"""Export camera.yaml for the ZED X rectified left image. Run on the ZED Box:

    python3 tools/zed_camera_yaml.py --output camera.yaml
    python3 tools/zed_camera_yaml.py --resolution HD1200 --fps 30 --output camera.yaml

Copy this file alone to the Box; dependencies are pyzed and PyYAML.
SDK stdout is never used as YAML. The file is replaced only after a successful
read and validation. Self-calibration defaults to disabled. With --self-calib,
this standalone camera instance cannot establish the intrinsics of images saved
by another instance: export from that capture instance instead. Compare with
calibration_parameters.left_cam, not the raw parameters shown by ZED Explorer.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import math
from pathlib import Path
import sys
import tempfile

import yaml

def validate_export(metadata: dict, source):
    """Keep the standalone Box tool independent of the core numerical packages."""
    for key in ("image_width", "image_height"):
        if type(metadata.get(key)) is not int or metadata[key] <= 0:
            raise ValueError(f"{source}: {key} must be a positive integer")
    for key in ("fx", "fy", "cx", "cy", "camera_fps"):
        value = metadata.get(key)
        if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
            raise ValueError(f"{source}: {key} must be a finite positive number")
    distortion = metadata.get("distortion")
    if not isinstance(distortion, list) or len(distortion) not in (4, 5, 8, 12, 14):
        raise ValueError(f"{source}: distortion needs 4, 5, 8, 12 or 14 coefficients")
    for index, value in enumerate(distortion):
        if type(value) not in (int, float) or not math.isfinite(value):
            raise ValueError(f"{source}: distortion[{index}] must be a finite number")


def camera_metadata(sl, args) -> dict:
    """Read one opened instance only; never acquire or save images."""
    init = sl.InitParameters()
    init.camera_resolution = getattr(sl.RESOLUTION, args.resolution)
    init.camera_fps = args.fps
    init.depth_mode = sl.DEPTH_MODE.NONE
    init.camera_image_flip = sl.FLIP_MODE.OFF
    init.camera_disable_self_calib = not args.self_calib
    init.sdk_verbose = 0
    camera = sl.Camera()
    try:
        status = camera.open(init)
        if status != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f"ZED open failed: {status}")
        actual = camera.get_init_parameters()
        if actual.camera_image_flip != sl.FLIP_MODE.OFF:
            raise ValueError("SDK did not confirm image_flip OFF")
        if bool(actual.camera_disable_self_calib) != (not args.self_calib):
            raise ValueError("SDK self-calibration setting differs from the requested setting")
        info = camera.get_camera_information()
        config = info.camera_configuration
        left = config.calibration_parameters.left_cam
        result = {
            "image_width": int(config.resolution.width),
            "image_height": int(config.resolution.height),
            "fx": float(left.fx), "fy": float(left.fy),
            "cx": float(left.cx), "cy": float(left.cy),
            "distortion": [float(value) for value in left.disto],
            "camera_serial": int(info.serial_number),
            "camera_model": str(info.camera_model),
            "zed_sdk": str(sl.Camera.get_sdk_version()),
            "resolution": str(actual.camera_resolution).split(".")[-1],
            "camera_fps": float(config.fps),
            "requested_resolution": args.resolution,
            "requested_fps": args.fps,
            "image_view": "LEFT",
            "rectified": True,
            "calibration_source": "calibration_parameters.left_cam",
            "image_flip": "OFF",
            "self_calibration": "enabled" if args.self_calib else "disabled",
            "intrinsics_scope": "standalone_export_instance",
            "exported_utc": datetime.now(timezone.utc).isoformat(),
            "image_source_requirement": (
                "Use original-resolution VIEW.LEFT images from the same camera and SDK configuration. "
                "With self-calibration enabled, export intrinsics from the SAME capture instance; "
                "another open may change them. With it disabled, also verify matching factory calibration."
            ),
        }
        validate_export(result, "ZED camera metadata")
        if result["camera_serial"] <= 0 or not result["zed_sdk"].strip():
            raise ValueError("SDK returned invalid camera serial or version")
        return result
    finally:
        camera.close()


def write_camera(path: Path, metadata: dict):
    """Validate before writing and atomically replace; failures preserve the old file."""
    validate_export(metadata, path)
    text = "# ZED SDK rectified LEFT intrinsics; see image_source_requirement below.\n"
    text += yaml.safe_dump(metadata, sort_keys=False, allow_unicode=True)
    path = Path(path).resolve()
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", newline="\n",
                                         dir=path.parent, prefix=f".{path.name}.",
                                         suffix=".tmp", delete=False) as handle:
            temporary = Path(handle.name)
            handle.write(text)
        temporary.replace(path)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--output", type=Path, required=True, help="destination camera.yaml (atomic file write)")
    parser.add_argument("--resolution", choices=("HD1200", "HD1080", "SVGA"), default="HD1200")
    parser.add_argument("--fps", type=int, default=30, help="requested FPS; actual camera FPS is recorded")
    parser.add_argument("--self-calib", action="store_true",
                        help="enable self-calibration; this separate open cannot supply K for another capture instance")
    args = parser.parse_args(argv)
    if args.fps <= 0:
        parser.error("--fps must be positive")
    try:
        import pyzed.sl as sl
        metadata = camera_metadata(sl, args)
        write_camera(args.output, metadata)
    except (ImportError, OSError, RuntimeError, ValueError, AttributeError, TypeError) as exc:
        print(f"Camera export failed: {exc}", file=sys.stderr)
        return 1
    print(f"Saved {args.output.resolve()}", file=sys.stderr)
    if args.self_calib:
        print("WARNING: self-calibration is enabled. Export from the same instance that saved your images; "
              "this standalone export does not certify another instance's intrinsics.", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
