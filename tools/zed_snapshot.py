"""Save ONE rectified left image for calibration. Run on the ZED Box:

    python3 zed_snapshot.py --output ~/calib/s001/images/0001.png

Opens the camera with the settings of the capture program (HD1080, 15 fps,
FLIP_MODE.OFF, self-calibration disabled, manual exposure 10 ms, analog gain
1000 mdB, digital gain 1), discards warm-up frames so the manual exposure has
settled, saves one VIEW.LEFT image and a JSON sidecar with the intrinsics of
this same camera instance. Writes only when invoked; nothing is recorded
continuously. Close any other program using the camera first.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import sys


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--output", type=Path, required=True, help="image path (.png)")
    parser.add_argument("--resolution", choices=("HD1200", "HD1080", "SVGA"), default="HD1080")
    parser.add_argument("--fps", type=int, default=15)
    parser.add_argument("--exposure-us", type=int, default=10000)
    parser.add_argument("--analog-gain-mdb", type=int, default=1000)
    parser.add_argument("--digital-gain", type=int, default=1)
    parser.add_argument("--warmup", type=int, default=20, help="frames discarded before saving")
    parser.add_argument("--force", action="store_true", help="overwrite an existing image")
    args = parser.parse_args(argv)
    output = args.output.expanduser().resolve()
    if output.suffix.lower() != ".png":
        parser.error("--output must be a .png file")
    if output.exists() and not args.force:
        parser.error(f"{output} exists; pass --force to overwrite")

    import pyzed.sl as sl

    init = sl.InitParameters()
    init.camera_resolution = getattr(sl.RESOLUTION, args.resolution)
    init.camera_fps = args.fps
    init.depth_mode = sl.DEPTH_MODE.NONE
    init.camera_image_flip = sl.FLIP_MODE.OFF
    init.camera_disable_self_calib = True
    init.sdk_verbose = 0
    camera = sl.Camera()
    status = camera.open(init)
    if status != sl.ERROR_CODE.SUCCESS:
        print(f"ZED open failed: {status}", file=sys.stderr)
        return 1
    try:
        for setting, value in ((sl.VIDEO_SETTINGS.AEC_AGC, 0), (sl.VIDEO_SETTINGS.EXPOSURE_TIME, args.exposure_us),
                               (sl.VIDEO_SETTINGS.ANALOG_GAIN, args.analog_gain_mdb),
                               (sl.VIDEO_SETTINGS.DIGITAL_GAIN, args.digital_gain)):
            if camera.set_camera_settings(setting, value) != sl.ERROR_CODE.SUCCESS:
                print(f"setting {setting} = {value} failed", file=sys.stderr)
                return 1
        runtime, image = sl.RuntimeParameters(), sl.Mat()
        grabbed = 0
        for _ in range(args.warmup + 30):
            if camera.grab(runtime) == sl.ERROR_CODE.SUCCESS:
                grabbed += 1
                if grabbed > args.warmup:
                    break
        if grabbed <= args.warmup or camera.retrieve_image(image, sl.VIEW.LEFT) != sl.ERROR_CODE.SUCCESS:
            print("could not grab a frame", file=sys.stderr)
            return 1
        stamp_ms = camera.get_timestamp(sl.TIME_REFERENCE.IMAGE).get_milliseconds()
        output.parent.mkdir(parents=True, exist_ok=True)
        if image.write(str(output)) != sl.ERROR_CODE.SUCCESS:
            print(f"writing {output} failed", file=sys.stderr)
            return 1
        info = camera.get_camera_information()
        config = info.camera_configuration
        left = config.calibration_parameters.left_cam
        pixels = image.get_data()
        meta = {
            "image": output.name, "zed_timestamp_ms": int(stamp_ms),
            "saved_utc": datetime.now(timezone.utc).isoformat(),
            "image_width": int(image.get_width()), "image_height": int(image.get_height()),
            "fx": float(left.fx), "fy": float(left.fy), "cx": float(left.cx), "cy": float(left.cy),
            "camera_serial": int(info.serial_number), "zed_sdk": str(sl.Camera.get_sdk_version()),
            "resolution": args.resolution, "camera_fps": args.fps, "image_view": "LEFT", "rectified": True,
            "image_flip": "OFF", "self_calibration": "disabled", "exposure_us": args.exposure_us,
            "analog_gain_mdb": args.analog_gain_mdb, "digital_gain": args.digital_gain,
            "mean_gray": float(pixels[:, :, :3].mean()),
        }
        output.with_suffix(".json").write_text(json.dumps(meta, indent=2), encoding="utf-8")
        print(f"Saved {output} ({meta['image_width']}x{meta['image_height']}, mean {meta['mean_gray']:.0f}/255)")
        return 0
    finally:
        camera.close()


if __name__ == "__main__":
    raise SystemExit(main())
