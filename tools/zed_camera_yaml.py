"""Print camera.yaml for the ZED X rectified left image. Run on the ZED Box:

    python3 zed_camera_yaml.py > camera.yaml
    python3 zed_camera_yaml.py --resolution HD1200 > camera.yaml

Opens the camera with the settings the calibration assumes (auto flip off),
reads calibration_parameters.left_cam (rectified, zero distortion) and prints
it in the session format. Only reads; writes nothing itself. Not tested on
this repository's development machine (no ZED hardware there): check the
output once against the values ZED Explorer or your capture program shows.
"""

import argparse
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--resolution", choices=("HD1200", "HD1080", "SVGA"), default="HD1200")
    parser.add_argument("--self-calib", action="store_true",
                        help="leave SDK self-calibration on (default: off); must match your capture program")
    args = parser.parse_args()
    import pyzed.sl as sl

    init = sl.InitParameters()
    init.camera_resolution = getattr(sl.RESOLUTION, args.resolution)
    init.depth_mode = sl.DEPTH_MODE.NONE
    init.camera_image_flip = sl.FLIP_MODE.OFF
    init.camera_disable_self_calib = not args.self_calib
    camera = sl.Camera()
    status = camera.open(init)
    if status != sl.ERROR_CODE.SUCCESS:
        sys.exit(f"ZED open failed: {status}")
    try:
        info = camera.get_camera_information()
        config = info.camera_configuration
        left = config.calibration_parameters.left_cam
        print("# ZED X rectified left intrinsics, written by tools/zed_camera_yaml.py")
        print(f"image_width: {config.resolution.width}")
        print(f"image_height: {config.resolution.height}")
        print(f"fx: {left.fx:.6f}\nfy: {left.fy:.6f}\ncx: {left.cx:.6f}\ncy: {left.cy:.6f}")
        print(f"distortion: [{', '.join(f'{v:.8f}' for v in list(left.disto)[:5])}]")
        print(f"camera_serial: {info.serial_number}")
        print(f'zed_sdk: "{sl.Camera.get_sdk_version()}"')
        print(f"resolution: {args.resolution}")
        print('image_flip: "OFF"')
        print(f"self_calibration: {'enabled' if args.self_calib else 'disabled'}")
    finally:
        camera.close()


if __name__ == "__main__":
    main()
