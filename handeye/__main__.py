"""Command line: python -m handeye {init, detect, check-poses, solve, compare, validate} ..."""

from __future__ import annotations

import argparse
import json
import math
import re
import shutil
import sys
from dataclasses import fields
from pathlib import Path

import cv2
import numpy as np
import yaml

from .board import detect_board, detector, read_board
from .dataset import load_session, read_camera, read_poses
from .geometry import read_image, write_image
from .kinematics import check_convention, frame_findings, row_consistency
from .solver import Options, calibrate
from .validation import compare_results, read_touch_points, validate_result

EXIT_OK, EXIT_ERROR, EXIT_NOT_ACCEPTED = 0, 1, 2
TEMPLATE = Path(__file__).resolve().parents[1] / "templates" / "session"
DEFAULT_TARGET = TEMPLATE / "target.yaml"


def json_safe(value):
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {key: json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(item) for item in value]
    if isinstance(value, np.generic):
        return json_safe(value.item())
    return value


def write_json(path: Path, data: dict):
    Path(path).write_text(json.dumps(json_safe(data), indent=2, ensure_ascii=False, allow_nan=False),
                          encoding="utf-8")


def init_command(args) -> int:
    destination = Path(args.session)
    if destination.exists():
        raise FileExistsError(f"{destination} already exists")
    shutil.copytree(TEMPLATE, destination)
    (destination / "images").mkdir()
    camera_path = destination / "camera.yaml"
    if args.camera:  # e.g. the output of tools/zed_camera_yaml.py
        shutil.copyfile(args.camera, camera_path)
    elif args.intrinsics or args.size:
        text = camera_path.read_text(encoding="utf-8")
        values = dict(zip(("image_width", "image_height"), args.size or ()))
        values.update(zip(("fx", "fy", "cx", "cy"), args.intrinsics or ()))
        for key, value in values.items():
            text = re.sub(rf"^{key}:.*$", f"{key}: {value}", text, count=1, flags=re.MULTILINE)
        camera_path.write_text(text, encoding="utf-8")
    if args.target:
        shutil.copyfile(args.target, destination / "target.yaml")
    read_camera(camera_path)
    read_board(destination / "target.yaml")
    placeholder = not (args.camera or args.intrinsics)
    print(f"Created {destination}\nNext:")
    print(f"  1. camera.yaml   {'REPLACE the placeholder intrinsics' if placeholder else 'check the intrinsics'} "
          "(python3 tools/zed_camera_yaml.py on the ZED Box)")
    print("  2. target.yaml   use the exact board definition: 6 x 6 tags, 55 mm, spacing 16.5 mm")
    print("  3. images/       one image per robot pose")
    print("  4. poses.csv     one row per image: flange pose in the base frame, plus joints")
    print(f"  5. python -m handeye solve --session {destination}")
    return EXIT_OK


def detect_command(args) -> int:
    board = read_board(args.target)
    image = read_image(args.image)
    if image is None:
        raise FileNotFoundError(args.image)
    _, image_points, ids = detect_board(image, board, detector())
    print(f"Detected {len(ids)} board tags: {ids}")
    if args.overlay:
        drawing = image.copy() if image.ndim == 3 and image.shape[2] == 3 else cv2.cvtColor(
            image, cv2.COLOR_GRAY2BGR if image.ndim == 2 else cv2.COLOR_BGRA2BGR)
        for tag_id, corners in zip(ids, image_points.reshape(-1, 4, 2)):
            cv2.putText(drawing, str(tag_id), tuple(np.mean(corners, axis=0).astype(int)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            for j, corner in enumerate(corners.astype(int)):
                cv2.circle(drawing, tuple(corner), 5, (0, 255, 0), -1)
                if j == 0:  # physical bottom-right corner of the tag
                    cv2.putText(drawing, "0", tuple(corner), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
        if not write_image(args.overlay, drawing):
            raise RuntimeError(f"Could not write overlay {args.overlay}")
        print(f"Overlay: {args.overlay}")
    return EXIT_OK


def check_poses_command(args) -> int:
    samples = read_poses(args.poses)
    if any(s.joints_deg is None for s in samples):
        raise ValueError("check-poses needs j1_deg..j6_deg columns in poses.csv")
    joints, poses = [s.joints_deg for s in samples], [s.pose_mm_deg for s in samples]
    result = check_convention(joints, poses)
    print(f"{result['poses']} poses ({result['distinct_poses']} distinct); joint spread (deg): {result['joint_spread_deg']}")
    for name, fit in result.get("fits", {}).items():
        print(f"  {name:16s} rms {fit['rms_mm']:9.4f} mm {fit['rms_deg']:9.5f} deg")
    if "leave_one_out" in result:
        print(f"  leave-one-out worst {result['leave_one_out']['max_mm']:.3f} mm / "
              f"{result['leave_one_out']['max_deg']:.4f} deg")
    consistency = row_consistency([s.row for s in samples], joints, poses) if len(samples) >= 3 else None
    if consistency:
        print(f"Per-row pose vs FK(joints): median {consistency['median_mm']:.3f} mm / "
              f"{consistency['median_deg']:.4f} deg")
        for row in consistency["suspect_rows"]:
            print(f"  SUSPECT row {row['row']}: {row['error_mm']} mm / {row['error_deg']} deg")
        if result["status"] != "insufficient_data":  # offsets are unidentifiable without varied poses
            print(f"  fitted offsets (expected near zero): tool {consistency['flange_offset']}, "
                  f"base {consistency['base_offset']}")
            for message in frame_findings(consistency):
                if message:
                    print(f"WARNING: {message}")
    print(f"STATUS: {result['status'].upper()}" + "".join(f"\n  - {r}" for r in result["reasons"]))
    print("SimMachine should agree with the nominal model to < 0.5 mm; a real FR5 may differ by a few mm.")
    if args.output:
        write_json(args.output, {"convention": result, "row_consistency": consistency})
    return EXIT_OK if result["status"] == "pass" else EXIT_NOT_ACCEPTED


def print_summary(result: dict):
    print(f"Views: {result['views']} in poses.csv, {len(result['accepted_views'])} usable, "
          f"{len(result['rejected_views'])} rejected; train {len(result['train_rows'])}, test {len(result['test_rows'])}")
    for view in result["rejected_views"]:
        print(f"  rejected row {view['row']} ({view['image']}): {view['reason']}")
    for item in result.get("candidates", []):
        cv_px = item["cv_pixel_rmse"]
        print(f"  {item['method']:22s} CV {cv_px if cv_px is None else round(cv_px, 3)} px | "
              f"test {item['test']['pixel_rmse']:.3f} px, board {item['test']['board_position_rmse_mm']:.2f} mm / "
              f"{item['test']['board_rotation_rmse_deg']:.3f} deg")
    if "chosen" in result:
        t = np.asarray(result["chosen"]["pose_moving_T_left_camera"])[:3, 3] * 1000
        moving = result["frames"]["pose_moving"]
        print(f"Chosen {result['chosen_method']}: {moving}_T_left_camera translation "
              f"[{t[0]:.2f}, {t[1]:.2f}, {t[2]:.2f}] mm")
        u = result.get("uncertainty", {})
        if "translation_sd_mm" in u:
            print(f"  jackknife sd: translation {u['translation_sd_norm_mm']:.3f} mm "
                  f"{u['translation_sd_mm']}, rotation {u['rotation_sd_deg']:.4f} deg (random errors only)")
        scale = result.get("board_scale_check")
        if scale and "board_scale" in scale:
            print(f"  board scale diagnostic (geometry remains fixed): {scale['board_scale']:.4f}x target.yaml")
        elif scale:
            print(f"  board scale diagnostic: {scale.get('status', 'unavailable')}")
    check = result.get("intrinsics_check", {})
    if check.get("status") == "ok":
        print(f"  intrinsics from the images vs camera.yaml: focal {check['focal_change'][0] * 100:+.2f}% / "
              f"{check['focal_change'][1] * 100:+.2f}%, principal point ({check['principal_point_shift_px'][0]:+.1f}, "
              f"{check['principal_point_shift_px'][1]:+.1f}) px, residual distortion "
              f"{check['distortion_corner_shift_px']:.2f} px at the image corner")
    print(f"STATUS: {result['status'].upper()}" + "".join(f"\n  - {r}" for r in result["reasons"]))
    for warning in result["warnings"]:
        print(f"WARNING: {warning}")


def solve_command(args) -> int:
    options = Options(**{f.name: getattr(args, f.name) for f in fields(Options)})
    session = load_session(args.session, args.target)
    result = calibrate(session, options)
    destination = args.output or session.path / "result.json"
    write_json(destination, result)
    print_summary(result)
    print(f"Saved {destination}")
    return EXIT_OK if result["status"] == "accepted" else EXIT_NOT_ACCEPTED


def _positive_threshold(value: float, name: str):
    if not math.isfinite(value) or value <= 0:
        raise ValueError(f"{name} must be finite and positive")


def validate_command(args) -> int:
    _positive_threshold(args.max_rms_mm, "--max-rms-mm")
    result = json.loads(Path(args.result).read_text(encoding="utf-8"))
    if not isinstance(result, dict):
        raise ValueError("result must be a JSON object")
    board = read_board(args.target) if args.target else result.get("board")
    if not isinstance(board, dict):
        raise ValueError("result has no board definition; run solve again")
    points = read_touch_points(args.points)
    session = load_session(args.session) if args.session else None
    report = validate_result(result, board, points, session)
    print(f"SCOPE: {report['scope']}\n{report['note']}")
    if report["scope"] == "board_pose_check":
        for point in report["points"]:
            print(f"  row {point['row']}: tag {point['tag_id']} {point['corner']}  error {point['error_mm']:.2f} mm")
        diff = report["board_pose_difference"]
        print(f"Board pose from touches vs saved Y: {diff['translation_mm']:.2f} mm / {diff['rotation_deg']:.3f} deg "
              f"(touch fit residual {diff['touch_fit_rms_mm']:.2f} mm)")
    else:
        for view in report["views"]:
            print(f"  independent image row {view['row']}: RMS {view['rms_mm']:.2f} mm, max {view['max_mm']:.2f} mm")
        for view in report["rejected_views"]:
            print(f"  rejected image row {view['row']}: {view['reason']}")
    for warning in report.get("warnings", []):
        print(f"WARNING: {warning}")
    print(f"Touch-point error: RMS {report['rms_mm']:.2f} mm, max {report['max_mm']:.2f} mm")
    passed = report["rms_mm"] <= args.max_rms_mm and report.get("max_view_rms_mm", 0) <= args.max_rms_mm
    report.update(status="passed" if passed else "rejected", max_rms_mm=args.max_rms_mm)
    print(f"STATUS: {report['status'].upper()} ({report['scope']})")
    if args.output:
        write_json(args.output, report)
    return EXIT_OK if passed else EXIT_NOT_ACCEPTED


def compare_command(args) -> int:
    for name in ("max_mm", "max_deg", "max_ratio"):
        _positive_threshold(getattr(args, name), "--" + name.replace("_", "-"))
    results = [json.loads(Path(path).read_text(encoding="utf-8")) for path in args.results]
    names = [str(path) for path in args.results]
    rows = compare_results(results, names)
    worst = 0.0
    for row in rows:
        for warning in row.get("warnings", []):
            print(f"WARNING: {warning}")
        expected = ""
        if row["expected_random_mm"] is not None:
            # Floors keep noise-free (simulated) results from producing huge ratios.
            ratio = max(row["translation_mm"] / max(row["expected_random_mm"], 0.01),
                        row["rotation_deg"] / max(row["expected_random_deg"], 0.001))
            worst = max(worst, ratio)
            expected = (f"; random-error scale {row['expected_random_mm']:.3f} mm / "
                        f"{row['expected_random_deg']:.4f} deg -> {ratio:.1f}x")
        print(f"{row['a']}\n  vs {row['b']}\n  differ by {row['translation_mm']:.3f} mm "
              f"{row['translation_delta_mm']} / {row['rotation_deg']:.4f} deg{expected}")
    too_far = [row for row in rows if row["translation_mm"] > args.max_mm or row["rotation_deg"] > args.max_deg]
    if too_far or worst > args.max_ratio:
        print(f"DIFFERENT: beyond {args.max_mm} mm / {args.max_deg} deg or {args.max_ratio}x the random-error scale. "
              "Check that the mount did not move and both sessions used the same camera settings and pose frame.")
        return EXIT_NOT_ACCEPTED
    print("CONSISTENT (this cannot reveal errors both sessions share: board size, intrinsics, image flip)")
    return EXIT_OK


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="python -m handeye", description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)

    init = commands.add_parser("init", help="Create a session folder from templates/session")
    init.add_argument("--session", type=Path, required=True, help="new folder, e.g. calibration-data/sessions/s001")
    init.add_argument("--camera", type=Path, help="camera.yaml to use, e.g. from tools/zed_camera_yaml.py")
    init.add_argument("--intrinsics", type=float, nargs=4, metavar=("FX", "FY", "CX", "CY"))
    init.add_argument("--size", type=int, nargs=2, metavar=("WIDTH", "HEIGHT"))
    init.add_argument("--target", type=Path, help="board YAML (default: templates/session/target.yaml)")
    init.set_defaults(func=init_command)

    detect = commands.add_parser("detect", help="Check Aprilgrid tag IDs and corner order in one image")
    detect.add_argument("--image", type=Path, required=True)
    detect.add_argument("--target", type=Path, default=DEFAULT_TARGET)
    detect.add_argument("--overlay", type=Path)
    detect.set_defaults(func=detect_command)

    check = commands.add_parser("check-poses", help="Check poses.csv against its joint angles (FR5 model)")
    check.add_argument("--poses", type=Path, required=True)
    check.add_argument("--output", type=Path)
    check.set_defaults(func=check_poses_command)

    solve = commands.add_parser("solve", help="Calibrate a session folder")
    solve.add_argument("--session", type=Path, required=True)
    solve.add_argument("--target", type=Path, help="board YAML if the session has no target.yaml")
    solve.add_argument("--output", type=Path, help="default: <session>/result.json")
    defaults = Options()
    solve.add_argument("--min-tags", type=int, default=defaults.min_tags)
    solve.add_argument("--max-pnp-rmse", type=float, default=defaults.max_pnp_rmse)
    solve.add_argument("--no-refine", dest="refine", action="store_false")
    solve.add_argument("--max-test-px", type=float, default=defaults.max_test_px)
    solve.add_argument("--max-view-px", type=float, default=defaults.max_view_px)
    solve.add_argument("--max-board-mm", type=float, default=defaults.max_board_mm)
    solve.add_argument("--max-board-deg", type=float, default=defaults.max_board_deg)
    solve.add_argument("--expected-translation-mm", type=float, nargs=3, metavar=("X", "Y", "Z"),
                       help="flange_T_left_camera translation estimated from the mount CAD")
    solve.add_argument("--expected-tolerance-mm", type=float, default=defaults.expected_tolerance_mm)
    solve.add_argument("--max-candidate-spread-mm", type=float, default=defaults.max_candidate_spread_mm)
    solve.add_argument("--max-candidate-spread-deg", type=float, default=defaults.max_candidate_spread_deg)
    solve.add_argument("--bootstrap", type=int, default=defaults.bootstrap, help="jackknife subsets; 0 = off")
    solve.add_argument("--seed", type=int, default=defaults.seed)
    solve.add_argument("--max-scale-error", type=float, default=defaults.max_scale_error)
    solve.add_argument("--max-focal-error", type=float, default=defaults.max_focal_error)
    solve.add_argument("--max-principal-point-px", type=float, default=defaults.max_principal_point_px)
    solve.add_argument("--max-distortion-px", type=float, default=defaults.max_distortion_px,
                       help="warn if residual radial distortion shifts the image corner by more")
    solve.add_argument("--allow-tcp-offset", action="store_true",
                       help="poses deliberately refer to a TCP: output is reported_tcp_T_left_camera, never a flange transform")
    solve.set_defaults(func=solve_command)

    compare = commands.add_parser("compare", help="Compare results of independent sessions of the same mount")
    compare.add_argument("--results", type=Path, nargs="+", required=True, help="two or more result.json files")
    compare.add_argument("--max-mm", type=float, default=2.0)
    compare.add_argument("--max-deg", type=float, default=0.2)
    compare.add_argument("--max-ratio", type=float, default=3.0,
                         help="allowed multiple of the combined jackknife spread")
    compare.set_defaults(func=compare_command)

    check_result = commands.add_parser("validate", help="Check saved board pose, or exercise hand-eye with an independent session")
    check_result.add_argument("--result", type=Path, required=True)
    check_result.add_argument("--points", type=Path, required=True)
    check_result.add_argument("--session", type=Path, help="new independent session to validate X; same fixed board/mount, tool 0 and base/work frame 0")
    check_result.add_argument("--target", type=Path, help="board YAML; default: the one stored in the result")
    check_result.add_argument("--max-rms-mm", type=float, default=3.0)
    check_result.add_argument("--output", type=Path)
    check_result.set_defaults(func=validate_command)
    return parser


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    try:
        return args.func(args)
    except (OSError, ValueError, RuntimeError, cv2.error, yaml.YAMLError) as error:
        print(f"Error: {error}", file=sys.stderr)
        return EXIT_ERROR


if __name__ == "__main__":
    sys.exit(main())
