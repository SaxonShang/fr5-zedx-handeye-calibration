"""Synthetic sessions for end-to-end checks of handeye.py.

`make` renders the Kalibr Aprilgrid as seen by a ZED X 2.2 mm rectified left
camera on an FR5 flange and writes a session in the same format as
`handeye.py capture`, plus truth.json. `compare` scores result.json against
truth.json. `suite` runs a clean case and several injected faults. Nothing
here talks to hardware.
"""

from __future__ import annotations

import argparse
import json
import math
import shutil
import subprocess
import sys
import time
from pathlib import Path

sys.dont_write_bytecode = True  # keep the source tree free of __pycache__
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import cv2  # noqa: E402
import numpy as np  # noqa: E402

from handeye import (  # noqa: E402
    inverse, parameters_to_matrix, read_board, rotation_angle_degrees,
    tag_object_corners, transform, write_image,
)

WIDTH, HEIGHT = 1920, 1200
# ZED X 2.2 mm rectified left at HD1200: nominal f = 2.2 mm / 3 um pixel.
# A centred principal point makes a 180 deg image flip an exact camera roll.
K_ZEDX_2_2MM = np.array([[733.0, 0.0, 959.5], [0.0, 733.0, 599.5], [0.0, 0.0, 1.0]])
SUPERSAMPLE = 3
PX_PER_M = 4000
MARGIN_M = 0.02
BLACK, WHITE, BACKGROUND = 30, 220, 120
FR5_SHOULDER = np.array([0.0, 0.0, 0.152])  # DH d1
MIN_VISIBLE_TAGS = 18

# Left optical frame roughly along the flange axis, on a side bracket.
TRUTH_FLANGE_T_CAMERA = parameters_to_matrix(np.array([0.03, -0.02, 1.571, -0.060, 0.075, 0.045]))
# Board flat on the table in front of the robot, yawed slightly.
TRUTH_BASE_T_BOARD = parameters_to_matrix(np.array([0.0, 0.0, 0.1, 0.25, -0.19, 0.0]))


def _rz(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def fr5_from_matrix(t: np.ndarray, order: str = "zyx") -> list[float]:
    """Inverse of handeye.pose_from_fr5, as the controller would report it.

    order="zyx" is R = Rz @ Ry @ Rx (what handeye.py assumes); "xyz" is
    R = Rx @ Ry @ Rz, used to simulate a wrong convention.
    """
    r = t[:3, :3]
    if order == "zyx":
        ry = math.asin(-np.clip(r[2, 0], -1.0, 1.0))
        rx, rz = math.atan2(r[2, 1], r[2, 2]), math.atan2(r[1, 0], r[0, 0])
    elif order == "xyz":
        ry = math.asin(np.clip(r[0, 2], -1.0, 1.0))
        rx, rz = math.atan2(-r[1, 2], r[2, 2]), math.atan2(-r[0, 1], r[0, 0])
    else:
        raise ValueError(f"Unknown Euler order {order}")
    # The controller reports three decimals.
    return [round(float(v), 3) for v in (*(t[:3, 3] * 1000.0), *np.rad2deg([rx, ry, rz]))]


def board_texture(board: dict) -> tuple[np.ndarray, np.ndarray]:
    """Kalibr Aprilgrid image and the map from its pixel centres to board metres."""
    size = float(board["tagSize"])
    gap = size * float(board["tagSpacing"])
    s, g, m = (int(round(v * PX_PER_M)) for v in (size, gap, MARGIN_M))
    if abs(s / PX_PER_M - size) > 2e-5 or abs(g / PX_PER_M - gap) > 2e-5:
        raise ValueError("Target dimensions do not fit the texture resolution")
    p, cols, rows = s + g, board["tagCols"], board["tagRows"]
    texture = np.full((2 * m + g + rows * p, 2 * m + g + cols * p), WHITE, np.uint8)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    for tag_id in range(rows * cols):
        row, col = divmod(tag_id, cols)
        marker = cv2.aruco.generateImageMarker(dictionary, tag_id, s, borderBits=2)
        # Kalibr draws each code rotated 180 deg from OpenCV's canonical marker.
        marker = np.where(np.rot90(marker, 2) == 0, BLACK, WHITE).astype(np.uint8)
        top, left = rows * p + m - row * p - s, col * p + g + m
        texture[top:top + s, left:left + s] = marker
    # Small squares at the gap intersections, as on the printed board.
    for row in range(rows + 1):
        for col in range(cols + 1):
            top, left = rows * p + m - row * p, col * p + m
            texture[top:top + g, left:left + g] = BLACK
    tex_to_board = np.array([[1.0 / PX_PER_M, 0.0, (0.5 - g - m) / PX_PER_M],
                             [0.0, -1.0 / PX_PER_M, (rows * p + m - 0.5) / PX_PER_M],
                             [0.0, 0.0, 1.0]])
    return texture, tex_to_board


def render_view(texture: np.ndarray, tex_to_board: np.ndarray, camera_t_board: np.ndarray,
                k: np.ndarray, rng: np.random.Generator, blur: float, noise: float,
                board_scale: float = 1.0) -> np.ndarray:
    """Grey image of the board, supersampled then area-averaged like a sensor."""
    ss = SUPERSAMPLE
    k_ss = np.array([[ss * k[0, 0], 0.0, ss * k[0, 2] + (ss - 1) / 2],
                     [0.0, ss * k[1, 1], ss * k[1, 2] + (ss - 1) / 2], [0.0, 0.0, 1.0]])
    plane = camera_t_board[:3][:, [0, 1, 3]]
    homography = k_ss @ plane @ np.diag([board_scale, board_scale, 1.0]) @ tex_to_board
    big = cv2.warpPerspective(texture, homography, (WIDTH * ss, HEIGHT * ss),
                              flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT,
                              borderValue=BACKGROUND)
    image = cv2.resize(big, (WIDTH, HEIGHT), interpolation=cv2.INTER_AREA).astype(np.float32)
    if blur > 0:
        image = cv2.GaussianBlur(image, (0, 0), blur)
    image += rng.normal(0.0, noise, image.shape).astype(np.float32)
    return np.clip(np.rint(image), 0, 255).astype(np.uint8)


def visible_tags(corners: np.ndarray, camera_t_board: np.ndarray, k: np.ndarray) -> int:
    points = camera_t_board[:3, :3] @ corners.T + camera_t_board[:3, 3:4]
    if np.any(points[2] <= 0.05):
        return 0
    uv = (k @ points)[:2] / points[2]
    inside = (uv[0] >= 8) & (uv[0] <= WIDTH - 9) & (uv[1] >= 8) & (uv[1] <= HEIGHT - 9)
    return int(inside.reshape(-1, 4).all(axis=1).sum())


def sample_views(rng: np.random.Generator, count: int, board: dict, k: np.ndarray,
                 board_scale: float, single_axis: bool) -> list[tuple[np.ndarray, np.ndarray]]:
    """(base_T_flange, camera_T_board) pairs an operator could plausibly collect.

    Camera 0.38-0.65 m from the board centre, up to 40 deg off its normal,
    +-90 deg roll; flange within an approximate FR5 reach (<= 0.85 m from the
    shoulder) and above the table. This is a reach sphere, not full IK.
    """
    ids = range(board["tagRows"] * board["tagCols"])
    corners = np.concatenate([tag_object_corners(i, board) for i in ids]) * board_scale
    pitch = board["tagSize"] * (1.0 + board["tagSpacing"])
    gap = board["tagSize"] * board["tagSpacing"]
    centre = np.array([board["tagCols"] * pitch - gap, board["tagRows"] * pitch - gap, 0.0]) / 2
    centre *= board_scale
    views, reference = [], None
    for _ in range(50000):
        if len(views) == count:
            return views
        if single_axis and reference is not None:
            # Only roll about the optical axis: every relative rotation shares one axis.
            base_t_camera = reference.copy()
            base_t_camera[:3, :3] = reference[:3, :3] @ _rz(rng.uniform(-math.pi / 2, math.pi / 2))
            offset = np.r_[rng.uniform(-0.06, 0.06, 2), rng.uniform(-0.05, 0.05)]
            base_t_camera[:3, 3] = reference[:3, 3] + reference[:3, :3] @ offset
        else:
            distance = rng.uniform(0.38, 0.65)
            tilt, azimuth = math.radians(rng.uniform(0.0, 40.0)), rng.uniform(0.0, 2 * math.pi)
            eye = centre + distance * np.array([math.sin(tilt) * math.cos(azimuth),
                                                math.sin(tilt) * math.sin(azimuth), math.cos(tilt)])
            target = centre + np.r_[rng.uniform(-0.06, 0.06, 2), 0.0]
            z = (target - eye) / np.linalg.norm(target - eye)
            x = np.array([1.0, 0.0, 0.0]) - z * z[0]
            x /= np.linalg.norm(x)
            rotation = np.column_stack([x, np.cross(z, x), z]) @ _rz(rng.uniform(-math.pi / 2, math.pi / 2))
            base_t_camera = TRUTH_BASE_T_BOARD @ transform(rotation, eye)
        base_t_flange = base_t_camera @ inverse(TRUTH_FLANGE_T_CAMERA)
        reach = np.linalg.norm(base_t_flange[:3, 3] - FR5_SHOULDER)
        if not 0.30 <= reach <= 0.85 or base_t_flange[2, 3] < 0.20:
            continue
        camera_t_board = inverse(base_t_camera) @ TRUTH_BASE_T_BOARD
        if visible_tags(corners, camera_t_board, k) < MIN_VISIBLE_TAGS:
            continue
        if single_axis and reference is None:
            reference = base_t_camera
        views.append((base_t_flange, camera_t_board))
    raise RuntimeError(f"Could only sample {len(views)} of {count} reachable views")


def make_session(output: Path, target: Path, poses: int = 25, seed: int = 0,
                 board_scale: float = 1.0, flip180: bool = False, euler_order: str = "zyx",
                 robot_noise_mm: float = 0.0, robot_noise_deg: float = 0.0,
                 pose_outliers: int = 0, single_axis: bool = False,
                 blur: float = 0.6, image_noise: float = 1.5) -> Path:
    """Write a capture-format session with known truth; returns the session path."""
    output = Path(output).resolve()
    if output.exists():
        raise FileExistsError(f"Session directory already exists: {output}")
    board = read_board(target)
    rng = np.random.default_rng(seed)
    k = K_ZEDX_2_2MM
    views = sample_views(rng, poses, board, k, board_scale, single_axis)
    outliers = set(rng.choice(poses, size=pose_outliers, replace=False).tolist()) if pose_outliers else set()
    texture, tex_to_board = board_texture(board)
    output.mkdir(parents=True)
    (output / "images").mkdir()
    shutil.copyfile(target, output / "target.yaml")
    samples = []
    for index, (base_t_flange, camera_t_board) in enumerate(views):
        image = render_view(texture, tex_to_board, camera_t_board, k, rng, blur, image_noise, board_scale)
        if flip180:  # what FLIP_MODE.AUTO does when the camera opens upside down
            image = cv2.rotate(image, cv2.ROTATE_180)
        filename = f"images/{index:04d}.png"
        if not write_image(output / filename, cv2.cvtColor(image, cv2.COLOR_GRAY2BGRA)):
            raise RuntimeError(f"Could not write {filename}")
        noise = np.r_[np.deg2rad(rng.normal(0.0, robot_noise_deg, 3)), rng.normal(0.0, robot_noise_mm / 1000, 3)]
        if index in outliers:  # pose read before the arm settled: 3 mm / 0.5 deg
            axis, direction = rng.normal(size=3), rng.normal(size=3)
            noise += np.r_[np.deg2rad(0.5) * axis / np.linalg.norm(axis), 0.003 * direction / np.linalg.norm(direction)]
        reported = fr5_from_matrix(base_t_flange @ parameters_to_matrix(noise), euler_order)
        samples.append({
            "image": filename, "flange_pose_mm_deg": reported, "flange_pose_after_mm_deg": reported,
            "zed_image_timestamp_ns": index * 1_000_000_000, "host_grab_before_ns": index * 1_000_000_000,
            "host_grab_after_ns": index * 1_000_000_000 + 40_000_000,
        })
    session = {
        "schema_version": 1, "camera": "SYNTHETIC ZED X 2.2 mm rectified left optical frame",
        "camera_serial": 0, "image_view": "LEFT", "zed_camera_image_flip": "OFF",
        "zed_self_calibration": False, "image_width": WIDTH, "image_height": HEIGHT,
        "K": k.tolist(), "distortion": [0.0] * 5, "robot": "FAIRINO FR5 (simulated)",
        "robot_pose_source": "GetActualToolFlangePose(0)", "robot_pose_units": "mm and degrees",
        "robot_pose_convention": "fixed XYZ, R=Rz(rz)@Ry(ry)@Rx(rx)", "board": board,
        "samples": samples,
    }
    (output / "session.json").write_text(json.dumps(session, indent=2), encoding="utf-8")
    truth = {
        "flange_T_left_camera": TRUTH_FLANGE_T_CAMERA.tolist(),
        "base_T_board": TRUTH_BASE_T_BOARD.tolist(),
        "settings": {"poses": poses, "seed": seed, "board_scale": board_scale, "flip180": flip180,
                     "euler_order": euler_order, "robot_noise_mm": robot_noise_mm,
                     "robot_noise_deg": robot_noise_deg, "pose_outliers": sorted(outliers),
                     "single_axis": single_axis, "blur_px": blur, "image_noise": image_noise},
    }
    (output / "truth.json").write_text(json.dumps(truth, indent=2), encoding="utf-8")
    return output


def compare_session(session: Path) -> dict:
    """Errors of every candidate in result.json against truth.json."""
    session = Path(session)
    truth = json.loads((session / "truth.json").read_text(encoding="utf-8"))
    result = json.loads((session / "result.json").read_text(encoding="utf-8"))
    true_camera = np.asarray(truth["flange_T_left_camera"])
    rows = []
    for item in result["candidates"]:
        estimate = np.asarray(item["flange_T_left_camera"])
        rows.append({
            "method": item["method"], "chosen": item["method"] == result["chosen_method"],
            "translation_error_mm": float(np.linalg.norm(estimate[:3, 3] - true_camera[:3, 3]) * 1000),
            "rotation_error_deg": rotation_angle_degrees(true_camera[:3, :3].T @ estimate[:3, :3]),
            "holdout_pixel_rmse": item["holdout"]["pixel_rmse"],
            "holdout_board_mm": item["holdout"]["board_position_rmse_mm"],
        })
    return {"session": str(session), "candidates": rows, "chosen": next(r for r in rows if r["chosen"])}


def print_comparison(report: dict):
    print(f"{'method':24s} {'err mm':>8s} {'err deg':>8s} {'holdout px':>11s} {'board mm':>9s}")
    for row in report["candidates"]:
        mark = " *" if row["chosen"] else ""
        print(f"{row['method']:24s} {row['translation_error_mm']:8.3f} {row['rotation_error_deg']:8.3f} "
              f"{row['holdout_pixel_rmse']:11.3f} {row['holdout_board_mm']:9.3f}{mark}")


def solve_session(session: Path) -> tuple[bool, str]:
    run = subprocess.run([sys.executable, "-B", str(ROOT / "handeye.py"), "solve", "--session", str(session)],
                         capture_output=True, text=True)
    return run.returncode == 0, (run.stdout + run.stderr).strip()


# name, make_session overrides, expectation, check(outcome) -> behaved as expected
SCENARIOS = [
    ("clean", {}, "error <= 0.5 mm / 0.05 deg",
     lambda o: o["solved"] and o["mm"] <= 0.5 and o["deg"] <= 0.05),
    ("robot-noise", {"robot_noise_mm": 0.2, "robot_noise_deg": 0.02}, "error <= 2 mm / 0.2 deg",
     lambda o: o["solved"] and o["mm"] <= 2.0 and o["deg"] <= 0.2),
    ("pose-outliers", {"pose_outliers": 3}, "3 bad poses tolerated: <= 2 mm / 0.2 deg",
     lambda o: o["solved"] and o["mm"] <= 2.0 and o["deg"] <= 0.2),
    ("board-scale", {"board_scale": 1.01}, "board 1% large: bias > 1 mm, not self-detectable",
     lambda o: o["solved"] and o["mm"] > 1.0),
    ("flip180", {"flip180": True}, "image flipped: ~180 deg error, not self-detectable",
     lambda o: o["solved"] and o["deg"] > 170.0),
    ("euler-order", {"euler_order": "xyz"}, "wrong Euler order: rejected or large holdout error",
     lambda o: not o["solved"] or o["holdout_px"] > 2.0 or o["holdout_board_mm"] > 5.0),
    ("single-axis", {"single_axis": True}, "one rotation axis: rejected by diversity check",
     lambda o: not o["solved"] and "distinct axes" in o["message"]),
]


def run_suite(output_root: Path, target: Path, poses: int, seed: int, keep_images: bool) -> list[dict]:
    output_root = Path(output_root).resolve()
    if output_root.exists():
        raise FileExistsError(f"Suite directory already exists: {output_root}")
    outcomes = []
    for name, overrides, expectation, check in SCENARIOS:
        started = time.monotonic()
        session = make_session(output_root / name, target, poses=poses, seed=seed, **overrides)
        solved, message = solve_session(session)
        outcome = {"scenario": name, "expectation": expectation, "solved": solved,
                   "message": message.splitlines()[-1] if message else "",
                   "mm": math.nan, "deg": math.nan, "holdout_px": math.nan, "holdout_board_mm": math.nan}
        if solved:
            chosen = compare_session(session)["chosen"]
            outcome.update(method=chosen["method"], mm=chosen["translation_error_mm"],
                           deg=chosen["rotation_error_deg"], holdout_px=chosen["holdout_pixel_rmse"],
                           holdout_board_mm=chosen["holdout_board_mm"])
        outcome["as_expected"] = bool(check(outcome))
        outcome["seconds"] = round(time.monotonic() - started, 1)
        if not keep_images:
            shutil.rmtree(session / "images")
        outcomes.append(outcome)
        print(f"{name:14s} {'OK ' if outcome['as_expected'] else 'BAD'} "
              f"err {outcome['mm']:7.3f} mm {outcome['deg']:8.3f} deg  "
              f"holdout {outcome['holdout_px']:6.3f} px {outcome['holdout_board_mm']:6.2f} mm  "
              f"[{expectation}]" + ("" if solved else f"  solve failed: {outcome['message']}"))
    (output_root / "summary.json").write_text(json.dumps(outcomes, indent=2), encoding="utf-8")
    return outcomes


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    make = commands.add_parser("make", help="Render one synthetic capture session")
    make.add_argument("--output", required=True, type=Path)
    make.add_argument("--target", type=Path, default=ROOT / "target.example.yaml")
    make.add_argument("--poses", type=int, default=25)
    make.add_argument("--seed", type=int, default=0)
    make.add_argument("--board-scale", type=float, default=1.0)
    make.add_argument("--flip180", action="store_true")
    make.add_argument("--euler-order", choices=("zyx", "xyz"), default="zyx")
    make.add_argument("--robot-noise-mm", type=float, default=0.0)
    make.add_argument("--robot-noise-deg", type=float, default=0.0)
    make.add_argument("--pose-outliers", type=int, default=0)
    make.add_argument("--single-axis", action="store_true")
    make.add_argument("--blur", type=float, default=0.6)
    make.add_argument("--image-noise", type=float, default=1.5)
    compare = commands.add_parser("compare", help="Score result.json against truth.json")
    compare.add_argument("--session", required=True, type=Path)
    suite = commands.add_parser("suite", help="Clean case plus injected faults")
    suite.add_argument("--output", type=Path,
                       default=ROOT / "calibration-data" / "sim_runs" / time.strftime("suite-%Y%m%d-%H%M%S"))
    suite.add_argument("--target", type=Path, default=ROOT / "target.example.yaml")
    suite.add_argument("--poses", type=int, default=25)
    suite.add_argument("--seed", type=int, default=0)
    suite.add_argument("--keep-images", action="store_true")
    args = parser.parse_args()

    if args.command == "make":
        session = make_session(args.output, args.target, args.poses, args.seed, args.board_scale,
                               args.flip180, args.euler_order, args.robot_noise_mm, args.robot_noise_deg,
                               args.pose_outliers, args.single_axis, args.blur, args.image_noise)
        print(f"Session: {session}\nNext: python handeye.py solve --session \"{session}\"")
    elif args.command == "compare":
        print_comparison(compare_session(args.session))
    else:
        outcomes = run_suite(args.output, args.target, args.poses, args.seed, args.keep_images)
        print(f"{sum(o['as_expected'] for o in outcomes)}/{len(outcomes)} scenarios as expected; "
              f"summary: {Path(args.output).resolve() / 'summary.json'}")


if __name__ == "__main__":
    try:
        main()
    except (OSError, RuntimeError, ValueError) as error:
        print(f"Error: {error}", file=sys.stderr)
        sys.exit(1)
