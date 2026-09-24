"""Synthetic sessions with known truth for testing the calibration.

`make` renders the Kalibr Aprilgrid as seen by a ZED X 2.2 mm rectified left
camera on an FR5 flange and writes a session folder in the format the solver
reads (camera.yaml, poses.csv with joints, images/, target.yaml), plus
truth.json and touch_points.csv. `compare` scores result.json against the
truth. `suite` runs clean data, injected faults and measurements.

What this cannot show: renderer and detector share one board definition, K
is nominal (distortion only when injected), poses are static (no timing
model), and the FR5 is its nominal kinematic model without collision checks.
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
import yaml  # noqa: E402

from handeye.board import board_point, read_board, tag_object_corners  # noqa: E402
from handeye.dataset import write_poses  # noqa: E402
from handeye.geometry import fr5_from_matrix, inverse, parameters_to_matrix, rotation_angle_degrees, \
    transform, write_image  # noqa: E402
from handeye.kinematics import fk, ik, near_singular  # noqa: E402

WIDTH, HEIGHT = 1920, 1200
# ZED X 2.2 mm rectified left at HD1200: nominal f = 2.2 mm / 3 um pixel.
# The centred principal point makes the flip180 scenario an exact camera roll;
# a real AUTO flip with an off-centre principal point is not modelled.
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
TOUCH_CORNERS = ((0, "BL"), (5, "BR"), (30, "TL"), (35, "TR"), (14, "TR"), (21, "BL"))


def _rz(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


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
    big = cv2.warpPerspective(texture, homography, (WIDTH * ss, HEIGHT * ss), flags=cv2.INTER_LINEAR,
                              borderMode=cv2.BORDER_CONSTANT, borderValue=BACKGROUND)
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
                 board_scale: float, single_axis: bool) -> list[tuple[np.ndarray, np.ndarray, np.ndarray]]:
    """(base_T_flange, camera_T_board, joints_deg) an operator could plausibly collect.

    Camera 0.38-0.65 m from the board centre, up to 40 deg off its normal,
    +-90 deg roll. Each flange pose must have a nominal FR5 IK solution inside
    the URDF joint limits, away from singularities, with the flange above the
    table. Collisions are not checked.
    """
    ids = range(board["tagRows"] * board["tagCols"])
    corners = np.concatenate([tag_object_corners(i, board) for i in ids]) * board_scale
    pitch = board["tagSize"] * (1.0 + board["tagSpacing"])
    gap = board["tagSize"] * board["tagSpacing"]
    centre = np.array([board["tagCols"] * pitch - gap, board["tagRows"] * pitch - gap, 0.0]) / 2 * board_scale
    views, reference = [], None
    for _ in range(5000):
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
        if base_t_flange[2, 3] < 0.20 or np.linalg.norm(base_t_flange[:3, 3] - FR5_SHOULDER) > 0.95:
            continue  # table clearance; cheap reach prefilter before IK
        camera_t_board = inverse(base_t_camera) @ TRUTH_BASE_T_BOARD
        if visible_tags(corners, camera_t_board, k) < MIN_VISIBLE_TAGS:
            continue
        joints = ik(base_t_flange, rng)
        if joints is None or near_singular(joints):
            continue
        if single_axis and reference is None:
            reference = base_t_camera
        views.append((base_t_flange, camera_t_board, joints))
    raise RuntimeError(f"Could only sample {len(views)} of {count} reachable views")


def make_session(output: Path, target: Path, poses: int = 25, seed: int = 0,
                 board_scale: float = 1.0, flip180: bool = False, euler_order: str = "zyx",
                 robot_noise_mm: float = 0.0, robot_noise_deg: float = 0.0,
                 pose_outliers: int = 0, outlier_split: str = "train", single_axis: bool = False,
                 joint_offset_deg: float = 0.0, k_error: tuple = (0.0, 0.0, 0.0),
                 jpeg_quality: int = 0, pose_typo: int = -1, touch_noise_mm: float = 0.1,
                 residual_k1: float = 0.0, tcp_offset_mm: float = 0.0, user_frame: bool = False,
                 blur: float = 0.6, image_noise: float = 1.5) -> Path:
    """Write a session folder with known truth; returns its path.

    joint_offset_deg: std of a fixed per-joint zero error, so reported poses are
    FK(q_true - offset): a systematic kinematic error, unlike robot_noise_*.
    k_error: (relative focal error, cx shift px, cy shift px) written into
    camera.yaml while images use the true K. jpeg_quality > 0 re-encodes images
    as JPEG, a stand-in for lossy video (not a model of H.265 artefacts).
    outlier_split: bad poses land in "train" or "test" views (every fifth view
    is a test view when all views are usable). pose_typo: view index whose
    x_mm is mistyped by +10 mm while its joints stay correct. residual_k1:
    radial distortion left in the images while camera.yaml says zero.
    tcp_offset_mm: poses read with a TCP this far along the flange z axis
    active; user_frame: poses read in a shifted, yawed work frame.
    """
    output = Path(output).resolve()
    if output.exists():
        raise FileExistsError(f"Session directory already exists: {output}")
    board = read_board(target)
    rng = np.random.default_rng(seed)
    k = K_ZEDX_2_2MM
    views = sample_views(rng, poses, board, k, board_scale, single_axis)
    pool = [i for i in range(poses) if (i % 5 == 0) == (outlier_split == "test")]
    outliers = set(rng.choice(pool, size=pose_outliers, replace=False).tolist()) if pose_outliers else set()
    joint_offsets = rng.normal(0.0, joint_offset_deg, 6) if joint_offset_deg else np.zeros(6)
    texture, tex_to_board = board_texture(board)
    if residual_k1:
        # For every distorted output pixel, where it came from in the ideal image.
        grid = np.stack(np.meshgrid(np.arange(WIDTH), np.arange(HEIGHT)), axis=-1).reshape(-1, 1, 2)
        source = cv2.undistortPoints(grid.astype(np.float64), k, np.array([residual_k1, 0, 0, 0, 0.0]), P=k)
        map_x, map_y = (source.reshape(HEIGHT, WIDTH, 2)[..., i].astype(np.float32) for i in (0, 1))
    tool = transform(np.eye(3), [0.0, 0.0, tcp_offset_mm / 1000])
    work = parameters_to_matrix(np.array([0.0, 0.0, 0.3, 0.2, 0.1, 0.0])) if user_frame else np.eye(4)
    (output / "images").mkdir(parents=True)
    shutil.copyfile(target, output / "target.yaml")
    rows = []
    for index, (base_t_flange, camera_t_board, joints) in enumerate(views):
        image = render_view(texture, tex_to_board, camera_t_board, k, rng, blur, image_noise, board_scale)
        if residual_k1:
            image = cv2.remap(image, map_x, map_y, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT,
                              borderValue=BACKGROUND)
        if flip180:  # a stand-in for FLIP_MODE.AUTO; exact only for a centred principal point
            image = cv2.rotate(image, cv2.ROTATE_180)
        if jpeg_quality:
            image = cv2.imdecode(cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, jpeg_quality])[1],
                                 cv2.IMREAD_GRAYSCALE)
        name = f"images/{index:04d}.png"
        if not write_image(output / name, image):
            raise RuntimeError(f"Could not write {name}")
        measured_joints = joints - joint_offsets
        reported = fk(measured_joints) if joint_offset_deg else base_t_flange
        noise = np.r_[np.deg2rad(rng.normal(0.0, robot_noise_deg, 3)), rng.normal(0.0, robot_noise_mm / 1000, 3)]
        if index in outliers:  # pose read before the arm settled: 3 mm / 0.5 deg
            axis, direction = rng.normal(size=3), rng.normal(size=3)
            noise += np.r_[np.deg2rad(0.5) * axis / np.linalg.norm(axis), 0.003 * direction / np.linalg.norm(direction)]
        pose = fr5_from_matrix(inverse(work) @ reported @ parameters_to_matrix(noise) @ tool, euler_order)
        if index == pose_typo:
            pose[0] = round(pose[0] + 10.0, 3)
        rows.append({"image": name, "pose": pose, "joints": measured_joints.round(3).tolist()})
    write_poses(output / "poses.csv", rows)
    camera = {"image_width": WIDTH, "image_height": HEIGHT,
              "fx": float(k[0, 0] * (1 + k_error[0])), "fy": float(k[1, 1] * (1 + k_error[0])),
              "cx": float(k[0, 2] + k_error[1]), "cy": float(k[1, 2] + k_error[2]),
              "distortion": [0.0] * 5, "source": "synthetic (sim/synthetic.py)"}
    (output / "camera.yaml").write_text(yaml.safe_dump(camera, sort_keys=False), encoding="utf-8")
    with (output / "touch_points.csv").open("w", encoding="utf-8") as handle:
        handle.write("tag_id,corner,x_mm,y_mm,z_mm\n")
        for tag_id, corner in TOUCH_CORNERS:
            point = TRUTH_BASE_T_BOARD @ np.r_[board_point(tag_id, corner, board) * board_scale, 1.0]
            x, y, z = point[:3] * 1000 + rng.normal(0.0, touch_noise_mm, 3)
            handle.write(f"{tag_id},{corner},{x:.3f},{y:.3f},{z:.3f}\n")
    truth = {
        "flange_T_left_camera": TRUTH_FLANGE_T_CAMERA.tolist(), "base_T_board": TRUTH_BASE_T_BOARD.tolist(),
        "K": k.tolist(), "board_scale": board_scale,
        "settings": {"poses": poses, "seed": seed, "flip180": flip180, "euler_order": euler_order,
                     "robot_noise_mm": robot_noise_mm, "robot_noise_deg": robot_noise_deg,
                     "pose_outliers": sorted(outliers), "single_axis": single_axis,
                     "joint_offsets_deg": joint_offsets.round(4).tolist(), "k_error": list(k_error),
                     "jpeg_quality": jpeg_quality, "pose_typo_row": pose_typo + 1 if pose_typo >= 0 else None,
                     "residual_k1": residual_k1, "tcp_offset_mm": tcp_offset_mm, "user_frame": user_frame,
                     "blur_px": blur, "image_noise": image_noise},
    }
    (output / "truth.json").write_text(json.dumps(truth, indent=2), encoding="utf-8")
    return output


def compare_session(session: Path) -> dict:
    """Status, warnings and every candidate's error against truth.json."""
    session = Path(session)
    truth = json.loads((session / "truth.json").read_text(encoding="utf-8"))
    result = json.loads((session / "result.json").read_text(encoding="utf-8"))
    true_camera = np.asarray(truth["flange_T_left_camera"])
    rows = []
    for item in result.get("candidates", []):
        estimate = np.asarray(item["flange_T_left_camera"])
        rows.append({"method": item["method"], "chosen": item["method"] == result.get("chosen_method"),
                     "translation_error_mm": float(np.linalg.norm(estimate[:3, 3] - true_camera[:3, 3]) * 1000),
                     "rotation_error_deg": rotation_angle_degrees(true_camera[:3, :3].T @ estimate[:3, :3]),
                     "cv_pixel_rmse": item["cv_pixel_rmse"], "test_pixel_rmse": item["test"]["pixel_rmse"],
                     "test_board_mm": item["test"]["board_position_rmse_mm"]})
    return {"session": str(session), "status": result["status"], "reasons": result["reasons"],
            "warnings": result["warnings"], "uncertainty": result.get("uncertainty", {}),
            "intrinsics_check": result.get("intrinsics_check"), "board_scale_check": result.get("board_scale_check"),
            "candidates": rows, "chosen": next((r for r in rows if r["chosen"]), None)}


def _num(value, digits=3) -> str:
    return "-" if value is None or (isinstance(value, float) and math.isnan(value)) else f"{value:.{digits}f}"


def print_comparison(report: dict):
    print(f"status: {report['status']}" + "".join(f"\n  - {r}" for r in report["reasons"]))
    for warning in report["warnings"]:
        print(f"  warning: {warning}")
    print(f"{'method':24s} {'err mm':>8s} {'err deg':>8s} {'CV px':>7s} {'test px':>8s} {'board mm':>9s}")
    for row in report["candidates"]:
        print(f"{row['method']:24s} {row['translation_error_mm']:8.3f} {row['rotation_error_deg']:8.3f} "
              f"{_num(row['cv_pixel_rmse']):>7s} {row['test_pixel_rmse']:8.3f} {row['test_board_mm']:9.3f}"
              f"{' *' if row['chosen'] else ''}")
    sd = report["uncertainty"].get("translation_sd_norm_mm")
    if sd is not None:
        print(f"jackknife sd: {sd:.3f} mm / {report['uncertainty']['rotation_sd_deg']:.4f} deg")


def solve_session(session: Path) -> tuple[str, str]:
    """Run `python -m handeye solve` as a separate process; (status, last output line)."""
    run = subprocess.run([sys.executable, "-B", "-m", "handeye", "solve", "--session", str(session)],
                         capture_output=True, text=True, cwd=ROOT)
    lines = (run.stdout + run.stderr).strip().splitlines()
    if (session / "result.json").exists() and run.returncode in (0, 2):
        return json.loads((session / "result.json").read_text(encoding="utf-8"))["status"], lines[-1]
    return "error", lines[-1] if lines else f"exit code {run.returncode}"


CORRECT_MM, CORRECT_DEG = 2.0, 0.2  # "calibration correct" for the summary column


def _warned(o, *words):
    return any(word in text for text in o["warnings"] + o["reasons"] for word in words)


# Diagnostics that must stay quiet on good data (checked in clean / robot-noise).
FALSE_ALARM_WORDS = ("board scale", "focal", "principal point", "residual radial distortion", "user/work frame",
                     "active TCP")
FLAGS = (("S", ("board scale",)), ("K", ("focal", "principal point")), ("D", ("residual radial distortion",)),
         ("R", ("disagrees with its joints",)), ("T", ("active TCP",)), ("U", ("user/work frame",)))


def _accepted_within(mm, deg, quiet=True):
    """Accepted, within tolerance, and (quiet) no diagnostic false alarm."""
    return lambda o: (o["status"] == "accepted" and o["mm"] <= mm and o["deg"] <= deg
                      and not (quiet and _warned(o, *FALSE_ALARM_WORDS)))


# name, make_session overrides, kind, prediction, check(outcome). "check" scenarios have a
# prediction that must hold (the suite exits 1 otherwise); "measure" scenarios only report.
# A prediction can be that the tool accepts a wrong result without noticing.
SCENARIOS = [
    ("clean", {}, "check", "accepted, <= 0.5 mm / 0.05 deg, no diagnostic warning", _accepted_within(0.5, 0.05)),
    ("robot-noise", {"robot_noise_mm": 0.2, "robot_noise_deg": 0.02}, "check",
     "accepted, <= 2 mm / 0.2 deg, no diagnostic warning", _accepted_within(2.0, 0.2)),
    ("pose-outliers", {"pose_outliers": 3}, "check",
     "3 bad training poses: accepted, <= 2 mm / 0.2 deg", _accepted_within(2.0, 0.2, quiet=False)),
    ("pose-typo", {"pose_typo": 3}, "check", "x of row 4 mistyped +10 mm: row 4 flagged",
     lambda o: _warned(o, "row 4:")),
    ("board-scale", {"board_scale": 1.01}, "check", "board 1% large: flagged by the scale check",
     lambda o: _warned(o, "board scale")),
    ("k-mismatch", {"k_error": (0.005, 3.0, -3.0)}, "check", "K focal +0.5%, pp (+3,-3) px: flagged",
     lambda o: _warned(o, "focal", "principal point")),
    ("flip180", {"flip180": True}, "check", "image flipped: ACCEPTED with ~180 deg error (undetectable)",
     lambda o: o["status"] == "accepted" and o["deg"] > 170.0),
    ("euler-order", {"euler_order": "xyz"}, "check", "wrong Euler order: rejected",
     lambda o: o["status"] == "rejected"),
    ("single-axis", {"single_axis": True}, "check", "one rotation axis: insufficient_data",
     lambda o: o["status"] == "insufficient_data"),
    ("residual-dist", {"residual_k1": -0.0008}, "check", "k1 -0.0008 (~2 px at the corner) left in images: flagged",
     lambda o: _warned(o, "residual radial distortion")),
    ("tcp-frame", {"tcp_offset_mm": 120.0}, "check", "poses of a 120 mm TCP: rejected as TCP",
     lambda o: o["status"] == "rejected" and _warned(o, "active TCP")),
    ("user-frame", {"user_frame": True}, "check", "poses in a work frame: accepted, correct, flagged",
     lambda o: o["status"] == "accepted" and o["calibration_correct"] and _warned(o, "user/work frame")),
    ("robot-noise-x2.5", {"robot_noise_mm": 0.5, "robot_noise_deg": 0.05}, "measure",
     "pose noise 0.5 mm / 0.05 deg", None),
    ("joint-offsets", {"joint_offset_deg": 0.02}, "measure", "systematic 0.02 deg joint zero errors", None),
    ("jpeg-70", {"jpeg_quality": 70}, "measure", "JPEG q70 as a lossy-video stand-in", None),
    ("test-outliers", {"pose_outliers": 2, "outlier_split": "test"}, "measure", "2 bad poses among test views", None),
    ("combined", {"robot_noise_mm": 0.2, "robot_noise_deg": 0.02, "board_scale": 1.01, "k_error": (0.005, 3.0, -3.0)},
     "measure", "robot noise + board 1% + K error together", None),
]


def run_suite(output_root: Path, target: Path, poses: int, seed: int, keep_images: bool) -> list[dict]:
    output_root = Path(output_root).resolve()
    if output_root.exists():
        raise FileExistsError(f"Suite directory already exists: {output_root}")
    outcomes = []
    print(f"{'scenario':17s} {'kind':7s} {'status':17s} {'err mm':>7s} {'err deg':>7s} {'sd mm':>6s} "
          f"{'test px':>7s} {'correct':>7s} flags  prediction")
    for name, overrides, kind, prediction, check in SCENARIOS:
        started = time.monotonic()
        session = make_session(output_root / name, target, poses=poses, seed=seed, **overrides)
        status, message = solve_session(session)
        outcome = {"scenario": name, "kind": kind, "prediction": prediction, "status": status, "message": message,
                   "mm": math.nan, "deg": math.nan, "sd_mm": math.nan, "test_px": math.nan,
                   "warnings": [], "reasons": []}
        if (session / "result.json").exists():
            report = compare_session(session)
            outcome["warnings"], outcome["reasons"] = report["warnings"], report["reasons"]
            outcome["sd_mm"] = report["uncertainty"].get("translation_sd_norm_mm", math.nan)
            if report["chosen"]:
                outcome.update(method=report["chosen"]["method"], mm=report["chosen"]["translation_error_mm"],
                               deg=report["chosen"]["rotation_error_deg"],
                               test_px=report["chosen"]["test_pixel_rmse"])
        outcome["calibration_correct"] = bool(outcome["mm"] <= CORRECT_MM and outcome["deg"] <= CORRECT_DEG)
        outcome["as_predicted"] = bool(check(outcome)) if check else None
        outcome["seconds"] = round(time.monotonic() - started, 1)
        if not keep_images:
            shutil.rmtree(session / "images")
        outcomes.append(outcome)
        flags = "".join(letter for letter, words in FLAGS if _warned(outcome, *words)) or "-"
        mark = "  <-- NOT AS PREDICTED" if outcome["as_predicted"] is False else ""
        print(f"{name:17s} {kind:7s} {status:17s} {_num(outcome['mm']):>7s} {_num(outcome['deg']):>7s} "
              f"{_num(outcome['sd_mm']):>6s} {_num(outcome['test_px']):>7s} "
              f"{'yes' if outcome['calibration_correct'] else 'NO':>7s} {flags:5s}  {prediction}{mark}")
    (output_root / "summary.json").write_text(json.dumps(outcomes, indent=2, allow_nan=True), encoding="utf-8")
    return outcomes


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    make = commands.add_parser("make", help="Render one synthetic session")
    make.add_argument("--output", required=True, type=Path)
    make.add_argument("--target", type=Path, default=ROOT / "templates" / "session" / "target.yaml")
    make.add_argument("--poses", type=int, default=25)
    make.add_argument("--seed", type=int, default=0)
    make.add_argument("--board-scale", type=float, default=1.0)
    make.add_argument("--flip180", action="store_true")
    make.add_argument("--euler-order", choices=("zyx", "xyz"), default="zyx")
    make.add_argument("--robot-noise-mm", type=float, default=0.0)
    make.add_argument("--robot-noise-deg", type=float, default=0.0)
    make.add_argument("--pose-outliers", type=int, default=0)
    make.add_argument("--outlier-split", choices=("train", "test"), default="train")
    make.add_argument("--single-axis", action="store_true")
    make.add_argument("--joint-offset-deg", type=float, default=0.0)
    make.add_argument("--k-error", type=float, nargs=3, default=(0.0, 0.0, 0.0), metavar=("FOCAL_REL", "CX_PX", "CY_PX"))
    make.add_argument("--jpeg-quality", type=int, default=0)
    make.add_argument("--pose-typo", type=int, default=-1, help="0-based view index whose x_mm gets +10 mm")
    make.add_argument("--residual-k1", type=float, default=0.0, help="radial distortion left in the images")
    make.add_argument("--tcp-offset-mm", type=float, default=0.0, help="poses read with this TCP active")
    make.add_argument("--user-frame", action="store_true", help="poses read in a shifted work frame")
    compare = commands.add_parser("compare", help="Score result.json against truth.json")
    compare.add_argument("--session", required=True, type=Path)
    suite = commands.add_parser("suite", help="Clean data, injected faults and measurements")
    suite.add_argument("--output", type=Path,
                       default=ROOT / "calibration-data" / "sim_runs" / time.strftime("suite-%Y%m%d-%H%M%S"))
    suite.add_argument("--target", type=Path, default=ROOT / "templates" / "session" / "target.yaml")
    suite.add_argument("--poses", type=int, default=25)
    suite.add_argument("--seed", type=int, default=0)
    suite.add_argument("--keep-images", action="store_true")
    args = parser.parse_args()

    if args.command == "make":
        session = make_session(
            args.output, args.target, poses=args.poses, seed=args.seed, board_scale=args.board_scale,
            flip180=args.flip180, euler_order=args.euler_order, robot_noise_mm=args.robot_noise_mm,
            robot_noise_deg=args.robot_noise_deg, pose_outliers=args.pose_outliers, outlier_split=args.outlier_split,
            single_axis=args.single_axis, joint_offset_deg=args.joint_offset_deg, k_error=tuple(args.k_error),
            jpeg_quality=args.jpeg_quality, pose_typo=args.pose_typo, residual_k1=args.residual_k1,
            tcp_offset_mm=args.tcp_offset_mm, user_frame=args.user_frame)
        print(f"Session: {session}\nNext: python -m handeye solve --session \"{session}\"")
    elif args.command == "compare":
        print_comparison(compare_session(args.session))
    else:
        outcomes = run_suite(args.output, args.target, args.poses, args.seed, args.keep_images)
        checks = [o for o in outcomes if o["kind"] == "check"]
        missed = [o["scenario"] for o in checks if not o["as_predicted"]]
        silent = [o["scenario"] for o in outcomes if o["status"] == "accepted" and not o["calibration_correct"]
                  and not _warned(o, *(word for _, words in FLAGS for word in words))]
        print(f"{len(checks) - len(missed)}/{len(checks)} check scenarios as predicted; accepted wrong results "
              f"with no warning: {silent or 'none'}; summary: {Path(args.output).resolve() / 'summary.json'}")
        if missed:
            print(f"NOT AS PREDICTED: {missed}", file=sys.stderr)
            sys.exit(1)


if __name__ == "__main__":
    try:
        main()
    except (OSError, RuntimeError, ValueError) as error:
        print(f"Error: {error}", file=sys.stderr)
        sys.exit(1)
