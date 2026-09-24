"""Session folder: camera.yaml + poses.csv + images/ (+ optional target.yaml).

poses.csv, one row per photo, lines starting with # are comments:

    image,x_mm,y_mm,z_mm,rx_deg,ry_deg,rz_deg[,j1_deg,...,j6_deg]

image is a path relative to the session folder. The pose is the FLANGE
(tool 0) in the robot BASE frame (user/work frame 0), exactly as the FR5
shows it. Joint angles are optional; when present they let the solver
catch transcription errors and check the Euler convention.
"""

from __future__ import annotations

import csv
import io
import math
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import yaml

from .board import read_board
from .geometry import pose_from_fr5

POSE_COLUMNS = ("x_mm", "y_mm", "z_mm", "rx_deg", "ry_deg", "rz_deg")
JOINT_COLUMNS = tuple(f"j{i}_deg" for i in range(1, 7))


@dataclass
class Sample:
    row: int  # data row number in poses.csv, 1-based, for messages
    image: str
    pose_mm_deg: list[float]
    base_t_flange: np.ndarray = field(repr=False)
    joints_deg: list[float] | None = None


@dataclass
class Session:
    path: Path
    board: dict
    k: np.ndarray
    distortion: np.ndarray
    width: int
    height: int
    samples: list[Sample]
    camera: dict  # camera.yaml as read, including free-form notes


def validate_camera(camera: dict, source="camera.yaml") -> dict:
    """Validate the documented rectified-left contract; old metadata is optional."""
    if not isinstance(camera, dict):
        raise ValueError(f"{source}: expected a mapping")
    for key in ("image_width", "image_height"):
        if type(camera.get(key)) is not int or camera[key] <= 0:
            raise ValueError(f"{source}: {key} must be a positive integer")
    for key in ("fx", "fy", "cx", "cy"):
        if (type(camera.get(key)) not in (int, float) or not math.isfinite(camera[key])
                or camera[key] <= 0):
            raise ValueError(f"{source}: missing or invalid {key}: expected a finite positive number")
    distortion = camera.get("distortion", [0.0] * 5)
    if not isinstance(distortion, list):
        raise ValueError(f"{source}: distortion must be a list of numbers")
    if len(distortion) not in (4, 5, 8, 12, 14):  # the lengths OpenCV accepts
        raise ValueError(f"{source}: distortion needs 4, 5, 8, 12 or 14 coefficients, got {len(distortion)}")
    for index, value in enumerate(distortion):
        if type(value) not in (int, float) or not math.isfinite(value):
            raise ValueError(f"{source}: distortion[{index}] must be a finite number")
    if "image_flip" in camera:
        flip = camera["image_flip"]
        # PyYAML interprets an unquoted OFF as False and ON as True.
        if flip is not False and not (isinstance(flip, str) and flip.upper().split(".")[-1] == "OFF"):
            raise ValueError(f"{source}: image_flip must be OFF for the calibration camera frame")
    for key in ("image_view", "view"):
        if key in camera and str(camera[key]).upper().split(".")[-1] != "LEFT":
            raise ValueError(f"{source}: {key} must be LEFT (rectified left image)")
    for key in ("rectified", "image_rectified"):
        if key in camera and camera[key] is not True:
            raise ValueError(f"{source}: {key} must be true (raw images are not supported)")
    if ("calibration_source" in camera
            and camera["calibration_source"] != "calibration_parameters.left_cam"):
        raise ValueError(f"{source}: calibration_source must be calibration_parameters.left_cam")
    if "camera_fps" in camera:
        fps = camera["camera_fps"]
        if type(fps) not in (int, float) or not math.isfinite(fps) or fps <= 0:
            raise ValueError(f"{source}: camera_fps must be a finite positive number")
    return camera


def read_camera(path: Path) -> dict:
    try:
        camera = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    except yaml.YAMLError as exc:
        raise ValueError(f"{path}: not valid YAML: {exc}") from exc
    return validate_camera(camera, path)


def camera_matrix(camera: dict) -> np.ndarray:
    return np.array([[camera["fx"], 0.0, camera["cx"]], [0.0, camera["fy"], camera["cy"]], [0.0, 0.0, 1.0]])


def read_poses(path: Path) -> list[Sample]:
    numbered_lines = [(number, line) for number, line in
                      enumerate(Path(path).read_text(encoding="utf-8-sig").splitlines(), 1)
                      if line.strip() and not line.lstrip().startswith("#")]
    reader = csv.DictReader(io.StringIO("\n".join(line for _, line in numbered_lines)))
    header = [name.strip() for name in reader.fieldnames or []]
    if len(set(header)) != len(header):
        raise ValueError(f"{path}: repeated column names in header")
    missing = [name for name in ("image", *POSE_COLUMNS) if name not in header]
    if missing:
        raise ValueError(f"{path}: missing columns {missing}")
    has_joints = [name in header for name in JOINT_COLUMNS]
    if any(has_joints) and not all(has_joints):
        raise ValueError(f"{path}: give all of {list(JOINT_COLUMNS)} or none")
    samples, seen = [], set()
    for number, raw in enumerate(reader, start=1):
        line = numbered_lines[min(reader.line_num - 1, len(numbered_lines) - 1)][0]
        location = f"{path}: row {number} (line {line})"
        if None in raw:
            raise ValueError(f"{location}: more fields than the header")
        row = {key.strip(): (value or "").strip() for key, value in raw.items() if key}
        image = row["image"]
        if not image or image in seen:
            raise ValueError(f"{location}: empty or repeated image name {image!r}")
        seen.add(image)

        def values(columns):
            result = []
            for name in columns:
                try:
                    value = float(row[name])
                except (ValueError, KeyError) as exc:
                    raise ValueError(f"{location} ({image}): {name} must be a finite number") from exc
                if not math.isfinite(value):
                    raise ValueError(f"{location} ({image}): {name} must be a finite number")
                result.append(value)
            return result

        pose = values(POSE_COLUMNS)
        joints = values(JOINT_COLUMNS) if all(has_joints) else None
        matrix = pose_from_fr5(pose)
        samples.append(Sample(number, image, pose, matrix, joints))
    if not samples:
        raise ValueError(f"{path}: no pose rows; add one row per image below the header")
    return samples


def load_session(path: Path, target: Path | None = None) -> Session:
    path = Path(path).resolve()
    target = Path(target) if target else path / "target.yaml"
    if not target.exists():
        raise FileNotFoundError(f"No board definition: pass --target or put target.yaml in {path}")
    camera = read_camera(path / "camera.yaml")
    return Session(path=path, board=read_board(target), k=camera_matrix(camera),
                   distortion=np.asarray(camera.get("distortion", [0.0] * 5), dtype=np.float64),
                   width=int(camera["image_width"]), height=int(camera["image_height"]),
                   samples=read_poses(path / "poses.csv"), camera=camera)


def write_poses(path: Path, rows: list[dict]):
    """rows: {"image": str, "pose": [6], "joints": [6] or None}."""
    with_joints = all(row.get("joints") is not None for row in rows)
    with Path(path).open("w", encoding="utf-8", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["image", *POSE_COLUMNS, *(JOINT_COLUMNS if with_joints else ())])
        for row in rows:
            writer.writerow([row["image"], *row["pose"], *(row["joints"] if with_joints else ())])
