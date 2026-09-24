"""Independent check: board corners touched with a calibrated pointer tool.

touch_points.csv (lines starting with # are comments):

    tag_id,corner,x_mm,y_mm,z_mm

corner is the physical corner of that tag as seen on the board: BL, BR, TL
or TR (Kalibr layout: tag 0 bottom-left, +x right, +y up). x/y/z is the
pointer TCP in the robot base frame. The calibration predicts these points
as base_T_board @ corner; the difference tests the whole chain (hand-eye,
robot accuracy, board size) against a measurement it never saw. The pointer
TCP's own calibration error adds to the result.
"""

from __future__ import annotations

import csv
import io
from pathlib import Path

import numpy as np

from .board import CORNER_NAMES, board_point
from .geometry import inverse, rotation_angle_degrees, transform


def read_touch_points(path: Path) -> list[dict]:
    lines = [line for line in Path(path).read_text(encoding="utf-8-sig").splitlines()
             if line.strip() and not line.lstrip().startswith("#")]
    points = []
    for number, row in enumerate(csv.DictReader(io.StringIO("\n".join(lines))), start=1):
        row = {key.strip(): (value or "").strip() for key, value in row.items() if key}
        try:
            corner = row["corner"].upper()
            if corner not in CORNER_NAMES:
                raise ValueError(f"corner must be one of {CORNER_NAMES}")
            points.append({"row": number, "tag_id": int(row["tag_id"]), "corner": corner,
                           "base_mm": [float(row[name]) for name in ("x_mm", "y_mm", "z_mm")]})
        except (KeyError, ValueError) as exc:
            raise ValueError(f"{path}: row {number}: {exc}") from exc
    return points


def kabsch(model: np.ndarray, measured: np.ndarray) -> np.ndarray:
    """Rigid transform T minimising |T @ model_i - measured_i| (both Nx3, metres)."""
    cm, cp = model.mean(axis=0), measured.mean(axis=0)
    u, _, vt = np.linalg.svd((model - cm).T @ (measured - cp))
    d = np.sign(np.linalg.det(vt.T @ u.T))
    rotation = vt.T @ np.diag([1.0, 1.0, d]) @ u.T
    return transform(rotation, cp - rotation @ cm)


def compare_results(results: list[dict], names: list[str]) -> list[dict]:
    """Pairwise difference of flange_T_left_camera between independent calibrations.

    Each result's jackknife spread covers its random errors, so two sessions of
    the same mount should differ by about sqrt(sd_a^2 + sd_b^2). A much larger
    difference means something changed or was wrong in one session (mount
    moved, warm-up, pose frame, bad poses). Agreement does not rule out errors
    both share: board size, intrinsics, image flip, systematic robot error.
    """
    rows = []
    for i in range(len(results)):
        for j in range(i + 1, len(results)):
            a, b = (np.asarray(results[n]["chosen"]["flange_T_left_camera"]) for n in (i, j))
            delta = inverse(a) @ b
            sd = [results[n].get("uncertainty", {}) for n in (i, j)]
            combined_mm = combined_deg = None
            if all("translation_sd_norm_mm" in s for s in sd):
                combined_mm = float(np.hypot(sd[0]["translation_sd_norm_mm"], sd[1]["translation_sd_norm_mm"]))
                combined_deg = float(np.hypot(sd[0]["rotation_sd_deg"], sd[1]["rotation_sd_deg"]))
            rows.append({"a": names[i], "b": names[j],
                         "translation_mm": float(np.linalg.norm(delta[:3, 3]) * 1000),
                         "translation_delta_mm": ((b[:3, 3] - a[:3, 3]) * 1000).round(3).tolist(),
                         "rotation_deg": rotation_angle_degrees(delta[:3, :3]),
                         "expected_random_mm": combined_mm, "expected_random_deg": combined_deg})
    return rows


def validate(base_t_board: np.ndarray, board: dict, points: list[dict]) -> dict:
    tags = board["tagRows"] * board["tagCols"]
    for p in points:
        if not 0 <= p["tag_id"] < tags:
            raise ValueError(f"touch point row {p['row']}: tag {p['tag_id']} is not on this {tags}-tag board")
    model = np.array([board_point(p["tag_id"], p["corner"], board) for p in points])
    measured = np.array([p["base_mm"] for p in points]) / 1000.0
    predicted = (base_t_board[:3, :3] @ model.T + base_t_board[:3, 3:4]).T
    errors = np.linalg.norm(predicted - measured, axis=1) * 1000.0
    result = {"points": [{**p, "predicted_mm": (q * 1000).round(3).tolist(), "error_mm": round(float(e), 3)}
                         for p, q, e in zip(points, predicted, errors)],
              "rms_mm": float(np.sqrt(np.mean(errors ** 2))), "max_mm": float(errors.max())}
    spread = np.linalg.svd(model - model.mean(axis=0), compute_uv=False)
    if len(points) >= 3 and spread[1] > 0.01:  # not collinear
        measured_t_board = kabsch(model, measured)
        fitted = (measured_t_board[:3, :3] @ model.T + measured_t_board[:3, 3:4]).T
        delta = inverse(base_t_board) @ measured_t_board
        result["board_pose_difference"] = {
            "translation_mm": float(np.linalg.norm(delta[:3, 3]) * 1000),
            "rotation_deg": rotation_angle_degrees(delta[:3, :3]),
            # residual of the touched points against the board model: pointer and board flatness
            "touch_fit_rms_mm": float(np.sqrt(np.mean(np.sum((fitted - measured) ** 2, axis=1))) * 1000)}
    return result
