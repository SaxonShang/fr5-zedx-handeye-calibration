"""Rigid transforms, the FR5 pose convention and Unicode-safe image I/O.

Frames use A_T_B to map points expressed in B into frame A. Calculations use
metres and radians; FR5 poses are [x, y, z] in mm and [rx, ry, rz] in deg.
"""

from __future__ import annotations

import math
from pathlib import Path

import cv2
import numpy as np


def transform(rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    result = np.eye(4)
    result[:3, :3] = np.asarray(rotation).reshape(3, 3)
    result[:3, 3] = np.asarray(translation).reshape(3)
    return result


def inverse(t: np.ndarray) -> np.ndarray:
    result = np.eye(4)
    result[:3, :3] = t[:3, :3].T
    result[:3, 3] = -result[:3, :3] @ t[:3, 3]
    return result


def rotation_angle_degrees(rotation: np.ndarray) -> float:
    # atan2 stays accurate near 0 and 180 deg, unlike arccos of the trace
    # (whose floor, ~5e-4 deg, is comparable to the errors being reported).
    r = np.asarray(rotation)
    sine = np.linalg.norm([r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1]]) / 2.0
    return float(np.rad2deg(np.arctan2(sine, (np.trace(r) - 1.0) / 2.0)))


def rotation_mean(rotations: list[np.ndarray]) -> np.ndarray:
    u, _, vt = np.linalg.svd(np.sum(rotations, axis=0))
    return u @ np.diag([1.0, 1.0, np.linalg.det(u @ vt)]) @ vt


def transform_mean(transforms: list[np.ndarray]) -> np.ndarray:
    return transform(rotation_mean([t[:3, :3] for t in transforms]),
                     np.mean([t[:3, 3] for t in transforms], axis=0))


def rotation_vector(rotation: np.ndarray) -> np.ndarray:
    """Axis * angle [rad] of a rotation matrix, accurate for tiny angles.

    cv2.Rodrigues returns exactly zero below ~1e-5 rad, so a residual built
    on it cannot see such errors (an IK round trip stopped 2e-6 rad off).
    Near 180 degrees the axis direction comes from cv2.Rodrigues (which
    rounds that angle to pi) and the angle from atan2.
    """
    r = np.asarray(rotation, dtype=np.float64)
    v = 0.5 * np.array([r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1]])
    s, c = float(np.linalg.norm(v)), (np.trace(r) - 1.0) / 2.0
    if c < 0.0 and s < 1e-3:
        axis = cv2.Rodrigues(r)[0].ravel()
        axis /= np.linalg.norm(axis)
        if np.dot(axis, v) < 0.0:
            axis = -axis
        return axis * math.atan2(s, c)
    return v if s < 1e-12 else v * (math.atan2(s, c) / s)


def matrix_to_parameters(t: np.ndarray) -> np.ndarray:
    """[rotation vector (3), translation (3)]."""
    return np.r_[rotation_vector(t[:3, :3]), t[:3, 3]]


def parameters_to_matrix(p: np.ndarray) -> np.ndarray:
    rot, _ = cv2.Rodrigues(np.asarray(p[:3], dtype=np.float64))
    return transform(rot, p[3:6])


def pose_from_fr5(raw) -> np.ndarray:
    """FR5 [x, y, z mm, rx, ry, rz deg] -> 4x4 in metres.

    FAIRINO's moving-axis ZYX convention equals fixed-axis XYZ:
    R = Rz(rz) @ Ry(ry) @ Rx(rx) (robot brief introduction manual).
    """
    if len(raw) != 6 or not all(math.isfinite(float(v)) for v in raw):
        raise ValueError("An FR5 pose needs six finite values")
    x, y, z = np.asarray(raw[:3], dtype=float) / 1000.0
    rx, ry, rz = np.deg2rad(np.asarray(raw[3:], dtype=float))
    cx, sx, cy, sy, cz, sz = math.cos(rx), math.sin(rx), math.cos(ry), math.sin(ry), math.cos(rz), math.sin(rz)
    rot_x = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    rot_y = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    rot_z = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    return transform(rot_z @ rot_y @ rot_x, [x, y, z])


def fr5_from_matrix(t: np.ndarray, order: str = "zyx", decimals: int = 3) -> list[float]:
    """Inverse of pose_from_fr5, rounded like the controller display.

    order="xyz" (R = Rx @ Ry @ Rz) exists only to simulate a wrong convention.
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
    return [round(float(v), decimals) for v in (*(t[:3, 3] * 1000.0), *np.rad2deg([rx, ry, rz]))]


def read_image(path: Path) -> np.ndarray | None:
    """cv2.imread that also works for non-ASCII Windows paths."""
    try:
        data = np.fromfile(str(path), dtype=np.uint8)
    except OSError:
        return None
    return cv2.imdecode(data, cv2.IMREAD_UNCHANGED) if data.size else None


def write_image(path: Path, image: np.ndarray) -> bool:
    ok, encoded = cv2.imencode(Path(path).suffix or ".png", image)
    if ok:
        Path(path).write_bytes(encoded.tobytes())
    return bool(ok)
