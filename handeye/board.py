"""Kalibr Aprilgrid geometry and corner detection."""

from __future__ import annotations

from pathlib import Path

import cv2
import numpy as np
import yaml

# Physical names of OpenCV's corner order 0..3 on a Kalibr tag (see tag_object_corners).
CORNER_NAMES = ("BR", "BL", "TL", "TR")


def read_board(path: Path) -> dict:
    try:
        board = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    except yaml.YAMLError as exc:
        raise ValueError(f"{path}: not valid YAML: {exc}") from exc
    if not isinstance(board, dict) or board.get("target_type") != "aprilgrid":
        raise ValueError(f"{path}: target_type must be aprilgrid")
    for key in ("tagCols", "tagRows"):
        if not isinstance(board.get(key), int) or board[key] < 2:
            raise ValueError(f"{path}: invalid {key}")
    for key in ("tagSize", "tagSpacing"):
        if not isinstance(board.get(key), (int, float)) or board[key] <= 0:
            raise ValueError(f"{path}: invalid {key}")
    if board["tagCols"] * board["tagRows"] > 587:
        raise ValueError(f"{path}: more tags than AprilTag 36h11 supports")
    return {k: board[k] for k in ("target_type", "tagCols", "tagRows", "tagSize", "tagSpacing")}


def tag_object_corners(tag_id: int, board: dict) -> np.ndarray:
    """Corners of one Kalibr tag [m], in OpenCV's canonical marker corner order.

    The Kalibr board origin is the bottom-left of tag 0; +x right, +y up.
    IDs grow left-to-right, then bottom-to-top. The Kalibr PDF generator
    rotates each code 180 degrees from OpenCV's canonical AprilTag code, so
    OpenCV corners 0..3 are the physical BR, BL, TL, TR corners.
    """
    row, col = divmod(tag_id, board["tagCols"])
    size = float(board["tagSize"])
    pitch = size * (1.0 + float(board["tagSpacing"]))
    x, y = col * pitch, row * pitch
    return np.array([[x + size, y, 0], [x, y, 0], [x, y + size, 0], [x + size, y + size, 0]],
                    dtype=np.float64)


def board_point(tag_id: int, corner: str, board: dict) -> np.ndarray:
    """One named corner (BR, BL, TL, TR) of a tag, in board coordinates [m]."""
    return tag_object_corners(tag_id, board)[CORNER_NAMES.index(corner.upper())]


def board_outline(board: dict) -> np.ndarray:
    """Outer corners of the tag array (not the small gap squares) [m]."""
    pitch = board["tagSize"] * (1.0 + board["tagSpacing"])
    w = board["tagCols"] * pitch - board["tagSize"] * board["tagSpacing"]
    h = board["tagRows"] * pitch - board["tagSize"] * board["tagSpacing"]
    return np.array([[0, 0, 0], [w, 0, 0], [w, h, 0], [0, h, 0]], dtype=np.float64)


def detector():
    if not hasattr(cv2, "aruco") or not hasattr(cv2.aruco, "ArucoDetector"):
        raise RuntimeError("OpenCV aruco module missing; install opencv-contrib-python >= 4.8")
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    params = cv2.aruco.DetectorParameters()
    params.markerBorderBits = 2  # Kalibr kalibr_create_target_pdf default
    # Line fits give the closest start for the cornerSubPix pass in detect_board.
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_CONTOUR
    return cv2.aruco.ArucoDetector(dictionary, params)


def detect_board(image: np.ndarray, board: dict, aruco_detector) -> tuple[np.ndarray, np.ndarray, list[int]]:
    """(object points [m], image points [px], tag ids); 4 points per tag, ids ascending."""
    if image.ndim == 2:
        gray = image
    elif image.shape[2] == 4:
        gray = cv2.cvtColor(image, cv2.COLOR_BGRA2GRAY)
    else:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = aruco_detector.detectMarkers(gray)
    if ids is None:
        return np.empty((0, 3)), np.empty((0, 2)), []
    pairs = sorted(zip(ids.reshape(-1).tolist(), corners), key=lambda pair: pair[0])
    object_points, image_points, used_ids = [], [], []
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 50, 0.001)
    for tag_id, detected in pairs:
        if not 0 <= tag_id < board["tagRows"] * board["tagCols"]:
            continue
        # Kalibr's gap squares touch every tag corner, making it an X-junction.
        # ArucoDetector's corners sit ~1-2 px off there and its own SUBPIX
        # window (~0.3 module) cannot pull them back; rendered boards gave
        # 1.4 px mean error without this pass and 0.03 px with it. Keep the
        # window inside the gap square so it sees only this junction. Corners
        # that start further off than the window stay put; fit_pose drops them.
        quad = np.asarray(detected, dtype=np.float32).reshape(4, 1, 2)
        edge = float(np.mean(np.linalg.norm(quad[:, 0] - np.roll(quad[:, 0], 1, axis=0), axis=1)))
        half = int(np.clip(round(0.4 * edge * board["tagSpacing"]), 3, 10))
        cv2.cornerSubPix(gray, quad, (half, half), (-1, -1), criteria)
        object_points.extend(tag_object_corners(tag_id, board))
        image_points.extend(quad.reshape(4, 2))
        used_ids.append(int(tag_id))
    return (np.asarray(object_points, dtype=np.float64).reshape(-1, 3),
            np.asarray(image_points, dtype=np.float64).reshape(-1, 2), used_ids)
