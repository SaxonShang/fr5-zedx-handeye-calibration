"""Board-pose checks and independent hand-eye-chain validation using touched corners.

Touch coordinates are in the robot base frame, measured with a calibrated
pointer. Checking a saved base_T_board tests that board pose only. Checking
X needs a separate image/robot-pose session with the board and mount unchanged.
"""

from __future__ import annotations

import csv
import hashlib
import io
import math
from pathlib import Path

import cv2
import numpy as np

from .board import CORNER_NAMES, board_outline, board_point
from .dataset import validate_camera
from .geometry import inverse, read_image, rotation_angle_degrees, transform


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
            xyz = [float(row[name]) for name in ("x_mm", "y_mm", "z_mm")]
            if not all(math.isfinite(v) for v in xyz):
                raise ValueError("touch coordinates must be finite")
            points.append({"row": number, "tag_id": int(row["tag_id"]), "corner": corner, "base_mm": xyz})
        except (KeyError, ValueError) as exc:
            raise ValueError(f"{path}: row {number}: {exc}") from exc
    return points


def checked_transform(value, name: str) -> np.ndarray:
    try:
        matrix = np.asarray(value, dtype=float)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name}: expected a finite SE(3) matrix") from exc
    if (matrix.shape != (4, 4) or not np.all(np.isfinite(matrix))
            or not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-8, rtol=0)
            or not np.allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3), atol=1e-6, rtol=0)
            or not np.isclose(np.linalg.det(matrix[:3, :3]), 1.0, atol=1e-6, rtol=0)):
        raise ValueError(f"{name}: expected a finite SE(3) matrix")
    return matrix


def _result_frames(result: dict, require_base: bool = False) -> dict:
    if not isinstance(result, dict):
        raise ValueError("result must be a JSON object")
    if result.get("schema_version") != 2 or not isinstance(result.get("frames"), dict):
        raise ValueError("result has no supported frame declaration; run solve again with the current version")
    if result.get("status") != "accepted" or not isinstance(result.get("chosen"), dict):
        raise ValueError(f"result is {result.get('status')!r}, not an accepted calibration")
    frames = result["frames"]
    if frames.get("pose_moving") != "flange":
        raise ValueError("this check requires a flange result; an unknown reported TCP cannot be compared or validated")
    if frames.get("pose_reference") not in ("base", "reported_work_frame"):
        raise ValueError("result has an unknown pose reference frame; run solve again")
    if require_base and frames.get("pose_reference") != "base":
        raise ValueError("touch coordinates require a result declared in the robot base frame")
    if (frames.get("camera") != "zed_left_rectified_optical"
            or frames.get("board") != "kalibr_aprilgrid"):
        raise ValueError("unsupported camera or board frame declaration")
    chosen = result["chosen"]
    x = checked_transform(chosen.get("pose_moving_T_left_camera"), "pose_moving_T_left_camera")
    y = checked_transform(chosen.get("pose_reference_T_board"), "pose_reference_T_board")
    aliases = {"left_camera_T_pose_moving": inverse(x), "flange_T_left_camera": x,
               "left_camera_T_flange": inverse(x), "base_T_board": y}
    for name, expected in aliases.items():
        if name in chosen:
            if name == "base_T_board" and frames["pose_reference"] != "base":
                raise ValueError("base_T_board alias contradicts the declared reference frame")
            matrix = checked_transform(chosen[name], name)
            if not np.allclose(matrix, expected, atol=1e-8, rtol=0):
                raise ValueError(f"{name} contradicts the generic transform; regenerate the result with solve")
    validate_camera(result.get("camera"), "result camera")
    return frames


def _camera_compatible(a: dict, b: dict) -> list[str]:
    warnings = []
    validate_camera(a, "first camera")
    validate_camera(b, "second camera")
    keys = ("image_width", "image_height", "fx", "fy", "cx", "cy")
    try:
        values = [np.asarray([camera[key] for key in keys], dtype=float) for camera in (a, b)]
        distortion = [np.asarray(camera.get("distortion", [0.] * 5), dtype=float) for camera in (a, b)]
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError("camera configuration is missing or invalid") from exc
    if not all(np.all(np.isfinite(v)) for v in values + distortion):
        raise ValueError("camera configuration must be finite")
    if (not np.allclose(*values, rtol=1e-9, atol=1e-6)
            or distortion[0].shape != distortion[1].shape
            or not np.allclose(*distortion, rtol=1e-9, atol=1e-9)):
        raise ValueError("camera intrinsics/resolution/distortion differ between sessions")
    missing = []
    for key in ("camera_serial", "camera_model", "image_flip", "resolution", "self_calibration", "view",
                "image_view", "rectified", "image_rectified", "calibration_source", "zed_sdk", "camera_fps"):
        va, vb = a.get(key), b.get(key)
        unknown = (None, "", "?")
        if va in unknown or vb in unknown or (key == "camera_serial" and (va in (0, "0") or vb in (0, "0"))):
            missing.append(key)
            continue
        if key == "image_flip":  # validate_camera already established both mean OFF
            va = vb = "OFF"
        if key in ("view", "image_view"):
            va, vb = str(va).upper().split(".")[-1], str(vb).upper().split(".")[-1]
        if va != vb:
            raise ValueError(f"camera identity/settings differ: {key}")
    if missing:
        warnings.append("unrecorded camera metadata (compatibility relies on operator confirmation): " + ", ".join(missing))
    return warnings


def _pixel_hash(path: Path) -> str:
    image = read_image(path)
    if image is None:
        raise ValueError(f"cannot read source image {path}; keep the original calibration images for independence checks")
    # The detector uses luminance; normalise colour/alpha so a grayscale-to-RGB
    # conversion cannot disguise the same observations as an independent session.
    if image.ndim == 3:
        image = cv2.cvtColor(image, cv2.COLOR_BGRA2GRAY if image.shape[2] == 4 else cv2.COLOR_BGR2GRAY)
    # Hash decoded pixels to detect copied images even when losslessly re-encoded.
    digest = hashlib.sha256(str((image.shape, image.dtype.str)).encode("ascii"))
    digest.update(np.ascontiguousarray(image).tobytes())
    return digest.hexdigest()


def _source_hashes(result: dict, required: bool = False) -> set[str] | None:
    source = result.get("session")
    views = result.get("accepted_views", [])
    if not source or not views:
        if required:
            raise ValueError("result lacks source-session images; run solve again and retain its original session")
        return None
    paths = [Path(source) / view["image"] for view in views]
    if not all(path.is_file() for path in paths):
        if required:
            raise ValueError("original calibration images are unavailable; retain them to check validation independence")
        return None
    return {_pixel_hash(path) for path in paths}


def _touch_geometry(board: dict, points: list[dict]):
    tags = board["tagRows"] * board["tagCols"]
    for point in points:
        if not 0 <= point["tag_id"] < tags:
            raise ValueError(f"touch point row {point['row']}: tag {point['tag_id']} is not on this {tags}-tag board")
        if point["corner"] not in CORNER_NAMES:
            raise ValueError(f"touch point row {point['row']}: invalid corner")
    if len(points) < 3:
        raise ValueError("need at least 3 distinct, non-collinear touch points")
    names = [(p["tag_id"], p["corner"]) for p in points]
    if len(set(names)) != len(names):
        raise ValueError("repeated tag/corner touch point; use distinct board corners")
    model = np.asarray([board_point(p["tag_id"], p["corner"], board) for p in points], dtype=float)
    measured = np.asarray([p["base_mm"] for p in points], dtype=float) / 1000.0
    if measured.shape != model.shape or not np.all(np.isfinite(model)) or not np.all(np.isfinite(measured)):
        raise ValueError("touch coordinates and board geometry must be finite Nx3 arrays")
    for values in (model, measured):
        distances = np.linalg.norm(values[:, None] - values[None, :], axis=2)
        np.fill_diagonal(distances, np.inf)
        if np.any(distances < 1e-8):
            raise ValueError("repeated physical touch coordinate; measure distinct corners")
    spreads = [np.linalg.svd(values - values.mean(axis=0), compute_uv=False) / np.sqrt(len(points))
               for values in (model, measured)]
    # Require at least 10 mm RMS spread in the second direction and avoid nearly
    # collinear points. This is a project validation default, not a vendor limit.
    if any(s[1] < 0.01 or s[1] / s[0] < 0.05 for s in spreads):
        raise ValueError("touch points are collinear, nearly collinear, or too tightly clustered; spread them over the board")
    extents = np.ptp(board_outline(board), axis=0)[:2]
    coverage = {"board_span_fraction_xy": (np.ptp(model, axis=0)[:2] / extents).tolist(),
                "model_second_axis_rms_mm": float(spreads[0][1] * 1000),
                "measured_second_axis_rms_mm": float(spreads[1][1] * 1000)}
    return model, measured, coverage


def kabsch(model: np.ndarray, measured: np.ndarray) -> np.ndarray:
    """Rigid transform T minimising |T @ model_i - measured_i| (both Nx3, metres)."""
    cm, cp = model.mean(axis=0), measured.mean(axis=0)
    u, _, vt = np.linalg.svd((model - cm).T @ (measured - cp))
    d = np.sign(np.linalg.det(vt.T @ u.T))
    rotation = vt.T @ np.diag([1.0, 1.0, d]) @ u.T
    return transform(rotation, cp - rotation @ cm)


def compare_results(results: list[dict], names: list[str]) -> list[dict]:
    """Compare accepted flange/camera results from distinct sessions.

    Agreement is repeatability, not accuracy. Shared systematic errors remain
    invisible; missing original images prevent verification of independence.
    """
    if len(results) < 2 or len(names) != len(results):
        raise ValueError("compare requires at least two independent result files")
    if len({Path(name).resolve() for name in names}) != len(names):
        raise ValueError("compare requires distinct result file paths")
    if any(not isinstance(r, dict) or not r.get("session") for r in results):
        raise ValueError("result has no source session; run solve again")
    if len({Path(r["session"]).resolve() for r in results}) != len(results):
        raise ValueError("compare requires different source sessions, not copies of one result")
    frames = [_result_frames(result) for result in results]
    hashes = [_source_hashes(result) for result in results]
    rows = []
    for i in range(len(results)):
        for j in range(i + 1, len(results)):
            frame_keys = ("pose_moving", "pose_reference", "camera", "board")
            if any(frames[i].get(key) != frames[j].get(key) for key in frame_keys):
                raise ValueError("result frame declarations differ")
            warnings = _camera_compatible(results[i].get("camera", {}), results[j].get("camera", {}))
            if frames[i].get("basis") != frames[j].get("basis"):
                warnings.append("frame declarations have different joint-check evidence")
            if hashes[i] is not None and hashes[j] is not None:
                if hashes[i] & hashes[j]:
                    raise ValueError("source sessions reuse image pixels; compare needs independent observations")
            else:
                warnings.append("original images unavailable: session independence relies on recorded source paths")
            a, b = (checked_transform(results[n]["chosen"]["pose_moving_T_left_camera"], "hand-eye") for n in (i, j))
            delta = inverse(a) @ b
            sd = [results[n].get("uncertainty", {}) for n in (i, j)]
            combined_mm = combined_deg = None
            if all("translation_sd_norm_mm" in s and "rotation_sd_deg" in s for s in sd):
                values = [s[key] for s in sd for key in ("translation_sd_norm_mm", "rotation_sd_deg")]
                if not all(isinstance(v, (int, float)) and math.isfinite(v) and v >= 0 for v in values):
                    raise ValueError("uncertainty values must be finite and nonnegative")
                combined_mm = float(np.hypot(sd[0]["translation_sd_norm_mm"], sd[1]["translation_sd_norm_mm"]))
                combined_deg = float(np.hypot(sd[0]["rotation_sd_deg"], sd[1]["rotation_sd_deg"]))
            rows.append({"a": names[i], "b": names[j], "scope": "repeatability_check", "warnings": warnings,
                         "translation_mm": float(np.linalg.norm(delta[:3, 3]) * 1000),
                         "translation_delta_mm": ((b[:3, 3] - a[:3, 3]) * 1000).round(3).tolist(),
                         "rotation_deg": rotation_angle_degrees(delta[:3, :3]),
                         "expected_random_mm": combined_mm, "expected_random_deg": combined_deg})
    return rows


def validate(base_t_board: np.ndarray, board: dict, points: list[dict]) -> dict:
    """Check a supplied board pose only; this does not test flange_T_camera."""
    base_t_board = checked_transform(base_t_board, "base_T_board")
    model, measured, coverage = _touch_geometry(board, points)
    predicted = (base_t_board[:3, :3] @ model.T + base_t_board[:3, 3:4]).T
    errors = np.linalg.norm(predicted - measured, axis=1) * 1000.0
    measured_t_board = kabsch(model, measured)
    fitted = (measured_t_board[:3, :3] @ model.T + measured_t_board[:3, 3:4]).T
    delta = inverse(base_t_board) @ measured_t_board
    return {"scope": "board_pose_check", "note": "Checks saved board pose Y only; does not validate hand-eye X.",
            "touch_coverage": coverage,
            "points": [{**p, "predicted_mm": (q * 1000).round(3).tolist(), "error_mm": round(float(e), 3)}
                       for p, q, e in zip(points, predicted, errors)],
            "rms_mm": float(np.sqrt(np.mean(errors ** 2))), "max_mm": float(errors.max()),
            "board_pose_difference": {
                "translation_mm": float(np.linalg.norm(delta[:3, 3]) * 1000),
                "rotation_deg": rotation_angle_degrees(delta[:3, :3]),
                "touch_fit_rms_mm": float(np.sqrt(np.mean(np.sum((fitted - measured) ** 2, axis=1))) * 1000)}}


def validate_result(result: dict, board: dict, points: list[dict], session=None) -> dict:
    """Validate saved Y, or independently exercise X with a new static session."""
    frames = _result_frames(result, require_base=True)
    if board != result.get("board"):
        raise ValueError("target definition differs from the calibration; use the same exact board geometry")
    if session is None:
        report = validate(result["chosen"]["pose_reference_T_board"], board, points)
    else:
        from .observations import build_observations
        from .solver import check_motion_diversity

        source = result.get("session")
        if not source or Path(source).resolve() == session.path.resolve():
            raise ValueError("hand-eye validation requires a different, independently collected session")
        if session.board != board:
            raise ValueError("validation target differs from the calibration")
        warnings = _camera_compatible(result.get("camera", {}), session.camera)
        original = _source_hashes(result, required=True)
        validation_hashes = [_pixel_hash(session.path / sample.image) for sample in session.samples]
        if original.intersection(validation_hashes):
            raise ValueError("validation session reuses calibration image pixels; collect new observations")
        if len(set(validation_hashes)) != len(validation_hashes):
            raise ValueError("validation session contains repeated image pixels")
        observations, rejected = build_observations(session)
        if len(observations) < 3:
            raise ValueError("hand-eye validation needs at least 3 usable independent views")
        check_motion_diversity(observations)
        x = checked_transform(result["chosen"]["pose_moving_T_left_camera"], "flange_T_left_camera")
        views = []
        for obs in observations:
            y = checked_transform(obs["base_T_flange"] @ x @ obs["camera_T_board"], "independent chain board pose")
            view = validate(y, board, points)
            views.append({"row": obs["row"], "image": obs["image"], **view})
        report = {"scope": "handeye_chain_check", "source_session": str(session.path),
                  "note": "Uses new base_T_flange @ fixed X @ camera_T_board and touches; saved Y is not used. "
                          "Keep board and camera mount unchanged; record tool 0 poses in base/work frame 0.",
                  "warnings": warnings, "views": views, "rejected_views": rejected,
                  "rms_mm": float(np.sqrt(np.mean([view["rms_mm"] ** 2 for view in views]))),
                  "max_mm": max(view["max_mm"] for view in views),
                  "max_view_rms_mm": max(view["rms_mm"] for view in views)}
    report["source_status"] = result["status"]
    report["frames"] = frames
    return report
