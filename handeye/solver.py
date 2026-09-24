"""Eye-in-hand calibration: unknown flange_T_camera (X) and base_T_board (Y).

Every view i gives base_T_flange_i @ X @ camera_T_board_i = Y. Park and
Daniilidis solve the relative-motion form AX = XB; the refinement minimises
corner reprojection error over X and Y jointly (robot poses taken as exact).
The method is chosen by k-fold cross-validation on training views; every
fifth view is a test view used only for reporting and the quality gate.
"""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass

import cv2
import numpy as np

from . import diagnostics
from .dataset import Session
from .geometry import inverse, matrix_to_parameters, parameters_to_matrix, rotation_angle_degrees, \
    transform, transform_mean
from .kinematics import frame_findings, row_consistency
from .observations import build_observations

HAND_EYE_METHODS = (("Park", "CALIB_HAND_EYE_PARK"), ("Daniilidis", "CALIB_HAND_EYE_DANIILIDIS"))
REFINED = "Park+pixel_refinement"
CV_FOLDS = 4
MIN_VIEWS = 12


@dataclass
class Options:
    min_tags: int = 4
    max_pnp_rmse: float = 2.0
    refine: bool = True
    # Quality gate: project defaults to tune on site, not vendor accuracy specs.
    max_test_px: float = 2.0
    max_view_px: float = 4.0
    max_board_mm: float = 3.0
    max_board_deg: float = 0.5
    expected_translation_mm: tuple | None = None  # from the mount CAD
    expected_tolerance_mm: float = 10.0
    max_candidate_spread_mm: float = 2.0
    max_candidate_spread_deg: float = 0.2
    allow_tcp_offset: bool = False  # poses deliberately of a TCP rather than the flange
    # Diagnostics (warnings only)
    bootstrap: int = 30
    seed: int = 0
    max_scale_error: float = 0.003
    max_focal_error: float = 0.003
    max_principal_point_px: float = 3.0
    max_distortion_px: float = 1.0


def check_motion_diversity(observations: list[dict]):
    first = observations[0]["base_T_flange"]
    vectors = []
    for obs in observations[1:]:
        rvec, _ = cv2.Rodrigues((inverse(first) @ obs["base_T_flange"])[:3, :3])
        if np.linalg.norm(rvec) > np.deg2rad(8):
            vectors.append(rvec.reshape(3) / np.linalg.norm(rvec))
    if len(vectors) < 2 or max(float(np.linalg.norm(np.cross(a, b)))
                               for a in vectors for b in vectors) < math.sin(math.radians(20)):
        raise ValueError("Robot rotations lack two distinct axes; collect more varied poses")


def handeye(observations: list[dict], method: int) -> np.ndarray:
    rot, trans = cv2.calibrateHandEye(
        [o["base_T_flange"][:3, :3] for o in observations],
        [o["base_T_flange"][:3, 3].reshape(3, 1) for o in observations],
        [o["camera_T_board"][:3, :3] for o in observations],
        [o["camera_T_board"][:3, 3].reshape(3, 1) for o in observations],
        method=method,
    )
    result = transform(rot, trans)
    if not np.all(np.isfinite(result)) or abs(np.linalg.det(result[:3, :3]) - 1) > 0.01:
        raise ValueError("Hand-eye solver returned an invalid transform")
    return result


def board_poses(observations: list[dict], flange_t_camera: np.ndarray) -> list[np.ndarray]:
    return [o["base_T_flange"] @ flange_t_camera @ o["camera_T_board"] for o in observations]


def score(observations: list[dict], flange_t_camera: np.ndarray, base_t_board: np.ndarray,
          k: np.ndarray, distortion: np.ndarray) -> dict:
    corner_errors, position_errors, angle_errors = [], [], []
    for obs in observations:
        delta = inverse(base_t_board) @ obs["base_T_flange"] @ flange_t_camera @ obs["camera_T_board"]
        position_errors.append(float(np.linalg.norm(delta[:3, 3]) * 1000.0))
        angle_errors.append(rotation_angle_degrees(delta[:3, :3]))
        camera_t_board = inverse(obs["base_T_flange"] @ flange_t_camera) @ base_t_board
        if np.any((camera_t_board[:3, :3] @ obs["object_points"].T + camera_t_board[:3, 3:4])[2] <= 0):
            return {"pixel_rmse": math.inf, "board_position_rmse_mm": math.inf, "board_rotation_rmse_deg": math.inf}
        rvec, _ = cv2.Rodrigues(camera_t_board[:3, :3])
        projected, _ = cv2.projectPoints(obs["object_points"], rvec, camera_t_board[:3, 3], k, distortion)
        corner_errors.extend(np.sum((projected.reshape(-1, 2) - obs["image_points"]) ** 2, axis=1))
    return {"pixel_rmse": float(np.sqrt(np.mean(corner_errors))),
            "board_position_rmse_mm": float(np.sqrt(np.mean(np.square(position_errors)))),
            "board_rotation_rmse_deg": float(np.sqrt(np.mean(np.square(angle_errors))))}


def refine(train: list[dict], initial_camera: np.ndarray, initial_board: np.ndarray,
           k: np.ndarray, distortion: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    from scipy.optimize import least_squares

    def residual(parameters):
        flange_t_camera, base_t_board = parameters_to_matrix(parameters[:6]), parameters_to_matrix(parameters[6:])
        values = []
        for obs in train:
            camera_t_board = inverse(obs["base_T_flange"] @ flange_t_camera) @ base_t_board
            if np.any((camera_t_board[:3, :3] @ obs["object_points"].T + camera_t_board[:3, 3:4])[2] <= 0):
                values.append(np.full(obs["image_points"].size, 10000.0))
                continue
            rvec, _ = cv2.Rodrigues(camera_t_board[:3, :3])
            projected, _ = cv2.projectPoints(obs["object_points"], rvec, camera_t_board[:3, 3], k, distortion)
            values.append((projected.reshape(-1, 2) - obs["image_points"]).ravel())
        return np.concatenate(values)

    result = least_squares(residual, np.r_[matrix_to_parameters(initial_camera), matrix_to_parameters(initial_board)],
                           loss="huber", f_scale=2.0, x_scale="jac", max_nfev=200)
    if not result.success and result.status <= 0:
        raise RuntimeError(f"Pixel refinement did not converge: {result.message}")
    return parameters_to_matrix(result.x[:6]), parameters_to_matrix(result.x[6:])


def fit_method(train: list[dict], name: str, k: np.ndarray, distortion: np.ndarray) -> dict:
    """One candidate fitted on `train`; raises cv2.error/ValueError/RuntimeError on failure."""
    if name == REFINED:
        start = fit_method(train, "Park", k, distortion)
        flange_t_camera, base_t_board = refine(train, start["flange_T_left_camera"], start["base_T_board"],
                                               k, distortion)
    else:
        flange_t_camera = handeye(train, getattr(cv2, dict(HAND_EYE_METHODS)[name]))
        base_t_board = transform_mean(board_poses(train, flange_t_camera))
    return {"method": name, "flange_T_left_camera": flange_t_camera, "base_T_board": base_t_board}


def method_names(refine_enabled: bool) -> list[str]:
    return [name for name, _ in HAND_EYE_METHODS] + ([REFINED] if refine_enabled else [])


def fit_candidates(train, k, distortion, refine_enabled, notes=None) -> list[dict]:
    candidates = []
    for name in method_names(refine_enabled):
        try:
            candidates.append(fit_method(train, name, k, distortion))
        except (cv2.error, ValueError, RuntimeError) as exc:
            if notes is not None:
                notes.append(f"{name} failed: {exc}")
    return candidates


def cross_validated_rmse(train, k, distortion, refine_enabled) -> dict[str, float]:
    """Corner RMSE of each method on training views left out of its fit (k-fold).

    Used to choose the method, so test views stay untouched. A method that
    fails on any fold scores inf.
    """
    squared, corners, folds = {}, {}, {}
    for fold in range(CV_FOLDS):
        fit_set = [o for i, o in enumerate(train) if i % CV_FOLDS != fold]
        check = [o for i, o in enumerate(train) if i % CV_FOLDS == fold]
        count = sum(len(o["image_points"]) for o in check)
        for item in fit_candidates(fit_set, k, distortion, refine_enabled):
            rmse = score(check, item["flange_T_left_camera"], item["base_T_board"], k, distortion)["pixel_rmse"]
            name = item["method"]
            squared[name] = squared.get(name, 0.0) + rmse ** 2 * count
            corners[name] = corners.get(name, 0) + count
            folds[name] = folds.get(name, 0) + 1
    return {name: (math.sqrt(squared[name] / corners[name]) if folds[name] == CV_FOLDS else math.inf)
            for name in squared}


def view_errors(observations, flange_t_camera, base_t_board, k, distortion) -> list[dict]:
    rows = []
    for obs in observations:
        values = score([obs], flange_t_camera, base_t_board, k, distortion)
        rows.append({"row": obs["row"], "image": obs["image"], "pixel_rmse": values["pixel_rmse"],
                     "board_position_mm": values["board_position_rmse_mm"],
                     "board_rotation_deg": values["board_rotation_rmse_deg"]})
    return rows


def quality_gate(best: dict, candidates: list[dict], test_views: list[dict],
                 options: Options) -> tuple[list[str], list[str]]:
    """(reasons to reject, warnings)."""
    reasons, warnings = [], []
    test = best["test"]
    if test["pixel_rmse"] > options.max_test_px:
        reasons.append(f"test corner RMSE {test['pixel_rmse']:.2f} px > {options.max_test_px} px")
    worst = max(test_views, key=lambda row: row["pixel_rmse"])
    if worst["pixel_rmse"] > options.max_view_px:
        reasons.append(f"test view row {worst['row']} ({worst['image']}) RMSE {worst['pixel_rmse']:.2f} px "
                       f"> {options.max_view_px} px")
    if test["board_position_rmse_mm"] > options.max_board_mm:
        reasons.append(f"fixed-board position spread {test['board_position_rmse_mm']:.2f} mm > "
                       f"{options.max_board_mm} mm on test views")
    if test["board_rotation_rmse_deg"] > options.max_board_deg:
        reasons.append(f"fixed-board rotation spread {test['board_rotation_rmse_deg']:.3f} deg > "
                       f"{options.max_board_deg} deg on test views")
    if options.expected_translation_mm is not None:
        offset = float(np.linalg.norm(best["flange_T_left_camera"][:3, 3] * 1000.0
                                      - np.asarray(options.expected_translation_mm)))
        if offset > options.expected_tolerance_mm:
            reasons.append(f"translation is {offset:.1f} mm from the mount estimate (> {options.expected_tolerance_mm} mm)")
    else:
        warnings.append("no expected translation given: not checked against the mount CAD")
    for other in candidates:
        if other is best:
            continue
        delta = inverse(best["flange_T_left_camera"]) @ other["flange_T_left_camera"]
        spread_mm, spread_deg = float(np.linalg.norm(delta[:3, 3]) * 1000.0), rotation_angle_degrees(delta[:3, :3])
        if spread_mm > options.max_candidate_spread_mm or spread_deg > options.max_candidate_spread_deg:
            warnings.append(f"{other['method']} differs from {best['method']} by {spread_mm:.2f} mm / "
                            f"{spread_deg:.3f} deg: poorly conditioned poses or biased pose data")
    return reasons, warnings


def _finite(item: dict) -> bool:
    return (math.isfinite(item["cv_pixel_rmse"])
            and all(math.isfinite(v) for phase in ("train", "test") for v in item[phase].values()))


def _serialise(item: dict) -> dict:
    return {"method": item["method"],
            "flange_T_left_camera": item["flange_T_left_camera"].tolist(),
            "left_camera_T_flange": inverse(item["flange_T_left_camera"]).tolist(),
            "base_T_board": item["base_T_board"].tolist(),
            "cv_pixel_rmse": item["cv_pixel_rmse"], "train": item["train"], "test": item["test"]}


def calibrate(session: Session, options: Options | None = None) -> dict:
    """Full pipeline; returns the result dict with status accepted / rejected / insufficient_data."""
    options = options or Options()
    k, distortion = session.k, session.distortion
    observations, rejected = build_observations(session, options.min_tags, options.max_pnp_rmse)
    test = [o for i, o in enumerate(observations) if i % 5 == 0]
    train = [o for i, o in enumerate(observations) if i % 5 != 0]
    warnings = []
    result = {
        "status": None, "reasons": [], "warnings": warnings,
        "note": "accepted = these internal consistency checks passed; it is not a measured task accuracy. "
                "Board size, intrinsics and image flip cannot be fully checked from the data; see diagnostics "
                "and run 'validate' against independently measured points.",
        "options": asdict(options), "session": str(session.path), "camera": session.camera,
        "board": session.board, "views": len(session.samples),
        "accepted_views": [{"row": o["row"], "image": o["image"], "tag_ids": o["tag_ids"],
                            "dropped_tag_ids": o["dropped_tag_ids"], "pnp_rmse_px": o["pnp_rmse_px"]}
                           for o in observations],
        "rejected_views": rejected,
        "train_rows": [o["row"] for o in train], "test_rows": [o["row"] for o in test],
    }
    consistency = None
    joints = [s.joints_deg for s in session.samples]
    if all(q is not None for q in joints) and len(joints) >= 3:
        consistency = row_consistency([s.row for s in session.samples], joints,
                                      [s.pose_mm_deg for s in session.samples])
        result["pose_joint_consistency"] = consistency
        for row in consistency["suspect_rows"]:
            warnings.append(f"poses.csv row {row['row']}: pose disagrees with its joints by {row['error_mm']} mm / "
                            f"{row['error_deg']} deg (typo, wrong tool/user frame, or read at another moment?)")
    else:
        warnings.append("poses.csv has no joint angles: typos and the tool/user frame of the poses cannot be "
                        "checked; add j1_deg..j6_deg or at least give --expected-translation-mm")
    if observations:
        result["data_coverage"] = diagnostics.coverage(observations, session)
        warnings.extend(result["data_coverage"].pop("warnings"))
        # Needs no robot data, so it runs even when the hand-eye data are insufficient.
        result["intrinsics_check"] = diagnostics.intrinsics_from_images(observations, session, options)
        warnings.extend(result["intrinsics_check"]["warnings"])
    if len(observations) < MIN_VIEWS:
        return _status(result, "insufficient_data",
                       [f"only {len(observations)} usable views; collect at least {MIN_VIEWS}, ideally 20-30"])
    try:
        check_motion_diversity(train)
    except ValueError as exc:
        return _status(result, "insufficient_data", [str(exc)])

    frame_reasons = []  # the constant tool/base offsets are only identifiable with varied rotations
    if consistency is not None:
        tool_frame, user_frame = frame_findings(consistency)
        if tool_frame:
            # The result would be camera relative to that TCP, off by the tool offset (120 mm in simulation).
            (warnings if options.allow_tcp_offset else frame_reasons).append(
                tool_frame + ("" if options.allow_tcp_offset else "; or pass --allow-tcp-offset if intended"))
        if user_frame:
            warnings.append(user_frame)

    notes = []
    candidates = fit_candidates(train, k, distortion, options.refine, notes)
    cv_rmse = cross_validated_rmse(train, k, distortion, options.refine)
    for item in candidates:
        item["train"] = score(train, item["flange_T_left_camera"], item["base_T_board"], k, distortion)
        item["cv_pixel_rmse"] = cv_rmse.get(item["method"], math.inf)
        item["test"] = score(test, item["flange_T_left_camera"], item["base_T_board"], k, distortion)
    warnings.extend(notes)
    result["candidates"] = [_serialise(item) for item in candidates]
    usable = [item for item in candidates if _finite(item)]
    if not usable:
        return _status(result, "rejected", frame_reasons + ["no hand-eye method produced finite results"])
    best = min(usable, key=lambda item: item["cv_pixel_rmse"])
    test_views = view_errors(test, best["flange_T_left_camera"], best["base_T_board"], k, distortion)
    reasons, gate_warnings = quality_gate(best, usable, test_views, options)
    reasons = frame_reasons + reasons
    warnings.extend(gate_warnings)
    result.update({
        "chosen_method": best["method"], "chosen": _serialise(best), "test_views": test_views,
        "selection": {"rule": f"lowest {CV_FOLDS}-fold cross-validated corner RMSE on training views; "
                              "test views are never used to choose",
                      "cv_pixel_rmse": {item["method"]: item["cv_pixel_rmse"] for item in candidates}},
    })
    result["uncertainty"] = diagnostics.bootstrap(train, best, k, distortion, options)
    result["board_scale_check"] = diagnostics.board_scale(observations, best, session, options)
    warnings.extend(result["board_scale_check"]["warnings"])
    return _status(result, "rejected" if reasons else "accepted", reasons)


def _status(result: dict, status: str, reasons: list[str]) -> dict:
    result["status"], result["reasons"] = status, reasons
    return result
