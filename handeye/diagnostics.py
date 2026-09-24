"""Diagnostics beyond the pass/fail gate: coverage, uncertainty, board scale and intrinsics."""

from __future__ import annotations

import math

import cv2
import numpy as np

from .board import board_outline
from .geometry import inverse, matrix_to_parameters, parameters_to_matrix, rotation_angle_degrees, rotation_vector


def coverage(observations: list[dict], session) -> dict:
    """How well the views cover the image, distances, tilts and robot rotations."""
    scale = 8
    union = np.zeros((session.height // scale + 1, session.width // scale + 1), np.uint8)
    outline = board_outline(session.board)
    centre = outline.mean(axis=0)
    areas, distances, tilts = [], [], []
    for obs in observations:
        ctb = obs["camera_T_board"]
        rvec, _ = cv2.Rodrigues(ctb[:3, :3])
        points, _ = cv2.projectPoints(outline, rvec, ctb[:3, 3], session.k, session.distortion)
        view = np.zeros_like(union)
        cv2.fillConvexPoly(view, np.round(points.reshape(-1, 2) / scale).astype(np.int32), 1)
        areas.append(float(view.mean()))
        union |= view
        distances.append(float(np.linalg.norm(ctb[:3, :3] @ centre + ctb[:3, 3])))
        tilts.append(float(np.degrees(np.arccos(min(1.0, abs(ctb[2, 2]))))))
    flanges = [o["base_T_flange"] for o in observations]
    rotation = max((rotation_angle_degrees(a[:3, :3].T @ b[:3, :3]) for a in flanges for b in flanges), default=0.0)
    spread = max((float(np.linalg.norm(a[:3, 3] - b[:3, 3]) * 1000) for a in flanges for b in flanges), default=0.0)
    result = {"image_area_covered": float(union.mean()), "board_area_per_view": [min(areas), max(areas)],
              "distance_m": [min(distances), max(distances)], "tilt_deg": [min(tilts), max(tilts)],
              "max_relative_rotation_deg": rotation, "flange_position_spread_mm": spread, "warnings": []}
    if result["image_area_covered"] < 0.5:
        result["warnings"].append(f"the board covered only {result['image_area_covered']:.0%} of the image over all "
                                  "views; move it towards the image edges and corners too")
    if max(tilts) < 20.0:
        result["warnings"].append(f"board tilt only up to {max(tilts):.0f} deg; add views at 20-40 deg")
    if rotation < 30.0:
        result["warnings"].append(f"largest robot rotation between views is {rotation:.0f} deg; "
                                  "20-40 deg about several axes improves the translation estimate")
    return result


def bootstrap(train: list[dict], best: dict, k: np.ndarray, distortion: np.ndarray, options) -> dict:
    """Spread of the chosen method over random view subsets (delete-d jackknife).

    Resampling views captures per-view random errors, including robot pose
    noise that the Jacobian-based covariance ignores (it understated the error
    about tenfold in simulation). It does not capture errors shared by all
    views: board size, intrinsics, systematic kinematic error.
    """
    from .solver import REFINED, fit_method, refine

    n = len(train)
    if options.bootstrap <= 0 or n < 8:
        return {"method": "disabled"}
    d = max(1, round(0.2 * n))
    rng = np.random.default_rng(options.seed)
    reference = best["flange_T_left_camera"]
    translations, rotations = [], []
    for _ in range(options.bootstrap):
        subset = [train[i] for i in sorted(rng.choice(n, n - d, replace=False))]
        try:
            if best["method"] == REFINED:  # warm start: converges in a few iterations
                x, _ = refine(subset, reference, best["base_T_board"], k, distortion)
            else:
                x = fit_method(subset, best["method"], k, distortion)["flange_T_left_camera"]
        except (cv2.error, ValueError, RuntimeError):
            continue
        translations.append(x[:3, 3] * 1000.0)
        rotations.append(rotation_vector(reference[:3, :3].T @ x[:3, :3]))
    if len(translations) < 5:
        return {"method": "failed: too few successful subsets"}
    inflation = (n - d) / d  # delete-d jackknife variance factor
    t_var = np.var(translations, axis=0, ddof=1) * inflation
    r_var = np.var(rotations, axis=0, ddof=1) * inflation
    return {"method": f"delete-{d} jackknife over {n} training views, {len(translations)} subsets",
            "translation_sd_mm": np.sqrt(t_var).round(4).tolist(),
            "translation_sd_norm_mm": float(np.sqrt(t_var.sum())),
            "rotation_sd_deg": float(np.degrees(np.sqrt(r_var.sum()))),
            "covers": "random per-view errors only; not board size, intrinsics or systematic robot error"}


def _corner_radius(k: np.ndarray, width: int, height: int) -> float:
    """Normalised image radius of the farthest image corner."""
    return max(math.hypot((u - k[0, 2]) / k[0, 0], (v - k[1, 2]) / k[1, 1])
               for u in (0, width - 1) for v in (0, height - 1))


def intrinsics_from_images(observations: list[dict], session, options) -> dict:
    """Check camera.yaml against the images alone: fx, fy, cx, cy and radial k1.

    A planar-target camera calibration (OpenCV calibrateCameraExtended) with
    every view's board pose free, started from camera.yaml. Robot poses are not
    used, so robot error cannot leak in; a joint robot+intrinsics fit mistook
    0.5 mm robot noise for a 2.7 px distortion in simulation. A difference is
    flagged when it exceeds both the option threshold and 3 standard
    deviations. It cannot see board size (scale-free) and assumes a flat board.
    """
    if len(observations) < 5:
        return {"status": "skipped: fewer than 5 views", "warnings": []}
    k0 = session.k
    objects = [o["object_points"].astype(np.float32) for o in observations]
    images = [o["image_points"].astype(np.float32) for o in observations]
    distortion0 = np.zeros(5)
    distortion0[:min(5, len(session.distortion))] = session.distortion[:5]
    flags = (cv2.CALIB_USE_INTRINSIC_GUESS | cv2.CALIB_ZERO_TANGENT_DIST | cv2.CALIB_FIX_K2 | cv2.CALIB_FIX_K3)
    try:
        rms, k, distortion, _, _, std, _, _ = cv2.calibrateCameraExtended(
            objects, images, (session.width, session.height), k0.copy(), distortion0.copy(), flags=flags)
    except cv2.error as exc:
        return {"status": f"failed: {exc}", "warnings": []}
    sd = std.ravel()  # fx, fy, cx, cy, k1, ...
    r = _corner_radius(k0, session.width, session.height)
    k1_change = float(distortion.ravel()[0] - distortion0[0])
    result = {"status": "ok", "pixel_rmse": float(rms),
              "fx": float(k[0, 0]), "fy": float(k[1, 1]), "cx": float(k[0, 2]), "cy": float(k[1, 2]),
              "focal_change": [float(k[0, 0] / k0[0, 0] - 1), float(k[1, 1] / k0[1, 1] - 1)],
              "focal_change_sd": [float(sd[0] / k0[0, 0]), float(sd[1] / k0[1, 1])],
              "principal_point_shift_px": [float(k[0, 2] - k0[0, 2]), float(k[1, 2] - k0[1, 2])],
              "principal_point_sd_px": [float(sd[2]), float(sd[3])],
              "radial_k1_change": k1_change, "radial_k1_sd": float(sd[4]),
              "distortion_corner_shift_px": abs(k1_change) * r ** 3 * k0[0, 0],
              "distortion_corner_shift_sd_px": float(sd[4]) * r ** 3 * k0[0, 0], "warnings": []}
    focal = max(zip(map(abs, result["focal_change"]), result["focal_change_sd"]))
    if focal[0] > max(options.max_focal_error, 3 * focal[1]):
        result["warnings"].append(f"focal length from the images differs from camera.yaml by "
                                  f"{result['focal_change'][0] * 100:+.2f}% / {result['focal_change'][1] * 100:+.2f}%: "
                                  "verify the intrinsics")
    shift, shift_sd = result["principal_point_shift_px"], result["principal_point_sd_px"]
    if any(abs(s) > max(options.max_principal_point_px, 3 * d) for s, d in zip(shift, shift_sd)):
        result["warnings"].append(f"principal point from the images differs from camera.yaml by "
                                  f"({shift[0]:+.1f}, {shift[1]:+.1f}) px: verify the intrinsics")
    corner, corner_sd = result["distortion_corner_shift_px"], result["distortion_corner_shift_sd_px"]
    if corner > max(options.max_distortion_px, 3 * corner_sd):
        result["warnings"].append(f"residual radial distortion in the images shifts the image corner by {corner:.1f} px "
                                  f"(k1 {k1_change:+.5f}): are they really the rectified left view matching camera.yaml?")
    return result


def board_scale(observations: list[dict], best: dict, session, options) -> dict:
    """Refit hand-eye with a free board scale (K fixed to camera.yaml).

    Robot translations are metric, so a wrong board size conflicts with them;
    images alone cannot see it. In simulation a 1% error was recovered exactly,
    while 0.2-0.5 mm robot noise moved the estimate by <= 0.1%. A wrong K
    leaks in too (0.5% focal error -> -0.2%), but the image check flags that.
    """
    from scipy.optimize import least_squares

    k, distortion = session.k, session.distortion

    def residual(p):
        x, y, s = parameters_to_matrix(p[:6]), parameters_to_matrix(p[6:12]), math.exp(p[12])
        values = []
        for obs in observations:
            ctb = inverse(obs["base_T_flange"] @ x) @ y
            rvec, _ = cv2.Rodrigues(ctb[:3, :3])
            projected, _ = cv2.projectPoints(obs["object_points"] * s, rvec, ctb[:3, 3], k, distortion)
            values.append((projected.reshape(-1, 2) - obs["image_points"]).ravel())
        return np.concatenate(values)

    start = np.r_[matrix_to_parameters(best["flange_T_left_camera"]), matrix_to_parameters(best["base_T_board"]), 0.0]
    fit = least_squares(residual, start, loss="huber", f_scale=2.0, x_scale="jac", max_nfev=300)
    scale = math.exp(fit.x[12])
    delta = inverse(best["flange_T_left_camera"]) @ parameters_to_matrix(fit.x[:6])
    result = {"board_scale": scale,
              "hand_eye_change": {"translation_mm": float(np.linalg.norm(delta[:3, 3]) * 1000),
                                  "rotation_deg": rotation_angle_degrees(delta[:3, :3])},
              "pixel_rmse": float(np.sqrt(np.mean(fit.fun ** 2))), "warnings": []}
    if abs(scale - 1.0) > options.max_scale_error:
        result["warnings"].append(f"board scale fits {scale:.4f}x target.yaml ({(scale - 1) * 100:+.2f}%): "
                                  "measure the printed board (a 1% error biased the translation ~4.5 mm in "
                                  "simulation); large robot pose errors can also cause this")
    return result
