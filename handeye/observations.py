"""Per-image board pose (camera_T_board) by PnP, with whole-tag outlier removal."""

from __future__ import annotations

import math

import cv2
import numpy as np

from .board import detect_board, detector
from .dataset import Session
from .geometry import read_image, transform


def fit_pose(obj: np.ndarray, img: np.ndarray, k: np.ndarray, distortion: np.ndarray,
             rvec: np.ndarray, tvec: np.ndarray, min_tags: int):
    """LM pose refinement that drops whole tags with a corner far off the fit.

    Such a corner locked onto the wrong feature; a few of them can hide under
    the view RMSE limit while still biasing the pose. Points come 4 per tag.
    """
    if len(obj) != len(img) or len(obj) % 4 or len(obj) < 4 * min_tags:
        raise ValueError("insufficient complete tags for PnP refinement")
    subset = np.arange(len(obj))
    for _ in range(len(obj) // 4 - min_tags + 1):
        rvec, tvec = cv2.solvePnPRefineLM(obj[subset], img[subset], k, distortion, rvec, tvec)
        projected, _ = cv2.projectPoints(obj[subset], rvec, tvec, k, distortion)
        errors = np.linalg.norm(projected.reshape(-1, 2) - img[subset], axis=1)
        if not np.isfinite(errors).all():
            raise ValueError("non-finite PnP residuals")
        tags = subset // 4
        bad = np.unique(tags[errors > max(1.0, 5.0 * float(np.median(errors)))])
        if not len(bad):
            return rvec, tvec, subset, float(np.sqrt(np.mean(errors ** 2)))
        if len(np.unique(tags)) - len(bad) < min_tags:
            raise ValueError("too few good complete tags after PnP outlier removal")
        subset = subset[~np.isin(tags, bad)]
    raise ValueError("PnP outlier removal did not converge")


def _rmse(obj, img, rvec, tvec, k, distortion) -> float:
    projected, _ = cv2.projectPoints(obj, rvec, tvec, k, distortion)
    return float(np.sqrt(np.mean(np.sum((projected.reshape(-1, 2) - img) ** 2, axis=1))))


def build_observations(session: Session, min_tags: int = 4,
                       max_pnp_rmse: float = 2.0) -> tuple[list[dict], list[dict]]:
    """(accepted observations, rejected {row, image, reason})."""
    k, distortion, board = session.k, session.distortion, session.board
    aruco_detector = detector()
    accepted, rejected = [], []
    for sample in session.samples:
        def reject(reason):
            rejected.append({"row": sample.row, "image": sample.image, "reason": reason})

        image = read_image(session.path / sample.image)
        if image is None:
            reject("image missing or unreadable")
            continue
        if (image.shape[1], image.shape[0]) != (session.width, session.height):
            reject(f"image is {image.shape[1]}x{image.shape[0]}, camera.yaml says {session.width}x{session.height}")
            continue
        obj, img, ids = detect_board(image, board, aruco_detector)
        if len(ids) < min_tags:
            reject(f"only {len(ids)} tags detected")
            continue
        subset, rmse = np.arange(len(obj)), math.inf
        try:
            # A planar Aprilgrid gives ITERATIVE PnP a homography initialisation.
            ok, rvec, tvec = cv2.solvePnP(obj, img, k, distortion, flags=cv2.SOLVEPNP_ITERATIVE)
            if ok:
                rvec, tvec, subset, rmse = fit_pose(obj, img, k, distortion, rvec, tvec, min_tags)
        except (cv2.error, ValueError):
            pass
        if rmse > max_pnp_rmse:  # RANSAC only as a fallback
            try:
                ok, rvec, tvec, inliers = cv2.solvePnPRansac(
                    obj, img, k, distortion, iterationsCount=300, reprojectionError=3.0,
                    confidence=0.999, flags=cv2.SOLVEPNP_EPNP)
                if not ok or inliers is None or len(inliers) < max(12, math.ceil(0.6 * len(obj))):
                    raise ValueError("insufficient PnP inliers")
                # RANSAC can keep only three corners of a bad tag. Refit only
                # complete inlier tags, then apply the same whole-tag checks.
                inlier_indices = np.unique(inliers.reshape(-1))
                whole_tags = [tag for tag in np.unique(inlier_indices // 4)
                              if np.count_nonzero(inlier_indices // 4 == tag) == 4]
                subset = np.asarray([i for tag in whole_tags for i in range(4 * tag, 4 * tag + 4)], dtype=int)
                rvec, tvec, kept, rmse = fit_pose(
                    obj[subset], img[subset], k, distortion, rvec, tvec, min_tags)
                subset = subset[kept]
            except (cv2.error, ValueError):
                reject("PnP failed")
                continue
        if not math.isfinite(rmse) or rmse > max_pnp_rmse:
            reject(f"PnP RMSE {rmse:.2f} px")
            continue
        # Outlier removal must still leave a spatially spread set of whole tags,
        # not a corner count that one strip of tags can meet.
        kept_ids = sorted({ids[i // 4] for i in subset.tolist() if np.sum(subset // 4 == i // 4) == 4})
        rows = {tag // board["tagCols"] for tag in kept_ids}
        cols = {tag % board["tagCols"] for tag in kept_ids}
        if len(kept_ids) < min_tags or len(rows) < 2 or len(cols) < 2:
            reject(f"{len(kept_ids)} complete tags in {len(rows)} rows x {len(cols)} columns")
            continue
        complete = np.asarray([i for i in subset.tolist() if ids[i // 4] in kept_ids])
        if len(complete) != len(subset):
            subset = complete
            rvec, tvec = cv2.solvePnPRefineLM(obj[subset], img[subset], k, distortion, rvec, tvec)
            rmse = _rmse(obj[subset], img[subset], rvec, tvec, k, distortion)
            if rmse > max_pnp_rmse:
                reject(f"PnP RMSE {rmse:.2f} px")
                continue
        rot, _ = cv2.Rodrigues(rvec)
        accepted.append({
            "row": sample.row, "image": sample.image, "base_T_flange": sample.base_t_flange,
            "joints_deg": sample.joints_deg, "camera_T_board": transform(rot, tvec),
            "object_points": obj[subset], "image_points": img[subset], "tag_ids": kept_ids,
            "dropped_tag_ids": sorted(set(ids) - set(kept_ids)), "pnp_rmse_px": rmse,
        })
    return accepted, rejected
