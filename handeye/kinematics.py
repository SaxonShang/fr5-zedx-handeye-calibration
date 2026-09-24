"""FR5 nominal kinematics and checks of recorded poses against recorded joints.

DH values are the FAIRINO manual's FR5 table; joint limits are the official
fairino5_v6.urdf (wrist centre agrees with the DH model to 0.1 mm; the URDF
a3 is 395.01 mm). Nominal only: a real controller may use per-robot
kinematic calibration, so its poses can differ from this model by a few mm.
"""

from __future__ import annotations

import math

import cv2
import numpy as np

from .geometry import inverse, matrix_to_parameters, parameters_to_matrix, pose_from_fr5, \
    rotation_angle_degrees, rotation_vector, transform

# (a [mm], alpha [rad], d [mm]); theta offsets are 0.
FR5_DH = ((0.0, math.pi / 2, 152.0), (-425.0, 0.0, 0.0), (-395.0, 0.0, 0.0),
          (0.0, math.pi / 2, 102.0), (0.0, -math.pi / 2, 102.0), (0.0, 0.0, 100.0))
JOINT_LOWER_DEG = np.array([-175.0, -265.0, -162.0, -265.0, -175.0, -175.0])
JOINT_UPPER_DEG = np.array([175.0, 85.0, 162.0, 85.0, 175.0, 175.0])
SINGULAR_MARGIN_DEG = 10.0


def fk(joints_deg) -> np.ndarray:
    """base_T_flange [m] from the standard DH parameters."""
    result = np.eye(4)
    for theta, (a, alpha, d) in zip(np.deg2rad(np.asarray(joints_deg, dtype=float)), FR5_DH):
        ct, st, ca, sa = math.cos(theta), math.sin(theta), math.cos(alpha), math.sin(alpha)
        result = result @ np.array([[ct, -st * ca, st * sa, a / 1000 * ct],
                                    [st, ct * ca, -ct * sa, a / 1000 * st],
                                    [0.0, sa, ca, d / 1000], [0.0, 0.0, 0.0, 1.0]])
    return result


def near_singular(joints_deg) -> bool:
    """UR-type singularities: wrist (q5 = 0/180), elbow (q3 = 0/180), shoulder."""
    q = np.deg2rad(np.asarray(joints_deg, dtype=float))
    margin = math.sin(math.radians(SINGULAR_MARGIN_DEG))
    if abs(math.sin(q[4])) < margin or abs(math.sin(q[2])) < margin:
        return True
    flange = fk(joints_deg)
    centre = flange[:3, 3] - flange[:3, 2] * FR5_DH[5][2] / 1000  # remove d6
    return bool(np.hypot(centre[0], centre[1]) < 0.10)


def ik(target: np.ndarray, rng: np.random.Generator, starts: int = 24) -> np.ndarray | None:
    """Joint angles [deg] within limits reaching base_T_flange, or None (numeric, multi-start)."""
    from scipy.optimize import least_squares

    def residual(q):
        delta = inverse(target) @ fk(q)
        return np.r_[delta[:3, 3] * 1000.0, rotation_vector(delta[:3, :3]) * 1000.0]

    for _ in range(starts):
        fit = least_squares(residual, rng.uniform(JOINT_LOWER_DEG, JOINT_UPPER_DEG),
                            bounds=(JOINT_LOWER_DEG, JOINT_UPPER_DEG), xtol=1e-12, ftol=1e-12, gtol=1e-12)
        if np.max(np.abs(fit.fun)) < 1e-4:  # 0.1 um / 0.1 urad
            return fit.x
    return None


# ---- recorded poses vs recorded joints -------------------------------------------------

HANDEYE_CONVENTION = "Rz@Ry@Rx"
MIN_POSES = 6
MIN_JOINT_SPREAD_DEG = 20.0
DISTINCT_POSE_DEG = 5.0
# A wrong convention is off by degrees to tens of degrees; nominal-vs-real
# kinematics by up to a few mm.
FIT_MM, FIT_DEG = 5.0, 0.5


def _axis(axis: int, angle: float) -> np.ndarray:
    rvec = np.zeros(3)
    rvec[axis] = angle
    return cv2.Rodrigues(rvec)[0]


CONVENTIONS = {
    HANDEYE_CONVENTION: None,  # geometry.pose_from_fr5 itself
    "Rx@Ry@Rz": lambda rx, ry, rz: _axis(0, rx) @ _axis(1, ry) @ _axis(2, rz),
    "rotation vector": lambda rx, ry, rz: cv2.Rodrigues(np.array([rx, ry, rz]))[0],
}


def controller_matrix(raw, convention: str = HANDEYE_CONVENTION) -> np.ndarray:
    if convention == HANDEYE_CONVENTION:
        return pose_from_fr5(raw)
    rx, ry, rz = np.deg2rad(np.asarray(raw[3:], dtype=float))
    return transform(CONVENTIONS[convention](rx, ry, rz), np.asarray(raw[:3], dtype=float) / 1000)


def _deltas(parameters, controller, model):
    x, y = parameters_to_matrix(parameters[:6]), parameters_to_matrix(parameters[6:])
    return [inverse(c) @ x @ m @ y for c, m in zip(controller, model)]


def _residual(parameters, controller, model):
    return np.concatenate([np.r_[d[:3, 3] * 1000.0, rotation_vector(d[:3, :3]) * 1000.0]
                           for d in _deltas(parameters, controller, model)])  # mm and mrad


def fit_offsets(controller: list[np.ndarray], model: list[np.ndarray], start=None,
                robust: bool = False) -> dict:
    """Constant base offset X and flange offset Y with controller_i ~= X @ model_i @ Y.

    Starts from identity (a matching model needs no offset) and falls back to
    32 axis-aligned starts. robust=True uses a soft-L1 loss so a few bad rows
    cannot drag the fit.
    """
    from scipy.optimize import least_squares

    if start is not None:
        starts = [start]
    else:
        starts = [np.r_[matrix_to_parameters(transform(_axis(2, kx * math.pi / 2), np.zeros(3))),
                        matrix_to_parameters(transform(_axis(2, ky * math.pi / 2) @ _axis(0, flip), np.zeros(3)))]
                  for kx in range(4) for ky in range(4) for flip in (0.0, math.pi)]
    best = None
    for s in starts:
        fit = least_squares(_residual, s, args=(controller, model),
                            loss="soft_l1" if robust else "linear", f_scale=1.0)
        if best is None or fit.cost < best.cost:
            best = fit
        errors = _errors(best.x, controller, model)
        if np.median(errors[0]) < 0.1 * FIT_MM and np.median(errors[1]) < 0.1 * FIT_DEG:
            break
    errors_mm, errors_deg = _errors(best.x, controller, model)
    return {"parameters": best.x, "errors_mm": errors_mm, "errors_deg": errors_deg,
            "rms_mm": float(np.sqrt(np.mean(np.square(errors_mm)))),
            "rms_deg": float(np.sqrt(np.mean(np.square(errors_deg)))),
            "base_offset": _describe(parameters_to_matrix(best.x[:6])),
            "flange_offset": _describe(parameters_to_matrix(best.x[6:]))}


def _errors(parameters, controller, model) -> tuple[list[float], list[float]]:
    deltas = _deltas(parameters, controller, model)
    return ([float(np.linalg.norm(d[:3, 3]) * 1000.0) for d in deltas],
            [rotation_angle_degrees(d[:3, :3]) for d in deltas])


def _describe(t: np.ndarray) -> dict:
    return {"translation_mm": (t[:3, 3] * 1000.0).round(3).tolist(),
            "rotation_deg": round(rotation_angle_degrees(t[:3, :3]), 4)}


def _fits(fit: dict) -> bool:
    return fit["rms_mm"] < FIT_MM and fit["rms_deg"] < FIT_DEG


def distinct_poses(joints: np.ndarray) -> int:
    kept = []
    for q in joints:
        if all(np.max(np.abs(q - other)) > DISTINCT_POSE_DEG for other in kept):
            kept.append(q)
    return len(kept)


def check_convention(joints_deg: list, poses_mm_deg: list) -> dict:
    """Does the recorded (rx, ry, rz) follow Rz@Ry@Rx? pass / fail / insufficient_data.

    Pass needs: handeye's convention fits FK(joints) up to constant offsets, no
    other convention does, and the offsets predict each left-out pose.
    """
    joints = np.asarray(joints_deg, dtype=float).reshape(-1, 6)
    spread = np.ptp(joints, axis=0) if len(joints) else np.zeros(6)
    result = {"poses": len(joints), "distinct_poses": distinct_poses(joints),
              "joint_spread_deg": spread.round(1).tolist(),
              "weak_joints": [i + 1 for i, s in enumerate(spread) if s < MIN_JOINT_SPREAD_DEG]}
    reasons = []
    if result["distinct_poses"] < MIN_POSES:
        reasons.append(f"only {result['distinct_poses']} distinct poses (need {MIN_POSES}, joints "
                       f"differing by > {DISTINCT_POSE_DEG:.0f} deg)")
    if result["weak_joints"]:
        reasons.append(f"joints {result['weak_joints']} moved < {MIN_JOINT_SPREAD_DEG:.0f} deg")
    if reasons:
        return {**result, "status": "insufficient_data", "reasons": reasons}
    model = [fk(q) for q in joints]
    fits = {name: fit_offsets([controller_matrix(p, name) for p in poses_mm_deg], model) for name in CONVENTIONS}
    own = fits[HANDEYE_CONVENTION]
    controller = [controller_matrix(p) for p in poses_mm_deg]
    worst_mm = worst_deg = 0.0
    for i in range(len(controller)):  # leave-one-out
        keep = [j for j in range(len(controller)) if j != i]
        fit = fit_offsets([controller[j] for j in keep], [model[j] for j in keep], own["parameters"])
        error_mm, error_deg = _errors(fit["parameters"], [controller[i]], [model[i]])
        worst_mm, worst_deg = max(worst_mm, error_mm[0]), max(worst_deg, error_deg[0])
    others = [name for name in fits if name != HANDEYE_CONVENTION and _fits(fits[name])]
    result.update(
        fits={name: {k: v for k, v in fit.items() if k in ("rms_mm", "rms_deg", "base_offset", "flange_offset")}
              for name, fit in fits.items()},
        best_convention=min(fits, key=lambda n: fits[n]["rms_mm"] + 10.0 * fits[n]["rms_deg"]),
        leave_one_out={"max_mm": worst_mm, "max_deg": worst_deg})
    if not _fits(own):
        return {**result, "status": "fail", "reasons": [
            f"Rz@Ry@Rx leaves {own['rms_mm']:.2f} mm / {own['rms_deg']:.3f} deg; check Euler order, "
            "units, joint zeros and that the pose is the flange in the base frame"]}
    if others:
        return {**result, "status": "insufficient_data", "reasons": [
            f"conventions {others} fit as well: poses cannot tell them apart; vary J4-J6 more"]}
    if worst_mm >= FIT_MM or worst_deg >= FIT_DEG:
        return {**result, "status": "fail", "reasons": [
            f"fitted offsets do not predict left-out poses ({worst_mm:.2f} mm / {worst_deg:.3f} deg)"]}
    return {**result, "status": "pass", "reasons": []}


# Constant offsets between FK(joints) and the recorded poses. The flange frame
# of the DH model is the controller's flange, so a large flange offset means
# the poses were read with a tool (TCP) active; a large base offset means a
# user/work frame was active. Kinematic calibration differences stay well below.
FRAME_OFFSET_MM, FRAME_OFFSET_DEG = 5.0, 0.5


def _offset_is_large(offset: dict) -> bool:
    return (float(np.linalg.norm(offset["translation_mm"])) > FRAME_OFFSET_MM
            or offset["rotation_deg"] > FRAME_OFFSET_DEG)


def frame_findings(consistency: dict) -> tuple[str | None, str | None]:
    """(tool-frame message, user-frame message) from row_consistency's fitted offsets."""
    tool = base = None
    flange, origin = consistency["flange_offset"], consistency["base_offset"]
    if _offset_is_large(flange):
        tool = (f"poses look like an active TCP, not the flange: FK(joints) needs a "
                f"{np.linalg.norm(flange['translation_mm']):.1f} mm / {flange['rotation_deg']:.2f} deg tool offset "
                f"{flange['translation_mm']} to match; set the WebApp tool coordinate system to 0")
    if _offset_is_large(origin):
        base = (f"poses look like they are in a user/work frame "
                f"({np.linalg.norm(origin['translation_mm']):.1f} mm / {origin['rotation_deg']:.2f} deg from the "
                "robot base): the hand-eye result is unaffected, but base_T_board and touch points are in that frame")
    return tool, base


def row_consistency(rows: list[int], joints_deg: list, poses_mm_deg: list) -> dict:
    """Per-row mismatch between recorded pose and FK(recorded joints) after a robust offset fit.

    Rows far above the others are likely transcription errors, a pose read in
    another tool/user frame, or joints and pose taken at different moments.
    """
    model = [fk(q) for q in joints_deg]
    controller = [controller_matrix(p) for p in poses_mm_deg]
    fit = fit_offsets(controller, model, robust=True)
    errors_mm, errors_deg = np.asarray(fit["errors_mm"]), np.asarray(fit["errors_deg"])
    limit_mm = max(1.0, 5.0 * float(np.median(errors_mm)))
    limit_deg = max(0.1, 5.0 * float(np.median(errors_deg)))
    suspect = [{"row": row, "error_mm": round(float(e_mm), 3), "error_deg": round(float(e_deg), 4)}
               for row, e_mm, e_deg in zip(rows, errors_mm, errors_deg) if e_mm > limit_mm or e_deg > limit_deg]
    return {"median_mm": float(np.median(errors_mm)), "median_deg": float(np.median(errors_deg)),
            "limit_mm": limit_mm, "limit_deg": limit_deg, "suspect_rows": suspect,
            "base_offset": fit["base_offset"], "flange_offset": fit["flange_offset"]}
