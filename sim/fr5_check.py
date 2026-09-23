"""Read-only check of the FR5 flange pose convention (FAIRINO SimMachine or a real FR5).

Jog the arm by hand (SimMachine web UI or teach pendant) and press Enter at
each pose. The script reads the joint angles and GetActualToolFlangePose(0),
computes forward kinematics from the FR5 DH table in the FAIRINO manual, and
tests which Euler convention makes the controller's pose agree with it. A
constant base offset and flange offset are fitted as well, so they cannot
cause a false alarm. No motion command is sent. Records are written once, at
the end, and only with --output.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

sys.dont_write_bytecode = True  # keep the source tree free of __pycache__
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import cv2  # noqa: E402
import numpy as np  # noqa: E402

from handeye import (  # noqa: E402
    inverse, matrix_to_parameters, parameters_to_matrix, pose_from_fr5, read_flange,
    rotation_angle_degrees, transform,
)

# FAIRINO manual, FR5 DH table: (a [mm], alpha [rad], d [mm]); theta offsets are 0.
FR5_DH = ((0.0, math.pi / 2, 152.0), (-425.0, 0.0, 0.0), (-395.0, 0.0, 0.0),
          (0.0, math.pi / 2, 102.0), (0.0, -math.pi / 2, 102.0), (0.0, 0.0, 100.0))
HANDEYE_CONVENTION = "Rz@Ry@Rx"
MIN_POSES = 6
MIN_JOINT_SPREAD_DEG = 20.0
# Nominal DH matches SimMachine to ~0.1 mm (URDF a3 is 395.01 mm). A real FR5
# may use per-robot kinematic calibration, so allow a few mm; a wrong
# convention is off by degrees to tens of degrees.
PASS_RMS_MM, PASS_RMS_DEG = 5.0, 0.5


def _axis(axis: int, angle: float) -> np.ndarray:
    rvec = np.zeros(3)
    rvec[axis] = angle
    return cv2.Rodrigues(rvec)[0]


CONVENTIONS = {
    HANDEYE_CONVENTION: None,  # handeye.pose_from_fr5 itself
    "Rx@Ry@Rz": lambda rx, ry, rz: _axis(0, rx) @ _axis(1, ry) @ _axis(2, rz),
    "rotation vector": lambda rx, ry, rz: cv2.Rodrigues(np.array([rx, ry, rz]))[0],
}


def fk(joints_deg) -> np.ndarray:
    """base_T_flange in metres from the standard DH parameters."""
    result = np.eye(4)
    for theta, (a, alpha, d) in zip(np.deg2rad(joints_deg), FR5_DH):
        ct, st, ca, sa = math.cos(theta), math.sin(theta), math.cos(alpha), math.sin(alpha)
        result = result @ np.array([[ct, -st * ca, st * sa, a / 1000 * ct],
                                    [st, ct * ca, -ct * sa, a / 1000 * st],
                                    [0.0, sa, ca, d / 1000], [0.0, 0.0, 0.0, 1.0]])
    return result


def controller_matrix(raw, convention: str) -> np.ndarray:
    if convention == HANDEYE_CONVENTION:
        return pose_from_fr5(raw)  # exercise the code path the calibration uses
    rx, ry, rz = np.deg2rad(np.asarray(raw[3:], dtype=float))
    return transform(CONVENTIONS[convention](rx, ry, rz), np.asarray(raw[:3], dtype=float) / 1000)


def fit_offsets(controller: list[np.ndarray], model: list[np.ndarray]) -> dict:
    """Fit constant X, Y with controller_i ~= X @ model_i @ Y (multi-start LM)."""
    from scipy.optimize import least_squares

    def residual(p):
        x, y = parameters_to_matrix(p[:6]), parameters_to_matrix(p[6:])
        values = []
        for c, m in zip(controller, model):
            delta = inverse(c) @ x @ m @ y
            values.extend(delta[:3, 3] * 1000.0)  # mm
            values.extend(cv2.Rodrigues(delta[:3, :3])[0].ravel() * 1000.0)  # mrad
        return np.asarray(values)

    best = None
    for kx in range(4):
        for ky in range(4):
            for flip in (0.0, math.pi):
                x0 = transform(_axis(2, kx * math.pi / 2), np.zeros(3))
                y0 = transform(_axis(2, ky * math.pi / 2) @ _axis(0, flip), np.zeros(3))
                fit = least_squares(residual, np.r_[matrix_to_parameters(x0), matrix_to_parameters(y0)])
                if best is None or fit.cost < best.cost:
                    best = fit
    x, y = parameters_to_matrix(best.x[:6]), parameters_to_matrix(best.x[6:])
    errors_mm, errors_deg = [], []
    for c, m in zip(controller, model):
        delta = inverse(c) @ x @ m @ y
        errors_mm.append(np.linalg.norm(delta[:3, 3]) * 1000.0)
        errors_deg.append(rotation_angle_degrees(delta[:3, :3]))
    return {"rms_mm": float(np.sqrt(np.mean(np.square(errors_mm)))),
            "rms_deg": float(np.sqrt(np.mean(np.square(errors_deg)))),
            "base_offset": describe(x), "flange_offset": describe(y)}


def describe(t: np.ndarray) -> dict:
    return {"translation_mm": (t[:3, 3] * 1000.0).round(3).tolist(),
            "rotation_deg": round(rotation_angle_degrees(t[:3, :3]), 4)}


def analyse(records: list[dict]) -> dict:
    if len(records) < MIN_POSES:
        raise ValueError(f"Need at least {MIN_POSES} poses, got {len(records)}")
    joints = np.asarray([r["joints_deg"] for r in records], dtype=float)
    model = [fk(j) for j in joints]
    fits = {name: fit_offsets([controller_matrix(r["flange_pose_mm_deg"], name) for r in records], model)
            for name in CONVENTIONS}
    best = min(fits, key=lambda name: fits[name]["rms_mm"] + 10.0 * fits[name]["rms_deg"])
    own = fits[HANDEYE_CONVENTION]
    spread = np.ptp(joints, axis=0)
    return {
        "poses": len(records), "joint_spread_deg": spread.round(1).tolist(), "fits": fits,
        "best_convention": best,
        "handeye_convention_ok": (best == HANDEYE_CONVENTION and own["rms_mm"] < PASS_RMS_MM
                                  and own["rms_deg"] < PASS_RMS_DEG),
        "weak_joints": [i + 1 for i, s in enumerate(spread) if s < MIN_JOINT_SPREAD_DEG],
    }


def print_analysis(result: dict):
    print(f"{result['poses']} poses; joint spread (deg): {result['joint_spread_deg']}")
    for name, fit in result["fits"].items():
        mark = " <- handeye.py" if name == HANDEYE_CONVENTION else ""
        print(f"  {name:16s} rms {fit['rms_mm']:9.4f} mm {fit['rms_deg']:9.5f} deg{mark}")
    own = result["fits"][HANDEYE_CONVENTION]
    print(f"Best: {result['best_convention']}. Fitted offsets for handeye.py's convention: "
          f"base {own['base_offset']}, flange {own['flange_offset']} (expected near zero)")
    if result["weak_joints"]:
        print(f"WARNING: joints {result['weak_joints']} moved < {MIN_JOINT_SPREAD_DEG:.0f} deg; "
              "the test is weak. Vary every joint.")
    print("PASS: handeye.py pose convention and units agree with FR5 kinematics"
          if result["handeye_convention_ok"] else
          "FAIL: handeye.py convention does not match; check Euler order, units and joint zeros")
    print("Residual expectation: SimMachine < 0.5 mm; a real FR5 may show a few mm "
          "from per-robot kinematic calibration.")


def read_joints(robot) -> list[float]:
    reply = robot.GetActualJointPosDegree(0)
    if not isinstance(reply, (tuple, list)) or len(reply) != 2 or reply[0] != 0:
        raise RuntimeError(f"GetActualJointPosDegree failed: {reply!r}")
    return [float(v) for v in reply[1]]


def record(robot_ip: str) -> list[dict]:
    try:
        from fairino import Robot
    except ImportError as exc:
        raise RuntimeError("Recording needs the FAIRINO Python SDK (fairino)") from exc
    robot = Robot.RPC(robot_ip)
    records = []
    try:
        print("Jog the arm by hand; vary all six joints (J4-J6 by 30+ deg).")
        print(f"Press Enter at each stationary pose ({MIN_POSES}+); type q to finish.")
        while True:
            if input("fr5> ").strip().lower() in ("q", "quit", "exit"):
                break
            before, pose, after = read_joints(robot), read_flange(robot), read_joints(robot)
            if max(abs(a - b) for a, b in zip(before, after)) > 0.01:
                print("Skipped: arm moved while reading")
                continue
            records.append({"joints_deg": before, "flange_pose_mm_deg": pose})
            print(f"Recorded pose {len(records)}")
    finally:
        if hasattr(robot, "CloseRPC"):
            robot.CloseRPC()
    return records


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--robot-ip", help="SimMachine or controller IP; the arm is never commanded")
    source.add_argument("--input", type=Path, help="Re-analyse records saved with --output")
    parser.add_argument("--output", type=Path, help="Save records and analysis (JSON) at the end")
    args = parser.parse_args()
    if args.input:
        records = json.loads(args.input.read_text(encoding="utf-8"))["records"]
    else:
        records = record(args.robot_ip)
    result = analyse(records)
    print_analysis(result)
    if args.output:
        args.output.write_text(json.dumps({"records": records, "analysis": result}, indent=2),
                               encoding="utf-8")
        print(f"Saved {args.output}")


if __name__ == "__main__":
    try:
        main()
    except (OSError, RuntimeError, ValueError) as error:
        print(f"Error: {error}", file=sys.stderr)
        sys.exit(1)
