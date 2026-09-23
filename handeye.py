"""FR5 + ZED X (rectified left camera) eye-in-hand calibration.

Frames use A_T_B to map points expressed in B into frame A. All calculations
use metres and radians; the FAIRINO SDK's raw millimetres/degrees are retained
in the capture data. No command in this program moves the robot.
"""

from __future__ import annotations

import argparse
import json
import math
import shutil
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import yaml


def read_image(path: Path) -> np.ndarray | None:
    """cv2.imread that also works for non-ASCII Windows paths (e.g. this repo's)."""
    try:
        data = np.fromfile(str(path), dtype=np.uint8)
    except OSError:
        return None
    return cv2.imdecode(data, cv2.IMREAD_UNCHANGED) if data.size else None


def write_image(path: Path, image: np.ndarray) -> bool:
    """cv2.imwrite counterpart of read_image."""
    ok, encoded = cv2.imencode(Path(path).suffix or ".png", image)
    if ok:
        Path(path).write_bytes(encoded.tobytes())
    return bool(ok)


def read_board(path: Path) -> dict:
    board = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(board, dict) or board.get("target_type") != "aprilgrid":
        raise ValueError("Target YAML must have target_type: aprilgrid")
    for key in ("tagCols", "tagRows"):
        if not isinstance(board.get(key), int) or board[key] < 2:
            raise ValueError(f"Invalid {key} in target YAML")
    for key in ("tagSize", "tagSpacing"):
        if not isinstance(board.get(key), (int, float)) or board[key] <= 0:
            raise ValueError(f"Invalid {key} in target YAML")
    if board["tagCols"] * board["tagRows"] > 587:
        raise ValueError("Target has more IDs than AprilTag 36h11 supports")
    return {k: board[k] for k in ("target_type", "tagCols", "tagRows", "tagSize", "tagSpacing")}


def tag_object_corners(tag_id: int, board: dict) -> np.ndarray:
    """Corners of one Kalibr tag, in OpenCV's canonical marker corner order.

    The Kalibr board origin is the bottom-left of tag 0; +x right, +y up.
    IDs grow left-to-right, then bottom-to-top. The Kalibr PDF generator
    rotates each code 180 degrees from OpenCV's canonical AprilTag code, so
    OpenCV corners 0..3 correspond to physical BR, BL, TL, TR respectively.
    """
    cols = board["tagCols"]
    row, col = divmod(tag_id, cols)
    size = float(board["tagSize"])
    pitch = size * (1.0 + float(board["tagSpacing"]))
    x, y = col * pitch, row * pitch
    return np.array(
        [[x + size, y, 0], [x, y, 0],
         [x, y + size, 0], [x + size, y + size, 0]], dtype=np.float64
    )


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
    return np.asarray(object_points, dtype=np.float64).reshape(-1, 3), np.asarray(image_points, dtype=np.float64).reshape(-1, 2), used_ids


def pose_from_fr5(raw: list[float]) -> np.ndarray:
    """Convert FR5 fixed-axis XYZ Euler pose [mm, deg] to base_T_flange.

    FAIRINO's moving-axis ZYX convention is equivalent to fixed-axis XYZ:
    Rz(rz) @ Ry(ry) @ Rx(rx). See its robot brief introduction manual.
    """
    if len(raw) != 6 or not all(math.isfinite(float(v)) for v in raw):
        raise ValueError("FR5 flange pose must contain six finite values")
    x, y, z = np.asarray(raw[:3], dtype=float) / 1000.0
    rx, ry, rz = np.deg2rad(np.asarray(raw[3:], dtype=float))
    cx, sx = math.cos(rx), math.sin(rx)
    cy, sy = math.cos(ry), math.sin(ry)
    cz, sz = math.cos(rz), math.sin(rz)
    rot_x = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    rot_y = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    rot_z = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    result = np.eye(4)
    result[:3, :3] = rot_z @ rot_y @ rot_x
    result[:3, 3] = [x, y, z]
    return result


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
    cosine = np.clip((np.trace(rotation) - 1.0) / 2.0, -1.0, 1.0)
    return float(np.rad2deg(np.arccos(cosine)))


def rotation_mean(rotations: list[np.ndarray]) -> np.ndarray:
    u, _, vt = np.linalg.svd(np.sum(rotations, axis=0))
    diag = np.diag([1.0, 1.0, np.linalg.det(u @ vt)])
    return u @ diag @ vt


def transform_mean(transforms: list[np.ndarray]) -> np.ndarray:
    return transform(rotation_mean([t[:3, :3] for t in transforms]),
                     np.mean([t[:3, 3] for t in transforms], axis=0))


def matrix_to_parameters(t: np.ndarray) -> np.ndarray:
    rvec, _ = cv2.Rodrigues(t[:3, :3])
    return np.r_[rvec.reshape(3), t[:3, 3]]


def parameters_to_matrix(p: np.ndarray) -> np.ndarray:
    rot, _ = cv2.Rodrigues(np.asarray(p[:3], dtype=np.float64))
    return transform(rot, p[3:6])


def read_flange(robot) -> list[float]:
    reply = robot.GetActualToolFlangePose(0)
    if not isinstance(reply, (tuple, list)) or len(reply) != 2 or reply[0] != 0:
        raise RuntimeError(f"GetActualToolFlangePose failed: {reply!r}")
    pose = [float(v) for v in reply[1]]
    pose_from_fr5(pose)
    return pose


def capture(args):
    try:
        import pyzed.sl as sl
        from fairino import Robot
    except ImportError as exc:
        raise RuntimeError("Capture needs the vendor ZED SDK (pyzed.sl) and FAIRINO Python SDK") from exc

    board = read_board(args.target)
    output = args.output.resolve()
    if output.exists():
        raise FileExistsError(f"Session directory already exists: {output}")

    camera = sl.Camera()
    init = sl.InitParameters()
    init.camera_resolution = getattr(sl.RESOLUTION, args.resolution)
    init.camera_fps = args.fps
    init.depth_mode = sl.DEPTH_MODE.NONE
    # AUTO flip rotates images 180 deg if the IMU sees the camera upside down
    # at open time; on a moving arm that silently changes the optical frame.
    init.camera_image_flip = sl.FLIP_MODE.OFF
    # Self-calibration may nudge the rectified left frame on every open; use
    # the same fixed factory calibration here and in the runtime application.
    init.camera_disable_self_calib = True
    if args.stream_ip:
        init.set_from_stream(args.stream_ip, args.stream_port)
    status = camera.open(init)
    if status != sl.ERROR_CODE.SUCCESS:
        camera.close()
        raise RuntimeError(f"ZED open failed: {status}")
    robot = None
    try:
        info = camera.get_camera_information()
        left = info.camera_configuration.calibration_parameters.left_cam
        resolution = info.camera_configuration.resolution
        session = {
            "schema_version": 1,
            "camera": "ZED X rectified left optical frame",
            "camera_serial": int(info.serial_number),
            "image_view": "LEFT",
            "zed_camera_image_flip": "OFF",
            "zed_self_calibration": False,
            "image_width": int(resolution.width),
            "image_height": int(resolution.height),
            "K": [[float(left.fx), 0.0, float(left.cx)],
                  [0.0, float(left.fy), float(left.cy)], [0.0, 0.0, 1.0]],
            "distortion": np.asarray(left.disto, dtype=float).reshape(-1).tolist(),
            "robot": "FAIRINO FR5",
            "robot_pose_source": "GetActualToolFlangePose(0)",
            "robot_pose_units": "mm and degrees",
            "robot_pose_convention": "fixed XYZ, R=Rz(rz)@Ry(ry)@Rx(rx)",
            "board": board,
            "samples": [],
        }
        robot = Robot.RPC(args.robot_ip)
        read_flange(robot)  # verify the connection before creating the session
        output.mkdir(parents=True, exist_ok=False)
        (output / "images").mkdir()
        shutil.copyfile(args.target, output / "target.yaml")
        (output / "session.json").write_text(json.dumps(session, indent=2), encoding="utf-8")
        frame = sl.Mat()
        print(f"Session: {output}\nMove robot manually; keep board fixed and in view.")
        print("Press Enter after each pose is stationary; type q to finish.")
        while True:
            answer = input("capture> ").strip().lower()
            if answer in ("q", "quit", "exit"):
                break
            before = read_flange(robot)
            t_before = time.monotonic_ns()
            grab = camera.grab()
            if grab != sl.ERROR_CODE.SUCCESS:
                print(f"Skipped: ZED grab failed ({grab})")
                continue
            retrieved = camera.retrieve_image(frame, sl.VIEW.LEFT)
            if retrieved != sl.ERROR_CODE.SUCCESS:
                print(f"Skipped: ZED left image retrieval failed ({retrieved})")
                continue
            image_time_ns = int(camera.get_timestamp(sl.TIME_REFERENCE.IMAGE).get_nanoseconds())
            t_after = time.monotonic_ns()
            after = read_flange(robot)
            relative = inverse(pose_from_fr5(before)) @ pose_from_fr5(after)
            moved_mm = float(np.linalg.norm(relative[:3, 3]) * 1000.0)
            moved_deg = rotation_angle_degrees(relative[:3, :3])
            if moved_mm > args.max_motion_mm or moved_deg > args.max_motion_deg:
                print(f"Skipped: robot moved {moved_mm:.2f} mm / {moved_deg:.2f} deg during grab")
                continue
            image = frame.get_data().copy()
            if (image.shape[1], image.shape[0]) != (session["image_width"], session["image_height"]):
                raise RuntimeError("Image size differs from saved ZED intrinsics")
            filename = f"images/{len(session['samples']):04d}.png"
            if not write_image(output / filename, image):
                raise RuntimeError(f"Could not write {filename}")
            session["samples"].append({
                "image": filename,
                "flange_pose_mm_deg": before,
                "flange_pose_after_mm_deg": after,
                "zed_image_timestamp_ns": image_time_ns,
                "host_grab_before_ns": t_before,
                "host_grab_after_ns": t_after,
            })
            (output / "session.json").write_text(json.dumps(session, indent=2), encoding="utf-8")
            print(f"Saved {filename}; {len(session['samples'])} poses total")
    finally:
        camera.close()
        if robot is not None and hasattr(robot, "CloseRPC"):
            robot.CloseRPC()


def detect_command(args):
    board = read_board(args.target)
    image = read_image(args.image)
    if image is None:
        raise FileNotFoundError(args.image)
    object_points, image_points, ids = detect_board(image, board, detector())
    print(f"Detected {len(ids)} board tags: {ids}")
    if args.overlay:
        if image.ndim == 2:
            drawing = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        elif image.shape[2] == 4:
            drawing = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
        else:
            drawing = image.copy()
        for tag_id, corners in zip(ids, image_points.reshape(-1, 4, 2)):
            centre = np.mean(corners, axis=0).astype(int)
            cv2.putText(drawing, str(tag_id), tuple(centre), cv2.FONT_HERSHEY_SIMPLEX,
                        0.8, (0, 0, 255), 2)
            for j, corner in enumerate(corners.astype(int)):
                cv2.circle(drawing, tuple(corner), 5, (0, 255, 0), -1)
                if j == 0:
                    cv2.putText(drawing, "0", tuple(corner), cv2.FONT_HERSHEY_SIMPLEX,
                                0.6, (255, 0, 0), 2)
        if not write_image(args.overlay, drawing):
            raise RuntimeError(f"Could not write overlay {args.overlay}")
        print(f"Overlay: {args.overlay}")


def fit_pose(obj: np.ndarray, img: np.ndarray, k: np.ndarray, distortion: np.ndarray,
             rvec: np.ndarray, tvec: np.ndarray, min_tags: int):
    """LM pose refinement that drops whole tags with a corner far off the fit.

    Such a corner locked onto the wrong feature; a few of them can hide under
    the view RMSE limit while still biasing the pose. Points come 4 per tag.
    """
    subset = np.arange(len(obj))
    for _ in range(4):
        rvec, tvec = cv2.solvePnPRefineLM(obj[subset], img[subset], k, distortion, rvec, tvec)
        projected, _ = cv2.projectPoints(obj[subset], rvec, tvec, k, distortion)
        errors = np.linalg.norm(projected.reshape(-1, 2) - img[subset], axis=1)
        tags = subset // 4
        bad = np.unique(tags[errors > max(1.0, 5.0 * float(np.median(errors)))])
        if not len(bad) or len(np.unique(tags)) - len(bad) < min_tags:
            break
        subset = subset[~np.isin(tags, bad)]
    else:
        rvec, tvec = cv2.solvePnPRefineLM(obj[subset], img[subset], k, distortion, rvec, tvec)
        projected, _ = cv2.projectPoints(obj[subset], rvec, tvec, k, distortion)
        errors = np.linalg.norm(projected.reshape(-1, 2) - img[subset], axis=1)
    return rvec, tvec, subset, float(np.sqrt(np.mean(errors ** 2)))


def observations_from_session(session_dir: Path, board: dict, session: dict,
                              min_tags: int, max_pnp_rmse: float) -> tuple[list[dict], list[dict]]:
    k = np.asarray(session["K"], dtype=np.float64)
    distortion = np.asarray(session["distortion"], dtype=np.float64)
    aruco_detector = detector()
    accepted, rejected = [], []
    for index, sample in enumerate(session["samples"]):
        image = read_image(session_dir / sample["image"])
        if image is None:
            rejected.append({"index": index, "reason": "image missing"})
            continue
        if (image.shape[1], image.shape[0]) != (session["image_width"], session["image_height"]):
            rejected.append({"index": index, "reason": "resolution mismatch"})
            continue
        obj, img, ids = detect_board(image, board, aruco_detector)
        if len(ids) < min_tags:
            rejected.append({"index": index, "reason": f"only {len(ids)} tags"})
            continue
        subset = np.arange(len(obj))
        rmse = float("inf")
        try:
            # A planar Aprilgrid gives ITERATIVE PnP a homography initialisation.
            # Clean frames should retain every corner; RANSAC is only a fallback.
            ok, rvec, tvec = cv2.solvePnP(
                obj, img, k, distortion, flags=cv2.SOLVEPNP_ITERATIVE
            )
            if ok:
                rvec, tvec, subset, rmse = fit_pose(obj, img, k, distortion, rvec, tvec, min_tags)
        except cv2.error:
            pass
        if rmse > max_pnp_rmse:
            try:
                ok, rvec, tvec, inliers = cv2.solvePnPRansac(
                    obj, img, k, distortion, iterationsCount=300,
                    reprojectionError=3.0, confidence=0.999,
                    flags=cv2.SOLVEPNP_EPNP
                )
                if not ok or inliers is None or len(inliers) < max(12, math.ceil(0.6 * len(obj))):
                    raise ValueError("insufficient PnP inliers")
                subset = inliers.reshape(-1)
                rvec, tvec = cv2.solvePnPRefineLM(
                    obj[subset], img[subset], k, distortion, rvec, tvec
                )
                projected, _ = cv2.projectPoints(obj[subset], rvec, tvec, k, distortion)
                rmse = float(np.sqrt(np.mean(np.sum(
                    (projected.reshape(-1, 2) - img[subset]) ** 2, axis=1
                ))))
            except (cv2.error, ValueError):
                rejected.append({"index": index, "reason": "PnP failed"})
                continue
        if not math.isfinite(rmse) or rmse > max_pnp_rmse:
            rejected.append({"index": index, "reason": f"PnP RMSE {rmse:.2f} px"})
            continue
        rot, _ = cv2.Rodrigues(rvec)
        kept_ids = sorted({ids[i // 4] for i in subset.tolist()})
        accepted.append({
            "index": index, "base_T_flange": pose_from_fr5(sample["flange_pose_mm_deg"]),
            "camera_T_board": transform(rot, tvec), "object_points": obj[subset],
            "image_points": img[subset], "tag_ids": kept_ids,
            "dropped_tag_ids": sorted(set(ids) - set(kept_ids)), "pnp_rmse_px": rmse,
        })
    return accepted, rejected


def check_motion_diversity(observations: list[dict]):
    first = observations[0]["base_T_flange"]
    vectors = []
    for obs in observations[1:]:
        relative = inverse(first) @ obs["base_T_flange"]
        rvec, _ = cv2.Rodrigues(relative[:3, :3])
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


def score(observations: list[dict], flange_t_camera: np.ndarray,
          base_t_board: np.ndarray, k: np.ndarray, distortion: np.ndarray) -> dict:
    corner_errors, position_errors, angle_errors = [], [], []
    for obs in observations:
        estimated = obs["base_T_flange"] @ flange_t_camera @ obs["camera_T_board"]
        delta = inverse(base_t_board) @ estimated
        position_errors.append(float(np.linalg.norm(delta[:3, 3]) * 1000.0))
        angle_errors.append(rotation_angle_degrees(delta[:3, :3]))
        camera_t_board = inverse(obs["base_T_flange"] @ flange_t_camera) @ base_t_board
        if np.any((camera_t_board[:3, :3] @ obs["object_points"].T +
                   camera_t_board[:3, 3:4])[2] <= 0):
            return {"pixel_rmse": float("inf"), "board_position_rmse_mm": float("inf"),
                    "board_rotation_rmse_deg": float("inf")}
        rvec, _ = cv2.Rodrigues(camera_t_board[:3, :3])
        projected, _ = cv2.projectPoints(obs["object_points"], rvec,
                                         camera_t_board[:3, 3], k, distortion)
        corner_errors.extend(np.sum((projected.reshape(-1, 2) - obs["image_points"]) ** 2, axis=1))
    return {
        "pixel_rmse": float(np.sqrt(np.mean(corner_errors))),
        "board_position_rmse_mm": float(np.sqrt(np.mean(np.square(position_errors)))),
        "board_rotation_rmse_deg": float(np.sqrt(np.mean(np.square(angle_errors)))),
    }


def refine(train: list[dict], initial_camera: np.ndarray, initial_board: np.ndarray,
           k: np.ndarray, distortion: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    from scipy.optimize import least_squares

    initial = np.r_[matrix_to_parameters(initial_camera), matrix_to_parameters(initial_board)]

    def residual(parameters):
        flange_t_camera = parameters_to_matrix(parameters[:6])
        base_t_board = parameters_to_matrix(parameters[6:])
        values = []
        for obs in train:
            camera_t_board = inverse(obs["base_T_flange"] @ flange_t_camera) @ base_t_board
            camera_points = camera_t_board[:3, :3] @ obs["object_points"].T + camera_t_board[:3, 3:4]
            if np.any(camera_points[2] <= 0):
                values.extend(np.full(obs["image_points"].size, 10000.0))
                continue
            rvec, _ = cv2.Rodrigues(camera_t_board[:3, :3])
            projected, _ = cv2.projectPoints(obs["object_points"], rvec,
                                             camera_t_board[:3, 3], k, distortion)
            values.extend((projected.reshape(-1, 2) - obs["image_points"]).ravel())
        return np.asarray(values, dtype=np.float64)

    result = least_squares(residual, initial, loss="huber", f_scale=2.0,
                           x_scale="jac", max_nfev=200)
    if not result.success and result.status <= 0:
        raise RuntimeError(f"Pixel refinement did not converge: {result.message}")
    return parameters_to_matrix(result.x[:6]), parameters_to_matrix(result.x[6:])


def solve(args):
    required_api = ("calibrateHandEye", "CALIB_HAND_EYE_PARK",
                    "CALIB_HAND_EYE_DANIILIDIS", "solvePnPRefineLM")
    missing = [name for name in required_api if not hasattr(cv2, name)]
    if missing:
        raise RuntimeError(f"OpenCV lacks {missing}; install opencv-contrib-python >=4.8,<5")
    session_dir = args.session.resolve()
    session = json.loads((session_dir / "session.json").read_text(encoding="utf-8"))
    if session.get("image_view") != "LEFT" or session.get("schema_version") != 1:
        raise ValueError("Expected a v1 session with rectified ZED left images")
    if session.get("zed_camera_image_flip") != "OFF" or session.get("zed_self_calibration") is not False:
        print("WARNING: session predates fixed ZED flip/self-calibration settings; "
              "the left optical frame may not match runtime. Recapture if possible.",
              file=sys.stderr)
    board = read_board(args.target or session_dir / "target.yaml")
    if board != session["board"]:
        print("WARNING: board YAML differs from capture metadata; check the printed target.", file=sys.stderr)
    observations, rejected = observations_from_session(
        session_dir, board, session, args.min_tags, args.max_pnp_rmse
    )
    if len(observations) < 12:
        raise ValueError(f"Only {len(observations)} usable views; collect at least 12, ideally 20-30")
    holdout = [o for i, o in enumerate(observations) if i % 5 == 0]
    train = [o for i, o in enumerate(observations) if i % 5 != 0]
    check_motion_diversity(train)
    k = np.asarray(session["K"], dtype=np.float64)
    distortion = np.asarray(session["distortion"], dtype=np.float64)
    candidates = []
    for name, method in (("Park", cv2.CALIB_HAND_EYE_PARK),
                         ("Daniilidis", cv2.CALIB_HAND_EYE_DANIILIDIS)):
        try:
            flange_t_camera = handeye(train, method)
            base_t_board = transform_mean(board_poses(train, flange_t_camera))
            candidates.append({
                "method": name, "flange_T_left_camera": flange_t_camera,
                "base_T_board": base_t_board,
                "train": score(train, flange_t_camera, base_t_board, k, distortion),
                "holdout": score(holdout, flange_t_camera, base_t_board, k, distortion),
            })
        except (cv2.error, ValueError) as exc:
            print(f"{name} failed: {exc}", file=sys.stderr)
    candidates = [item for item in candidates
                  if all(math.isfinite(value)
                         for phase in ("train", "holdout")
                         for value in item[phase].values())]
    if not candidates:
        raise RuntimeError("Both hand-eye solvers failed or produced invalid projections")
    park = next((item for item in candidates if item["method"] == "Park"), candidates[0])
    if not args.no_refine:
        try:
            flange_t_camera, base_t_board = refine(
                train, park["flange_T_left_camera"], park["base_T_board"], k, distortion
            )
            candidates.append({
                "method": "Park+pixel_refinement", "flange_T_left_camera": flange_t_camera,
                "base_T_board": base_t_board,
                "train": score(train, flange_t_camera, base_t_board, k, distortion),
                "holdout": score(holdout, flange_t_camera, base_t_board, k, distortion),
            })
        except (ImportError, RuntimeError, ValueError, cv2.error) as exc:
            print(f"Pixel refinement unavailable: {exc}", file=sys.stderr)
    candidates = [item for item in candidates
                  if all(math.isfinite(value)
                         for phase in ("train", "holdout")
                         for value in item[phase].values())]
    best = min(candidates, key=lambda item: item["holdout"]["pixel_rmse"])
    def serialise(item):
        return {"method": item["method"],
                "flange_T_left_camera": item["flange_T_left_camera"].tolist(),
                "left_camera_T_flange": inverse(item["flange_T_left_camera"]).tolist(),
                "base_T_board": item["base_T_board"].tolist(),
                "train": item["train"], "holdout": item["holdout"]}
    result = {
        "chosen_method": best["method"], "chosen": serialise(best),
        "candidates": [serialise(item) for item in candidates],
        "accepted_sample_indices": [o["index"] for o in observations],
        "accepted_views": [{"index": o["index"], "tag_ids": o["tag_ids"],
                            "dropped_tag_ids": o["dropped_tag_ids"],
                            "pnp_rmse_px": o["pnp_rmse_px"]} for o in observations],
        "holdout_sample_indices": [o["index"] for o in holdout],
        "rejected": rejected, "board": board,
        "camera_serial": session["camera_serial"], "K": session["K"],
        "zed_camera_image_flip": session.get("zed_camera_image_flip", "unknown (AUTO)"),
        "zed_self_calibration": session.get("zed_self_calibration", "unknown (enabled)"),
        "robot_pose_convention": session["robot_pose_convention"],
        "warning": "Verify FR5 Euler convention and physical tag size before use on the robot",
    }
    destination = args.output or session_dir / "result.json"
    destination.write_text(json.dumps(result, indent=2, allow_nan=False), encoding="utf-8")
    print(f"Accepted {len(observations)} views; rejected {len(rejected)}; holdout {len(holdout)}")
    for item in candidates:
        print(f"{item['method']}: holdout {item['holdout']['pixel_rmse']:.3f} px, "
              f"board {item['holdout']['board_position_rmse_mm']:.2f} mm / "
              f"{item['holdout']['board_rotation_rmse_deg']:.2f} deg")
    print(f"Selected {best['method']}; saved {destination}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    capture_parser = commands.add_parser("capture", help="Manually collect paired ZED/FR5 data")
    capture_parser.add_argument("--robot-ip", required=True)
    capture_parser.add_argument("--target", type=Path, default=Path(__file__).with_name("target.example.yaml"))
    capture_parser.add_argument("--output", required=True, type=Path)
    capture_parser.add_argument("--resolution", choices=("HD1200", "HD1080", "SVGA"), default="HD1200")
    capture_parser.add_argument("--fps", type=int, default=30)
    capture_parser.add_argument("--stream-ip", help="ZED SDK stream sender IP, if camera is on Jetson")
    capture_parser.add_argument("--stream-port", type=int, default=30000)
    capture_parser.add_argument("--max-motion-mm", type=float, default=0.5)
    capture_parser.add_argument("--max-motion-deg", type=float, default=0.1)
    capture_parser.set_defaults(func=capture)

    detect_parser = commands.add_parser("detect", help="Check Aprilgrid tag IDs in one image")
    detect_parser.add_argument("--image", type=Path, required=True)
    detect_parser.add_argument("--target", type=Path, default=Path(__file__).with_name("target.example.yaml"))
    detect_parser.add_argument("--overlay", type=Path)
    detect_parser.set_defaults(func=detect_command)

    solve_parser = commands.add_parser("solve", help="Calibrate from a captured session")
    solve_parser.add_argument("--session", required=True, type=Path)
    solve_parser.add_argument("--target", type=Path, help="Corrected Kalibr YAML, if different from capture")
    solve_parser.add_argument("--output", type=Path)
    solve_parser.add_argument("--min-tags", type=int, default=4)
    solve_parser.add_argument("--max-pnp-rmse", type=float, default=2.0)
    solve_parser.add_argument("--no-refine", action="store_true")
    solve_parser.set_defaults(func=solve)
    args = parser.parse_args()
    args.func(args)


if __name__ == "__main__":
    try:
        main()
    except (OSError, RuntimeError, ValueError, yaml.YAMLError) as error:
        parser_message = f"Error: {error}"
        print(parser_message, file=sys.stderr)
        sys.exit(1)
