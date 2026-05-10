"""
Hand-eye calibration for the wrist-mounted RealSense D435 (sn 153122074137).

Solves T_ee_camera (4x4) — the rigid transform from EE flange to camera frame
— so the per-frame world-frame extrinsic can be computed as
T_base_camera = T_w_e @ T_ee_camera at runtime, replacing the static
provisional 3.3 cm calibration used by the exterior camera.

Usage:
    python calibrate_handeye.py --marker-size-m 0.0508 [--num-poses 25] ...

Pre-conditions:
- polymetis robot server (50051) and gripper server (50052) running on the NUC.
- ip_debug_ui server stopped (it holds the camera and the impedance controller).
- Marker printed, taped flat to table, and visible in the wrist camera at the
  current arm pose. Run with --probe-only first to verify detection.
- Arm in a pose that puts the marker near image center, ~25-40 cm above it.
"""
import argparse
import sys
import time
from pathlib import Path

import cv2
import numpy as np


def _gather_dicts():
    """Build dict-name → cv2.aruco.DICT_* map, skipping any constants the
    installed OpenCV version doesn't define. Includes AprilTag families."""
    names = [
        "DICT_4X4_50", "DICT_5X5_50", "DICT_5X5_100", "DICT_6X6_50",
        "DICT_APRILTAG_16h5", "DICT_APRILTAG_25h9",
        "DICT_APRILTAG_36h10", "DICT_APRILTAG_36h11",
    ]
    return {n: getattr(cv2.aruco, n) for n in names if hasattr(cv2.aruco, n)}


ARUCO_DICTS = _gather_dicts()


def make_pose(pos: np.ndarray, quat_xyzw: np.ndarray) -> np.ndarray:
    """Pack (pos[3], quat[4] xyzw) → SE(3) 4x4 matrix."""
    from scipy.spatial.transform import Rotation as R
    T = np.eye(4)
    T[:3, :3] = R.from_quat(quat_xyzw).as_matrix()
    T[:3,  3] = pos
    return T


def open_camera(serial: str, w: int, h: int, fps: int):
    """Open a RealSense pipeline and return (pipe, align_to_color, K, dist_coefs)."""
    import pyrealsense2 as rs

    cfg = rs.config()
    cfg.enable_device(serial)
    cfg.enable_stream(rs.stream.color, w, h, rs.format.bgr8, fps)
    cfg.enable_stream(rs.stream.depth, w, h, rs.format.z16, fps)
    pipe = rs.pipeline()
    prof = pipe.start(cfg)
    align = rs.align(rs.stream.color)
    color_prof = prof.get_stream(rs.stream.color).as_video_stream_profile()
    intr = color_prof.get_intrinsics()
    K = np.array(
        [[intr.fx, 0, intr.ppx],
         [0, intr.fy, intr.ppy],
         [0, 0, 1]], dtype=np.float64,
    )
    dist = np.array(intr.coeffs, dtype=np.float64)
    return pipe, align, K, dist


def grab_color_frame(pipe, align, warmup: int = 5) -> np.ndarray:
    """Grab a color frame, dropping `warmup` frames first to flush stale ones."""
    frames = None
    for _ in range(warmup):
        frames = pipe.wait_for_frames(timeout_ms=2000)
    frames = align.process(frames)
    color = frames.get_color_frame()
    if color is None:
        raise RuntimeError("no color frame")
    return np.asanyarray(color.get_data())  # BGR


def _detect_aruco_markers(rgb_bgr, aruco_dict):
    """Common ArUco marker detection — returns (corners, ids) regardless of OpenCV version."""
    gray = cv2.cvtColor(rgb_bgr, cv2.COLOR_BGR2GRAY)
    if hasattr(cv2.aruco, "ArucoDetector"):
        detector = cv2.aruco.ArucoDetector(aruco_dict, cv2.aruco.DetectorParameters())
        return detector.detectMarkers(gray)[:2]
    params = cv2.aruco.DetectorParameters_create()
    corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=params)
    return corners, ids


def make_charuco_board(cols: int, rows: int, square_m: float, marker_m: float,
                        dict_id: int, legacy_pattern: bool = False):
    """Construct a CharucoBoard, handling both new (4.7+) and legacy OpenCV APIs.

    legacy_pattern=True is needed for boards generated with OpenCV < 4.6 (or
    other tools that mirror that layout) — the marker IDs are arranged in a
    different chessboard order. Set this if your board was printed years ago
    or if `setLegacyPattern` returns more corners than not.
    """
    aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)
    if hasattr(cv2.aruco, "CharucoBoard") and hasattr(cv2.aruco.CharucoBoard, "__init__"):
        try:
            board = cv2.aruco.CharucoBoard((cols, rows), square_m, marker_m, aruco_dict)
            if legacy_pattern and hasattr(board, "setLegacyPattern"):
                board.setLegacyPattern(True)
            return board, aruco_dict
        except TypeError:
            pass
    board = cv2.aruco.CharucoBoard_create(cols, rows, square_m, marker_m, aruco_dict)
    return board, aruco_dict


def detect_charuco_pose(rgb_bgr, K, dist, board, aruco_dict, min_corners: int = 6):
    """
    Detect the ChArUco board and solve its pose. Returns
    (T_camera_board 4x4, n_charuco_corners) or (None, 0) if not enough corners.
    """
    gray = cv2.cvtColor(rgb_bgr, cv2.COLOR_BGR2GRAY)

    # 1. Detect markers
    if hasattr(cv2.aruco, "CharucoDetector"):
        # New API (>= 4.7) — single-call detection
        detector = cv2.aruco.CharucoDetector(board)
        ch_corners, ch_ids, _, _ = detector.detectBoard(gray)
    else:
        # Legacy: detect markers, then interpolate ChArUco corners
        marker_corners, marker_ids = _detect_aruco_markers(rgb_bgr, aruco_dict)
        if marker_ids is None or len(marker_ids) == 0:
            return None, 0
        _, ch_corners, ch_ids = cv2.aruco.interpolateCornersCharuco(
            marker_corners, marker_ids, gray, board,
        )

    if ch_corners is None or ch_ids is None or len(ch_corners) < min_corners:
        return None, 0 if ch_corners is None else len(ch_corners)

    # 2. Solve pose. Prefer matchImagePoints + solvePnP (new API).
    if hasattr(board, "matchImagePoints"):
        obj_pts, img_pts = board.matchImagePoints(ch_corners, ch_ids)
        if obj_pts is None or len(obj_pts) < 4:
            return None, len(ch_corners)
        ok, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, K, dist, flags=cv2.SOLVEPNP_ITERATIVE)
    else:
        # Legacy: estimatePoseCharucoBoard
        rvec = np.zeros((3, 1))
        tvec = np.zeros((3, 1))
        ok, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(
            ch_corners, ch_ids, board, K, dist, rvec, tvec,
        )

    if not ok:
        return None, len(ch_corners)

    Rmat, _ = cv2.Rodrigues(rvec)
    T = np.eye(4)
    T[:3, :3] = Rmat
    T[:3,  3] = tvec.reshape(3)
    return T, len(ch_corners)


def detect_marker(rgb_bgr, K, dist, marker_id: int, marker_size_m: float, dict_id: int):
    """
    Detect the requested ArUco marker and solve PnP. Returns
    (T_camera_marker 4x4, image_corners (4,2)) or (None, None) if not found.
    """
    aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)
    corners, ids = _detect_aruco_markers(rgb_bgr, aruco_dict)

    if ids is None:
        return None, None
    ids_flat = ids.flatten().tolist()
    if marker_id not in ids_flat:
        return None, None

    img_pts = corners[ids_flat.index(marker_id)][0].astype(np.float64)  # (4,2)
    s = marker_size_m / 2.0
    obj_pts = np.array(
        [[-s,  s, 0],
         [ s,  s, 0],
         [ s, -s, 0],
         [-s, -s, 0]], dtype=np.float64,
    )
    flag = cv2.SOLVEPNP_IPPE_SQUARE if hasattr(cv2, "SOLVEPNP_IPPE_SQUARE") else cv2.SOLVEPNP_ITERATIVE
    ok, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, K, dist, flags=flag)
    if not ok:
        return None, None
    Rmat, _ = cv2.Rodrigues(rvec)
    T = np.eye(4)
    T[:3, :3] = Rmat
    T[:3,  3] = tvec.reshape(3)
    return T, img_pts


def perturb_pose_in_ee(T_w_e_init: np.ndarray, xyz_range: float, rpy_range: float, rng) -> np.ndarray:
    """Generate a target world-frame pose by perturbing T_w_e_init in the EE frame.
    Translation in [-xyz_range, +xyz_range] m on each axis, RPY in
    [-rpy_range, +rpy_range] rad on each axis (EE-frame Euler XYZ).
    """
    from scipy.spatial.transform import Rotation as R
    dxyz = rng.uniform(-xyz_range, xyz_range, size=3)
    drpy = rng.uniform(-rpy_range, rpy_range, size=3)
    T_perturb = np.eye(4)
    T_perturb[:3, :3] = R.from_euler("xyz", drpy).as_matrix()
    T_perturb[:3,  3] = dxyz
    return T_w_e_init @ T_perturb


def solve_handeye(samples):
    R_w_e = [s[0][:3, :3] for s in samples]
    t_w_e = [s[0][:3,  3] for s in samples]
    R_c_m = [s[1][:3, :3] for s in samples]
    t_c_m = [s[1][:3,  3] for s in samples]

    # cv2.calibrateHandEye signature: gripper2base + target2cam → ee_in_camera...
    # actually: returns R, t such that T_ee_camera satisfies AX = XB. Method PARK
    # is robust and closed-form for rotation.
    R_ec, t_ec = cv2.calibrateHandEye(
        R_gripper2base=R_w_e,
        t_gripper2base=t_w_e,
        R_target2cam=R_c_m,
        t_target2cam=t_c_m,
        method=cv2.CALIB_HAND_EYE_PARK,
    )
    T_ec = np.eye(4)
    T_ec[:3, :3] = R_ec
    T_ec[:3,  3] = t_ec.reshape(3)
    return T_ec


def validate(samples, T_ec):
    """Compute T_w_marker per frame; std across frames = quality measure."""
    from scipy.spatial.transform import Rotation as R
    T_wm = np.stack([T_w_e @ T_ec @ T_c_m for (T_w_e, T_c_m) in samples])
    t_std = T_wm[:, :3, 3].std(axis=0)
    eulers = np.stack([R.from_matrix(Ti[:3, :3]).as_euler("xyz", degrees=True) for Ti in T_wm])
    e_std = eulers.std(axis=0)
    return T_wm, t_std, e_std


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--robot-host", default="localhost")
    ap.add_argument("--robot-port", type=int, default=50051)
    ap.add_argument("--camera-serial", default="153122074137")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--fps", type=int, default=30)
    # --- single-marker mode ---
    ap.add_argument("--marker-size-m", type=float, default=None,
                    help="(Single-marker mode) PHYSICAL side length of the printed marker in m. "
                         "Required when --charuco is NOT set.")
    ap.add_argument("--marker-id", type=int, default=0)
    ap.add_argument("--marker-dict", choices=list(ARUCO_DICTS), default="DICT_5X5_50")
    # --- ChArUco mode ---
    ap.add_argument("--charuco", action="store_true",
                    help="ChArUco-board mode. Recommended over single marker (subpixel "
                         "corners, more constraints, robust to partial occlusion).")
    ap.add_argument("--cb-cols", type=int, default=9, help="ChArUco squares along X (long edge).")
    ap.add_argument("--cb-rows", type=int, default=6, help="ChArUco squares along Y (short edge).")
    ap.add_argument("--cb-square-m", type=float, default=0.021,
                    help="ChArUco physical square side length (m).")
    ap.add_argument("--cb-marker-m", type=float, default=0.015,
                    help="ChArUco marker side length inside each square (m).")
    ap.add_argument("--cb-dict", choices=list(ARUCO_DICTS), default="DICT_4X4_50")
    ap.add_argument("--cb-legacy-pattern", action="store_true", default=True,
                    help="Use OpenCV < 4.6 ChArUco layout (column-major / shifted parity). "
                         "Default True for the board we have on hand. Pass --no-cb-legacy-pattern "
                         "to disable.")
    ap.add_argument("--no-cb-legacy-pattern", dest="cb_legacy_pattern", action="store_false")
    ap.add_argument("--cb-min-corners", type=int, default=6,
                    help="Minimum ChArUco corners to accept a frame.")
    ap.add_argument("--discover", action="store_true",
                    help="Probe-only mode that scans ALL known dicts (ArUco + AprilTag) "
                         "and reports which dict + which IDs are visible. Use this when "
                         "you don't yet know what's on your board.")
    ap.add_argument("--num-poses", type=int, default=25)
    ap.add_argument("--xyz-range", type=float, default=0.025,
                    help="Translation perturbation half-range per EE axis (m). Default 2.5 cm.")
    ap.add_argument("--rpy-range", type=float, default=0.20,
                    help="Rotation perturbation half-range per EE axis (rad). Default ~11°. "
                         "Larger triggers joint velocity safety limits, especially joint 6.")
    ap.add_argument("--time-per-move", type=float, default=5.0,
                    help="Seconds for each pose-to-pose move. Default 5 s — slow enough to "
                         "stay well inside polymetis joint velocity limits (2.51 rad/s).")
    ap.add_argument("--settle-s", type=float, default=0.6,
                    help="Wait this long after each move before grabbing the frame, so the "
                         "arm fully settles and auto-exposure adapts.")
    ap.add_argument("--out", type=Path, default=Path("T_ee_camera.npy"))
    ap.add_argument("--debug-dir", type=Path, default=None,
                    help="If set, dump captured frames per pose for inspection.")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--probe-only", action="store_true",
                    help="Only verify camera + marker detection at current pose, then exit "
                         "without moving the arm. Use this to check setup before committing.")
    args = ap.parse_args()

    rng = np.random.default_rng(args.seed)
    if args.debug_dir is not None:
        args.debug_dir.mkdir(parents=True, exist_ok=True)

    # --- camera ---
    print(f"[cal] opening camera {args.camera_serial} @ {args.width}x{args.height}@{args.fps}")
    pipe, align, K, dist = open_camera(args.camera_serial, args.width, args.height, args.fps)
    print(f"[cal] K =\n{K}")
    print(f"[cal] dist coeffs = {dist}")

    try:
        # --- discover mode: scan all known dicts, report what's visible, exit ---
        if args.discover:
            rgb = grab_color_frame(pipe, align)
            print(f"[cal] --discover: scanning {len(ARUCO_DICTS)} dicts")
            print(f"[cal] available dicts: {list(ARUCO_DICTS)}")
            found_any = False
            for name, dict_id in ARUCO_DICTS.items():
                aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)
                gray = cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY)
                if hasattr(cv2.aruco, "ArucoDetector"):
                    det = cv2.aruco.ArucoDetector(aruco_dict, cv2.aruco.DetectorParameters())
                    corners, ids, _ = det.detectMarkers(gray)
                else:
                    params = cv2.aruco.DetectorParameters_create()
                    corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=params)
                if ids is not None and len(ids) > 0:
                    found_any = True
                    print(f"[cal]   {name}: ids = {ids.flatten().tolist()}")
            if not found_any:
                print("[cal] no markers detected with any dict.")
                print("[cal] check: marker visible in camera? lighting? out of focus?")
            if args.debug_dir is not None:
                cv2.imwrite(str(args.debug_dir / "discover.jpg"), rgb)
                print(f"[cal] saved frame to {args.debug_dir / 'discover.jpg'}")
            return

        # --- choose detector based on mode ---
        if args.charuco:
            board, cb_aruco_dict = make_charuco_board(
                args.cb_cols, args.cb_rows, args.cb_square_m, args.cb_marker_m,
                ARUCO_DICTS[args.cb_dict], legacy_pattern=args.cb_legacy_pattern,
            )
            print(f"[cal] mode=ChArUco {args.cb_cols}x{args.cb_rows} squares, "
                  f"square={args.cb_square_m * 1000:.1f}mm marker={args.cb_marker_m * 1000:.1f}mm "
                  f"dict={args.cb_dict} legacy={args.cb_legacy_pattern}")

            def detect(rgb):
                T, n = detect_charuco_pose(rgb, K, dist, board, cb_aruco_dict, args.cb_min_corners)
                return T, f"{n} ChArUco corners"
        else:
            if args.marker_size_m is None:
                print("[cal] FAIL: --marker-size-m is required when --charuco is not set.",
                      file=sys.stderr)
                sys.exit(2)
            print(f"[cal] mode=single marker dict={args.marker_dict} id={args.marker_id} "
                  f"size={args.marker_size_m * 1000:.1f}mm")

            def detect(rgb):
                T, _ = detect_marker(
                    rgb, K, dist, args.marker_id, args.marker_size_m,
                    ARUCO_DICTS[args.marker_dict],
                )
                return T, "marker"

        # --- pre-flight: target visible? ---
        rgb0 = grab_color_frame(pipe, align)
        T_cm0, info0 = detect(rgb0)
        if T_cm0 is None:
            print(f"[cal] FAIL: target NOT detected in initial frame ({info0}).", file=sys.stderr)
            print( "      check: target visible in cam? lighting? right dict / size?",
                  file=sys.stderr)
            if args.debug_dir is not None:
                cv2.imwrite(str(args.debug_dir / "00_init_FAIL.jpg"), rgb0)
                print(f"      saved frame to {args.debug_dir / '00_init_FAIL.jpg'}", file=sys.stderr)
            sys.exit(2)
        print(f"[cal] target detected ({info0}). distance={np.linalg.norm(T_cm0[:3, 3]) * 100:.1f} cm")
        if args.debug_dir is not None:
            cv2.imwrite(str(args.debug_dir / "00_init_ok.jpg"), rgb0)

        if args.probe_only:
            print("[cal] --probe-only specified, exiting without moving arm.")
            return

        # --- robot ---
        print(f"[cal] connecting to polymetis at {args.robot_host}:{args.robot_port}")
        from polymetis import RobotInterface
        robot = RobotInterface(ip_address=args.robot_host, port=args.robot_port)

        # If a controller is already running (e.g. UI's impedance), stop it.
        try:
            robot.terminate_current_policy()
            time.sleep(0.3)
        except Exception:
            pass

        pos_init_t, quat_init_t = robot.get_ee_pose()
        pos_init = pos_init_t.numpy().astype(np.float64)
        quat_init = quat_init_t.numpy().astype(np.float64)
        T_w_e_init = make_pose(pos_init, quat_init)
        print(f"[cal] initial EE pos (m): {pos_init.round(4)}")
        print(f"[cal] initial EE quat (xyzw): {quat_init.round(4)}")

        # --- collect ---
        samples = [(T_w_e_init.copy(), T_cm0.copy())]
        failed = 0
        import torch
        from scipy.spatial.transform import Rotation as R

        for i in range(args.num_poses):
            T_target = perturb_pose_in_ee(T_w_e_init, args.xyz_range, args.rpy_range, rng)
            pos_t = T_target[:3, 3]
            quat_t = R.from_matrix(T_target[:3, :3]).as_quat()  # xyzw

            try:
                print(f"[cal] {i + 1:02d}/{args.num_poses} → pos={pos_t.round(3)} "
                      f"quat={quat_t.round(3)}", flush=True)
                robot.move_to_ee_pose(
                    position=torch.tensor(pos_t, dtype=torch.float32),
                    orientation=torch.tensor(quat_t, dtype=torch.float32),
                    time_to_go=args.time_per_move,
                )
                time.sleep(args.settle_s)

                pos_meas, quat_meas = robot.get_ee_pose()
                T_w_e = make_pose(pos_meas.numpy().astype(np.float64),
                                  quat_meas.numpy().astype(np.float64))
                rgb = grab_color_frame(pipe, align)
                T_cm, info = detect(rgb)
                if T_cm is None:
                    failed += 1
                    print(f"[cal]   target NOT detected ({info}), skipping")
                    if args.debug_dir is not None:
                        cv2.imwrite(str(args.debug_dir / f"{i + 1:02d}_FAIL.jpg"), rgb)
                    continue
                samples.append((T_w_e, T_cm))
                if args.debug_dir is not None:
                    cv2.imwrite(str(args.debug_dir / f"{i + 1:02d}_ok.jpg"), rgb)
            except KeyboardInterrupt:
                print("\n[cal] interrupted")
                break
            except Exception as e:
                failed += 1
                print(f"[cal]   pose {i + 1} failed: {e!r}")

        # --- return arm to initial pose ---
        print("[cal] returning to initial pose")
        try:
            robot.move_to_ee_pose(
                position=torch.tensor(pos_init, dtype=torch.float32),
                orientation=torch.tensor(quat_init, dtype=torch.float32),
                time_to_go=3.0,
            )
        except Exception as e:
            print(f"[cal] return-to-init failed (non-fatal): {e!r}")
    finally:
        try:
            pipe.stop()
        except Exception:
            pass

    print(f"\n[cal] collected {len(samples)} samples, failed {failed}")
    if len(samples) < 8:
        print("[cal] FAIL: too few valid samples for stable Park solve (need >= 8).",
              file=sys.stderr)
        sys.exit(3)

    # --- save raw samples for re-solving offline ---
    samples_path = args.out.with_suffix(".samples.npz")
    np.savez(
        samples_path,
        T_w_e=np.stack([s[0] for s in samples]),
        T_c_m=np.stack([s[1] for s in samples]),
        K=K,
        dist=dist,
    )
    print(f"[cal] saved raw samples to {samples_path}")

    # --- solve ---
    T_ec = solve_handeye(samples)
    np.save(args.out, T_ec)
    print(f"\n[cal] saved T_ee_camera to {args.out}")
    print(f"T_ee_camera =\n{T_ec}")
    from scipy.spatial.transform import Rotation as R
    print(f"  translation (mm): {(T_ec[:3, 3] * 1000).round(2)}")
    print(f"  rotation (xyz deg): {R.from_matrix(T_ec[:3, :3]).as_euler('xyz', degrees=True).round(2)}")

    # --- validate ---
    _, t_std, e_std = validate(samples, T_ec)
    t_norm = float(np.linalg.norm(t_std))
    e_max = float(e_std.max())
    print(f"\n=== Validation: T_w_marker stability across {len(samples)} frames ===")
    print(f"  translation std (mm):  xyz = {(t_std * 1000).round(2)}, |.| = {t_norm * 1000:.2f}")
    print(f"  rotation std (deg):    xyz = {e_std.round(2)}, max = {e_max:.2f}")
    if t_norm > 0.005:
        print("  WARNING: translation std > 5 mm. Calibration noisy. Consider redo with "
              "more rotation diversity (--rpy-range, --num-poses).")
    if e_max > 2.0:
        print("  WARNING: rotation std > 2°. Calibration noisy.")


if __name__ == "__main__":
    main()
