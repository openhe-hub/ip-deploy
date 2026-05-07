"""Franka NUC IP executor.

Connects to the IP inference server on nyu-127 and applies returned EE pose
targets via polymetis Cartesian impedance + the gripper.

Modes:
    --dry-run               do everything except call update_desired_ee_pose /
                            gripper. Prints what it would have sent.
    --max-steps N           stop after N OBS/ACT roundtrips (default 200).

Safety:
    - leash on every applied target: max 4cm / 17 deg ahead of measured pose.
    - SIGINT (Ctrl-C) cleanly stops the impedance controller.
    - if --enable-gripper is not set, the gripper is never commanded.
"""

from __future__ import annotations

import argparse
import os
import signal
import sys
import time
from pathlib import Path

import cv2
import numpy as np

# --- Imports we'll only need on the NUC. Wrapped so the module imports for ---
# --- syntax-check on the dev machine; runtime guards check availability.   ---
try:
    import torch
except Exception:
    torch = None  # type: ignore

# Local sibling imports
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))  # /home/franka/ICRT
from ip_executor.client import IPClient
from ip_executor.safety import leash


def _load_polymetis():
    from polymetis import RobotInterface, GripperInterface
    return RobotInterface, GripperInterface


def _load_realsense(serial: str, width: int = 640, height: int = 480,
                    fps: int = 6):
    """Return (cam, read_fn, K_tuple). read_fn() -> (color_bgr, depth_uint16_mm).

    Mirrors the API in ICRT/show_rgbd.py: RealSenseCamera starts the pipeline
    in __init__, exposes poll() and intrinsics_dict() (no separate start()).
    """
    from show_rgbd import RealSenseCamera
    cam = RealSenseCamera(serial=serial, name=serial,
                          width=width, height=height, fps=fps)
    intr = cam.intrinsics_dict()
    fx, fy = float(intr["fx"]), float(intr["fy"])
    cx, cy = float(intr["cx"]), float(intr["cy"])
    depth_scale = float(cam.depth_scale)  # meters per raw unit

    def read():
        # poll() can return (None, None) for the first few frames while the
        # sensor warms up; retry a handful of times.
        for _ in range(50):
            color_bgr, depth_raw = cam.poll()
            if color_bgr is not None and depth_raw is not None:
                # Z16 is uint16 with `depth_scale` m/unit. Convert to mm so
                # the server's lift_mask (which expects mm) is right for any
                # scale variant. For D435/D435I scale=0.001 this is identity.
                depth_mm = (depth_raw.astype(np.float32) * depth_scale * 1000.0)
                depth_mm = depth_mm.astype(np.uint16)
                return color_bgr, depth_mm
            time.sleep(0.02)
        raise RuntimeError("RealSense did not produce a frame within ~1s")

    return cam, read, (fx, fy, cx, cy)


def _ee_pose_from_polymetis(robot) -> tuple[np.ndarray, np.ndarray]:
    """Return (pos[3], quat_xyzw[4]) in base frame as float32."""
    pos, quat = robot.get_ee_pose()  # tensors
    pos = np.asarray(pos.cpu(), dtype=np.float32).reshape(3)
    quat = np.asarray(quat.cpu(), dtype=np.float32).reshape(4)
    return pos, quat


def _T_from_pos_quat(pos: np.ndarray, quat_xyzw: np.ndarray) -> np.ndarray:
    from ip_executor.safety import quat_xyzw_to_R
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = quat_xyzw_to_R(quat_xyzw)
    T[:3, 3] = pos
    return T


def _send_target(robot, pos: np.ndarray, quat_xyzw: np.ndarray):
    pos_t = torch.from_numpy(np.asarray(pos, dtype=np.float32))
    quat_t = torch.from_numpy(np.asarray(quat_xyzw, dtype=np.float32))
    robot.update_desired_ee_pose(position=pos_t, orientation=quat_t)


def _send_gripper(gripper, grip_cmd: int, last_grip_state: int,
                  open_width_m: float = 0.08, close_width_m: float = 0.0,
                  speed: float = 0.1, force: float = 30.0) -> int:
    """Edge-triggered: only act when grip_cmd flips. Returns new grip_state."""
    if grip_cmd == 0:
        return last_grip_state
    # +1 open, -1 close
    desired = 1 if grip_cmd > 0 else 0
    if desired == last_grip_state:
        return last_grip_state
    if desired == 1:
        gripper.goto(width=open_width_m, speed=speed, force=force)
    else:
        gripper.grasp(speed=speed, force=force, grasp_width=close_width_m)
    return desired


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--server", default="tcp://10.224.36.127:5556")
    ap.add_argument("--prompt", required=True,
                    help='GroundingDINO prompt, e.g. "yellow glue stick . wooden cube ."')
    ap.add_argument("--camera-serial", default="925622071356")
    ap.add_argument("--max-steps", type=int, default=200)
    ap.add_argument("--dry-run", action="store_true",
                    help="do not call polymetis update_desired_ee_pose / gripper")
    ap.add_argument("--enable-gripper", action="store_true",
                    help="also command the gripper (off by default for safety)")
    ap.add_argument("--gripper-threshold-m", type=float, default=0.04,
                    help="width > threshold -> grip=1 (open) sent to server")
    ap.add_argument("--max-pos-step-m", type=float, default=0.04)
    ap.add_argument("--max-rot-step-deg", type=float, default=17.0)
    ap.add_argument("--gd-box-threshold", type=float, default=0.18)
    ap.add_argument("--gd-text-threshold", type=float, default=0.18)
    ap.add_argument("--robot-host", default="localhost")
    ap.add_argument("--robot-port", type=int, default=50051)
    ap.add_argument("--gripper-host", default="localhost")
    ap.add_argument("--gripper-port", type=int, default=50052)
    ap.add_argument("--no-polymetis", action="store_true",
                    help="skip polymetis entirely (uses identity pose). For "
                         "camera+server smoke when polymetis isn't running.")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--fps", type=int, default=6)
    ap.add_argument("--step-sleep", type=float, default=0.05,
                    help="seconds to sleep between non-regrasp steps. Bump to "
                         "2-5s for slow-motion live debugging.")
    ap.add_argument("--regrasp-sleep", type=float, default=0.5,
                    help="seconds to sleep on a grip-flip step.")
    args = ap.parse_args()

    print(f"[exec] server={args.server} dry_run={args.dry_run}"
          f" enable_gripper={args.enable_gripper}", flush=True)

    # 1) connect to robot
    robot = None
    gripper = None
    if not args.no_polymetis:
        try:
            RobotInterface, GripperInterface = _load_polymetis()
            robot = RobotInterface(ip_address=args.robot_host, port=args.robot_port)
            # Always construct GripperInterface so we can READ width.
            # --enable-gripper still gates writes (goto/grasp).
            try:
                gripper = GripperInterface(ip_address=args.gripper_host,
                                           port=args.gripper_port)
            except Exception as ge:
                print(f"[exec] gripper read unavailable: {ge}", flush=True)
                gripper = None
            if not args.dry_run:
                robot.start_cartesian_impedance()
                print("[exec] cartesian impedance started", flush=True)
        except Exception as e:
            if not args.dry_run:
                print(f"[exec] polymetis required but failed: {e}", file=sys.stderr)
                raise
            print(f"[exec] polymetis unavailable, falling back to identity pose ({e})",
                  flush=True)
            robot = None

    # 2) camera
    cam, read_frame, K = _load_realsense(
        args.camera_serial, width=args.width, height=args.height, fps=args.fps)
    print(f"[exec] RealSense {args.camera_serial} ready  K={K}", flush=True)

    # 3) IP client
    client = IPClient(args.server)
    print("[exec] connected to server, sending RESET ...", flush=True)
    ack = client.reset(args.prompt,
                       gd_box_threshold=args.gd_box_threshold,
                       gd_text_threshold=args.gd_text_threshold)
    print(f"[exec] RESET ack: {ack}", flush=True)

    # 4) signal handling
    stop = {"flag": False}
    def _stop(*_):
        stop["flag"] = True
    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    last_grip_state = 1  # start assuming gripper open
    max_rot_step_rad = np.deg2rad(args.max_rot_step_deg)

    try:
        for step_i in range(args.max_steps):
            if stop["flag"]:
                print("[exec] stop signal received", flush=True)
                break

            color_bgr, depth = read_frame()

            if robot is not None:
                pos_meas, quat_meas = _ee_pose_from_polymetis(robot)
                if gripper is not None:
                    g_state = gripper.get_state()
                    width_m = float(getattr(g_state, "width", 0.08))
                else:
                    width_m = 0.08 if last_grip_state == 1 else 0.0
            else:
                pos_meas = np.zeros(3, dtype=np.float32)
                quat_meas = np.array([0, 0, 0, 1], dtype=np.float32)
                width_m = 0.08

            T_w_e = _T_from_pos_quat(pos_meas, quat_meas)
            grip_in = 1 if width_m > args.gripper_threshold_m else 0

            try:
                act = client.step(color_bgr, depth, K, T_w_e, grip_in)
            except Exception as e:
                print(f"[exec] step {step_i}: client error: {e}", flush=True)
                break

            tgt_pos, tgt_quat = leash(
                pos_meas, quat_meas,
                act.target_pos.astype(np.float32),
                act.target_quat_xyzw.astype(np.float32),
                max_pos_step_m=args.max_pos_step_m,
                max_rot_step_rad=max_rot_step_rad,
            )

            print(f"[exec] step={act.step:>3}"
                  f" pos {pos_meas.round(3).tolist()} -> {tgt_pos.round(3).tolist()}"
                  f"  grip_in={grip_in} grip_cmd={act.grip_cmd}"
                  f"{' [REGRASP]' if act.regrasp_required else ''}"
                  f"  rtt={act.info.get('rtt_ms', -1):.0f}ms",
                  flush=True)

            if args.dry_run:
                continue

            _send_target(robot, tgt_pos, tgt_quat)
            if args.enable_gripper and gripper is not None:
                last_grip_state = _send_gripper(gripper, act.grip_cmd, last_grip_state)

            # IP recommends re-querying when grip flips; otherwise loop pace
            if act.regrasp_required:
                time.sleep(args.regrasp_sleep)
            else:
                time.sleep(args.step_sleep)
    finally:
        try:
            if not args.dry_run and robot is not None:
                robot.terminate_current_policy()
        except Exception as e:
            print(f"[exec] terminate_current_policy failed: {e}", flush=True)
        try:
            cam.stop()
        except Exception:
            pass
        client.close()
        print("[exec] shutdown complete", flush=True)


if __name__ == "__main__":
    # Help OS schedule camera + ZMQ on different cores; not required.
    os.environ.setdefault("OMP_NUM_THREADS", "2")
    main()
