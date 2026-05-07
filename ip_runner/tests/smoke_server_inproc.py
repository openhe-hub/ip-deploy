"""In-process smoke test of the streaming server: load model+demo, take one
real frame from a recording, build OBS, run handler, print result.

This skips ZMQ entirely so failures are easy to localize.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import pandas as pd


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--demo", type=Path, required=True)
    ap.add_argument("--checkpoint", type=Path, required=True)
    ap.add_argument("--instant-policy-dir", type=Path, required=True)
    ap.add_argument("--T-base-camera", type=Path, required=True,
                    dest="T_base_camera")
    ap.add_argument("--sam2-root", type=Path,
                    default=Path("/home/nyuair/zhewen/sam2"))
    ap.add_argument("--sam2-ckpt", default="sam2.1_hiera_large.pt")
    ap.add_argument("--sam2-cfg",
                    default="configs/sam2.1/sam2.1_hiera_l.yaml")
    ap.add_argument("--gd-model", default="IDEA-Research/grounding-dino-tiny")
    ap.add_argument("--recording", type=Path, required=True,
                    help="path to recording_<ts>_full dir")
    ap.add_argument("--camera", default="925622071356")
    ap.add_argument("--frame", type=int, default=0,
                    help="which frame index in the recording")
    ap.add_argument("--prompt", required=True)
    ap.add_argument("--num-demos", type=int, default=1)
    ap.add_argument("--num-diffusion-steps", type=int, default=4)
    ap.add_argument("--depth-filter-mm", type=float, default=120.0)
    ap.add_argument("--min-area", type=float, default=400.0)
    ap.add_argument("--max-area", type=float, default=120000.0)
    ap.add_argument("--min-points", type=int, default=64)
    ap.add_argument("--gripper-threshold-m", type=float, default=0.04)
    args = ap.parse_args()

    # Avoid pulling in zmq for in-process test
    sys.path.insert(0, "/home/nyuair/zhewen/robo/dexycb_pipeline")
    from ip_runner.server import Server  # noqa: E402

    # Build a Server with shimmed args (no .bind needed for in-proc)
    class _A:
        pass
    sa = _A()
    for k in ("demo", "checkpoint", "instant_policy_dir", "T_base_camera",
              "sam2_root", "sam2_ckpt", "sam2_cfg", "gd_model",
              "num_demos", "num_diffusion_steps",
              "depth_filter_mm", "min_area", "max_area", "min_points"):
        setattr(sa, k, getattr(args, k))
    sa.bind = "tcp://*:65530"  # placeholder, will be closed below

    server = Server(sa)
    server.sock.close()  # we won't use ZMQ in this test

    # Load the recording frame
    cam_dir = args.recording / f"cam_{args.camera}"
    color_path = cam_dir / f"color_{args.frame:06d}.png"
    depth_path = cam_dir / f"depth_{args.frame:06d}.png"
    color_bgr = cv2.imread(str(color_path), cv2.IMREAD_COLOR)
    depth = cv2.imread(str(depth_path), cv2.IMREAD_UNCHANGED)
    assert color_bgr is not None and depth is not None, \
        f"failed to load frame {args.frame} from {cam_dir}"

    # K from metadata
    meta = json.loads((args.recording / "metadata.json").read_text())
    cam_meta = next(c for c in meta["cameras"] if str(c["serial"]) == args.camera)
    intr = cam_meta["intrinsics"]
    fx, fy = float(intr["fx"]), float(intr["fy"])
    cx, cy = float(intr["cx"]), float(intr["cy"])

    # Match frame ts -> pose -> gripper
    frames = pd.read_csv(cam_dir / "frames.csv")
    frame_ts = float(frames.loc[frames["frame_idx"] == args.frame, "ts"].iloc[0])
    poses = pd.read_csv(args.recording / "poses.csv")
    pose_row = poses.iloc[(poses["ts"] - frame_ts).abs().idxmin()]
    pos = np.array([pose_row[c] for c in ("ee_x", "ee_y", "ee_z")], dtype=np.float64)
    quat_xyzw = np.array(
        [pose_row[c] for c in ("ee_qx", "ee_qy", "ee_qz", "ee_qw")],
        dtype=np.float64)
    grip_w = float(pose_row["gripper_width"])
    grip = 1 if grip_w > args.gripper_threshold_m else 0

    T_w_e = np.eye(4, dtype=np.float64)
    T_w_e[:3, :3] = _quat_xyzw_to_R(quat_xyzw)
    T_w_e[:3, 3] = pos

    # Encode like the NUC would
    ok, color_jpeg = cv2.imencode(".jpg", color_bgr,
                                   [int(cv2.IMWRITE_JPEG_QUALITY), 90])
    ok2, depth_png = cv2.imencode(".png", depth)
    assert ok and ok2

    # Reset
    print("--- RESET ---", flush=True)
    print(server._handle_reset({
        "type": "reset",
        "episode_id": "smoke-1",
        "prompt": args.prompt,
        "gd_box_threshold": 0.18,
        "gd_text_threshold": 0.18,
    }))

    print("--- OBS ---", flush=True)
    obs_msg = {
        "type": "obs",
        "episode_id": "smoke-1",
        "step": int(args.frame),
        "color_bgr_jpeg": bytes(color_jpeg),
        "depth_uint16_png": bytes(depth_png),
        "K": (fx, fy, cx, cy),
        "T_w_e": T_w_e,
        "grip": grip,
    }
    t0 = time.time()
    reply = server._handle_obs(obs_msg)
    dt = (time.time() - t0) * 1e3
    print(f"OBS handled in {dt:.0f}ms")

    if reply["type"] != "act":
        print("FAIL:", reply)
        sys.exit(1)
    print(f"target_pos          = {reply['target_pos'].round(4).tolist()}")
    print(f"current_pos         = {T_w_e[:3,3].round(4).tolist()}")
    print(f"target_quat_xyzw    = {reply['target_quat_xyzw'].round(4).tolist()}")
    print(f"grip_cmd            = {reply['grip_cmd']}")
    print(f"regrasp_required    = {reply['regrasp_required']}")
    print(f"info                = {reply['info']}")
    print(f"detected boxes      = {reply['boxes']}")

    # Steady-state: re-send same frame a few times. GD won't rerun. IP
    # warm-up should fade after the second call.
    print("--- steady-state (5x same frame) ---", flush=True)
    for i in range(5):
        obs_msg["step"] = int(args.frame) + 1 + i
        t0 = time.time()
        r = server._handle_obs(obs_msg)
        dt = (time.time() - t0) * 1e3
        if r["type"] != "act":
            print("FAIL:", r); sys.exit(1)
        info = r["info"]
        print(f"  step={r['step']:>3} sam2={info['t_sam2_ms']:.0f}ms"
              f" pcd={info['t_pcd_ms']:.0f}ms"
              f" ip={info['t_ip_ms']:.0f}ms"
              f" total={info['t_total_ms']:.0f}ms")
    print("OK")


def _quat_xyzw_to_R(q: np.ndarray) -> np.ndarray:
    x, y, z, w = q
    n = x*x + y*y + z*z + w*w
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    return np.array([
        [1 - s*(y*y+z*z),     s*(x*y-z*w),       s*(x*z+y*w)],
        [    s*(x*y+z*w), 1 - s*(x*x+z*z),       s*(y*z-x*w)],
        [    s*(x*z-y*w),     s*(y*z+x*w),   1 - s*(x*x+y*y)],
    ])


if __name__ == "__main__":
    main()
