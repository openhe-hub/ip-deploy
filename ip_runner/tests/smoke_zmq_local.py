"""ZMQ-based smoke test simulating the NUC: bind server in-proc-thread, send
RESET + a few OBS via REQ, verify ACT round-trips. Runs entirely on nyu-127.

This validates the full wire format and codec without any robot/camera hardware.
"""

from __future__ import annotations

import argparse
import json
import sys
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import pandas as pd
import zmq

from ip_runner import codec
from ip_runner.server import Server


class _ServerThread(threading.Thread):
    def __init__(self, args):
        super().__init__(daemon=True)
        self.args = args
        self.ready = threading.Event()
        self.stop_evt = threading.Event()
        self.server: Server | None = None

    def run(self):
        self.server = Server(self.args)
        self.ready.set()
        try:
            self.server.serve()
        except SystemExit:
            pass


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
    ap.add_argument("--recording", type=Path, required=True)
    ap.add_argument("--camera", default="925622071356")
    ap.add_argument("--prompt", required=True)
    ap.add_argument("--bind", default="tcp://127.0.0.1:5557",
                    help="loopback for in-proc smoke")
    ap.add_argument("--num-demos", type=int, default=1)
    ap.add_argument("--num-diffusion-steps", type=int, default=4)
    ap.add_argument("--depth-filter-mm", type=float, default=120.0)
    ap.add_argument("--min-area", type=float, default=400.0)
    ap.add_argument("--max-area", type=float, default=120000.0)
    ap.add_argument("--min-points", type=int, default=64)
    ap.add_argument("--gripper-threshold-m", type=float, default=0.04)
    ap.add_argument("--steps", type=int, default=4)
    args = ap.parse_args()

    sys.path.insert(0, "/home/nyuair/zhewen/robo/dexycb_pipeline")

    # Spin up server in a background thread bound to loopback
    srv = _ServerThread(args)
    srv.start()
    print("[smoke] waiting for server ready ...", flush=True)
    srv.ready.wait(timeout=600)
    print("[smoke] server ready", flush=True)
    time.sleep(0.5)

    ctx = zmq.Context.instance()
    sock = ctx.socket(zmq.REQ)
    sock.setsockopt(zmq.LINGER, 0)
    sock.setsockopt(zmq.RCVTIMEO, 60_000)
    sock.connect(args.bind)

    # Reset
    sock.send(codec.encode({
        "type": "reset",
        "episode_id": "smoke-zmq",
        "prompt": args.prompt,
        "gd_box_threshold": 0.18,
        "gd_text_threshold": 0.18,
    }))
    ack = codec.decode(sock.recv())
    print(f"[smoke] RESET ack: {ack}")
    assert ack["type"] == "ack" and ack["ok"]

    # Frame loader
    cam_dir = args.recording / f"cam_{args.camera}"
    meta = json.loads((args.recording / "metadata.json").read_text())
    cam_meta = next(c for c in meta["cameras"] if str(c["serial"]) == args.camera)
    intr = cam_meta["intrinsics"]
    K = (float(intr["fx"]), float(intr["fy"]),
         float(intr["cx"]), float(intr["cy"]))
    frames_csv = pd.read_csv(cam_dir / "frames.csv")
    poses = pd.read_csv(args.recording / "poses.csv")

    def load_obs_for_frame(fi: int) -> dict:
        cf = cam_dir / f"color_{fi:06d}.png"
        df = cam_dir / f"depth_{fi:06d}.png"
        color_bgr = cv2.imread(str(cf), cv2.IMREAD_COLOR)
        depth = cv2.imread(str(df), cv2.IMREAD_UNCHANGED)
        ts = float(frames_csv.loc[frames_csv["frame_idx"] == fi, "ts"].iloc[0])
        pose_row = poses.iloc[(poses["ts"] - ts).abs().idxmin()]
        pos = np.array([pose_row[c] for c in ("ee_x", "ee_y", "ee_z")],
                       dtype=np.float64)
        from ip_executor.safety import quat_xyzw_to_R
        q = np.array([pose_row[c] for c in ("ee_qx", "ee_qy", "ee_qz", "ee_qw")],
                     dtype=np.float64)
        T = np.eye(4); T[:3, :3] = quat_xyzw_to_R(q); T[:3, 3] = pos
        grip = 1 if float(pose_row["gripper_width"]) > args.gripper_threshold_m else 0
        ok1, jpeg = cv2.imencode(".jpg", color_bgr,
                                 [int(cv2.IMWRITE_JPEG_QUALITY), 90])
        ok2, png = cv2.imencode(".png", depth)
        return {
            "type": "obs",
            "episode_id": "smoke-zmq",
            "step": fi,
            "color_bgr_jpeg": bytes(jpeg),
            "depth_uint16_png": bytes(png),
            "K": K,
            "T_w_e": T,
            "grip": grip,
        }

    rtts = []
    for i, fi in enumerate(range(0, args.steps * 10, 10)):
        msg = load_obs_for_frame(int(fi))
        t0 = time.time()
        sock.send(codec.encode(msg))
        rep = codec.decode(sock.recv())
        rtt = (time.time() - t0) * 1e3
        rtts.append(rtt)
        if rep["type"] != "act":
            print(f"[smoke] FAIL step {fi}: {rep}", flush=True)
            break
        info = rep.get("info", {})
        print(f"[smoke] step={rep['step']:>3} rtt={rtt:.0f}ms"
              f" total={info.get('t_total_ms', 0):.0f}ms"
              f" sam2={info.get('t_sam2_ms', 0):.0f}ms"
              f" ip={info.get('t_ip_ms', 0):.0f}ms"
              f" pcd={info.get('pcd_n_points', '?'):>5}pts"
              f" grip_cmd={rep['grip_cmd']}",
              flush=True)

    if rtts:
        print(f"\n[smoke] mean RTT (incl. first slow): {np.mean(rtts):.0f}ms")
        print(f"[smoke] mean RTT (skip first):       {np.mean(rtts[1:]):.0f}ms"
              if len(rtts) > 1 else "")
    print("OK")


if __name__ == "__main__":
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))  # ip-deploy/
    main()
