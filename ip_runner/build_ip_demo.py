"""Build an IP demo dict from a dexycb_pipeline `object_pointclouds_ee.npz`.

Output schema (pickled):
    {"pcds":   [np.ndarray(M_i, 3) float32 in base frame, len = num_waypoints],
     "T_w_es": [np.ndarray(4, 4) float64,                  len = num_waypoints],
     "grips":  [int  (0=closed, 1=open),                   len = num_waypoints]}

The point cloud is the union of all per-object segmented points already in base
frame (`points_base` in the npz). Invalid (NaN-padded) slots are dropped via
`pcd_valid`. Gripper width is read from `poses.csv`, linearly interpolated to
the camera-frame timestamps in the npz, then thresholded.
"""

from __future__ import annotations

import argparse
import pickle
from pathlib import Path

import numpy as np
import pandas as pd


def load_gripper_at_frame_ts(poses_csv: Path, frame_ts: np.ndarray) -> np.ndarray:
    df = pd.read_csv(poses_csv)
    return np.interp(frame_ts, df["ts"].values, df["gripper_width"].values)


def gather_scene_pcd(points_base: np.ndarray, pcd_valid: np.ndarray) -> np.ndarray:
    """points_base: (K_obj, P, 3); pcd_valid: (K_obj, P) -> (M, 3) base frame."""
    flat_pts = points_base.reshape(-1, 3)
    flat_valid = pcd_valid.reshape(-1)
    return flat_pts[flat_valid].astype(np.float32, copy=False)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--recording", type=Path, required=True,
                    help="recording dir containing poses.csv")
    ap.add_argument("--pcd-npz", type=Path, required=True,
                    help="object_pointclouds_ee.npz from dexycb_pipeline")
    ap.add_argument("--out", type=Path, required=True,
                    help="output pickle path")
    ap.add_argument("--num-waypoints", type=int, default=10)
    ap.add_argument("--gripper-threshold-m", type=float, default=0.04,
                    help="width > threshold => grip=1 (open)")
    args = ap.parse_args()

    npz = np.load(args.pcd_npz, allow_pickle=True)
    points_base = npz["points_base"]    # (T, K, P, 3) float32
    pcd_valid = npz["pcd_valid"]        # (T, K, P) bool
    T_base_ee = npz["T_base_ee"]        # (T, 4, 4) float64
    frame_ts = npz["frame_ts"]          # (T,) float64
    T = points_base.shape[0]

    poses_csv = args.recording / "poses.csv"
    grip_widths = load_gripper_at_frame_ts(poses_csv, frame_ts)
    grips_full = (grip_widths > args.gripper_threshold_m).astype(np.int64)

    idxs = np.linspace(0, T - 1, args.num_waypoints).astype(int)
    pcds = [gather_scene_pcd(points_base[i], pcd_valid[i]) for i in idxs]
    T_w_es = [T_base_ee[i].astype(np.float64) for i in idxs]
    grips = [int(grips_full[i]) for i in idxs]

    args.out.parent.mkdir(parents=True, exist_ok=True)
    with args.out.open("wb") as f:
        pickle.dump({"pcds": pcds, "T_w_es": T_w_es, "grips": grips}, f)

    point_counts = [p.shape[0] for p in pcds]
    print(f"saved {args.out}")
    print(f"  num_waypoints = {len(pcds)}")
    print(f"  picked frames = {idxs.tolist()}")
    print(f"  pcd point counts = {point_counts}")
    print(f"  grips = {grips}")
    print(f"  grip_widths = {[f'{grip_widths[i]:.4f}' for i in idxs]}")


if __name__ == "__main__":
    main()
