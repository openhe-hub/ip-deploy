"""Sanity-check an IP demo pickle."""

from __future__ import annotations

import argparse
import pickle
import sys
from pathlib import Path

import numpy as np


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("path", type=Path)
    args = ap.parse_args()

    with args.path.open("rb") as f:
        demo = pickle.load(f)

    keys = sorted(demo.keys())
    expected = ["T_w_es", "grips", "pcds"]
    assert keys == expected, f"unexpected keys: {keys}"

    n = len(demo["pcds"])
    assert len(demo["T_w_es"]) == n and len(demo["grips"]) == n, "length mismatch"

    print(f"num_waypoints = {n}")
    for i, (pcd, T, g) in enumerate(zip(demo["pcds"], demo["T_w_es"], demo["grips"])):
        assert pcd.ndim == 2 and pcd.shape[1] == 3, f"pcd[{i}] bad shape {pcd.shape}"
        assert T.shape == (4, 4), f"T_w_es[{i}] bad shape {T.shape}"
        assert g in (0, 1), f"grips[{i}] not in {{0,1}}: {g}"
        # T_w_e last row should be [0,0,0,1]
        assert np.allclose(T[3], [0, 0, 0, 1], atol=1e-6), f"T_w_es[{i}] bad bottom row"
        print(f"  wp {i}: pcd={pcd.shape[0]:>5}pts  pos={T[:3,3].round(3).tolist()}  grip={g}")

    grip_arr = np.array(demo["grips"])
    transitions = int(np.abs(np.diff(grip_arr)).sum())
    print(f"grip transitions = {transitions}  (expect >=1 if recording covers a grasp)")

    pos = np.stack([T[:3, 3] for T in demo["T_w_es"]])
    step_disp = np.linalg.norm(np.diff(pos, axis=0), axis=1)
    print(f"per-step EE displacement (m): max={step_disp.max():.4f} mean={step_disp.mean():.4f}")
    if step_disp.max() > 0.5:
        print("WARNING: large EE jump between waypoints — check pose source", file=sys.stderr)


if __name__ == "__main__":
    main()
