"""In-process smoke test: load IP model + demo, fake one live obs, run predict_actions.

Uses waypoint 0 of the demo itself as the fake live observation. Confirms the
end-to-end shape contract works without any robot or camera in the loop.
"""

from __future__ import annotations

import argparse
import pickle
import sys
from pathlib import Path

import numpy as np
import torch

# instant_policy/ must be on sys.path; deploy.py imports `utils.transform_pcd`
# from a sibling utils.py, so we run with cwd at instant_policy/ to keep that
# import working. Caller passes --instant-policy-dir.


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--demo", type=Path, required=True)
    ap.add_argument("--checkpoint", type=Path, required=True)
    ap.add_argument("--instant-policy-dir", type=Path, required=True)
    ap.add_argument("--num-diffusion-steps", type=int, default=4)
    args = ap.parse_args()

    sys.path.insert(0, str(args.instant_policy_dir))
    from instant_policy import sample_to_cond_demo, GraphDiffusion  # noqa: E402
    from utils import transform_pcd, subsample_pcd  # noqa: E402

    device = "cuda" if torch.cuda.is_available() else "cpu"
    print(f"device = {device}")

    model = GraphDiffusion.load_from_checkpoint(
        str(args.checkpoint), device=device, strict=True, map_location=device
    )
    model.set_num_demos(1)
    model.set_num_diffusion_steps(args.num_diffusion_steps)
    model.eval()
    print("model loaded")

    with args.demo.open("rb") as f:
        demo = pickle.load(f)
    num_traj_wp = len(demo["pcds"])
    print(f"demo loaded: num_waypoints={num_traj_wp}, "
          f"first pcd shape={demo['pcds'][0].shape}")

    full_sample = {"demos": [dict()], "live": dict()}
    full_sample["demos"][0] = sample_to_cond_demo(demo, num_traj_wp)
    cond_obs_len = len(full_sample["demos"][0]["obs"])
    print(f"cond_obs len = {cond_obs_len}  (expect {num_traj_wp})")
    assert cond_obs_len == num_traj_wp

    pcd_w = demo["pcds"][0]      # (M, 3) base frame
    T_w_e = demo["T_w_es"][0]    # (4, 4)
    grip = demo["grips"][0]
    full_sample["live"]["obs"] = [transform_pcd(subsample_pcd(pcd_w), np.linalg.inv(T_w_e))]
    full_sample["live"]["grips"] = [grip]
    full_sample["live"]["T_w_es"] = [T_w_e]

    print("calling predict_actions ...")
    with torch.no_grad():
        actions, grips = model.predict_actions(full_sample)
    print(f"actions: type={type(actions).__name__}", end="")
    if hasattr(actions, "shape"):
        print(f"  shape={tuple(actions.shape)}  dtype={actions.dtype}")
    else:
        print()
    print(f"grips:   type={type(grips).__name__}", end="")
    if hasattr(grips, "shape"):
        print(f"  shape={tuple(grips.shape)}  dtype={grips.dtype}")
    else:
        print()

    a0 = np.asarray(actions[0]) if hasattr(actions, "__getitem__") else None
    if a0 is not None and a0.shape == (4, 4):
        T_w_next = T_w_e @ a0
        print(f"T_w_next pos = {T_w_next[:3, 3].round(4).tolist()}")
        print(f"current pos  = {T_w_e[:3, 3].round(4).tolist()}")

    print("OK")


if __name__ == "__main__":
    main()
