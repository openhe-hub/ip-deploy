"""Persist the provisional 3.3cm-tag T_base_camera as npy.

This matrix is the single source of truth on both demo and live paths. Source:
docs/ip-demo.md (translation residual ~1.02cm; visualization/test only).
"""

from __future__ import annotations

from pathlib import Path

import numpy as np

T_BASE_CAMERA = np.array([
    [-0.0443204970,  0.1476216871, -0.9880503687, 1.1877870404],
    [ 0.9978588566, -0.0410750485, -0.0508973733, 0.1634945834],
    [-0.0480977729, -0.9881906081, -0.1454851413, 0.4333322336],
    [ 0.0,           0.0,           0.0,          1.0          ],
], dtype=np.float64)


def main():
    out = Path(__file__).parent / "T_base_camera_3.3cm.npy"
    np.save(out, T_BASE_CAMERA)
    print(f"saved {out}")
    print(T_BASE_CAMERA)


if __name__ == "__main__":
    main()
