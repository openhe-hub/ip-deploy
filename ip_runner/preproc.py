"""Streaming preprocessing for IP: GroundingDINO once + SAM2 image per frame.

Borrows the GD invocation pattern from dexycb_pipeline's
`tools/phase3/run_real_recording_gd_sam2.py` (NMS, post-processing, label
filtering) and replaces the SAM2 video predictor with `SAM2ImagePredictor` so
masks are produced from a single live (color, depth) at robot rate.

Reuses `dexycb_pipeline.object_pcd.lift_mask` for mask + depth + K -> camera
frame point cloud, then applies T_base_camera to land in the base frame.

Public API
----------
GroundingDinoOnce
    load(...).detect(rgb, prompt, ...) -> list of (label, score, xyxy) boxes.

SAM2ImageMasker
    load(...).step(rgb, boxes_xyxy) -> list[np.ndarray bool (H,W)] per object.

obs_to_pcd_w(rgb, depth_mm, K, masks, T_base_camera) -> np.ndarray (N, 3)
    Lift each mask, depth-filter, union, transform into base frame.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

import numpy as np
import torch

# These imports succeed lazily — server constructs the loaders once at startup.
# We don't import dexycb_pipeline.object_pcd at module top because it would
# require the user to set PYTHONPATH for ip_runner alone. The server adds it.


@dataclass
class GDBox:
    label: str
    score: float
    xyxy: np.ndarray  # (4,) float32 [x1, y1, x2, y2] in pixels


class GroundingDinoOnce:
    """Run GroundingDINO once at episode start, cache boxes for the rest."""

    def __init__(self, model_id: str = "IDEA-Research/grounding-dino-tiny",
                 device: str = "cuda"):
        from transformers import AutoModelForZeroShotObjectDetection, AutoProcessor

        self.device = torch.device(device if torch.cuda.is_available() else "cpu")
        self.processor = AutoProcessor.from_pretrained(model_id)
        self.model = (
            AutoModelForZeroShotObjectDetection.from_pretrained(model_id)
            .to(self.device)
            .eval()
        )

    @torch.no_grad()
    def detect(self, rgb: np.ndarray, prompt: str,
               box_threshold: float = 0.18,
               text_threshold: float = 0.18,
               nms_iou: float = 0.5,
               containment_threshold: float = 0.85,
               min_area: float = 0.0,
               max_area: float = float("inf")) -> list[GDBox]:
        from PIL import Image
        if rgb.ndim != 3 or rgb.shape[2] != 3:
            raise ValueError(f"rgb must be HxWx3, got {rgb.shape}")
        pil = Image.fromarray(rgb)
        inputs = self.processor(images=pil, text=prompt,
                                return_tensors="pt").to(self.device)
        outputs = self.model(**inputs)
        results = self.processor.post_process_grounded_object_detection(
            outputs,
            inputs.input_ids,
            box_threshold=box_threshold,
            text_threshold=text_threshold,
            target_sizes=[pil.size[::-1]],
        )[0]
        boxes = results["boxes"].detach().cpu().numpy().astype(np.float32)
        scores = results["scores"].detach().cpu().numpy().astype(np.float32)
        labels_src = (results.get("text_labels")
                      or results.get("labels")
                      or [""] * len(boxes))
        labels = [str(x) for x in labels_src]

        keep = _nms(boxes, scores, nms_iou, containment_threshold,
                    min_area, max_area)
        return [GDBox(labels[i], float(scores[i]), boxes[i].copy()) for i in keep]


def _nms(boxes: np.ndarray, scores: np.ndarray, iou_thresh: float,
         contain_thresh: float, min_area: float, max_area: float) -> list[int]:
    """Greedy NMS with containment-aware suppression. Mirrors the dexycb impl."""
    order = scores.argsort()[::-1]
    keep: list[int] = []
    for i in order.tolist():
        a = boxes[i]
        area_a = max(0.0, a[2] - a[0]) * max(0.0, a[3] - a[1])
        if area_a < min_area or area_a > max_area:
            continue
        suppressed = False
        for j in keep:
            b = boxes[j]
            ix1, iy1 = max(a[0], b[0]), max(a[1], b[1])
            ix2, iy2 = min(a[2], b[2]), min(a[3], b[3])
            inter = max(0.0, ix2 - ix1) * max(0.0, iy2 - iy1)
            area_b = max(0.0, b[2] - b[0]) * max(0.0, b[3] - b[1])
            union = area_a + area_b - inter
            iou = inter / union if union > 0 else 0.0
            cont = inter / area_a if area_a > 0 else 0.0
            if iou > iou_thresh or cont > contain_thresh:
                suppressed = True
                break
        if not suppressed:
            keep.append(i)
    return keep


class SAM2ImageMasker:
    """Per-frame SAM2 image predictor; takes pre-cached boxes per object."""

    def __init__(self, sam2_root: Path, ckpt: str = "sam2.1_hiera_large.pt",
                 cfg: str = "configs/sam2.1/sam2.1_hiera_l.yaml",
                 device: str = "cuda"):
        import sys as _sys
        _sys.path.insert(0, str(sam2_root))
        from sam2.build_sam import build_sam2
        from sam2.sam2_image_predictor import SAM2ImagePredictor

        self.device = device if torch.cuda.is_available() else "cpu"
        ckpt_path = str(Path(sam2_root) / "checkpoints" / ckpt)
        sam2_model = build_sam2(cfg, ckpt_path, device=self.device)
        self.predictor = SAM2ImagePredictor(sam2_model)

    @torch.no_grad()
    def step(self, rgb: np.ndarray, boxes_xyxy: Sequence[np.ndarray]
             ) -> list[np.ndarray]:
        """Returns list of (H, W) bool masks, one per box."""
        if rgb.ndim != 3 or rgb.shape[2] != 3:
            raise ValueError(f"rgb must be HxWx3, got {rgb.shape}")
        self.predictor.set_image(rgb)
        masks_out: list[np.ndarray] = []
        for box in boxes_xyxy:
            box_arr = np.asarray(box, dtype=np.float32).reshape(1, 4)
            masks, _, _ = self.predictor.predict(
                box=box_arr, multimask_output=False
            )
            # masks shape: (1, H, W) float / bool depending on version
            m = np.asarray(masks[0]) > 0
            masks_out.append(m)
        return masks_out


def obs_to_pcd_w(depth_mm: np.ndarray,
                 K: tuple,
                 masks: Sequence[np.ndarray],
                 T_base_camera: np.ndarray,
                 depth_filter_mm: float = 120.0) -> np.ndarray:
    """Union all object masks, lift to camera frame, transform to base frame.

    Returns (N, 3) float32 in base frame. May be empty if all masks are blank.
    """
    from dexycb_pipeline.object_pcd import lift_mask, filter_by_depth

    all_pts = []
    for m in masks:
        if m is None:
            continue
        m = np.asarray(m, dtype=bool)
        pcd_cam = lift_mask(depth_mm, m, K)
        if depth_filter_mm > 0 and len(pcd_cam):
            pcd_cam = filter_by_depth(pcd_cam, depth_filter_mm)
        if len(pcd_cam):
            all_pts.append(pcd_cam)

    if not all_pts:
        return np.zeros((0, 3), dtype=np.float32)
    pcd_cam = np.concatenate(all_pts, axis=0)

    # base = T_base_camera @ [pcd_cam ; 1]
    R = T_base_camera[:3, :3].astype(np.float32)
    t = T_base_camera[:3, 3].astype(np.float32)
    pcd_base = (R @ pcd_cam.T).T + t
    return pcd_base.astype(np.float32, copy=False)
