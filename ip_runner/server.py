"""IP inference server. ZMQ REP at tcp://*:<port>.

Lifecycle per episode:
    NUC sends RESET    -> server arms (resets state, defers GD until first OBS)
    NUC sends OBS #0   -> server runs GD on rgb0, caches boxes, runs SAM2,
                          builds pcd_w, calls IP, returns ACT
    NUC sends OBS #1+  -> server runs SAM2 only (boxes cached), pcd_w, IP, ACT

The first ACT after RESET also includes the detected boxes via an ACK message
sent before that OBS is processed, so the operator can sanity-check detection.
We piggyback boxes inside the first ACT instead, keeping the protocol REQ/REP.

Note on EE pose convention: T_w_e is polymetis flange in base frame (per
ip-demo.md decision); same as what the offline demo uses. No grasp offset.
"""

from __future__ import annotations

import argparse
import pickle
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import torch
import zmq

from ip_runner import codec
from ip_runner.preproc import GroundingDinoOnce, SAM2ImageMasker, obs_to_pcd_w


class _Episode:
    """Per-episode mutable state."""

    def __init__(self, episode_id: str, prompt: str,
                 gd_box_threshold: float, gd_text_threshold: float):
        self.episode_id = episode_id
        self.prompt = prompt
        self.gd_box_threshold = gd_box_threshold
        self.gd_text_threshold = gd_text_threshold
        self.boxes_xyxy: list[np.ndarray] = []
        self.box_labels: list[str] = []
        self.box_scores: list[float] = []
        self.gd_done = False
        self.steps_seen = 0


class Server:
    def __init__(self, args):
        self.args = args
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        print(f"[server] device = {self.device}", flush=True)

        # dexycb_pipeline provides object_pcd.lift_mask used inside preproc.
        # Prepend the parent dir so `import dexycb_pipeline.object_pcd` works.
        sys.path.insert(0, str(args.dexycb_pipeline_dir))

        # IP model
        sys.path.insert(0, str(args.instant_policy_dir))
        from instant_policy import sample_to_cond_demo, GraphDiffusion
        from utils import transform_pcd, subsample_pcd
        self.sample_to_cond_demo = sample_to_cond_demo
        self.transform_pcd = transform_pcd
        self.subsample_pcd = subsample_pcd

        print("[server] loading IP model ...", flush=True)
        t0 = time.time()
        self.model = GraphDiffusion.load_from_checkpoint(
            str(args.checkpoint), device=self.device, strict=True,
            map_location=self.device)
        self.model.set_num_demos(args.num_demos)
        self.model.set_num_diffusion_steps(args.num_diffusion_steps)
        self.model.eval()
        print(f"[server] IP model loaded in {time.time() - t0:.1f}s",
              flush=True)

        # Demo
        with args.demo.open("rb") as f:
            demo = pickle.load(f)
        num_traj_wp = len(demo["pcds"])
        self.cond_demo = self.sample_to_cond_demo(demo, num_traj_wp)
        assert len(self.cond_demo["obs"]) == num_traj_wp
        print(f"[server] demo loaded: {num_traj_wp} waypoints,"
              f" first pcd N={demo['pcds'][0].shape[0]}", flush=True)

        # Calibration. Two modes:
        #   static: load fixed T_base_camera (front-facing external cam)
        #   wrist:  load T_ee_camera, compute T_base_camera = T_w_e @ T_ee_camera
        #           per OBS (wrist-mounted cam, follows EE)
        if args.T_base_camera is not None:
            self.camera_mode = "static"
            self.T_base_camera_static = np.load(
                str(args.T_base_camera)).astype(np.float64)
            assert self.T_base_camera_static.shape == (4, 4), \
                f"T_base_camera must be 4x4, got {self.T_base_camera_static.shape}"
            self.T_ee_camera = None
            print(f"[server] camera_mode=static\n"
                  f"T_base_camera=\n{self.T_base_camera_static}", flush=True)
        else:
            self.camera_mode = "wrist"
            self.T_base_camera_static = None
            self.T_ee_camera = np.load(
                str(args.T_ee_camera)).astype(np.float64)
            assert self.T_ee_camera.shape == (4, 4), \
                f"T_ee_camera must be 4x4, got {self.T_ee_camera.shape}"
            print(f"[server] camera_mode=wrist\n"
                  f"T_ee_camera=\n{self.T_ee_camera}", flush=True)

        # GD + SAM2 — load once, reuse across episodes
        print("[server] loading GroundingDINO ...", flush=True)
        t0 = time.time()
        self.gd = GroundingDinoOnce(model_id=args.gd_model, device=self.device)
        print(f"[server] GD loaded in {time.time() - t0:.1f}s", flush=True)

        print("[server] loading SAM2 image predictor ...", flush=True)
        t0 = time.time()
        self.sam2 = SAM2ImageMasker(
            sam2_root=args.sam2_root, ckpt=args.sam2_ckpt,
            cfg=args.sam2_cfg, device=self.device)
        print(f"[server] SAM2 loaded in {time.time() - t0:.1f}s", flush=True)

        # Debug dump
        self.debug_dir: Path | None = None
        if args.debug_dump_dir is not None:
            self.debug_dir = Path(args.debug_dump_dir)
            self.debug_dir.mkdir(parents=True, exist_ok=True)
            # Dump demo waypoints once so we can compare side-by-side later.
            demo_pcds = [np.asarray(p, dtype=np.float32)
                         for p in demo["pcds"]]
            demo_T_w_es = np.stack([np.asarray(T, dtype=np.float64)
                                    for T in demo["T_w_es"]], axis=0)
            demo_grips = np.asarray(demo["grips"], dtype=np.int8)
            np.savez_compressed(
                self.debug_dir / "demo.npz",
                T_w_es=demo_T_w_es, grips=demo_grips,
                # pad-stack into a single array indexed by waypoint
                **{f"pcd_{i:02d}": p for i, p in enumerate(demo_pcds)})
            (self.debug_dir / "camera_mode.txt").write_text(self.camera_mode)
            if self.camera_mode == "static":
                np.save(self.debug_dir / "T_base_camera.npy",
                        self.T_base_camera_static)
            else:
                np.save(self.debug_dir / "T_ee_camera.npy", self.T_ee_camera)
            print(f"[server] debug dump dir: {self.debug_dir}", flush=True)

        # Episode state
        self.episode: _Episode | None = None

        # ZMQ
        self.ctx = zmq.Context.instance()
        self.sock = self.ctx.socket(zmq.REP)
        self.sock.bind(args.bind)
        print(f"[server] listening on {args.bind}", flush=True)

    # -- Handlers ----------------------------------------------------------

    def _handle_reset(self, msg: dict) -> dict:
        self.episode = _Episode(
            episode_id=msg["episode_id"],
            prompt=msg["prompt"],
            gd_box_threshold=float(msg.get("gd_box_threshold", 0.18)),
            gd_text_threshold=float(msg.get("gd_text_threshold", 0.18)),
        )
        print(f"[server] reset episode={self.episode.episode_id}"
              f" prompt={self.episode.prompt!r}", flush=True)
        return {"type": "ack", "ok": True, "msg": "reset", "boxes": []}

    def _handle_obs(self, msg: dict) -> dict:
        if self.episode is None:
            return {"type": "err", "step": msg.get("step"),
                    "msg": "no active episode; send RESET first"}
        if msg["episode_id"] != self.episode.episode_id:
            return {"type": "err", "step": msg.get("step"),
                    "msg": f"episode mismatch: got {msg['episode_id']!r},"
                           f" active {self.episode.episode_id!r}"}

        ep = self.episode
        step = int(msg["step"])
        info: dict[str, float | int | str] = {}

        t_total0 = time.time()

        # Decode RGB and depth
        t0 = time.time()
        color_bgr = cv2.imdecode(np.frombuffer(msg["color_bgr_jpeg"], np.uint8),
                                 cv2.IMREAD_COLOR)
        depth_mm = cv2.imdecode(np.frombuffer(msg["depth_uint16_png"], np.uint8),
                                cv2.IMREAD_UNCHANGED)
        if color_bgr is None or depth_mm is None:
            return {"type": "err", "step": step, "msg": "image decode failed"}
        rgb = cv2.cvtColor(color_bgr, cv2.COLOR_BGR2RGB)
        K = tuple(float(x) for x in msg["K"])
        T_w_e = np.asarray(msg["T_w_e"], dtype=np.float64).reshape(4, 4)
        grip = int(msg["grip"])
        info["t_decode_ms"] = (time.time() - t0) * 1e3

        # GD: re-run every frame in skip_ip (debug-UI) mode so boxes track
        # moving objects; otherwise only on the first OBS of the episode.
        force_gd = bool(msg.get("skip_ip", False))
        ack_boxes = []
        if force_gd or not ep.gd_done:
            t0 = time.time()
            boxes = self.gd.detect(
                rgb, ep.prompt,
                box_threshold=ep.gd_box_threshold,
                text_threshold=ep.gd_text_threshold,
                min_area=self.args.min_area, max_area=self.args.max_area)
            if not boxes:
                return {"type": "err", "step": step,
                        "msg": "GroundingDINO produced no boxes for prompt "
                               f"{ep.prompt!r}"}
            ep.boxes_xyxy = [b.xyxy for b in boxes]
            ep.box_labels = [b.label for b in boxes]
            ep.box_scores = [b.score for b in boxes]
            if not ep.gd_done:
                ack_boxes = [{"label": b.label, "score": b.score,
                              "xyxy": b.xyxy.tolist()} for b in boxes]
                print(f"[server] GD seeded {len(boxes)} boxes:", flush=True)
                for b in boxes:
                    print(f"        {b.label!r} score={b.score:.3f}"
                          f" box={b.xyxy.round(1).tolist()}", flush=True)
            ep.gd_done = True
            info["t_gd_ms"] = (time.time() - t0) * 1e3

        # SAM2 masks
        t0 = time.time()
        masks = self.sam2.step(rgb, ep.boxes_xyxy)
        info["t_sam2_ms"] = (time.time() - t0) * 1e3

        # Lift + base-frame point cloud. In wrist mode T_base_camera is
        # recomputed each OBS from the live EE pose.
        if self.camera_mode == "wrist":
            T_base_camera = T_w_e @ self.T_ee_camera
        else:
            T_base_camera = self.T_base_camera_static
        t0 = time.time()
        pcd_w = obs_to_pcd_w(depth_mm, K, masks, T_base_camera,
                             depth_filter_mm=self.args.depth_filter_mm)
        info["t_pcd_ms"] = (time.time() - t0) * 1e3
        info["pcd_n_points"] = int(pcd_w.shape[0])

        if pcd_w.shape[0] < self.args.min_points:
            return {"type": "err", "step": step,
                    "msg": f"too few points after segmentation: "
                           f"{pcd_w.shape[0]} < {self.args.min_points}"}

        # Debug-UI fast path: skip IP forward, return seg products only.
        if bool(msg.get("skip_ip", False)):
            mask_union = np.zeros(rgb.shape[:2], dtype=np.uint8)
            for m in masks:
                if m is not None:
                    mask_union |= np.asarray(m, dtype=bool).astype(np.uint8) * 255
            ok, mask_png = cv2.imencode(".png", mask_union)
            # Subsample pcd to keep payload small (~1k pts is plenty for viz)
            pcd_viz = pcd_w
            if pcd_viz.shape[0] > 1024:
                idx = np.random.choice(pcd_viz.shape[0], 1024, replace=False)
                pcd_viz = pcd_viz[idx]
            info["t_total_ms"] = (time.time() - t_total0) * 1e3
            return {
                "type": "seg",
                "step": step,
                "mask_png": bytes(mask_png) if ok else b"",
                "boxes_xyxy": [b.tolist() for b in ep.boxes_xyxy],
                "box_labels": list(ep.box_labels),
                "box_scores": [float(s) for s in ep.box_scores],
                "pcd_w": pcd_viz.astype(np.float32),
                "info": info,
                "ack_boxes": ack_boxes,
            }

        # IP forward
        t0 = time.time()
        full_sample = {
            "demos": [self.cond_demo],
            "live": {
                "obs": [self.transform_pcd(self.subsample_pcd(pcd_w),
                                           np.linalg.inv(T_w_e))],
                "grips": [grip],
                "T_w_es": [T_w_e],
            },
        }
        with torch.no_grad():
            actions, grips_pred = self.model.predict_actions(full_sample)
        info["t_ip_ms"] = (time.time() - t0) * 1e3

        action0 = np.asarray(actions[0], dtype=np.float64)  # (4,4)
        T_w_next = T_w_e @ action0
        target_pos = T_w_next[:3, 3].astype(np.float32)
        target_quat = _rot_to_quat_xyzw(T_w_next[:3, :3]).astype(np.float32)

        grip_next = float(np.asarray(grips_pred[0]).item())
        # IP encoding: -1 close, +1 open. Our cmd echoes that.
        grip_cmd = int(np.sign(grip_next)) if abs(grip_next) > 0.1 else 0
        # regrasp signal: predicted grip differs from the input grip
        in_grip_pm = (1 if grip == 1 else -1)
        regrasp = (grip_cmd != 0) and (np.sign(grip_cmd) != in_grip_pm)

        ep.steps_seen += 1
        info["t_total_ms"] = (time.time() - t_total0) * 1e3
        info["steps_seen"] = ep.steps_seen

        if self.debug_dir is not None:
            ep_dir = self.debug_dir / ep.episode_id
            ep_dir.mkdir(exist_ok=True)
            tag = f"step_{step:04d}"
            cv2.imwrite(str(ep_dir / f"{tag}_rgb.jpg"), color_bgr)
            cv2.imwrite(str(ep_dir / f"{tag}_depth_mm.png"),
                        depth_mm.astype(np.uint16))
            mask_union = np.zeros(rgb.shape[:2], dtype=np.uint8)
            for m in masks:
                if m is not None:
                    mask_union |= np.asarray(m, dtype=bool).astype(np.uint8) * 255
            cv2.imwrite(str(ep_dir / f"{tag}_mask.png"), mask_union)
            np.savez_compressed(
                ep_dir / f"{tag}.npz",
                pcd_w=pcd_w.astype(np.float32),
                T_w_e=T_w_e.astype(np.float64),
                action0=action0.astype(np.float64),
                T_w_next=T_w_next.astype(np.float64),
                target_pos=target_pos,
                target_quat=target_quat,
                grip_in=np.int8(grip),
                grip_pred_raw=np.float32(grip_next),
                grip_cmd=np.int8(grip_cmd),
                regrasp=np.bool_(regrasp),
                K=np.asarray(K, dtype=np.float64),
                boxes_xyxy=np.stack(ep.boxes_xyxy, axis=0)
                    if ep.boxes_xyxy else np.zeros((0, 4), dtype=np.float32),
                box_labels=np.asarray(ep.box_labels, dtype=object),
                box_scores=np.asarray(ep.box_scores, dtype=np.float32),
            )

        reply = {
            "type": "act",
            "step": step,
            "target_pos": target_pos,
            "target_quat_xyzw": target_quat,
            "grip_cmd": int(grip_cmd),
            "regrasp_required": bool(regrasp),
            "horizon_idx": 0,
            "info": info,
            "boxes": ack_boxes,  # populated only on first OBS of episode
        }
        # Optionally include seg viz so a debug UI can render bbox/mask/pcd
        # without needing a separate skip_ip request.
        if bool(msg.get("include_seg", False)):
            mask_union = np.zeros(rgb.shape[:2], dtype=np.uint8)
            for m in masks:
                if m is not None:
                    mask_union |= np.asarray(m, dtype=bool).astype(np.uint8) * 255
            ok, mask_png = cv2.imencode(".png", mask_union)
            pcd_viz = pcd_w
            if pcd_viz.shape[0] > 1024:
                idx = np.random.choice(pcd_viz.shape[0], 1024, replace=False)
                pcd_viz = pcd_viz[idx]
            reply["mask_png"] = bytes(mask_png) if ok else b""
            reply["boxes_xyxy"] = [b.tolist() for b in ep.boxes_xyxy]
            reply["box_labels"] = list(ep.box_labels)
            reply["box_scores"] = [float(s) for s in ep.box_scores]
            reply["pcd_w"] = pcd_viz.astype(np.float32)
            reply["grip_pred_raw"] = float(grip_next)
        return reply

    # -- Main loop ---------------------------------------------------------

    def serve(self):
        try:
            while True:
                buf = self.sock.recv()
                try:
                    msg = codec.decode(buf)
                except Exception as e:
                    self.sock.send(codec.encode(
                        {"type": "err", "step": None,
                         "msg": f"decode failed: {e!r}"}))
                    continue

                t = msg.get("type")
                try:
                    if t == "reset":
                        reply = self._handle_reset(msg)
                    elif t == "obs":
                        reply = self._handle_obs(msg)
                    else:
                        reply = {"type": "err", "step": msg.get("step"),
                                 "msg": f"unknown msg type {t!r}"}
                except Exception as e:
                    import traceback as tb
                    reply = {"type": "err", "step": msg.get("step"),
                             "msg": f"{type(e).__name__}: {e}",
                             "trace": tb.format_exc()}

                self.sock.send(codec.encode(reply))
                if reply.get("type") == "act":
                    info = reply.get("info", {})
                    print(f"[server] step={reply['step']:>3}"
                          f" pcd={info.get('pcd_n_points', '?'):>5}pts"
                          f" sam2={info.get('t_sam2_ms', 0):.0f}ms"
                          f" ip={info.get('t_ip_ms', 0):.0f}ms"
                          f" total={info.get('t_total_ms', 0):.0f}ms"
                          f" grip_cmd={reply['grip_cmd']}"
                          f"{' [REGRASP]' if reply['regrasp_required'] else ''}",
                          flush=True)
                elif reply.get("type") == "err":
                    print(f"[server] ERR step={reply.get('step')}:"
                          f" {reply.get('msg')}", flush=True)
        except KeyboardInterrupt:
            print("[server] interrupted, shutting down", flush=True)
        finally:
            self.sock.close()


def _rot_to_quat_xyzw(R: np.ndarray) -> np.ndarray:
    """Rotation matrix -> [x, y, z, w] quaternion. Avoids scipy dependency."""
    m = R
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0:
        s = 0.5 / np.sqrt(tr + 1.0)
        w = 0.25 / s
        x = (m[2, 1] - m[1, 2]) * s
        y = (m[0, 2] - m[2, 0]) * s
        z = (m[1, 0] - m[0, 1]) * s
    elif (m[0, 0] > m[1, 1]) and (m[0, 0] > m[2, 2]):
        s = 2.0 * np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2])
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = 2.0 * np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2])
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1])
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    return np.array([x, y, z, w], dtype=np.float64)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bind", default="tcp://*:5556")
    ap.add_argument("--demo", type=Path, required=True)
    ap.add_argument("--checkpoint", type=Path, required=True)
    ap.add_argument("--instant-policy-dir", type=Path, required=True)
    ap.add_argument("--dexycb-pipeline-dir", type=Path,
                    default=Path("/home/nyuair/zhewen/robo/dexycb_pipeline"),
                    help="parent dir of `dexycb_pipeline/` python package")
    ap.add_argument("--T-base-camera", type=Path, default=None,
                    dest="T_base_camera",
                    help="Static base<-camera extrinsic (front-cam mode). "
                         "Mutually exclusive with --T-ee-camera.")
    ap.add_argument("--T-ee-camera", type=Path, default=None,
                    dest="T_ee_camera",
                    help="Hand-eye ee<-camera extrinsic (wrist-cam mode). "
                         "T_base_camera is recomputed per OBS as T_w_e @ T_ee_camera. "
                         "Mutually exclusive with --T-base-camera.")
    ap.add_argument("--sam2-root", type=Path,
                    default=Path("/home/nyuair/zhewen/sam2"))
    ap.add_argument("--sam2-ckpt", default="sam2.1_hiera_large.pt")
    ap.add_argument("--sam2-cfg",
                    default="configs/sam2.1/sam2.1_hiera_l.yaml")
    ap.add_argument("--gd-model", default="IDEA-Research/grounding-dino-tiny")
    ap.add_argument("--num-demos", type=int, default=1)
    ap.add_argument("--num-diffusion-steps", type=int, default=4)
    ap.add_argument("--depth-filter-mm", type=float, default=120.0)
    ap.add_argument("--min-area", type=float, default=400.0)
    ap.add_argument("--max-area", type=float, default=120000.0)
    ap.add_argument("--min-points", type=int, default=64,
                    help="minimum points after segmentation; below -> ERR")
    ap.add_argument("--debug-dump-dir", type=Path, default=None,
                    help="If set, dump per-step rgb/depth/mask/pcd_w/action "
                         "for offline visualization.")
    args = ap.parse_args()

    if (args.T_base_camera is None) == (args.T_ee_camera is None):
        sys.exit("must pass exactly one of --T-base-camera / --T-ee-camera")

    Server(args).serve()


if __name__ == "__main__":
    main()
