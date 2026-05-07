"""ZMQ REQ client to the IP inference server on nyu-127."""

from __future__ import annotations

import time
import uuid
from dataclasses import dataclass

import cv2
import numpy as np
import zmq

from ip_executor import codec


@dataclass
class ActionMsg:
    step: int
    target_pos: np.ndarray         # (3,) float32 base frame
    target_quat_xyzw: np.ndarray   # (4,) float32
    grip_cmd: int                  # -1 close, +1 open, 0 no-op
    regrasp_required: bool
    horizon_idx: int
    info: dict
    boxes: list                    # populated only on first OBS of an episode


class IPClient:
    def __init__(self, server_addr: str, recv_timeout_ms: int = 30_000):
        self.ctx = zmq.Context.instance()
        self.sock = self.ctx.socket(zmq.REQ)
        self.sock.setsockopt(zmq.LINGER, 0)
        self.sock.setsockopt(zmq.RCVTIMEO, recv_timeout_ms)
        self.sock.connect(server_addr)
        self.episode_id = ""
        self._step = 0

    def reset(self, prompt: str,
              gd_box_threshold: float = 0.18,
              gd_text_threshold: float = 0.18,
              episode_id: str | None = None) -> dict:
        self.episode_id = episode_id or uuid.uuid4().hex[:12]
        self._step = 0
        msg = {
            "type": "reset",
            "episode_id": self.episode_id,
            "prompt": prompt,
            "gd_box_threshold": gd_box_threshold,
            "gd_text_threshold": gd_text_threshold,
        }
        self.sock.send(codec.encode(msg))
        ack = codec.decode(self.sock.recv())
        if ack.get("type") != "ack" or not ack.get("ok"):
            raise RuntimeError(f"reset rejected: {ack}")
        return ack

    def step(self,
             color_bgr: np.ndarray,
             depth_uint16_mm: np.ndarray,
             K: tuple,
             T_w_e: np.ndarray,
             grip: int,
             jpeg_quality: int = 90) -> ActionMsg:
        self._step += 1
        ok1, jpeg = cv2.imencode(".jpg", color_bgr,
                                 [int(cv2.IMWRITE_JPEG_QUALITY), jpeg_quality])
        ok2, png = cv2.imencode(".png", depth_uint16_mm)
        if not (ok1 and ok2):
            raise RuntimeError("imencode failed")

        msg = {
            "type": "obs",
            "episode_id": self.episode_id,
            "step": self._step,
            "color_bgr_jpeg": bytes(jpeg),
            "depth_uint16_png": bytes(png),
            "K": tuple(float(x) for x in K),
            "T_w_e": np.asarray(T_w_e, dtype=np.float64),
            "grip": int(grip),
        }
        t0 = time.time()
        self.sock.send(codec.encode(msg))
        rep = codec.decode(self.sock.recv())
        rtt_ms = (time.time() - t0) * 1e3
        if rep.get("type") == "err":
            raise RuntimeError(f"server err: {rep.get('msg')}")
        if rep.get("type") != "act":
            raise RuntimeError(f"unexpected reply: {rep}")
        info = dict(rep.get("info", {}))
        info["rtt_ms"] = rtt_ms
        return ActionMsg(
            step=int(rep["step"]),
            target_pos=np.asarray(rep["target_pos"], dtype=np.float32),
            target_quat_xyzw=np.asarray(rep["target_quat_xyzw"], dtype=np.float32),
            grip_cmd=int(rep["grip_cmd"]),
            regrasp_required=bool(rep["regrasp_required"]),
            horizon_idx=int(rep["horizon_idx"]),
            info=info,
            boxes=list(rep.get("boxes", [])),
        )

    def close(self):
        self.sock.close()
