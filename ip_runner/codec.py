"""ZMQ message codec for IP server <-> NUC executor.

Both ends are Python; we use pickle for simplicity. JPEG-encode color and
PNG-16-encode depth on the NUC side to keep frames under a few MB. The server
decodes them with cv2.imdecode.

Schemas
-------
RESET (nuc -> 127):  must be sent before the first OBS of an episode
    {"type": "reset", "episode_id": str, "prompt": str,
     "gd_box_threshold": float, "gd_text_threshold": float}

OBS (nuc -> 127):    one per inference step
    {"type": "obs", "episode_id": str, "step": int,
     "color_bgr_jpeg": bytes,     # cv2.imencode(".jpg", color_bgr)
     "depth_uint16_png": bytes,   # cv2.imencode(".png", depth_uint16)
     "K": (fx, fy, cx, cy),       # 4 floats
     "T_w_e": np.ndarray (4,4) float64,
     "grip": int}                 # 0 closed, 1 open

ACK (127 -> nuc):    response to RESET
    {"type": "ack", "ok": bool, "msg": str, "boxes": list[dict]}
        # boxes: [{"label": str, "score": float, "xyxy": [x1,y1,x2,y2]}]

ACT (127 -> nuc):    response to OBS
    {"type": "act", "step": int,
     "target_pos": np.ndarray (3,) float32,    # base frame
     "target_quat_xyzw": np.ndarray (4,) float32,
     "grip_cmd": int,                 # -1 close, +1 open, 0 no-op
     "regrasp_required": bool,
     "horizon_idx": int,              # always 0 for one-step rollout
     "info": dict}                    # diagnostics (timings, point count, ...)

ERR (127 -> nuc):    on any server-side failure
    {"type": "err", "step": int|None, "msg": str}
"""

from __future__ import annotations

import pickle


def encode(msg: dict) -> bytes:
    return pickle.dumps(msg, protocol=pickle.HIGHEST_PROTOCOL)


def decode(buf: bytes) -> dict:
    return pickle.loads(buf)
