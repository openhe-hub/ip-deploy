"""Backend for the IP debug UI. Runs on franka-nuc.

- WebSocket /ws: pushes per-frame {rgb_jpeg, mask_png, boxes, pcd_w, ee_pose,
  gripper} to the connected browser; receives keyboard events and applies them
  via polymetis.update_desired_ee_pose / gripper.goto / gripper.grasp.
- GET /: serves static/index.html.
- The mask + bbox + pcd_w come from the existing IP server (nyu-127) running
  in skip_ip mode (no model forward, just GD+SAM2+pcd_lift).

Usage:
    ssh franka-backup
    cd /home/franka/ICRT
    PYTHONPATH=/home/franka/ICRT python -m ip_debug_ui.server \
      --ip-server tcp://10.224.36.127:5556 \
      --prompt "red cube . green cube . blue cube . yellow cube ."

Then on local Mac:
    ssh -L 8000:localhost:8000 franka-backup
    open http://localhost:8000
"""
from __future__ import annotations

import argparse
import asyncio
import base64
import json
import pickle
import sys
import time
import uuid
from pathlib import Path
from typing import Optional

import cv2
import numpy as np
import torch
import zmq
from fastapi import FastAPI, WebSocket, WebSocketDisconnect

# Python 3.8 compatibility shim for asyncio.to_thread (added in 3.9).
if not hasattr(asyncio, "to_thread"):
    import functools
    async def _to_thread(fn, *args, **kwargs):
        loop = asyncio.get_event_loop()
        return await loop.run_in_executor(
            None, functools.partial(fn, *args, **kwargs))
    asyncio.to_thread = _to_thread  # type: ignore[attr-defined]

from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
import uvicorn

# Reuse local helpers
sys.path.insert(0, "/home/franka/ICRT")
from show_rgbd import RealSenseCamera  # noqa: E402
from ip_executor import codec  # noqa: E402
from ip_executor.safety import quat_xyzw_to_R, leash  # noqa: E402

HOME_Q_RAD = [0.0, -0.785398, 0.0, -2.356194, 0.0, 1.570796, 0.785398]
# EE quat at home (xyzw) — gripper z axis pointing straight down on the table.
HOME_QUAT_XYZW = [0.9262571334838867, -0.37645044922828674,
                  -0.002783368807286024, -0.018029851838946342]


def _R_to_quat_xyzw(R: np.ndarray) -> np.ndarray:
    m = R
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0:
        s = 0.5 / np.sqrt(tr + 1.0)
        return np.array([(m[2, 1] - m[1, 2]) * s,
                         (m[0, 2] - m[2, 0]) * s,
                         (m[1, 0] - m[0, 1]) * s,
                         0.25 / s], dtype=np.float64)
    if (m[0, 0] > m[1, 1]) and (m[0, 0] > m[2, 2]):
        s = 2.0 * np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2])
        return np.array([0.25 * s,
                         (m[0, 1] + m[1, 0]) / s,
                         (m[0, 2] + m[2, 0]) / s,
                         (m[2, 1] - m[1, 2]) / s], dtype=np.float64)
    if m[1, 1] > m[2, 2]:
        s = 2.0 * np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2])
        return np.array([(m[0, 1] + m[1, 0]) / s,
                         0.25 * s,
                         (m[1, 2] + m[2, 1]) / s,
                         (m[0, 2] - m[2, 0]) / s], dtype=np.float64)
    s = 2.0 * np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1])
    return np.array([(m[0, 2] + m[2, 0]) / s,
                     (m[1, 2] + m[2, 1]) / s,
                     0.25 * s,
                     (m[1, 0] - m[0, 1]) / s], dtype=np.float64)


def _Rx(a: float) -> np.ndarray:
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def _Ry(a: float) -> np.ndarray:
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def _Rz(a: float) -> np.ndarray:
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


class State:
    """Shared state between websocket coros and background loops."""

    def __init__(self, args):
        self.args = args
        self.cam: Optional[RealSenseCamera] = None
        self.K: tuple = (0.0, 0.0, 0.0, 0.0)
        self.depth_scale: float = 0.001
        self.robot = None
        self.gripper = None
        self.zmq_ctx = zmq.Context.instance()
        self.zmq_sock: zmq.Socket = self._new_zmq_sock()
        self.zmq_lock = asyncio.Lock()
        self.episode_id = ""
        self.last_frame: Optional[dict] = None
        self.last_seg: Optional[dict] = None
        self.boxes_cached: list = []
        self.box_labels: list = []
        self.box_scores: list = []

        # Recording state for capturing IP demos
        self.recording: bool = False
        self.recording_buffer: list = []  # list of dicts
        self.recording_started_at: float = 0.0
        self.demos_dir = Path(args.demos_dir)
        self.demos_dir.mkdir(parents=True, exist_ok=True)

        # IP closed-loop state
        self.running_ip: bool = False
        self.ip_step_i: int = 0
        self.last_grip_state: int = 1  # assume open at start

        # Latest observations (kept fresh by the camera loop)
        self.color_bgr: Optional[np.ndarray] = None
        self.depth_mm: Optional[np.ndarray] = None
        self.ee_pos: np.ndarray = np.zeros(3)
        self.ee_quat: np.ndarray = np.array([0, 0, 0, 1.0])
        self.gripper_width: float = 0.0

    def _new_zmq_sock(self) -> zmq.Socket:
        s = self.zmq_ctx.socket(zmq.REQ)
        s.setsockopt(zmq.LINGER, 0)
        s.setsockopt(zmq.RCVTIMEO, 10_000)
        s.connect(self.args.ip_server)
        return s

    def reconnect_zmq(self):
        try:
            self.zmq_sock.close(linger=0)
        except Exception:
            pass
        self.zmq_sock = self._new_zmq_sock()
        # Force a re-handshake with the (possibly restarted) server.
        self.episode_id = ""

    def init_hardware(self):
        a = self.args
        from polymetis import RobotInterface, GripperInterface
        self.cam = RealSenseCamera(
            serial=a.camera_serial, name=a.camera_serial,
            width=a.width, height=a.height, fps=a.fps)
        intr = self.cam.intrinsics_dict()
        self.K = (float(intr["fx"]), float(intr["fy"]),
                  float(intr["cx"]), float(intr["cy"]))
        self.depth_scale = float(self.cam.depth_scale)
        self.robot = RobotInterface(ip_address=a.robot_host, port=a.robot_port)
        try:
            self.gripper = GripperInterface(
                ip_address=a.gripper_host, port=a.gripper_port)
        except Exception as e:
            print(f"[ui] gripper unavailable: {e}", flush=True)
            self.gripper = None
        try:
            self.robot.terminate_current_policy()
            time.sleep(0.3)
        except Exception:
            pass
        self.robot.start_cartesian_impedance()
        print("[ui] cartesian impedance started", flush=True)

    def reset_episode(self, prompt: str, gd_box_threshold: float = 0.40,
                      gd_text_threshold: float = 0.40) -> dict:
        self.episode_id = uuid.uuid4().hex[:12]
        self.boxes_cached = []
        self.box_labels = []
        self.box_scores = []
        # Drop stale seg data so the UI doesn't render the previous episode's
        # boxes/mask on top of fresh RGB until the next seg result lands.
        self.last_seg = None
        msg = {
            "type": "reset", "episode_id": self.episode_id, "prompt": prompt,
            "gd_box_threshold": gd_box_threshold,
            "gd_text_threshold": gd_text_threshold,
        }
        self.zmq_sock.send(codec.encode(msg))
        ack = codec.decode(self.zmq_sock.recv())
        return ack


def _build_app(state: State, args) -> FastAPI:
    app = FastAPI()
    here = Path(__file__).parent
    app.mount("/static", StaticFiles(directory=str(here / "static")),
              name="static")

    @app.get("/")
    def index():
        return FileResponse(str(here / "static" / "index.html"))

    @app.get("/api/init")
    def init_info():
        return {
            "K": list(state.K),
            "width": args.width, "height": args.height,
            "prompt": args.prompt,
        }

    @app.websocket("/ws")
    async def ws_handler(ws: WebSocket):
        await ws.accept()
        print("[ui] client connected", flush=True)
        # Send initial config
        await ws.send_json({
            "type": "config",
            "K": list(state.K),
            "width": args.width, "height": args.height,
            "prompt": args.prompt,
            "boxes": state.boxes_cached,
            "box_labels": state.box_labels,
            "box_scores": state.box_scores,
        })

        async def push_loop():
            while True:
                if state.color_bgr is not None:
                    jpeg = cv2.imencode(
                        ".jpg", state.color_bgr,
                        [int(cv2.IMWRITE_JPEG_QUALITY), 70])[1]
                    payload = {
                        "type": "frame",
                        "rgb_jpeg_b64": base64.b64encode(bytes(jpeg)).decode(),
                        "ee_pos": state.ee_pos.tolist(),
                        "ee_quat": state.ee_quat.tolist(),
                        "gripper_width": float(state.gripper_width),
                    }
                    if state.last_seg is not None:
                        payload["mask_png_b64"] = state.last_seg["mask_png_b64"]
                        payload["boxes"] = state.last_seg["boxes"]
                        payload["box_labels"] = state.last_seg["box_labels"]
                        payload["box_scores"] = state.last_seg["box_scores"]
                        payload["pcd_w"] = state.last_seg["pcd_w"]
                        payload["pcd_n"] = state.last_seg["pcd_n"]
                        payload["seg_age_ms"] = (
                            time.time() - state.last_seg["t"]) * 1e3
                    payload["recording"] = state.recording
                    payload["recording_n"] = len(state.recording_buffer)
                    if state.recording:
                        payload["recording_dur_s"] = (
                            time.time() - state.recording_started_at)
                    payload["running_ip"] = state.running_ip
                    payload["ip_step"] = state.ip_step_i
                    try:
                        await ws.send_json(payload)
                    except Exception:
                        return
                await asyncio.sleep(0.1)  # 10 Hz to browser

        async def recv_loop():
            while True:
                try:
                    raw = await ws.receive_text()
                except WebSocketDisconnect:
                    print("[ui] client disconnected", flush=True)
                    return
                except Exception:
                    return
                try:
                    msg = json.loads(raw)
                except Exception:
                    continue
                t = msg.get("type")
                if t == "ee_delta":
                    await _apply_ee_delta(state, msg, args)
                elif t == "gripper":
                    await _apply_gripper(state, msg)
                elif t == "home":
                    await _go_home(state)
                elif t == "straighten":
                    await _straighten(state)
                elif t == "reset_episode":
                    ack = await asyncio.to_thread(
                        state.reset_episode,
                        msg.get("prompt", args.prompt),
                        float(msg.get("gd_box_threshold", 0.40)),
                        float(msg.get("gd_text_threshold", 0.40)))
                    await ws.send_json({"type": "reset_ack", "ack": ack})
                elif t == "record_start":
                    state.recording_buffer = []
                    state.recording_started_at = time.time()
                    state.recording = True
                    name = msg.get("name", "")
                    print(f"[ui] recording started"
                          f" name={name!r}", flush=True)
                    await ws.send_json({
                        "type": "record_started", "name": name})
                elif t == "record_stop":
                    state.recording = False
                    name = msg.get("name", "").strip()
                    out = await asyncio.to_thread(
                        _save_demo, state, args, name)
                    await ws.send_json({"type": "record_saved",
                                        **out})
                elif t == "record_cancel":
                    state.recording = False
                    state.recording_buffer = []
                    print("[ui] recording cancelled", flush=True)
                    await ws.send_json({"type": "record_cancelled"})
                elif t == "ip_run_start":
                    state.running_ip = True
                    state.ip_step_i = 0
                    print("[ui] IP closed-loop START", flush=True)
                    await ws.send_json({"type": "ip_run_started"})
                elif t == "ip_run_stop":
                    state.running_ip = False
                    print("[ui] IP closed-loop STOP", flush=True)
                    await ws.send_json({"type": "ip_run_stopped"})
                else:
                    pass

        await asyncio.gather(push_loop(), recv_loop())

    return app


async def _apply_ip_act(state: "State", args, act_msg: dict) -> None:
    """Apply target_pos/quat from server's act response, plus gripper edge-trigger."""
    if state.robot is None:
        return
    target_pos = np.asarray(act_msg.get("target_pos"), dtype=np.float32).reshape(3)
    target_quat = np.asarray(act_msg.get("target_quat_xyzw"),
                             dtype=np.float32).reshape(4)
    # leash against current measured pose
    pos_meas = state.ee_pos.astype(np.float32)
    quat_meas = state.ee_quat.astype(np.float32)
    pos_new, quat_new = leash(
        pos_meas, quat_meas, target_pos, target_quat,
        max_pos_step_m=args.max_pos_step_m,
        max_rot_step_rad=np.deg2rad(args.max_rot_step_deg),
    )
    pos_t = torch.from_numpy(pos_new)
    quat_t = torch.from_numpy(quat_new)
    await asyncio.to_thread(
        state.robot.update_desired_ee_pose, pos_t, quat_t)

    grip_cmd = int(act_msg.get("grip_cmd", 0))
    if grip_cmd != 0 and state.gripper is not None:
        desired = 1 if grip_cmd > 0 else 0
        if desired != state.last_grip_state:
            # polymetis gripper goto/grasp are blocking RPCs; fire-and-forget
            # so the IP control loop doesn't stall waiting on the hand.
            def _do_grip(g, d):
                try:
                    if d == 1:
                        g.goto(0.08, 0.1, 30.0)
                    else:
                        g.grasp(0.1, 30.0, 0.0)
                except Exception as e:
                    print(f"[ui] gripper cmd err: {e!r}", flush=True)
            loop = asyncio.get_event_loop()
            loop.run_in_executor(None, _do_grip, state.gripper, desired)
            state.last_grip_state = desired
            print(f"[ui] grip {'open' if desired else 'close'} (async)",
                  flush=True)


def _save_demo(state: "State", args, name: str = "") -> dict:
    """Pickle the current recording buffer in IP demo format."""
    buf = state.recording_buffer
    if len(buf) < args.demo_min_frames:
        return {"ok": False,
                "msg": f"too few frames ({len(buf)} < {args.demo_min_frames})"}
    n_wp = args.demo_num_waypoints
    idxs = np.linspace(0, len(buf) - 1, n_wp).astype(int).tolist()
    pcds = [buf[i]["pcd_w"].astype(np.float32) for i in idxs]
    T_w_es = [buf[i]["T_w_e"].astype(np.float64) for i in idxs]
    grips = [
        1 if buf[i]["gripper_width_m"] > args.gripper_threshold_m else 0
        for i in idxs]
    name_part = name.replace(" ", "_") + "_" if name else ""
    ts = time.strftime("%Y-%m-%d_%H-%M-%S")
    path = state.demos_dir / f"demo_{name_part}{ts}.pkl"
    with path.open("wb") as f:
        pickle.dump({"pcds": pcds, "T_w_es": T_w_es, "grips": grips}, f)

    grip_widths = [buf[i]["gripper_width_m"] for i in idxs]
    pcd_counts = [int(p.shape[0]) for p in pcds]
    print(f"[ui] saved demo: {path}", flush=True)
    print(f"     {len(buf)} buffer frames -> {n_wp} waypoints", flush=True)
    print(f"     pcd counts {pcd_counts}", flush=True)
    print(f"     grips {grips}", flush=True)
    return {
        "ok": True,
        "path": str(path),
        "buffer_n": len(buf),
        "waypoints": n_wp,
        "picked_idxs": idxs,
        "pcd_counts": pcd_counts,
        "grips": grips,
        "grip_widths": [round(w, 4) for w in grip_widths],
    }


async def _straighten(state: State):
    """Reorient EE to the home (straight-down) quat, keeping current position."""
    if state.robot is None:
        return
    pos = state.ee_pos.astype(np.float32)
    quat_target = np.array(HOME_QUAT_XYZW, dtype=np.float32)
    pos_t = torch.from_numpy(pos)
    quat_t = torch.from_numpy(quat_target)
    await asyncio.to_thread(
        state.robot.update_desired_ee_pose, pos_t, quat_t)
    print(f"[ui] straighten: pos={pos.tolist()} quat=HOME", flush=True)


async def _go_home(state: State):
    if state.robot is None:
        return
    try:
        state.robot.terminate_current_policy()
    except Exception:
        pass
    await asyncio.sleep(0.3)
    await asyncio.to_thread(
        state.robot.move_to_joint_positions,
        torch.tensor(HOME_Q_RAD), 4.0)
    await asyncio.sleep(0.5)
    state.robot.start_cartesian_impedance()
    if state.gripper is not None:
        await asyncio.to_thread(state.gripper.goto, 0.08, 0.1, 10.0)
    print("[ui] homed", flush=True)


async def _apply_gripper(state: State, msg: dict):
    if state.gripper is None:
        return
    action = msg.get("action")

    # Fire-and-forget so we don't block the WS recv coroutine; also keep the
    # IP loop's edge-trigger in sync so it doesn't immediately undo this.
    def _do(g, a):
        try:
            if a == "open":
                g.goto(0.08, 0.1, 30.0)
            elif a == "close":
                g.grasp(0.1, 30.0, 0.0)
        except Exception as e:
            print(f"[ui] gripper {a!r} err: {e!r}", flush=True)
    loop = asyncio.get_event_loop()
    loop.run_in_executor(None, _do, state.gripper, action)
    if action == "open":
        state.last_grip_state = 1
    elif action == "close":
        state.last_grip_state = 0
    print(f"[ui] manual gripper {action} (last_grip_state={state.last_grip_state})",
          flush=True)


async def _apply_ee_delta(state: State, msg: dict, args):
    """Apply a small Cartesian or rotational delta to current EE target."""
    if state.robot is None:
        return
    dpos = np.asarray(msg.get("dpos", [0, 0, 0]), dtype=np.float64)
    drpy_deg = np.asarray(msg.get("drpy_deg", [0, 0, 0]), dtype=np.float64)

    # Use current measured pose as base (impedance holds last commanded; small
    # accumulated drift between command and measured is OK for teleop).
    pos = state.ee_pos.copy()
    quat = state.ee_quat.copy()
    R = quat_xyzw_to_R(quat)
    dR = _Rz(np.deg2rad(drpy_deg[2])) @ _Ry(np.deg2rad(drpy_deg[1])) \
        @ _Rx(np.deg2rad(drpy_deg[0]))
    R_new = R @ dR  # delta in EE frame
    pos_new = pos + dpos  # base-frame translation
    quat_new = _R_to_quat_xyzw(R_new)

    pos_new, quat_new = leash(
        pos.astype(np.float32), quat.astype(np.float32),
        pos_new.astype(np.float32), quat_new.astype(np.float32),
        max_pos_step_m=args.max_pos_step_m,
        max_rot_step_rad=np.deg2rad(args.max_rot_step_deg),
    )
    pos_t = torch.from_numpy(np.asarray(pos_new, dtype=np.float32))
    quat_t = torch.from_numpy(np.asarray(quat_new, dtype=np.float32))
    await asyncio.to_thread(
        state.robot.update_desired_ee_pose, pos_t, quat_t)


async def camera_loop(state: State):
    """Poll RealSense + polymetis pose at the camera fps."""
    while True:
        try:
            color, depth_raw = state.cam.poll()
        except Exception:
            await asyncio.sleep(0.05)
            continue
        if color is None or depth_raw is None:
            await asyncio.sleep(0.02)
            continue
        depth_mm = (depth_raw.astype(np.float32) * state.depth_scale * 1000.0
                    ).astype(np.uint16)
        state.color_bgr = color
        state.depth_mm = depth_mm
        if state.robot is not None:
            try:
                pos, quat = state.robot.get_ee_pose()
                state.ee_pos = np.asarray(pos.cpu(), dtype=np.float64).reshape(3)
                state.ee_quat = np.asarray(quat.cpu(), dtype=np.float64).reshape(4)
            except Exception:
                pass
        if state.gripper is not None:
            try:
                gs = state.gripper.get_state()
                state.gripper_width = float(getattr(gs, "width", 0.0))
            except Exception:
                pass
        await asyncio.sleep(0)


async def seg_loop(state: State, args):
    """Periodically POST the current obs to nyu-127 in skip_ip mode."""
    step_i = 0
    while True:
        if state.color_bgr is None or state.depth_mm is None:
            await asyncio.sleep(0.1)
            continue
        if not state.episode_id:
            try:
                ack = await asyncio.to_thread(
                    state.reset_episode, args.prompt)
                print(f"[ui] auto-reset: {ack}", flush=True)
            except Exception as e:
                print(f"[ui] auto-reset failed ({e!r}) — reconnecting",
                      flush=True)
                state.reconnect_zmq()
                await asyncio.sleep(0.5)
            continue
        step_i += 1
        # Build OBS
        ok1, jpeg = cv2.imencode(
            ".jpg", state.color_bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
        ok2, png = cv2.imencode(".png", state.depth_mm)
        if not (ok1 and ok2):
            await asyncio.sleep(0.1)
            continue
        T_w_e = np.eye(4, dtype=np.float64)
        T_w_e[:3, :3] = quat_xyzw_to_R(state.ee_quat)
        T_w_e[:3, 3] = state.ee_pos
        grip_in = 1 if state.gripper_width > args.gripper_threshold_m else 0
        ip_mode = state.running_ip
        if ip_mode:
            state.ip_step_i += 1
        msg = {
            "type": "obs", "episode_id": state.episode_id,
            "step": state.ip_step_i if ip_mode else step_i,
            "color_bgr_jpeg": bytes(jpeg), "depth_uint16_png": bytes(png),
            "K": state.K, "T_w_e": T_w_e, "grip": grip_in,
            "skip_ip": (not ip_mode),
            "include_seg": ip_mode,  # in IP mode, ask server to bundle viz back
        }
        async with state.zmq_lock:
            try:
                await asyncio.to_thread(
                    state.zmq_sock.send, codec.encode(msg))
                rep = await asyncio.to_thread(state.zmq_sock.recv)
            except Exception as e:
                print(f"[ui] zmq err: {e!r} — reconnecting", flush=True)
                state.reconnect_zmq()
                await asyncio.sleep(0.5)
                continue
        try:
            rep_msg = codec.decode(rep)
        except Exception:
            continue
        if rep_msg.get("type") not in ("seg", "act"):
            print(f"[ui] unexpected reply: {rep_msg.get('type')!r} "
                  f"{rep_msg.get('msg', '')}", flush=True)
            await asyncio.sleep(0.2)
            continue
        mask_png = rep_msg.get("mask_png", b"")
        boxes = rep_msg.get("boxes_xyxy", [])
        labels = rep_msg.get("box_labels", [])
        scores = rep_msg.get("box_scores", [])
        pcd = np.asarray(rep_msg.get("pcd_w", np.zeros((0, 3))),
                         dtype=np.float32)

        # In IP mode, also apply the action.
        if rep_msg.get("type") == "act":
            await _apply_ip_act(state, args, rep_msg)
        state.boxes_cached = boxes
        state.box_labels = labels
        state.box_scores = scores
        state.last_seg = {
            "mask_png_b64": base64.b64encode(mask_png).decode() if mask_png else "",
            "boxes": boxes,
            "box_labels": labels,
            "box_scores": scores,
            "pcd_w": pcd.tolist(),
            "pcd_n": int(pcd.shape[0]),
            "t": time.time(),
        }
        if state.recording and pcd.shape[0] >= args.demo_min_pcd_points:
            # Snapshot what IP demo wants: pcd_w (M,3), T_w_e (4,4), grip width.
            T_we = np.eye(4, dtype=np.float64)
            T_we[:3, :3] = quat_xyzw_to_R(state.ee_quat)
            T_we[:3, 3] = state.ee_pos
            state.recording_buffer.append({
                "ts": time.time(),
                "pcd_w": pcd.copy(),
                "T_w_e": T_we,
                "gripper_width_m": float(state.gripper_width),
            })
        await asyncio.sleep(args.seg_period)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ip-server", default="tcp://10.224.36.127:5556")
    ap.add_argument("--prompt",
                    default="red cube . green cube . blue cube . yellow cube .")
    ap.add_argument("--camera-serial", default="925622071356")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--fps", type=int, default=6)
    ap.add_argument("--robot-host", default="localhost")
    ap.add_argument("--robot-port", type=int, default=50051)
    ap.add_argument("--gripper-host", default="localhost")
    ap.add_argument("--gripper-port", type=int, default=50052)
    ap.add_argument("--max-pos-step-m", type=float, default=0.04)
    ap.add_argument("--max-rot-step-deg", type=float, default=10.0)
    ap.add_argument("--gripper-threshold-m", type=float, default=0.04)
    ap.add_argument("--seg-period", type=float, default=0.3,
                    help="seconds between skip_ip seg requests (~3 Hz)")
    ap.add_argument("--demos-dir",
                    default="/home/franka/ip-deploy/demos",
                    help="where recorded IP demos get pickled (kept outside "
                         "ICRT/ since that's a collaborator's tree)")
    ap.add_argument("--demo-num-waypoints", type=int, default=10)
    ap.add_argument("--demo-min-frames", type=int, default=20,
                    help="abort save if buffer has fewer frames than this")
    ap.add_argument("--demo-min-pcd-points", type=int, default=64,
                    help="drop frames whose pcd_w has fewer points than this")
    ap.add_argument("--listen-host", default="0.0.0.0")
    ap.add_argument("--listen-port", type=int, default=8000)
    args = ap.parse_args()

    state = State(args)
    state.init_hardware()
    app = _build_app(state, args)

    @app.on_event("startup")
    async def startup():
        asyncio.create_task(camera_loop(state))
        asyncio.create_task(seg_loop(state, args))

    uvicorn.run(app, host=args.listen_host, port=args.listen_port,
                log_level="warning")


if __name__ == "__main__":
    main()
