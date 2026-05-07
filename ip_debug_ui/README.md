# IP debug UI

Browser-based teleop + multi-view inspector for the IP deployment.

## What it shows

- **RGB + bbox** — live RGB with cached GD detections overlaid (color-coded per object label).
- **RGB + mask** — same RGB tinted green where SAM2 says "object pixel".
- **pcd_w top-down** — the segmented + transformed point cloud in base frame, overlaid with the EE marker on a 0.9 m × 0.8 m grid (x_base × y_base).
- **state** — `ee_pos`, `ee_quat`, gripper width, GD box count, pcd_w point count, seg-result age, browser FPS.

## Keyboard

| keys | action |
|---|---|
| W/S | x_base ±1 cm |
| A/D | y_base ±1 cm |
| Q/E | z_base ±1 cm |
| ↑/↓ | pitch ±5° (EE frame) |
| ←/→ | roll ±5° (EE frame) |
| J/L | yaw ±5° (EE frame) |
| Shift held | step size × 3 |
| Ctrl held | step size × 0.2 |
| Space | gripper open |
| Shift+Space | gripper close |
| R | home (joint move to layout home_q) |
| top-bar buttons | reset episode / home / open / close |

## Launch

**Pre-req:** the existing IP server (`ip_runner.server`) must be running on nyu-127. The server binary needs to support the `skip_ip` field on OBS messages — that lives in this repo's `ip_runner/server.py`. If you start the server before pulling this branch you'll need to restart it.

```bash
# 1. on nyu-127: server (same as before)
ssh nyu-127
cd ~/zhewen/robo/ip-deploy
source ~/miniconda3/etc/profile.d/conda.sh && conda activate ip_deploy
python -m ip_runner.server --bind tcp://*:5556 \
  --demo ip_runner/demos/demo_grasp_red.pkl \
  --checkpoint ./instant_policy/checkpoints/model.pt \
  --instant-policy-dir ./instant_policy \
  --T-base-camera ip_runner/calib/T_base_camera_3.3cm.npy \
  --dexycb-pipeline-dir /home/nyuair/zhewen/robo/dexycb_pipeline

# 2. on franka-nuc: this UI
ssh franka-backup
source /home/franka/conda/etc/profile.d/conda.sh && conda activate polymetis-local
cd /home/franka/ICRT
PYTHONPATH=/home/franka/ICRT python -m ip_debug_ui.server \
  --ip-server tcp://10.224.36.127:5556 \
  --prompt "red cube . green cube . blue cube . yellow cube ."

# 3. on local Mac: tunnel + open browser
ssh -L 8000:localhost:8000 franka-backup
open http://localhost:8000
```

## Recording IP demos

A red `● record` button in the topbar starts buffering the live `(pcd_w, T_w_e, gripper_width)` stream. While recording, the button pulses; click `■ stop & save (N)` to finalize. Frames with too few segmented points (`< --demo-min-pcd-points`, default 64) are dropped.

On stop the backend evenly downsamples the buffer to `--demo-num-waypoints` (default 10) and pickles them in the same schema `build_ip_demo.py` produces:

```python
{"pcds":   [np.ndarray(M_i, 3) float32, ...],
 "T_w_es": [np.ndarray(4, 4)   float64, ...],
 "grips":  [int  (0=closed, 1=open),    ...]}
```

Files land in `--demos-dir` (default `/home/franka/ip-deploy/demos/`, *not* under ICRT — that tree belongs to a collaborator). Filename is `demo_<name>_<YYYY-MM-DD_HH-MM-SS>.pkl` if you typed something in the optional name field.

The grip threshold is `--gripper-threshold-m` (default 0.04 m), same as the executor's.

To use the recorded demo for inference, restart the IP server pointing at the new pickle:

```bash
python -m ip_runner.server ... --demo /home/franka/ip-deploy/demos/demo_pickplace_2026-05-06_….pkl
```

**Workflow tip:** because GD re-runs every frame in skip_ip mode, you can freely move the cube around mid-record and the boxes/masks will follow. The recording captures the segmentation that would actually happen at inference time, so you don't need to re-run dexycb_pipeline afterwards.

## Notes / caveats

- Uses polymetis Cartesian impedance. The leash (`max_pos_step_m=0.04`, `max_rot_step_deg=10°`) clamps every keypress against the *current measured* pose so a stuck key won't fly the arm off.
- Rotations are applied in the **EE frame**; translations in the **base frame**. That matches what the user usually wants when watching a top-down map plus an end-effector camera.
- `R` (home) is a joint-space `move_to_joint_positions` to the layout-recorded `home_q_rad`. It blocks for ~4 s.
- `seg_period` defaults to 0.3 s (~3 Hz) which is plenty for visual feedback. Bump it if SAM2 gets bogged down.
- The pcd_w panel uses a fixed view extent (x∈[0, 0.9], y∈[-0.4, 0.4]). Points outside are clipped, so if you don't see anything it usually means GD/SAM2 produced an empty pcd or it's all behind the base origin.

## Troubleshooting

**"too few points after segmentation"** — GD didn't detect anything, or its boxes were tiny. Try lowering `--gd-box-threshold` on the server, or change the prompt.

**RealSense `errno=16 EBUSY`** — kernel driver state went stale. `echo q1w2 | sudo -S modprobe -r uvcvideo && echo q1w2 | sudo -S modprobe uvcvideo`.

**Buttons / arrow keys move the page instead of the arm** — click anywhere inside the canvases first to focus the document.
