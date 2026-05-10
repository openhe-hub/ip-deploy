# Instant Policy 实机部署管线

从录制的 RGB-D demo 到 Franka FR3 实时控制的完整数据流。两台机器分工:
**nyu-127**(GPU + ip_deploy env)负责离线 demo 构造 + 在线感知/推理;
**franka-nuc**(PREEMPT_RT + polymetis-local)负责相机采集 + 实时控制。

两条等价路径并存:**A. CLI executor**(`ip_executor/run.py`,原始管线);
**B. Web UI**(`ip_debug_ui/`,2026-05-06 加入,见 [`ui.md`](./ui.md))。
两者共用同一个 ZMQ 协议与 IP server。今日实测的局限见
[`review/problem-05-06.md`](./review/problem-05-06.md)。

## 1. 离线 demo 构造(一次性)

```
┌─────────────────────────────────────────────────────────────────────────┐
│                          OFFLINE: build demo (one-shot)                  │
│                                                                          │
│  recording_2026-05-04_12-12-47_full/                                     │
│  ├─ poses.csv (50Hz)              ─┐                                     │
│  ├─ cam_<sn>/frames.csv (6Hz)     ─┤                                     │
│  └─ cam_<sn>/{color,depth}_*.png  ─┘                                     │
│              │                                                           │
│              ▼                                                           │
│  dexycb_pipeline (offline, 完整序列):                                    │
│      GroundingDINO 1次 → SAM2 video propagate → seg masks (180帧)        │
│      depth + masks + K → lift_mask → points_camera                       │
│      points_base = T_base_camera @ points_camera                         │
│              │                                                           │
│              ▼                                                           │
│  object_pointclouds_ee.npz                                               │
│      points_base (180,2,1024,3) + pcd_valid + T_base_ee + frame_ts       │
│              │                                                           │
│              ▼                                                           │
│  ip_runner/build_ip_demo.py:                                             │
│   1) concat axis=1 两物体 + 用 pcd_valid 去 NaN                          │
│   2) gripper_width(米)按 frame_ts 插值 → > 40mm 二值化为 grip ∈ {0,1}    │
│   3) np.linspace(0,179,10) 均匀挑 10 个 waypoint                         │
│              │                                                           │
│              ▼                                                           │
│  demo_12-12-47.pkl  =  {pcds:[10×(M,3)], T_w_es:[10×(4,4)], grips:[10]}  │
└─────────────────────────────────────────────────────────────────────────┘
```

**核心不变量:** demo 与实时两条路径都送 IP **base 帧 + 已分割** 的点云,共用同一个
`T_base_camera`(`ip_runner/calib/T_base_camera_3.3cm.npy`)。

### 1b. 替代路径:UI 内录 demo

`ip_debug_ui/` 在 skip_ip 模式下每帧从 IP server 拿回
`(mask_png, boxes_xyxy, pcd_w)`,把 `(pcd_w, T_w_e, gripper_width_m)`
按时间戳追加到 buffer。点 stop 时:

```
recording_buffer (N frames, ~3 Hz)
        │
        │  np.linspace(0, N-1, num_waypoints).astype(int)
        ▼
demo_<name>_<ts>.pkl  =  {pcds:[10×(M_i,3) f32],
                          T_w_es:[10×(4,4) f64],
                          grips:[10  (gripper_width_m > threshold)]}
```

schema 与 `build_ip_demo.py` 输出位对位等价。**不经过** dexycb_pipeline /
SAM2 video propagate / npz —— GD 在 skip_ip 模式下逐帧重跑,
所以 buffer 里的 mask 就是 inference 时会看到的 mask。
默认落在 `--demos-dir /home/franka/ip-deploy/demos/`(刻意放到 ICRT/ 之外)。
帧若 `pcd_w.shape[0] < demo_min_pcd_points`(默认 64)被丢弃。

## 2. 在线推理与控制

### 2a. Variant A — CLI executor(原始路径)

```
┌──── franka-nuc (PREEMPT_RT) ────┐         ┌──── nyu-127 (CUDA + ip_deploy env) ────┐
│                                 │         │                                          │
│  RealSense D435I (6Hz)          │         │  IP server boot:                        │
│   poll() →                      │         │   load demo.pkl → cond_demo (10 wps)    │
│   color_bgr (640×480 BGR8)      │         │   load instant_policy.so + model.pt     │
│   depth_raw (uint16, ×scale=m)  │         │   load T_base_camera_3.3cm.npy          │
│   K = (fx, fy, cx, cy)          │         │   load GroundingDINO + SAM2-image       │
│                                 │         │   bind tcp://*:5556 (ZMQ REP)           │
│  polymetis RobotInterface       │         │                                          │
│   get_ee_pose() → (pos, quat)   │         │   ┌──────────────────────────────────┐  │
│   gripper.get_state().width     │         │   │ on RESET: store prompt, clear    │  │
│                                 │         │   │   GD cache, reset episode state  │  │
│  ┌──── ip_executor/run.py ────┐ │         │   └──────────────────────────────────┘  │
│  │                            │ │         │   ┌──────────────────────────────────┐  │
│  │  loop:                     │ │         │   │ on first OBS of episode:         │  │
│  │   depth_mm = depth_raw     │ │         │   │   GD.detect(rgb, prompt)         │  │
│  │       × scale × 1000       │ │         │   │       → boxes (xyxy, label,score)│  │
│  │   T_w_e = pos+quat→4×4     │ │         │   │   cache boxes for episode        │  │
│  │   grip = (width > 40mm)    │ │         │   └──────────────────────────────────┘  │
│  │                            │ │         │   ┌──────────────────────────────────┐  │
│  │   OBS = pickle({           │ │         │   │ every OBS:                       │  │
│  │     color_bgr_jpeg,        │ │  REQ    │   │   decode JPEG + PNG-16           │  │
│  │     depth_uint16_png,      │ ├────────►│   │   SAM2.set_image(rgb)            │  │
│  │     K, T_w_e, grip,        │ │  ZMQ    │   │   for each cached box:           │  │
│  │     step, episode_id })    │ │ tcp:5556│   │     SAM2.predict(box) → mask     │  │
│  │                            │ │         │   │   union masks                    │  │
│  │   ◄ ACT                    │ ├◄────────┤   │   lift_mask(depth_mm, mask, K)   │  │
│  │                            │ │  REP    │   │       → pcd_camera (m)           │  │
│  │   leash:                   │ │         │   │   pcd_w = T_base_camera @ ...    │  │
│  │    Δpos clamp ≤ 4cm        │ │         │   │                                  │  │
│  │    Δrot clamp ≤ 17° (slerp)│ │         │   │   ┌─ IP forward ──────────────┐  │  │
│  │                            │ │         │   │   │ live = {                  │  │  │
│  │   robot.update_desired_    │ │         │   │   │   obs:[transform_pcd(     │  │  │
│  │       ee_pose(pos, quat)   │ │         │   │   │      subsample(pcd_w),    │  │  │
│  │                            │ │         │   │   │      inv(T_w_e))],        │  │  │
│  │   gripper edge-trigger:    │ │         │   │   │   T_w_es:[T_w_e],         │  │  │
│  │    if grip_cmd flipped:    │ │         │   │   │   grips:[grip] }          │  │  │
│  │     +1 → goto(80mm)        │ │         │   │   │ full = {demos:[cond_demo],│  │  │
│  │     -1 → grasp(0mm)        │ │         │   │   │         live:live}        │  │  │
│  │                            │ │         │   │   │ actions, grips_pred =     │  │  │
│  │  R2 dead-man (planned)     │ │         │   │   │   model.predict_actions() │  │  │
│  │  L1+Circle e-stop          │ │         │   │   └───────────────────────────┘  │  │
│  └────────────────────────────┘ │         │   │                                  │  │
│                                 │         │   │   T_w_next = T_w_e @ actions[0]  │  │
│   ↑                             │         │   │   target_pos = T_w_next[:3,3]    │  │
│   │ Cartesian impedance         │         │   │   target_quat = R→quat_xyzw      │  │
│   │ (50Hz internal)             │         │   │   grip_cmd = sign(grips_pred[0]) │  │
│   ▼                             │         │   │   regrasp = (sign flipped)       │  │
│  Franka FR3 (172.16.0.2)        │         │   │                                  │  │
│   joint torques + IK            │         │   │   ACT = pickle({                 │  │
│                                 │         │   │     target_pos, target_quat,     │  │
└─────────────────────────────────┘         │   │     grip_cmd, regrasp_required,  │  │
                                            │   │     step, info })                │  │
                                            │   └──────────────────────────────────┘  │
                                            └──────────────────────────────────────────┘
```

### 2b. Variant B — Web UI(2026-05-06 起)

UI 取代 executor 拥有相机 + polymetis;ZMQ 客户端逻辑搬到 `seg_loop`。
浏览器只看不算,通过 WebSocket 拿帧推回键盘事件。两种工作模式
由 OBS 里的 `skip_ip` 旗标切换。

```
┌─ local Mac ─┐    ┌──── franka-nuc ──────────────────┐    ┌──── nyu-127 ────┐
│             │    │                                   │    │                  │
│ browser     │ WS │  FastAPI (uvicorn :8000)          │    │  ip_runner.server│
│ index.html  │◄──►│  ip_debug_ui/server.py            │REQ │  (同 2a)         │
│  RGB+bbox   │    │   ├ camera_loop  (RealSense 6Hz)  │───►│   tcp://*:5556   │
│  RGB+mask   │    │   ├ seg_loop    (~3 Hz)           │    │                  │
│  pcd_w 2D   │    │   │   build OBS → ZMQ REQ         │REP │  分支:           │
│  state+log  │    │   │   apply ACT (if running_ip)   │◄───│   skip_ip=True   │
│  keyboard   │    │   ├ ws push_loop (10 Hz)          │    │     → seg reply  │
│             │    │   └ ws recv_loop                  │    │   skip_ip=False  │
│ ssh -L 8000 │    │      ee_delta / gripper / home    │    │     → act reply  │
│             │    │      record_start/stop, ip_run    │    │     (+seg if     │
└─────────────┘    │  polymetis: update_desired_ee_pose│    │      include_seg)│
                   │  gripper: goto / grasp            │    └──────────────────┘
                   │  leash 4cm / 10°                  │
                   │  demos sink: /home/franka/        │
                   │              ip-deploy/demos/     │
                   └───────────────────────────────────┘
```

两种模式:
- **debug / 录 demo 模式** —— `running_ip=False` →  OBS 带 `skip_ip=True`,
  server 只跑 GD+SAM2+lift,不调 IP model,每帧 force_gd(无缓存)。
  键盘 teleop 1cm/5° step 由 leash 钳;录制按钮把每帧 `(pcd_w, T_w_e, grip_w)`
  追加到 buffer。
- **IP run 模式** —— `running_ip=True` → OBS 带 `skip_ip=False, include_seg=True`,
  server 跑完整推理并把 seg 信息也打包回来供可视化。UI 收到 `act` 立即调
  `_apply_ip_act`(同样过 leash + edge-trigger gripper)。

## 3. 时序与频率

```
steady-state per step:
   NUC: poll ~ms, encode ~10ms, ZMQ wait ~500ms, decode + leash + polymetis ~5ms
   server: decode 15ms + SAM2 ~200ms + lift 5ms + IP forward ~250ms = ~470ms
   wall-clock loop:  ~500ms (~2 Hz inference)
   robot Cartesian impedance keeps holding latest target between updates (50Hz)
```

控制环 50 Hz、推理环 ~2 Hz。控制器在两次 ACT 之间持续 hold 上一目标姿态,
所以低推理频率不会引起抖动。

## 4. ZMQ 协议(pickle 编解码)

| 消息 | 方向 | 关键字段 |
|---|---|---|
| `RESET` | NUC → 127 | `episode_id`, `prompt`, `gd_box_threshold`, `gd_text_threshold` |
| `ACK`   | 127 → NUC | `ok`, `boxes`(GD 检出 box,首帧返回) |
| `OBS`   | NUC → 127 | `color_bgr_jpeg`, `depth_uint16_png`, `K`, `T_w_e (4,4)`, `grip ∈ {0,1}`, `step`, *(可选)* `skip_ip: bool`, *(可选)* `include_seg: bool`。`K` 与 `T_w_e` 始终对应**当前 IP 源 cam**(由 UI 的 `--ip-camera-source` 决定;wrist 模式下 server 端 `T_base_camera = T_w_e @ T_ee_camera`) |
| `ACT`   | 127 → NUC | `target_pos (3,)`, `target_quat_xyzw (4,)`, `grip_cmd ∈ {-1,0,+1}`, `regrasp_required`, `info`;若 OBS 带 `include_seg=True` 还附带 `mask_png`, `boxes_xyxy`, `box_labels`, `box_scores`, `pcd_w` |
| `SEG`   | 127 → NUC | OBS 带 `skip_ip=True` 时返回:`mask_png`, `boxes_xyxy`, `box_labels`, `box_scores`, `pcd_w`(无 IP forward,`type="seg"`) |
| `ERR`   | 127 → NUC | `msg`, `trace` — 任意服务端失败兜底 |

**GD 缓存语义:**
- `skip_ip=False`(常规 ACT)— 首个 OBS 跑 GD 缓存 boxes,后续 OBS 复用同一组 box 喂 SAM2。
- `skip_ip=True`(UI debug / 录 demo)— `force_gd`,**每帧**重跑 GroundingDINO,
  这样 demo 录制时移动物体,box 会跟随。代价是每帧多 ~150ms。

## 5. 关键文件

**nyu-127:**
- `~/zhewen/robo/ip-deploy/ip_runner/build_ip_demo.py` — 离线 demo 构造
- `~/zhewen/robo/ip-deploy/ip_runner/preproc.py` — GD-once + SAM2-image + obs_to_pcd_w
- `~/zhewen/robo/ip-deploy/ip_runner/server.py` — ZMQ REP + IP 推理主循环
- `~/zhewen/robo/ip-deploy/ip_runner/calib/T_base_camera_3.3cm.npy` — provisional 标定
- `~/zhewen/robo/ip-deploy/ip_runner/demos/demo_12-12-47.pkl` — IP demo
- 复用: `~/zhewen/robo/dexycb_pipeline/dexycb_pipeline/object_pcd.py:lift_mask`
- 复用: `~/zhewen/robo/ip-deploy/instant_policy/instant_policy.so + checkpoints/model.pt`
- 复用: `~/zhewen/sam2/sam2/sam2_image_predictor.py:SAM2ImagePredictor`

**franka-nuc — Variant A (CLI executor):**
- `/home/franka/ICRT/ip_executor/run.py` — 主循环,RealSense + polymetis + ZMQ client
- `/home/franka/ICRT/ip_executor/client.py` — ZMQ REQ + JPEG/PNG 编码
- `/home/franka/ICRT/ip_executor/safety.py` — 4cm/17° leash + slerp
- `/home/franka/ICRT/ip_executor/codec.py` — 与 ip_runner.codec wire-compatible
- 复用: `/home/franka/ICRT/show_rgbd.py:RealSenseCamera`
- 复用: polymetis `RobotInterface` (50051) + `GripperInterface` (50052)

**franka-nuc — Variant B (Web UI):**
- `/home/franka/ICRT/ip_debug_ui/server.py` — FastAPI + camera_loop + seg_loop + WS handler
- `/home/franka/ICRT/ip_debug_ui/static/index.html` + `*.js` — 浏览器端 RGB/mask/pcd panes
- `/home/franka/ICRT/ip_debug_ui/README.md` — UI 操作手册(键位、录 demo 流程)
- 复用 `ip_executor/{codec,safety}` 与 `show_rgbd.py:RealSenseCamera`
- 录 demo 默认目录:`/home/franka/ip-deploy/demos/`(刻意放在 `ICRT/` 之外,
  ICRT/ 是 collaborator 树)

## 6. 启动指令

> SSH 注释:`franka-nuc` 别名走 `chatsign-jump`,2026-05-06 该跳板下线;
> 当前可用路由是 `franka-backup`(直连 `franka@10.224.36.29:10003`,通过 nyu-127
> 端口转发,key auth 已配)。下面命令统一写 `franka-backup`。
> 详情见 `~/.claude/projects/.../memory/franka_nuc_ssh_routes.md`。

**nyu-127 启 server(两条路径共用):**

二选一传相机标定。`--T-base-camera` 是 front-cam 静态外参(老路径);
`--T-ee-camera` 是 wrist-cam 手眼标定,server 每帧用 `T_w_e @ T_ee_camera`
重算 T_base_camera。两者互斥,必须传恰好一个。

```bash
ssh nyu-127
source ~/miniconda3/etc/profile.d/conda.sh && conda activate ip_deploy
cd ~/zhewen/robo/ip-deploy

# Variant 1 — front cam (静态外参,老路径)
python -m ip_runner.server \
  --bind tcp://*:5556 \
  --demo ip_runner/demos/demo_12-12-47.pkl \
  --checkpoint ./instant_policy/checkpoints/model.pt \
  --instant-policy-dir ./instant_policy \
  --T-base-camera ip_runner/calib/T_base_camera_3.3cm.npy

# Variant 2 — wrist cam (动态外参,2026-05-10 新路径)
python -m ip_runner.server \
  --bind tcp://*:5556 \
  --demo ip_runner/demos/demo_12-12-47.pkl \
  --checkpoint ./instant_policy/checkpoints/model.pt \
  --instant-policy-dir ./instant_policy \
  --T-ee-camera /home/nyuair/zhewen/calibration/T_ee_camera.npy
```

`T_ee_camera.npy` 需 rsync 到 nyu-127(源:Mac `calibration/wrist-camera/T_ee_camera.npy`)。


### 6a. Variant A — CLI executor

**dry-run 验证:**
```bash
ssh franka-backup
source /home/franka/conda/etc/profile.d/conda.sh && conda activate polymetis-local
cd /home/franka/ICRT
python -m ip_executor.run \
  --server tcp://10.224.36.127:5556 \
  --prompt "yellow glue stick . wooden cube . wooden block ." \
  --dry-run --max-steps 20
```

**实机闭环(去 `--dry-run`,可选 `--enable-gripper`):**
```bash
python -m ip_executor.run \
  --server tcp://10.224.36.127:5556 \
  --prompt "yellow glue stick . wooden cube . wooden block ." \
  --max-steps 50
```

### 6b. Variant B — Web UI

**NUC 启 UI(front cam 默认):**
```bash
ssh franka-backup
source /home/franka/conda/etc/profile.d/conda.sh && conda activate polymetis-local
cd /home/franka/ICRT
PYTHONPATH=/home/franka/ICRT python -m ip_debug_ui.server \
  --ip-server tcp://10.224.36.127:5556 \
  --prompt "red cube . green cube . blue cube . yellow cube ."
```

**NUC 启 UI(wrist cam 作 IP 源):** server 端要用 Variant 2 启动。
启动后可在 UI topbar `camera` 下拉框运行时切换 front ↔ wrist。
```bash
PYTHONPATH=/home/franka/ICRT python -m ip_debug_ui.server \
  --ip-server tcp://10.224.36.127:5556 \
  --camera wrist \
  --alt-width 640 --alt-height 480 --alt-fps 30 \
  --prompt "red cube ."
```

**本地 Mac 端口转发 + 开浏览器:**
```bash
ssh -L 8000:localhost:8000 franka-backup
open http://localhost:8000
```

进入后:debug 模式自动开始(skip_ip),按 record 录 demo,或按 IP run 切到闭环。
键位与录制流程见 [`ui.md`](./ui.md) 与 `ip_debug_ui/README.md`。

## 7. 已知限制

- `T_base_camera` 是 provisional 3.3cm 标定(残差 ~1.02cm)
- 单相机,无多视角融合
- IP `num_demos = 1`,只有 1 个 demo(可能弱化 in-context 信号)
- horizon 当前只取 `actions[0]`,未做"执行到 grip 翻转再 query"优化
- 未集成 PS4 dead-man(R2 / L1+Circle e-stop 是计划中)
- **IP 1-shot 本身的能力上限**:2026-05-06 实测同 demo 的开环 + UI 录制
  闭环都难以稳定复现 pick-and-place,任务结构信号不足。详见
  [`review/problem-05-06.md`](./review/problem-05-06.md)。
- **录 demo 时 gripper 触底**:闭合时若指尖之间没有物体直接夹紧,polymetis
  的 grasp 把 finger 卡在物理零点,后续 open 命令延迟 / 失效;录 demo 中段
  应避免 close 空气。
- **grip 预测过平滑**:demo 里 wp4→wp5 的 close 翻转在预测中被磨成连续值,
  靠 `abs(grip_next) > 0.1` 阈值 + sign() 离散化,容易在边界附近抖动而错过翻转时刻。
