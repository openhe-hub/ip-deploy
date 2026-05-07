# IP debug UI

`ip_debug_ui/` —— 2026-05-06 加入的浏览器端调试 + 录 demo + IP 闭环工具,与
原 CLI executor (`ip_executor/run.py`) 并列,共用同一个 IP server (ZMQ REP @
`tcp://*:5556`)。

## 1. 用途

1. **实时观察** GD + SAM2 的分割效果(box / mask / pcd_w)边 teleop 边看,
   不必先录完再 dexycb_pipeline 离线检查。
2. **直接录 IP demo**,跳过 dexycb_pipeline + `build_ip_demo.py`:录的就是
   IP 在 inference 时会看到的 segmentation,demo 与实测条件按构造一致。
3. **IP 闭环** 控制时把 segmentation 视图带回来,出问题立刻能看见 mask 漂没漂、
   pcd_w 是不是在桌面上。

## 2. 架构

```
┌────────────── local Mac ──────────────┐    ┌──────── franka-nuc ─────────┐
│  browser @ http://localhost:8000      │    │  FastAPI ip_debug_ui.server │
│   - 4 panels (RGB+box, RGB+mask,      │    │   - RealSenseCamera 6 Hz    │
│     pcd_w top-down, state+kbd+log)    │    │   - polymetis Robot/Gripper │
│   - keyboard teleop, topbar buttons   │    │   - cartesian impedance     │
│        ▲                              │    │        ▲                    │
│        │ ws://…/ws  (~10 Hz push)     │    │        │ get_ee_pose,       │
│        │                              │    │        │ get_state, poll    │
│        └──── ssh -L 8000:…:8000 ──────┼────┤────────┘                    │
└───────────────────────────────────────┘    │  seg_loop ~3 Hz             │
                                             │   ZMQ REQ ─────────┐        │
                                             └────────────────────┼────────┘
                                                                  │
                                                       tcp://10.224.36.127:5556
                                                                  │
                                             ┌────── nyu-127 ─────▼────────┐
                                             │  ip_runner.server (REP)     │
                                             │   skip_ip=True  → "seg"     │
                                             │     re-runs GD every frame  │
                                             │   skip_ip=False             │
                                             │     + include_seg=True      │
                                             │     → "act" + bundled seg   │
                                             └─────────────────────────────┘
```

`seg_loop` 每 ~300 ms 发一条 OBS,根据 `running_ip` 状态切换 `skip_ip` 与
`include_seg` 两个 flag。WS push_loop 在 ~10 Hz 把最新一帧 RGB + 上一次 seg
缓存合并送给浏览器。

## 3. 文件

| 路径 | 作用 |
|---|---|
| `ip_debug_ui/server.py` | FastAPI 后端,跑在 franka-nuc。摄像头 + polymetis + ZMQ client + 录 demo + IP 闭环。 |
| `ip_debug_ui/static/index.html` | 顶栏 + 2×2 grid(RGB+box / RGB+mask / pcd_w / state+keyboard+log)。 |
| `ip_debug_ui/static/main.js` | WS client、canvas 绘制、键盘 teleop(IP 闭环时 EE 锁但 gripper / R / T 放行)。 |
| `ip_debug_ui/static/style.css` | 暗色主题 + 录制/运行按钮的脉冲动画 + 键位高亮。 |
| `ip_debug_ui/README.md` | 简短启动 + 录 demo 指引;此文件是更全面的参考。 |

## 4. 两种运行模式

| 模式 | `skip_ip` | `include_seg` | server reply | 键盘 |
|---|---|---|---|---|
| **Debug**(默认) | `True` | (server 行为同 skip) | `seg` —— mask + boxes + pcd_w,GD 每帧重跑 | 全开,EE / 旋转 / gripper / R / T 都生效 |
| **IP run**(点 `▶ run IP`) | `False` | `True` | `act` —— `target_pos / target_quat_xyzw / grip_cmd`,附带 seg 视图 | EE 平移/旋转锁;gripper、R(home)、T(straighten)放行作为人工救援 |

切换由 `running_ip` flag 控制,`▶ run IP` 按钮在 IP 闭环开启时绿色脉冲,
按钮文字变成 `■ stop IP (<step>)`。

`_apply_ip_act` 拿到 act 后:
- 用 `safety.leash` 把 `(target_pos, target_quat)` 相对当前 measured pose
  限幅到 `--max-pos-step-m`(默认 4 cm)+ `--max-rot-step-deg`(默认 10°),
- `robot.update_desired_ee_pose(pos_t, quat_t)`,
- `grip_cmd ∈ {-1, 0, +1}` 做边沿触发,`+1 → goto(0.08, 0.1, 30)`,
  `-1 → grasp(0.1, 30, 0)`,fire-and-forget(不阻塞控制环)。

## 5. 键位

| 键 | 动作 | frame |
|---|---|---|
| W / S | x ±2 cm | base |
| A / D | y −2 cm / +2 cm(注意 A=−y, D=+y) | base |
| Q / E | z +2 cm / −2 cm | base |
| ↑ / ↓ | pitch ±10° | EE |
| ← / → | roll ±10° | EE |
| J / L | yaw ±10° | EE |
| Shift(按住) | 步长 ×3 | — |
| Ctrl(按住) | 步长 ×0.2 | — |
| Space | gripper open(`goto(0.08, 0.1, 30)`) | — |
| Shift+Space | gripper close(`grasp(0.1, 30, 0)`) | — |
| R | home —— 关节空间 `move_to_joint_positions(HOME_Q_RAD, 4 s)` 后重启 cartesian impedance,gripper goto 0.08 m | — |
| T | straighten —— 保持当前 xyz,quat 直接覆盖为 `HOME_QUAT_XYZW`(夹爪 z 轴指向桌面) | base+EE |

IP run 时:`KEY_DPOS` / `KEY_DRPY` 都被吞,但 Space / Shift+Space / R / T
仍然透传 —— 用于"IP 把开手压进桌面了立刻 close 救一下"。

## 6. 顶栏

| 控件 | 作用 |
|---|---|
| status | `connecting…` / `connected`(绿) / `disconnected — retrying`(红);WS 自动 1.5 s 重连。 |
| `prompt-input` | GD 文本 prompt,默认随服务端 `--prompt` 回填。`reset episode` 时随消息发出。 |
| `thresh` | GD 的 `box_threshold` 与 `text_threshold`,**两者用同一值**,默认 0.40。 |
| `reset episode` | 发 `{type:"reset_episode", prompt, gd_box_threshold, gd_text_threshold}`,服务端清 `last_seg`、生新 `episode_id`,server 端清 GD 缓存。 |
| `demo-name` | 可选,会进文件名:`demo_<name>_<ts>.pkl`。 |
| `● record` | 开始录;录制时红色脉冲,文字 `■ stop & save (N)`;再点保存。 |
| `▶ run IP` | 启 IP 闭环;有 `confirm("start IP closed-loop? robot will move autonomously")` 二次确认;运行时绿色脉冲。 |
| `straighten (T)` / `home (R)` / `open (Space)` / `close (Shift+Space)` | 与键盘等价的按钮入口。 |

state 面板实时显示:`ee_pos`、`ee_quat`、gripper width(mm)、box 数、`pcd_n`、
`seg_age`(ms,上次 seg reply 至今)、浏览器 fps、recording 状态、IP run 步数。

## 7. 录 demo 流程

1. 顶栏填 `demo-name`(可选),点 `● record`。
2. 服务端在 `seg_loop` 里每个 reply 都会 append `(pcd_w, T_w_e, gripper_width_m)`
   到 `recording_buffer`,频率 ≈ 3 Hz。
3. 帧若 `pcd_w.shape[0] < --demo-min-pcd-points`(默认 64)被丢弃。
4. teleop 把场景演完,点 `■ stop & save (N)`。

保存路径:
- 默认 `--demos-dir = /home/franka/ip-deploy/demos/`(故意不放 `ICRT/`,
  那是合作方的工作树)。
- 文件名 `demo_<name>_<YYYY-MM-DD_HH-MM-SS>.pkl`(name 为空就只剩时间戳)。

落盘格式:
```python
{"pcds":   [np.ndarray(M_i, 3) float32, ...],   # len = num_waypoints
 "T_w_es": [np.ndarray(4, 4)   float64, ...],   # polymetis flange in base
 "grips":  [int  (1 if gripper_width_m > --gripper-threshold-m else 0)]}
```
与 `build_ip_demo.py` 输出位对位等价。

下采样:`np.linspace(0, len(buffer)-1, --demo-num-waypoints).astype(int)`(默认 10 路标点)。
若 `len(buffer) < --demo-min-frames`(默认 20),保存被拒,前端 log 报
`save FAILED: too few frames (... < 20)`。

**为什么这能跳过 dexycb_pipeline:** `seg_loop` 在 skip_ip 模式下让 server 每帧
重跑 GD + SAM2,所以 buffer 里的 `pcd_w` 已经是 base 帧 + 已分割的点云,
schema 与 IP 推理需要的一致。Demo-time 的 segmentation **就是** inference-time
会发生的 segmentation,两条路径同源。

## 8. 录制质量经验(2026-05-06)

- **别把开口夹爪压到方块/桌面**:留 5–10 mm 间隙(`z ≈ cube_top + 0.005~0.010 m`)。
  夹爪压实后位移测量噪音飙升,IP 拿到的 grip head 信号也乱。
- **抓取相位要"暂停 + 关爪"**:xyz 保持不动几帧,纯做 gripper close,这样 IP 能
  清楚识别"这一帧是关爪 moment",不会把关爪和位移混在一起学。
- **真做 pick-and-place 必须含 xy 平移**。当天第一版只录了 z 轴 pick + drop,
  在桌面同一点起落 —— IP 推理时无判别力,基本无效。
- **长度**:目标 ≥ 20 个 seg tick(@3 Hz 即 ≥7 s),给 10 路标点的均匀下采样留余量;
  否则 `--demo-min-frames=20` 直接拒收。

## 9. 启动 + 隧道

```bash
# 1) nyu-127: IP server(同 CLI 路径,见 pipeline.md)
ssh nyu-127
source ~/miniconda3/etc/profile.d/conda.sh && conda activate ip_deploy
cd ~/zhewen/robo/ip-deploy
python -m ip_runner.server --bind tcp://*:5556 \
  --demo ip_runner/demos/demo_grasp_red.pkl \
  --checkpoint ./instant_policy/checkpoints/model.pt \
  --instant-policy-dir ./instant_policy \
  --T-base-camera ip_runner/calib/T_base_camera_3.3cm.npy \
  --dexycb-pipeline-dir /home/nyuair/zhewen/robo/dexycb_pipeline

# 2) franka-nuc: UI(用 franka-backup 别名,不是 franka-nuc;详见 review/problem-05-06.md)
ssh franka-backup
source /home/franka/conda/etc/profile.d/conda.sh && conda activate polymetis-local
cd /home/franka/ICRT
PYTHONPATH=/home/franka/ICRT python -m ip_debug_ui.server \
  --ip-server tcp://10.224.36.127:5556 \
  --prompt "red cube . green cube . blue cube . yellow cube ."

# 3) local Mac: 隧道 + 浏览器
ssh -L 8000:localhost:8000 franka-backup
open http://localhost:8000
```

## 10. Troubleshooting

| 症状 | 处理 |
|---|---|
| RealSense `errno=16 EBUSY` | `echo q1w2 \| sudo -S modprobe -r uvcvideo && echo q1w2 \| sudo -S modprobe uvcvideo`,然后重启 UI。 |
| ZMQ REQ 卡住(server 重启后) | UI 在 zmq err / decode 失败时会调 `state.reconnect_zmq()` 自愈;若仍卡住,Ctrl-C 重启 UI。 |
| reset 后 box / mask 显示旧帧 | 已修:`reset_episode` 现在会把 `state.last_seg = None`,新一次 seg reply 落地前不再渲染。 |
| 按钮 / 键盘没反应 | 浏览器焦点不在 document 上,先点一下任意 canvas 再按键。 |
| `'asyncio' object has no attribute 'to_thread'` | 服务端 Python 3.8 没这 API,`server.py` 顶部 shim 已经 polyfill,正常情况看不到此错。 |
| GD 没框 → `too few points after segmentation` | 调低 `thresh`(默认 0.40),或换 prompt。注意 `prompt-input` 里要带 `.` 分隔符,例如 `red cube . green cube .`。 |

## 11. 交叉引用

- [`pipeline.md`](./pipeline.md) §1b、§4 —— UI 录 demo 与 ZMQ 协议。
- [`review/problem-05-06.md`](./review/problem-05-06.md) —— 当天遇到的具体问题
  (含 `franka-backup` vs `franka-nuc` 别名混淆、第一版 demo 无 xy 位移等)。
