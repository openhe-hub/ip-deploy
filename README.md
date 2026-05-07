# ip-deploy

Instant Policy 在 Franka FR3 上的实机部署层。三个独立模块,跑在两台机器上,
通过 ZMQ 协议串起一条 RGB-D → 分割 → IP 推理 → 笛卡尔阻抗控制 的闭环。

完整设计与时序见 [`docs/pipeline.md`](docs/pipeline.md);
浏览器 UI 的细节见 [`docs/ui.md`](docs/ui.md);
当前的实测局限见 [`docs/review/problem-05-06.md`](docs/review/problem-05-06.md)。

## 仓库布局

```
ip-deploy/
├── ip_runner/        # nyu-127 (GPU + ip_deploy env)
│   ├── server.py            # ZMQ REP @ tcp://*:5556 — IP 推理主循环
│   ├── preproc.py           # GD-once + SAM2-image + lift_mask → pcd_w
│   ├── build_ip_demo.py     # 离线 demo 构造 (npz → pkl, 10 waypoints)
│   ├── codec.py             # JPEG / PNG-16 / pickle wire 编解码
│   ├── calib/               # T_base_camera 标定文件
│   ├── demos/               # IP demo (.pkl) 落盘目录 (本地 gitignored)
│   └── tests/               # smoke / check_demo
│
├── ip_executor/      # franka-nuc (PREEMPT_RT + polymetis-local) — CLI 路径
│   ├── run.py               # RealSense + polymetis + ZMQ client 主循环
│   ├── client.py            # ZMQ REQ + 编码
│   ├── safety.py            # leash: 4cm / 17° (slerp)
│   ├── reset_to_home.py     # 关节空间归位
│   └── codec.py             # 与 ip_runner.codec wire-compatible
│
├── ip_debug_ui/      # franka-nuc — Web UI 路径 (2026-05-06 起)
│   ├── server.py            # FastAPI: camera_loop + seg_loop + WS
│   ├── static/              # index.html / main.js / style.css
│   └── README.md            # 启动 + 录 demo 简介
│
└── docs/
    ├── pipeline.md          # 主参考: 离线/在线管线 + ZMQ 协议 + 启动指令
    ├── ui.md                # UI 架构 / 模式切换 / 键位 / 录 demo 流程
    └── review/              # 实测复盘
```

## 两条执行路径(共用同一个 IP server)

| 路径 | 入口 | 用途 |
|---|---|---|
| **A. CLI executor** | `python -m ip_executor.run` | 原始无头闭环;最快验证 IP 推理;无可视化 |
| **B. Web UI** | `python -m ip_debug_ui.server` + 浏览器 | 实时看 GD/SAM2 输出;键盘 teleop;直接录 IP demo;UI 内点按钮启 IP 闭环 |

两者通过 ZMQ REQ/REP 与 nyu-127 上的 `ip_runner.server` 对话,
协议见 [`docs/pipeline.md` §4](docs/pipeline.md#4-zmq-协议pickle-编解码)。

## 核心数据约定

- **基坐标系点云**:demo 与实时两条路径都送 IP **base 帧 + 已分割** 的点云,
  共用同一份 `T_base_camera`(见 `ip_runner/calib/`)。
- **Demo 格式**:`{pcds: [10×(M,3) f32], T_w_es: [10×(4,4) f64], grips: [10 ∈ {0,1}]}`,
  10 个均匀路标。`build_ip_demo.py`(离线)与 UI 录制(在线)产出位对位等价。
- **Gripper 二值化阈值**:width > 40 mm → open(`grip=1`),否则 close(`grip=0`)。

## 快速启动

完整命令(含 conda env、SSH 别名、端口转发)见 [`docs/pipeline.md` §6](docs/pipeline.md#6-启动指令)。
最简单的烟雾测试:

```bash
# nyu-127: 启 IP server
python -m ip_runner.server \
  --bind tcp://*:5556 \
  --demo ip_runner/demos/demo_12-12-47.pkl \
  --checkpoint ./instant_policy/checkpoints/model.pt \
  --instant-policy-dir ./instant_policy \
  --T-base-camera ip_runner/calib/T_base_camera_3.3cm.npy

# franka-nuc: dry-run 验证 (不动机器人)
python -m ip_executor.run \
  --server tcp://10.224.36.127:5556 \
  --prompt "yellow glue stick . wooden cube . wooden block ." \
  --dry-run --max-steps 20
```

## 依赖(未在本仓库)

- **Instant Policy** 二进制 + checkpoint(`instant_policy.so` + `model.pt`)
- **GroundingDINO**(GD-once 物体检测)
- **SAM2**(逐帧 image predictor 用于 segmentation refine)
- **dexycb_pipeline**(离线 demo 构造时复用其 `lift_mask`)
- **polymetis-local**(franka-nuc 上的实时控制端,Cartesian impedance @ 50 Hz)

env 在 nyu-127 是 `ip_deploy`,在 franka-nuc 是 `polymetis-local`。

## 运行频率

控制环 50 Hz(polymetis 内部维持);推理环 ~2 Hz(单次 ZMQ round-trip ≈ 500 ms,
其中 SAM2 ~200 ms + IP forward ~250 ms)。控制器在两次 ACT 之间持续 hold 上一目标,
所以低推理频率不会引起抖动。

## 已知限制

简要列在 [`docs/pipeline.md` §7](docs/pipeline.md#7-已知限制),核心是:

- `T_base_camera` 仍是 provisional 标定(残差 ~1 cm 量级)
- `num_demos = 1` 的 IP 1-shot 在 pick-and-place 上可重复性差(2026-05-06 实测)
- 单相机、无多视角、无 PS4 dead-man

## 文档导航

- 看管线 / 协议 / 启动 → [`docs/pipeline.md`](docs/pipeline.md)
- 看 UI 操作 → [`docs/ui.md`](docs/ui.md) 或 [`ip_debug_ui/README.md`](ip_debug_ui/README.md)
- 看复盘 → [`docs/review/`](docs/review/)
