# 2026-05-06 IP 实机部署 postmortem

本日目标:在 Franka 上把 Instant Policy 闭环跑起来,复现一次最简单的
pick-and-place。结论:**没复现成功**。本文复盘当天踩到的基础设施坑、
实现的工具改动、GD prompt 调参过程,以及三次 IP 闭环的失败模式与
诊断假设。区分**已验证事实**与**假设**两类断言。

## 1. TL;DR

- **可用的:** 整条 segmentation + ZMQ + polymetis 通路通了;新建的
  `ip_debug_ui` 能 teleop、能录 demo、能跑 IP 闭环;GD prompt 调到
  `red/green/blue/yellow cube` + threshold 0.40 后 4 个 box 干净
  (scores 0.55–0.67)。
- **不可用的:** 三次 IP 闭环都没闭上 —— 要么 EE 漂移幅度过小、
  要么 gripper 死开砸到桌面;`grip_cmd` 整段恒为 +1 (open) 没翻转。
- **头号开放问题:** IP 在所有三次尝试里都不输出 close 指令 ——
  是 1-shot conditioning 不够、还是 EE/base 帧搞反、还是 demo 录得
  太浅没拉开 phase 信号?**需要 dump v2 第三次的 action 序列才能
  分清。**

## 2. 当日目标

把 Franka 上的 IP 闭环跑通到能从桌面抓起一块木块再放下。本地 macbook
启浏览器 UI、franka-NUC 跑 polymetis + 相机 + WS server、nyu-127
跑 IP 推理 server。把昨天离线 demo 路径搬到 UI 内录 demo,这样可以
现场录、立刻跑;不依赖 dexycb_pipeline + SAM2 video propagate
+ build_ip_demo 的 offline 链。

## 3. 基础设施问题

### 3.1 SSH 链路:chatsign-jump 失联

`franka-nuc` 这个原始 alias 走 `chatsign-jump` (10.228.229.127) 跳板。
当日 ping 超时、port 22 拒绝连接;退回早先备用 `lab-macbook 10.228.204.221`
也无应答。

User 给了新通路:`franka@10.224.36.29:10003`(lamuda 端口转发,经过
nyu-127)。在 `~/.ssh/config` 加 `franka-backup` alias、push 本地
`id_ed25519.pub` 到 NUC 的 `authorized_keys` 实现免密登录。NUC
sudo 密码 `q1w2`(passwordless sudo 没开)。这两条都进了
`memory/franka_nuc_ssh_routes.md`。

### 3.2 RealSense `errno=16 EBUSY`

上一次 session 中 librealsense 进程异常退出后,kernel `uvcvideo`
驱动把 V4L2 端点卡在 busy 状态。`device.hardware_reset()` 不解决。

修复:`echo q1w2 | sudo -S modprobe -r uvcvideo && echo q1w2 | sudo -S modprobe uvcvideo`
(免密 sudo 没开,只能 stdin 喂密码)。已写进 ui README 的
Troubleshooting。

### 3.3 Python 3.8 缺 `asyncio.to_thread`

`polymetis-local` 是 Python 3.8 环境,`asyncio.to_thread` 是 3.9
新增。`ip_debug_ui/server.py` 顶部加 shim:

```python
if not hasattr(asyncio, "to_thread"):
    async def _to_thread(fn, *args, **kwargs):
        loop = asyncio.get_event_loop()
        return await loop.run_in_executor(
            None, functools.partial(fn, *args, **kwargs))
    asyncio.to_thread = _to_thread
```

代价:多一层 partial 包装,语义等价。

### 3.4 ZMQ REQ socket recovery

UI 的 `seg_loop` 用 ZMQ REQ socket 跟 nyu-127 通信。REQ 严格交替
send/recv,如果 server 中途被 kill,REQ 状态机会卡在
"等 reply" 阶段,后续 send 报 `Operation cannot be accomplished
in current state`。

修复:`State.reconnect_zmq()` 关旧 socket 重建。`seg_loop` 的
send/recv 跟自动 reset 都包了 try/except,捕获到任何 zmq 异常就
重连(并清空 `episode_id` 触发新 episode 的 RESET 握手)。

## 4. 当天构建的管线改动

### 4.1 `ip_runner/server.py` 新增 `skip_ip` + `force_gd`

OBS 消息加可选字段 `skip_ip: bool` 和 `include_seg: bool`:

- `skip_ip=False`(默认,与昨天行为一致)— 首帧 GD 缓存 box,后续
  帧只跑 SAM2 复用 box,返回 `act`。
- `skip_ip=True` — server 不走 IP forward,直接回一个 `seg`
  reply (`mask_png`, `boxes_xyxy`, `box_labels`, `box_scores`,
  `pcd_w`)。**且每帧重跑 GD**,这样录 demo 时移动物体 box 会跟着
  挪。代价是每帧多 ~150 ms。
- `include_seg=True`(IP run 模式启用)— 走完 IP forward 之后把
  seg viz 也打包进 `act` reply,UI 可以一边跑一边看 mask/bbox。

> **代码备注:** 检索此仓库 `/Users/zhewen/Workspace/robo/ip-deploy/ip_runner/server.py`
> 当前内容并不包含 `skip_ip` / `include_seg` 分支,只有原始的
> "GD-once + IP forward 返回 act" 路径。**ui/README 与 pipeline.md 都假定这两个
> 字段在 server 上已实装,实际部署的 server 应当在 nyu-127 的工作树里。
> 待验证:本地仓库的 server.py 是否落后于 nyu-127 上跑的版本。**

### 4.2 `ip_debug_ui/`:从零开始的浏览器 UI

FastAPI + uvicorn 起在 NUC 上,本地 Mac `ssh -L 8000:localhost:8000`
转发。架构详见 `docs/pipeline.md` §2b 与 ui README。当日按以下顺序
增量加进去:

1. 基础流:camera_loop 拉 RealSense + polymetis pose,seg_loop 每
   0.3 s 跑一次 skip_ip request,push_loop 10 Hz 推浏览器。
2. 键盘 teleop(WASD/QE = 1 cm base,arrows/JL = 5° EE)。
3. `reset_episode` + 在 UI 里改 prompt / GD threshold。
4. Demo recording。
5. `straighten` (T 键,把 EE quat 对齐到 home 方向但保持位置)。
6. IP run mode (`running_ip` 切换 OBS 的 `skip_ip` 标志)。
7. IP run 中允许手动按 Space / Shift+Space 强制开/合 gripper(用于
   Attempt B 之后想做应急救场)。

### 4.3 UI 内录 demo

走一条全新的、绕开 dexycb_pipeline + SAM2 video propagate 的 demo
构造路径。skip_ip 模式下 server 每帧返回 `pcd_w`,UI 把
`(pcd_w, T_w_e, gripper_width_m)` 按时间戳追加到 buffer。点 stop
后 `np.linspace(0, N-1, n_wp).astype(int)` 均匀挑出
`--demo-num-waypoints`(默认 10)条,pickle 成与 `build_ip_demo.py`
位对位等价的 schema。

落点:`/home/franka/ip-deploy/demos/`(刻意放在 ICRT/ 之外,
ICRT/ 是 collaborator 树,不污染对方代码)。

## 5. GD detection 调参 narrative

### 5.1 prompt 0.18:Franka 臂被误检

初始用旧 demo 的 prompt
`"yellow glue stick . wooden cube . wooden block ."`,
threshold 0.18。Score 0.405 的一个 box 把整条银白色 Franka 臂
匹配进去。结果:pcd_w 里 ~22 k 个手臂点 + ~3 k 个真正的 cube 点
混在一起,IP 拿到的几何完全不是它该看的。

### 5.2 `wooden cube .` + 0.18:臂仍残留

只留 `wooden cube .`,4 个 cube 进 box,但同时多了一个 score 0.190
的低置信 box 仍然贴着臂。

### 5.3 threshold 0.30:漏检 cube

抬到 0.30,臂 box 掉了,但只检出 4 个 cube 中的 2 个 —— 另两个
score 在 0.20–0.28 区间被一刀切掉。

### 5.4 prompt 颜色化 + 0.40:成功

最终 prompt
`"red cube . green cube . blue cube . yellow cube ."` +
threshold 0.40,4 个 box 干净落到对应 cube,scores 0.55–0.67。
颜色名给 GD 提供了更窄、更具区分性的 text feature,所以即使阈值
拉高也都能过线。

**经验:** GD 的 prompt 不只是 "对象类别",而是 detector 训练分布
里的语言 prior。"wooden cube" 的视觉先验跟亮金属臂部分接近;
显式颜色词把检测从形状-prior 转移到颜色-prior,误检率立降。

## 6. 三次 IP 闭环失败的细节

| Attempt | demo | 现场 | 观察 | 失败模式 |
|---|---|---|---|---|
| A | `demo_grasp_red.pkl`(collaborator 录的 task_001,红 cube ≈ (0.396, +0.156)) | 4 块 cube 在 (0.50, -0.10) 与 (0.58, -0.16) 附近 | 60 步 EE 漂移 < 5 cm,主要往 +y 方向蠕动 | scene 与 demo xy 不一致,1-shot IP OOD,EE 不到目标 |
| B | UI 录的 `demo_recorded.pkl`,原地拿起再放下 | 同一 cube,无 xy 平移 | EE 顺利下降到 z=0.381;gripper 始终开;砸到桌面 | `grip_cmd=+1` for all 41 steps,从未翻转到 close |
| C | UI 录的 `demo_v2.pkl`,phase 更清晰 (z=0.392, wp3-4-5 hold-close-lift) | 同 B | 据 user 报告仍然砸桌 | 跟 B 类似,但 dump 没拉回来,只能等下次 session 验证 |

**已验证事实:**
- A 中 demo 与 live 的 cube xy 偏差 > 10 cm;`memory/ip_demo_scene_match.md`
  早就记录过 1-shot OOD 这个 failure mode,这次再次复现。
- B 中 server log 全 41 步 `grip_cmd=1`(从 server 侧 stdout 直接看到)。

**未验证 / 假设:**
- B 中 EE "想下" 是因为 IP 直接输出 `action0` 还是因为 demo 里
  最低点 z 比当前更低 —— **需要 dump v2 第三次的 actions 才能确认**。
- C 中究竟是同一 failure mode 重复,还是新 mode(比如 quat 漂移到
  不再垂直),**dump 没拉,无法判断**。

## 7. 诊断:为什么 1-shot IP 一直闭不上

下面区分**已验证**和**假设**两层。

### 7.1 已验证:scene-EE 匹配在 1 demo 下很弱

`memory/ip_demo_scene_match.md` 记录的 OOD failure mode 在 Attempt A
精确复现。当 live cube xy 偏离 demo waypoint 的 pcd geometry
> 10 cm 时,IP 输出近零 delta,EE 蠕动。诊断步骤(看 dump 比对
top-down bbox)有效。

### 7.2 假设但有强证据:`grip_cmd` 输出过于平滑、恒为 +1

server 侧每步打印 `grip_cmd=1` 持续 41 步(B)。

`server.py` 里:
```python
grip_next = float(np.asarray(grips_pred[0]).item())
grip_cmd = int(np.sign(grip_next)) if abs(grip_next) > 0.1 else 0
```

如果 IP 模型的 `grips_pred[0]` 始终在 +0.9 到 +1.0 之间,符号永远
取正,就永远出 +1。1 个 demo 里只有 1 处 wp4→wp5 的 open→close
转折,模型几乎没机会学到何时 flip,默认输出 "保持 open" 是合理的
推断。

**待验证:** dump `grips_pred[0]` 的浮点原值序列(不只是 sign)。
如果整段都在 +0.95 以上,confirms hypothesis。如果在 ±0.3 之间
来回跳但被 0.1 死区清零,那是另一个故事(grip prediction 实际是
模糊的,只是 threshold 把它切平了)。

### 7.3 假设:动作的 frame 约定容易误读

`server.py` 里 `T_w_next = T_w_e @ action0`,所以 `action0` 是
**EE 帧 delta**(右乘),不是 base 帧。当 EE 朝下抓取时,EE 的
+z 轴沿夹爪方向、跟 base +z 反向。所以 server log 上看 `action0[2,3]
= +0.01` 实际是 base 帧下降 1 cm 而不是上升。

**这块 server 既没在日志里标注 frame、UI 也没 decode 出 base-frame
delta 来显示,所以肉眼读 log 容易判错方向。**

**待验证:** 找一个明确朝下的 EE 配置,人为输入一个 +z EE-frame
delta、看 base z 是涨是落。或者从代码里数学证一遍 quat 的旋转
矩阵。**当前写在 briefing 里的方向感是基于猜测,没经过实际验证**。

### 7.4 假设:demo 录制本身的几何裕度太小

v2 demo 的最低 z=0.392;按 `layout.json`(briefing 里给的)
`cube_height=0.031`、`table_grasp_z=0.387`,纯几何裕度 0.392 −
0.387 = 5 mm,而 cube 顶面在 0.387+0.031=0.418 —— 等等,这个数
**不一致**。要么 table_grasp_z 是 EE flange 与 cube 顶面接触时的
flange z(那么 0.392 在 cube 顶上方 0 mm);要么 0.387 是别的
什么参考。**briefing 里给的数我没去交叉核对,这一段当作 hypothesis
保留,需读 layout.json 才能下结论。**

### 7.5 假设:executor / UI 没有 z-min safety

`_apply_ip_act` 里 leash 只钳每步增量(4 cm/10°),不钳绝对 z。
如果 IP 持续输出 "继续往下",leash 一段段放,EE 就一路压到桌面
为止。

修复方向(未实现):在 `_apply_ip_act` 里加
```python
if pos_new[2] < table_grasp_z + 0.002 and grip_open:
    pos_new[2] = table_grasp_z + 0.002
```
作为最后一道兜底。

## 8. 下次 session 要做的事

按优先级:

1. **拉 v2 dump.** `rsync nyu-127:/tmp/ip_dump/ /tmp/ip_dump_local/`,
   跑 `/tmp/inspect_dump.py` + `viz_dump.py`。重点看:
   - `actions[0]` 序列里 z 分量(EE 帧)随 step 怎么走;
   - `grips_pred[0]` 的浮点原值,不要被 sign+死区截断后的版本骗。
2. **验 frame 约定.** 写一段 standalone 测试:固定 EE 朝下 + 输入
   纯 +z EE 帧 delta,看 base z 涨还是落。把结论加注到
   `server.py` 的 `T_w_next = T_w_e @ action0` 那行。
3. **加 z-floor safety.** UI `_apply_ip_act` 里硬 clamp:open 状态
   下 pos_new.z 不能低于 `table_grasp_z + 2 mm`。这条独立于
   IP 是否正确,纯是机器保护。
4. **多 demo conditioning.** 用 UI 录 3-5 段不同起始位置的 pick-place,
   server 加 `--num-demos 3` 起。看 grip flip 是否更早出现。
5. **校准 `T_base_camera`.** 当前 3.3 cm 标定 + 1 cm 残差直接进入
   pcd_w,跟 demo 的 pcd 在 base 帧里相差 1 cm 起,这本身就给
   IP 的 phase matching 喂噪声。
6. **更多 demo waypoints.** 当前固定 10 个,如果 phase 信号被
   `np.linspace` 强行均匀化抹掉了 close 那一帧的瞬时性,改成
   保留 grip transition 帧或抬到 20 个会有帮助。

## 9. 当日存的 memory

- `~/.claude/projects/-Users-zhewen-Workspace-robo-ip-deploy/memory/franka_nuc_ssh_routes.md`
  — `franka-backup` alias 路径、sudo 密码 `q1w2`、modprobe 修复套路。
- `~/.claude/projects/-Users-zhewen-Workspace-robo-ip-deploy/memory/ip_demo_scene_match.md`
  — 1-shot OOD failure mode 与 dump 比对诊断流程,Attempt A 复现
  确认。

不在这里复述内容,需要时去原文件读。
