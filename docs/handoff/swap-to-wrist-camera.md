# 任务

把 IP 实机部署的点云源从**外置相机** (`925622071356`) 切到**腕装相机** (`153122074137`),并把当前的静态 `T_base_camera_3.3cm.npy` 替换成**每帧动态计算** `T_base_camera = T_w_e @ T_ee_camera`。

T_ee_camera 已经标定完成(2026-05-10),精度 ~5mm/1°,落盘:
- Mac:  `calibration/wrist-camera/T_ee_camera.npy`
- NUC:  `/home/franka/ICRT/calibration/wrist-camera/result/T_ee_camera.npy`

详见 `docs/calibration.md` 第 1 / 2 / 4 节。

# 为什么要换

1. **几何精度**:外置标定残差 ~1cm(provisional 3.3cm 标的);腕装链路 `T_w_e (polymetis) @ T_ee_camera (一次标定)` 实测 ~5mm,提升一倍以上
2. **几何"自然"**:IP 内部本来就要 `transform_pcd(pcd_w, inv(T_w_e))` 到 EE 帧,腕装把这一步从"间接"变"直接"
3. **机械固定 vs 桌面放置**:外置相机偶尔被人挪、桌子动一动标定就失效;腕装跟着 EE 走,只要相机不被换 mount,标定永远有效

# 当前可用资源

- **`ip_runner/server.py`** — IP server,跑在 nyu-127。当前从启动 args `--T-base-camera` 加载静态矩阵,每帧 `pcd_cam → pcd_w` 用这个静态矩阵
- **`ip_debug_ui/server.py`** — UI 后端,跑在 NUC。两个相机:`state.cam`(主,925622071356,用于发 OBS 给 IP)+ `state.cam_alt`(腕装 153122074137,目前只用于可视化)
- **OBS 协议** — ZMQ 消息已经携带 `T_w_e`,所以协议不用改,只是 server 端解释 `K` 和怎么算 `T_base_camera` 要变
- **腕装相机内参** — 640×480@30:`fx=605.79, fy=605.54, cx=317.28, cy=249.25, dist=[0,0,0,0,0]`(出厂已矫正)

# 需要做的修改(按文件)

## 1. `ip_runner/server.py`

- **加新 CLI arg** `--T-ee-camera` 指向 `T_ee_camera.npy`(路径上要支持 nyu-127 / NUC 两种部署位置)
- 启动时加载 T_ee_camera (4×4 numpy)
- **OBS 处理改造**:每收到一条 OBS,从 OBS 里取 `T_w_e`(已经有),计算 `T_base_camera = T_w_e @ T_ee_camera`,代替原来的静态矩阵
- 旧的 `--T-base-camera` arg 可以**保留作为 fallback**(未传 `--T-ee-camera` 时退回旧行为),也可以**直接弃用**(grep 一下哪些其他脚本依赖)
- 注意:server 内部 `transform_points(pcd_cam, T_base_camera)` 这一行不用动,只是输入的 T_base_camera 来源换了

## 2. `ip_debug_ui/server.py`

当前默认 `state.cam` = 主相机(外置),IP 看的是主相机。要切换到腕装作为 IP 数据源:

**选项 A(推荐,简单)**:CLI default 反过来 — `--camera-serial 153122074137`(腕装为主),`--camera-serial-alt 925622071356`(外置为辅)。改 default 即可,代码逻辑不动

**选项 B**:加 `--ip-camera-source {primary, alt}` flag,精细控制哪台相机的帧用于发 OBS。代码改动稍多,适合长期保留双相机

不管选哪个,**确保发到 IP server 的 OBS 里 K 是腕装相机的 K,不是外置的**(注意:UI 现在读 K 是从 RealSense pipeline 自带的 intrinsics 取,所以只要 cam 切到腕装,K 自动跟着切)

## 3. CLI 启动命令(更新 `docs/pipeline.md` §9)

旧:`python -m ip_runner.server --T-base-camera ip_runner/calib/T_base_camera_3.3cm.npy ...`
新:`python -m ip_runner.server --T-ee-camera ip_runner/calib/T_ee_camera.npy ...`

把 `T_ee_camera.npy` 拷到 `ip_runner/calib/` 或者保留在 `calibration/wrist-camera/` 让 server 跨目录读 — 你定。

# 不要做的事

- **不要重录 demo** — 这是单独大任务,需要操作员配合 + 一段 wrist 视角下的标定场景。先做完 pipeline 切换 + smoke test,demo 重录留下一个 handoff
- **不要碰 `calibration/wrist-camera/`** — 标定脚本和结果都已经稳定
- **不要改 OBS / ACT 协议** — 字段已经够用,改了会破坏 NUC 端 client
- **不要 deprecate 整个 ip_runner 旧路径** — 万一新路径出问题,有 fallback 是必要的

# 阅读顺序(给跨 session 接手的人)

按这个顺序读才能拼出全貌:

1. `docs/calibration.md` §1-§2 + §4 — 数学 + 标定结果
2. `docs/pipeline.md` §1-§3 — 当前 IP server 架构 + ZMQ 协议
3. `docs/ui.md` §2-§4 — UI 后端怎么发 OBS / 录制 / 闭环切换
4. `ip_runner/server.py` — main + OBS handler + transform_points 调用点
5. `ip_debug_ui/server.py` — `state.cam` / `state.cam_alt` / seg_loop 发 OBS 的代码段(grep `cam.poll` / `K` / `send.*obs`)
6. `calibration/wrist-camera/T_ee_camera.npy` — 4×4 float64,可以 `np.load(...)` 验证

# 验证(Definition of Done)

实现完成后跑这 3 个 smoke test:

1. **离线对一帧**:在 nyu-127 写一个 standalone script,从某个录制的 OBS pickle 加载 (color, depth, K_wrist, T_w_e),套你的新 server 流程,产出 `pcd_w`,用 open3d 可视化看是否合理(桌面 z≈0、目标物体在 z>0、点云不在墙后或地下)
2. **server 在线连通**:启 ip_runner.server (新 path),启 ip_debug_ui (主相机切腕装),从 UI 戳一次 reset_episode,确认 server 不报错且回 `seg` reply,UI 的 pcd_w 视图能渲染出合理点云
3. **回归**:旧 pipeline 走老 `--T-base-camera` 还能起来(如果选项 A 保留 fallback)

不通过的话,**先解决再说**,不要往后续 IP 闭环 / 重录 demo 推进。

# 已知坑

- 腕装相机和外置相机看的是**不同视角**,所以现有 demo `recording_2026-05-04_12-12-47_full` 的 GD prompt(red/green/blue/yellow cube 等)在腕装视野下可能根本拍不到那些物体或角度完全错位。**smoke test 时换个简单场景**(比如桌面放一个红色方块,prompt = "red cube ."),否则 GD 会返回空 box → server 报 "too few points after segmentation"
- 腕装 D435 USB 3.2,可以跑 30 fps,**比外置 D435I (USB 2.1, 6 fps) 快 5 倍**;改完之后 IP 推理频率自然能提上来,但要注意 server 端 SAM2 推理本身会成新瓶颈
- 标定时 ChArUco 板的 `legacyPattern=True` 是**那块板特有**的 quirk(印的时候用 OpenCV<4.6),和 IP pipeline 完全无关 — 这条只对 redo 标定的人重要,你不需要管

# 引用 / memory

- `docs/calibration.md` — 标定全文档
- `docs/franka.md` — Franka 启动 SOP(冷启动 / Joint Position Error 恢复 / Safety Operator)
- `docs/pipeline.md` §4 — ZMQ OBS/ACT schema 权威定义
- `~/.claude/projects/.../memory/feedback_polymetis_motion_speed.md` — polymetis 移动安全约束(本任务不涉及移动臂,但触碰任何机械臂代码前都该读)
