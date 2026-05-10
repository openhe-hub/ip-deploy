# 腕装相机 hand-eye 标定

求 `T_ee_camera`(法兰 → 腕装 D435 `153122074137` 的固定刚体变换),
之后 pipeline 里 `T_base_camera = T_w_e @ T_ee_camera` 每帧动态算,
取代外置相机用的静态 provisional 3.3cm 标定矩阵。

数学原理见 `docs/start_franka.md` 同级或聊天记录里 "Hand-eye 标定公式原理"。
本目录是工具脚本 + flow 文档。

## 文件

| 文件 | 作用 |
|---|---|
| `generate_marker.py` | 生成 ArUco PNG 给你打印 |
| `calibrate_handeye.py` | 移臂 + 采数据 + cv2.calibrateHandEye + 验证 + 落盘 |

## 一次性准备

### 1. 生成并打印 marker

```bash
python generate_marker.py --out aruco_marker.png
# 默认: DICT_5X5_50 ID 0, 600 px 黑方块 + 60 px 白边
# 在 300 DPI 打印下黑方块边长 ~5.08 cm
```

打印时**勾掉 "fit to page" / "缩放"**,必须 100% 打印。

### 2. 测量

打印完用直尺量出**黑色方块的实际边长**(精确到 mm),记下来。
比如 51 mm = 0.051 m,等下传给标定脚本。

### 3. 贴桌

平整地用胶带把 marker 贴在桌面 — 机械臂能在腕相机视野里看到 marker 中心。

### 4. 调初始姿态

用 `ip_debug_ui` 的键盘 teleop 把 EE 摆到一个 **腕相机能看到 marker 居中**
的姿态(典型:marker 上方 25-40 cm,EE z 轴朝下)。

## 运行标定

**前置条件:**

- polymetis robot server 跑着(50051)
- polymetis gripper server 跑着(50052)
- **`ip_debug_ui` 必须停掉**(它独占相机 + 占着 cartesian impedance)
- 上面 4 步准备做完

**步骤:**

1. **停 UI 释放相机:**
   ```bash
   ssh franka-backup 'pkill -f ip_debug_ui'
   ```

2. **预检(不动臂,只验相机 + marker 检测):**
   ```bash
   ssh franka-backup
   source /home/franka/conda/etc/profile.d/conda.sh && conda activate polymetis-local
   cd /home/franka/ICRT/calibration/wrist-camera   # 假设代码已同步到 NUC
   python calibrate_handeye.py --marker-size-m 0.051 --probe-only \
     --debug-dir /tmp/cal_debug
   ```
   期望输出:`marker detected. distance=XX.X cm`。
   如果失败,检查 `/tmp/cal_debug/00_init_FAIL.jpg`,排查光线 / 视野 / dict。

3. **正式标定:**
   ```bash
   python calibrate_handeye.py --marker-size-m 0.051 \
     --num-poses 25 \
     --out T_ee_camera.npy \
     --debug-dir /tmp/cal_debug
   ```

   脚本流程:
   - 验首帧 marker 可见(失败则不动臂直接 exit 2)
   - 循环 25 次:`move_to_ee_pose` 到扰动后的目标 → 等 0.4s → 抓帧 → solvePnP
   - 收集到的 `(T_w_e, T_c_m)` 全存到 `T_ee_camera.samples.npz`(可离线重解)
   - `cv2.calibrateHandEye(method=PARK)` 求解
   - 计算每帧 `T_w_marker = T_w_e @ T_ec @ T_c_m`,报告标准差
   - 落盘 `T_ee_camera.npy`
   - 最后回到初始姿态

   时间:~2-3 分钟。

4. **重启 UI:**
   ```bash
   ssh franka-backup 'nohup /tmp/ui_helper.sh > /tmp/ui.log 2>&1 & disown'
   ```

## 质量验收

脚本最后会打印:

```
=== Validation: T_w_marker stability across N frames ===
  translation std (mm):  xyz = [a, b, c], |.| = X.XX
  rotation std (deg):    xyz = [a, b, c], max = X.XX
```

**通过判定:**
- translation std `|.|` < **5 mm**
- rotation std max < **1°**

不过的话最常见原因 + 修法:

| 症状 | 原因 | 修 |
|---|---|---|
| translation std > 1 cm | 姿态变化不够 / `--rpy-range` 太小 | `--rpy-range 0.35 --num-poses 35` 重跑 |
| rotation std > 2° | 同上,或 marker 边长测错 | 重测边长;增加 num-poses |
| 大量帧 `marker NOT detected` | `--xyz-range`/`--rpy-range` 太大,marker 出视野 | 调小到 `0.025` / `0.20` |
| 一帧都没成功 | dict 错 / marker 太小 / 距离太远 | `--probe-only` 单独排查 |

`samples.npz` 留着 — 不动臂只想换解法或换参数重解,可以离线 `np.load + cv2.calibrateHandEye` 跑,几秒搞定,不用再动机械臂。

## 把结果接到 IP pipeline

标定完得到 `T_ee_camera.npy`,接下来 pipeline 改造(**待实施,不属于本目录**):

- `ip_runner/server.py` 加载 `T_ee_camera.npy`
- 每帧 OBS 包含 `T_w_e`(本来已经有了)
- 计算 `T_base_camera = T_w_e @ T_ee_camera` 替换原来 load 的静态 `.npy`
- 注意:demo 也得换成腕相机录的 — 老 demo `recording_2026-05-04_12-12-47_full` 是外置相机,几何不兼容

## 常见问题

- **`scipy.spatial.transform` 报 import error**:`pip install scipy` 进 polymetis-local
- **`cv2.aruco` 报 import error**:OpenCV 装的是 headless 简版,需要 `pip install opencv-contrib-python`(注意不要和 `opencv-python` 混装,会冲)
- **`move_to_ee_pose` 卡住**:UI 的 cartesian impedance 没停干净。脚本开头有 `terminate_current_policy()` 兜底,但偶尔被 race。手动 `pkill -f ip_debug_ui` + 重连 polymetis 再试
- **标定中急停**:Ctrl-C 会跳出循环,然后用已有 samples 解算并落盘 — 即使没采满 25 帧也能给个结果
