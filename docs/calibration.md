# 腕装 RealSense hand-eye 标定

`calibration/wrist-camera/` —— 2026-05-10 完成的腕相机 (D435 sn `153122074137`)
hand-eye 标定。求得固定刚体变换 `T_ee_camera`,运行时 `T_base_camera =
T_w_e @ T_ee_camera` 每帧动态计算,取代外置相机的静态 provisional 3.3cm 标定。

## 1. 背景与目的

| 路径 | 相机 | T_base_camera 来源 | 残差 |
|---|---|---|---|
| 旧路径(外置) | `925622071356`,固定支架 | 静态 `ip_runner/calib/T_base_camera_3.3cm.npy` | ~1 cm(provisional 3.3cm 标定) |
| 新路径(腕装) | `153122074137`,法兰侧 D435 USB 3.2 | 每帧 `T_w_e @ T_ee_camera`,`T_w_e` 来自 polymetis,`T_ee_camera` 一次性标定 | ~5 mm(本次结果) |

收益:
- `T_w_e` 的 polymetis 关节正运动学非常准,误差贡献几乎只剩 `T_ee_camera`;
- `T_ee_camera` 是机械固定刚体,可重复测、可参考 CAD,且不会随机器人位置漂移;
- 不再依赖"外置相机相对世界"的 extrinsic 标定 —— 那一段是 1cm 残差的来源;
- 副作用:demo 也要重新录(腕相机视野完全不同)。详见 §8。

## 2. 数学原理(AX = XB)

四个坐标系:
- `W` —— 机器人 base
- `E_i` —— 第 i 帧时的 EE flange
- `C_i` —— 第 i 帧时的相机
- `M` —— marker(在 W 系中固定不动)

约束:对任意一帧 i,marker 在 W 中的位姿都相同:

```
T_W_E_i · T_E_C · T_C_M_i  =  T_W_M  =  const
```

取两帧 i, j,消掉 `T_W_M`:

```
T_W_E_i · T_E_C · T_C_M_i  =  T_W_E_j · T_E_C · T_C_M_j

⇒  (T_W_E_j^-1 · T_W_E_i) · T_E_C  =  T_E_C · (T_C_M_j · T_C_M_i^-1)
       └────── A_ij ──────┘   └X─┘     └X─┘   └────── B_ij ──────┘
```

得到经典的 `A_ij · X = X · B_ij`。OpenCV `cv2.calibrateHandEye` 求 `X = T_E_C`。
本脚本默认 `method=CALIB_HAND_EYE_PARK`(Park 1994):旋转部分有闭合解,
平移再走线性最小二乘。

**为什么需要旋转多样性:** 平移那一步是
`(R_A − I) · t_X = R_X · t_B − t_A` 的线性系统;若所有姿态间的旋转 `R_A` 都
接近 `I`,系数矩阵奇异,平移分量解不出来。所以采集要在多个旋转轴上扰动,
不能只平移。

## 3. 实现细节

`calibration/wrist-camera/calibrate_handeye.py`:

| 项 | 默认值 | 说明 |
|---|---|---|
| `--charuco` | 推荐打开 | ChArUco 板:N×M 方格亚像素角点 vs 单 ArUco 4 个角,约束多得多,部分遮挡仍可解 |
| `--cb-cols / --cb-rows` | 9 / 6 | 注意新 OpenCV API 取 `(cols, rows)`,即 cols=9 rows=6 |
| `--cb-dict` | `DICT_4X4_50` | |
| `--cb-square-m / --cb-marker-m` | 0.021 / 0.015 | 21 mm 方格 + 15 mm 内 marker |
| `--cb-legacy-pattern` | **True** | 关键:我们手头打印的板子是 OpenCV < 4.6 的列优先 / 错位奇偶布局;不开此 flag 即便检出 27 个 raw ArUco markers,ChArUco 角点仍是 0 |
| `--camera-serial` | `153122074137` | 腕装 D435 |
| `--width / --height / --fps` | 640 / 480 / 30 | 出厂校准 → `dist coeffs` 全 0 |
| `--num-poses` | 25 | 围绕初始姿态扰动 |
| `--xyz-range / --rpy-range` | 0.025 m / 0.20 rad | EE 系扰动半幅;~2.5 cm + ~11° |
| `--time-per-move` | 5.0 s | 保守值,joint 6 速度安全限是 2.51 rad/s,见交叉引用的 polymetis-motion-speed feedback memory |
| `--settle-s` | 0.6 s | 移动到位后等臂稳 + 自动曝光 |

每个姿态:`robot.move_to_ee_pose` → 等 `settle_s` → 5 帧 warmup 抓最新 color
帧 → `solvePnP` 出 `T_camera_marker` → 与当前 polymetis `T_w_e` 配对存进
samples 列表。

求解器入口:

```python
R_ec, t_ec = cv2.calibrateHandEye(
    R_gripper2base=R_w_e,  t_gripper2base=t_w_e,
    R_target2cam =R_c_m,   t_target2cam =t_c_m,
    method=cv2.CALIB_HAND_EYE_PARK,
)
```

落盘:
- `T_ee_camera.npy` —— 4×4 float64 最终结果
- `T_ee_camera.samples.npz` —— 原始 `(T_w_e, T_c_m, K, dist)`,可离线换 solver / 换 outlier 阈值重解,不用再动机械臂

## 4. 2026-05-10 标定结果

```
T_ee_camera =
[[ 0.7141 -0.6153  0.3339 -0.0825]
 [ 0.6999  0.6160 -0.3615  0.0379]
 [ 0.0167  0.4919  0.8705  0.0200]
 [ 0       0       0       1     ]]

translation (mm):       [-82.47, +37.85, +20.01]   (|.| ≈ 91 mm,法兰外侧偏置)
rotation (xyz Euler °): [+29.47, -0.96, +44.42]
camera z-axis in EE:    [0.334, -0.362, 0.871]
```

`cam_z[2] = 0.871` 表示相机 z 轴(光轴)在 EE 系里 z 分量 > 0.85,即相机大致
顺着 EE +z 方向看 —— 与"夹爪朝下、相机也朝下"的腕装几何一致。

落盘位置:
- Mac 仓库:`calibration/wrist-camera/T_ee_camera.npy`
- NUC 运行时:`/home/franka/ICRT/calibration/wrist-camera/result/T_ee_camera.npy`(已 copy)

## 5. 验证方法(4 项独立检查)

仅看 solver 残差不够 —— PARK 是闭合解,会"硬把数据拟合上"。下面 4 项是
互不依赖的横切验证。

### 5.1 方向自洽

腕装几何要求相机朝向与 EE z 轴大致同向。检查 `T_ee_camera[:3, 2]` 第三分量:

| 量 | 值 | 阈值 | 结果 |
|---|---|---|---|
| `cam_z` 在 EE 系 z 分量 | 0.871 | > 0.85 | PASS |

物理意义:确认 `T_ee_camera` 不是某个数学上拟合得通但翻转/镜像了的局部解。

### 5.2 In-sample T_w_marker 稳定性

marker 钉在桌上不动,`T_w_M = T_w_e · T_ee_camera · T_c_m` 应在所有帧间
保持常数。结果(剔 2 帧 outlier 后,n=24):

```
平移误差(mm,相对 marker 中位数姿态):  med 4.5,  p90 8.3,  max 9.2
旋转误差(°,quaternion-aware):         med 1.0,  p90 1.4,  max 2.6
```

注:旋转散度必须用 quaternion-aware 距离(`acos(2·〈q1,q2〉²−1)`),不能直接对
xyz Euler 取 std —— Euler 在 ±180° wraparound 处会把 0.5° 的真实差算成 ~70°
的伪差,`calibrate_handeye.py` 内置的 `validate()` 用 Euler std 是粗看,
真验收用脚本之外的 quaternion 距离。

### 5.3 Leave-one-out CV

留 1 验证 24:每次 hold out 一帧,在剩余 23 帧上重解 `T_ee_camera`,
然后用解出的矩阵预测 hold-out 帧的 `T_w_M`,看与"全数据 median `T_w_M`"
的距离:

```
LOO 平移误差(mm): med 4.7,  max 10.4
LOO 旋转误差(°):  med 1.05, max 2.83
```

LOO ≈ in-sample,说明 solver 在泛化、没有过拟合 24 帧噪声。

### 5.4 算法交叉验证(4 个 solver 独立解)

跑 PARK / TSAI / DANIILIDIS / HORAUD 四个 solver(同一份 samples),互比:

```
4 个解的两两差异:平移 ~2 mm,旋转 ~0.2°
```

四个不同算法在同一数据上几乎得到同一答案,强证据 AX=XB 良态、没有
退化旋转造成的局部极小。

**结论:整体精度 ~5 mm / 1°,比外置相机 provisional 3.3cm 标定的 ~1 cm
残差好约 2×。**

## 6. Outlier 处理

26 帧采集中剔除 2 帧:

| idx | 平移误差 | 原因 |
|---|---|---|
| 0 | 51 mm | 初始帧,**未经任何运动 → 没等 settle_s** → 自动曝光 / depth align 还没稳 |
| 1 | 12 mm | 第 1 个扰动姿态,摆动小,姿态间 baseline 不足 |

筛法:median + MAD 风格 —— 平移与中位数姿态偏差 > 10 mm,或旋转偏差 > 3°,
丢弃。

**已知 bug:** `calibrate_handeye.py` 当前在采集开头把 `(T_w_e_init, T_cm0)`
作为 sample[0] append 进 samples 列表(脚本 `samples = [(T_w_e_init.copy(),
T_cm0.copy())]` 那一行)。这个 sample 没经过 `move_to_ee_pose` + `settle_s`,
是 5.2/5.3 里 idx 0 的 51 mm outlier 来源。**应改为不 append,只把 init 帧
当作 pre-flight detection 的 sanity check,不入 samples**。

## 7. 如何重做 / 改进

### 7.1 重做(标准流程)

完整操作步骤见 [`calibration/wrist-camera/README.md`](../calibration/wrist-camera/README.md)。
摘要:

```bash
# NUC 上,先停 UI 释放相机
ssh franka-backup 'pkill -f ip_debug_ui'

ssh franka-backup
source /home/franka/conda/etc/profile.d/conda.sh && conda activate polymetis-local
cd /home/franka/ICRT/calibration/wrist-camera

# 预检
python calibrate_handeye.py --charuco --probe-only --debug-dir /tmp/cal_debug

# 正式
python calibrate_handeye.py --charuco \
  --num-poses 25 --out T_ee_camera.npy --debug-dir /tmp/cal_debug
```

### 7.2 把精度推到 < 5 mm

| 方向 | 调法 |
|---|---|
| 旋转多样性更大 | `--rpy-range 0.30`(~17°/轴),目前 0.20 |
| 姿态更多 | `--num-poses 50` |
| 更高分辨率 | `--width 1280 --height 720` |
| 背景干净 | ChArUco 板周围别放干扰物体 |
| 修 init-frame 入 samples 的 bug | 见 §6 |

每条都能独立尝试;samples.npz 留着,改完离线重解几秒就出结果。

## 8. 接通 IP pipeline(待实施)

**这一节是 outline,不是已完成工作。**

当前 `ip_runner/server.py` 在启动时 load 一份静态 `T_base_camera`(传参
`--T-base-camera ip_runner/calib/T_base_camera_3.3cm.npy`)。腕装相机方案下:

- IP server 改 load `T_ee_camera.npy`(腕装),不再 load 世界系静态矩阵;
- 每帧 OBS 已经带 `T_w_e`,server 在 lift 之前算 `T_base_camera = T_w_e @
  T_ee_camera`,然后照常 `pcd_w = T_base_camera @ pcd_camera`;
- **所有 demo 必须重录** —— 现有 `demo_12-12-47.pkl` 等用的是外置相机
  (`925622071356`)在桌面正前方某个固定视角拍的;腕相机视野完全不同,
  demo 与 inference 几何不兼容会让 IP 失效;
- ZMQ 协议本身不变(OBS 已带 `T_w_e`),只是 server 端解释 `T_base_camera`
  的方式从"静态 load"改成"逐帧合成";
- 改完先 dry-run 验证 `pcd_w` 仍落在桌面平面、IP run 不抖,再正式跑闭环。

## 9. 故障排查表

| 症状 | 原因 | 处理 |
|---|---|---|
| 0 ChArUco corners 但 raw ArUco IDs = 27 | 板子是旧 layout(列优先错位奇偶) | `--cb-legacy-pattern`(默认已开;若不小心用 `--no-cb-legacy-pattern` 关了就会出此症状) |
| 4 个 ArUco dict + 4 个 AprilTag dict 全部静默 | marker 出视野 / 光线太暗 / 用错 dict | 先跑 `--discover` 模式扫所有 dict,看有没有任何一个能检出 |
| In-sample 旋转 std 看起来 ~70° | Euler 在 ±180° 处 wraparound | 用 quaternion-aware 距离,不要直接对 Euler 取 std;脚本内置 validate 输出仅作粗看,验收以 §5.2 量为准 |
| `move_to_ee_pose` 报 joint velocity safety limit | `--time-per-move` 设得太小,或 `--rpy-range` 太大 | 默认 `--time-per-move 5.0` + `--rpy-range 0.20` 不会触发;别动这两个默认值 |
| 标定中 Ctrl-C | 跳出循环,用已采到的 samples 解算并落盘 | 即使没采满 25 帧也能给个结果,但样本数 < 8 时 sys.exit(3) |
| `T_w_marker` std 大但单帧 PnP 看起来正常 | 旋转多样性不够,平移那一步奇异 | `--rpy-range` 加大,`--num-poses` 加多 |

## 10. 交叉引用

- [`calibration/wrist-camera/README.md`](../calibration/wrist-camera/README.md) —— operator 视角的 step-by-step 手册
- [`calibration/wrist-camera/calibrate_handeye.py`](../calibration/wrist-camera/calibrate_handeye.py) —— 脚本源码(本文档所有事实的 ground truth)
- [`calibration/wrist-camera/generate_marker.py`](../calibration/wrist-camera/generate_marker.py) —— 单 ArUco marker 打印生成器(ChArUco 板用现成的)
- [`pipeline.md`](./pipeline.md) §2、§4 —— IP server / ZMQ 协议;`T_ee_camera` 接入点见 §8
- [`ui.md`](./ui.md) §5、§9 —— 调初始姿态(让 marker 在腕相机视野中央)用的是 UI 的键盘 teleop
- polymetis-motion-speed feedback memory —— joint 6 速度安全限 2.51 rad/s 的来源,本脚本 `--time-per-move 5.0` 默认值就是按这个 cap 反推
