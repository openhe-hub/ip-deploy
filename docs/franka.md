# Franka 运维手册 (FR3)

> docs/franka.md — Franka Research 3 在我们这个项目里的所有运维知识。
> 物理 + 网络 + Desk + polymetis + ip_debug_ui + 错误恢复 + 关机。
> 上游权威 manual: docs/franka_research_3_*.pdf 三份(operating manual / product manual / watchman)。
> NUC 上还有 `~/franka_scripts/{franka_runbook,AGENT}.md` 是另一份独立 runbook(那份偏 NUC 视角,本文偏 Mac 视角)。

整条链路:

```
Franka 控制柜上电 → eno1 carrier → ping FCI 172.16.0.2 →
SSH 隧道 → Desk 登录 → REQUEST CONTROL + 按蓝圆按钮 → Unlock Joints → Activate FCI →
launch_gripper.py (50052) → launch_robot.py (50051) → ip_debug_ui (8000)
```

冷启动按 §1 → §9 顺序走;碰到弹窗 / 报错按 §10 / §12 查;关机按 §11。

---

## 1. 物理 / 硬件

| 项 | 状态 | 备注 |
|---|---|---|
| Franka 控制柜电源 | 后侧白色摇臂 ON | **从不要拔电关机** — 必须走 Desk Shut down,否则可能进 Rescue Mode |
| 急停按钮 | popped(顺时针拧弹起) | 按下 → FCI 激活会被拒,报 `Robot not operational` |
| 控制柜钥匙 | 运行档(右侧) | 锁档无法解 brake |
| Ethernet:Franka ↔ NUC | 一头 Franka 的 `Shop floor` / `LAN` 口,另一头 NUC `eno1` | 不要插 `Master` 口,会 NO-CARRIER |
| External Enabling Device | 接到机械臂底座 X4(M12 4-pin) | 平时可以不接,§10 joint position recovery **必须有** |
| 工作空间 | 机械臂周围约 1 m 球内空着 | 解锁瞬间会下沉几 mm,gripper homing 还会全开+全合 |

控制柜上电后等 ~60 s 再继续,master controller 启动需要时间。

机械臂底座 LED 指示(引 Arm Manual §4.10.1):
- 黄 → 待解锁 / 初始化
- 白 → ready / FCI 已 connect
- 红慢闪 → safety violation(走 §10 / §12 排查)

## 2. 网络 layer

从本地 Mac 验证(SSH 别名 `franka-backup`):

```bash
ssh franka-backup 'ip -br link show eno1; ip -4 addr show eno1 | grep inet; ping -c 2 -W 1 172.16.0.2'
```

**期望:**

```
eno1     UP    1c:69:7a:00:82:c4 <BROADCAST,MULTICAST,UP,LOWER_UP>
    inet 172.16.0.1/24 brd 172.16.0.255 scope global noprefixroute eno1
2 packets transmitted, 2 received, 0% packet loss
```

`LOWER_UP` 必须有(物理层握上手),`172.16.0.1/24` 必须有(NUC 的静态 IP),
`ping 172.16.0.2 0% loss` 必须有(摸得到 Franka 控制器)。

任何一项不满足:回 §1 查物理层。

**额外验证 Desk 没进 Rescue Mode:**

```bash
ssh franka-backup 'curl -sk -o /dev/null -w "/desk/=%{http_code}\n/rescue=%{http_code}\n" https://172.16.0.2/desk/ https://172.16.0.2/rescue'
```

期望 `/desk/=200` + `/rescue=302`。如果 `/login` 跳到 `/rescue` 且那里返回
"Rescue System" 标题,说明 master controller 没启动正常系统镜像 — **物理重启控制柜**
(墙插拔电等 30s 重插)。**不要点 Rescue UI 里的 "Factory Reset"**,会清掉所有 apps / safety scenarios / End-Effector 配置。

## 3. SSH 隧道 → Desk web

`172.16.0.2` 只在 NUC 的 LAN,Mac 走 SSH 端口转发:

```bash
ssh -N -o ServerAliveInterval=60 -o ServerAliveCountMax=99999 \
    -o TCPKeepAlive=yes -o ExitOnForwardFailure=yes \
    -L 8443:172.16.0.2:443 franka-backup
```

留着别关。本地浏览器:

```
https://localhost:8443/desk/
```

**注意尾部 `/desk/`** — 直接访问 `https://localhost:8443/` 会被 nginx 301 跳到
`https://localhost/desk/`(不带端口),浏览器去 443 找,挂。

证书自签的,Chrome `Advanced → Proceed`。

可以把 8443 + 8000 合并到一条隧道:

```bash
ssh -N -o ServerAliveInterval=60 -o ServerAliveCountMax=99999 \
    -L 8443:172.16.0.2:443 -L 8000:localhost:8000 franka-backup
```

## 4. Take Control:登录 + 按蓝圆按钮

**Desk 登录账号(admin):**

```
frankaUser / frankaPass123
```

(不是 `admin` — FR3 出厂默认 admin 账户名是 `frankaUser`。)

**进入后:**

1. 如果 header 已经显示 **"Release Control"** → 已经在 hold control,跳到 §5。
   **不要点 Release Control**,那是放弃控制。
2. 如果 header 显示 **"REQUEST CONTROL"**(或弹窗 "User has control") → 点 **REQUEST CONTROL → Enforce**。
3. 弹窗 **"Confirm physical access"** + 60s 倒计时。
4. **去机械臂跟前**,按机械臂**最后一节连杆 / 法兰盘旁边的小控制板**(Franka 官方叫 **"robot pilot"**)上的**蓝色圆形按钮**,**按住 ~2 秒**,弹窗会自动关闭。

**这个蓝按钮必须在机械臂上按,没有软件 bypass**。它不是:
- 控制柜(地上的机柜)上的任何按钮
- 机械臂底座上的白按钮
- 单独的红色急停盒

如果出现 **"FSoE Connection error"** 弹窗,点 **ACKNOWLEDGE**(safety bus 报告控制柜→机械臂链路瞬断,常发于刚开机)。重复出现就检查电缆 / 急停。

## 5. Unlock Joints

右侧栏 **Joints** 区下两个图标:**右边开锁(unlock),左边闭锁(lock)**。点右边。

弹窗 **"Open fail safe locking system / Robot will move slightly..."** — 按钮文字是 **OPEN**,**不是 CONFIRM**(通用 confirm-button handler 在这里失效)。

听到 7 声 "咔咔咔",机械臂下沉几 mm — LED 黄→白,Desk 显示 **"Ready"**,`Joints` 区图标变开锁。

**这一步不需要按住蓝按钮**(不是 deadman action)。

### 卡住对照

| 症状 | 真因 | 修法 |
|---|---|---|
| 找不到 unlock 按钮 | 没看对位置 | 右侧栏中段的 `Joints` 区,**右**边图标(开锁);左边是反向 lock |
| 弹窗里没看到 "确认" 之类按钮 | 按钮文字是 **OPEN**,不是 CONFIRM/确认 | 直接点 OPEN |
| unlock 图标灰着点不了 | 没 hold control | 回 §4:REQUEST CONTROL → Enforce → 机械臂上蓝圆按钮按 2 秒 |
| 点了 OPEN 但 LED 还是黄 / 状态不变 | brake 没真解开,可能急停按下 / 钥匙锁档 / FSoE 报错没 ACK | 回 §1 物理检查;回 §4 ack FSoE 弹窗;再点一次 OPEN |
| **unlock 完点 Activate FCI 立刻又弹 "Brakes are closed"** | unlock 后 brakes 在没 FCI client 的情况下 auto-relock(常见,延迟 ~30s) | **重做 §5,然后 30 秒内立刻点 Activate FCI**,中间不要停留 |

## 6. Activate FCI

点 header 上的 `Franka Research 3` 下拉菜单 → **Activate FCI** → 弹窗 **CONFIRM**。

**激活成功:**右侧栏出现 **`FCI: ON`**,下拉菜单变成 **Deactivate FCI**。

**坑:**
- **"Service is not available"**:刚冷启动 FCI service 还没起完,等 30–60s 再点,不要重启或回前面步骤(2026-04-30 在本机观察过)
- **"Brakes are closed, please unlock the brakes"**:Unlock Joints 和 Activate FCI 之间停太久,brakes 自动重锁(没 FCI client 释放 brake 不安全)。回 §5 立刻重做,然后秒点 FCI

## 7. End-Effector profile(一次性)

**症状:**`launch_gripper.py` 报 `Connection to FCI refused`,但 FCI 显示 ON,`launch_robot.py` 也能连上。

**真因:**End-Effector 被设成 "Generic Device",gripper service 拒接。

**修法:**
1. Desk → **SETTINGS → End Effector**
2. 在 hold control 状态下激活带 Franka Hand 的 profile(本机用 **"Migrated Profile"**)
3. 回 Dashboard,**End Effector** 字段应当读 `Franka Hand`(不是 `none / other`)

激活后 libfranka 报告的 `m_ee` 从 1.2 kg 变 0.73 kg(标准 Franka Hand 质量),
gravity compensation 也对了。

**这个配置持久化**,只需要在别人换过 profile 时做。改 End-Effector 属于 admin 权限范围(引 Operating Manual §7.2.2),`frankaUser` 直接能改。

## 8. Launch polymetis(从 Mac 通过 SSH)

NUC 上的 helper 脚本(在 `/tmp/`,重启会清,需要时重写):

```bash
# /tmp/launch_gripper_helper.sh
#!/bin/bash
set -e
source /home/franka/conda/etc/profile.d/conda.sh
conda activate polymetis-local
exec launch_gripper.py gripper=franka_hand gripper.executable_cfg.robot_ip=172.16.0.2
```

```bash
# /tmp/launch_robot_helper.sh
#!/bin/bash
set -e
source /home/franka/conda/etc/profile.d/conda.sh
conda activate polymetis-local
echo q1w2 | sudo -S -v
exec launch_robot.py robot_client=franka_hardware robot_client.executable_cfg.robot_ip=172.16.0.2
```

为什么需要 helper:
1. `launch_robot.py` 内部调 `subprocess.run(["sudo", "echo", ...])` 要 tty,
   nohup / setsid 直接跑会 `sudo: a terminal is required`。helper 里
   `echo q1w2 | sudo -S -v` 提前刷新缓存,后续不再要 tty
2. `polymetis-local` 加载 `libtorchscript_pinocchio.so` / `libtorchrot.so` 需要
   `CONDA_PREFIX`,bare nohup 下没有,必须 `conda activate`

启动顺序(gripper 先,更轻):

```bash
ssh franka-backup
nohup /tmp/launch_gripper_helper.sh > /tmp/gripper.log 2>&1 & disown
sleep 5
tail -20 /tmp/gripper.log     # 期望: "RPC server listening on 0.0.0.0:50052"

nohup /tmp/launch_robot_helper.sh > /tmp/robot.log 2>&1 & disown
sleep 8
tail -30 /tmp/robot.log       # 期望: "RPC server listening on 0.0.0.0:50051"
```

**验收:**

```bash
ss -tlnp 2>/dev/null | grep -E '50051|50052'   # 各一行 LISTEN
```

## 9. 启动 ip_debug_ui

详见 [`ui.md`](./ui.md) §9,简化:

```bash
ssh franka-backup
source /home/franka/conda/etc/profile.d/conda.sh && conda activate polymetis-local
cd /home/franka/ICRT
nohup python -m ip_debug_ui.server > /tmp/ui.log 2>&1 & disown

# Mac 另开终端
ssh -L 8000:localhost:8000 franka-backup
open http://localhost:8000
```

---

## 10. Joint Position Error 恢复

2026-05-10 实测整出来的恢复流程。这一节 standalone,跟 §1–§9 的冷启动是分开两条事件路径。

### 10.1 症状

Desk 弹窗 **"Joint Position Error detected"**:

> Joints are misaligned, likely from power loss or emergency unlocking.
> This recovery procedure is restricted to safety operators.
> Alert a safety operator to address this error.

只给 **Shut Down** / **Reboot** 两个按钮 — **都不会修**。Reboot 之后 error 还会回来
(状态写在内部盘,不是临时态)。

也可能伴随 **机械臂底座红灯慢闪**(safety violation 视觉信号,引 Arm Manual §4.10.1)。

引 Operating Manual §9.1.5 (p.56) Figure 39。

### 10.2 根因

`saved_position_at_last_shutdown` vs `current_encoder_reading` 差超过阈值。最常见触发:

- 控制柜断电 / 强制 power-cycle 时关节微动
- 用户用 Emergency Unlocking Tool 手动扳过关节(引 Arm Manual §3.4.1 Emergency unlock label)

ISO 10218 / ISO TS 15066 要求该类 error 必须由 **trained safety personnel 现场目视确认** 才能恢复 — Franka 把这一硬性 requirement 绑到 Safety Operator 角色,**没有任何软件 bypass**。

### 10.3 FR3 用户角色体系(关键认知)

引 Operating Manual §7.2 (p.46) + Watchman §2 Roles and Personae:

| 角色 | 能做的 | ACK Joint Position Error? |
|---|---|---|
| User / Operator | 跑预设 task,view safety settings,start/stop task,lock/unlock joints | ✗ |
| **Administrator** | 用户管理 / 改密码 / End-Effector 配置 / FCI 激活 / non-safety 系统配置 | **✗ — admin 不够!** |
| **Safety Operator** | safety configuration(Watchman) + recovery of specific safety errors | ✓ |

`frankaUser` 是 admin,但 admin **不是** Safety Operator(Franka 故意分开,防止单人既配置又确认 — cross-check 安全机制)。Watchman 文档原话:

> A user in the Administrator role can create a Safety Operator but cannot edit any safety functions himself. Only in the user level Safety Operator can settings be made in Watchman. The Safety Operator is responsible for the correct safety configuration and documentation of the safety functions.

### 10.4 恢复路径

总体:**A. 用 admin 创建 Safety Operator → B. 登入 Safety Operator → C. 走 START RECOVERY 流程**。

#### A. 用 admin (frankaUser) 创建 Safety Operator 账号

引 Operating Manual §7.2.4.1 Creating and editing users (p.47):

1. 登 `frankaUser` / `frankaPass123`
2. 进 **Settings**(独立接口,不在 Desk 主界面里 — 从 Desk 顶栏 / 右上用户菜单 / 或单独 URL 访问;参考 Operating Manual §7.2.4)
3. 点 **Users** tab
4. **Add new user** → role 选 **Safety Operator** → 设用户名 + 密码

注:此步在 admin 角色内 OK,不需要物理验证按钮。但安全敏感的 Watchman 编辑仍需以 Safety Operator 身份重新登录,admin 永远碰不到 safety 配置。

#### B. 登出 admin → 登入 Safety Operator

弹窗里现在出现 **START RECOVERY** 按钮(admin 看不到,Safety Operator 才显示;引 Operating Manual §9.1.5 p.56)。

#### C. 跑 recovery

引 Operating Manual §9.1.5 (p.56-58) Figure 40-44:

1. 点 **START RECOVERY** → 弹出 "Please resolve the joint position error by moving to the reference position",左侧列出 7 个 joints,失败的 joint 是红圈
2. 点击具体某个失败 joint → 进入 "Joint recovery locked" 视图(Figure 41),该 joint 显示锁图标
3. **左手按住 External Enabling Device**(见 §10.5)半按状态 + 持续按住 — 屏幕右上角 "X4 - External enabling device active" 变绿
4. 屏幕上点该 joint 的 unlock 图标 → 听到电磁刹车释放,视图变 "Joint recovery ready for movement"(Figure 42)
5. 屏幕上点 **"Move Joint To Ref. Position"** + 持续按住,机械臂自动转该关节到参考位置(也可用 +/- 微调)
6. 提示 "reference position 到达"(Figure 43,绿条提示 "You can now verify that the joint is in the reference position")
7. **目视确认** joint 真的到位(Operating Manual §9.1.5 NOTICE:"Visually check whether the affected joint has moved to the reference position")
8. 释放 External Enabling Device
9. 勾选 confirm 复选框 + 点 **CONFIRM RECOVERY** → 该 joint recovery 完成
10. 重复 2-9 给其他失败 joints
11. 全部完成后,顶部 **CONFIRM** 按钮亮起,点确认整体 recovery(Figure 47)

**两个常见提示:**
- 按钮没按住够久 → "Please hold the button longer. It was released too soon."(Figure 45)
- 还没到参考位置就松手 → "The robot has not moved to the reference position yet. Continue holding the button until a success message appears."(Figure 46)

**如果某个 joint 有干涉无法直接到参考位置**:可以先用 +/- 键移动其他 joints 让出空间(Operating Manual §9.1.5 提示:"you can move other joints in any order to move out of the obstruction")。

### 10.5 External Enabling Device

引 Product Manual §3.4.3 (p.13) + §6.1 Figure 29 (p.44 — scope of delivery devices) + §7 Figure 31 (p.46 — overview of interfaces, X4 标在机械臂底座)。

物理外观(Figure 29):
- 手持瘦长握把(类似工业 pendant grip 但只有一个主按钮)
- 顶端 3 段式 deadman 按钮
- 线末端是 M12 4-pin 圆形 connector
- 接到机械臂底座的 **X4** 接口(注意:X4 在 **机械臂底座**,不是控制柜!Product Manual §7 Figure 31 把 X4 标在 Arm 上;X3 是 Emergency Stop Device 接口)

3 段操作:
- 完全松开 → 锁定(不动)
- **半按 → enabled,机械臂可动**
- 完全按到底(panic state) → 锁定

**没有这个设备 recovery 完全做不了** — 这是 IEC 60204-1 / IEC 60947-5-8 Cat.3 PL d 安全输入,不能用普通按钮代替。FR3 出厂随箱配一个(Product Manual §6.1 Devices "1x External Enabling Device"),如果丢了从 Franka 订 spare(Product Manual §6.3 spare parts 含 "External Enabling Device")。

### 10.6 安全注意

- 整个过程**人在机械臂工作半径之外**(Operating Manual §9.1.5 NOTICE:"When operating the External Enabling Device, make sure that you are outside the hazardous area to check the execution of the recovery from a safe distance.")
- recovery 期间机械臂会**自动转动到 reference position**,确保该关节运动路径上没东西
- 若运动方向不对 / 撞到东西 → 立即松开 External Enabling Device = 立即停止
- 视觉确认到位再 CONFIRM,有任何疑问直接联系 Franka 支持(Operating Manual §9.1.5 末尾)

### 10.7 重要 don'ts

- **不要点弹窗的 Reboot** — error 写盘,reboot 不清
- **不要点弹窗的 Shut Down 然后拔电** — 拔电可能进 Rescue Mode,从 Rescue 出来再来一次更难
- **不要试图用 Emergency Unlocking Tool 手动把关节扳回 saved_position** — (1) brake 锁着扳不动会损伤机械结构 (Arm Manual §3.4.1 WARNING "Falling heavy Arm when using Emergency Unlocking Tool / Do not use the Emergency Unlocking Tool while the Arm is powered on.");(2) 即使能扳回也不会清 error,Desk 比较的是 saved 和 encoder,扳哪个都不会让 admin 突然能 ACK
- **不要尝试 Factory Reset**(从 Rescue Mode 进入)— 会清掉所有 apps、tasks、End-Effector 配置、safety scenarios

---

## 11. 关机(必须走 Desk!)

**绝不拔电** — 控制柜在写内部存储时被切电会损坏系统镜像,下次开机进 Rescue Mode。

完整步骤参考 `~/franka_scripts/franka_runbook.md` §9,要点:

1. **先 home pose**:`echo_robot_state` / `move_to_home`(`~/franka_scripts/move_to_home`),让机械臂收到紧凑姿态再 brake — 否则 brake 长时间承重会损伤
2. (可选)Desk 下拉 → **Deactivate FCI**(Shut down 也会做)
3. Desk header 下拉 → **Shut down → CONFIRM**
4. 从 Mac 探:`ssh franka-backup 'until ! ping -c1 -W1 172.16.0.2 >/dev/null; do sleep 2; done; echo controller_off'`
5. 三次连续失败后(~6s)才安全切墙电

---

## 12. 常见报错对照表

| 报错 | 真因 | 处理 |
|---|---|---|
| `franka::NetworkException: Connection timeout` | eno1 NO-CARRIER 或 IP 没拿到 | 回 §1 / §2 |
| `Connection to FCI refused`(launch_robot 也报) | Desk FCI 没激活 | 回 §6 |
| `Connection to FCI refused`(只 launch_gripper 报,robot 正常) | End-Effector 不是 Franka Hand | 回 §7 |
| `Service is not available`(Desk 内点 Activate FCI) | FCI service 还没起完 | 等 30–60s 再点,**不要**重启 |
| `Brakes are closed, please unlock the brakes` | Unlock 和 Activate FCI 之间停太久,自动重锁 | 回 §5 立刻重做 |
| `UDP receive: Timeout`(TCP 连得上但 UDP 收不到) | UFW 规则掉了 / 别的客户端没释放控制 | NUC 上 `sudo iptables -L ufw-user-input -n -v` 看 `eno1 from 172.16.0.0/16 ACCEPT` 在不在 |
| `Robot not operational` | 初始化没完 / FSoE error 没 ACK | 等 10s,ack §4 的 FSoE 弹窗 |
| `KeyError: 'CONDA_PREFIX'` | 没 `conda activate polymetis-local` | 用 §8 的 helper 脚本 |
| `sudo: a terminal is required to read the password` | nohup/setsid 下 sudo 拿不到 tty | 用 §8 的 helper 脚本(`echo pw \| sudo -S -v` 预刷) |
| RealSense `errno=16 EBUSY` | uvcvideo 卡死 | `echo q1w2 \| sudo -S modprobe -r uvcvideo && echo q1w2 \| sudo -S modprobe uvcvideo` |
| 浏览器跳 `/rescue` 而不是 `/desk/` | 控制柜没启动正常镜像 | 物理 power-cycle 控制柜,**不要**点 Rescue 里的 "Factory Reset" |
| `Running kernel does not have realtime capabilities` | 启动到 non-RT kernel | GRUB 选 `Advanced options → 5.15.0-rt17`(默认就是 RT) |
| Desk 弹窗 "Joint Position Error detected" 只给 Shut Down / Reboot 两个按钮 | 用 admin (frankaUser) 登的 — admin 看不到 ACK 按钮 | §10:用 admin 创建 Safety Operator → 登入 Safety Operator → START RECOVERY |
| Safety Operator 登入后弹窗有 START RECOVERY,但点了之后机械臂不动 | 没插 External Enabling Device,或没接到机械臂底座 X4 | §10.5:接到 X4,半按持续按住 |
| 机械臂底座红灯慢闪 | safety violation 视觉指示(引 Arm Manual §4.10.1) | 走 §10 全套恢复流程 |
| Settings → Users 里建 Safety Operator 时要求物理验证 | (可能)Watchman/Settings 在某些版本需要按机械臂蓝按钮 | 现场按住手腕 robot pilot 蓝按钮 ~2 秒(参考 Operating Manual §7.2.4) |
| 弹窗 "Joint Limit Violation detected"(类似 §10 但是 limit 不是 position) | joint 被推出 hardware limit | 同样需要 Safety Operator + External Enabling Device,引 Operating Manual §9.1.4 |

---

## Credentials

- Desk web admin: `frankaUser` / `frankaPass123`
- **Safety Operator** 账号 credentials 不是出厂默认 — 需要 admin 手动创建(§10.4 流程),账号 + 密码自定。**强烈建议给项目存一份**(项目内部知识库 / 1Password / 等)防止下次只剩一台 Franka 但没人知道凭据的窘境
- NUC user: `franka`,sudo 密码 `q1w2`(在 helper 脚本里硬编码,改 helper 时同步)
- Robot IP(shop floor net,eno1): `172.16.0.2`
- NUC IP 同网: `172.16.0.1`
- Internal robot net(master controller ↔ arm): `192.168.3.x`(一般不直接交互)

## 引用

- `docs/franka_research_3_operating_manual_5.6.0.pdf`
  - §7.1 Personnel + §7.2 User Roles(Operator / Administrator / Safety Operator)
  - §7.2.4 Creating and editing users(p.47)— 建 Safety Operator 流程
  - §9.1.4 Joint limit error
  - §9.1.5 Joint position error(p.56-58)— 整套 recovery 流程 + 截图
- `docs/franka_research_3_product_manual.pdf`
  - §3.4.1 Emergency unlock label + Emergency Unlocking Tool warnings(p.12)
  - §3.4.3 External Enabling Device 类型标签(p.13)
  - §3.4.4 Emergency Stop Device(p.14)
  - §6.1 Scope of delivery / §6.3 Available spare parts(External Enabling Device 列在备件)
  - §7 Mounting and installation,Figure 31 Overview of interfaces(X4 在 Arm 底座)
- `docs/franka_watchman_operating_instructions.pdf`
  - §2 Watchman 简介 — Roles and Personae,admin 创建 Safety Operator,只有 Safety Operator 能编辑 Watchman / safety configuration
  - §2.2 Overview — Read-only scenarios 包含 "Position Error Recovery" / "Joint Limit Recovery" 等(对应 §10 走的就是这条 read-only scenario)
- `~/franka_scripts/franka_runbook.md` (NUC) — 上游另一份完整 runbook(libfranka build / move 限位 / shutdown 细节 / pre-built motion programs),Mac-side 视角看本文
- `~/franka_scripts/AGENT.md` (NUC) — agent-friendly compressed 版,带具体脚本路径
- [`ui.md`](./ui.md) — IP debug UI 启动 + 录 demo
- [`pipeline.md`](./pipeline.md) — IP 推理 server / ZMQ 协议
