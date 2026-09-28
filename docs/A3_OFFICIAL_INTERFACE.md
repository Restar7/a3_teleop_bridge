# A3_OFFICIAL_INTERFACE.md — 官方 aimdk v3.2 接口对照与实机连接

> 来源:[AimDK A3 高层接口开发指南 v3.2](https://open.agibot.com/docs/aimdk/a3/v3_2/dev_guide)
> (适用 A3 软件版本 3.2.x,包名 `AimDK-A3-V3.2-0815`)。
> 本文只摘录与本项目相关的部分,并说明**我们的栈怎么接上去**。
> 官方文档优先于本文;任何冲突以官方为准。

---

## 1. 三种部署形态(官方 §5)

| 形态 | 说明 | 适用 |
| --- | --- | --- |
| **本体 HDU 部署** | 程序直接跑在机器人自带的 HDU 上;推荐目录 `/agibot/data/home/agi/Desktop`(磁盘清理白名单) | 资源占用小的二开程序;占用大时需要先关掉 HDU 上原有程序释放 CPU/GPU |
| **三方工控机部署** | 额外一台工控机(如你的 5060 机器),通过 Type-C/网络与 HDU 通信 | **算力充足、机上软件功能全部保留** —— 本项目的推荐形态 |
| Docker | C++ 或需要改系统依赖时使用;**用官方包安装,不要 apt**;`data-root` 改到白名单路径;容器要 `--ipc=host --net=host`,用户 uid/gid 1001,并共用 `/agibot/software/v0/entry/cfg/ros_dds_configuration.xml`(`ROS_DOMAIN_ID=232`、`FASTRTPS_DEFAULT_PROFILES_FILE`) | 交叉编译 / 非 python 依赖 |

**有线连接(官方 §5.2)**:机器人**头部有与 HDU 连接的 Type-C 接口**,把它接到你的机器上即可与 HDU 通信。
这是「5060 当部署机」的物理连接方式。

---

## 2. 官方运动控制接口(官方 §7.1)

**重要认知**:A3 **出厂自带运控(MC)**,行走/上肢/跳舞都由它内部状态机驱动,二开只发**高层指令**。
状态机:`PASSIVE`(默认)→ `DAMPING` → `PD_STAND` → **`MOTION`**(可走、可做上肢动作) → 坐/躺/起身系列。
一般**只允许在力控状态之间切换**,其余切换要吊装、只能手动。

### 2.1 状态机 RPC(HTTP JSON)

```text
SetAction            POST http://<HDU_IP>:56322/rpc/aimdk.protocol.MotionControlActionService/SetAction
GetAction            POST .../GetAction
GetAvailableActions  POST .../GetAvailableActions
```

### 2.2 ROS2 Topic

| 话题 | 作用 | 关键约束 |
| --- | --- | --- |
| `/motion/control/locomotion_velocity` | 行走(前/侧/转向,`-1..1` 比例系数,MC 自己算速度) | 仅 `MOTION` 状态;消息类型 `ros2_plugin_proto/msg/RosMsgWrapper`,需 `source prebuilt/ros2_plugin_proto_aarch64/share/ros2_plugin_proto/local_setup.bash` |
| `/motion/control/arm_joint_command` | **14 个手臂关节**目标角(`sensor_msgs/JointState`) | 推荐 **100 Hz**;相邻指令间隔 ≤30 ms;速度 ≤4 rad/s;建议低通滤波;仅 `MOTION` 状态 |
| `/motion/control/arm_joint_state` | 手臂状态(角度/速度/力矩) | 无 Action 限制 |
| `/motion/control/move_waist` | 腰 yaw `±1.6 rad` + 腰高 `-0.3..0 m` | 需先关 `motion_player` |
| `/motion/control/neck_joint_command` | 头 yaw `±1.0472`、pitch `-0.43633..0.26180` | 仅 `MOTION` 状态 |

**手臂关节名字(官方原文,顺序即下发顺序)**:

```text
left_shoulder_pitch_joint, left_shoulder_roll_joint, left_shoulder_yaw_joint,
left_elbow_joint, left_wrist_roll_joint, left_wrist_pitch_joint, left_wrist_yaw_joint,
right_shoulder_pitch_joint, right_shoulder_roll_joint, right_shoulder_yaw_joint,
right_elbow_joint, right_wrist_roll_joint, right_wrist_pitch_joint, right_wrist_yaw_joint
```

**手臂限位(官方原文,rad)**:shoulder_pitch `±2.87979`;shoulder_roll 左 `-0.08727..2.61799` / 右 `-2.61799..0.08727`;
shoulder_yaw `±2.79253`;elbow `-0.95993..1.74533`;wrist_roll `±2.79253`;wrist_pitch `±1.62316`;wrist_yaw `±1.62316`。

> ✅ 这组名字与顺序**正好等于**我们 `generated/a3_contract.json` 里的 A3 policy 手臂段与
> `a3_teleop_joint_order.hpp` 的 `kA3PolicyJointNames[3..16]`。
> 也就是说之前记在 `A3_ONBOARD.md` 的「proto 注释里手臂命名不同」的疑点,在**官方 ROS2 接口**这一侧是明确的:
> A3 的手臂命名就是 `elbow / wrist_roll / wrist_pitch / wrist_yaw`,按名字映射即可。

### 2.3 必须先关 `motion_player`

除行走外的关节控制话题默认被 `motion_player` 占用:

```bash
# 关闭(下发关节命令前)
curl -i -H 'content-type: application/json' -X POST \
  'http://127.0.0.1:50080/json/stop_app' -d '{"app_name": "motion_player"}'
# 恢复
curl -i -H 'content-type: application/json' -X POST \
  'http://127.0.0.1:50080/json/start_app' -d '{"app_name": "motion_player"}'
```

关闭后无法再播放资源管理模块里的动作。

---

## 3. 官方仿真:AimSim(官方 §9)——**上机前先在它里面试**

官方提供 MuJoCo 试验场,内含官方运控模块,用于「实机运行前的基本验证,降低损坏机器人本体的风险」。

```bash
# 1) 安装(需要 python 3.10)
python3.10 -m pip install aimsim-3.1.3-py3-none-any.whl      # 位于 AimDK 包 example/AimSim 下
python3.10 -m pip list | grep aimsim

# 2) 起 MuJoCo(默认机型 a3_t2d5,与运控的通信后端 iceoryx)
aimsim mujoco start
#   终端里键入 “l” 可重置机器人到初始位置

# 3) 起官方运控模块,并按状态机切换
./example/AimSim/mc/script/motion_control/start_motion_control.sh
./example/mc/S_SetAction.py     # 切到 get_up,机器人站起
./example/mc/S_SetAction.py     # 切到 motion
./example/mc/walk.py            # 让机器人行走
```

可换机型 / `ros_domain_id` / 通信后端(iceoryx 或 **ros2**):

```bash
aimsim mujoco init-config --user-config-path {user_path}
# 改 {user_path}/mujoco/config_mujoco.yaml 或 .../mujoco_simulator_cfg_sil.yaml
aimsim mujoco start --user-config-path {user_path}
```

**对我们的意义**:这是**官方认可的、和真机同一套接口**的仿真。所以:

```text
先在 AimSim 里用官方脚本把 action/话题调通(证明网络与话题对)
      ↓
再把我们的 PICO→UMR→参考 接到同样的接口上(AimSim 里跑遥操)
      ↓
最后才上真机(悬吊)
```

---

## 4. 两条技术路线(必须明确选一条,不要混)

### 路线 A — 用官方运控 + 上肢遥操(**低风险,官方文档直接覆盖**)

```text
PICO → SMPL → UMR → A3 参考(29 关节)
                        ├── 手臂 14 关节 → /motion/control/arm_joint_command(100 Hz)
                        ├── 腰 3 关节    → /motion/control/move_waist(yaw + 高度)
                        ├── 头 2 关节    → /motion/control/neck_joint_command
                        └── 行走         → /motion/control/locomotion_velocity(速度比例)
```

- 优点:**不接管行走**,机器人用自己的运控,安全性最高;先在 AimSim 里验证;接口是 ROS2,文档完整。
- 代价:不是"人怎么动机器人怎么动"的全身遥操 —— 下肢只给速度,腰部只给 yaw/高度,**不能复现抬脚/屈膝等下肢姿态**。
- 我们已有的东西可直接复用:online UMR 输出的 29 关节参考里,手臂/腰/头都能按名字取出;
  需要新写一个 **ROS2 适配节点**(≈200 行)替代现在的 AimRT 版本(publisher 已抽象好)。

### 路线 B — SONIC 全身 policy(A3-fast)接管全身 29 关节

```text
Orin/5060: A3_REFERENCE_V1 ──► /body_drive/*_joint_command(或 TA 通道)
A3 MDU:    A3-fast(RKNN)50 Hz → 29 关节 → 官方安全层 → MDU → 电机
```

- 优点:真正的全身遥操(含下肢),这正是 `sonic_for_a3` 与 GR00T 的路子,我们已经把
  Python/C++ 两侧都写好并测过。
- 代价:要**接管 body drive 话题**(官方文档里这些属于更底层的二开路径),
  需要按官方 §6.1 把 Motion 功能组改成只起 `agent`、并确保 `motion_control` 不再下发;
  风险与验证量明显更大。

> **本项目决定走路线 B**(2026-09:见 `DEPLOY_TARGET_DECISION.md`)。下面的 A 路线分析保留作为
> 备选与风险对照 —— 如果 B 的 MDU 接管在真机上遇到阻塞,A 可以随时接回来(两者共用同一份
> UMR 输出,只差一个发布后端)。

> 建议:**先做路线 A**(一到两周内能上机、风险低、AimSim 可全流程验证),
> 路线 B 作为第二阶段在路线 A 稳定后推进。这也是方案 §52/§85「不要第一版就绕过 A3 原装控制板」的直接落地。

---

## 5. 待现场确认清单(官方文档没写死、必须实测)

```text
[ ] HDU 的 IP / Type-C 连接后的网段(文档里 RPC 示例是 10.42.10.12)
[ ] ROS2 环境:source /agibot/software/v0/entry/cfg 相关 setup、ROS_DOMAIN_ID=232 是否已设
[ ] motion_player 关闭/恢复脚本在你的软件版本上的实际行为(关闭后哪些功能不可用)
[ ] /motion/control/* 话题的实际频率与延迟(用 ros2 topic hz / delay 测)
[ ] 关节限位与我们的 reference 是否一致(用 generated/a3_joint_limits.yaml 对拍)
[ ] MOTION 状态下同时下发手臂命令 + 行走速度是否被允许(文档说明两者可同时)
[ ] 5060 与 HDU 之间的带宽/抖动(参考流 50 Hz × ~3 KB 很小,但 ROS2 DDS 发现可能对网络敏感)
```

---

## 6. 与官方文档的对照表(我们 ↔ 官方)

| 我们的组件 | 官方对应 | 备注 |
| --- | --- | --- |
| `A3_REFERENCE_V1`(ZMQ,10×29) | 无直接对应 | 我们自己的传输层;真机侧由适配节点转成官方接口 |
| `a3_teleop_command_source.cpp`(il→policy 置换 + 发布泵) | 官方 TA 通道布局(policy view) | 名字与顺序已与官方 ROS2 手臂接口对齐 |
| `a3_teleop_channel_publisher.cpp`(AimRT 发布) | AimRT/TA 通道 | 路线 B 用 |
| (待写)ROS2 适配节点 | `/motion/control/*` | **路线 A 用**,优先做 |
| `docs/SIM_TELEOP.md`(SONIC policy 仿真) | 官方 AimSim(运控仿真) | 两者互补:前者验证我们的 policy 链路,后者验证与官方运控的接口 |
| `A3_ONBOARD.md` §6(MDU 服务配置) | 官方 §6.1 MDU bring-up | 路线 B 才需要改 `sm_config.yaml`;路线 A 不需要 |
