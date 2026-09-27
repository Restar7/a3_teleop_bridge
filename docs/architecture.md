# architecture.md — 系统架构与数据边界

> 目标链路:`PICO 4 → XRoboToolkit → SMPL → UMR → A3 canonical reference →
> short-horizon predictor → A3-fast/SONIC → A3 whole-body policy → A3 真机`

## 1. 数据边界(唯一允许跨阶段的接口)

```text
PICO / XRoboToolkit
   ↓   (packed ZMQ: topic + 1280B JSON header + binary fields)
HumanSmplFrame            [24×3 joints, 21×3 axis-angle, root quat]
   ↓   UMR(离线 / online step)
A3CanonicalState          [root pos/quat, 29 joint pos/vel, valid]
   ↓   predictor(One Euro + 常速外推 + 四元数指数映射)
A3ReferenceWindow         [10 × (root pos/quat + 29 q + 29 dq)]
   ↓   A3_REFERENCE_V1(单帧 ZMQ,CONFLATE)
StreamingReferenceProvider(SONIC 侧)
   ↓   SONIC 自己的 build_tokenizer_terms / root_ori_diff_6d
obs_dict[1570] → A3-fast → action[29] → A3 safety/runtime → robot
```

四个结构定义在 `src/a3_teleop_bridge/types.py`,协议定义在
`src/a3_teleop_bridge/transport/protocol.py`,两者在 4090 阶段结束后**冻结**
(方案 §51)。

## 2. 进程架构(4090 阶段)

```text
pico_zmq (SONIC streamer)          bridge                            sonic_for_a3
─────────────────────────  ─────────────────────────────  ─────────────────────────
XRoboToolkit ─► PUB ──────► PicoPoseSubscriber
                             │ HumanSmplFrame
                             ▼
                          (UMR offline / online)  ─► A3CanonicalState
                             │
                             ▼
                          A3ReferencePredictor    ─► A3ReferenceWindow
                             │
                             ▼
                          ReferencePublisher PUB ──► StreamingReferenceProvider
                                                        │ (encoder il order)
                                                        ▼
                                                     A3-fast + MuJoCo
```

* **bridge 负责轨迹**(canonical state / window / 传输)
* **SONIC 负责 observation 构造**(6D orientation diff、local transform、heading)
  —— 见方案 §41

## 3. 进程架构(真机 V1,方案 §71/§88)

```text
Jetson Orin │ pico_receiver / smpl_adapter / umr_online / a3_predictor /
            │ reference_publisher / camera_source / vision_app
            │            │ Ethernet(A3_REFERENCE_V1)
            ▼            ▼
A3 MDU/RK3588 │ reference_subscriber / A3-fast RKNN 50Hz / robot state /
              │ safety watchdog / serial+parallel solver / motor command
```

第一版**不**绕过 MDU、不重写电机控制与安全层,Orin 只发布参考轨迹。

## 4. 频率与延迟预算(方案 §72,实测见 `docs/benchmark.md`)

| 阶段 | 目标 | 实测(4090) |
| --- | --- | --- |
| PICO | 到达频率 | — |
| UMR | 25–50 Hz | **36 Hz**(27.8 ms/帧) |
| predictor | 50 Hz | 0.48 ms/帧 |
| reference publish | 50 Hz | 50.0 Hz(实测) |
| network(localhost) | — | 0.13 ms |
| A3 policy | 50 Hz | 20 ms 周期 |

桥接侧额外开销(UMR 之后 → policy 之前)≈ **0.76 ms**,远低于 20 ms 周期。

## 5. 已知边界

* `dance1_subject2`(LaFan1 舞蹈,含跳跃/旋转)在 tick 1504 倒地 —— 属于方案 §21
  明确排除的动作类型,当前版本不追求该类动作。
* 并行机构(踝/腰)的 motor joints 在 UMR 中被锁死为 0:retarget 只优化 29 个
  policy joints,杆件在 UMR 可视化中不随动;MuJoCo sim2sim 用的是官方 MJCF 与
  官方 solver,不受影响。
* `--batch-once` 的 MuJoCo 循环不受实时时钟约束,耐久测试中会复用最新窗口;
  实机/实时模式必须使用真实时基。
