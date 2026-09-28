# DEPLOY_TARGET_DECISION.md — 路线 B 下:部署机选 **Orin**

> 决策:**路线 B 的机载部署机 = Jetson Orin**;5060(或 4090)只做开发/仿真/构建/兜底。
> 本文说明为什么,以及在什么条件下这个决策会翻转、翻转的代价有多小。

---

## 1. 先把「路线 B 里谁干什么」摆清楚(这是决策的前提)

路线 B = **SONIC A3-fast 全身 policy 接管 29 关节**,而 policy 跑在 **A3 原装计算板(MDU/RK3588,RKNN)** 上:

```text
部署机(Orin / 5060)                         A3 本体
──────────────────────────────              ──────────────────────────────────────
PICO 接收 + SMPL                     ──►    (只收参考)
online UMR(CPU 1–2 核)                       A3-fast policy 50 Hz(RKNN)
predictor + 发布 A3_REFERENCE_V1   ──以太网─►  官方安全层 → MDU → 电机
适配节点(→ /ta/whole_body_command / AimRT)
```

**关键事实:部署机在这条路线里不需要 GPU。** policy 在 A3 上跑,部署机只做:
SMPL-X 前向(单帧 CPU,7.3 ms)、online UMR(Clarabel QP,CPU)、predictor、ZMQ 发布、AimRT 适配。

我们已实测:

| 项 | 数值 | 是否吃 GPU |
| --- | --- | --- |
| SMPL-X 单帧前向 | 7.3 ms(CPU 比 GPU 快) | 否 |
| online UMR 单帧 | 48 ms = prepare 15.5 + solve 34.3 | 否(`CUDA_VISIBLE_DEVICES=""` 下照跑) |
| predictor + 发布 | <1 ms | 否 |
| 适配节点(解码+置换+发布) | µs 级 | 否 |

所以「5060 的 GPU 更强」在这条路线上**换不来任何东西**;而它带来的代价是:C 口线/网线要拖到机器人、
Wi-Fi 抖动直接进参考流、机器人一动线就成负担。

---

## 2. 决策矩阵

| 维度 | **Orin(选它)** | 5060 |
| --- | --- | --- |
| 部署机需要的 GPU | 不需要 | 不需要(白给) |
| 与机器人的物理连接 | **机载,随机供电,无线** | 需 Type-C 线或机器人网络,存在被拽/断连风险 |
| 现场可靠性 | 高(不依赖外部网络) | 中(Wi-Fi/线缆都是单点) |
| 延迟抖动 | 机内直连,最小 | 多一跳网络 |
| CPU 算力 | 8×A78AE,≈ x86 的 1/3–1/2 → UMR 可能 10–20 Hz | 强,UMR 轻松 20 Hz |
| 官方形态 | 符合方案 §52/§71/§88 与官方「本体部署」精神 | 官方也支持「三方工控机部署」 |
| 开发/仿真/构建 | **不适合**(JetPack 版 torch、无 x86 Docker 交叉编译便利) | **适合**(仿真、AimSim、交叉编译 rockchip 包) |
| 结论 | **机载部署机** | **开发/仿真/构建 + 兜底** |

---

## 3. 决策:Orin 上机,5060/4090 当开发机与兜底

```text
4090 / 5060(开发机)                     Orin(机载部署机)              A3 MDU
──────────────────────────              ───────────────────           ──────────────
训练/仿真(MuJoCo、AimSim)          ──►   PICO + SMPL + online UMR  ──►  A3-fast 50 Hz
构建 rockchip 部署包(Docker)             predictor + 发布              安全层
离线 UMR 批量                            (可选)适配节点                 MDU → 电机
UMR 兜底(见 §4)
```

理由浓缩成三条:

1. **路线 B 的部署机是 CPU 活**,Orin 够用,5060 的 GPU 用不上;
2. **遥操要的是"永不掉链"**:机载供电 + 机内直连,比"PC + 线/Wi-Fi"稳一个量级;
3. **开发与部署分开更安全**:仿真/构建在 4090(已就绪),Orin 上只放经过验证的运行时副本。

---

## 4. 这个决策唯一的技术风险与它的兜底

**风险**:Orin 的 CPU 比 4090 主机弱,online UMR 可能达不到 20 Hz。
**判据**(上机第一件事,10 分钟就能测):

```bash
# Orin 上
SSH 到 Orin,cd ~/a3_teleop_ws/a3_teleop_bridge && source scripts/env_orin.sh
$PY_UMR -m a3_teleop_bridge.apps.retarget_live \
    --source recording --backend umr-online \
    --recording $A3WS/recordings/m5_twist_torso_left \
    --duration 20 --no-publish --playback-hz 30 \
    --stats /tmp/orin_umr_stats.json
python3 - <<'PY'
import json; s=json.load(open('/tmp/orin_umr_stats.json'))
lat=s['solver_latency_ms']; print('p50', round(lat['p50'],1), 'p95', round(lat['p95'],1), '→ Hz≈', round(1000/lat['p50'],1))
PY
```

| Orin 实测 p50 | 判定 | 行动 |
| --- | --- | --- |
| ≤ 50 ms(≥20 Hz) | 达标 | Orin 全量跑,结束 |
| 50–100 ms(10–20 Hz) | 可用 | 继续 Orin,靠 latest-only + SONIC 有界插值吸收(方案 §47/§48 允许 UMR 25–50 Hz、policy 50 Hz) |
| > 100 ms(<10 Hz) | 不达标 | **兜底**:UMR 留在 4090/5060,只把 `A3_REFERENCE_V1` 过以太网发给 A3;Orin 只跑发送端+发布(几乎不耗算力) |

**切换代价 = 一行配置**:`configs/network.yaml` 的 `reference.bind_host/connect_host`。
协议与 A3 侧完全不变(方案 §51 冻结接口),所以这个决策**不是一次性的**,随时可以改。

---

## 5. 路线 B 的上机路径(与 A 的区别用 ⚠️ 标出)

```text
0. 开发机(4090):交叉编译 rockchip 部署包(需要 ROS2 + aarch64 ONNXRuntime;Docker 才能交叉)
   ⚠️ 包内已包含我们的 a3_reference_stream / a3_teleop_command_source /
      a3_teleop_channel_message / a3_teleop_channel_publisher(24+19+16 项单测通过)
1. 传到 MDU(经 HDU 跳板)
   ⚠️ 官方 §6.1:Motion 功能组改成只起 agent,确认 motion_control 不再下发
      —— 这是路线 B「接管全身」的核心动作,路线 A 不需要
2. 包预检 + receive-only probe(A3_PROBE_SOURCE=g1,taskset -c 4-5 ./run_a3_probe.sh)
   ⚠️ 此时策略在跑但不发指令,先核对关节顺序/IMU/新鲜度
3. 起参考:Orin 上 run_orin_live.sh → 适配节点发布 /ta/whole_body_command
   ⚠️ 关节顺序由生成的置换表保证(il ↔ policy),单测里逐名字核对过
4. 悬吊 + 10 级动作(stand → shoulder → elbow → wrist → torso → knee → weight shift
   → foot unload → foot lift → slow step),任一级失败即停
5. 有限自由遥操(方案 §64/§84 M12)
```

安全边界(路线 B 必须写进现场规程):

```text
· policy 在 A3 上,但**参考来自外部**;参考断了 → 50 ms 内 hold,250 ms 内 safe halt(已实现并测过)
· 我们**不碰**官方安全层 / MDU / 电机;只提供参考
· 悬吊、防坠、物理急停、安全员,一个都不能省;没有过 MuJoCo + AimSim 不上真机
```

---

## 6. 一句话结论

> **手上只有 5060 的话:直接用 5060,照 `ORIN_FULL_RUNBOOK.md` 做**(差异见它的 §13,
> 只有 4 处,主要是仿真要 cu128+ 的 torch)。本决策的前提是「两台都有、要挑一台」——
> 那种情况下挑 Orin(机载可靠);只有一台时,**任何一台都能完整跑通全流程**。

> **部署机选 Orin。** 5060(或现在的 4090)是开发机:跑仿真、跑 AimSim、交叉编译部署包,
> 并在 Orin 的 UMR 速率不达标时接管 UMR(改一行 `bind_host` 即可)。
> 路线 B 的 GPU 在 A3 上(RKNN),不在部署机上——所以选型看的是**可靠性与机载性**,不是算力。
