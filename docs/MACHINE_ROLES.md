# MACHINE_ROLES.md — 哪台机器跑什么(4090 / 5060 / Orin / HDU / MDU)

> **决策更新(路线 B,见 `DEPLOY_TARGET_DECISION.md`)**:**机载部署机选 Orin**;
> 5060 / 4090 作为开发机(仿真、AimSim、交叉编译部署包)与 UMR 兜底。
> 路线 B 的 policy 跑在 A3 的 RKNN 上,部署机是纯 CPU 活,**不需要 GPU** ——
> 所以 5060 的 GPU 在部署角色上用不上,而它作为开发机非常合适。

> 结论先给:**RTX 5060 的机器完全可以当部署机** —— 官方支持「三方工控机部署」
> ([aimdk §5](https://open.agibot.com/docs/aimdk/a3/v3_2/dev_guide/05-second_develop_program_deployment)),
> 我们的在线链路又几乎不吃 GPU。但有几条硬约束要先确认(见 §3)。

---

## 1. 角色分工

| 机器 | 建议角色 | 跑什么 | 能不能省 |
| --- | --- | --- | --- |
| **4090 工作站** | 开发/训练/离线批量 | 离线 UMR 批量、MuJoCo 长跑验收、打包 | 可省(5060 也能跑,只是批量慢) |
| **5060 机器** | **部署机 + 遥操主机**(推荐) | PICO(PC Service + 发送端)、online UMR、predictor、参考流发布、AimSim 仿真、ROS2/AimRT 适配节点 | 这是本方案的主角 |
| **Orin** | 机器人本体侧的遥操端(V1 目标形态) | 同上,但搬上机器人侧、功耗低 | 5060 可先替代它做验证 |
| **HDU**(机器人自带) | 官方推荐的本体部署位置 | 二开程序 / Docker;`/agibot/data/home/agi/Desktop` | 与 5060 二选一 |
| **A3 MDU/RK3588** | 官方运控 + 安全 + 电机 | 路线 A:官方运控;路线 B:A3-fast(RKNN) | 必须保留,不可绕过 |

```text
路线 A(推荐先做):
  PICO ──Wi-Fi──► 5060(PC Service + 发送端 + online UMR + ROS2 适配节点)
                        │  Type-C / 以太网(官方 §5.2:机器人头部 Type-C 接 HDU)
                        ▼
                   A3 HDU ──► 官方运控(motion_control)──► MDU ──► 电机

路线 B(第二阶段):
  5060 / Orin ──A3_REFERENCE_V1──► A3 MDU(A3-fast RKNN)──► 官方安全层 ──► 电机
```

---

## 2. 各部件对算力的实际需求(实测)

| 部件 | 资源 | 实测/说明 |
| --- | --- | --- |
| online UMR | **CPU ≈ 1–2 核** | 单帧 48 ms(prepare 15.5 + solve 34.3),**不需要 CUDA**(见下) |
| SMPL-X 单帧前向 | CPU | 7.3 ms(batch=1 时 CPU 比 GPU 更快) |
| predictor + publisher | CPU,可忽略 | encode/decode 0.052/0.094 ms,ZMQ 回环 0.130 ms |
| PICO PC Service + SDK | CPU,轻 | Qt6 服务 + pybind |
| SONIC A3-fast policy(仿真) | **GPU** | 50 Hz 推理,显存 <2 GB |
| AimSim / MuJoCo | GPU/CPU 皆可 | 官方运控仿真 |
| 离线 UMR 批量 | GPU(可选) | 批量 SMPL-X 在 4090 上更快;在线路径不用 |

**结论**:5060 的 8 GB 显存对「policy 仿真 + AimSim」绰绰有余;在线遥操链路本身是 CPU 活。

---

## 3. 5060 的四条硬约束(先确认,再动手)

### 3.1 GPU 架构是 Blackwell(sm_120)→ 必须用新 CUDA 的 torch

RTX 50 系是 `sm_120`。老 wheel(如 `cu121`)**不含 sm_120 内核**,加载即报
`no kernel image is available for execution on the device`。

```bash
# 在 5060 上确认
nvidia-smi --query-gpu=name,compute_cap,driver_version --format=csv,noheader
# 期望:..., 12.0, 5xx.xx
python -c "import torch; print(torch.__version__, torch.version.cuda, torch.cuda.is_available())"
```

| 用途 | 需要的 torch | 说明 |
| --- | --- | --- |
| **SONIC 仿真 / policy**(吃 GPU) | CUDA **≥12.8** 的构建(`cu128`/`cu129`/`cu130`) | 我们 4090 上现有的是 `2.14.0+cu130`,这一代支持 sm_120 |
| **在线 UMR**(我们已改成 CPU) | 任意 torch 都能跑 | 实测 `CUDA_VISIBLE_DEVICES=""` 下照常工作,solve 23–100 ms |
| 离线 UMR 批量(可选) | 同上 CUDA 版本 | 没有 GPU 也能跑,只是慢 |

> 换句话说:**我们的遥操链路不受 sm_120 影响**(它是 CPU 活);只有 SONIC policy 仿真需要新版 torch。
> 如果你的 5060 机器上装的是老 cu121 wheel,SMPL-X/UMR 照跑,但 **MuJoCo policy 仿真会失败**。

### 3.2 系统建议 Ubuntu(而不是 Windows)

官方二开文档、ROS2、AimRT、iceoryx、Docker 流程都按 Ubuntu 写。Windows 上「PICO PC Service」有官方包,
但我们的 bridge / UMR / ROS2 节点在 Windows 上属于未验证路径。**建议 5060 装 Ubuntu 22.04(与 4090 一致)**。

### 3.3 与机器人的网络

官方路径是「机器人头部 Type-C ↔ HDU」;三维工控机形态则需要能访问 HDU 的 IP
(文档示例 `10.42.10.12`、RPC 端口 `56322`、服务端口 `50080`)。先在 5060 上确认:

```bash
ping <HDU_IP>
curl -s -X POST http://<HDU_IP>:50080/json/stop_app -H 'content-type: application/json' \
     -d '{"app_name": "motion_player"}'        # 仅在确认要接管关节话题时才执行
```

ROS2 侧需要与机上一致的 DDS 配置:

```bash
export ROS_DOMAIN_ID=232
export FASTRTPS_DEFAULT_PROFILES_FILE=/path/to/ros_dds_configuration.xml   # 从 HDU 拷
ros2 topic list        # 应能看到 /motion/control/* 等话题
```

### 3.4 时间同步

参考流用时间戳标定新鲜度(桥梁的 watchdog 是 50 ms 量级)。5060 与机器人侧请开 NTP/chrony:

```bash
timedatectl status
sudo chronyc tracking 2>/dev/null || sudo timedatectl set-ntp true
```

---

## 4. 在 5060 上从零到能跑(命令)

```bash
# 0) 系统与驱动
nvidia-smi                                  # 驱动正常、compute_cap 12.0
sudo apt-get update && sudo apt-get install -y git git-lfs python3-venv rsync curl

# 1) 取代码(SSH 部署密钥见 docs/DELIVERY.md §3)
mkdir -p ~/a3_teleop_ws && cd ~/a3_teleop_ws
git clone -b feat/a3-streaming-reference git@github.com:Restar7/sonic_for_a3.git
git clone git@github.com:Restar7/a3_teleop_bridge.git
git clone -b feat/a3-online-retarget git@github.com:Restar7/UMR.git

# 2) 环境(一条命令;脚本会检查前置)
cd a3_teleop_bridge
source scripts/env_orin.sh                  # 路径派生自脚本位置,5060 同样适用
bash scripts/orin_preflight.sh              # 系统/依赖/zmq 自检 -> orin_system_info/
bash scripts/orin_bootstrap.sh --with-umr   # 建 .venv_bridge 与 .venv_umr
T中若 torch 需要换 cu128/cu130 wheel:
#   .venv_umr/bin/pip install torch --index-url https://download.pytorch.org/whl/cu130
bash scripts/check_orin_ready.sh            # 期望 16 ok, 0 failed

# 3) 许可证资产(不在仓库里)
#    从 4090 拷:UMR/smpl/SMPLX_NEUTRAL.pkl -> ~/a3_teleop_ws/UMR/smpl/

# 4) 不带头显先跑起来(回归:A 离线、B 录制驱动)
#    recordings/ 不在仓库里;新机器上先造一份合成录制:
#    .venv_bridge/bin/python tools/make_synthetic_pico_recording.py \
#        --clip <smplx npz> --out ~/a3_teleop_ws/recordings/m5_twist_torso_left
.venv_bridge/bin/python -m pytest tests integration -q
$PY_UMR -m a3_teleop_bridge.apps.retarget_live --source recording --backend umr-online \
    --recording ~/a3_teleop_ws/recordings/m5_twist_torso_left \
    --duration 8 --no-publish --stats /tmp/online.json

# 5) SONIC 仿真(需要新版 CUDA torch)
cd ~/a3_teleop_ws/sonic_for_a3 && python check_environment.py
bash install_scripts/install_mujoco_sim.sh
.venv_sim/bin/python download_from_hf.py --component pt onnx rknn
cd ~/a3_teleop_ws/a3_teleop_bridge && source scripts/env_orin.sh
bash scripts/run_a3_streaming.py --csv ../logs/a3_validation/endurance_loop.csv \
     --policy-steps 3000 --duration 300        # 期望 ACCEPTED / fall=false

# 6) 真 PICO 遥操仿真(见 docs/SIM_TELEOP.md)
#    终端1:PICO 发送端  终端2:run_live_chain.py --pico

# 7) 官方运控仿真 AimSim(先证明和官方接口通)
python3.10 -m pip install <AimDK>/example/AimSim/aimsim-3.1.3-py3-none-any.whl
aimsim mujoco start                        # 另开终端
./example/AimSim/mc/script/motion_control/start_motion_control.sh
./example/mc/S_SetAction.py && ./example/mc/walk.py
```

---

## 5. 什么时候必须上 Orin / HDU

```text
可以一直用 5060:
  仿真(SONIC policy、AimSim)、PICO 遥操仿真、离线/在线 UMR 验证、接口联调

需要搬到机器人侧(Orin 或 HDU)的理由:
  · 现场不希望拖一根线到工控机 / 需要机器人自走
  · 网络抖动导致参考流抖动(PC Service 与发送端同机时可把 PICO 放机器人侧)
  · 官方推荐形态(本体 HDU 部署)

不需要的理由(别被"必须 Orin"绑住):
  · 5060 的算力比 Orin 强,遥操链路本身是 CPU 活
  · 官方明确支持三方工控机部署
```

---

## 6. 相关文档

```text
docs/A3_OFFICIAL_INTERFACE.md   官方 aimdk v3.2 接口对照 + 两条路线 + AimSim 流程
docs/SIM_TELEOP.md              PICO 遥操在 MuJoCo 里的完整步骤
docs/DEPLOY_ORIN.md             Orin 部署(搬到机器人侧时用;脚本同样适用 5060/HDU)
docs/A3_ONBOARD.md              机载侧(MDU/body drive)与 AimRT 适配节点
docs/DELIVERY.md                交付物、仓库、部署公钥、推送命令
docs/GO_LIVE_CHECKLIST.md       从打包到悬吊真机的单页清单
```
