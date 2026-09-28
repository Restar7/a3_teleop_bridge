# ORIN_FULL_RUNBOOK.md — Orin 从零到推理:克隆 / 环境 / 仿真 / 上机

> 一条主线,全部命令可直接复制:**Orin 开箱 → clone 三个仓库 → 建环境 → 下模型 → 自检 →
> 仿真(PICO 遥操)→ 参考推理 → 连真机(路线 B)**。
> 这是把 `SIM_TELEOP.md`(仿真遥操)与 `GO_LIVE_CHECKLIST.md` / `A3_ONBOARD.md`(真机)
> 在 **Orin** 上的合并版;细节原理仍以那三份为准。
>
> 选型依据见 `DEPLOY_TARGET_DECISION.md`:**路线 B 的 policy 跑在 A3 的 RKNN 上,Orin 只做
> 参考生成与发布(纯 CPU 活,不需要 CUDA)**。

---

## 0. 前置条件

```text
[ ] Jetson Orin(已刷 JetPack,能联网),与 PICO 头显同一 Wi-Fi;与 A3 可连(以太网/Type-C)
[ ] Orin 上装了 git / git-lfs / python3.10(或 3.10+) / rsync
[ ] 一个 GitHub 账号(三个仓库:sonic_for_a3 / a3_teleop_bridge / UMR)
[ ] SMPL-X 人体模型(许可证资产,不在任何仓库里;从已有机器拷贝)
[ ] A3 侧:官方 AimDK 包(含 AimSim、example 脚本)
```

端口约定(全链路只有两个,记住):

```text
5556  PICO pose 流(sender 发布 → bridge 订阅)
5560  A3_REFERENCE_V1(bridge 发布 → A3 / MuJoCo 订阅)
```

---

## 1. 系统准备(Orin)

```bash
# 1.1 基本信息(不要假设 JetPack 版本)
uname -m                                    # 期望 aarch64
cat /etc/nv_tegra_release 2>/dev/null || true
nvidia-smi 2>/dev/null | head -5 || true
free -h; df -h / | tail -1

# 1.2 基础包
sudo apt-get update
sudo apt-get install -y git git-lfs python3-venv python3-pip rsync curl build-essential cmake
git lfs install

# 1.3 时间同步(参考流的新鲜度判据是 50 ms 量级,必须同步)
sudo timedatectl set-ntp true && timedatectl status | head -4

# 1.4 工作区
mkdir -p ~/a3_teleop_ws && cd ~/a3_teleop_ws
```

**GPU 说明**:Orin 的 CUDA 只在跑 MuJoCo policy 仿真时才用得上,而且**不是必需** ——
policy 是很小的 MLP,CPU 推理也能跑(慢一些)。参考链路(UMR/SMPL-X/predictor)在 4090 上实测
关掉 CUDA 照常工作。所以 Orin 上不必先去折腾 JetPack 版 CUDA torch。

---

## 2. 从 GitHub 克隆三个仓库(Orin 上直接 clone)

### 2.1 方式一:SSH 部署密钥(推荐,和打包机同一把)

```bash
# 在 Orin 上生成一把专用密钥(如果没有)
ssh-keygen -t ed25519 -N "" -C "a3-teleop-orin" -f ~/.ssh/id_ed25519_github_a3
cat ~/.ssh/id_ed25519_github_a3.pub          # 把这行加到 GitHub → Settings → SSH keys

# 配置 ~/.ssh/config
cat >> ~/.ssh/config <<'CFG'
Host github.com
    User git
    IdentityFile ~/.ssh/id_ed25519_github_a3
    IdentitiesOnly yes
CFG

ssh -T git@github.com                        # 期望 "Hi <账号>! You've successfully authenticated"
```

> 也可以用打包机上那把(`docs/DELIVERY.md` §3 的公钥/指纹),但**私钥不要到处拷** ——
> 推荐每个机器一把,各自加到账号。

### 2.2 克隆(注意分支)

```bash
cd ~/a3_teleop_ws

git clone -b feat/a3-streaming-reference git@github.com:Restar7/sonic_for_a3.git
git clone                               git@github.com:Restar7/a3_teleop_bridge.git
git clone -b feat/a3-online-retarget    git@github.com:Restar7/UMR.git

# 校验
git -C sonic_for_a3  log --oneline -1
git -C a3_teleop_bridge log --oneline -1
git -C UMR           log --oneline -1
```

HTTPS 兜底(没有 SSH 时):

```bash
git clone -b feat/a3-streaming-reference https://github.com/Restar7/sonic_for_a3.git
git clone https://github.com/Restar7/a3_teleop_bridge.git
git clone -b feat/a3-online-retarget https://github.com/Restar7/UMR.git
```

完全离线时:在打包机执行 `bash a3_teleop_bridge/scripts/package_bundle.sh --with-umr`,
把 `dist/a3_teleop_orin_<stamp>.tar.gz`(14 MB)拷过来解包即可。

---

## 3. 环境变量与路径

```bash
cd ~/a3_teleop_ws/a3_teleop_bridge
source scripts/env_orin.sh
# [env] A3WS=... SONIC_A3_ROOT=... UMR_ROOT=... PY_BRIDGE=... reference bind=tcp://0.0.0.0:5560
# 建议写进 ~/.bashrc:
echo 'source ~/a3_teleop_ws/a3_teleop_bridge/scripts/env_orin.sh >/dev/null' >> ~/.bashrc
```

---

## 4. 建环境(一条命令)

```bash
cd ~/a3_teleop_ws/a3_teleop_bridge

bash scripts/orin_preflight.sh          # 系统/依赖/zmq 自检 → orin_system_info/
bash scripts/orin_bootstrap.sh --with-umr
bash scripts/check_orin_ready.sh        # 期望 "16 ok, 0 failed"(含 online UMR 真解一帧)
```

`orin_bootstrap.sh` 建两个环境:

| 环境 | 内容 | 用途 |
| --- | --- | --- |
| `.venv_bridge` | numpy scipy pyzmq PyYAML msgpack pytest(`requirements-bridge.txt`) | bridge 全链路、录制/回放、参考流 |
| `.venv_umr` | torch(**CPU 版即可**)trimesh clarabel smplx + 本仓库 editable | online UMR 求解 |

如果 `torch` 装的是 PyPI 的 aarch64 CPU wheel,这是**预期**的(在线路径不需要 CUDA)。要跑
MuJoCo policy 仿真再用 Jetson 对应的 CUDA wheel 重建 `.venv_umr`/`.venv_sim`。

**仿真/发送端需要的第三个环境**(sonic_for_a3,含 PICO 发送端与 MuJoCo sim):

```bash
cd ~/a3_teleop_ws/sonic_for_a3
python3.10 -m venv .venv_sim
.venv_sim/bin/pip install --upgrade pip wheel
.venv_sim/bin/pip install numpy scipy mujoco pyyaml msgpack pyzmq
.venv_sim/bin/pip install torch --index-url https://download.pytorch.org/whl/cpu   # 或 Jetson CUDA wheel
.venv_sim/bin/python check_environment.py

# PICO 发送端专用最小环境(官方脚本,自动处理 aarch64 的 SDK 库路径)
bash install_scripts/install_pico_minimal.sh        # → .venv_pico_minimal
```

> `install_pico_minimal.sh` 会在 `external_dependencies/.../lib/aarch64` 下取 SDK,
> 并按架构选择库路径(Orin 就是 aarch64 分支)。

---

## 5. 模型与许可证资产

### 5.1 A3 checkpoint / ONNX / RKNN(官方 HuggingFace)

```bash
cd ~/a3_teleop_ws/sonic_for_a3
.venv_sim/bin/python download_from_hf.py --component pt onnx rknn
# 可选:--component all  或  --component sysroot(Docker 交叉编译才需要)
find checkpoints/035_step200000 -maxdepth 1 -type f | sort
find gear_sonic_deploy/assets -name "*.rknn" -o -name "*.onnx" | head
```

没有外网时,在 4090 上下好后 `rsync -avP` 整个 `checkpoints/` 与 `gear_sonic_deploy/assets/`。

### 5.2 SMPL-X 人体模型(**必须手工拷贝,任何仓库里都没有**)

```bash
# 在 4090(或保存处)上推到 Orin
rsync -avP ~/a3_teleop_ws/UMR/smpl/SMPLX_NEUTRAL.pkl <USER>@<ORIN_IP>:~/a3_teleop_ws/UMR/smpl/
# 校验(两端 sha256 应一致)
sha256sum ~/a3_teleop_ws/UMR/smpl/SMPLX_NEUTRAL.pkl
```

### 5.3 示例数据(新机器上 `recordings/` 与 `logs/` 都不存在)

两条路,任选:

**A. 从 4090 拷(最快,推荐)**

```bash
# 在 4090 上
rsync -avP ~/a3_teleop_ws/recordings/ <USER>@<ORIN_IP>:~/a3_teleop_ws/recordings/
rsync -avP ~/a3_teleop_ws/logs/a3_validation/ <USER>@<ORIN_IP>:~/a3_teleop_ws/logs/a3_validation/
```

**B. 在 Orin 上现场生成**(没有 4090 可用时;需要 SMPL-X 模型已就位)

```bash
cd ~/a3_teleop_ws/a3_teleop_bridge && source scripts/env_orin.sh

# ① 合成 SMPL-X 验收动作(9 条:站立/抬手/屈膝/转体/抬脚/慢步)
.venv_bridge/bin/python tools/make_smplx_validation_motions.py \
    --out ~/a3_teleop_ws/data/smplx_validation --duration 5

# ② 造一份 PICO 录制(PICO → UMR 链路用它做回归)
.venv_bridge/bin/python tools/make_synthetic_pico_recording.py \
    --clip ~/a3_teleop_ws/data/smplx_validation/twist_torso_left.npz \
    --out ~/a3_teleop_ws/recordings/m5_twist_torso_left

# ③ 录制 → SMPL-X 源(适配器)
.venv_bridge/bin/python tools/convert_pico_recording.py \
    --recording ~/a3_teleop_ws/recordings/m5_twist_torso_left \
    --out ~/a3_teleop_ws/data/pico_smplx/twist_torso_left

# ④ 离线 UMR → A3 结果(需要 .venv_umr)
.venv_bridge/bin/python tools/run_umr_a3_batch.py \
    --data-dir ~/a3_teleop_ws/data/pico_smplx/twist_torso_left \
    --out-dir ~/a3_teleop_ws/UMR/output/a3_pico --force

# ⑤ A3 结果 → flat CSV(SONIC 的 motion 输入)
.venv_bridge/bin/python tools/run_a3_validation_suite.py \
    --data-dir ~/a3_teleop_ws/UMR/output/a3_pico \
    --out-dir ~/a3_teleop_ws/logs/a3_validation --only m5_twist_torso_left --skip-mujoco
```

> `--skip-mujoco` 只导 CSV 不跑仿真(Orin 上更省事);要跑仿真去掉它即可。
> ③④⑤ 也可换成一次 `tools/run_m5_pico_chain.py --clips twist_torso_left`(全流程串起来)。
> CSV 落点固定为 `<out-dir>/<clip>/<clip>.csv`(上面就是 `logs/a3_validation/m5_twist_torso_left/m5_twist_torso_left.csv`)。

**本手册后面几条命令用到的两个 CSV**:

```bash
# ① 站立(仿真时当策略的状态输入 / 兜底 motion)
ls ~/a3_teleop_ws/logs/a3_validation/stand/stand.csv  || .venv_bridge/bin/python tools/run_a3_validation_suite.py \
        --data-dir ~/a3_teleop_ws/UMR/output/a3_validation \
        --out-dir ~/a3_teleop_ws/logs/a3_validation --only stand --skip-mujoco

# ② 耐久循环(把 9 条验收动作拼接 + 20 帧交叉淡化,长跑/仿真都用它)
.venv_bridge/bin/python tools/make_endurance_clip.py \
    --clips-dir ~/a3_teleop_ws/logs/a3_validation \
    --out ~/a3_teleop_ws/logs/a3_validation/endurance_loop.csv --repeats 8
```

> 本节 ①→⑤ 已在开发机上完整跑通验证(60 帧片段:UMR cost mean 0.0063,CSV 导出 PASS)。

### 5.4 自检(必须全绿再往下)

```bash
cd ~/a3_teleop_ws/a3_teleop_bridge && source scripts/env_orin.sh
bash scripts/check_orin_ready.sh                 # 16 项
.venv_bridge/bin/python -m pytest tests integration -q     # 167 passed, 1 skipped
bash tools/run_cpp_teleop_command_test.sh        # 24 项,纯 C++17
bash tools/run_cpp_channel_message_test.sh       # 19 项,需要 protoc + libprotobuf
```

---

## 6. 仿真一:PICO 遥操在 MuJoCo 里(Orin 版,见 `SIM_TELEOP.md`)

### 6.1 不接头显先跑通(B 层回归,2 分钟)

```bash
cd ~/a3_teleop_ws/a3_teleop_bridge && source scripts/env_orin.sh

# 没有录制文件时先造一份
.venv_bridge/bin/python tools/make_synthetic_pico_recording.py \
    --clip ~/a3_teleop_ws/data/smplx_validation/twist_torso_left.npz \
    --out ~/a3_teleop_ws/recordings/m5_twist_torso_left

$PY_UMR tools/run_live_chain.py \
    --recording ~/a3_teleop_ws/recordings/m5_twist_torso_left \
    --csv ~/a3_teleop_ws/logs/a3_validation/endurance_loop.csv --csv-fps 30 \
    --policy-steps 300 --duration 120 --port 5560 \
    --out-dir ~/a3_teleop_ws/logs/orin_sim/replay
```

### 6.2 真 PICO(XRoboToolkit PC Service 必须在**同一台 Orin**上)

```bash
# 终端 1:PICO → ZMQ(默认 PAUSED,必须 --start_unpaused 或按手柄 A 键)
cd ~/a3_teleop_ws/sonic_for_a3
.venv_pico_minimal/bin/python gear_sonic/scripts/pico_pose_zmq_minimal.py \
    --port 5556 --target_fps 50 --start_unpaused

# 终端 2+3:参考生成 + MuJoCo(一条命令)
cd ~/a3_teleop_ws/a3_teleop_bridge && source scripts/env_orin.sh
$PY_UMR tools/run_live_chain.py --pico \
    --csv ~/a3_teleop_ws/logs/a3_validation/stand/stand.csv --csv-fps 30 \
    --policy-steps 3000 --duration 150 --port 5560 \
    --out-dir ~/a3_teleop_ws/logs/orin_sim/pico_$(date +%H%M%S)
```

判据:终端 1 `sent` 递增 → T2 状态机进 `TRACKING`、`rejected=0` → T3 `fall=false`、root z ≈1.07 m。
排查见 `SIM_TELEOP.md` §5。

**Orin 上算力不够时**(仿真与参考生成抢 CPU):把 MuJoCo 挪到 4090,用同一条流:

```bash
# 4090(与 Orin 同网)
cd ~/a3_teleop_ws/a3_teleop_bridge && source scripts/env_orin.sh
bash scripts/run_mujoco_consumer.sh --endpoint tcp://<ORIN_IP>:5560 \
     --motion ~/a3_teleop_ws/logs/a3_validation/stand/stand.csv --steps 3000
```

---

## 7. 仿真二:官方 AimSim + 运控(可选但强烈建议)

证明你和 A3 官方接口的认知一致,且不需要真机:

```bash
python3.10 -m pip install <AimDK>/example/AimSim/aimsim-3.1.3-py3-none-any.whl
aimsim mujoco start                                   # 另开一个终端;键入 “l” 重置
./example/AimSim/mc/script/motion_control/start_motion_control.sh
./example/mc/S_SetAction.py                           # get_up → motion
./example/mc/walk.py                                  # 走起来
```

换机型 / `ros_domain_id` / 通信后端(iceoryx 或 ros2):

```bash
aimsim mujoco init-config --user-config-path ~/aimsim_cfg
aimsim mujoco start --user-config-path ~/aimsim_cfg
```

---

## 8. 参考推理:Orin 上真正要跑的那部分(路线 B)

### 8.1 起参考(不接机器人)

```bash
cd ~/a3_teleop_ws/a3_teleop_bridge && source scripts/env_orin.sh
bash scripts/run_orin_live.sh --duration 600 \
     --endpoint tcp://0.0.0.0:5560 \
     --save-calibration ~/a3_teleop_ws/logs/orin_live/calibration.json
# 判据:首个窗口 ~15 s 后出现;DISCONNECTED → CALIBRATION →(载入标定后)TRACKING
```

### 8.2 性能判据(Orin 与 4090 的唯一硬指标)

```bash
python3 - <<'PY'
import json; s=json.load(open('/tmp/orin_umr_stats.json'))   # run_orin_live.sh --stats 产物
lat=s['solver_latency_ms']; print('p50', round(lat['p50'],1), 'ms →', round(1000/lat['p50'],1), 'Hz')
PY
```

| p50 | 判定 | 行动 |
| --- | --- | --- |
| ≤ 50 ms(≥20 Hz) | 达标 | Orin 全量跑 |
| 50–100 ms | 可用 | 继续;靠 latest-only + SONIC 有界插值 |
| > 100 ms | 不达标 | **兜底**:UMR 留 4090/5060,只把参考流发过来(见 §9.3) |

### 8.3 策略推理放在哪(路线 B 的答案)

```text
正式:policy 在 A3 MDU/RK3588 上跑(RKNN),Orin 只发参考 —— 见 §9
自检:可以在 Orin 上单独跑一次 A3-fast ONNX,确认模型与输入输出形状
      (obs_dict[1,1570] → action[1,29]);这只需要 onnxruntime,不需要 CUDA
```

---

## 9. 连真机(路线 B 的 Orin 侧)

### 9.1 网络

```bash
ip -4 addr show | grep inet                      # 记下 Orin 的 IP
ping -c 3 <HDU_IP>                               # 通到机器人侧(官方示例 10.42.10.12)
# 参考流 bind 0.0.0.0:5560(env_orin.sh 默认),A3 侧连 <ORIN_IP>:5560
```

### 9.2 起参考并让 A3 消费

```bash
cd ~/a3_teleop_ws/a3_teleop_bridge && source scripts/env_orin.sh
bash scripts/run_orin_live.sh --duration 1800 --endpoint tcp://0.0.0.0:5560
```

A3 侧(MDU)流程见 `A3_ONBOARD.md`:部署 rockchip 包 → §6.1 服务配置(只起 agent)→
receive-only probe → 起适配节点(`/ta/whole_body_command`)→ 悬吊 10 级动作。

### 9.3 兜底:UMR 不在 Orin 上跑

```bash
# 4090/5060 上(与 Orin、A3 同网)
bash scripts/run_orin_live.sh --endpoint tcp://0.0.0.0:5560 --duration 3600
# Orin 只做 PICO 转发 / 或什么都不做
# A3 侧 --reference-endpoint 指向 4090 的 IP:5560
```

改的就是 `configs/network.yaml` 的 `bind_host/connect_host` 一行,协议不变。

---

## 10. 长跑与故障演练(上机前必须做一遍)

```bash
# 30 分钟连续参考(不接机器人)
bash scripts/run_orin_live.sh --duration 1800 --stats ~/a3_teleop_ws/logs/orin_live/stats30m.json
# 观察:frames_published 线性增长;state_history 不出现 SAFE_STOP;solver p95 不恶化;内存稳定

# 故障注入(测试已覆盖 13 项,现场再手演)
.venv_bridge/bin/python -m pytest tests/test_fault_injection.py tests/test_online_pipeline.py -q
# 手动:拔网线 / kill PICO sender → HOLD/SAFE_STOP;发 NaN → reject
```

---

## 11. 常见问题(Orin 专属)

| 现象 | 处理 |
| --- | --- |
| `check_orin_ready.sh` 报 SMPL-X 缺失 | §5.2 手工拷贝(许可证资产,仓库里没有) |
| `${SONIC_A3_ROOT}` 字面量出现在报错路径里 | 没 `source scripts/env_orin.sh` |
| `torch` 装了但 policy 仿真起不来 | 用的是 CPU wheel;仿真需要 Jetson 对应 CUDA wheel(参考链路不受影响) |
| 端口 5556 收不到帧 | sender 仍 PAUSED(`--start_unpaused`/A 键);PC Service 不在同一台机器 |
| 仿真与参考抢 CPU 导致 solver p95 飙高 | MuJoCo 挪到 4090(`run_mujoco_consumer.sh`),或用 §9.3 兜底 |
| `download_from_hf.py` 拉不动 | 在 4090 下好后 rsync `checkpoints/` 与 `gear_sonic_deploy/assets/` |
| MuJoCo 在 Orin 上报渲染/EGL 错误 | 用 `--batch-once` 无窗口模式(本仓库脚本默认如此),不要设 `MUJOCO_GL=egl` |
| `protoc` 缺失(channel message 测试) | `sudo apt-get install -y protobuf-compiler libprotobuf-dev` |

---

## 12. 一页速查

```bash
# ① clone
cd ~/a3_teleop_ws
git clone -b feat/a3-streaming-reference git@github.com:Restar7/sonic_for_a3.git
git clone                               git@github.com:Restar7/a3_teleop_bridge.git
git clone -b feat/a3-online-retarget    git@github.com:Restar7/UMR.git

# ② 环境
cd a3_teleop_bridge && source scripts/env_orin.sh
bash scripts/orin_preflight.sh && bash scripts/orin_bootstrap.sh --with-umr
cd ../sonic_for_a3 && bash install_scripts/install_pico_minimal.sh

# ③ 模型
.venv_sim/bin/python download_from_hf.py --component pt onnx rknn      # A3 模型
rsync -avP <保存处>/SMPLX_NEUTRAL.pkl <USER>@<ORIN_IP>:~/a3_teleop_ws/UMR/smpl/   # SMPL-X

# ④ 自检
cd ../a3_teleop_bridge && bash scripts/check_orin_ready.sh             # 16/16
.venv_bridge/bin/python -m pytest tests integration -q

# ⑤ 仿真(PICO 遥操)
cd ../sonic_for_a3 && .venv_pico_minimal/bin/python \
    gear_sonic/scripts/pico_pose_zmq_minimal.py --port 5556 --start_unpaused
cd ../a3_teleop_bridge && $PY_UMR tools/run_live_chain.py --pico \
    --csv ~/a3_teleop_ws/logs/a3_validation/stand/stand.csv --policy-steps 3000 --duration 150

# ⑥ 上机
bash scripts/run_orin_live.sh --duration 1800 --endpoint tcp://0.0.0.0:5560
```

相关文档:`DEPLOY_TARGET_DECISION.md`(为什么是 Orin)· `SIM_TELEOP.md`(仿真细节)·
`A3_ONBOARD.md`(机载与适配节点)· `GO_LIVE_CHECKLIST.md`(单页清单)·
`A3_OFFICIAL_INTERFACE.md`(官方接口对照)· `DEPLOY_ORIN.md`(部署原理与排查)
