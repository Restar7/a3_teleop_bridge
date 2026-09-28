# 5060_FULL_RUNBOOK.md — ThinkBook + RTX 5060 从零到推理的完整手册

> **这台机器的实际情况**(按你贴的终端):
> ```text
> 用户/主机 : wusichen@wusichen-ThinkBook-16-G7-IAH
> home      : /home/wusichen
> 已有      : miniconda3(conda base)、GR00T-WholeBodyControl、cuda_12.8.0_570 安装包
> 目标      : A3 全身遥操(路线 B)的部署机 + 仿真机
> ```
> 本文所有命令**从零开始、逐条可复制**。工作区统一放在 `/home/wusichen/a3_teleop_ws`。
>
> 与 Orin 版(`ORIN_FULL_RUNBOOK.md`)的关系:**步骤结构相同**,差异集中在
> §4(torch/CUDA,5060 是 Blackwell)和 §3(conda 与 venv 的关系)。文末 §16 有差异表。

---

## 速查:两条命令

装好之后(§1–§5 走完),剩下就只有两件事,**各一条命令**:

```bash
cd /home/wusichen/a3_teleop_ws

# ⓪ 先同步三个仓库(改动分布在三个仓库,只 pull 一个会很难查;详见 §17.0.1)
git -C a3_teleop_bridge pull --ff-only
git -C UMR             pull --ff-only
git -C sonic_for_a3    pull --ff-only

cd a3_teleop_bridge && source scripts/env_orin.sh

# ① PICO 头显 → 仿真遥操(MuJoCo 里的 A3 跟着你动)
bash scripts/run_pico_sim.sh --check     # 先自检:11 ok / 0 failed 再往下
bash scripts/run_pico_sim.sh

# ② PICO 头显 → 真机(发布 A3_REFERENCE_V1,A3 侧订阅)
bash scripts/run_robot_live.sh --check
bash scripts/run_robot_live.sh --a3-host <A3的IP> --duration 1800 --confirm-live
```

两条都先做前置自检(缺什么、怎么补都会直接打出来),都不需要你手动起第二个终端:

| | ① 仿真 | ② 真机 |
| --- | --- | --- |
| 一条命令 | `scripts/run_pico_sim.sh` | `scripts/run_robot_live.sh` |
| 只自检不起进程 | `scripts/run_pico_sim.sh --check` | `scripts/run_robot_live.sh --check` |
| 头显不在手边 | `scripts/run_pico_sim.sh --replay $A3WS/recordings/all/m5_twist_torso_left` | — |
| 内部链路 | PICO→online UMR→A3_REFERENCE_V1→A3-fast→**MuJoCo** | PICO→online UMR→A3_REFERENCE_V1→**A3 真机** |
| 前置 | §17.1 | §17.2 + **A3 侧部署包**(要拷到机器人上) |
| 详细章节 | §17.1 | §17.2 |

**§17 是这两条的完整说明(前置、判据、故障、A3 侧命令)。** 下面的 §0–§16 是它们的前置:
环境、模型、自检、以及不上头显也能跑的仿真 A/B 两层。

---

## 0. 先认清这台机器(3 条命令,决定后面装哪个 torch)

```bash
nvidia-smi --query-gpu=name,compute_cap,driver_version,memory.total --format=csv,noheader
python3 --version
df -h /home | tail -1
```

预期与判读:

| 输出 | 含义 | 后面的选择 |
| --- | --- | --- |
| `NVIDIA GeForce RTX 5060 Laptop GPU, 12.0, 5xx.xx` | **Blackwell(sm_120)** | torch 必须 **cu128 或更高**(老 cu121 wheel 没有 sm_120 内核) |
| driver `>= 570.x` | 支持 CUDA 12.8 | 用 `cu128` 索引(推荐,和你 home 里的 `cuda_12.8.0_570` 安装包一致) |
| driver `>= 580.x` | 支持 CUDA 13.0 | 也可以用 `cu130` |
| 磁盘剩余 `< 20 GB` | 空间紧张 | 三个仓库 + 模型 ≈ 3–5 GB,仿真 venv ≈ 3 GB,先腾空间 |

> **不需要**安装 CUDA Toolkit(pip 的 torch wheel 自带 CUDA 运行时)。
> 你 home 里的 `cuda_12.8.0_570.86.10_linux.run` 只有在你**要自己编译** CUDA 程序时才需要
> (比如以后编译 TensorRT/自定义算子)。装 torch 用不上它。

---

## 1. 工作区与系统准备

```bash
# 1.1 工作区
mkdir -p /home/wusichen/a3_teleop_ws
cd /home/wusichen/a3_teleop_ws

# 1.2 基础包
sudo apt-get update
sudo apt-get install -y git git-lfs python3-venv python3-pip rsync curl \
                        build-essential cmake protobuf-compiler libprotobuf-dev
git lfs install

# 1.3 时间同步(参考流新鲜度判据是 50 ms 量级,必须同步)
sudo timedatectl set-ntp true && timedatectl status | head -4

# 1.4 笔记本性能模式(否则 UMR 会莫名变慢)
#    插电 + 在“设置 → 电源”里选“性能”;或:
powerprofilesctl list 2>/dev/null | head -5 || true
powerprofilesctl set performance 2>/dev/null || true
```

---

## 2. 从 GitHub 克隆三个仓库

### 2.1 生成这台机器自己的 SSH 密钥(推荐)

```bash
ssh-keygen -t ed25519 -N "" -C "wusichen-thinkbook-5060" -f ~/.ssh/id_ed25519_github_a3
cat ~/.ssh/id_ed25519_github_a3.pub
```

把这行公钥加到 **GitHub → 头像 → Settings → SSH and GPG keys → New SSH key**。

```bash
mkdir -p ~/.ssh && chmod 700 ~/.ssh
grep -q "a3-teleop" ~/.ssh/config 2>/dev/null || cat >> ~/.ssh/config <<'CFG'
# a3-teleop deploy key (ThinkBook 5060)
Host github.com
    User git
    IdentityFile ~/.ssh/id_ed25519_github_a3
    IdentitiesOnly yes
CFG
chmod 600 ~/.ssh/config
ssh -T git@github.com      # 期望:Hi <账号>! You've successfully authenticated
```

### 2.2 克隆(注意分支)

```bash
cd /home/wusichen/a3_teleop_ws

git clone -b feat/a3-streaming-reference git@github.com:Restar7/sonic_for_a3.git
git clone                               git@github.com:Restar7/a3_teleop_bridge.git
git clone -b feat/a3-online-retarget    git@github.com:Restar7/UMR.git

git -C sonic_for_a3     log --oneline -1
git -C a3_teleop_bridge log --oneline -1
git -C UMR              log --oneline -1
```

没有 SSH 时(HTTPS 兜底):

```bash
git clone -b feat/a3-streaming-reference https://github.com/Restar7/sonic_for_a3.git
git clone                               https://github.com/Restar7/a3_teleop_bridge.git
git clone -b feat/a3-online-retarget    https://github.com/Restar7/UMR.git
```

---

## 3. 环境总览:conda 还是 venv?

你有 conda(`(base)`),但**本工程的环境变量脚本指向 venv**:

```text
$A3WS/a3_teleop_bridge/.venv_bridge   ← bridge 全链路(numpy/scipy/pyzmq/yaml/msgpack)
$A3WS/UMR/.venv_umr                   ← online UMR(torch/trimesh/clarabel/smplx)
$A3WS/sonic_for_a3/.venv_sim          ← MuJoCo 仿真 + A3-fast policy(torch/mujoco)
$A3WS/sonic_for_a3/.venv_pico_minimal ← PICO 发送端(官方脚本创建)
```

**建议:就用 venv**(`scripts/env_orin.sh`、`check_orin_ready.sh` 等全部按这些路径找解释器)。
conda 留在 base 不用即可;如果你更习惯 conda,把 `scripts/env_orin.sh` 里的 `PY_BRIDGE`/`PY_UMR`
改成 conda 环境里的 python 也能跑 —— 但**别混着用**(同一份代码用两个解释器跑会踩依赖坑)。

Python 版本:桥接层 `requires-python >= 3.10`,开发机上 3.10 与 3.12 都跑过;
`install_pico_minimal.sh` 默认找 `python3.10`,没有的话用 `PYTHON_BIN=python3` 覆盖。

```bash
cd /home/wusichen/a3_teleop_ws/a3_teleop_bridge
source scripts/env_orin.sh          # 打印四个路径,确认都对
echo 'source /home/wusichen/a3_teleop_ws/a3_teleop_bridge/scripts/env_orin.sh >/dev/null' >> ~/.bashrc
```

---

## 4. 建三个环境 + torch(Blackwell 关键一步)

### 4.1 bridge 与 UMR

```bash
cd /home/wusichen/a3_teleop_ws/a3_teleop_bridge
bash scripts/orin_preflight.sh         # 系统/依赖/zmq 自检 → orin_system_info/
bash scripts/orin_bootstrap.sh --with-umr
```

> 脚本默认的 torch 索引是 `cu121`,**在 5060 上会打印 Blackwell 警告**。参考链路本身跑 CPU,
> 所以 UMR 那个环境用 CPU/任意 torch 都能工作(我们实测 `CUDA_VISIBLE_DEVICES=""` 下正常)。
> 想让 UMR 也用上 GPU(仅离线批量才需要),重跑一次:
> ```bash
> TORCH_INDEX=https://download.pytorch.org/whl/cu128 bash scripts/orin_bootstrap.sh --with-umr
> ```

### 4.2 仿真环境(`.venv_sim`)—— **这里必须 cu128+**

```bash
cd /home/wusichen/a3_teleop_ws/sonic_for_a3
python3 -m venv .venv_sim
.venv_sim/bin/pip install --upgrade pip wheel
.venv_sim/bin/pip install numpy scipy mujoco pyyaml msgpack pyzmq

# Blackwell(sm_120):cu128(驱动 ≥570)或 cu130(驱动 ≥580)
.venv_sim/bin/pip install torch --index-url https://download.pytorch.org/whl/cu128
# 若上面报找不到 wheel,换:--index-url https://download.pytorch.org/whl/cu130

# 验证:两个都必须 True / 12.0
.venv_sim/bin/python -c "import torch;print(torch.__version__, torch.version.cuda, torch.cuda.is_available());print(torch.cuda.get_device_capability())"
.venv_sim/bin/python check_environment.py
```

**常见坑**:`torch.cuda.is_available()` 为 `False` 或报
`no kernel image is available for execution on the device` → wheel 不是 cu128+ 或驱动太旧。
这不影响 §9 的参考链路(CPU),只影响 §7 的 MuJoCo policy 仿真。

### 4.3 PICO 发送端环境(官方脚本)

```bash
cd /home/wusichen/a3_teleop_ws/sonic_for_a3

# 有 python3.10 就直接跑;没有就用系统 python3(脚本会自己识别 x86_64 的 SDK 库路径)
bash install_scripts/install_pico_minimal.sh
# 或:PYTHON_BIN=python3 bash install_scripts/install_pico_minimal.sh

# 验证 SDK
.venv_pico_minimal/bin/python -c "import xrobotoolkit_sdk as xrt; print('sdk ok')"
```

> 你 home 里的 `GR00T-WholeBodyControl` 是上游全量栈(PICO 可视化用),本流程**不必需**;
> 想要 `--vis_vr3pt --vis_smpl` 可视化时再按 `docs/pico_setup.md` §4.2 用它。

---

## 5. 模型与许可证资产

### 5.1 A3 checkpoint / ONNX / RKNN(官方 HuggingFace)

```bash
cd /home/wusichen/a3_teleop_ws/sonic_for_a3
.venv_sim/bin/python download_from_hf.py --component pt onnx rknn
# 交叉编译 rockchip 包时才需要:--component sysroot(以及 aarch64 ONNX Runtime 压缩包)
find checkpoints/035_step200000 -maxdepth 1 -type f | sort
find gear_sonic_deploy/assets -name "*.rknn" | head
```

### 5.2 SMPL-X 人体模型(许可证资产,任何仓库里都没有)

```bash
mkdir -p /home/wusichen/a3_teleop_ws/UMR/smpl
# 从你已有的地方拷:
#   cp /path/to/SMPLX_NEUTRAL.pkl /home/wusichen/a3_teleop_ws/UMR/smpl/
# 或从另一台机器 rsync:
#   rsync -avP <user>@<host>:~/a3_teleop_ws/UMR/smpl/SMPLX_NEUTRAL.pkl \
#         /home/wusichen/a3_teleop_ws/UMR/smpl/
sha256sum /home/wusichen/a3_teleop_ws/UMR/smpl/SMPLX_NEUTRAL.pkl
```

> 没有这个文件时 `check_orin_ready.sh` 会明确报 **SMPL-X model (licensed, copy by hand)** 失败,
> 这是唯一一个"必须人工提供"的资产。

### 5.3 示例数据(新克隆里没有 `recordings/` 和 `logs/`)

```bash
cd /home/wusichen/a3_teleop_ws/a3_teleop_bridge && source scripts/env_orin.sh

# ① 合成 9 条 SMPL-X 验收动作
.venv_bridge/bin/python tools/make_smplx_validation_motions.py \
    --out /home/wusichen/a3_teleop_ws/data/smplx_validation --duration 5

# ② 造 PICO 录制(PICO→UMR 链路的回归输入)
.venv_bridge/bin/python tools/make_synthetic_pico_recording.py \
    --clip /home/wusichen/a3_teleop_ws/data/smplx_validation/twist_torso_left.npz \
    --out /home/wusichen/a3_teleop_ws/recordings/m5_twist_torso_left

# ③ 录制 → SMPL-X 源
.venv_bridge/bin/python tools/convert_pico_recording.py \
    --recording /home/wusichen/a3_teleop_ws/recordings/m5_twist_torso_left \
    --out /home/wusichen/a3_teleop_ws/data/pico_smplx/twist_torso_left

# ④ 离线 UMR → A3 结果
.venv_bridge/bin/python tools/run_umr_a3_batch.py \
    --data-dir /home/wusichen/a3_teleop_ws/data/pico_smplx/twist_torso_left \
    --out-dir /home/wusichen/a3_teleop_ws/UMR/output/a3_pico --force

# ⑤ A3 结果 → flat CSV(后面仿真/长跑要用的 motion)
.venv_bridge/bin/python tools/run_a3_validation_suite.py \
    --data-dir /home/wusichen/a3_teleop_ws/UMR/output/a3_pico \
    --out-dir /home/wusichen/a3_teleop_ws/logs/a3_validation --only m5_twist_torso_left --skip-mujoco

# ⑥ 耐久循环 CSV(长跑/仿真都用它)
.venv_bridge/bin/python tools/make_endurance_clip.py \
    --clips-dir /home/wusichen/a3_teleop_ws/logs/a3_validation \
    --out /home/wusichen/a3_teleop_ws/logs/a3_validation/endurance_loop.csv --repeats 8
```

> 这一串(①→⑤)已在开发机上完整跑通:UMR cost mean 0.0063、CSV 导出 PASS。

### 5.4 自检(全绿才往下)

```bash
cd /home/wusichen/a3_teleop_ws/a3_teleop_bridge && source scripts/env_orin.sh
bash scripts/check_orin_ready.sh                            # 16 ok, 0 failed
.venv_bridge/bin/python -m pytest tests integration -q      # 167 passed, 1 skipped
bash tools/run_cpp_teleop_command_test.sh                   # 24 项(纯 C++17)
bash tools/run_cpp_channel_message_test.sh                  # 19 项(protoc + libprotobuf)
```

---

## 6. 仿真 A:不接头显,先把链路跑起来(2 分钟)

```bash
cd /home/wusichen/a3_teleop_ws/a3_teleop_bridge && source scripts/env_orin.sh

# 只跑参考侧(不启 MuJoCo)
$PY_UMR -m a3_teleop_bridge.apps.retarget_live \
    --source recording --backend umr-online \
    --recording /home/wusichen/a3_teleop_ws/recordings/m5_twist_torso_left \
    --duration 8 --no-publish --playback-hz 30 --stats /tmp/online.json
# 判据:solved>0、solver ≈40 ms 量级、rejected=0
```

---

## 7. 仿真 B:PICO 遥操在 MuJoCo 里(完整链路)

### 7.1 启动 XRoboToolkit PC Service(与发送端**同一台机器**)

```bash
# 官方仓库:https://github.com/XR-Robotics/XRoboToolkit-PC-Service
# 用官方 Linux x86_64 安装包,只起服务进程:
runService.sh            # 或按官方的启动方式
# 启动后:头显连同一 Wi-Fi,填入本机 IP;确认服务界面/日志里设备在线
```

### 7.2 三个终端

```bash
# ---------- 终端 1:PICO → ZMQ ----------
cd /home/wusichen/a3_teleop_ws/sonic_for_a3
.venv_pico_minimal/bin/python gear_sonic/scripts/pico_pose_zmq_minimal.py \
    --port 5556 --target_fps 50 --start_unpaused
# 必须看到: "Stream state: RUNNING" 且 sent 递增
# 默认是 PAUSED:不加 --start_unpaused 就按手柄 A 键切换

# ---------- 终端 2+3:参考生成 + MuJoCo(一条命令拉起) ----------
cd /home/wusichen/a3_teleop_ws/a3_teleop_bridge && source scripts/env_orin.sh
$PY_UMR tools/run_live_chain.py --pico \
    --csv /home/wusichen/a3_teleop_ws/logs/a3_validation/m5_twist_torso_left/m5_twist_torso_left.csv \
    --csv-fps 30 --policy-steps 3000 --duration 150 --port 5560 \
    --out-dir /home/wusichen/a3_teleop_ws/logs/5060_sim/pico_$(date +%H%M%S)
```

判据(逐条看):

```text
终端1 sent 递增                     → PICO/SDK 正常
T2 状态机 DISCONNECTED→CALIBRATION→TRACKING   → 标定与数据流正常
T2 rejected=0                        → 没有 NaN/越限
T2 solver p50 30–60 ms               → 5060 上应该比 Orin 更快
T3 fall=false,root z ≈1.07 m         → 策略跟得上
视觉:抬右臂→右臂抬;转体→腰转          → 关节顺序/镜像正确
```

停止:Ctrl-C 即可;参考停止后 MuJoCo 侧 50 ms hold、250 ms safe stop,机器人保持站立(期望行为)。

### 7.3 第一批动作(严格按序)

```text
站立 → 轻微摆臂 → 单臂抬起 → 双臂抬起 → 慢速屈膝 → 重心转移 → 轻微抬脚 → 慢速迈一步
禁止:跳跃 / 快跑 / 深蹲到底 / 跪 / 躺 / 大幅单腿站立 / 快速 180° 旋转
```

### 7.4 录下来(以后不用头显也能复现)

```bash
cd /home/wusichen/a3_teleop_ws/a3_teleop_bridge && source scripts/env_orin.sh
.venv_bridge/bin/python -m a3_teleop_bridge.apps.record_pico --duration 30 \
    --out /home/wusichen/a3_teleop_ws/recordings/$(date +%Y%m%d_%H%M%S)_5060
.venv_bridge/bin/python tools/record_reference.py --duration 30 \
    --out /home/wusichen/a3_teleop_ws/examples/reference_recording/5060_session
# 重播(不需要 PICO/UMR):
.venv_bridge/bin/python -m a3_teleop_bridge.apps.replay_reference \
    --windows /home/wusichen/a3_teleop_ws/examples/reference_recording/5060_session \
    --publish-hz 50 --loop
```

---

## 8. 仿真 C(可选):官方 AimSim + 运控

```bash
python3 -m pip install <AimDK>/example/AimSim/aimsim-3.1.3-py3-none-any.whl
aimsim mujoco start                                     # 另开终端;键入 “l” 重置
./example/AimSim/mc/script/motion_control/start_motion_control.sh
./example/mc/S_SetAction.py && ./example/mc/walk.py
```

---

## 9. 参考推理:这台机器上真正要跑的那部分(路线 B)

```bash
cd /home/wusichen/a3_teleop_ws/a3_teleop_bridge && source scripts/env_orin.sh
mkdir -p /home/wusichen/a3_teleop_ws/logs/orin_live
bash scripts/run_orin_live.sh --duration 600 \
     --endpoint tcp://0.0.0.0:5560 \
     --save-calibration /home/wusichen/a3_teleop_ws/logs/orin_live/calibration.json \
     --stats /home/wusichen/a3_teleop_ws/logs/orin_live/pipeline_stats.json
```

性能判据:

```bash
python3 - <<'PY'
import json; s=json.load(open('/home/wusichen/a3_teleop_ws/logs/orin_live/pipeline_stats.json'))
lat=s['solver_latency_ms']; print('p50', round(lat['p50'],1),'ms →', round(1000/lat['p50'],1),'Hz')
PY
```

| p50 | 判定 | 行动 |
| --- | --- | --- |
| ≤ 50 ms(≥20 Hz) | 达标 | 就用这台机器 |
| 50–100 ms | 可用 | 继续;靠 latest-only + SONIC 有界插值 |
| > 100 ms | 不达标 | 插电+性能模式;仍不达标则把 UMR 放到 4090(见 §12.3) |

**策略推理在哪**:正式跑在 A3 的 MDU/RK3588(RKNN)上,本机只发参考。想在 5060 上自检模型:

```bash
cd /home/wusichen/a3_teleop_ws/sonic_for_a3
python3 -m venv .venv_ort && .venv_ort/bin/pip install onnxruntime numpy
.venv_ort/bin/python - <<'PY'
import onnxruntime as ort, numpy as np, glob
p = sorted(glob.glob('gear_sonic_deploy/assets/**/*a3_fast*.onnx', recursive=True))
print('onnx:', p[:2])
s = ort.InferenceSession(p[0], providers=['CPUExecutionProvider'])
print('inputs :', [(i.name, i.shape) for i in s.get_inputs()])
print('outputs:', [(o.name, o.shape) for o in s.get_outputs()])
PY
```

---

## 10. 网络:5060 ↔ A3

```bash
ip -4 addr show | grep inet          # 记下本机 IP(有线口优先)
ping -c 3 <HDU_IP>                   # 通到机器人侧(官方示例 10.42.10.12)
```

物理连接两种(官方 §5.2):**机器人头部 Type-C ↔ 本机**,或**以太网**(经 HDU)。
参考流端口 **5560/tcp**(本机发布,A3 侧连本机 IP)。ROS2 侧(若走官方话题路线)还要:

```bash
export ROS_DOMAIN_ID=232
export FASTRTPS_DEFAULT_PROFILES_FILE=/path/to/ros_dds_configuration.xml   # 从 HDU 拷
ros2 topic list
```

---

## 11. 上真机(路线 B)

```bash
# 5060 侧:持续发参考(30 分钟起)
cd /home/wusichen/a3_teleop_ws/a3_teleop_bridge && source scripts/env_orin.sh
bash scripts/run_orin_live.sh --duration 1800 --endpoint tcp://0.0.0.0:5560
```

A3 侧按 `A3_ONBOARD.md`:交叉编译 rockchip 包(在 5060/4090 上做,需要 Docker + ROS 2 + aarch64 ONNX Runtime)
→ 传输到 MDU → 官方 §6.1 服务配置(只起 agent)→ receive-only probe →
适配节点发布 `/ta/whole_body_command` → 悬吊 10 级动作。

上机前必须完成:`GO_LIVE_CHECKLIST.md` 的 A→G。
现场必须:安全吊带/防坠、物理急停、安全员、清场、**没过 MuJoCo + AimSim 不上真机**。

---

## 12. 长跑、故障演练与兜底

### 12.1 30 分钟连续(不接机器人)

```bash
bash scripts/run_orin_live.sh --duration 1800 \
     --stats /home/wusichen/a3_teleop_ws/logs/orin_live/stats30m.json
# 观察:frames_published 线性增长;不出现 SAFE_STOP;solver p95 不恶化;内存稳定(htop)
```

### 12.2 故障注入

```bash
.venv_bridge/bin/python -m pytest tests/test_fault_injection.py tests/test_online_pipeline.py -q
# 手动:拔网线 → HOLD/SAFE_STOP;kill PICO sender → 同上;发 NaN → reject
```

### 12.3 兜底:UMR 不在这台机器上跑

```bash
# 在 4090(与 5060、A3 同网)上:
bash scripts/run_orin_live.sh --endpoint tcp://0.0.0.0:5560 --duration 3600
# A3 侧的 --reference-endpoint 改成 4090 的 IP:5560
```

只改 `configs/network.yaml` 的 `bind_host/connect_host`,协议与 A3 侧不变。

---

## 13. 笔记本专属注意事项

```text
[ ] 插电 + 性能模式:否则 CPU 降频,online UMR 的 p95 会明显变差
[ ] 关闭省电/挂起:遥操过程中睡眠 = 参考断流(虽然会安全 hold,但很烦)
     systemd-inhibit --what=idle:sleep bash scripts/run_orin_live.sh ...
[ ] Wi-Fi 只用于 PICO;机器人链路尽量用有线(延迟与抖动差一个量级)
[ ] PICO PC Service 与发送端必须同机(本机),头显连本机 Wi-Fi IP
[ ] 散热:长时间 30 分钟遥操时注意降频,必要时垫高/加散热垫
```

---

## 14. 一页速查(从零到上机)

```bash
# ① 系统
sudo apt-get install -y git git-lfs python3-venv rsync build-essential protobuf-compiler libprotobuf-dev

# ② clone
mkdir -p ~/a3_teleop_ws && cd ~/a3_teleop_ws
git clone -b feat/a3-streaming-reference git@github.com:Restar7/sonic_for_a3.git
git clone                               git@github.com:Restar7/a3_teleop_bridge.git
git clone -b feat/a3-online-retarget    git@github.com:Restar7/UMR.git

# ③ 环境
cd a3_teleop_bridge && source scripts/env_orin.sh
bash scripts/orin_preflight.sh && bash scripts/orin_bootstrap.sh --with-umr
cd ../sonic_for_a3
python3 -m venv .venv_sim && .venv_sim/bin/pip install -U pip wheel
.venv_sim/bin/pip install numpy scipy mujoco pyyaml msgpack pyzmq
.venv_sim/bin/pip install torch --index-url https://download.pytorch.org/whl/cu128
bash install_scripts/install_pico_minimal.sh

# ④ 模型 + SMPL-X
.venv_sim/bin/python download_from_hf.py --component pt onnx rknn
#   cp <SMPLX_NEUTRAL.pkl> ~/a3_teleop_ws/UMR/smpl/

# ⑤ 自检
cd ../a3_teleop_bridge && bash scripts/check_orin_ready.sh      # 16/16
.venv_bridge/bin/python -m pytest tests integration -q

# ⑥ 仿真里的 PICO 遥操(一条命令,内部起发送端 + 参考 + MuJoCo)
cd ../a3_teleop_bridge && source scripts/env_orin.sh
bash scripts/run_pico_sim.sh
#   没头显也想验链路:bash scripts/run_pico_sim.sh --replay $A3WS/recordings/all/m5_twist_torso_left
#   只自检:          bash scripts/run_pico_sim.sh --check

# ⑦ 真机(一条命令;先 --check,--confirm-live 才会真的发布)
#   前置:A3 侧 rockchip 部署包 + aarch64 ONNX Runtime + 适配节点 -> 见 §17.2.2
bash scripts/run_robot_live.sh --check
bash scripts/run_robot_live.sh --a3-host <A3的IP> --duration 1800 --confirm-live
```

> **最后两步的完整说明在 §17**(前置、判据、A3 侧要拷贝的东西、安全闸)。

---

## 15. 故障排查(逐条对应现象)

| 现象 | 原因 / 处理 |
| --- | --- |
| `torch.cuda.is_available()` False | wheel 不是 cu128+ 或驱动 <570;`nvidia-smi` 看驱动版本 |
| `no kernel image ... on the device` | 经典 sm_120 不匹配 → 换 cu128/cu130 wheel |
| `check_orin_ready.sh` 报 SMPL-X 缺失 | §5.2 手工拷贝(许可证资产) |
| 报错路径里出现字面量 `${SONIC_A3_ROOT}` | 没 `source scripts/env_orin.sh` |
| 终端 1 没有 `sent` 递增 | 发送端仍 PAUSED → `--start_unpaused` 或按 A 键 |
| T2 `received 0` | 端口不是 5556;PC Service 不在本机;`ss -ltnp | grep 5556` |
| 链路报 `pipeline published nothing` | 参考流一个窗口都没发 → 回到 PICO 侧查 |
| 状态机停在 CALIBRATION | 没有标定文件:用 `--save-calibration` 生成后 `--calibration` 载入 |
| solver p50 突然 >100 ms | 没插电/没性能模式;后台跑了大任务;或 MuJoCo 与 UMR 抢 CPU(把 MuJoCo 挪到 4090) |
| 关节动但部位/方向不对 | 关节顺序或镜像问题:先用录制回放复现,再查 `generated/a3_contract.json` 与 `coordinate_frames.md` |
| `protoc` 缺失 | `sudo apt-get install -y protobuf-compiler libprotobuf-dev` |
| `run_pico_sim.sh` 报 `xrobotoolkit_sdk missing` | PICO SDK 没装。**先 `git lfs pull`**(`libPXREARobotSDK.so` 是 LFS 文件),再 `cd $A3WS/sonic_for_a3 && PYTHON_BIN=python3.12 PIP_INDEX_URL=https://pypi.tuna.tsinghua.edu.cn/simple bash install_scripts/install_pico_minimal.sh` |
| SDK 编译在**链接**阶段失败 `file format not recognized ... treating as linker script` | `libPXREARobotSDK.so` 还是 133 字节的 LFS 指针:`git lfs pull --include="external_dependencies/.../lib/*"` |
| 发送端起来了但日志没有 `Stream state: RUNNING` | 发送端默认 PAUSED:按手柄 **A** 键;或 PC Service 不在本机(它必须与发送端同机) |
| `run_pico_sim.sh` 报 sim/UMR 解释器缺模块 | 本机用 conda:先 `export PY_BRIDGE=$PY_UMR=$PY_SIM=<你的python>`,见 §17.0 |
| `run_robot_live.sh` 退出码 3 | 没给 `--confirm-live`(安全闸,防止误发动作到真机);自检通过后再加上 |
| 真机侧连不上 5560 | 本机防火墙/交换机隔离,或 A3 侧 endpoint 写错;本机 `ss -tnp | grep 5560` 看有没有连接 |
| `check_orin_ready.sh` 少了几项 `[ OK ]` | 解释器不存在时 3/5、4/5、5/5 三段会被跳过 —— 设 `PY_BRIDGE`/`PY_UMR` 指向真实解释器 |
| `download_from_hf.py` 拉不动 | 在能上网的机器下好后 rsync `checkpoints/` 与 `gear_sonic_deploy/assets/` |

---

## 16. 与 Orin 手册的差异(一张表)

| 项 | ThinkBook 5060(本文) | Orin(`ORIN_FULL_RUNBOOK.md`) |
| --- | --- | --- |
| 架构 | x86_64 | aarch64 |
| 仿真 torch | **必须 cu128+**(Blackwell) | Jetson wheel 或 CPU |
| PICO SDK 库路径 | `lib/`(脚本自动) | `lib/aarch64`(脚本自动) |
| 机器人连接 | Type-C 或以太网(线缆/无线) | 机载供电 + Type-C/以太网 |
| 算力 | 更强(UMR 更快、仿真可用 GPU) | 更弱但机载可靠 |
| 其余命令 | **完全相同** | 完全相同 |

## 17. 最后两步:各一条命令

> §0–§16 走完(`check_orin_ready.sh` 16 ok / 0 failed,`pytest tests integration` 全绿)之后,
> 剩下就是这两条。**两条都自带前置自检**,缺什么会直接告诉你补什么,不会静默失败。

### 17.0 环境:本机用 conda,已经不需要任何 export

`scripts/env_orin.sh` 的解释器解析顺序是:

```text
① 显式 PY_BRIDGE / PY_UMR / PY_SIM  ② runbook §3 建的 venv
③ conda 环境(默认名 a3_bridge,可用 A3_CONDA_ENV=<name> 改)  ④ 当前 python3
```

本机三个 venv 都不存在,但 conda `a3_bridge` 里 numpy/scipy/zmq/torch/mujoco/onnxruntime
已经齐了,所以 ③ 会自动生效 —— **直接 source 就能用,不用再 export 任何东西**:

```bash
cd /home/wusichen/a3_teleop_ws/a3_teleop_bridge
source scripts/env_orin.sh
# [env] PY_BRIDGE=/home/wusichen/miniconda3/envs/a3_bridge/bin/python
# [env] PY_UMR=...
# [env] PY_SIM=...
# [env] note: project venvs absent; using conda env 'a3_bridge'      ← 就是这行

bash scripts/check_orin_ready.sh      # 期望 16 ok, 0 failed(零配置)
```

> 只有 PICO 发送端(`.venv_pico_minimal`)是单独的,见 §17.1。
> 想换解释器还是可以显式 `export PY_UMR=<你的python>`,优先级最高。

---

### 17.0.1 开工前:同步三个仓库(每次改完代码都要做)

改动分布在**三个仓库**,只 pull 一个会得到"离线对了、现场还是老行为"这种最难查的状态。
逐条复制:

```bash
cd /home/wusichen/a3_teleop_ws

# ① bridge(验收工具、validator、runbook、live 链路)
git -C a3_teleop_bridge pull --ff-only

# ② UMR(膝姿态先验 solver.joint_map_cost;没有它 A3 的膝永远不弯)
git -C UMR pull --ff-only

# ③ sonic_for_a3(PICO 发送端:root_translation / 帧率)
git -C sonic_for_a3 pull --ff-only

# 确认三个都到位(应该都是最新 commit、没有 dirty)
git -C a3_teleop_bridge log --oneline -1
git -C UMR             log --oneline -1
git -C sonic_for_a3    log --oneline -1
```

> `dirty=N`(N>0)说明有本地改动,先 `git -C <repo> status` 看一眼再继续。
> UMR 里有 3 个 untracked(`data/`、`humanoid_retarget_defaults_a3_validation.json`、
> `smpl/version.txt`)是正常的,那是运行产物,不是你的改动。

**一条命令做完上面全部(含自检)**:

```bash
cd /home/wusichen/a3_teleop_ws/a3_teleop_bridge && source scripts/env_orin.sh \
  && for r in . ../UMR ../sonic_for_a3; do git -C "$r" pull --ff-only; done \
  && bash scripts/check_orin_ready.sh
```

看到 `[ready] 16 ok, 0 failed` 就可以往下走。

---

### 17.1 仿真里的 PICO 遥操 —— `run_pico_sim.sh`

```bash
cd /home/wusichen/a3_teleop_ws/a3_teleop_bridge && source scripts/env_orin.sh
bash scripts/run_pico_sim.sh                 # 直接开始遥操(MuJoCo 窗口默认打开)
bash scripts/run_pico_sim.sh --check         # 只自检,不起进程
bash scripts/run_pico_sim.sh --duration 300 --policy-steps 6000
bash scripts/run_pico_sim.sh --replay $A3WS/recordings/all/m5_twist_torso_left   # 没头显也能验链路
bash scripts/run_pico_sim.sh --no-viewer     # 无显示环境/CI:跑批处理模式
```

**MuJoCo 窗口(默认开)**

```text
Space        暂停/继续
. / →        暂停时单步前进一个 policy 帧
, / ←        回退一个参考帧并重置仿真
= / -        下一个 / 上一个动作片段
R            重置到当前片段第 0 帧
关掉窗口      结束本次运行
```

窗口里机器人是实时的,**半透明的"参考影子"**叠在上面(来自参考流),所以你能直接看出
策略跟没跟上。窗口模式与 `--batch-once` 批处理模式**写同样的 metrics**,
`fall` / `root z` / `RMSE(29)` 一样可比;需要无人值守时用 `--no-viewer`。

> 没有显示的环境(纯 ssh/CI)会自动回落到批处理:脚本检测 `DISPLAY`/`WAYLAND_DISPLAY`,
> 缺了就打印 `--viewer requested but no DISPLAY/WAYLAND_DISPLAY; running headless`。

这一条命令内部起了**两个**进程,并在退出时一起收掉:

```text
① PICO 发送端  sonic_for_a3/gear_sonic/scripts/pico_pose_zmq_minimal.py --port 5556 --start_unpaused
② 参考 + 仿真  tools/run_live_chain.py --pico --port 5560
     ├ retarget_live --source pico --backend umr-online --publish   (PICO→online UMR→A3_REFERENCE_V1)
     └ sim2sim_a3_mujoco  --reference-source stream --realtime      (A3-fast 50 Hz → MuJoCo)
```

**前置(脚本会逐条自检)**

```text
[ ] §1–§5 已完成:三仓库、模型/checkpoint、SMPL-X、check_orin_ready.sh 全绿
[ ] XRoboToolkit PC Service **跑在本机**(头显连它;端口默认 63901)
[ ] 头显与本机同一 Wi-Fi;头显开发者模式已开
[ ] PICO SDK 装好(本机已装;换机器/重装时):
      cd $A3WS/sonic_for_a3
      # ① 先确认 LFS 二进制真的拉下来了(见下方"坑 1")
      git lfs pull --include="external_dependencies/XRoboToolkit-PC-Service-Pybind_X86_and_ARM64/lib/*"
      # ② 默认找 python3.10;本机用 3.12。PyPI 不通时加国内镜像(实测清华/阿里/中科大可用):
      PYTHON_BIN=python3.12 PIP_INDEX_URL=https://pypi.tuna.tsinghua.edu.cn/simple \
        bash install_scripts/install_pico_minimal.sh
      .venv_pico_minimal/bin/python -c "import xrobotoolkit_sdk as xrt; print('sdk ok')"
```

> **坑 1(本机踩过,值得记住)**:`libPXREARobotSDK.so` 是 **git-lfs 文件**。
> 如果当初 clone 时没拉 LFS,这个文件只有 133 字节(一个文本指针),SDK 编译会在
> **链接阶段**失败,报 `file format not recognized ... treating as linker script` ——
> 看起来像编译器/环境问题,其实是**缺二进制**。一条命令修:
> ```bash
> cd $A3WS/sonic_for_a3
> git lfs pull --include="external_dependencies/XRoboToolkit-PC-Service-Pybind_X86_and_ARM64/lib/*"
> ls -l external_dependencies/XRoboToolkit-PC-Service-Pybind_X86_and_ARM64/lib/libPXREARobotSDK.so
> # 期望 ~23.7 MB 的 ELF;若是 133 字节就是指针,没拉下来
> ```
>
> **坑 2**:SDK 的 CMake 用 `find_package(pybind11)`。手动 `cmake` 编译时要给
> `-Dpybind11_DIR=$(python -m pybind11 --cmakedir)`;用官方 `install_pico_minimal.sh`
> 则不用管(pip 的构建隔离会处理)。

```text
[ ] 参考端口 5560 空闲、PICO 端口 5556 空闲(脚本会查)
```

#### 17.1.1 PICO 那一端要做什么(两个侧,别只做头显)

头显**不是**直接连本机的 UMR —— 中间必须有 **PC Service**,而 PC Service 跑在**本机**:

```text
PICO 头显 ──Wi-Fi──► PC Service(本机,监听 127.0.0.1:60061)
                          │ xrobotoolkit_sdk
                          ▼
                pico_pose_zmq_minimal.py ──ZMQ:5556──► bridge → UMR → A3 参考流
```

**A. 本机(PC Service 侧)** —— 发送端自己会尝试拉起它,但**只在官方标准安装路径**下:

```text
[ ] PC Service 装在本机,并且是 /opt/apps/roboticsservice/runService.sh
      └ 有这个路径:run_pico_sim.sh 起发送端时会自动把它拉起来,你不用管
      └ 装在别处:自己先起好,或者把它放到上面那个路径
[ ] 确认在监听:  ss -ltn | grep 60061
```

> **只看头显是起不来的。** SDK 连的是 **localhost:60061**,PC Service 不在本机,
> 发送端会一直停在 `waiting for body data...`(`xrobotoolkit_sdk` 装得再对也没用)。

**B. 头显侧(一次性 + 每次)**

```text
[ ] 头显开开发者模式,与本机**同一 Wi-Fi / 同一网段**
[ ] 头显里安装并打开 XRoboToolkit 应用,PC IP 填**本机**的 IP(脚本会打印,本机是 192.168.4.39)
[ ] 确认全身追踪可用(app 里能看到 body data)
[ ] 操作者:站直、双脚自然分开、双臂自然下垂,做一次标定姿势
[ ] 手柄 **A 键** = 开始/暂停发送(脚本已带 --start-unpaused,起来就是 RUNNING;A 用于暂停)
```

**C. 卡住时怎么一眼看出是哪一侧**(发送端日志 `pico_sender.log`)

| 日志停在 | 说明 | 去哪修 |
| --- | --- | --- |
| `robotics service script not found: /opt/apps/...` 然后一直 `waiting for body data...` | **PC Service 没起**(本机侧) | A。`ss -ltn \| grep 60061` 应该有人监听 |
| `initialize sdk,connect127.0.0.1:60061` 后一直 `waiting for body data...` | PC Service 起了,但**头显没连上 / 没开身体追踪 / 中途掉线** | B。见下方"头显掉线" |
| 出现 `Stream state: RUNNING` + `sent=` 递增 | **头显侧 OK** | 往下看 T2/T3 判据 |
| `Stream state: PAUSED` | 忘了 unpause | 按手柄 **A**,或确认脚本带了 `--start-unpaused` |
| **进了 MuJoCo 机器人立刻往后倒** | **不是关节对应问题**:发送端只发 root 相对的局部关节 + 朝向,**不发 root 世界平移**;修前 bridge 把它兜底成 `[0,0,0]`,于是参考的骨盆比机器人站的地方低 0.975 m,策略被命令"把骨盆放到地面"→ 塌下去 | **已修**(2026-09-28):缺 root 时重建为站立高度 `[0,0,0.975]`。症状特征:`fall=True` 在 ~0.8 s、`root_height≈0.25`、`roll/pitch≈180°`、状态机卡在 `CALIBRATION` |

`run_pico_sim.sh` 会替你走完 A 的检查,并在发送端卡住 15 s 后**直接把上面这张表打出来**。

**⚠️ 真发送端的 root 姿态约定和录制不一致(2026-09-28 第二次实测)**

头显发送端发的是**机载 deploy 运行时的"调整后 root 局部系"**:

```text
body_quat_w = (Y_TO_Z_UP ⊗ xr_root ⊗ R_y(180°)) ⊗ SMPL_BASE_ROT_CONJ     ← 发送端
root_quat   =  Y_TO_Z_UP ⊗ xr_root                                      ← 录制/离线/UMR 期望
```

而 `sequence_from_frame` 把 `body_quat_w` **直接当 SMPL-X 的 root 旋转**喂给 UMR。
多出来的两个因子把整个人转歪 → 参考 root 离机器人 1.24 m → 0.8 s 倒地,
而 **joint_l1 一直只有 0.22–0.35 rad(关节其实是好的)**。

症状辨识:站立不动也倒、`root_err` 单调涨到 ~1.24 m、`joint_l1` 正常。
现在 bridge 在 live 边界(`zmq_subscriber`)做精确还原(对任意姿态误差 1e-16,
站立时正好还原成录制里的 `[0.7071, 0.7071, 0, 0]`)。录制的 frame 里是**已转换过**的,
所以只作用于 live,`--replay` 不受影响。

**⚠️ live PICO 的 root 是重建出来的(2026-09-28 实测踩过)**

头显发送端的 payload 只有三个字段,全是对 root 相对的局部量:

```python
{"smpl_pose": ..., "smpl_joints": ..., "body_quat_w": ...}    # 没有 root_translation
```

修前 bridge 在没有 root 时兜底成 `[0,0,0]` → 参考的骨盆在地面上(录制文件里是 `z=0.975`)。
后果:机器人一进 MuJoCo 就往后倒(0.8 s 内 `fall=True`)。现在缺 root 时重建为站立高度,
可用 `A3_PICO_STANDING_PELVIS_M` 覆盖。遥操是**原地**的,所以常量站高既正确、
也和录制路径一致。

**⚠️ live PICO 需要标定才会进 TRACKING**

状态机只有在 `calibration is not None` 时才进 TRACKING。自动标定原本只覆盖
`trajectory`/`recording`,**`pico` 没有路径** → 真机永远停在 CALIBRATION 不会跟随。
现在 live 也接了:服务端会用**头显前 ~1.5 秒的帧**自动标定(操作者这段时间站直别动),
也可以用 `--save-calibration` 存下来、下次 `--calibration` 直接载入。

**⚠️ 源动作原本是"人把手臂平举着"(2026-09-28 修复)**

`make_smplx_validation_motions.py` 一直从 **SMPL-X rest 骨架**插值,而 rest 骨架的手臂是
**74–82°(接近水平)** 的。retarget 忠实地复现了它 → A3 的肩 roll 停在 **1.545 rad(=它的 T-pose)**,
而 A3 **自己的 keyframe 标称站姿是 +0.112(手臂自然下垂)**。也就是说整个验收集演练的是
"一个人把手臂端着",跟真人站在头显前的姿势完全不同。

现在生成器先算出一个**自然站姿**基准(实测搜索,手臂 9.5°/12.0° 下垂),所有 clip 从它出发:

```text
                     旧(手臂平举)      新(自然下垂)
m5_stand 肩 roll     +1.545            +0.241        (A3 keyframe = +0.112)
raise_left_arm       0.241→(同左)      0.241→1.502   (抬手仍然有效)
bend_knees 膝        0.000             0.237→0.805
```

重新生成后的全套验收 **9/9 PASS**,而且跟踪质量整体变好:stand RMSE 0.0568→**0.0397**、
raise_left_arm 0.0893→**0.0532**、raise_right_arm 0.1028→**0.0492**。

**⚠️ 走不动:发送端原来根本没发位移**

`pico_pose_zmq_minimal.py` 一直算 `positions = body_poses_np[:, :3]` 然后**丢掉**,
所以操作者的位移从来没离开过头显 —— 大步迈也只能原地踏步。现在发送端把它按
Z-up 转换 + **锚定到起始帧**(头显 tracking 空间原点无意义),并放在站立高度上发布:

```text
root_translation = [Δx, Δy, 0.975 + Δz]     # 相对你按 A / 重新开始的位置
```

bridge 端本来就优先使用发布的 root,不用改。`--no-root-translation` 可关掉(退回常量站高)。

**⚠️ 反复初始化:输入频率 / solver 吞吐不匹配**

solver p50 ≈ 46 ms(≈21 Hz),而发送端按 50 Hz 推 → 近半帧被丢、参考短时变旧 → 状态在
`TRACKING/HOLD` 之间跳。而 **HOLD 帧会被消费者原样转发并清空待处理窗口**
(`reference_provider.py`:`_pending = None; _current = None`),表现出来就是"参考被重置"。

已做:发送端默认降到 **30 Hz**(`--pico-fps` 可调)、`hold_after_ms` 50→**150 ms**
(`invalid_after_ms` 仍 250 ms 兜底)。**没解决**:solver 本身 ~21 Hz 是硬瓶颈,
要彻底消除抖动需要提速 solver 或再降输入频率。

**没有头显也能测 live 路径**(`--replay` 测不到它)

`--replay` 走的是录制加载器,**完全不经过 ZMQ 订阅端**,所以上面这两个 live 才有的问题
它一个都测不出来。要测就得造一个"假头显",发和真发送端**逐字节同格式**的包:

```bash
cd /home/wusichen/a3_teleop_ws/a3_teleop_bridge && source scripts/env_orin.sh
$PY_BRIDGE tools/fake_pico_sender.py --recording $A3WS/recordings/all/m5_stand --duration 60 &
$PY_UMR -m a3_teleop_bridge.apps.retarget_live --source pico --backend umr-online \
    --no-publish --duration 20
# 期望:[live] states [..., 'TRACKING', ...]  且 [live] auto-calibrated from N live frames
```

（payload 只有 `smpl_pose`/`smpl_joints`/`body_quat_w`,**故意不带 root** —— 和真发送端一致;
`--with-root` 可以验证"显式给了 root 时仍然优先用它"。）

**⚠️ 端口通 ≠ 头显在推数据**(本机实测踩过):

PC Service 会**先接受头显的 TCP 连接**(63901),而 `is_body_data_available()` 仍然是 False ——
头显 App 必须真的在推 body tracking。**所以在 preflight 里只查端口会给出假绿灯**:

```text
20:38:02  new RTC device connected: "TestDevice"     ← 头显连上
20:44:01  rtc device offline, uid: TestDevice        ← 掉线
20:51:00  run_pico_sim.sh                            ← 已掉线 7 分钟 → 白等 30 s
```

现在 preflight 会**直接问 SDK**(`tools/probe_pico_sdk.py --timeout 8`):

```text
[ OK ] PC Service listening on 127.0.0.1:60061
[FAIL] PC Service is up but NO body data is arriving from the headset
```

看到这条就去查头显 App 是否还活着。想看服务侧证据:

```bash
grep -E 'device' ~/.local/share/PICOBusinessSuitData/log/$(date +%Y%m%d).txt | tail
# "new RTC device connected" → 连过;"rtc device offline" → 掉了
```

确实想"先起链路、后戴头显"就加 `--skip-pico-probe`(发送端会在 30 s 内等第一帧)。

**判据(逐条看)**

```text
PICO 发送端       日志出现 "Stream state: RUNNING" 且 sent 递增
                  没出现 → 按手柄 A 键(发送端默认 PAUSED),或查 PC Service 是否在本机
T2 状态机         DISCONNECTED → CALIBRATION → TRACKING
T2 rejected       0
T2 solver p50     30–60 ms(5060 比 Orin 快;本机实测 36–48 ms)
T3 fall           false
T3 root z         ≈1.07 m
视觉              抬右臂→右臂抬;转体→腰转
```

**第一批动作(严格按序,方案 §45)**

```text
1 站立 → 2 轻微摆臂 → 3 单臂抬起 → 4 双臂抬起
5 慢速屈膝 → 6 左右重心转移 → 7 轻微抬脚 → 8 慢速迈一步
禁止:跳跃 / 快跑 / 深蹲到底 / 跪 / 躺 / 大幅单腿站立 / 快速 180° 旋转
```

**随时可停**:`Ctrl-C` 即可 —— 发送端会被一起收掉;参考停发后 MuJoCo 侧 50 ms hold、
250 ms safe stop,机器人保持站立(期望行为)。

**产物**

```text
$A3WS/logs/sim_teleop/<时间戳>/
   pico_sender.log      发送端日志(看 RUNNING / sent)
   retarget_live.log    参考侧日志(看状态机 / rejected / solver p50)
   sim2sim.log          MuJoCo 侧日志
   metrics.json         fall / root z / RMSE(29)
   live_chain_report.json   链路判定(ACCEPTED 才算过)
```

> **没接上头显时是安全的**:第一个参考窗口到之前,SIM 用 motion 首帧当站立占位,
> 最多等 30 s,策略照常 50 Hz 跑、机器人站着。超时后日志直接告诉你
> `is the PICO sender RUNNING (--start-unpaused or the controller's A button) on port 5556?`
> 实测(本机无头显、无 PC Service):
>
> ```text
> [live-chain] fall=False root_z=1.0702  published=0
> [live-chain] NOT ACCEPTED -- 2 problem(s):
>   - pipeline published nothing
>   - no reference was ever published: is the PICO sender RUNNING
>     (--start_unpaused or the controller's A button) on port 5556?
> ```
>
> 发送端那边的日志会停在 `initialize sdk,connect127.0.0.1:60061` ——
> **那个端口就是本机 PC Service 的地址**,起没起服务一看便知。

---

### 17.2 真机 —— `run_robot_live.sh`

```bash
cd /home/wusichen/a3_teleop_ws/a3_teleop_bridge && source scripts/env_orin.sh
bash scripts/run_robot_live.sh --check                         # 先自检(不会发任何东西)
bash scripts/run_robot_live.sh --a3-host 10.42.10.12 --duration 1800 --confirm-live
```

方向:**本机发布、A3 订阅**。本机 bind `tcp://0.0.0.0:5560`,A3 侧连
`tcp://<本机IP>:5560`;脚本会自己打出本机 LAN IP,并 `ping` 一下 `--a3-host`。

**安全闸**:不给 `--confirm-live` 直接拒绝启动;给了还会打印 A→G 清单并要求输入 `GO`
(想跳过交互加 `--yes`,适合脚本化)。这是防止误发动作到真机的最后一道手工闸。

#### 17.2.1 本机前置

```text
[ ] §17.1 的仿真已经在 MuJoCo 里跑通(没过仿真不上真机,方案 §13/§85)
[ ] XRoboToolkit PC Service + 头显就绪(和仿真同一条 PICO 链路)
[ ] 本机与 A3/HDU 网络互通:ip -4 addr show | grep inet ; ping -c 3 <A3的IP>
[ ] 端口 5560/tcp 没被防火墙/交换机隔离
[ ] SMPL-X 模型在位;§17.0 的解释器变量已导出
```

#### 17.2.2 A3 侧前置(**需要拷东西到机器人上**,只做一次)

这一段是「要下到对应实际机子」的部分。在**本机**交叉编译出 rockchip 包:

```bash
cd $A3WS/sonic_for_a3

# ① 构建输入:Rockchip sysroot(HuggingFace)
python download_from_hf.py --component sysroot
(cd gear_sonic_deploy/thirdparty/rockchip_sysroot && \
  sha256sum -c rockchip-1.0-aarch64-sysroot.tar.gz.sha256)

# ② aarch64 ONNX Runtime(仓库不附带,必须自己准备一个 tar.gz)
export A3_ONNXRUNTIME_AARCH64_TARBALL='/absolute/path/onnxruntime-aarch64-1.19.2.tar.gz'

# ③ 交叉编译 rockchip 部署包(x86 包只能自检,rockchip 包才是上机的)
gear_sonic_deploy/scripts/build_a3_deploy_pkg.sh \
  --arch rockchip --jobs 20 \
  --onnxruntime-aarch64-tarball "$A3_ONNXRUNTIME_AARCH64_TARBALL" \
  --runtime-cfg gear_sonic_deploy/src/g1/g1_deploy_onnx_ref/config/a3_runtime_config.yaml

# ④ 还有模型权重(RKNN)与我们的适配节点
.venv_sim/bin/python download_from_hf.py --component pt onnx rknn
```

传到机器人(HDU 只当跳板):

```bash
rsync -avP <pkg-dir>/ <user>@<hdu>:/tmp/a3_pkg/
ssh <hdu> "rsync -avP /tmp/a3_pkg/ <mdu-user>@<mdu>:/agibot/<sonic-package-root>/"
```

**在 MDU 上**(官方 runtime 照旧,我们只把参考流接进它已经在消费的入口):

```bash
cd /agibot/software/v0/config/sm
sudo cp -a sm_config.yaml "sm_config.yaml.before_sonic_$(date +%Y%m%d_%H%M%S)"
sudoedit sm_config.yaml        # Motion 功能组里把含 "mc" 的列表改成 [ "agent" ]
sudo systemctl restart agibot_pm
# 等约 3 分钟
systemctl is-active --quiet agibot_pm && echo OK
ps aux | grep '[m]otion_control'      # 必须无输出
```

然后起**适配节点**:把 `A3_REFERENCE_V1` 转成官方 runtime 已经在读的
`/ta/whole_body_command`(`q_mujoco[29] / dq_mujoco[29] / head_q[2]`)。
关节顺序必须用 `generated/a3_contract.json` 的 `policy_to_il_index` 置换 ——
**禁止 `qpos[7:36]` 这种切片**。完整步骤见 `A3_ONBOARD.md` §4–§8。

先做 **receive-only 预检**(不发布电机命令),确认能收到参考帧再进下一步:

```bash
# MDU 上,适配节点 --receive-only;本机这时用 --check 或先不 --confirm-live
```

#### 17.2.3 上线(悬吊 + 10 级动作)

```text
[ ] 安全吊带/防坠已挂,物理急停在手,安全员在场,清场
[ ] 先发站立,再按 §17.1 的 8 步动作**分级**做
[ ] 任何异常:松手/急停 —— 参考停发后 A3 侧 50 ms hold、250 ms safe stop
```

`Ctrl-C` 停止发布。跑完把结果追加到 `docs/progress.md`(`A3_ONBOARD.md` §12 有记录模板)。

#### 17.2.4 判据

```text
本机 客户端连接数       ≥1(ss -tnp | grep 5560)
本机 frames_published   线性增长
本机 rejected           0
本机 solver p50         ≤50 ms(>100 ms 就先插电+性能模式,见 §13)
A3   receive-only       seq 单调、无 rejected
A3   首次动作           站立保持、无抖动;再做分级动作
```

> **兜底**:5060 上 UMR 跟不上时,把参考侧整条搬到 4090(§12.3),
> A3 的 `--reference-endpoint` 改成 4090 的 IP 即可,协议与 A3 侧完全不变。

---

### 17.3 剩余步骤一览

| 步骤 | 命令/入口 | 状态 |
| --- | --- | --- |
| §0–§5 环境/模型/自检 | 见本文 | ✅ 本机已完成(`check_orin_ready.sh` 16/0) |
| 仿真 A/B(无头显) | `run_a3_validation_suite.py` / `run_live_chain.py --recording` | ✅ 9/9 与 ACCEPTED(见 `mujoco_validation.md`) |
| **仿真 C(真头显)** | **`bash scripts/run_pico_sim.sh`** | ⏳ 需 PC Service + 头显 |
| **真机** | **`bash scripts/run_robot_live.sh --confirm-live`** | ⏳ 需 A3 侧部署包 + 悬吊 + 安全员 |
| 可选:官方 AimSim | §8 | 需 AimDK 的 `aimsim` wheel,未做 |

**第一次跑之前先看 §17.4 的自检清单**(对几条就能判断修复有没有生效);§17.5 是 2026-09-28 这轮「按现象查根因」的完整对照表。

### 17.4 修复后应该看到的变化(自检清单)

拉完代码第一次跑,对着这几条看,就知道修复有没有生效:

```text
[ ] scripts/run_pico_sim.sh --check
      期望 11 ok / 0 failed,其中必须有:
        [ OK ] headset is streaming body tracking (SDK sees body data)
      若是 [FAIL] ... NO body data → 头显 App 掉了,重开 App(见 §17.1.1)

[ ] 启动后终端里出现
      [live] auto-calibrated from N live frames: ...
      [live-chain] states [..., 'TRACKING', ...]
      没有 TRACKING 就说明标定没过,先站直 1~2 秒别动

[ ] 站着不动时,机器人应该是【手臂自然垂在身体两侧】
      旧版本是【手臂向两边平举】(T-pose)。这是 §17.2.6 那个源动作问题的修复效果

[ ] 走两步 → 机器人应该跟着【整体移动】
      旧版本只会原地踏步(发送端根本没发位移)

[ ] 状态栏不应频繁在 TRACKING/HOLD 之间跳
      仍会偶发(solver ~21 Hz 是瓶颈),但比修复前(48% 丢帧)明显减少
```

### 17.5 本轮修了什么(2026-09-28,按现象查)

| 现象 | 根因 | 修在哪 |
| --- | --- | --- |
| 站立不动也**直接倒地**,root_err→1.24 m 但 joint_l1 正常 | 发送端发的 `body_quat_w` 是**机载 deploy 运行时**的调整后坐标系,而 UMR 要的是 SMPL-X 的 root 朝向;多出两个因子把整个人转歪 | `zmq_subscriber.sender_root_quat_to_smplx()`(精确还原,任意姿态误差 1e-16) |
| 参考的骨盆**在地面上**(比机器人站的地方低 0.975 m) | 发送端不发 root 平移,bridge 兜底成 `[0,0,0]` | `DEFAULT_STANDING_PELVIS_HEIGHT_M`(缺 root 时重建为站立高度) |
| 状态机**永远停在 CALIBRATION** | 自动标定只覆盖 `trajectory`/`recording`,`pico` 没有路径 | `retarget_live.py` 用 live 前 ~1.5 s 的帧自动标定 |
| **走不动**,大步迈→原地踏步 | 发送端算了 `positions` 却丢弃,位移从未离开头显 | 发送端发布 `root_translation`(Z-up + 首帧锚定 + 站立高度) |
| **手臂向两边平举** | 源动作生成器从 SMPL-X rest 骨架插值,而 rest 骨架手臂是 74–82°(几乎水平);retarget 忠实复现了它 | 生成器加**自然站姿**(手臂 9.5°/12° 下垂),全链重建 |
| **反复初始化**(参考被重置) | solver p50 ~46 ms(≈21 Hz) vs 50 Hz 输入 → 近半帧被丢 → `TRACKING/HOLD` 抖动,而 HOLD 帧会清空消费者的待处理窗口 | 发送端默认 30 Hz + `hold_after_ms` 50→150 ms(未根治,瓶颈在 solver) |
| 膝**从来不弯** | 目标函数里膝无约束,foot 点云项主导 | UMR 实现 `solver.joint_map_cost` 膝姿态先验 |
| 忘了按 A 键时**报 JSONDecodeError** | stats 里的 `None` 不是合法 JSON | `ast.literal_eval` 解析 |
| SDK 编译在链接阶段失败 | `libPXREARobotSDK.so` 是 git-lfs 指针(133 字节) | `git lfs pull` |

相关文档:`DEPLOY_TARGET_DECISION.md`(选型)· `ORIN_FULL_RUNBOOK.md`(姊妹篇)·
`SIM_TELEOP.md`(仿真细节与判据)· `A3_ONBOARD.md`(机载与适配节点)·
`GO_LIVE_CHECKLIST.md`(单页清单)· `A3_OFFICIAL_INTERFACE.md`(官方接口对照)·
`DEPLOY_ORIN.md`(部署原理与排查)· `DELIVERY.md`(交付物与部署公钥)
