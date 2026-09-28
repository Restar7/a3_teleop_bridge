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

# ⑥ 仿真(PICO 遥操)
cd ../sonic_for_a3 && .venv_pico_minimal/bin/python \
    gear_sonic/scripts/pico_pose_zmq_minimal.py --port 5556 --start_unpaused
cd ../a3_teleop_bridge && $PY_UMR tools/run_live_chain.py --pico \
    --csv ~/a3_teleop_ws/logs/a3_validation/m5_twist_torso_left/m5_twist_torso_left.csv \
    --policy-steps 3000 --duration 150

# ⑦ 上机
bash scripts/run_orin_live.sh --duration 1800 --endpoint tcp://0.0.0.0:5560
```

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

相关文档:`DEPLOY_TARGET_DECISION.md`(选型)· `ORIN_FULL_RUNBOOK.md`(姊妹篇)·
`SIM_TELEOP.md`(仿真细节与判据)· `A3_ONBOARD.md`(机载与适配节点)·
`GO_LIVE_CHECKLIST.md`(单页清单)· `A3_OFFICIAL_INTERFACE.md`(官方接口对照)·
`DEPLOY_ORIN.md`(部署原理与排查)· `DELIVERY.md`(交付物与部署公钥)
