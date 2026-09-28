# SIM_TELEOP.md — 在仿真里用 PICO 遥操(真机之前的最后一道关)

> 目标:操作者戴上 PICO,**在 MuJoCo 里**驱动 A3 跟随自己的动作。
> 全链路:`PICO → SMPL → online UMR → predictor → A3_REFERENCE_V1 → SONIC A3-fast → MuJoCo`。
>
> 这条链路就是 M7 的内容,只把「A3 真机」换成「MuJoCo」。**仿真里摔倒是免费的,
> 所以这里该把所有问题都暴露出来**,包括标定、延迟、丢帧、动作越界。
> 相关文档:`pico_setup.md`(PICO 环境)、`GO_LIVE_CHECKLIST.md`(上线清单)、`A3_ONBOARD.md`(后面才用)。

---

## 0. 三种仿真验证,按顺序做

| 层次 | 参考来源 | 需要头显 | 验证什么 | 命令 |
| --- | --- | --- | --- | --- |
| **A** 离线回放 | 录制文件 → 离线 UMR → CSV | 否 | SONIC 与 A3 参考是否跟得住 | `tools/run_a3_validation_suite.py` |
| **B** 录制驱动 online UMR | `recordings/m5_*` | 否 | online UMR + predictor + 发布/订阅是否通 | `tools/run_live_chain.py --recording …` |
| **C** **真 PICO 遥操仿真** | 头显实时数据 | **是** | 操作者能不能真的遥操(本文重点) | `tools/run_live_chain.py --pico` |

A、B 两层的实测结果(本仓库已跑过):

```text
A 9/9 通过,RMSE 0.054–0.133 rad,全部 fall=false
B 标定版 1500 步:fall=false,root z 1.0711 m,RMSE 0.177,状态机 DISCONNECTED→TRACKING,
  求解器 p50 37.5 ms,rejected=0
```

**先把 A、B 跑通再上 C**:这样 C 出问题时,可以确定问题在 PICO 侧,而不是在 UMR/SONIC 侧。

---

## 1. 准备(一次性)

### 1.1 硬件与网络

```text
[ ] PICO 4 / PICO 4 Pro(开发者模式已开)
[ ] 头显与 4090 在同一 Wi-Fi / 同一网段
[ ] 4090 上跑 XRoboToolkit PC Service(头显连它;端口默认 63901)
```

> PC Service 与 **PICO 发送端必须在同一台机器**上:SDK 的 `PXREAInit()` 不带地址参数,
> 它连的是本机服务。所以「头显 → 4090(PC Service + 发送端 + bridge + MuJoCo)」这一形态
> 就是仿真阶段最简单的部署。

### 1.2 PC Service(4090)

用官方发行包(Linux x86_64)或自行编译,只启动服务进程:

```bash
# 官方仓库:https://github.com/XR-Robotics/XRoboToolkit-PC-Service
runService.sh          # 安装包自带(Windows 下是 runService.bat)
# 启动后确认头显已连上:服务界面/日志里能看到设备在线
```

### 1.3 Python SDK(`xrobotoolkit_sdk`)

```bash
cd $A3WS/sonic_for_a3/external_dependencies/XRoboToolkit-PC-Service-Pybind_X86_and_ARM64
bash setup_ubuntu.sh          # x86_64;Orin 上用 setup_orin.sh
python setup.py install       # 装进「跑发送端」的那个解释器
python -c "import xrobotoolkit_sdk as xrt; print('sdk ok')"
```

### 1.4 本仓库环境

```bash
cd $A3WS/a3_teleop_bridge && source scripts/env_orin.sh    # 4090 上同样适用
bash scripts/check_orin_ready.sh                            # 期望 16 ok, 0 failed
```

---

## 2. C 层:真 PICO 遥操仿真(三个终端)

### 终端 1 — PICO → ZMQ(发送端)

```bash
cd $A3WS/sonic_for_a3
.venv_sim/bin/python gear_sonic/scripts/pico_pose_zmq_minimal.py \
    --port 5556 --target_fps 50 --start_unpaused
```

**必须看到**:

```text
[minimal] Stream state: RUNNING (press A to toggle)
[minimal] ... sent=<递增> skipped=<少量> state=RUNNING
```

> 发送端**默认 PAUSED**:不加 `--start_unpaused` 就按手柄 **A 键**切到 RUNNING。
> 这一步没通,后面所有环节都会「静默收不到数据」。
> 操作者此时先站直、双脚自然分开做一次标定姿势。

### 终端 2 + 3 — 参考生成 + MuJoCo(一条命令拉起)

```bash
cd $A3WS/a3_teleop_bridge && source scripts/env_orin.sh

$PY_UMR tools/run_live_chain.py --pico \
    --csv   $A3WS/logs/a3_validation/stand/stand.csv \
    --csv-fps 30 \
    --policy-steps 3000 --duration 150 \
    --port 5560 \
    --out-dir $A3WS/logs/sim_teleop/pico_$(date +%H%M%S)
```

这一条命令内部:

```text
T2  retarget_live --source pico --backend umr-online --publish --endpoint tcp://0.0.0.0:5560
     └ PICO 订阅(5556)→ online UMR(≈20 Hz)→ predictor(50 Hz)→ 发布 A3_REFERENCE_V1
T3  SONIC sim2sim --reference-source stream --reference-endpoint tcp://127.0.0.1:5560 --realtime
     └ A3-fast 50 Hz → MuJoCo
```

> `--csv .../stand.csv` 只提供策略自身的状态输入(初始化/兜底),**动作参考来自你的实时数据**。
> 所以仿真里机器人「站住」靠的是 policy,「跟随你」靠的是这条流。

### 分开两个终端(想看得更清楚时)

```bash
# T2:参考生成
$PY_UMR -m a3_teleop_bridge.apps.retarget_live \
    --source pico --backend umr-online --publish \
    --endpoint tcp://0.0.0.0:5560 --duration 600 \
    --save-calibration $A3WS/logs/sim_teleop/calibration.json \
    --stats $A3WS/logs/sim_teleop/pipeline_stats.json

# T3:MuJoCo 消费
bash scripts/run_mujoco_consumer.sh --endpoint tcp://127.0.0.1:5560 \
     --motion $A3WS/logs/a3_validation/stand/stand.csv --steps 3000
```

---

## 3. 判据(每一条都要看)

| 观察点 | 期望 | 不满足说明什么 |
| --- | --- | --- |
| 终端 1 `sent` | 持续递增(≈50 Hz) | PICO/SDK/PC Service 有问题,不是 UMR |
| T2 日志 `received` | > 0 且持续 | 端口不是 5556,或发送端仍 PAUSED |
| T2 状态机 | `DISCONNECTED → CALIBRATION`;载入标定后 **`TRACKING`** | 没有标定文件,或首帧没有真实人体 |
| T2 `solver p50` | 30–60 ms(4090 实测 37.5) | 同机 CPU 竞争;试着关掉别的负载 |
| T2 `rejected` | **0** | 有 NaN/越限包,查 UMR 输出 |
| T3 `fall` | **false** | 参考跳变过大或策略跟不上,见 §5 |
| T3 root z | ≈1.07 m | 塌陷/滑步 |
| 视觉 | 抬右臂 → 机器人右臂抬;转体 → 腰转 | 关节顺序或镜像错了(见 §5) |

**第一批动作(严格按序,方案 §45)**:

```text
1 站立 → 2 轻微摆臂 → 3 单臂抬起 → 4 双臂抬起
5 慢速屈膝 → 6 左右重心转移 → 7 轻微抬脚 → 8 慢速迈一步
```

禁止:跳跃、快跑、深蹲到底、跪、躺、大幅单腿站立、快速 180° 旋转。

**随时可停**:Ctrl-C 掉 T3(或整条命令)即可;published 停止后 MuJoCo 侧会在
`hold_after_ms`(默认 50 ms)后 hold,250 ms 后 safe stop —— 仿真里表现为机器人
停止跟随并保持站立,这是**期望行为**。

### 3.1 头显还没连上时会发生什么(已实测)

online UMR 首次装配要 ~15 s,所以**先起 MuJoCo 是安全的**,但需要一版配合。
本仓库已改成:第一个参考窗口到达之前,SONIC 用 motion 的第一帧当「站立占位」,
最多等 `--reference-startup-wait-s`(默认 30 s),期间策略照常 50 Hz 运行、机器人站着:

```text
[reference-stream] no A3_REFERENCE_V1 packet yet; holding the startup pose for up
to 30 s (is the PICO sender RUNNING and the bridge publishing?)
```

实测(4090,故意不发 PICO 数据):

```bash
$PY_UMR tools/run_live_chain.py --pico --csv $A3WS/logs/a3_validation/stand/stand.csv \
    --policy-steps 400 --duration 90 --port 15674 --out-dir $A3WS/logs/sim_teleop/pico_smoke
```

```text
策略步数 149(stand.csv 长度)  fall=false  roll/pitch max 1.54°
链路判定 NOT ACCEPTED,problems:
  - pipeline published nothing
  - no reference was ever published: is the PICO sender RUNNING
    (--start_unpaused or the controller's A button) on port 5556?
```

也就是说:**没数据时机器人站着不动、不会崩、也不会假装成功**,日志直接告诉你按 A 键。
超过等待时间后按原逻辑报错退出;`--reference-startup-wait-s 0` 可关掉等待。

---

## 4. 录下来,便于复现与回归

```bash
# 录制 PICO 原始流(方案 §24)
$PY_BRIDGE -m a3_teleop_bridge.apps.record_pico --duration 30 \
    --out $A3WS/recordings/$(date +%Y%m%d_%H%M%S)_sim

# 录制参考流本身(方案 §79:以后不需要头显也能重播)
$PY_BRIDGE tools/record_reference.py --duration 30 \
    --out $A3WS/examples/reference_recording/sim_session
# 重播刚才那段参考流(不需要 PICO/UMR):
$PY_BRIDGE -m a3_teleop_bridge.apps.replay_reference \
    --windows $A3WS/examples/reference_recording/sim_session --publish-hz 50 --loop
```

---

## 5. 故障排查

| 现象 | 原因 / 处理 |
| --- | --- |
| T2 一直 `received 0` | 发送端 PAUSED(加 `--start_unpaused` 或按 A 键);端口非 5556;PC Service 与头显没连上 |
| 机器人不动但没倒 | 参考窗口没在发:`ss -ltnp | grep 5560`;看 T2 的 `frames_published` 是否增长 |
| 状态机停在 CALIBRATION | 先 `--save-calibration` 生成标定,再用 `--calibration` 载入;确认操作者在镜头/追踪范围内站立 |
| 关节动但**方向/部位不对** | 关节顺序或镜像问题:先用录制回放(B 层)复现,再查 `generated/a3_contract.json` 与 `coordinate_frames.md` |
| 动作幅度很小 / 机器人跟不上 | human→A3 身高比例:看 T2 的 `auto-calibrated: scale=…`;必要时用固定标定文件 |
| 参考频繁跳变 | 头显丢帧或操作者瞬移:看 T2 `jumps`/`max_gap_ms`;SONIC 侧有界插值能吸收,但持续跳变要查 Wi-Fi |
| 求解器 p50 > 100 ms | 同机竞争(策略 + UMR 抢 CPU):降低可视化和别的负载;真机形态是 UMR 放 Orin |
| MuJoCo 里摔了 | 减小动作幅度,确认用的是 `stand.csv` 而不是别的 motion;查看 `logs/sim_teleop/.../metrics.json` 的 `fall_tick` |
| 链路报 `pipeline published nothing` | 参考流一个窗口都没发出去 → 回终端 1 检查 PICO(最常见的失败) |
| 日志出现 `holding the startup pose for up to 30 s` | 正常等待提示;若一直等到超时,说明 PICO 侧没数据 |
| `RuntimeError: reference provider returned joint_pos (1, 29), expected (10, 29)` | 启动占位只平铺了 1 帧的旧 bug,已修;更新到最新 `feat/a3-streaming-reference` |

---

## 6. 安全边界(现在与以后)

```text
✅ 仿真阶段:随便试,摔了重开
❌ 不要在这一步连真机:仿真通过 ≠ 可以上机
   上机前必须依次完成 GO_LIVE_CHECKLIST.md 的 E(receive-only probe)与 F(悬吊 10 级)
❌ 不要把相机推理塞进 50 Hz 环路
❌ 不要绕过 A3 官方 safety / MDU
```

---

## 7. 从仿真到真机,只换两件事

```text
仿真:T2 发布 → 本机 MuJoCo 订阅
真机:T2 发布 → A3(经 A3_ONBOARD.md §2 的适配节点)→ 官方 runtime
     T2 从 4090 迁到 Orin(DEPLOY_ORIN.md),协议与端口不变
```

也就是说:**仿真里调好的一切(标定、动作幅度、限幅、状态机)在真机上原样有效**,
真机多出来的只有 receive-only 校验与悬吊分级。
