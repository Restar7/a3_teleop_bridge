# pico_setup.md — PICO 4 / XRoboToolkit 完整接入步骤

> 目标:`真人 → PICO → SMPL → (online UMR) → A3 参考流`。
> 配套文档:`DEPLOY_ORIN.md`(Orin 部署)、`A3_ONBOARD.md`(机载)、`GO_LIVE_CHECKLIST.md`(上线清单)。

## 0. 拓扑与端口(先记住这两个数)

```text
PICO 4 头显 ──Wi-Fi──► XRoboToolkit PC Service(Orin 或 4090)
                              │  xrobotoolkit_sdk (python, pybind11)
                              ▼
                   pico_pose_zmq_minimal.py  ──ZMQ:tcp://*:5556──►  bridge
                     (或 GR00T pico_manager_thread_server.py)      (PicoPoseSubscriber)
                                                                            │
                                                    online UMR → predictor → A3_REFERENCE_V1
                                                    ──tcp://*:5560──► A3 / MuJoCo
```

| 端口 | 方向 | 说明 |
| --- | --- | --- |
| **5556** | 发布端 bind → bridge connect | PICO packed `pose` 话题;两个发布端的默认值都是 5556 |
| **5560** | bridge bind → A3/MuJoCo connect | `A3_REFERENCE_V1` |

> ⚠️ 本工程早期配置写过 5561,与两个发布端默认值不一致,会导致**收不到任何 PICO 帧且不报错**。
> 已全部对齐为 **5556**(`configs/teleop.yaml`、`configs/network.yaml`、代码默认值、文档)。

---

## 1. 头显侧准备(一次性)

```text
[ ] PICO 4 / PICO 4 Pro 开启开发者模式,与 PC 在**同一 Wi-Fi / 同一网段**
[ ] 头显内安装并打开 XRoboToolkit 应用,填入 PC 的 IP(PC Service 所在机器)
[ ] 头显内确认全身追踪可用(body data available)
[ ] 操作者:站直、双脚自然分开、双臂自然下垂做一次标定姿势
[ ] 手柄:记住 **A 键** 用来开始/暂停发送(见 §4)
```

## 2. PC Service(运行在 Orin 或 4090,与头显同网)

XRoboToolkit 官方 PC Service 提供本地服务,Python SDK 通过它取数据。按官方仓库说明在目标机上
启动它(默认监听本机,头显通过 Wi-Fi 连入):

```bash
# 官方仓库:https://github.com/XR-Robotics/XRoboToolkit-PC-Service
# 启动 PC Service 后,头显端填入该机 IP;先用官方 demo 确认能取到 body 数据
```

判断标准:`xrobotoolkit_sdk` 能 `init()` 且 `is_body_data_available()` 为真(见 §4 的日志)。

## 3. Python SDK(xrobotoolkit_sdk)

SDK 源码随 `sonic_for_a3` 一起交付:

```bash
cd ~/a3_teleop_ws/sonic_for_a3/external_dependencies/XRoboToolkit-PC-Service-Pybind_X86_and_ARM64
```

**x86_64(4090)**:

```bash
bash setup_ubuntu.sh          # 拉取官方 PC-Service、编译 PXREARobotSDK、拷贝 lib/include
python setup.py install       # 装到当前 python 环境(哪个解释器跑发送端就装哪个)
```

**aarch64(Jetson Orin)** —— 官方脚本自带 Orin 分支:

```bash
bash setup_orin.sh            # git clone -b orin、编译、拷 lib/aarch64 与 include/aarch64
python setup.py install
python -c "import xrobotoolkit_sdk as xrt; print('sdk ok')"
```

> 用 conda 时脚本走 `conda install -c conda-forge pybind11 libstdcxx-ng`;用 venv 时走
> `pip install pybind11`。两者都可以,但**跑发送端的解释器必须装了 SDK**。

## 4. 启动发布端(选一个)

### 4.1 A3 专用最小发送端(推荐,只发 A3 需要的字段)

```bash
cd ~/a3_teleop_ws/sonic_for_a3
.venv_sim/bin/python gear_sonic/scripts/pico_pose_zmq_minimal.py \
    --port 5556 --target_fps 50 --start_unpaused
```

**重要**:该发送端**默认 PAUSED**,不会发任何数据。两种开始方式:

```text
① 加 --start_unpaused(推荐,脚本化/无人值守)
② 按手柄 **A 键** 切换 RUNNING/PAUSED(日志打印 "A pressed: Stream state -> RUNNING")
```

日志出现下面两行才算真的在发:

```text
[minimal] Stream state: RUNNING (press A to toggle)
[minimal] ... sent=<递增> skipped=<少量> state=RUNNING
```

其他有用参数:`--max_abs_smpl_joint 5.0`(丢异常帧)、`--max_frame_jump 1.25`(丢跳变帧)、
`--human_joints_npz`(SMPL FK 数据,默认 `gear_sonic/data/human/human_joints_info.npz`)。

### 4.2 上游 GR00T 全量发送端(可视化用)

```bash
cd <GR00T-WholeBodyControl 路径>
bash install_scripts/install_pico.sh
source .venv_teleop/bin/activate
python gear_sonic/scripts/pico_manager_thread_server.py --manager --vis_vr3pt --vis_smpl
# 同样监听 5556,发布同一个 "pose" 话题
```

适合先验证 `真人 → PICO → SMPL avatar`,不接 A3 policy。

## 5. 桥接侧验证(不接机器人)

```bash
cd ~/a3_teleop_ws/a3_teleop_bridge && source scripts/env_orin.sh

# ① 只收不发:确认能解帧、形状/四元数校验通过
$PY_BRIDGE - <<'PY'
import time
from a3_teleop_bridge.pico.zmq_subscriber import PicoPoseSubscriber, TeleopConfig
sub = PicoPoseSubscriber(TeleopConfig.from_yaml())
t0 = time.time(); n = 0
while time.time() - t0 < 10:
    r = sub.poll(timeout_ms=200)
    if r and not r.rejected and r.smpl_frame is not None:
        n += 1
        if n == 1:
            f = r.smpl_frame
            print("first frame: joints", f.smpl_joints.shape, "pose", f.smpl_pose.shape, "seq", f.seq)
print("received", n, "frames in 10 s", sub.stats())
PY

# ② 录一段(方案 §24 的动作脚本,终端会逐段提示)
$PY_BRIDGE -m a3_teleop_bridge.apps.record_pico --duration 20 \
    --out $A3WS/recordings/$(date +%Y%m%d_%H%M%S)_session
```

录制动作脚本(便于回归):

```text
0-3s 站立 | 3-6s 抬左手 | 6-9s 抬右手 | 9-12s 屈膝
12-15s 抬左脚 | 15-18s 抬右脚 | 18-20s 站立
```

`received 0` 的排查顺序:`--start_unpaused` / A 键 → 端口是否 5556 → PC Service 与头显是否连通 →
`ss -ltnp | grep 5556` 看发送端是否真的在 bind。

## 6. 没有头显时怎么验证同一条链路

用录制文件当源(只换 `--source`):

```bash
$PY_UMR -m a3_teleop_bridge.apps.retarget_live \
    --source recording --backend umr-online \
    --recording $A3WS/recordings/m5_twist_torso_left \
    --duration 8 --no-publish --playback-hz 30 --stats /tmp/online.json
# 实测:frames in=240 solved=182,solver 38.7 ms,rejected=0
```

## 7. 已知约束

* 发送端**不发布 root 世界平移**:只有 root 相对的局部关节 + 朝向(`body_quat_w`)。bridge 的
  source adapter 优先用 `root_translation`/世界关节,没有时按局部关节 + 朝向重建,并在 metadata 标注。
* `joint_pos` / `joint_vel`(29 维)是 **G1 风格手腕估计,不是 A3 关节**,bridge 一律忽略。
* 头显数据 60–90 Hz 到达,bridge 用 latest-only(HWM=1 + CONFLATE)只取最新帧,
  绝不排队回放旧帧(方案 §31/§38)。
* 标定:首次 `--save-calibration` 生成 `calibration.json`,之后 `--calibration` 载入,
  状态机才会从 CALIBRATION 进 TRACKING(方案 §73/§74)。
