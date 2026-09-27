# pico_setup.md — PICO / XRoboToolkit 接入

## 1. 上游(GR00T-WholeBodyControl,XRoboToolkit 全量栈)

```bash
cd ~/a3_teleop_ws/GR00T-WholeBodyControl
bash install_scripts/install_pico.sh
source .venv_teleop/bin/activate
python gear_sonic/scripts/pico_manager_thread_server.py --manager --vis_vr3pt --vis_smpl
```

这一阶段只验证 `真人 → PICO → SMPL avatar`,**不接 A3 policy**。

## 2. A3 专用最小发送端(smll,推荐)

`sonic_for_a3/gear_sonic/scripts/pico_pose_zmq_minimal.py` 直接发布 A3 需要的
packed `pose` 话题(topic + 1280B JSON header + 二进制字段):

```text
字段         形状          说明
smpl_pose    (N,21,3)      body axis-angle(不含 root)
smpl_joints  (N,24,3)      **root 相对**的局部关节位置
body_quat_w  (N,4)         人体朝向(wxyz)
joint_pos    (N,29)        G1 风格手腕估计 —— bridge **忽略**
joint_vel    (N,29)        同上,忽略
timestamp_realtime / timestamp_monotonic  (1,)
```

桥接侧订阅与校验:`src/a3_teleop_bridge/pico/zmq_subscriber.py`
(形状断言 (24,3)/(21,3)、四元数归一化、latest-only、seq/丢帧统计)。

## 3. 录制

```bash
cd ~/a3_teleop_ws/a3_teleop_bridge
python -m a3_teleop_bridge.apps.record_pico --duration 20 \
    --out ~/a3_teleop_ws/recordings/20260927_session
```

操作脚本(方案 §24,录制时终端会逐段提示):

```text
0-3s 站立 | 3-6s 抬左手 | 6-9s 抬右手 | 9-12s 屈膝
12-15s 抬左脚 | 15-18s 抬右脚 | 18-20s 站立
```

输出 `smpl.npz`(joints/pose/root/timestamps)、`metadata.json`、`stats.json`。

## 4. 已知约束

* 发送端不发布 root 世界平移(只有 root 相对的局部关节 + 朝向);
  bridge 的 source adapter(方案 §25)以 `root_translation`/`smpl_joints_world`
  存在时优先使用,否则用局部关节 + 朝向重建,并在 metadata 中标注。
* PICO 的 ZMQ 端口与 bridge reference 端口必须分开(默认 5561 / 5560)。
