# safety.md — 安全设计、状态机与故障处理

## 1. 状态机(方案 §73)

```text
DISCONNECTED ──► CALIBRATION ──► READY ──► TRACKING
                     ▲                        │
                     │                        ▼
                   (重标定) ◄── SAFE_STOP ◄── HOLD
```

* 启动时**不会**直接进入 `TRACKING`:必须先有参考数据,再经过标定。
* `HOLD`:参考过期(≥ `hold_after_ms`,默认 50 ms)——冻结最后一次安全状态,
  关节速度置零,**绝不继续外推**。
* `SAFE_STOP`:过期时间继续扩大(≥ `invalid_after_ms`,默认 250 ms)或数据非法 ——
  下游必须进入安全状态。
* 阈值与官方 runtime 的 frame-age watchdog(50 ms)对齐,不随意放宽。

实现:`a3/predictor.py`(`StaleWatchdog` + `window()`)、`clocks.py`。

## 2. 故障处理矩阵(方案 §50 实测)

| 故障 | 行为 | 测试 |
| --- | --- | --- |
| publisher 停止 | subscriber 不再返回窗口(不发明数据),predictor HOLD→SAFE_STOP | `test_publisher_stop_leads_to_hold` |
| 包内 NaN/Inf | 整包 reject,保留上一个好窗口 | `test_nan_joint_rejected` / `test_inf_root_rejected` |
| 关节越界 | 转换阶段 clamp 到限位并记录;协议层不做静默修正 | `test_out_of_limit_joints_are_clamped` |
| 速度爆炸 | 按硬件速度限幅 | `test_velocity_explosion_is_clamped` |
| 延迟 500 ms | 判为 stale → SAFE_STOP,窗口冻结 | `test_delayed_packet_becomes_stale` |
| seq 乱序/重复/跳变 | 计数并拒绝当作新数据 | `test_out_of_order_and_duplicate_sequences` |
| 协议版本/布局/关节顺序不符 | reject(不尝试兼容) | `test_wrong_version_rejected` 等 |
| solver 失败 | 帧标记 invalid,下游 HOLD | `test_solver_failure_is_held_downstream` |
| solver 卡死(dt 过大) | 帧 invalid,HOLD | `test_bad_dt_marks_invalid_and_downstream_holds` |
| 队列积压 | HWM=1 + CONFLATE + 排空循环,只处理最新帧 | `test_stream_latest_only` |

## 3. 真机安全前置条件(方案 §62-§64)

必须在**全部**满足后才能进入下一级:

```text
1. MuJoCo 全链路连续 10 分钟无 NaN / 无掉帧 / 无 queue buildup(M6/M8 已具备)
2. receive-only bring-up:只读 A3 状态,不发布任何电机命令
3. 安全吊带 / 防坠 / 物理急停 / 现场安全员
4. 真机动作按 10 级顺序,任一级失败立即停止
```

真机 10 级动作顺序:

```text
1 stand      2 shoulder   3 elbow      4 wrist      5 torso small rotation
6 knee small bend          7 weight shift            8 foot unload
9 foot lift                10 slow single step
```

## 4. 绝对禁止(方案 §85)

```text
绕过 A3 现有安全层 / 绕过 MDU 直接发 motor command
没跑 MuJoCo 就上真机 / 让 UMR 每帧重算 correspondence
用阻塞 queue 积压旧 PICO 帧 / 在 future reference 里复制错误的 180 ms 旧帧
没有 timestamp 就做实时控制 / 用 Euler 线性预测 orientation
把 camera inference 放进 50 Hz WBC loop
```

## 5. 软件回退

* 任何阶段都可以中断 publisher:SONIC 侧会退回 HOLD,不会 extrapolate。
* `--reference-source csv` 是 streaming 的等价回退路径(回归测试保证逐位一致)。
* UMR 掉出实时(< 25 Hz)时的回退见方案 §47/§48:预测器补足中间帧,
  policy loop 不跟随 UMR 阻塞。
