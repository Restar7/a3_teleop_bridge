# A3_ONBOARD.md — 部署到 A3 机载(RK3588 + MDU)并接入遥操

> **前置**:`DEPLOY_ORIN.md` 已跑通(Orin 能发 `A3_REFERENCE_V1`,且 MuJoCo 消费端
> `fall=false`)。**没有过 MuJoCo 不准上真机**(方案 §13/§85)。
>
> 官方原始流程:`sonic_for_a3/docs/a3_training2sim2deploy.md` §5/§6、§6.1。
> 本文只增加「**我们的参考流怎么接进去**」以及完整命令序列。

---

## 1. 数据边界(不要越界)

```text
Orin                                      A3 MDU / RK3588
──────────────────────────────            ────────────────────────────────
PICO → SMPL → UMR online → predictor      safety watchdog → A3-fast(RKNN) 50 Hz
      → A3_REFERENCE_V1  ──Ethernet──►    → 29DoF action → parallel solver → MDU → motors
```

- A3 侧**保持官方**:`policy / safety / motor mapping / parallel solver / MDU` 全部沿用。
- 我们只增加**参考输入**:把 `A3_REFERENCE_V1` 变成官方 runtime 已经在消费的东西。
- 严禁:绕过安全层、绕过 MDU 直接发 motor command、在 50 Hz loop 里塞相机推理。

**关节顺序(最容易出错的一点)**

| 数据 | 顺序 |
| --- | --- |
| 我们的 wire payload(`A3_REFERENCE_V1`) | **il / IsaacLab 顺序**(`joint_order: a3_il_v1`) |
| 官方 runtime 的 `/ta/whole_body_command`(`q_mujoco`、`dq_mujoco`) | **policy / MuJoCo 顺序** |
| A3 flat CSV | CSV 顺序(含 head,31 列) |

所以适配节点**必须**用 `generated/a3_contract.json` 里的 `policy_to_il_index` 做置换,
再按名字断言;禁止 `qpos[7:36]` 之类的切片(方案 §0 第 5/6 条)。

---

## 2. 接入路线(选一条,推荐 A)

### 路线 A(推荐):适配节点 → `/ta/whole_body_command`

官方 runtime 已经有完全对应的入口(`A3TeleopReference`,`config/a3_runtime_config.yaml`
的 `teleop:` 段,默认 topic `/ta/whole_body_command`,字段 `q_mujoco[29] / dq_mujoco[29] /
head_q[2]`)。因此**不需要改官方 C++**:在 MDU 上跑一个薄适配节点:

```text
A3_REFERENCE_V1 (ZMQ, 10×29@il order) ──► 取第 0 帧 + 置换到 policy order ──► AimRT publish
                                                                              /ta/whole_body_command
```

**现状(已实现,只剩最后的 publish 调用)**:

```text
include/a3_deploy/a3_reference_stream.hpp/.cpp        收包/解码/校验(16 项单测)
include/a3_deploy/a3_teleop_joint_order.hpp           生成的 il<->policy 置换表(勿手改)
include/a3_deploy/a3_teleop_command_source.hpp/.cpp   window -> 通道字段 + 发布泵(24 项单测)
unit_tests/test_a3_teleop_command_source.cpp          独立测试,不需要 AimRT/ZMQ
```

`A3TeleopCommandPump::PushWindow()` 产出的 `A3WholeBodyCommandFields` 已经就是
`/ta/whole_body_command` 的字段分组(腰 3 / 左臂 7 / 右臂 7 / 左腿 6 / 右腿 6 /
pelvis 四元数 / 速度 30 槽),机器人侧只剩**把字段填进 `TaWholeBodyCommandChannel`
然后 publish**这一处 AimRT 代码:

```cpp
// 机器人侧接线(需要 AimRT + TA proto,故不在本机编译)
A3ReferenceStream stream;            // 收 A3_REFERENCE_V1
stream.Start({.endpoint = "tcp://<orin-ip>:5560", .enforce_limits = true});
A3TeleopCommandPump pump;            // window -> 通道字段
while (running) {
  stream.PollOnce();
  A3ReferenceWindow window;
  if (stream.Latest(&window)) {
    std::vector<A3WholeBodyCommandFields> cmds;
    if (pump.PushWindow(window, now_ns(), &cmds)) {
      for (const auto& c : cmds) PublishWholeBodyCommand(c);   // 约 20 行:AimRT publish
    }
  }
  if (pump.ShouldHold(now_ns())) { /* 停止 publish:runtime 自己 hold + 50 ms 后 safe halt */ }
  sleep_until_next_tick();           // 50 Hz
}
```

关键行为(都有单测):

```text
il -> policy 置换          kA3IlToPolicyIndex / kA3PolicyToIlIndex(由 contract 生成)
一窗多帧                  20 Hz 窗口拆成 10 条命令 → 官方 50 Hz 缓冲一直是密的
时间戳                    首窗标定时钟偏移,之后 t + i*dt;严格单调
去重                      重叠窗口里已发过的帧不再重发
安全                      NaN / 越限 / 相邻 tick 跳变 → 整窗拒绝
断流                      stale_after_ms(默认 50,与官方 frame-age watchdog 一致)→ 停止发布
head                      协议里没有头部数据,has_head_command=false,runtime 保持自己的头部目标
```

编译:`a3_teleop_command_source.cpp` 与 `a3_reference_stream.cpp` 都在
`gear_sonic_deploy/src/CMakeLists.txt` 的构建目标里(同一个包同一份 ZMQ 依赖)。
本机(无 AimRT)验证:

```bash
cd ~/a3_teleop_ws/a3_teleop_bridge
bash tools/run_cpp_teleop_command_test.sh      # 24 项检查,纯 C++17
bash tools/run_cpp_reference_test.sh           # 解码侧 16 项检查
```

**注意**:官方 `policy_parameters.hpp` 里那套 `isaaclab_to_mujoco` /
`mujoco_to_isaaclab` 是 **G1 约定**的顺序,**不是**本通道的顺序。本通道(policy view)
的顺序由官方 `ConvertTaWholeBodyCommand` 明确注释为
`waist[0..2] / left arm[3..9] / right arm[10..16] / left leg[17..22] / right leg[23..28]`,
与 bridge contract 的 `policy_joint_names` 一致 —— 置换表由
`tools/make_cpp_joint_order.py` 从 contract 生成,单测里逐名字核对,禁止手写。

### 路线 B:直接把 `A3ReferenceStream` 挂进 runtime

把 `A3ReferenceStream` 的解码结果喂给 `A3TeleopReferenceBuffer`(替换
`/ta/whole_body_command` 回调),相当于在 runtime 内部换源:

```text
main.cpp:
  a3_deploy::A3ReferenceStream stream;              // 已有
  stream.Start({.endpoint = "tcp://<orin-ip>:5560"});
  // 每 tick(50 Hz):stream.PollOnce(); stream.Latest(&w);
  //                teleop_reference.Push(w 的第 0 帧, policy 顺序)
```

代价:要动官方 `main.cpp`。**第一版不做**,等路线 A 跑稳、且 §63 悬吊验证通过后再考虑。

---

## 3. 网络(A3 ↔ Orin)

```bash
# 在 MDU 上(示例网段,按现场实际)
ip -4 addr show | grep inet
ping -c 3 192.168.10.10          # Orin 的 IP
# Orin 侧:bash scripts/run_orin_live.sh --endpoint tcp://0.0.0.0:5560
```

端口:参考流 **5560/tcp**(Orin 发布 → A3 订阅)。若现场用交换机,确认没有隔离 5560。

---

## 4. 构建 A3 部署包(在 x86 开发机上交叉编译)

```bash
cd ~/a3_teleop_ws/sonic_for_a3

# 4.1 Rockchip sysroot(HuggingFace,构建输入)
python download_from_hf.py --component sysroot
(cd gear_sonic_deploy/thirdparty/rockchip_sysroot && \
  sha256sum -c rockchip-1.0-aarch64-sysroot.tar.gz.sha256)

# 4.2 aarch64 ONNX Runtime(仓库不附带,必须自己准备)
export A3_ONNXRUNTIME_AARCH64_TARBALL='/absolute/path/onnxruntime-aarch64-1.19.2.tar.gz'

# 4.3 打包(x86 包先用来自检,rockchip 包才是上机的)
gear_sonic_deploy/scripts/build_a3_deploy_pkg.sh \
  --arch x86_64 --jobs 20 \
  --runtime-cfg gear_sonic_deploy/src/g1/g1_deploy_onnx_ref/config/a3_runtime_config.yaml

gear_sonic_deploy/scripts/build_a3_deploy_pkg.sh \
  --arch rockchip --jobs 20 \
  --onnxruntime-aarch64-tarball "$A3_ONNXRUNTIME_AARCH64_TARBALL" \
  --runtime-cfg gear_sonic_deploy/src/g1/g1_deploy_onnx_ref/config/a3_runtime_config.yaml
```

模型(RKNN)若还没有:

```bash
.venv_rknn/bin/python -m pip install -r gear_sonic_deploy/requirements-rknn.txt
.venv_rknn/bin/python gear_sonic_deploy/scripts/convert_a3_onnx_to_rknn.py \
  --onnx <a3_fast.onnx> --out-dir assets/a3_runtime/rknn_models/035_reexported
```

---

## 5. 传输到 MDU

```bash
# 开发机只负责构建与传输;HDU 只当 ssh/rsync 跳板
rsync -avP <pkg-dir>/ <user>@<hdu>:/tmp/a3_pkg/
ssh <hdu> "rsync -avP /tmp/a3_pkg/ <mdu-user>@<mdu>:/agibot/<sonic-package-root>/"
```

---

## 6. MDU 服务配置(**在 MDU 上执行**)

```bash
cd /agibot/software/v0/config/sm
BACKUP="sm_config.yaml.before_sonic_$(date +%Y%m%d_%H%M%S)"
sudo cp -a sm_config.yaml "$BACKUP"
sudoedit sm_config.yaml         # Motion 功能组里把含 "mc" 的列表改成 [ "agent" ]
sudo systemctl restart agibot_pm
# 等约 3 分钟
systemctl is-active --quiet agibot_pm && echo OK
ps aux | grep '[m]otion_control'   # 必须无输出
```

任一条件不满足就恢复备份并停下:

```bash
sudo cp -a "$BACKUP" sm_config.yaml && sudo systemctl restart agibot_pm
```

> 不要混用 `systemctl stop agibot_pm` + 手动 `start_hal_ethercat.sh` 的旧路径。

---

## 7. 包预检 + receive-only(不发布电机命令)

```bash
cd /agibot/<sonic-package-root>
file ./a3_deploy_onnx_ref | grep -F 'ARM aarch64'
grep -Eq '^[[:space:]]*backend:[[:space:]]*rknn[[:space:]]*$' config/a3_runtime_config.yaml
test -f models/model_step_200000_g1.rknn
test -f models/model_step_200000_a3_fast.rknn

export A3_ROBOT_ENV=/agibot/software/v0/entry/env/env.sh
test -r "$A3_ROBOT_ENV"

# 035 没有 SMPL encoder,**必须**用 g1 选路做 probe
A3_TRANSPORT=iceoryx A3_PROBE_SOURCE=g1 A3_LATENCY_LOG=verbose \
  taskset -c 4-5 ./run_a3_probe.sh
```

probe 通过判据:六路 body-drive 状态齐全、freshness/watchdog 正常、后端是 RKNN、
没有持续 stale、没有 safe halt。传输问题用 `taskset -c 4-5 ./run_a3.sh --dry-run` 定位。

---

## 8. 接入遥操参考(路线 A)

```text
1. 先确认 Orin 正在发流(bash scripts/run_orin_live.sh --endpoint tcp://0.0.0.0:5560)
2. 在 MDU 上启动适配节点(输出 /ta/whole_body_command)
3. 确认 runtime 日志出现 [teleop] first whole_body_command received
4. 参考新鲜度 > 50 ms → runtime 会 hold 并 safe halt(这是**期望行为**,不是 bug)
```

`config/a3_runtime_config.yaml` 中与遥操相关的量(保持官方默认,除非重新验证过):

```yaml
teleop:
  enabled: true
  topic: /ta/whole_body_command
  future_frame_skip: ...   # 与 Orin 侧 predictor 的窗口对齐
  delay_ms: 900            # 官方默认延迟窗口
  fast_future_frame_skip: 1
```

---

## 9. 悬吊 + 10 级动作(方案 §63/§64)

```bash
taskset -c 4-5 ./run_a3.sh
```

严格按序,**任一级失败立即停并回到上一级**:

```text
1 stand      2 shoulder   3 elbow     4 wrist      5 torso small rotation
6 knee small 7 weight shift 8 foot unload 9 foot lift 10 slow single step
```

每级:观察 ≥30 s → 记录(下面字段)→ 无异常才继续。现场必须:安全吊带/防坠、
物理急停、安全员、机器人周围清场。

---

## 10. 安全与降级(与官方对齐,不要放宽)

| 机制 | 官方默认 | 说明 |
| --- | --- | --- |
| policy 时钟 | 50 Hz | 不要改 |
| per-channel freshness | 10 ms | 诊断用 |
| frame-age watchdog | **50 ms** | 超时 → safe halt |
| `backend.sync_mode` | `latest_cache` | 不要在真机上换模式 |
| 我们的状态机 | DISCONNECTED→CALIBRATION→TRACKING→HOLD→SAFE_STOP | `HOLD`:50 ms;`SAFE_STOP`:250 ms(`configs/network.yaml`) |

故障演练(已在上机前用测试覆盖,现场再手演一遍):拔网线 → hold/safe halt;
kill PICO sender → hold;发 NaN/越界 → reject;乱序/重复 seq → 丢弃。

---

## 11. 参考来源切换

| 来源 | 用途 | 切换方式 |
| --- | --- | --- |
| 官方 CSV 回放 | 台架自检 / `a3_runtime_config.yaml` 的 motion 列表 | 官方方式,不动 |
| **我们的流**(Orin → `/ta/whole_body_command`) | 真机遥操 | 启动/停止适配节点 |
| MuJoCo 消费端 | 上机前验证同一条流 | `scripts/run_mujoco_consumer.sh --endpoint tcp://<orin-ip>:5560` |

切换只改**上游**,协议与 A3 侧不变(方案 §51 冻结接口)。

---

## 12. 记录模板(每次上机追加到 `docs/progress.md`)

```text
阶段 / commit SHA(a3_teleop_bridge + sonic_for_a3 + UMR)/ 执行命令 / 输入 / 输出 /
结果 / 已知问题 / 下一阶段
+ 包名与 sha256、checkpoint sha256、reference 录制文件、现场安全员
```
