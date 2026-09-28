# a3_teleop_bridge

PICO → SMPL → UMR → AgiBot A3 canonical reference → future predictor →
SONIC A3-fast → MuJoCo / A3 真机。

主计划: [`docs/PLAN.md`](docs/PLAN.md) · 执行记录: [`docs/progress.md`](docs/progress.md)
· 架构: [`docs/architecture.md`](docs/architecture.md)
· 坐标系与关节顺序: [`docs/coordinate_frames.md`](docs/coordinate_frames.md)

当前状态(2026-09-27):

```text
M1  官方 A3-fast MuJoCo baseline                 PASS
M2  官方 UMR G1 baseline                         PASS
M3  SMPL-X → UMR → A3                            PASS
M4  A3 → flat CSV → A3-fast → MuJoCo (9/9 动作)  PASS
M6  trajectory → ZMQ → streaming → MuJoCo        PASS(含 5 分钟耐久)
M5  recorded PICO → UMR → A3 → MuJoCo            PASS(4/4,docs/m5_pico_chain.md)
M7  实时 PICO                                   需要 PICO 4 硬件
```

---

## 0. 一次性环境准备

```bash
# 工作区(仓库已按 <workspace>/ 布局)
export A3WS=~/a3_teleop_ws                 # -> /inspire/hdd/.../wsc-workspace/a3_teleop_ws
export SONIC_A3_ROOT=$A3WS/sonic_for_a3
source $A3WS/env.sh                        # 可选:统一导出上面的变量
```

| 组件 | 环境 | 说明 |
| --- | --- | --- |
| `sonic_for_a3` | `$SONIC_A3_ROOT/.venv_sim` | MuJoCo sim2sim + A3-fast(uv venv,py3.10) |
| `UMR` | `$A3WS/UMR/.venv_umr` | 重定向(py3.12 + torch cu121) |
| bridge | `$A3WS/a3_teleop_bridge/.venv_bridge` | 本工程(py3.10) |

关键资产:SMPL-X neutral(`UMR/smpl/SMPLX_NEUTRAL.pkl`)、035 step-200000
checkpoint(`sonic_for_a3/checkpoints/035_step200000/model_step_200000.pt`,
sha256 `9cf33be2…`)。

---

## Mode A — 离线 UMR 测试(SMPL-X → A3 → MuJoCo)

```bash
# Terminal 1: 生成验收动作(若尚未生成)
cd $A3WS/a3_teleop_bridge
python tools/make_smplx_validation_motions.py

# Terminal 2: UMR 重定向(SMPL-X → A3),每个 clip 一次
python tools/run_umr_a3_batch.py --data-dir ~/a3_teleop_ws/data/smplx_validation --force

# Terminal 3: 数值验收 + CSV 导出 + A3-fast/MuJoCo 全链路
python tools/run_a3_validation_suite.py \
    --data-dir $A3WS/UMR/output/a3_validation \
    --out-dir   $A3WS/logs/a3_validation
# => 9/9 PASS,报告在 $A3WS/logs/a3_validation/report.json
```

单条轨迹手动跑:

```bash
python -m a3_teleop_bridge.apps.retarget_offline \
    --umr-result $A3WS/UMR/output/a3_validation/stand_smplx_agibot_a3.npz \
    --out-csv    $A3WS/logs/a3_validation/stand/stand.csv \
    --report     $A3WS/logs/a3_validation/stand/export_report.json
python tools/run_a3_baseline.py --motion $A3WS/logs/a3_validation/stand/stand.csv \
    --csv-source-fps 30 --csv-frame-stride 1
```

> **注意**:自研 CSV 是原生 30 fps,必须显式 `--csv-source-fps 30 --csv-frame-stride 1`;
> 仓库默认的 `stride 4` 是给 120 fps 老 sample 用的,用错会让 policy 倒地。

## Mode B — 录制回放(PICO 录制文件 → UMR → A3 → MuJoCo)

```bash
# Terminal 1: 回放 PICO 录制(合成示例已随仓库提供)
cd $A3WS/a3_teleop_bridge
python -m a3_teleop_bridge.apps.record_pico --synthetic --duration 20 \
    --out $A3WS/recordings/example_synthetic

# Terminal 2: 录制 → source adapter → UMR → A3 → MuJoCo(一条命令)
python tools/run_m5_pico_chain.py --clips stand,lift_left_foot,step_forward_slow

# Terminal 3: 轨迹 → ZMQ → MuJoCo(与 Mode C 相同,只是参考来自录制)
python -m a3_teleop_bridge.apps.replay_reference \
    --csv $A3WS/logs/a3_validation/stand/stand.csv --publish-hz 50 --loop
```

## Mode C — 实时 trajectory → ZMQ → A3-fast → MuJoCo(M6,无需 PICO)

```bash
cd $A3WS/a3_teleop_bridge
# 一条命令即可:内部启动 publisher(=Terminal 1)并运行 SONIC sim2sim(=Terminal 2)
python tools/run_a3_streaming.py \
    --csv $A3WS/logs/a3_validation/endurance_loop.csv \
    --policy-steps 18000 --duration 400
# => fall=false, root z ~1.07 m, publisher 50 Hz, rejected=0
```

分开两个终端时:

```bash
# Terminal 1(发布参考)
python -m a3_teleop_bridge.apps.replay_reference \
    --csv $A3WS/logs/a3_validation/stand/stand.csv --fps 30 --publish-hz 50 --loop

# Terminal 2(SONIC sim2sim,参考来自 stream)
cd $SONIC_A3_ROOT && source .venv_sim/bin/activate
python gear_sonic/scripts/sim2sim_a3_mujoco.py \
    --checkpoint checkpoints/035_step200000/model_step_200000.pt \
    --motion $A3WS/logs/a3_validation/stand/stand.csv \
    --csv-source-fps 30 --csv-frame-stride 1 \
    --encoder-mode a3_fast \
    --mjcf gear_sonic/data/assets/robot_description/mjcf/a3_t2d5_loop_passive_foot_twostage_fit_optimized.xml \
    --reference-source stream --reference-endpoint tcp://127.0.0.1:5560 \
    --batch-once
```

(上面的 `--mjcf` 请使用仓库默认值;示例里写全路径只是为了说明可覆盖。)

## Mode C.5 — recorded PICO → **online UMR** → ZMQ → A3-fast → MuJoCo(M7 除头显外)

与 Mode C 同一条实时链路,但参考不再来自回放,而是**真实 online UMR 求解器**
逐帧解算 recorded PICO 的人体帧。这是 M7 在没有头显时能验证的全部内容。

```bash
cd $A3WS/a3_teleop_bridge
source ../env.sh          # 需要 SONIC_A3_ROOT 等环境变量
# online UMR 会话需要一次性装配(约 15 s),故默认先等 25 s 再启动策略
$A3WS/UMR/.venv_umr/bin/python tools/run_live_chain.py \
    --csv $A3WS/logs/a3_validation/endurance_loop.csv --csv-fps 30 \
    --recording $A3WS/recordings/m5_twist_torso_left \
    --policy-steps 1500 --duration 90 --port 15664 \
    --out-dir $A3WS/logs/live_chain_umr_calib
# => fall=false, root z 1.071 m, RMSE 0.177 rad, states DISCONNECTED→TRACKING,
#    求解器 p50 37.5 ms, rejected=0
```

可用录制片段:`recordings/m5_{stand,step_forward_slow,twist_torso_left,lift_left_foot}`。

只跑参考侧(不启 MuJoCo)时:

```bash
$A3WS/UMR/.venv_umr/bin/python -m a3_teleop_bridge.apps.retarget_live \
    --source recording --backend umr-online \
    --recording $A3WS/recordings/m5_twist_torso_left \
    --duration 8 --no-publish --playback-hz 30 --stats /tmp/online.json
# => frames in=240 solved=182, solver 38.7 ms, rejected=0
```

> **注意**:online UMR 单帧 ≈ 40–60 ms(20–25 Hz),低于离线批量的 36 Hz;
> 参考流是 latest-only,策略按 50 Hz 消费,缺口由 SONIC 侧有界插值补齐
> (上面长跑 3000 步:interpolated 1499、rejected 0、未倒)。
> 同一台机器上策略与 UMR 抢 CPU 会把 p95 拉到 200 ms 以上,真机按方案 §22–24
> 把 UMR 放到 Orin 即可消除。

## 推送用的部署公钥

```text
ssh-ed25519 AAAAC3NzaC1lZDI1NTE5AAAAIDrJ4lUXNPn69rNCjgRhkAieGpu3AMAnOzjeAaeTCXPi a3-teleop-orin-deploy
```

指纹 `SHA256:WJ5l9QV0feolYc1LTRAOn0Tjd4G44CBCi/M0E2NQn0o`;加到 GitHub 账号级
**Settings → SSH and GPG keys**(不是单仓库 Deploy key,因为要推多个仓库)。
私钥仅存在打包机 `~/.ssh/id_ed25519_github_a3`(600),不要在仓库里出现。
详见 [`docs/DELIVERY.md`](docs/DELIVERY.md) §3。

## 部署与迁移到真机(Orin + A3 机载)

交付一个包、两台机器、两条手册:

```bash
# 1) 在 4090 上打包(16 MB,含 MANIFEST 与 sha256)
bash scripts/package_bundle.sh --with-umr
# 2) 传到 Orin 并解包
bash scripts/sync_to_orin.sh --bundle $A3WS/dist/a3_teleop_orin_<stamp>.tar.gz --host <ORIN_IP>
```

| 手册 | 内容 |
| --- | --- |
| [`docs/DEPLOY_ORIN.md`](docs/DEPLOY_ORIN.md) | Orin 全流程:预检 → 环境重建 → SMPL-X 手工拷贝 → 就绪门禁(16 项)→ 网络 → PICO+UMR live → MuJoCo 消费端验证 → 长跑/故障注入 → fallback |
| [`docs/A3_ONBOARD.md`](docs/A3_ONBOARD.md) | 机载:含**已实现的参考→通道桥**(`a3_teleop_command_source.*`,24 项 C++ 单测)与唯一的 AimRT 接线点 |
| [`docs/GO_LIVE_CHECKLIST.md`](docs/GO_LIVE_CHECKLIST.md) | **单页上线清单**:打包→Orin→PICO→MuJoCo 复核→机载→悬吊 10 级,每步带命令与判据 |
| [`docs/pico_setup.md`](docs/pico_setup.md) | PICO 头显 + PC Service + `xrobotoolkit_sdk`(x86/Orin)+ 发送端 + 收帧验证 |
| [`docs/DELIVERY.md`](docs/DELIVERY.md) | 交付清单、目标仓库、**部署公钥**、推送命令、bundle 离线路径 |
| [`docs/A3_ONBOARD.md`](docs/A3_ONBOARD.md) | A3 机载:交叉编译 rockchip 包 → 传输 → MDU 服务配置(agent-only)→ receive-only probe → 接入 `A3_REFERENCE_V1` → 悬吊 10 级动作 → 安全降级 |

脚本一览(全部 `--help` 可用):

```text
scripts/env_orin.sh              导出 A3WS/SONIC_A3_ROOT/UMR_ROOT/PY_*/端点
scripts/package_bundle.sh        打包(排除 venv/权重/SMPL-X/生成文件)
scripts/sync_to_orin.sh          rsync + 远端解包
scripts/orin_preflight.sh        系统/依赖/zmq 自检 → orin_system_info/
scripts/orin_bootstrap.sh        重建 aarch64 环境(--check-only 可先dry run)
scripts/check_orin_ready.sh      上机门禁:16 项(含 online UMR 真解一帧)
scripts/run_orin_live.sh         Orin 侧:PICO → online UMR → publisher
scripts/run_mujoco_consumer.sh   4090 侧:消费 Orin 的流跑 SONIC sim2sim
```

> PICO 端口是 **5556**(不是 5561):`pico_pose_zmq_minimal.py` 与 GR00T
> `pico_manager_thread_server.py` 的默认值都是 5556,本仓库配置已对齐。

## Mode D — 实时 PICO → A3(需要 PICO 4 / XRoboToolkit + A3 真机)

```bash
# Terminal A:XRoboToolkit PC service(Windows/PC 端)
# Terminal B:PICO → SMPL 流(先只验证 avatar,不接 policy)
cd $A3WS/GR00T-WholeBodyControl && source .venv_teleop/bin/activate
python gear_sonic/scripts/pico_manager_thread_server.py --manager --vis_vr3pt --vis_smpl

# Terminal C:订阅 SMPL 并录制/转成 UMR 输入
cd $A3WS/a3_teleop_bridge
python -m a3_teleop_bridge.apps.record_pico --duration 20

# Terminal D:bridge 在线链路(PICO → UMR online → predictor → publisher)
python -m a3_teleop_bridge.apps.retarget_live        # 需要 UMR online step API

# Terminal E:A3 MuJoCo streaming(同 Mode C 的 Terminal 2)
# 真机:见 docs/a3_bringup.md(receive-only → 悬吊 → 分级动作)
```

> Mode D 的 PICO 硬件与真机部分尚未执行:`docs/a3_bringup.md` 与
> `docs/orin_deployment.md` 记录了完整的执行步骤与安全前置条件。

---

## 工具速查

```text
tools/inspect_a3_contract.py          从源码提取并断言 A3 契约 -> generated/a3_contract.json
tools/extract_a3_joint_limits.py      MJCF+URDF+deploy 参数 -> generated/a3_joint_limits.yaml
tools/build_a3_tpose.py               FK 求解 A3 T-pose -> generated/a3_tpose.json
tools/make_umr_a3_config.py           -> UMR/robot_configs/humanoid_retarget_agibot_a3.json
tools/make_joint_map.py               -> configs/a3_joint_map.yaml
tools/make_predictor_config.py        -> configs/predictor.yaml
tools/make_network_config.py          -> configs/network.yaml
tools/make_teleop_config.py           -> configs/teleop.yaml
tools/make_smplx_validation_motions.py 生成方案 §21 验收动作
tools/run_umr_a3_batch.py             SMPL-X 批处理重定向
tools/validate_a3_motion.py           A3 轨迹数值验收(14 项)
tools/run_a3_baseline.py              官方 sim2sim 封装 + 验收
tools/run_a3_validation_suite.py      Mode A 全链路 + 报告
tools/run_a3_streaming.py             Mode C 全链路 + 报告
tools/benchmark_latency.py            -> benchmarks/4090_live.json
tools/inspect_umr_result.py           UMR 结果检查
```

## 测试

```bash
cd $A3WS/a3_teleop_bridge
.venv_bridge/bin/python -m pytest -q          # 137 tests
```

## 铁律(方案 §85)

* joint 一律按**名称**映射;线上数据是 **encoder(il)顺序**,不是 policy 顺序。
* `sonic_for_a3` 只做最小 interface 修改:CSV 路径经回归测试确认**逐位不变**。
* UMR 只暴露 online step API,不改核心算法。
* 真机前必须过 MuJoCo;不绕过 A3 MDU 与安全层;相机不进入 50 Hz WBC loop。
