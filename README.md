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
