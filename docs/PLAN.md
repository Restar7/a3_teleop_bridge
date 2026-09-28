# A3 Teleop Bridge — 主计划 (Master Plan)

> 目标链路:
> `PICO 4 → XRoboToolkit → SMPL → UMR → A3 canonical reference → future predictor → A3-fast/SONIC → A3 whole-body policy → A3 真机`

本文件把需求方案(89 节 / 27 个 commit / M1–M12 里程碑)映射成本仓库可执行的阶段,
并记录每个阶段的**验收标准**与**当前状态**。执行细节与日志见 `progress.md`。

---

## 0. 环境与路径约定(与本机实际适配)

| 方案原文 | 本机实际 | 说明 |
| --- | --- | --- |
| `~/a3_teleop_ws` | `$A3WS`,并软链为 `~/a3_teleop_ws` | 家目录 `/root` 在 overlay 上,工程数据放在 gpfs 大盘 |
| `conda env umr` | `UMR/.venv_umr` (uv, Python 3.12) | 本机无 conda,使用 uv venv 等价隔离 |
| `conda env a3_bridge` | `a3_teleop_bridge/.venv_bridge` (uv, Python 3.10) | 同上 |
| `sonic_for_a3` | `a3_teleop_ws/sonic_for_a3` (Restar7/sonic_for_a3 fork) | **必须 clone**:当前 cwd 的 `GR00T-WholeBodyControl` clone 不含 A3 代码 |
| `GR00T-WholeBodyControl` | `a3_teleop_ws/GR00T-WholeBodyControl` → 软链到已有 clone | 按用户要求**不重复 clone** |
| `UMR` | `a3_teleop_ws/UMR` (hanyang9/UMR) | 本阶段 clone |

上游版本固定在 `docs/upstream_versions.md`。

---

## 1. 阶段表(commit → 验收)

图例: ✅ 完成 / 🔄 进行中 / ⏸ 待做(依赖未满足) / ⛔ 阻塞(缺外部资产/硬件)

### 阶段 A — 工作区与契约 (TASK-001)

| commit | 内容 | 验收 | 状态 |
| --- | --- | --- | --- |
| 01 | workspace/bootstrap:目录、上游版本、计划、progress | 三个 repo SHA 落盘;分支建立 | ✅ |
| 02 | `tools/inspect_a3_contract.py` → `generated/a3_contract.json` | 从**源码**提取 29 joints / dt=0.02 / 10 frames / obs 维度 并 assert | ✅ |
| — | **M1**:官方 A3-fast MuJoCo baseline | `logs/baseline/` 记录 checkpoint/MJCF/motion/Hz/输入输出 shape;无 NaN、不立即倒地 | ✅ 1652 步 / fall=false / 0 NaN |
| 03 | bridge 数据类型 `types.py` + `clocks.py` + `a3/limits.py` | 单测:shape/dtype/valid 语义 | ✅ 46 tests |

### 阶段 B — UMR → A3 (TASK-002/003)

| commit | 内容 | 验收 | 状态 |
| --- | --- | --- | --- |
| — | **M2**:UMR 官方 G1 baseline | `output/unitree_g1_retarget/*.npz` 存在,qpos/fps/robot_joint_names 正常 | ✅ 3945 帧 / 0 NaN / cost 0.056 |
| 04 | `robot_configs/humanoid_retarget_agibot_a3.json` + env var 展开 | 使用 sonic_for_a3 的 035 MJCF;膝下界裁到 0 | ✅ UMR 91895f3 |
| 05 | `configs/a3_joint_map.yaml`(名称映射,禁止 index 假设) | assert 29 唯一 joint,head/passive 不在 policy | ✅ |
| 06 | `umr/state_converter.py` → `A3CanonicalState` | 名称映射 + clamp + 有限差分速度;dt 异常置 invalid | ✅ 74 tests |
| 07 | `a3/csv_export.py` + `tests/test_csv_roundtrip.py` | 与官方 sample CSV round-trip 一致 | ✅ 与官方 loader 逐数组一致 |
| 08 | **M3/M4**:`SMPL-X → UMR → A3 → CSV → A3-fast → MuJoCo` | 9 个验收动作(站立/左右抬手/屈膝/左右转体/左右抬脚/慢步)全部通过 | ✅ 9/9 PASS |

> ✅ 外部依赖已解决:用户提供官方 SMPL-X v1.1 包,已装入 `UMR/smpl/SMPLX_NEUTRAL.{pkl,npz}`
> (sha256 记录在 `progress.md`)。

### 阶段 C — PICO 链路

| commit | 内容 | 验收 | 状态 |
| --- | --- | --- | --- |
| 09 | `pico/zmq_subscriber.py` | 订阅 SONIC ZMQ,校验 (24,3)/(21,3),统计丢帧 | ✅ 9 tests;M5/M7 链路在用 |
| 10 | `pico/recorder.py` + `apps/record_pico.py` | 录制 npz + metadata + stats | ✅ 5 tests;`recordings/m5_*` 即其产物 |
| 11 | `umr/source_adapter.py` + root orientation 策略 | 四元数连续性、hip+shoulder 朝向社会 | ✅ 8 tests |
| 12 | **M5**:recorded PICO → UMR → A3 → MuJoCo | 离线全链路复现 | ✅ 4/4 PASS |

### 阶段 D — 在线化与网络

| commit | 内容 | 验收 | 状态 |
| --- | --- | --- | --- |
| 13 | `OnlineUMRRetargeter`(correspondence 只初始化一次 + warm start) | 单帧 step 无重复建图;记录 iterations/ms/cost | ✅ `92dee36` 装配完成;prepare 15.5 ms + solve 34.3 ms = 20 Hz |
| 14 | `a3/predictor.py`(One Euro + 常速外推 + 四元数指数映射) | 静止/匀速/限幅 3 组单测 | ✅ 16 tests(含静止/匀速/限幅);实时链路在用 |
| 15 | `transport/protocol.py` `A3_REFERENCE_V1` + publisher/subscriber | msgpack 二进制,latest-only,roundtrip 单测 | ✅ 13 tests;encode/decode 0.052/0.094 ms;C++ 端 16 项对拍通过 |
| 16 | `apps/replay_reference.py` | 离线轨迹 50Hz 模拟发布 | ✅ 实测 50.0 Hz |
| 17 | sonic_for_a3:`ReferenceProvider` 抽象(Csv/Streaming) | **回归测试**:新旧 obs 最大差 0.0 | ✅ 301d4f1 |
| 18 | **M6**:trajectory → ZMQ → StreamingReferenceProvider → A3-fast → MuJoCo | 连续 5 分钟无泄漏/无积压/无 NaN | ✅ 16000 步 / 265 s;后扩到 **36000 步 / 924 s** / fall=false / rejected=0 |
| 19 | **M7**:live PICO → UMR online → A3-fast → MuJoCo | 首批受限动作通过 | 🔄 **recorded PICO → online UMR 全链路 ACCEPTED**(1500/3000 步,fall=false,RMSE 0.177,TRACKING);只差真实 PICO 头显 |
| 20 | `tools/benchmark_latency.py` → `benchmarks/4090_live.json` | P50/P90/P95/P99/MAX 分项 | ✅ UMR online 20 Hz(离线批量 36 Hz),桥接 <1 ms |
| 21 | **M8**:故障注入 + watchdog(HOLD/INVALID/SAFE_STOP) | 断流/NaN/越界/乱序/延迟 全部安全降级 | ✅ 13 项故障注入测试;**§49 十分钟长跑 36000 步 / 924 s / fall=false / RMSE 0.1202** |

### 阶段 E — 迁移与真机

| commit | 内容 | 验收 | 状态 |
| --- | --- | --- | --- |
| 22 | Orin 打包 + `ORIN_BLOCKER.md` 机制 | **M9**:Orin 上 UMR/reference 可运行 | 📄 文档+模板就绪,执行需 Orin |
| 23 | A3 C++ `StreamingReferenceProvider` + watchdog | 与 Python 端字节级同协议 | ✅ 16 项 C++ 检查通过 |
| 24 | receive-only bring-up | **M10**:不 publish motor command | ⛔ 需 A3 真机 |
| 25 | 悬吊真机测试(10 级动作顺序) | **M11** | ⛔ 需 A3 真机 + 安全员 |
| 26 | camera adapter(独立进程) | 不进入 50Hz loop | 📄 接口文档就绪,需 A3 实际相机栈 |
| 27 | V2 TensorRT parity | **M12** 之后 | 📄 放在 Orin/V2 阶段 |

---

## 2. 不可违反的约束(方案 §0/§85)

1. 不修改 upstream main/master;每仓库用 feature branch。
2. 不重新训练 SONIC;使用官方 035 step-200000 checkpoint。
3. 所有坐标系/单位/joint order 必须来自源码验证;**禁止** `qpos[7:36]` 式 index 假设。
4. 4090 阶段不碰 A3 电机控制 / 安全控制 / parallel mechanism solver。
5. 真机前必须过 MuJoCo;不可绕过 MDU;不可把 camera 放进 50Hz WBC loop。
6. UMR correspondence 只初始化一次;队列 latest-only;必须带 timestamp。

## 3. 本机环境事实(2026-09-27)

- 2× RTX 4090,Driver 595.71.05 / CUDA 13.2
- Ubuntu 22.04 系,Python 3.12.3,gcc/cmake 3.31.6 可用
- `git-lfs 3.4.1` 已安装;**无 conda**,改用 uv;无 `gh`(用 https clone)
- 网络:github.com / huggingface.co / pypi.org 均可达
