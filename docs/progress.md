# progress.md — 阶段执行记录

每条记录格式:阶段 / commit SHA / 执行命令 / 输入 / 输出 / 结果 / 已知问题 / 下一阶段

---

## 阶段 00 — 工作区侦察与选址

- **时间**: 2026-09-27
- **执行命令**:
  ```bash
  pwd; ls -la
  git -C <cwd> remote -v; git -C <cwd> rev-parse HEAD
  nvidia-smi; uname -a; python3 --version
  df -h /root <parent-of-workspace>
  git ls-remote https://github.com/Restar7/sonic_for_a3.git HEAD
  ```
- **输入**: 当前 cwd = 上级工作区里的 `GR00T-WholeBodyControl` 克隆
- **输出 / 发现**:
  1. 当前目录 **不是** `sonic_for_a3`,而是 `Restar7/GR00T-WholeBodyControl`
     (HEAD `3c2dbe1` "适配flowbody"),其文件树中**不存在**任何 A3 资产:
     - 无 `gear_sonic/scripts/sim2sim_a3_mujoco.py`
     - 无 `gear_sonic/data/assets/robot_description/mjcf/a3_t2d5_*.xml`
     - 无 `a3_data/agibot_a3/*.csv`
     - 无 `gear_sonic_deploy/assets/a3_runtime/**`
  2. `Restar7/sonic_for_a3` 是独立仓库 (fork of `AgibotTech/sonic_for_a3`),
     HEAD `fe6868b`,包含上述全部 A3 资产 → **必须 clone**。
  3. 本机无 conda / 无 gh / 无 git-lfs(已补装 git-lfs 3.4.1)。
  4. 家目录 `/root` 在 3.5T overlay;工程数据盘 `/inspire/hdd/...` 剩 4.3T。
- **结果**: PASS(选址结论)
- **已知问题**: 用户原话"我们当前不是在sonic目录里面吗"与事实不符,已按"复用已有
  GR00T clone + 只 clone 缺失的 sonic_for_a3/UMR"处理,不重复 clone 已有仓库。
- **下一阶段**: 阶段 01 — bootstrap 目录、上游版本落盘、feature branch。

---

## 阶段 01 — workspace/bootstrap

- **时间**: 2026-09-27
- **执行命令**:
  ```bash
  apt-get install -y git-lfs && git lfs install --skip-repo
  WS=$A3WS
  mkdir -p "$WS" && ln -sfn "$WS" /root/a3_teleop_ws
  git clone https://github.com/Restar7/sonic_for_a3.git "$WS/sonic_for_a3"
  git clone https://github.com/hanyang9/UMR.git "$WS/UMR"
  ln -sfn <existing GR00T clone> "$WS/GR00T-WholeBodyControl"
  ```
- **输入**: 网络可达 github.com / huggingface.co / pypi.org
- **输出**:
  - `~/a3_teleop_ws/{sonic_for_a3,UMR,a3_teleop_bridge,system_info,logs}`
  - `docs/upstream_versions.md`(3 个 repo SHA)
  - `docs/PLAN.md`(89 节方案 → 阶段表)
- **结果**: 见本次 commit
- **已知问题**: 环境用 uv venv 替代 conda(本机无 conda)。
- **下一阶段**: 阶段 02 — `tools/inspect_a3_contract.py`(TASK-001)

---

## 阶段 02 — A3 contract inspector(TASK-001 前半)

- **commit**: `512c621` (bridge) — 与阶段 01 合并提交
- **执行命令**:
  ```bash
  python tools/inspect_a3_contract.py
  ```
- **输入**: `sonic_for_a3` 源码
  - `gear_sonic/scripts/sim2sim_a3_mujoco.py`(AST 求值模块级常量)
  - `gear_sonic_deploy/.../a3_deploy/a3_obs_builder.hpp`(C++ 常量)
  - `a3_data/agibot_a3/001_walk_front_slow.csv`(CSV 表头)
- **输出**: `generated/a3_contract.json`
- **结果**: PASS,全部 assert 通过
  ```text
  policy joints     = 29   (waist 3 + L_arm 7 + R_arm 7 + L_leg 6 + R_leg 6)
  excluded joints   = head_yaw_joint, head_pitch_joint
  passive joints    = left/right foot forefoot+toe (4)  -> 不在 policy
  reference dt      = 0.02 s (50 Hz)
  window            = 10 frames, a3_fast skip=1
  future horizon    = 0.18 s
  encoder frame     = 29 q + 29 dq + 6d ori = 64
  encoder input     = 10 x 64 = 640
  obs_dict          = 640 + 930 proprio = 1570
  action            = 29
  csv               = 38 列 = Frame + 6 root + 31 joints
  ```
- **已知问题**: 无
- **下一阶段**: M1 官方 sim2sim baseline

---

## 阶段 M1 — 官方 A3-fast MuJoCo baseline

- **commit**: `984671d`(tools/run_a3_baseline.py 与阶段 03 合并;证据见下)
- **执行命令**(即官方 `docs/a3_training2sim2deploy.md` §4 命令):
  ```bash
  cd ~/a3_teleop_ws/sonic_for_a3
  export MODEL_035_PT="$PWD/checkpoints/035_step200000/model_step_200000.pt"
  .venv_sim/bin/python gear_sonic/scripts/sim2sim_a3_mujoco.py \
    --checkpoint "$MODEL_035_PT" \
    --motion a3_data/agibot_a3/001_walk_front_slow.csv \
    --encoder-mode a3_fast \
    --mjcf gear_sonic/data/assets/robot_description/mjcf/a3_t2d5_loop_passive_foot_twostage_fit_optimized.xml \
    --batch-once --metrics-out ... --timeseries-out ...
  ```
  (同一命令的封装与验收检查: `python tools/run_a3_baseline.py [--smoke]`)
- **输入**:
  - checkpoint `checkpoints/035_step200000/model_step_200000.pt`
    sha256 `9cf33be2f4e602858b68ce31d5824113ab1dda250acaab5842b6d3bf88b70f2d`(与发布一致)
  - MJCF `a3_t2d5_loop_passive_foot_twostage_fit_optimized.xml`
  - motion `a3_data/agibot_a3/001_walk_front_slow.csv`(3965 行 → stride 4 → 1652 帧)
- **输出**: `~/a3_teleop_ws/logs/baseline/`
  ```text
  smoke_metrics.json / smoke_timeseries.json / smoke_run.log / smoke_metadata.json
  full_metrics.json  / full_timeseries.json(39 MB)/ full_run.log / full_metadata.json
  ```
- **结果**: **ACCEPTED (M1)**
  ```text
  returncode       = 0
  fall             = false
  policy steps     = 1652 @ 50 Hz (elapsed 25.1 s ≈ 66x realtime)
  policy q range   = [-1.0684, 1.3928] rad
  non-finite frames= 0
  root height      = mean 1.0519 m, min 1.0294 m
  root roll/pitch  = max 9.62°
  all_29 tracking  = RMSE 0.0662 rad
  encoder          = 640D = 10 x 64 ('a3_fast')
  actor obs dim    = 930 (proprio) -> 930 + 640 = 1570
  29DoF mapping    = OK (logical_policy=29, total_model_joints=48)
  ```
- **已知问题**:
  - 本机没有 EGL 平台,不能设 `MUJOCO_GL=egl`(会导致 `import mujoco` 失败);
    `--batch-once` 模式不需要 viewer,因此无需 GL。
- **下一阶段**: M2 官方 UMR G1 baseline(依赖 SMPL-X body model)

---

## 阶段 UMR-ENV — UMR 运行环境

- **执行命令**:
  ```bash
  uv venv .venv_umr --python 3.12
  uv pip install --python .venv_umr/bin/python torch==2.4.1 --index-url https://download.pytorch.org/whl/cu121
  uv pip install --python .venv_umr/bin/python -r requirements-umr.txt
  ```
- **输出**: `.venv_umr`(torch 2.4.1+cu121, smplx 0.1.28, trimesh 5.1.0, warp-lang 1.17.0,
  viser 1.1.1, mujoco, coacd/cholespy/embreex 等)
- **结果**: PASS — `torch.cuda.is_available() == True`(RTX 4090)
- **已知问题**: 无(用 uv 替代方案中的 conda env `umr`)
- **下一阶段**: UMR 官方 G1 baseline(需 SMPL-X neutral body model)

---

## 阶段 SMPL-X — 外部许可资产落地(方案 §11)

- **来源**: 用户提供的官方 SMPL-X v1.1 压缩包 `models_smplx_v1_1.zip`
  (sha256 `cb593838a5d602395735081c7a8fd1d6b04fba261042b84e24644d875039be61`,870,108,517 B)
- **解压路径**: `~/a3_teleop_ws/cache/downloads/smplx_extract/models/smplx/`
- **最终选用**:
  ```text
  UMR/smpl/SMPLX_NEUTRAL.pkl  sha256 381c808965deb4f5e845f8c3eddb0cd69930cc72e5774ce4f34c4ce3cf058361
  UMR/smpl/SMPLX_NEUTRAL.npz  sha256 376021446ddc86e99acacd795182bbef903e61d33b76b9d8b359c2b0865bd992
  ```
  未使用 MALE/FEMALE / removed-head-bun / Julia / SMPL-X 2020。
- **同时完成**: `UMR/robot_configs/humanoid_retarget_agibot_a3.json` + UMR config loader
  的 `${VAR}`/`~` 展开(commit `91895f3` on `feat/a3-online-retarget`)
- **结果**: PASS
- **下一阶段**: M2

---

## 阶段 M2 — 官方 UMR G1 baseline

- **执行命令**:
  ```bash
  cd ~/a3_teleop_ws/UMR
  .venv_umr/bin/python scripts/humanoid_retarget_pipeline.py \
      --config robot_configs/humanoid_retarget_unitree_g1_example.json --skip-view
  ```
- **输入**: `sample_data/lafan1_smplx/dance1_subject2.npz` + `smpl/SMPLX_NEUTRAL.pkl`
- **输出**:
  ```text
  output/unitree_g1_retarget/dance1_subject2_smplx_unitree_g1.npz
  output/correspondence_unitree_g1_smplx_neutral_betas_c4eb7419e9/{checkpoint-final.pt,correspondence_slots_final.npz}
  logs/umr/g1_baseline.log
  ```
- **结果**: **PASS**(`tools/inspect_umr_result.py`)
  ```text
  qpos            = (3945, 36)   29 scalar joints, 36 = free(7) + 29
  fps / dt        = 30.0 / 0.0333 s
  NaN count       = 0
  root height     = mean 0.746 m
  |quat|          = [1.000000, 1.000000]
  correspondence  = chamfer 0.000106 (500 epochs, 4090)
  retarget cost   = mean 0.0559 max 0.2425
  ```
- **已知问题**:
  - UMR 结果里的 `robot_joint_names` 与 `qpos` **不是**连续对应的(自由关节 + 球关节
    占 4 个 qpos 槽),必须经 robot XML 解析 qpos 地址 — 已在 `umr/offline.py` 实现并断言。
  - mujoco 3.14 的 `mjtJoint` 枚举与 numpy 标量比较不对称(`x in (mjJNT_HINGE, ...)`
    静默为 False),已改为 `int()` 比较(此 bug 曾导致 qpos 地址解析为空)。
- **下一阶段**: M3 — 用同一个 UMR 把 SMPL-X 动作重定向到 A3

---

## 阶段 06+07 — UMR→A3 state converter 与 A3 flat CSV codec

- **commit**: `c576998`
- **执行命令**: `pytest -q`(74 tests)
- **输入/输出**:
  - `umr/offline.py`: UMR npz → `UmrResult`(名字→qpos 地址经 XML 解析并断言)
  - `umr/state_converter.py`: → `A3CanonicalState`(root + 名称映射 + clamp + 有限差分速度)
  - `a3/csv_export.py`: ↔ flat CSV(cm / 外旋 XYZ 欧拉角 / 31 列含 head / `_dof` 别名)
- **结果**: PASS
  - 与官方 `load_a3_flat_csv` 对比(在 sonic venv 内直接执行官方函数):
    `root_pos`/`dof29` 误差 < 1e-12,四元数夹角 < 4e-6°(CSV 只有 ~9 位有效数字)
  - 56 帧官方 sample CSV 的 load → write → load 数值一致
- **已知问题**: 无
- **下一阶段**: M3 / M4

---

## 阶段 M3 — SMPL-X → UMR → A3

- **执行命令**:
  ```bash
  cd ~/a3_teleop_ws/UMR
  export SONIC_A3_ROOT=~/a3_teleop_ws/sonic_for_a3
  .venv_umr/bin/python scripts/humanoid_retarget_pipeline.py \
      --config robot_configs/humanoid_retarget_agibot_a3.json --skip-view
  ```
- **输入**: `sample_data/lafan1_smplx/dance1_subject2.npz`(3945 帧 @30fps)
- **输出**:
  ```text
  output/agibot_a3_retarget/dance1_subject2_smplx_agibot_a3.npz   qpos=(3945, 72)
  output/correspondence_agibot_a3_smplx_neutral_betas_c4eb7419e9/{checkpoint-final.pt,correspondence_slots_final.npz}
  generated/m3_umr_a3_report.json / generated/m3_umr_a3_validation.json
  ```
- **结果**: PASS
  ```text
  qpos            = (3945, 72)   72 = free(7) + 41 标量关节 + 球关节(4x6)
  NaN             = 0
  root height     = mean 0.991 m, range [0.600, 1.207]
  |quat|          = [1.000000, 1.000000]
  A3 mapping      = ok (29 policy joints, head/passive 已剔除, 0 limit violation)
  retarget cost   = mean 0.0722 max 0.2934 (G1 baseline 0.0559 / 0.2425)
  knee 方向       = 左右膝均 ≥ 0(无反向膝)
  左右镜像        = shoulder/hip/ankle roll 均值左右反号
  速度/加速度     = max |dq| 10.6 rad/s, max |ddq| 334 rad/s²
  ```
- **已知问题**:
  1. UMR 会把 floating MJCF 写到**源 MJCF 同目录**
     (`sonic_for_a3/.../mjcf/<seq>.floating_mjcf.xml`)。
  2. `dance1_subject2` 含大量旋转/跳跃,属方案 §21 明确排除的动作;
     用 heading-frame 判据可看到机器人 172 帧"双脚交叉",而同一判据在**人类源动作**
     上有 1770+ 帧"交叉",说明该判据对旋转动作不可靠 —— 因此 dance 只作为压力测试,
     验收改用方案 §21 指定的 6 类简单动作(见 M3b)。
  3. 修复了 UMR 的一个路径 bug(`meshdir` + 嵌套相对路径),commit `d892361`。

- **下一阶段**: M3b/M4 使用方案指定的简单动作做验收

---

## 阶段 M3b/M4 — 验收动作集 + MuJoCo 全链路

- **动作集生成**: `tools/make_smplx_validation_motions.py`
  - 9 段 SMPL-X clip(stand / raise_left_arm / raise_right_arm / bend_knees /
    lift_left_foot / lift_right_foot / twist_torso_left / twist_torso_right /
    step_forward_slow),各 90 帧 @30fps
  - 所有旋转轴由 FK **实测搜索**得到(左右腿分别搜索,不复制),每段 clip 都有
    数值验收(如抬左手时左腕 z 增益 +0.50 m 而右腕为 0)
  - 输出 `~/a3_teleop_ws/data/smplx_validation/*.npz` + 每段 json + manifest
- **批处理**: `tools/run_umr_a3_batch.py`(复用 pipeline 自己的命令,只替换
  `--data/--seq-key/--out`)
- **CSV 导出**: `python -m a3_teleop_bridge.apps.retarget_offline`
  - round-trip 误差: pos 5.0e-9 m / joints 8.7e-9 rad / quat 3.0e-6°
  - 6 帧发生 clamp,0 invalid
- **MuJoCo**: `tools/run_a3_baseline.py --csv-source-fps 30 --csv-frame-stride 1`
- **已知问题(重要)**:
  - 用**默认** `--csv-frame-stride 4` 跑自研 CSV 会 4 倍降采样参考并导致 policy 倒地
    (tick 337)。原生 30fps 导出**必须**显式 `--csv-source-fps 30 --csv-frame-stride 1`;
    `retarget_offline` 会把这个提示写进 report。
  - `dance1_subject2` 全片(6574 policy step)在 tick 1504 倒地 —— 该动作含跳跃/旋转,
    按方案 §21 属于本阶段**不做**的动作,记录为已知边界,不作为验收项。

---

## 阶段 M3b + M4 — 验收动作集全链路 PASS(9/9)

- **commit**: `479d18a`(工具) + 本条记录
- **执行命令**:
  ```bash
  # 1) 生成方案 §21 的验收动作(SMPL-X)
  python tools/make_smplx_validation_motions.py
  # 2) UMR: SMPL-X → A3(每个 clip 一次;correspondence 只建一次)
  python tools/run_umr_a3_batch.py --data-dir ~/a3_teleop_ws/data/smplx_validation --force
  # 3) 数值验收 + CSV 导出 + A3-fast/MuJoCo 全链路
  python tools/run_a3_validation_suite.py \
      --data-dir ~/a3_teleop_ws/UMR/output/a3_validation \
      --out-dir   ~/a3_teleop_ws/logs/a3_validation
  ```
- **输入**: 9 段 SMPL-X clip(各 90 帧 @30fps)
- **输出**: `logs/a3_validation/report.json` + 每 clip 的
  `validation.json / export_report.json / <clip>.csv / mujoco/full_metrics.json`
- **结果**: **9/9 PASS**

  | clip | MuJoCo | fall | root z | RMSE(29) | CSV round-trip |
  | --- | --- | --- | --- | --- | --- |
  | stand | ok | false | 1.073 m | 0.0537 | 7.1e-10 m |
  | raise_left_arm | ok | false | 1.070 | 0.0935 | 4.9e-09 |
  | raise_right_arm | ok | false | 1.072 | 0.0810 | 5.0e-09 |
  | bend_knees | ok | false | 1.056 | 0.1333 | 5.0e-09 |
  | twist_torso_left | ok | false | 1.073 | 0.0696 | 4.8e-09 |
  | twist_torso_right | ok | false | 1.073 | 0.0582 | 4.9e-09 |
  | lift_left_foot | ok | false | 1.072 | 0.0944 | 4.9e-09 |
  | lift_right_foot | ok | false | 1.073 | 0.0903 | 5.0e-09 |
  | step_forward_slow | ok | false | 1.070 | 0.0894 | 5.0e-09 |

  数值验收(每 clip 13 项)全部通过:有限性、root 高度带、无 teleport、四元数单位化、
  关节限位、膝方向(不反折)、左右镜像、速度/加速度、双脚不交叉(heading 参考系)、
  双脚在地面之上。

- **过程中修复/发现**:
  1. **合成动作必须使用正确的世界约定**:SMPL-X rest skeleton 是 Y-up,LaFan1 数据在
     `poses[:,0]` 里带 ~+90° X 旋转并以 `trans[:,2]≈0.93 m` 站立。最初把 global orient
     设为 0、`trans_z=0`,UMR 于是把机器人重定向到 root z≈0.05 m(趴在地上)。
  2. **旋转轴必须实测**:左右腿的弯曲/抬起轴不同;早期用左腿轴驱动右腿,右腿完全不动。
     现在每条腿的轴独立搜索并用 FK 验证。
  3. **膝限位收紧**:A3 URDF/MJCF 允许膝 −0.1222 rad,retargeter 会把膝压到该下界
     (膝反折)。按方案 §17"先调 joint limits"把 UMR 配置里膝下界裁到 0
     (`NON_NEGATIVE_JOINTS`),重跑后膝方向检查通过。
  4. **UMR 结果缓存**:pipeline 以输出路径为缓存键,重新生成源 clip 后必须
     `--force-retarget`,否则会静默复用旧结果(曾导致一次误判)。
  5. 新动作模板为 `smplx_neutral`(betas=0),需要单独建立 correspondence;
     方案 §25 的 PICO adapter 也是 neutral betas,因此这正是线上路径所需的模板。

- **下一阶段**: commit 08 的 MuJoCo 验证已完成;进入 commit 16/17/18
  (replay → ZMQ → StreamingReferenceProvider → MuJoCo)

---

## 阶段 16/17/18 + M6 — streaming 参考链路

- **commit**:
  - `sonic_for_a3` `301d4f1`(ReferenceProvider 抽象)+ `db3110e`(joint order 校验)
  - bridge `2019bda`(回归测试)、`e4176eb`(replay/streaming 工具 + 修正)
- **执行命令**:
  ```bash
  # SONIC 侧新增 ReferenceProvider(csv / stream),并用回归测试确认 csv 路径不变
  pytest tests/test_reference_provider.py
  # 离线轨迹 → ZMQ → StreamingReferenceProvider → A3-fast → MuJoCo
  python tools/run_a3_streaming.py --csv <clip>.csv --policy-steps N --duration T
  ```
- **结果**:
  - **§40 回归**:官方 sample CSV 的 86 帧,旧路径与 provider 路径的 encoder input
    (10×64)与 tokenizer terms 最大绝对差 **恰好 0.0**
  - **跨实现**:bridge 编码的包由 SONIC 侧 `StreamingReferenceProvider` 解码,数值一致
  - **M6(短测 300 步)**:`fall=false`、root z **1.0733 m**、RMSE(29) **0.05365** ——
    与 CSV 模式逐项相同(root z 1.0733 / RMSE 0.05365)
  - **M6(耐久 10800 步 = 216 s 参考)**:`fall=false`、root z 1.0696、RMSE 0.156、
    publisher 7167 帧 @49.8 Hz、`rejected=0`
- **关键修复(重要)**:
  - **joint 顺序**:`A3_REFERENCE_V1` 的 `joint_pos/vel` 必须是 **encoder(URDF/IsaacLab,
    `dof_il`)顺序**,它是 CSV/MJCF policy 顺序的一个**真置换**(29 个同名关节,
    顺序完全不同)。第一版按 policy 顺序发布,policy 输出饱和到 `|action|=20`、
    机器人走飞并倒地。
    - `tools/inspect_a3_contract.py` 现在按名字从 URDF 推导 `il_joint_names` 与两个
      index map(BFS 顺序与 `load_urdf_actuated_joints` 完全一致)并断言互为逆置换
    - `transport/protocol.py` 上线前做置换,header 打 `joint_order: a3_il_v1` 并携带
      关节名;解码端校验并逆置换
    - SONIC 侧 provider 拒绝未打标签的包
  - `replay_reference.py` 实测稳定 **50.0 Hz**(500 帧 / 10 s)
- **已知问题(耐久测试中定位到的三个问题)**:
  1. `--batch-once` 下 MuJoCo 循环不受实时时钟约束(约 2.5× 实时),窗口会被复用 →
     SONIC 侧新增 `--realtime`,批量模式也按墙钟节拍。
  2. `replay_reference` 循环播放时重置 predictor:seq 归零(违反单调性)且会发出
     默认姿态占位窗口 → 现在**不重置**、并且不发布 DISCONNECTED 窗口。
     另外回放原来按发布频率推进(30 fps 素材被 1.67× 加速)→ 现在按 source 时间推进。
  3. 直接把 9 段动作首尾相接会造出参考姿态跳变,policy 会在跳变处摔倒
     —— 跳变参考本身就是故障输入。耐久必须用**连续**轨迹:
     `tools/make_endurance_clip.py` 对相邻 clip 做 20 帧 cross-fade
     (最大关节步进 0.087 rad、root 步进 0.0014 m),然后重复 8 次 = 240 s。
  4. 即使参考连续,**消费者比发布者慢**时 latest-only 流仍会把窗口向前跳若干 slot
     (实测 6 分钟 279 次跳变、最大 200 ms)→ policy 摔倒。修复:SONIC 侧
     `StreamingReferenceProvider` 保留上一拍窗口并朝最新窗口插值(四元数 nlerp),
     每拍最多前进一个 slot,并把 `interpolated / max_gap_ms` 计入统计
     (sonic_for_a3 `e735bba`)。
     最终耐久(18000 步 / 288 s 墙钟 / 360 s 参考):`fall=false`,root z 1.0729,
     RMSE 0.1262,publisher 49.0 Hz,receiver rejected=0。
- **下一阶段**: commit 19(实时 PICO → UMR online → MuJoCo,需要 PICO 硬件)、
  commit 20/21(benchmark / 故障注入)







---

## 阶段 M5 — recorded PICO → UMR → A3 → MuJoCo(4/4 PASS)

- **commit**: `5bd81fb`
- **执行命令**:
  ```bash
  python tools/run_m5_pico_chain.py \
      --clips stand,lift_left_foot,step_forward_slow,twist_torso_left
  ```
  (内部依次调用 `make_synthetic_pico_recording` → `convert_pico_recording` →
   `run_umr_a3_batch` → `run_a3_validation_suite`)
- **输入**: SMPL-X 验收动作 → 打包成 PICO 录制(局部关节 + 21×3 pose + body quat)
- **输出**: `logs/m5/m5_report.json`,每 clip 的 validation/export/mujoco 证据
- **结果**: **4/4 PASS**

  | clip | fall | root z | RMSE(29) |
  | --- | --- | --- | --- |
  | stand | false | 1.073 m | 0.0532 |
  | lift_left_foot | false | 1.073 | 0.0944 |
  | step_forward_slow | false | 1.070 | 0.0885 |
  | twist_torso_left | false | 1.073 | 0.0692 |

- **已知问题**:
  - `apps/record_pico.py --synthetic` 的内置示例是占位 body(单点),不能用于
    retarget;M5 用 `tools/make_synthetic_pico_recording.py`(经官方 rest skeleton FK)。
  - 真实 PICO 头显录制(Mode B/D)尚未执行,需要硬件;链路与坐标约定已验证。
- **下一阶段**: M7(实时 PICO,需硬件)、M9-M12(Orin / 真机)


---

## 阶段 13(part 1)/31/32/73/74/75 — 实时架构与标定(无硬件可验证部分)

- **commit**: `1e3a2a2` + 本条的 live-chain 工具
- **执行命令**:
  ```bash
  pytest tests/test_online_pipeline.py -q          # 13 项
  python -m a3_teleop_bridge.apps.retarget_live --source trajectory \
      --csv logs/a3_validation/endurance_loop.csv --duration 4
  python tools/run_live_chain.py --csv logs/a3_validation/endurance_loop.csv \
      --policy-steps 3000 --duration 90            # 实时架构全链路
  ```
- **输出**: `logs/live_chain/live_chain_report.json`、`generated/live_chain_report.json`
- **结果**: **ACCEPTED**
  ```text
  policy steps   3000(墙面 71.6 s,≈ 42 Hz 有效)
  fall           false
  root z         mean 1.0714 m
  published      3642 windows
  solver p50     0.023 ms;end-to-end p95 0.89 ms
  states         DISCONNECTED → CALIBRATION(未提供标定文件,故不进 TRACKING)
  ```
- **说明**:
  - 进程/线程划分与方案 §31 一致:receiver → solver → predictor → publisher,
    每一级 maxsize=1 的 latest-only 槽位(10k 次写入测试:dropped=9999,永不阻塞)。
  - §32 时间戳:每级都有 source/receive/solve/publish 时间与分项延迟统计。
  - §73 状态机由数据新鲜度驱动(测试覆盖 DISCONNECTED→CALIBRATION→TRACKING→HOLD→SAFE_STOP)。
  - §74/§75 标定:root yaw offset / pelvis height offset / body scale / 左右自检,
    可保存与加载;`apply_root` 只作用于 root pose(脚部运动仍由 UMR 约束处理)。
  - 本机用 **recorded trajectory** 代替 PICO+UMR(replay backend),因此验证的是
    Terminals C/D 的实时管线与网络;PICO 头显与 UMR online 求解仍是下一项。
- **已知问题(commit 13 剩余部分)**:
  - `UmrOnlineBackend` 的 **Stage I 装配**(`umr/umr_session.py::_prepare_geometry`)尚未实现,
    当前会抛 `UmrSessionError` 并给出所需调用序列,不会假装可用。
    已从 UMR 源码确认的装配顺序:
    `prepare_robot_xml` → `mj_model/MjData` → `build_robot_self_penetration_cache` /
    `build_ground_penetration_collision_cache` → `build_scalar_joint_limits` /
    `build_dof_max_dq_box` → `scalar_qpos_joint_addrs` →
    `source_template_vertices_joints_faces` → `load_slot_data`(robot/smpl) →
    `center_source_template` → `shared_smplx_correspondence.transfer_slots` →
    `bind_robot_slots` → `transport_tpose_robot_normals` → `bind_points_to_mesh`;
    每帧则调用 `bind_source_slots_with_normals` + `solve_frame_body_segment_qp`
    (warm start = 上一帧解)。
- **下一阶段**: 完成 Stage I 装配 → M7(真实 PICO 或 recorded PICO 驱动 online UMR)

## 阶段 13(part 2)— online UMR 求解打通 + 单帧性能(200 ms → 48 ms)

- **commit**: `92dee36`(本次);上游 UMR 未改动(仅使用其公开函数)
- **执行命令**:
  ```bash
  source env.sh                                     # 导出 SONIC_A3_ROOT 等
  UMR/.venv_umr/bin/python /tmp/test_online_session.py     # 逐帧求解探针
  UMR/.venv_umr/bin/python /tmp/profile_prepare.py         # cProfile 归因
  UMR/.venv_umr/bin/python /tmp/bench_smplx_device.py      # CPU vs CUDA 对比
  .venv_bridge/bin/python -m pytest tests -q               # 161 passed, 1 skipped
  ```
- **输入**: `recordings/m5_lift_left_foot`(recorded PICO,90 帧)+ A3 robot config
- **输出**: 无新文件;`UmrRetargetSession.initialize()` 现在返回可用的求解会话
- **结果**: **PASS — 在线求解可用,端到端 ≈ 20 Hz**
  ```text
  initialize     14.5 s(args 2.8 s / geometry 7.8 s / warmup 3.9 s)
  prepare/frame  15.5 ms(SMPL-X 前向 7.3 ms + 绑定 + ground contact)
  solve/frame    34.3 ms(iters=1,warm start)
  合计           49.7 ms → 20.1 Hz
  ```
- **两个根因(与 §0「不得猜测」一致,逐项从 UMR 源码对齐)**:
  1. `selected_slot_ids` 之前用的是全部 **4096** 个 correspondence slot;
     离线 pipeline 走的是 `sample_segment_slots`(按身体段、固定 seed 抽样,
     A3/SMPL-X 绑定为 **456** 个)。求解器里每个 Jacobian 循环因此长了约 9 倍。
     现在 online 与 offline 使用同一抽样逻辑,数值也可直接对比。
  2. `ground_contact_map_cost` / `ground_contact_anchor_cost` 已启用,但 online 传 `None`;
     离线传 `compute_source_slot_ground_contact` 的每帧结果。现按同一语义
     (取 z、snap 阈值内归零、weight = z − min z)在线计算。
- **额外发现(实测,不是猜的)**:单帧 SMPL-X 前向在 **CPU 上更快**
  (前向 7.3 ms vs 9.5 ms;整个 prepare 15.5 ms vs 37.8 ms)——batch=1 时
  CUDA 的 per-call launch/sync 摊不掉。因此 online 会话默认 `smplx_device="cpu"`,
  离线批量 pipeline 仍用配置里的设备。
- **已知问题**:
  - 20 Hz 低于方案 §45 的 25–50 Hz 目标。剩余时间在 `solve`(34 ms,单次
    Clarabel QP + 456 slot 的 Jacobian)与 SMPL-X 前向(7.3 ms);
    参考流是 latest-only + SONIC 侧有界插值,20 Hz 更新不会造成丢帧,
    但若要进一步提速,应在 UMR 侧批量求解(方案 §0 允许未来做,当前不改算法)。
  - 未跑真实 PICO(M7 剩余部分),M9–M12 仍需 Orin/A3 硬件。
- **下一阶段**: M7(真实 PICO)或继续 §46 的预测器/ V2 TensorRT parity(commit 27)


## 阶段 M7(除头显外全部打通)— recorded PICO → **online UMR** → A3-fast → MuJoCo

- **commit**: `7897e2c`(recording 源 + online backend 接入链路)、`3e42dbd`(标定修复)
- **执行命令**:
  ```bash
  # 新增:录制 PICO 直接驱动 online UMR(不需要头显)
  UMR/.venv_umr/bin/python tools/run_live_chain.py \
      --csv logs/a3_validation/endurance_loop.csv --csv-fps 30 \
      --recording recordings/m5_twist_torso_left \
      --policy-steps 1500 --duration 90 --port 15664 \
      --out-dir logs/live_chain_umr_calib
  ```
- **输入**: `recordings/m5_twist_torso_left`(recorded PICO,SMPL-X 真人体帧,循环播放)
  + SONIC 官方 `035_step200000` + A3-fast + MuJoCo
- **输出**: `logs/live_chain_umr_calib/{live_chain_report,pipeline_stats,metrics,timeseries}.json`
- **结果**: **ACCEPTED — M7 的整条架构已用真实 online UMR 求解器跑通**

  | 项 | 标定版(1500 步) | 长跑版(3000 步,无标定) |
  | --- | --- | --- |
  | fall | **false** | **false** |
  | root z mean | 1.0711 m | 1.0710 m |
  | all-29 RMSE | 0.177 rad | 0.178 rad |
  | 求解器 p50 / p95 / max | **37.5 / 63.6 / 202.7 ms** | 59.9 / 277 / 400 ms |
  | 发布窗口 | 843(输入 1934) | 1381(输入 7500) |
  | rejected | 0 | 0 |
  | 状态机 | DISCONNECTED → **TRACKING**(2 次转移) | CALIBRATION ↔ HOLD 抖动 |

  SONIC 侧 `stream_stats`(标定版):received 707、dropped 26、jumps 65、
  max_gap 2961 ms、interpolated 1499、rejected 0 —— 即参考流按 20–25 Hz 更新、
  策略按 50 Hz 消费时,差值全部由既有有界插值补齐,机器人未倒。
- **修掉的两个真实缺陷**:
  1. `--source recording` **从来自动标定不生效**(条件只写了 `trajectory`),
     状态机因为没有 calibration 永远停在 CALIBRATION。修复后
     `auto-calibrated: yaw_off=+0.000 rad scale=0.670 left_right_ok=True`,
     状态机进入 **TRACKING**。
  2. `run_live_chain.py` 打印的是 `--policy-steps` 请求值,而不是策略真正跑的步数:
     第一次用 `m5_twist_torso_left.csv`(约 3 s)当 motion 时只跑了 149 步却报 1500。
     现在从 `metrics.json` 读实际步数。
- **已知问题 / 下一步优化**:
  - 长跑版的求解器 p50 从 37.5 ms 涨到 59.9 ms、p95 到 277 ms:同一台机器上
    SONIC 策略(50 Hz,torch)与 UMR(SMPL-X + Clarabel,CPU)抢 CPU 所致,
    属于宿主资源竞争,不是算法回归。方案 §22–24 的部署形态本来就是
    UMR/参考流放 **Orin**、策略放 4090,迁移后可消除该竞争(M9 待硬件)。
  - 记录片段只有约 3 s,循环播放会在片段边界产生参考跳变(stream jumps 65);
    真机 M7 用真实连续数据不会有这个问题。
  - 尚未执行:真实 PICO 4 头显(M7 最后一步)、M9–M12(需 Orin/A3 硬件)。

## 阶段 78/79/86 — 参考流录制与一级 replay

- **commit**: `2910a6c`
- **执行命令**:
  ```bash
  python tools/record_reference.py --endpoint tcp://127.0.0.1:15680 \
      --duration 12 --out examples/reference_recording/stand_12s
  python -m a3_teleop_bridge.apps.replay_reference \
      --windows examples/reference_recording/stand_12s --publish-hz 50 --loop \
      --endpoint tcp://127.0.0.1:15682 --duration 60
  pytest tests integration -q            # 167 passed, 1 skipped
  ```
- **输入**: `logs/a3_validation/stand.csv` 经 `replay_reference` 发布(A3_REFERENCE_V1)
- **输出**: `examples/reference_recording/stand_12s/{windows.npz,metadata.json}`(129 KB)
- **结果**: **PASS**
  ```text
  录制      601 窗口,seq 187..787,实测 49.999 Hz,rejected=0
  重播      SONIC sim2sim 消费录制流:149 步,fall=false,roll/pitch max 1.54°
  测试      167 passed, 1 skipped(新增 2 项:格式逐字段往返、seq 单调)
  ```
- **说明**:
  - 重播**不改动 payload**,只把 `timestamp_ns` 重新盖到当前时钟上——否则消费侧的
    freshness watchdog 会把窗口判成 stale;`seq` 保持录制时的单调序列。
  - 这就是方案 §79 要求的“replay 一级功能”:真机出问题时,不需要机器人、不需要
    PICO、不需要 UMR,直接重播参考流即可复现。
  - `examples/README.md` 汇总了三类示例产物(PICO 录制 / UMR→A3 结果与 CSV /
    参考流录制)及各自的产生命令。
- **下一阶段**: 剩余可做项已不多;M7 真实头显与 M9–M12 真机/Orin 需硬件

## 阶段 49 / M8 — 10 分钟连续稳定性验收(方案 §49/§84)

- **commit**: 本次记录(代码即 `2910a6c` 之后的当前 HEAD;无新代码改动)
- **执行命令**:
  ```bash
  # 12 分钟动作(12 段验收动作拼接,20 帧交叉淡化,复用 make_endurance_clip.py)
  python tools/make_endurance_clip.py --repeats 40 --blend-frames 20 \
      --out ../logs/a3_validation/endurance_12min.csv

  python tools/run_a3_streaming.py --csv ../logs/a3_validation/endurance_12min.csv \
      --policy-steps 36000 --duration 1100 --port 15692 \
      --out-dir ../logs/m8_ten_minutes_long
  ```
- **输入**: `logs/a3_validation/endurance_12min.csv`(36000 帧 @30 fps)
- **输出**: `logs/m8_ten_minutes_long/{m6_report,metrics,timeseries,sim2sim.log,publisher.log}`
- **结果**: **ACCEPTED — 方案 §49「至少 10 分钟」达成**
  ```text
  策略步数        36000(= 720 s 策略时间 @50 Hz)
  墙钟时长        924.0 s(15.4 分钟,连续运行)
  fall            false
  root z mean     1.0731 m
  roll/pitch max  2.611°
  all-29 RMSE     0.1202 rad(历次最好)
  publisher       44353 窗口 / 920.6 s = 48.18 Hz,rejected=0
  receiver        received 35884,rejected 0,dropped 8304(latest-only 设计)
  消费侧          interpolated 35999(每步都有参考),无 NaN、无积压、无 root 跳变
  ```
- **说明**:
  - 这条是 **M6/M8 的纯 streaming 链路**(trajectory → ZMQ → StreamingReferenceProvider
    → A3-fast → MuJoCo),不含 PICO/UMR,因此可以在无硬件条件下把「长时间稳定性」
    这一项彻底做完;online UMR 版本的 60 s 长跑已在 M7 条目记录。
  - 交叉淡化由 `tools/make_endurance_clip.py` 生成:相邻片段 20 帧混合,
    最大关节步进 0.0874 rad、root 步进 0.0014 m,因此 12 段拼接不构成参考跳变。
  - 之前的 30000 步那次实际只跑到 motion 结尾(12000 帧 ≈ 240 s),
    说明「请求步数 ≠ 实际步数」;`run_live_chain.py` 已改为从 `metrics.json`
    读实际步数,本次也用 `num_policy_steps` 复核。
- **下一阶段**: 无硬件可做的验收项已全部完成;M7 真实头显、M9–M12(Orin/A3 真机)待硬件

## 阶段 交付 — 代码推送 GitHub(已完成)

- **执行命令**:
  ```bash
  ssh -T git@github.com                      # Hi Restar7! You've successfully authenticated
  git -C sonic_for_a3 push git@github.com:Restar7/sonic_for_a3.git feat/a3-streaming-reference
  git -C a3_teleop_bridge remote add origin git@github.com:Restar7/a3_teleop_bridge.git
  git -C a3_teleop_bridge push -u origin main
  ```
- **结果**: **PASS**
  ```text
  sonic_for_a3  feat/a3-streaming-reference  1fcdf7f  (LFS 108 objects / 73 MB)
  a3_teleop_bridge  main                     109d8b0  (新仓库首推)
  sonic_for_a3  main 未改动                  fe6868b
  UMR           feat/a3-online-retarget     9341764 → git@github.com:Restar7/UMR.git
  ```
- **验证**: 从 GitHub 全新 clone 三个仓库(UMR 用 `--branch feat/a3-online-retarget`),确认 `docs/a3_teleop_deployment.md`、
  `docs/{DELIVERY,DEPLOY_ORIN,A3_ONBOARD}.md`、`README.md`(含公钥)、`scripts/` 8 个脚本都在。
- **交付物**: `dist/a3_teleop_orin_20260927_121056.tar.gz`(14 MB,含 MANIFEST 记三仓库 SHA)、
  `dist/*.bundle`。
- **下一阶段**: Orin/M9–M12 需硬件;上机时按 `docs/DEPLOY_ORIN.md` → `docs/A3_ONBOARD.md` 执行。

## 阶段 59/60 — A3 侧参考→通道桥(C++ 实现 + 单测)

- **commit**: 本条目对应 sonic_for_a3 上的新提交(C++ 适配层)
- **执行命令**:
  ```bash
  python tools/make_cpp_joint_order.py                    # 生成 il<->policy 置换表
  bash tools/run_cpp_teleop_command_test.sh               # 24 项检查
  bash tools/run_cpp_reference_test.sh                    # 解码侧 16 项检查(回归)
  ```
- **输入**: `generated/a3_contract.json`(policy/il 名字与置换)、`A3_REFERENCE_V1` 窗口
- **输出**: `include/a3_deploy/a3_teleop_joint_order.hpp`(生成)、
  `include/a3_deploy/a3_teleop_command_source.hpp`、
  `src/a3_deploy/a3_teleop_command_source.cpp`、
  `unit_tests/test_a3_teleop_command_source.cpp`
- **结果**: **PASS(24/24)**
  ```text
  policy 顺序与官方 ConvertTaWholeBodyCommand 注释逐名字一致
  置换表为精确互逆;wire 每个索引都落到官方转换器读取的 policy 索引
  速度按 leg(12)+waist(3)+head(1)+arm(14) 布局往返
  NaN / invalid / 越限 / 帧索引越界 → 拒绝
  20 Hz 窗口拆成 10 条命令,时间戳严格单调、重叠帧不重发
  断流 50 ms → ShouldHold;相邻 tick 跳变 → 拒绝
  ```
- **重要发现(方案 §0 第 5 条的典型例子)**:官方 `policy_parameters.hpp` 里的
  `isaaclab_to_mujoco` / `mujoco_to_isaaclab` 是 **G1 约定**的顺序,**不是**
  `/ta/whole_body_command` 的顺序(后者由 `ConvertTaWholeBodyCommand` 注释给出:
  waist/左臂/右臂/左腿/右腿)。两者不同,若按 G1 表置换会得到**静默错误**的参考。
  本实现只使用由 contract 生成的置换表,并在单测里逐名字核对。
- **剩余**:机器人侧约 20 行 AimRT publish 接线(需要 TA proto,本机无 AimRT 无法编译)
- **下一阶段**: Orin/M9–M12 硬件到位后按 `GO_LIVE_CHECKLIST.md` 执行

## 阶段 60/61(续)— AimRT 发布接线 + 真实 protobuf 校验(不等硬件)

- **问题**:上一轮把"A3 侧接线"列为"要等 Orin"。其实只有**跑**在机器人上才需要硬件,
  **写和验证**都可以现在做 —— 本轮补完,并用真实 `protoc` 抓出一个真错误。
- **执行命令**:
  ```bash
  # 真实 protobuf 生成 + 编译(无需 AimRT/ROS/ORT)
  apt-get install -y protobuf-compiler libprotobuf-dev
  bash tools/run_cpp_channel_message_test.sh        # 19 项
  bash tools/run_cpp_teleop_command_test.sh         # 24 项(回归)
  bash tools/run_cpp_reference_test.sh              # 16 项(回归)
  # 关掉 CUDA 验证在线 UMR
  CUDA_VISIBLE_DEVICES="" UMR/.venv_umr/bin/python -c "<online session>"
  # aarch64 依赖可用性(直接查 PyPI,不需要 Orin)
  python3 - <<'PY'  (见本轮记录)
  ```
- **结果**: **PASS**
  ```text
  proto 级测试        19/19(真实 TaWholeBodyCommandChannel 序列化/反序列化 + 官方读法镜像)
  command source      24/24
  reference stream    16/16
  bridge python       167 passed / 1 skipped
  CUDA-free online    initialize OK;prepare 5.4–57.7 ms,solve 23.4–99.6 ms(无 CUDA)
  aarch64 wheel       numpy/scipy/pyzmq/PyYAML/msgpack 均有 cp310 aarch64 轮子;
                      torch 也有(PyPI 上 63 个 aarch64 轮子,含 cp310),但为 CPU 版
  ```
- **本轮抓到的真错误(值得记)**:
  1. 消息类型是 **`TaWholeBodyCommandChannel`**(`header` + `data` 两层),
     不是 `TaWholeBodyCommand`。手写 stub 编译发现不了,真实 `protoc` 一编就报
     `has no member named 'mutable_header'`。已改为填 channel 消息。
  2. 速度布局:官方 `ConvertTaWholeBodyCommand` 按**数组长度**分支 ——
     30 槽是 leg12+waist3+**head1**+arm14(arm 起点 16),31 槽是 leg12+waist3+**head2**+arm14
     (arm 起点 17,头部速度进 `head_dq`)。我们发 **31** 槽(与 proto 注释一致),
     测试镜像两个分支都覆盖。
- **不需要硬件的部分已全部完成**;剩下的只有"跑"和"测数字":
  ```text
  能在 4090 上做(已做)                       必须硬件
  ─────────────────────────────────────      ──────────────────────────────
  A3 侧 C++ 全链路 + 单测(24+19+16)          真实 PICO 头显(M7)
  在线 UMR 无 CUDA 运行验证                   Orin 实测延迟/内存(M9)
  aarch64 依赖可用性                          A3 MDU receive-only(M10)
  package_bundle / sync / preflight 脚本      悬吊 10 级(M11)
  x86 包构建?需要 ROS2 + ONNX Runtime(本机无,未装)  TensorRT/自由遥操(M12)
  rockchip 交叉编译?需要 Docker(本机无)
  ```
- **上机待确认(不要猜)**:proto 注释里手臂命名是 `elbow_pitch/elbow_yaw/wrist_pitch/wrist_yaw`,
  而 A3 MJCF 是 `elbow/wrist_roll/wrist_pitch/wrist_yaw`。官方转换器按**位置**拷贝,
  本实现同样按位置填;悬吊 2–4 级逐个关节确认映射即可,若错位只改生成器输入。
- **下一阶段**: 硬件到位后按 `GO_LIVE_CHECKLIST.md` 执行

## 阶段 M7(仿真侧)— PICO 遥操仿真的启动健壮性 + 步骤文档

- **执行命令**:
  ```bash
  tools/run_live_chain.py --pico --csv ../logs/a3_validation/stand/stand.csv \
      --policy-steps 400 --duration 90 --port 15674 --out-dir ../logs/sim_teleop/pico_smoke3
  pytest tests integration -q
  ```
- **结果**: **PASS(无头显时的失败模式已验证)**
  ```text
  没数据时:策略照常 50 Hz 运行、机器人站着(149 步 / fall=false / roll-pitch 1.54°)
  判定:NOT ACCEPTED,列出 3 条 problem(含 "is the PICO sender RUNNING ... A button")
  超时:30 s 后按原逻辑报错退出;--reference-startup-wait-s 0 可关闭等待
  ```
- **改了什么**:
  1. `sim2sim_a3_mujoco.py`:`--reference-source stream` 新增 `--reference-startup-wait-s`
     (默认 30 s);等待期把 motion 第一帧平铺成 10 帧作"站立占位"(与官方 C++
     `BuildDefaultStandTokenizerSlice` 同思路)。第一版只平铺 1 帧,触发
     `reference provider returned joint_pos (1,29), expected (10,29)` —— 同轮修掉。
  2. `reference_provider.py`:`StreamingReferenceProvider` 接受 `startup_window` /
     `startup_wait_s`,等待期打一条明确日志。
  3. `run_live_chain.py`:新增 `--pico` 模式(真头显 → online UMR → MuJoCo);失败时逐条打印 problem。
- **新增文档**: `docs/SIM_TELEOP.md`(仿真遥操三层验证 + 三终端命令 + 判据 + 排查表)
- **下一阶段**: 头显到位后按 `SIM_TELEOP.md` §2 跑 C 层;再按 `GO_LIVE_CHECKLIST.md` 上机

## 阶段 选型 — 路线 B:部署机定为 Orin

- **背景**:读官方 aimdk v3.2 文档后发现 A3 自带运控,存在「用官方运控做上肢遥操」的路线 A;
  本项目确认走**路线 B**(SONIC A3-fast 全身 policy 接管 29 关节)
- **决策**:机载部署机 = **Orin**;5060/4090 = 开发机(仿真、AimSim、交叉编译部署包)+ UMR 兜底
- **依据**:
  ```text
  路线 B 的 policy 跑在 A3 MDU(RKNN)上,部署机是纯 CPU 活
    实测:SMPL-X 单帧 7.3 ms(CPU)、online UMR 48 ms(关掉 CUDA 照跑)、predictor+发布 <1 ms
  遥操要的是可靠性:机载供电 + 机内直连 > PC + 线缆/Wi-Fi
  开发与部署分离:仿真/构建在 4090,Orin 只放验证过的运行时
  ```
- **风险与兜底**:Orin CPU 比 4090 弱,UMR 可能 <20 Hz。判据:Orin 上跑 20 s 录制回放看
  `solver_latency_ms.p50`;>100 ms 时把 UMR 留在 4090/5060,只把参考流过以太网发给 A3
  —— **切换代价是 `configs/network.yaml` 的一行 `bind_host`**(协议与 A3 侧不变,方案 §51)
- **文档**:新增 `docs/DEPLOY_TARGET_DECISION.md`;`MACHINE_ROLES.md`、`A3_OFFICIAL_INTERFACE.md` 已同步

## 阶段 移植性 — 路径解析跨机器化(仓库可移植)

- **触发**:在 5060 上 `python tools/make_smplx_validation_motions.py --out ...` 报找不到
  `/inspire/.../sonic_for_a3/gear_sonic/data/human/human_joints_info.npz`;显式传
  `--sonic-root` 就正常 —— 说明默认值来自**别的机器写进仓库的绝对路径**。
- **根因**:`generated/a3_contract.json` 是**提交进仓库**的,里面存了生成机器上的
  `sonic_root` 与 `assets.*` 绝对路径;`A3Contract.sonic_root / mjcf_path / urdf_path /
  sample_csv_path` 直接返回这些值。凡是以 contract 为默认值的工具,换台机器必挂。
  另外 `UMR/.npz` 里也存了生成机器上的 `robot_xml` 绝对路径,同样会在异地读取时失败。
- **修法(不替换字符串,而是把机制改对)**:
  1. 新增 `src/a3_teleop_bridge/paths.py` —— **唯一**的路径解析入口,优先级固定为
     `CLI 参数 → 环境变量 → contract/config 值(仅当在本机存在)→ 同级目录推断 → 明确报错`。
     没有任何"开发者固定路径"兜底。
  2. `contract.py`:`sonic_root` 改为**运行时解析**(带缓存);`assets` 改为**优先相对路径**,
     绝对路径只作兜底并会按记录的 root 重新锚定。
  3. `inspect_a3_contract.py`:写 contract 时 `sonic_root` 写 `null`(并给 hint),
     `assets.*` 一律写**相对 sonic_root** 的路径 —— 仓库里不再有机器路径。
  4. `extract_a3_joint_limits.py` / `build_a3_tpose.py`:来源信息改为相对路径。
  5. `umr/offline.py`:UMR 结果里的 `robot_xml` 缺失时,先按结果目录相对路径找,再把
     `sonic_for_a3/` 之后的部分**重新锚定到本机 checkout**,最后给出可操作的报错
     (提示设 `SONIC_A3_ROOT` / 用 `--force` 重新生成 / 显式传 `robot_xml=`)。
  6. 清掉代码里所有 `Path.home()/"a3_teleop_ws"` 之类的猜测:`umr_session.py`、
     `umr/backends.py`、`benchmark_latency.py`、`make_smplx_validation_motions.py` 等改用
     `paths.workspace_root()` / `resolve_umr_root()`。
  7. 已提交的产物与文档里的 `/inspire/...` 前缀统一改为 `$A3WS`(报告、PLAN、progress、README)。
- **验证(全部实测)**:
  ```text
  grep -RIn --exclude-dir=.git '<机器专属工作区前缀>' .   → 无输出(前缀本身不在文档里复现,避免自匹配)
  git grep 两个仓库的 tracked 文件                                            → 无机器路径
  pytest tests integration -q                                                  → 见下方全套结果
  新建“伪装的另一台机器” /tmp/foreignws/{a3_teleop_bridge,sonic_for_a3->symlink,UMR->symlink}
    不设 SONIC_A3_ROOT/UMR_ROOT/A3WS、不传 --sonic-root:
      make_smplx_validation_motions.py --out /tmp/foreign_out --duration 3      → wrote 9 clips / RESULT: OK
      make_synthetic_pico_recording.py --clip .../twist_torso_left.npz          → 90 帧 @30fps
      convert_pico_recording.py                                                 → 适配器 OK
      run_umr_a3_batch.py --out-dir /tmp/foreign_umr --force                     → cost mean 0.0063
      run_a3_validation_suite.py --only m5_twist_torso_left --skip-mujoco       → 1/1 clips PASS
  显式 --sonic-root 的旧用法仍可用                                             → RESULT: OK
  ```
- **新增测试**:`tests/test_paths.py`(12 项)——CLI 优先、env 压过 stale contract、
  contract 存在时可用、contract 缺失时 fallback、同级目录自动发现、找不到时报错可操作、
  源码里禁止机器路径(自检)、asset 重锚定、UMR robot_xml 重锚定、已提交 contract 不含机器路径。
- **注**:`*.floating_mjcf.xml` 是 UMR 的**运行产物**(被 gitignore),它记录的是**本机**路径,
  每次运行都会重写,不属于"仓库可移植"问题;本轮曾误删,已用 `run_umr_a3_batch.py --force` 重新生成。
- **下一阶段**: 5060 上 `git pull --ff-only` 后按同一命令复验(不带 `--sonic-root`)


---

## 阶段 M5b(5060 复验)— 验收动作集 9/9 复现 + `hip_roll_joint_mirror` 误杀修复

- **时间**: 2026-09-28(ThinkBook 5060,conda `a3_bridge` 解释器)
- **背景**: 5060 上用 `data/pico_smplx/all`(PICO 回环)重跑 UMR 得到
  `$A3WS/UMR/output/a3_pico_all/*.npz`,9 个 clip 里 5 个 FAIL,且**只**失败
  `hip_roll_joint_mirror` 一项;`finite / root_* / joint_limits / knee / velocity /
  acceleration / feet_*` 全部通过。

### 根因(实测,不是推断)

1. **静态中性偏置**:A3 中立姿态的 hip roll 本身不对称 ——
   `left_hip_roll = -0.04694 rad`,`right_hip_roll = -0.01795 rad`,**两侧同为负**。
   `m5_stand` clip 全程恒定在这两个值上(150 帧 range = 0.00000 rad),
   9 个 clip 的**首帧完全相同**(retarget pose-init)。
2. **判据用的是整段原始均值**:旧规则要求 `mean(left)` 与 `mean(right)` 符号相反。
   由于两侧中性都是负数,只要左侧偏离中性超过 `|0.05 − 0.04694| = 3 mrad`,
   显著性门槛就被左侧**静态偏置**吃满,然后拿一个被偏置主导的量去判"镜像"。
3. **9 个 clip 的动态行程本来就很小**:最大 hip roll 动态偏移 0.0712 rad(4.1°),
   其余 ≤ 0.035 rad;这些都是平衡补偿量级,轨迹本身无法区分
   "同向横移"与"镜像错误"。旧判据在这批数据上没有任何信息量,只有误杀。

证据:`logs/a3_validation_all_nomj/hip_roll_diagnostic.txt`(修复前存档)。

### 判据修改(改的是逻辑,不是阈值)

| | 旧 | 新 |
| --- | --- | --- |
| 参考量 | 整段**原始均值** | **参考帧相对**行程 `d(t) = q(t) − q(ref)`(默认 clip 首帧,可用 `--neutral-npz` 显式指定) |
| 显著性 | `max(|mean|) < 0.05` → n/a | `max(max|d|) < 0.05` → n/a(静态偏置不再占额度) |
| 硬失败条件 | 原始均值**同号**(任何幅度) | **两侧都**粗大扫掠(各 > 0.25 rad ≈ 14°)且**世界方向相同** → 关节符号/镜像约定被破坏 |
| 其余同向情况 | 直接 FAIL | **非致命诊断**(`report["diagnostics"]`),附 common-mode / scissor 分解 |

- 0.25 rad 不是"把 0.02 调大":它是一条**新的物理分界**(超出平衡补偿范围),
  而且判据要求**两侧同时**粗大才触发,单侧动作(抬单脚/抬单臂)永远不会误伤。
- 真正的符号交换仍会被抓住:1) 同向粗大扫掠命中该检查;2) A3 的 roll 限位本身是
  **镜像且强不对称**的(hip:左 `[-0.524,+1.606]` / 右 `[-1.606,+0.524]`),
  有实际幅度的符号交换会同时撞 `joint_limits`。回归测试两条都覆盖。
- **物理检查(限位/速度/加速度/足部/有限性/膝方向)全部保持 hard fail,一个都没删。**

### 修改文件

| 文件 | 变化 |
| --- | --- |
| `tools/validate_a3_motion.py` | mirror 判据改为参考帧相对 + 粗大同向扫描;新增 `--neutral-npz`;新增 `diagnostics`(非致命)与 `joint_excursion_rad` / `knee_excursion_rad` / `mirror_reference` |
| `tests/test_validate_a3_motion.py` | **新增**,15 项回归(见下) |
| `tools/run_live_chain.py` | sim 端解释器不再硬编码 `.venv_sim`,支持 `--sim-python` / `$PY_SIM`,与 `run_a3_baseline.py` 的 `find_python` 一致 |
| `scripts/check_orin_ready.sh` | 允许预导出的 `PY_BRIDGE`/`PY_UMR` 覆盖 venv 默认值;否则 interpreter 不存在时 3/5、4/5、5/5 三段会被**静默跳过** |

### 回归测试(`tests/test_validate_a3_motion.py`,15 项)

```text
静态中性偏置 + 正常动态            → PASS(旧规则在此 FAIL,即 m5_bend_knees 形状)
抬单脚 / 迈步这类非对称动作          → PASS(同向但不粗大 → 只出诊断)
肩部镜像抬手(正确反向)             → PASS
同向粗大 hip roll 0.40 rad         → FAIL(且此时 joint_limits 仍为 ok,证明该检查不可删)
肩部符号交换 0.60 rad              → FAIL(限位不越界,只有该检查能抓)
显式 --neutral-npz 参考            → 读到的参考值与行程随参考变化
NaN / 越限 / 反折膝 / root teleport / 速度尖峰 → 仍然 hard FAIL
诊断不掩盖物理失败                 → 同时报出 2 条 problem
9 个真实 npz 全通过                → PASS(数据缺失时自动 skip)
```

### 验收结果(全部实测)

```bash
# 1) 单元 + 集成
python -m pytest tests integration -q                    → 180 passed, 13 skipped

# 2) 数值 + CSV(--skip-mujoco)
python tools/run_a3_validation_suite.py \
  --data-dir $A3WS/UMR/output/a3_pico_all \
  --out-dir  $A3WS/logs/a3_validation_all_nomj --skip-mujoco   → 9/9 clips PASS

# 3) MuJoCo smoke(单 clip 100 步)
python tools/run_a3_validation_suite.py ... --only m5_twist_torso_left \
  --policy-steps 100 --out-dir $A3WS/logs/a3_validation_mujoco_smoke  → 1/1 PASS

# 4) MuJoCo 全量(9 clip,各 249 步)
python tools/run_a3_validation_suite.py \
  --data-dir $A3WS/UMR/output/a3_pico_all \
  --out-dir  $A3WS/logs/a3_validation_mujoco_full          → 9/9 PASS(67 s)
```

| clip | fall | steps | root z | roll/pitch max | RMSE(29) |
| --- | --- | --- | --- | --- | --- |
| m5_stand | false | 249 | 1.0727 | 1.55° | 0.0568 |
| m5_raise_left_arm | false | 249 | 1.0718 | 2.49° | 0.0893 |
| m5_raise_right_arm | false | 249 | 1.0677 | 3.59° | 0.1028 |
| m5_bend_knees | false | 249 | 1.0571 | 7.27° | 0.1436 |
| m5_twist_torso_left | false | 249 | 1.0729 | 1.55° | 0.0647 |
| m5_twist_torso_right | false | 249 | 1.0716 | 4.83° | 0.0665 |
| m5_lift_left_foot | false | 249 | 1.0697 | 7.41° | 0.1050 |
| m5_lift_right_foot | false | 249 | 1.0726 | 3.24° | 0.0986 |
| m5_step_forward_slow | false | 249 | 1.0692 | 4.72° | 0.0934 |

```bash
# 5) A3-fast 官方 baseline
python tools/run_a3_baseline.py --smoke                       → ACCEPTED(M1)
python tools/run_a3_baseline.py                               → 1652 步 @50 Hz,fall=false,ACCEPTED

# 6) 只读自检
bash scripts/check_orin_ready.sh                              → 16 ok, 0 failed
bash tools/run_cpp_teleop_command_test.sh                     → 25 项 PASS
bash tools/run_cpp_channel_message_test.sh                    → 19 项 PASS

# 7) online UMR(recorded PICO,硬件无关)
python -m a3_teleop_bridge.apps.retarget_live --source recording --backend umr-online \
  --recording $A3WS/recordings/all/m5_twist_torso_left --duration 8 --no-publish
  → frames in=240 solved=190 rejected=0,solver p50 40.37 ms / p95 69.91 ms(达标)

# 8) 完整 live chain(recorded PICO → online UMR → A3_REFERENCE_V1 → A3-fast/MuJoCo)
python tools/run_live_chain.py --recording $A3WS/recordings/all/m5_twist_torso_left \
  --csv $A3WS/logs/a3_validation_all_nomj/m5_twist_torso_left/m5_twist_torso_left.csv \
  --policy-steps 600 --duration 40 --port 15640
  → ACCEPTED:249 步,fall=false,root z 1.0686,published=857,solver p50 34.9 ms

# 9) 5 分钟连续发布(§12.1 的缩短版,无人值守)
python -m a3_teleop_bridge.apps.retarget_live --source recording --backend umr-online \
  --recording $A3WS/recordings/all/m5_twist_torso_left --loop --publish \
  --endpoint tcp://127.0.0.1:15642 --duration 300 --playback-hz 30 \
  --stats $A3WS/logs/orin_live/stats_5min.json
  → wall 300.35 s;frames in=9000 solved=7156 published=7156 rejected=0
     solver p50 36.6 / p95 71.8 / p99 87.8 ms(与 8 s 短跑的 40.4 / 69.9 持平,p95 不恶化)
     predictor p50 0.49 ms;end-to-end p50 38.4 / p95 75.3 ms
     state_history 只有 TRACKING/HOLD 交替,**无 SAFE_STOP**;queue_dropped human=1844
     (latest-only 的有界丢弃,state 队列 0)
```

### 本轮发现(未修改,留待决策)

- **A3 膝在整个验收集里基本不动**:9 个 clip 的 `left_knee_joint` 行程 = 0.00000 rad
  (恒定在 0.0;只有 `lift_right_foot` 到 +0.0139、`step_forward_slow` 到 +0.0218)。
  源 SMPL-X 的 `bend_knees` 膝关节屈曲约 36–43°,也就是说**膝屈曲没有被转移**,
  下蹲是由 ankle pitch(0.59 rad)+ hip pitch(0.43 rad)做出来的,root 高度只变化 6 mm。
  原因与 `NON_NEGATIVE_JOINTS`(膝下界裁到 0)一致:求解器偏好的方向在下界之外,
  于是**饱和在 0**。这正是 `mujoco_validation.md` 里"膝限位"那条修复的副作用。
  - 本轮**未改** UMR 配置/映射/求解器参数(按要求),只在 validator 里加了**非致命诊断**
    (`knee_excursion_rad` + "flexes by only … rad" 提示),让它在每次验收里可见。
  - 若要让 `bend_knees` 真的屈膝,需要单独一轮改 UMR 膝轴向/下界并把 9 个 clip 重跑
    (本批 npz 按约定复用,未重新生成)。
- `retarget_live` 在未 `source scripts/env_orin.sh` 时,`$SONIC_A3_ROOT` 未导出,
  UMR 会拿到字面量 `UMR/${SONIC_A3_ROOT}/...` 路径并以 `FileNotFoundError` 失败
  (runbook §15 已记录该现象;本轮踩到两次,仍建议以后加显式报错)。

- **下一阶段**: 无硬件可做的部分已全部完成;剩余为 PICO 头显 / AimSim / 真机
  (§7/§8/§11),需硬件在场。上机前按 `GO_LIVE_CHECKLIST.md` A→G 执行。

---

## 阶段 M5c — 膝屈曲缺陷:**已定位并修复**(UMR 膝姿态先验)

- **时间**: 2026-09-28(第二轮,用户选择方案 P1)
- **commit**: UMR `84d3650`(实现先验)+ `086d46c`(抽出共享 helper);本条记录

### 根因(在 UMR solver 内部实测定位,不是推断)

在 solver 里临时插桩(`A3_KNEE_DEBUG`,**已完全回退**,脚本与备份逐字节相同)后测到:

1. **代价函数本身就偏爱伸膝 → 不是优化器的问题。** 用 `joint_limits` 把膝钉死在不同角度、
   读求解器**自己报告**的 cost:膝 0.00→0.0192、0.15→0.0239、0.30→0.0296、0.45→0.0364、
   0.60→0.0443、0.90→0.0650,单调递增 ⇒ 膝=0 就是该目标的约束最优解。
2. **是谁把膝顶住:foot 的 point term。** `bend_knees` 最深帧(97 帧)扫左膝并按部位分解:
   Δ膝 +0.20 时总 cost 20.89→26.91,其中 **leftFoot 3.17→9.28**,而 leftLeg(小腿)
   反而 **1.83→1.72**(屈膝改善了小腿匹配)。
3. **为什么"屈膝反而让脚更远"。** 该帧目标左脚中心在机器人脚**上方 90 mm、后方 87 mm**;
   目标到髋距离 0.7867 m 比当前完全伸展的腿 0.8286 m **短 4.2 cm**(目标确实要求折叠变短)。
   但求解器收敛的姿态是"整条腿向后扫 ~22° + 膝伸直";在该姿态下屈膝把脚扫向**前下方**,
   正好背离"后上方"的目标 ⇒ point 目标的最优解就是"hip pitch 扫腿 + 膝停在伸展限位"。
   无约束 IK 能精确命中但需膝 −0.64 rad(超伸,模型不允许)+ `hip_yaw` +0.59 rad(34° 扭转)。

**结论:膝不是坏了,是"没人管"——目标函数里完全没有膝姿态信息**;UMR 里唯一的关节空间
先验开关 `--joint_map_cost` 是 **dead code**(声明了、代码从未使用)。correspondence 本身正常,
**不需要重建**(修正了上一轮"要重建 500 epoch"的猜测)。

### 修复(UMR `solver.joint_map_cost`,权重 600)

逐帧测源骨架膝屈曲(大腿-小腿夹角),并在模型上**实测**机器人自己的
`内角 = 180.00 − 57.30 × 膝角`(度)线性映射,把源膝角映射为机器人膝角目标,
每膝加一行 `sqrt(cost)·(q_knee − target)` 单位残差。权重 0 时行为与修复前完全一致。

**标定依据**:A3 自己的 MJCF **keyframe(标称站姿)膝角 = +0.2515 rad(14.4°)**,
源 `stand` 的内角 165.3° vs keyframe 165.59° —— 几乎相同。修复前的 0.0000 意味着
**腿比机器人自己的标称站姿还直**。

| clip | 膝角修前 | 膝角修后 | 源目标 | RMSE vs 源 | corr |
| --- | --- | --- | --- | --- | --- |
| stand | [0.00, 0.00] | [0.24, 0.24] | [0.26, 0.26] | 0.012 | — |
| bend_knees | [0.00, 0.00] | [0.24, 0.80] | [0.25, 0.88] | 0.060 | **+1.00** |
| lift_left_foot | [0.00, 0.00] | [0.24, 0.81] | [0.25, 0.88] | 0.055 | **+1.00** |
| step_forward_slow | [0.00, 0.00] | [0.24, 0.47] | [0.25, 0.51] | 0.035 | **+1.00** |

### 验收(全部复跑)

```bash
python tools/run_a3_validation_suite.py --data-dir $A3WS/UMR/output/a3_pico_all \
  --out-dir $A3WS/logs/a3_validation_all_nomj --skip-mujoco            → 9/9 PASS
... --out-dir $A3WS/logs/a3_validation_mujoco_smoke --only m5_twist_torso_left --policy-steps 100
                                                                       → 1/1 PASS
... --out-dir $A3WS/logs/a3_validation_mujoco_full                     → 9/9 PASS(全 fall=false)
python -m pytest tests integration -q                                  → 181 passed, 13 skipped
online UMR(recorded PICO)  rejected=0, solver p50 42.8 / p95 75.6 ms
tools/run_live_chain.py --recording ...  → ACCEPTED, published 795, fall=false, p50 36.0 ms
```

MuJoCo 逐 clip 对比(修前 → 修后 RMSE):

| clip | 修前 | 修后 | root z |
| --- | --- | --- | --- |
| bend_knees | 0.1436 | **0.1219** | 1.0571 → **1.0044**(真的蹲下去了;roll/pitch 7.27°→3.46°) |
| stand | 0.0568 | **0.0489** | 1.0727 → 1.0703 |
| twist_torso_left | 0.0647 | **0.0526** | |
| twist_torso_right | 0.0665 | **0.0538** | |
| raise_left_arm | 0.0893 | **0.0720** | |
| raise_right_arm | 0.1028 | **0.0837** | |
| lift_right_foot | 0.0986 | 0.1066 | |
| step_forward_slow | 0.0934 | 0.1036 | |
| lift_left_foot | 0.1050 | 0.1383 | |

5 个改善、4 个变差(最大 +0.033)、**无一摔倒**。

### 过程中处理的两个坑

1. **在线路径必须同步**:online session 有自己的逐帧求解循环,最初没接上先验 —— 会导致
   "离线数据弯膝、现场遥操仍塌膝"。已把 `knee_posture_targets` 拆成
   `source_knee_interior_deg` + `robot_knee_interior_calibration` 供两边共用,
   并在 `umr_session.py` 接上;实测 online 膝角 0.244–0.274 rad(与离线 0.24 一致)。
   延迟基本不变(p50 42.8 vs 修前 40.4 ms)。
2. **`*.floating_mjcf.xml` 是每个 clip 共享、每次 retarget 覆写的产物**:探针曾把它覆盖,
   导致 1 个 clip 读不出来。已用原配置重新生成并确认 npz **逐字节相同**。

### 回归防护

`tests/test_validate_a3_motion.py::test_bend_knees_actually_flexes_the_knee`
—— 断言 `bend_knees` 膝行程 > 0.40 rad、中立位 ≈0.2515 rad(与 A3 keyframe 一致)、
下蹲峰值 > 0.70 rad。若先验失效,这个测试会红,而不是让验收静默通过。

- **下一阶段**: 无硬件可做部分已全部完成;剩余 PICO 头显 / AimSim / 真机(§7/§8/§11)。

---

## 阶段 M5c — 吞吐瓶颈定位 + 下半身纳入验收(10/10)

用户提出的两个问题:**solver 只有 ~21 Hz**、**不能抬腿/深蹲/跳跃**。

### 过程:两次被自己的假设带偏

1. `cProfile` 显示 `solve_frame_body_segment_qp` 占 74% → 怀疑 Jacobian 组装
   (每帧 912 次 `mj_jac`)。实现了"每个 body 一次 `mj_jac` + 解析推导其余点"的版本,
   公式精确(3.3e-16),但 **A/B 实测比原版慢 2 倍**(1.437 vs 0.709 ms):
   `mj_jac` 一个只要 1.7 µs,numpy 构造 skew 矩阵反而更贵。**已丢弃。**
2. `cProfile` 给出 35 ms/帧,据此认为"算法是瓶颈"。改用**手工计时**后是 **21.3 ms/帧** ——
   cProfile 会把 Python 密集代码放大 1.6 倍。**之前的 35 ms 是测量假象。**

### 真因:torch 线程数(单变量,交错 A/B 确认)

| torch 线程 | p50/帧 | 速率 |
| --- | --- | --- |
| **1** | **25.8 ms** | **38.8 Hz** |
| 2 | 47.9 ms | 20.9 Hz |
| 4 | 75.5 ms | 13.3 Hz |
| 16(默认) | 133.6 ms | 7.5 Hz |

SMPL-X LBS 的矩阵太小,OpenMP 屏障同步开销远超算术收益。现场 19.3 Hz 对应"有效 4 线程"。

**修**:`umr_session.configure_torch_threads()`,在线路径默认 1 线程;离线批处理**不动**,
已验收 npz 保持有效。`A3_TORCH_THREADS` 可覆盖。
live 链路 A/B/复测:`off` 16.0/16.5 Hz → `1` **19.5 Hz**;进程内 7.5 → 38.8 Hz。
另用交错测量确认 **pipeline 线程结构本身零开销**(31.4 / 31.3 / 31.0 ms)。

### 下半身:骨盆被写成常量

`make_smplx_validation_motions.py` 的 `trans[:, 2] = root_height` 是常量,所以
`data/smplx_validation/*.npz` 与每条录制的 `trans[:,2]` 都同一位数。屈膝只能把脚踩穿地面,
骨盆不动 → 深蹲/跳跃/重心转移在构造上不可能。修复前 `m5_bend_knees` 屈膝 47°、骨盆落差
**9 mm**;`stand`/`raise_*`/`twist_*` 全 **0.000 m**。

**修**:`support_anchored_root()` —— 竖直让最低的脚保持站姿地面高度,水平让支撑脚留在原地
(软加权,换脚不瞬移)。新增 `m5_squat_deep`,对齐官方 `043`:膝 126.5°(官方 131.3°)、
hip_pitch −105.7°(官方 −102.6°)、骨盆落差 **0.523 m**(官方 **0.525 m**)。
髋角初值 1.85 rad 会顶到 A3 下限 −144°,降到 1.15 rad 后进入包络。

**验收 10/10 PASS(数值 + CSV + 全量 MuJoCo)**,深蹲 `fall=false`、
机器人 root 高度均值 **1.068 → 0.771 m**,倾角 35.8°、RMSE 0.151。

### 跳跃:未解决,且不是链路问题

官方 20 条参考 `root_translateZ`(cm):站立 ≈106,**最高 110.0**(举手动作),最低 53.8。
**没有一条有飞行阶段**,仓库也无 jump/hop/leap。策略**从未在腾空下被验证**,参考格式也没有
接触标志位。参考侧现在能表达(蹬伸抬起骨盆),但策略侧无依据 —— 必须先仿真验证落地。

### 新增回归防护

`test_all_ten_acceptance_clips_pass` · `test_the_set_contains_a_real_squat` ·
`test_online_session_caps_torch_threads` ·
`test_generated_motions_derive_the_root_from_the_pose`

- **下一阶段**: 跳跃需先仿真验证腾空与落地;其余无硬件可做部分已完成。

### M5c 补充:live 路径的深蹲会摔(以及一个把它藏起来的测试工具缺陷)

把深蹲接到 live 链路复测(假发送端 + 真 sim2sim,串行、逐条确认发送端绑定):

| 片段 | 离线验收 | live 链路 | live 机器人 root |
| --- | --- | --- | --- |
| m5_stand | fall=false | ACCEPTED | 1.061 m |
| m5_bend_knees(浅蹲) | fall=false | ACCEPTED | 1.000 m |
| m5_squat_deep(深蹲 52 cm) | fall=false | **FAILED(fall at tick 109, anchor_err 1.5 m)** | 0.482 m |

live 是 warm-start + `iters=1`,参考流有缺口(`max_gap_ms` 1594 ms、`jumps=34`),
深蹲在离线侧本就只是勉强站住(倾角 35.8°)。**live 已验证深度到浅蹲为止。**

**测试工具缺陷(同一个问题被藏住的原因)**:`fake_pico_sender.py` 默认**不发
`root_translation`**,帮助文字还写着"真发送端也不发" —— 那是过期描述,上一轮已让真发送端
发布 root。因此订阅端一直合成恒定站高骨盆,深蹲/迈步的骨盆运动在桌面测试里被静默丢掉:
**不带 root 不是"更小的测试",而是另一个测试。** 已改为默认发 root(`--no-root` 显式测兜底)。

排查中还踩了两个测量陷阱(记录以免重犯):
1. **`pkill -f` 会匹配到自己的 shell** —— 连续两次把调用它的命令行杀掉。改用
   `pgrep -f "[f]ake_pico_sender"` 或按 PID。
2. **发送端 stdout 被块缓冲**,日志里看不到 `bound tcp`,导致误判"绑定失败"。加 `-u`。
3. 不串行化时,前一个发送端仍占着 5556,后一个**静默失败**,于是三次"不同片段"的测试
   实际跑的是同一路输入 —— 三条 root_z 完全相同(1.060918091049555)才暴露出来。

### M5d — 比线程数更狠的一层:14 个核在空转

`torch.set_num_threads(1)` 在 **import 之后**才生效,而 torch 的 OpenMP 池在 **import 时**
就按整机核数建好了;限制线程数**不会缩小已存在的池子**,空闲 worker 在并行区之间忙等自旋。

单变量 A/B(同片段同机器):

| | bridge 线程数 | bridge CPU | live 速率 |
| --- | --- | --- | --- |
| 修复前 | **35** | **1453% 单核(≈14.5 核)** | 26.5 Hz(丢 135 帧) |
| 修复后 | **5** | **50% 单核** | **30.0 Hz(丢 0 帧,受 30 Hz 输入限速)** |

**CPU 降 29 倍。** bridge 实际工作量不到半个核 → ~14 个核在做屏障等待,而这些核本该给
旁边的策略(sim2sim 53 线程)。这也解释了 live 链路为何一直慢于进程内基准。

**修**:`run_live_chain.py:bridge_environment()` 在启动 bridge 时注入 `OMP_NUM_THREADS` /
`MKL_NUM_THREADS` / `OPENBLAS_NUM_THREADS` / `NUMEXPR_NUM_THREADS` = 1。**只作用于 bridge**,
sim2sim 的策略确实吃线程,不碰。`A3_OMP_THREADS` 可覆盖(`off` 还原)。

**顺带测清"关什么能更快"**(用户提问):

| 手段 | 实测 |
| --- | --- |
| `OMP_NUM_THREADS=1` | CPU 1453%→50%,26.5→30.0 Hz(**已内置**) |
| 关桌面程序 | 负载 ~22→~6,solver 45→26 ms(**外部最大因素**) |
| `--no-viewer` | bridge 54.4%→70.2%(±16% 单核);两次都 30 Hz 丢 0 帧 → **影响很小** |
| `--dump-frames` | **7.4 µs/帧 = 0.22 ms/s** → **不用关** |
| sim2sim | 53 线程但只吃 **0.70 核**(策略在 CUDA) → 不是瓶颈 |

**排查中的三个测量陷阱**(都会给出错误结论,记录以免重犯):

1. `pgrep -f` / `pkill -f` **匹配到自己的 shell** —— 连续三次把调用它的命令行杀掉,
   要写成 `[f]ake_pico_sender` 或按 PID。
2. **发送端 stdout 块缓冲**,日志里看不到 `bound tcp`,误判"绑定失败" —— 加 `-u`。
3. 不串行化时前一个发送端仍占 5556,后一个**静默失败**,于是"三个不同片段"实际跑同一路
   输入(三条 root_z 完全相同才暴露);必须逐条确认绑定成功再跑。

### M5e — 源端频率回到 50 Hz

用户提问:"现在是不是可以切回 50 Hz 了"。

**先分清楚三个频率**(混在一起会得出错误结论):

| 环节 | 是什么 | 现状 |
| --- | --- | --- |
| ① 源 PICO → 发送端 | 头显采样率,`--target_fps` 节流 | `--pico-fps` 默认 **30 → 50** |
| ② 求解 在线 UMR | 每个源帧解一次 | **p50 16.0 / p90 16.8 / p99 18.0 ms → 上限 62.6 Hz** |
| ③ 发布 → A3_REFERENCE_V1 | **等于 ②** | = 求解频率,**不是**固定 50 |

**③ 是个容易误解的点**:`configs/network.yaml` 写着 `reference_publish: 50`,但 predictor
线程是被**求解帧驱动**的(`state_slot.get()`),没有自己的 50 Hz 定时器。实测印证:
`frames_solved: 911, frames_published: 910`。真正跑 50 Hz 的是**机器人侧策略**,它把收到的
10 帧窗口(`dt=0.02`)插值到自己的 50 Hz(`interpolated=248`)。
**所以提高源端频率不会改变机器人收到的命令频率**,改变的是"被求解的采样点更新更细"。

**实测(两层线程修复后,假发送端 + 真 sim2sim):**

| 输入 | 求解速率 | 丢弃 | bridge CPU | fall |
| --- | --- | --- | --- | --- |
| 30 Hz | 30.0 Hz | 1(0%) | 56% 单核 | False |
| **50 Hz** | **50.0 Hz** | **1(0%)** | 87% 单核 | False |
| 60 Hz | 59.1 Hz | 30(1%) | 102% 单核 | False |

50 Hz 全额吃下、0 丢帧;60 Hz 是当前上限(掉 1%)。**`PICO_FPS` 默认改回 50。**

**没改 `hold_after_ms`(仍是 150 ms)**,故意的:它管的是"头显/网络真卡住"这类事故,
不是频率不匹配;而 HOLD 帧会清空消费者待处理窗口,阈值调紧反而让"参考被重置"更频繁。

#### 附带发现:OMP 修复还让求解本身变快了

`OMP_NUM_THREADS=1` 不只是省 CPU,求解从 ~26 ms 降到 **p50 16.0 ms**——
之前"torch 限 1 线程 → 25.8 ms"是在线程池仍空转自旋的情况下测的。

### M5f — 真头显实测:走路困难 + "随时在初始化"的根因

用户报告:手臂没问题了,但**走路很困难**,而且**感觉随时都在初始化**。

分析 `logs/sim_teleop/20260929_110418/`(真头显,109 秒,50 Hz)。

#### 先排除的(全部正常)

| 项 | 实测 |
| --- | --- |
| 源→求解 | 收到 49.4 Hz,求解 49.3 Hz,**5378 帧只丢 2 帧** |
| 状态机 | `DISCONNECTED → CALIBRATION → TRACKING`,**零次 HOLD**(109 秒只切 2 次) |
| 发送端 | 49.6~49.8 fps,`skipped=0`,**从未暂停**(不是 A 键) |
| 源→参考保真度 | 膝盖 1.01 倍,**1:1 忠实** |
| 头显追踪连续性 | 源信号帧间跳变(膝>20°/腿长>0.08m/髋>0.35rad):**0 次** |

#### 根因:头显的下肢幅度只有真实走路的约 1/3

100 秒走路段从 `live_frames.jsonl` 量出:

- **抬腿 70 次**(左 36 + 右 34,≈0.7 步/秒)—— **步频正常,腿确实在动**
- 但屈膝 p50 只有 **17.4°/12.0°**,p90 34.6°/36.9°,p99 75.5°/68.4°
- 真实走路需要每步 **66°(001_walk)/ 79°(050_march)**
- 腿长中位 0.780 m(完全伸直),95% 的帧屈膝 ≤ ~44°

参考忠实复现了这个"小幅度蹭",所以机器人**永远迈不出一步**。

"随时都在初始化"是同一件事的后果:操作者真的走开了,机器人跟不动,`root_err`
反复冲高再弹回(实测 0.035→0.447→0.056→0.464→0.050→0.420→0.044),
看起来就像在反复重置。对照假发送端同一段:`root_err` 稳定 **0.041**。

#### 排查中我自己犯的两个错(记录以免重犯)

1. **按"每秒中位数"采样,得出"膝盖只动 0.14 rad"的错误结论。** 走路的屈膝是快速
   往返,秒级中位数把峰值全洗掉了 —— 真实行程是 **106°**。
2. **误判仿真"没接上实时流"。** 看到 `ref` 按 249(=CSV 帧数)循环,就断定仿真在用
   自己的 CSV。做对照实验(假发送端 + 窗口模式)后发现:`ref` 循环 **191/191 同样成立**,
   `[reference-stream]` 统计行同样是 0 —— 那是**窗口模式的正常显示行为**,统计行只在
   正常退出时打印。**结论作废。**

#### 新增工具与文档

- `tools/check_pico_trackers.py` —— 查配了几颗动捕 tracker + 逐关节测幅度是不是
  "walk-grade"。**实测 `motion trackers paired = 0`**,但当时头显没在推流
  (`body data available: False`),所以这个 0 不能定论,必须在推流时重跑。
- runbook §17.7.1(实测数据)/ §17.7.2(一条命令分辨"缺动捕"还是"设置问题")
- 回归防护:`test_pico_tracker_probe_is_documented_and_runnable`

### M5g — 「人走开了机器人推不动」的真正原因:策略观测里没有位置

用户确认腿部 tracker 正常,并指出真问题是**人走开了、机器人推不动**。
我上一节"头显幅度不足"的判断需要修正 —— 输入不是瓶颈。

#### 拿官方参考直接测(绕开一切链路)

`tools/check_walk_following.py`,1200 步 = 24 s:

| 参考 | 参考走 | 机器人走 | 比例 | fall |
| --- | --- | --- | --- | --- |
| `001_walk_front_slow` | 1.085 m | 0.546 m | **50%** | False |
| 同一条,root 位移 ×2 | 2.170 m | **0.452 m** | 21% | False |
| `009_walk_left_fast` | 1.630 m | 0.143 m | 8.8% | False |

**放大参考反而走得更少** → 前进速度**饱和**(约 0.02 m/s),不是线性欠跟踪。
误差**线性累积**(24 s 到 0.45 m),全部 `fall=False`(平衡正常,就是不位移)。

#### 代码层面的原因

`sim2sim_a3_mujoco.py:135`:

```python
ENCODER_TERMS = ("command_multi_future_nonflat", "motion_anchor_ori_b_mf_nonflat")
ENCODER_FRAME_DIM = NUM_POLICY_DOFS * 2 + 6      # 29*2 关节 + 6 维朝向差
ENCODER_INPUT_DIM = NUM_FUTURE_FRAMES * 64       # 640
```

每帧只有 **58 维关节指令 + 6 维 anchor 朝向差**。**参考的水平位置从未进入观测**;
`anchor_pos_w` / `anchor_pos_error_m` 只在第 3297 行算出来写进 metrics。
5 个 encoder 预设共用同一套 `ENCODER_TERMS`,**没有任何一个带位置项** —— 是 checkpoint 的固有属性。

**所以 bridge / retarget 修不了**:参考播放已实测为 1.00×(无误),输入和链路都正常。

#### 下一步选项(已写进 runbook §17.7.1b)

| 方案 | 说明 |
| --- | --- |
| **A. 立刻可用** | 发送端 `--no-root-translation`:参考 root 固定站高,机器人与参考同处一地 → **原地踏步**,消除"被越拉越远"和"反复重置"的观感 |
| **B. 复验工具** | `tools/check_walk_following.py` —— 换 checkpoint 后重跑看比例 |
| **C. 根治** | 重训一个**观测里带 anchor 位置(或速度指令)**的策略 |
| **D. 查机载栈** | 真机运行时可能有独立的行走控制器,与 sim 策略不是一回事 |

#### 修正记录

上一节我把"头显下肢幅度只有真实走路 1/3"当成根因,用户指出 tracker 没问题后重查:
**即使输入完美,策略也不会位移**。头显幅度那组数据本身没错(70 次抬腿、屈膝 p50 17°),
但它不是"走不动"的原因 —— 用官方参考(幅度绝对标准)测试同样只走 50%。
两件事要分开:输入幅度、策略位移能力。**这一轮真正卡住的是后者。**

### M5h — 「踩踏板」:策略跟得上,但我们的参考把踝卡死了

用户问"抬脚尖踩踏板能不能完整完成"。分两半查,结论一半好一半坏。

#### 好的一半:踝是全身跟得最准的关节

拿**官方**动作测(绕开我们自己的片段):

| 关节 | 参考行程 | 实际行程 | 跟随比 | RMSE | fall |
| --- | --- | --- | --- | --- | --- |
| **L_ankle_pitch** | 16.6° | 18.0° | **108%** | **2.21°** | False |
| **L_ankle_roll** | 22.3° | 23.8° | **107%** | 1.48° | False |
| L_hip_pitch | 22.7° | 22.5° | 99% | 2.93° | False |

硬件前提:A3 **没有主动脚趾**(`foot_toe_joint` 是被动弹簧 ±13.2°,
`foot_forefoot_joint` 刚度 2396 ≈ 刚性),所以"抬脚尖"完全由 **ankle_pitch**
(−52°~+30°) 完成。官方 CSV 里也没有脚趾列 —— 一致。

#### 坏的一半:我们自己的片段把踝钉在下限

为这个任务新加了 `m5_press_pedal`(勾脚 → 踩下,`build_sequence` 绝对关键帧)。
源侧干净地 ±31.5° 穿过零点,但**重定向出来的参考是 [−52.0, −9.3],50% 的帧顶在下限,
从未到过正值** —— 机器人收到的是"一直保持最大勾脚",没有"踩下去"这半程。

**而且这是全局的**,不只新片段:

| 片段 | L踝pitch | 贴 −52 帧数 |
| --- | --- | --- |
| m5_bend_knees | −52.0 ~ −9.3 | 64/150 |
| m5_lift_left_foot | −52.0 ~ −9.3 | 73/150 |
| m5_squat_deep | −52.0 ~ −9.3 | 112/150 |
| m5_press_pedal | −52.0 ~ −9.3 | 125/249 |
| m5_lift_right_foot(只动右腿) | −9.9 ~ −9.2 | 0 |
| 官方 043_squat_deep_repeated | **−36.2 ~ +1.2** | **0** |

**根因和当初"膝从来不弯"完全同源**:`joint_map_cost` 姿态先验**只覆盖膝盖**
(`joint_prior_rows_for_frame` 只 append 膝的行),踝没有任何约束,就被脚部点云项
推到最近的边界上。**修法照抄膝盖那套**(source 度量 → 标定 → 加进 prior rows → 全链重建)。

#### 顺带修掉的验收盲区

`validate_a3_motion.py` 原来只在关节**卡死 >95%** 时报诊断,而踝是"卡 50%、
还剩 42.7° 行程",正好漏掉 —— 所以 **11/11 PASS 一直掩盖着这个问题**。
已补上 20%~95% 这一档的饱和诊断,现在每个片段的 `validation.json` 里都能看到。

#### 新增

- `m5_press_pedal` 片段(11 条验收集)+ `build_sequence()` 绝对关键帧构建器
- 验收的踝饱和诊断 + `test_ankle_saturation_is_reported`
- runbook §17.8.6(含修法步骤)

### M5i — 踝先验实现完成 + 发送端位移清零隐患

#### 1. 踝先验(用户要求实现)

照抄膝盖那套,在 `UMR/scripts/retarget_smpl_to_humanoid_surface_vector.py` 加:

- `source_ankle_flexion_deg()` —— 小腿-脚夹角;
- `robot_ankle_interior_calibration()` —— 两点探针量 A3 自己的换算关系;
- `joint_prior_rows_for_frame()` 里 append 踝的行。

顺带修了生成器:深蹲/屈膝的源里**踝根本不转**(实测 0.0°),而真实深蹲需要 ~37° 背屈
(官方 043 = 37.4°)。补 `SQUAT_ANKLE_RAD=0.35` / `BEND_ANKLE_RAD=0.25`。
`0.45` 会把踝 roll 顶到 +20° 限位并触发镜像硬失败,`0.35` 干净。

**结果(11/11 PASS,踝饱和全部归零):**

| 片段 | 修前 L踝pitch | 修后 | 贴限位 |
| --- | --- | --- | --- |
| `m5_press_pedal` | −52.0 ~ −9.3(61% 卡住) | **[−44.2, +12.6]** | **0** |
| `m5_squat_deep` | −52.0 ~ −9.3(75% 卡住) | [−33.9, −13.3] | 0 |
| `m5_bend_knees` | −52.0 ~ −9.3(43% 卡住) | [−27.7, −13.3] | 0 |

`press_pedal` 现在穿过零点,"勾起 → 踩下"两个半程都在。

#### 2. 策略只跟上了"踩下"半程

`fall=false`,但逐帧看左踝:

```text
t=1.5s  参考 -44.2   实际 -12.7   欠 31.5°   <- 脚悬空时勾不起来
t=3.6s  参考 +12.6   实际  +9.8   欠  2.8°   <- 踩下去跟得上
```

**不是硬件限制**:踝是正常驱动铰链(−52°~+30°,stiffness 0),官方
`055_lunge_front_alternating` 里踝实测到 **−41.0°,跟随 107%**。差别是那条**脚踩在地上**,
我们这条**脚悬空**。策略在"无载大幅背屈"下欠跟踪 —— 训练包络问题,和走路位移饱和同类。
**结论:真做踩踏板,参考应让脚踩在踏板上(有载)背屈,而不是悬空勾脚。**

#### 3. 修正一处数据归因错误

上一轮报的"踝跟随 108%"用的是**列 21**,而仿真里左踝其实是**列 13**(列 21/22 是别的关节)。
比值碰巧也是 107%,所以结论方向没错,但**归因是错的**。用正确列重测官方动作:
列 13 = [−38.4, +17.4] 参考 / [−41.0, +18.4] 实际 = **107%**,列 14(右踝)= 109%。已更正。

#### 4. 发送端:跳过不安全帧时不再清零已走位移

`pico_pose_zmq_minimal.py` 原来在 `_is_safe_pose_sample` 判否时执行 `root_anchor = None`,
**把已经累积的位移全部丢掉**。而判据里有 `max_frame_jump`,**走得快就会触发** ——
于是每触发一次,参考就瞬移回原点,表现出来正是"原地踏步"。

已改为**保留 anchor**(注释说明:丢一个采样不改变"操作者从哪里开始",而骨盆来自同一个
跟踪坐标系,两侧的累积位移依然有效)。只有 **A 键恢复**才重新锚定 —— 那才是操作者
真的站到了新位置。初始值那一处保留。

> 用户同时怀疑"是不是你把 root x,y 锁住了"。实测**没有**:`root_position(qpos)=qpos[:3]`
> 是原始浮动基;我改的 4 个文件里 `umr_session.py` 只**记录**了 x,y 到 dump,没有动输入。
> 那次真机运行 `skipped=0`、A 键 0 次,anchor 从未重置。用带 0.351 m 水平位移的录制走
> 真实包路径测过:参考出来 **0.427 m**,位移是通的(甚至放大 1.5 倍)。
> 新增的 `src_root_x/y` dump 字段下次真机跑就能直接看到源头位移。

#### 5. 在线路径的踝先验(补漏)

离线脚本和 live 路径**各自造 prior rows**。只改离线 → "离线验收看着修好了、现场遥操踝还是
卡住"。**膝盖当初踩过这个坑,踝又踩了一次。**

`umr_session.py` 补上 `self._ankle_prior`,与膝的行合并。实测(假发送端 + 真实包路径):

```text
[umr-online] ankle posture prior on (interior=121.43+57.30*ankle deg)
在线路径 左踝pitch [-44.0, +13.0]  行程 56.9°  贴-52: 0/919  穿过零点: 是
```

**端到端复跑(整链)**:`ACCEPTED`、`fall=False`、`solver p50 = 15.4 ms`、`e2e p95 = 19.3 ms`、
`VERDICT: OK`。(solver 从修前的 46~50 ms 降到现在 15.4 ms。)

回归防护:`test_online_session_wires_both_posture_priors` —— 断言两条路径都接了膝和踝。

### M5j — 两个真机反馈的根因

#### A. 「不断复原」= 仿真每 5 秒把机器人硬重置(已修)

用户:"本来没问题也不断复原,一直复原,稍微动一下就复原"。

桥侧完全干净(180 秒、49.0 Hz、丢 3 帧、**零 HOLD**、`skipped=0`、A 键 0 次)。
问题在 `sim2sim_a3_mujoco.py:4361`:

```python
self.current_ref_frame += 1
if self.current_ref_frame < self.reference.num_frames: return True
if self.config.batch_once: ...          # 无头模式走这条
if not self.playlist_mode:
    self.current_ref_frame = 0
    self._reset_to_current_reference()  # ← 把 data.qpos[:] 设回参考第 0 帧
```

`reference.num_frames = 249`(= `m5_stand.csv`),50 Hz 下**每 5.0 秒**把机器人
`qpos[:] = reference.qpos[0]` —— 硬拉回站立姿态。实测那 180 秒里 `ref` **回绕 39 次**。

**无头模式走 `batch_once` 分支所以从不触发** —— 这就是为什么验收全绿而开窗口没法用。

**修**:`reference_source == "stream"` 时,把 `current_ref_frame` 停在末帧,不 wrap、不 reset
(实时流没有"动作结束")。验证:`ref` 5→248 后**保持不动,0 次回绕**(修复前 39 次)。

> **我上一轮的误判**:做对照实验时看到 `ref` 每 249 步循环,我判断成"只是播放列表的显示
> 索引",并在文档里写了"结论作废"。那个判断是错的 —— 它真的在重置机器人。

#### B. 「抬脚尖不显示」= A3 的踝是闭链,而参考只写了从动关节

用户:"抬脚尖,不是抬脚后跟啊,不一样啊,我看你的一直在抬脚后跟"。

三层查下来:

1. **源动作是对的**(实测 SMPL-X 踝→脚尖仰角):
   站立基准 −24.49° → 勾起相 **+26.50°**(抬起 51°) → 踩下相 −31.89°。两个半程都在。
2. **`find_axis` 的方向也是对的**:`[-1,0,0]`,+0.55 rad 让 L_FOOT(脚尖)上升 +0.064 m。
3. **问题在 A3 侧**:这个踝是**闭链四连杆,由两个电机驱动**:

   | 关节 | 性质 | 参考里的值 |
   | --- | --- | --- |
   | `ankle_motor_up_joint` | **驱动电机** | **0.00°(全程不动)** |
   | `ankle_motor_down_joint` | **驱动电机** | **0.00°(全程不动)** |
   | `ankle_pitch_joint` | **闭链从动关节** | 命令到 −44.2° |

   **11 个片段里,两个踝电机的行程全部是 0.0°** —— 一次都没动过。而 `ankle_pitch` 被
   命令到 −44.2°,这个构型连杆**根本到不了**。

**为什么我的测量之前没发现**:直接设 `qpos[ankle_pitch]` 再 `mj_forward` 得到的是
**不满足闭链约束的假姿态**(模型有 neq=6),看起来"脚跟着转",实际物理里不成立。

**所以上一轮"踝先验修好了"这个结论只对了一半**:它确实让 `ankle_pitch` 不再停在限位上
(数值上饱和解除了),但**它约束的是一个从动关节,脚在物理上依然不动**。先验应该作用在
**电机**上,或者把期望的踝角通过闭链运动学换算成电机指令。

**下一步**(独立一轮):查 `liba3_ankle_waist_solver` 如何把参考的 ankle pitch 换算成
`ankle_motor_up/down`;若它做不到,retarget 就必须直接驱动电机而不是 `ankle_pitch`。

### M5k — 「抬脚尖」全链排查:机制是对的,卡在策略

用户:"抬脚尖,不是抬脚后跟啊,不一样啊,我看你的一直在抬脚后跟"。

逐层查完,**前面每一层都是对的**,问题在最后一层。

| 层 | 结论 | 证据 |
| --- | --- | --- |
| 源动作 | ✅ 对 | SMPL-X 踝→脚尖仰角:站立 −24.5° → 勾起 **+26.5°** → 踩下 −31.9° |
| 轴方向 | ✅ 对 | `find_axis(L_ANKLE→L_FOOT, up)` = `[-1,0,0]`,+0.55 rad 让脚尖升 0.064 m |
| 参考接口 | ✅ 对 | 仿真用 `solver.ankle_rl_pos(leg, kp, target−actual)` 把 ankle pitch 换算成**电机**指令;参考给 pitch 就是正确接口 |
| 闭链可达范围 | ✅ 够 | `ankle_ik → ankle_fk` 往返 **−52°~+30° 误差全 0.00°** |
| 符号约定 | ✅ 对 | 用**满足闭链**的电机值驱动:**pitch −44° → 脚尖仰角 −21.45°→+22.55°,即抬起 44°** |
| 参考列号 | ✅ 对 | 仿真第 13 列 = CSV 的 `left_ankle_pitch_joint`,逐点相关 **+0.9998** |
| **策略跟踪** | ❌ **跟不上** | 参考 [−43.0,+12.9] → 实际 [−12.9,−4.1],**只有 12~16%** |

#### 三个被排除的假设

1. **"参考写错了关节(该写电机)"** —— 错。pitch 就是正确接口,求解器负责换算。
2. **"闭链到不了 −44°"** —— 错。往返一致性证明全范围可达。
3. **"脚悬空所以跟不上"** —— 错。改成脚踩踏板(髋 0.70→0.22、膝 1.10→0.30)后**更差**(23.5°→8.9°)。

#### 但策略确实有这个能力

官方 `055_lunge_front_alternating`:**参考 [−38.4,+17.4] → 实际 [−41.0,+18.4],跟随 107%**。
同一个关节、同样的幅度,策略跟得住。

#### 发现的差异(不足以解释,但值得记)

| | 行程 | 最大角速度 | **中位角速度** |
| --- | --- | --- | --- |
| 我们的合成片段 | 55.9° | 64°/s | **22.3°/s** |
| 官方 055_lunge | 68.6° | 89°/s | **2.8°/s** |
| 官方 073_squat_leg_sweep | 50.6° | 74°/s | **1.0°/s** |

**官方动作的踝几乎一直静止,只在瞬间动一下;我的合成片段是连续匀速扫。** 已把 `press_pedal`
改成"保持 + 快速切换"(中位 0.0°/s、峰值 145°/s),**但跟踪仍只有 16%** —— 所以速度特征
不是根因。

#### 结论

**链路、重定向、机构、约定全部验证正确;卡在策略对这条合成动作的跟踪上。**
`press_pedal` 片段已按真实任务改正(脚踩在踏板上、保持+快速切换),留作复现用例。
下一步要查的是:为什么策略对"合成 SMPL-X 片段"的踝欠跟踪,而对官方真人动作能 107%。
值得怀疑的方向是合成片段的**下肢整体协调性**(官方动作里髋/膝/踝是耦合运动的,我的片段
里踝几乎独立扫)。

### M5l — 根因确认:策略需要"下肢协调运动",踝不能独立扫

上一轮留下的悬念:为什么策略对合成片段的踝欠跟踪,却能在官方动作上跟到 107%?

**量了各动作的髋/膝/踝耦合比例,答案很清楚:**

| 动作 | 髋/踝 幅度比 | 膝/踝 幅度比 |
| --- | --- | --- |
| 官方 055_lunge_front | 1.29 | 0.92 |
| 官方 073_squat_leg_sweep | 1.80 | 1.54 |
| 官方 043_squat_deep | 2.56 | 3.40 |
| **我们的 press_pedal(脚踩地版)** | **0.08** | **0.09** |
| 我们的 squat_deep | 5.41 | 5.50 |

**所有官方动作里,踝都是和髋/膝一起大幅运动的;我把踝做成了几乎独立扫(0.08)——
训练分布里不存在这种构型,所以策略不跟。**

**对照实验(单变量):**

| 版本 | 髋/踝比 | 踝跟随 | 勾起侧欠 | 踩下侧欠 |
| --- | --- | --- | --- | --- |
| 踝独立扫(踩地) | 0.08 | 16% | +30.1° | −17.0° |
| 踝独立扫(悬空) | 1.27 | 41% | +30.5° | −2.7° |
| **耦合 + 保持/切换** | **0.58** | **54%** | +26.3° | **−0.1°** |

**但"耦合度越高越好"这个说法是错的**(我一开始这么概括,补测后又推翻了):后来又量了
`squat_deep`(耦合比 5.41,脚踩地) —— 它的踝 **过跟到 222%**(参考 20.6° 行程,实际 45.8°,
还压到 −52.6°)。所以真实规律是:

| 动作 | 髋/踝比 | 踝跟随 |
| --- | --- | --- |
| 踝独立扫(踩地) | 0.08 | **16%** |
| 耦合+保持/切换(当前 press_pedal) | 0.58 | 54% |
| 踝独立扫(悬空) | 1.27 | 41% |
| **squat_deep(耦合+踩地)** | **5.41** | **222%(过跟)** |
| 官方 055_lunge_front | 1.29 | 107% |

**结论不是"要更高耦合",而是"踝不能单独动"**:只要髋/膝一起动(比例多少都行),踝就跟得住
甚至过跟;只有"踝独立扫"这一种构型策略不响应。这正好解释了 press_pedal 的原始设计为什么
失败 —— 它的脚本来就不该被抬起来。

**踩下(真正"踩踏板"那一半)现在欠 0.1°,等于完全跟上。**

#### 结论

`press_pedal` 现在:参考 [−44.2, +12.7],实际 [−17.9, +12.5]。
**踩下去完整执行,勾起脚尖只到 42%。** 11/11 PASS、`fall=false`。

**给用户的实用结论**:
- **"踩踏板"这个动作本身能完成**(踩下侧 100%)。
- **"抬脚尖"只到 −18°,不是 −44°** —— 因为它在我们的片段里是"腿抬起时踝单独勾",
  而这种构型策略没见过。
- 想完整做出大幅度勾脚尖,要么让动作更像真人(脚落地、靠身体前倾带动踝),
  要么从策略侧解决。

**方法论教训(值得记住)**:我一直用"合成片段能跑通验收"当作链路正确的证据,但验收只查
`fall` 和关节 RMSE,**查不出"参考超出策略训练分布"**。以后加新动作,应该先量它的
**关节耦合比例**是否落在官方集合的范围内 —— 这是比 RMSE 更早的预警信号。

### M5m — 摔倒自动复位(窗口会话)

用户:"现在这个程序倒了以后没有复原reset啊,加上去"。

背景:上一轮我修掉了"每 5 秒硬重置"那个 bug(那是错的),但修完之后**摔倒就一直躺到结束**,
交互会话第一次摔倒就废了。现在补上**只在摔倒时**复位。

**实现**(`sonic_for_a3/gear_sonic/scripts/sim2sim_a3_mujoco.py`):

- `_fall_reason()` —— 用与 metrics 完全相同的判据(`root 高度 < 0.45 m` 或 `|roll|/|pitch| > 60°`);
- `_recover_from_fall()` —— 把机器人放回**实时参考的当前位姿**:根位置用 `root_pos_m[0]`、
  朝向用 `anchor_quat_wxyz[0]`、29 个关节用 `dof_il[0]`,然后经
  **`fill_loop_qpos_motors()`** 换算 —— 这一步是关键,它会把闭链的踝/腰电机填成
  **与从动关节一致**的值,否则放下去的是一个连杆到不了的构型(前面踝那轮刚踩过这个坑);
  再清速度、清 action-delay buffer、重置 history。
- **冷却期 `FALL_RESET_GRACE_STEPS = 50`**(1 秒)。没有它的话,撑不住的姿势会每几步就
  触发一次 —— 实测深蹲片段 2500 步里复位了 **409 次**;加上之后降到 **48 次**,间隔严格 50 步。

**只在窗口会话生效**:`run_live_chain.py` 在 `--viewer` 分支加 `--reset-on-fall`,
无头分支仍是 `--batch-once`。**验收必须保持旧行为** —— 它靠 `fall` 判分,自动恢复会让它
量到"恢复后"的状态。已验证:`--skip-mujoco` 仍然 **11/11 PASS**。

**实测:**

| 片段 | 复位次数 | 说明 |
| --- | --- | --- |
| `m5_stand` | **0** | 正常动作零误触发 |
| `m5_squat_deep` | 48(每 50 步) | 深蹲在 live 路径本来就保持不住(已知限制),复位让会话能继续而不是结束 |

用实时位姿复位失败 **0 次**(修掉了一个 `root_pos_m` 是 `[10,3]` 窗口数组、我按 `[3]` 取
的 bug)。
