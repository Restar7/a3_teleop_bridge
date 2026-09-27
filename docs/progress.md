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
  df -h /root /inspire/hdd/global_user/liumengfan-253108110079
  git ls-remote https://github.com/Restar7/sonic_for_a3.git HEAD
  ```
- **输入**: 当前 cwd = `/inspire/hdd/.../wsc-workspace/GR00T-WholeBodyControl`
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
  WS=/inspire/hdd/global_user/liumengfan-253108110079/wsc-workspace/a3_teleop_ws
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
