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



