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

