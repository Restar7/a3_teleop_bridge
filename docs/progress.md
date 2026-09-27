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
