# 4090_setup.md — 开发机环境(实际使用版本)

```text
OS        Linux(容器),Ubuntu 22.04 用户态
GPU       2 × RTX 4090,Driver 595.71.05,CUDA 13.2
Python    3.12.3(系统);各仓库用 uv venv 隔离
git       2.43.0 + git-lfs 3.4.1(apt 安装)
cmake     3.31.6, gcc 可用
conda     无(改用 uv)
gh        无(用 https clone)
```

工作区布局:

```text
~/a3_teleop_ws                       (-> /inspire/hdd/.../wsc-workspace/a3_teleop_ws)
├── sonic_for_a3/                    Restar7/sonic_for_a3 @ fe6868b,分支 feat/a3-streaming-reference
├── UMR/                             hanyang9/UMR @ c56b630,分支 feat/a3-online-retarget
├── GR00T-WholeBodyControl ->       已有 clone(未重复 clone)
├── a3_teleop_bridge/                本工程
├── logs/                            所有运行证据(baseline / a3_validation / m6_*)
├── recordings/                      PICO 录制
├── data/smplx_validation/           方案 §21 验收动作(合成)
├── cache/downloads/                 SMPL-X 压缩包与解压结果
└── env.sh                           环境变量(见下)
```

环境创建(可复现):

```bash
export PATH=$HOME/.local/bin:$PATH
# sonic_for_a3(MuJoCo sim2sim)
cd ~/a3_teleop_ws/sonic_for_a3
uv venv .venv_sim --python 3.10
uv pip install --python .venv_sim/bin/python -e "gear_sonic[sim]" huggingface_hub pyzmq msgpack
# UMR
cd ~/a3_teleop_ws/UMR
uv venv .venv_umr --python 3.12
uv pip install --python .venv_umr/bin/python torch==2.4.1 --index-url https://download.pytorch.org/whl/cu121
uv pip install --python .venv_umr/bin/python -r requirements-umr.txt
# bridge
cd ~/a3_teleop_ws/a3_teleop_bridge
uv venv .venv_bridge --python 3.10
uv pip install --python .venv_bridge/bin/python numpy scipy pyzmq pyyaml pytest msgpack mujoco
uv pip install --python .venv_bridge/bin/python -e .
```

资产:

```bash
cd ~/a3_teleop_ws/sonic_for_a3
git lfs pull
.venv_sim/bin/python download_from_hf.py --component pt     # 035 step-200000
(cd checkpoints/035_step200000 && sha256sum -c SHA256SUMS)  # 9cf33be2...
# SMPL-X neutral(许可资产,由项目方提供)
ls ~/a3_teleop_ws/UMR/smpl/SMPLX_NEUTRAL.pkl
```

已知环境细节:

* 本机**没有 EGL 平台**:不要设置 `MUJOCO_GL=egl`(`import mujoco` 会失败);
  `--batch-once` 模式不需要 GL。
* mujoco 3.14 的 `mjtJoint` 枚举与 numpy 标量比较不对称 —— 必须 `int()` 后再比较。
