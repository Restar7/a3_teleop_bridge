# a3_teleop_bridge

PICO → SMPL → UMR → AgiBot A3 canonical reference → future predictor → SONIC
A3-fast → MuJoCo / A3 真机。

主计划与阶段验收: [`docs/PLAN.md`](docs/PLAN.md)
执行日志: [`docs/progress.md`](docs/progress.md)
上游版本: [`docs/upstream_versions.md`](docs/upstream_versions.md)

## 目录

```text
configs/     a3_joint_map.yaml / teleop.yaml / predictor.yaml / network.yaml
src/a3_teleop_bridge/
  types.py   四个冻结数据结构的唯一定义处
  clocks.py  时间戳/统计
  pico/      ZMQ 订阅、录制、标定、SMPL 帧
  umr/       source adapter / offline / online / state converter
  a3/        joint map / canonical state / reference window / predictor / limits / csv
  transport/ A3_REFERENCE_V1 协议、发布订阅
  apps/      可执行入口
tools/       inspect_a3_contract.py / inspect_umr_result.py / benchmark_latency.py
tests/       单元测试(见方案 §80 最低集合)
generated/   由源码自动生成、不可手改的契约文件
integration/ 端到端集成测试
```

## 环境

本机无 conda,使用 uv venv(与方案中的 conda env 一一对应):

```bash
uv venv .venv_bridge --python 3.10
uv pip install --python .venv_bridge/bin/python numpy scipy pyzmq pyyaml pytest msgpack
uv pip install --python .venv_bridge/bin/python -e .
```

## 铁律 (方案 §85)

- joint 一律按**名称**映射,禁止 `qpos[7:36]` 式 index 假设。
- `sonic_for_a3` 只做最小 streaming reference 接口修改,原 CSV 路径行为不变。
- UMR 只暴露 online step API,不改核心算法。
- 真机前必须过 MuJoCo;不绕过 A3 MDU / 安全层。
