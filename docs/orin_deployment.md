# orin_deployment.md — Orin 迁移(V1:Orin 只做参考,policy 仍在 A3)

> 方案 §52–§56、§88:**不要**第一阶段就把 policy 搬到 Orin;Orin 只跑
> PICO / SMPL / UMR / predictor 并把 `A3_REFERENCE_V1` 发给 A3 原装计算板。

## 1. 目标结构

```text
PICO ──Wi-Fi──► Orin ──Ethernet(A3_REFERENCE_V1)──► A3 RK3588 ──► MDU ──► motors
                │  pico_receiver / smpl_adapter / umr_online /
                │  a3_predictor / reference_publisher / camera_source
```

A3 侧保持官方:policy(RKNN)、safety、motor mapping、parallel solver、MDU。

## 2. 上机第一步:系统检测(不要假设 JetPack 版本)

```bash
mkdir -p ~/orin_system_info
{
  uname -a; uname -m
  cat /etc/nv_tegra_release 2>/dev/null || true
  nvcc --version 2>/dev/null || true
  dpkg -l | grep -i tensorrt || true
  python3 --version
  free -h; df -h
} | tee ~/orin_system_info/system_info.txt
```

把结果写入本仓库 `orin_system_info/` 并据此决定依赖版本。

## 3. 环境(重新创建,**不要**拷贝 4090 的 conda/venv 目录)

```bash
uv venv .venv_bridge --python 3.10        # 或系统 python3 + venv
uv pip install numpy scipy pyzmq pyyaml msgpack
# UMR 侧重点是 aarch64 wheel:
#   torch(匹配当前 JetPack 的 CUDA)、trimesh、warp-lang、coembreex、viser
```

逐项验证并在缺失时记录 **ORIN_BLOCKER.md**(模板见下):

```text
numpy / scipy / pyzmq / msgpack            ← bridge 必需
torch(+CUDA)                               ← UMR 必需
trimesh / warp-lang / embreex              ← UMR surface 采样
mujoco                                     ← 仅离线复现
pinocchio / pytorch3d / 自定义扩展          ← 若某依赖只有 x86
```

## 4. 离线基准(§55)

同一段录制的 PICO 动作在 4090 与 Orin 上各跑一次,比较:

```text
q trajectory difference(逐关节 max/mean 误差)
solver latency(P50/P95)
memory / GPU utilization
```

输出 `benchmarks/orin_umr.json`。

## 5. 实时门槛与 fallback(§56)

如果 Orin 上 UMR 不能实时:

```text
PICO → 4090(或外部 PC)UMR → Ethernet → Orin/A3
```

网络解耦是第一版就保留的能力(`configs/network.yaml` 只需把 `connect_host`
改成外部 PC 的地址)。

## 6. V2(仅当 V1 稳定后)

`A3-fast.onnx → TensorRT → Orin → 29 action → A3 MDU(经官方安全层)`,
并且必须先 shadow mode 对比 Orin 与 RK3588 的 action,不得直接 takeover。
TensorRT engine **不要**从 4090 拷贝,必须在 Orin 上重建(§67)。
