# benchmark.md — 4090 延迟与吞吐

数据:`benchmarks/4090_live.json`(`tools/benchmark_latency.py`,400 次采样)。

| 阶段 | mean | p50 | p95 | p99 | max |
| --- | --- | --- | --- | --- | --- |
| UMR → A3CanonicalState | 0.059 ms | 0.059 | 0.063 | 0.068 | 0.072 |
| predictor(push + window) | 0.478 | 0.474 | 0.499 | 0.540 | 0.572 |
| protocol encode | 0.052 | 0.052 | 0.054 | 0.060 | 0.064 |
| protocol decode | 0.094 | 0.093 | 0.101 | 0.112 | 0.116 |
| ZMQ localhost round-trip | 0.130 | 0.127 | 0.144 | 0.153 | 0.167 |
| CSV 读取(90 帧) | 1.771 | 1.754 | 1.885 | 1.941 | 1.955 |

UMR 求解(来自 `logs/umr/a3_retarget.log`,correspondence 已缓存):

```text
36.0 Hz(27.77 ms/帧),多段采样的中位数
```

结论:

* 桥接在 UMR 之后引入的总开销 ≈ **0.76 ms**(predictor + encode + network + decode),
  相对 20 ms 的 policy 周期可以忽略。
* UMR 处于方案 §47 期望的 **25–50 Hz** 区间,因此 §48 的插值 fallback 暂不需要;
  若后续动作复杂度上升导致 UMR 掉到 20–30 Hz,预测器已经能补足窗口。
* 包大小 < 3 KB,50 Hz 下带宽 < 150 KB/s。

## 复现

```bash
python tools/benchmark_latency.py            # 写入 benchmarks/4090_live.json
python tools/run_a3_validation_suite.py ...  # 端到端(含 MuJoCo)
```
