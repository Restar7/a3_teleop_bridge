# m5_pico_chain.md — recorded PICO → UMR → A3 → MuJoCo

同一段动作先在 SMPL-X 上定义(M4 验收动作),再**打包成 PICO 录制**(root 相对的
局部关节 + 21×3 body axis-angle + body quaternion —— 与 SONIC PICO 发送端一致),
然后走完整离线链路:

```text
recordings/m5_<clip>            PICO 录制
  → umr/source_adapter          SMPL-X 序列(neutral betas / hands=face=0)
  → UMR surface retargeting     A3 qpos
  → A3CanonicalState → flat CSV
  → A3-fast + MuJoCo
```

一条命令:

```bash
python tools/run_m5_pico_chain.py --clips stand,lift_left_foot,step_forward_slow,twist_torso_left
```

结果(4/4 PASS,详见 `generated/m5_pico_chain_report.json`):

| clip | 链路 | fall | root z | RMSE(29) |
| --- | --- | --- | --- | --- |
| stand | PASS | False | 1.073 | 0.0532 |
| lift_left_foot | PASS | False | 1.073 | 0.0944 |
| step_forward_slow | PASS | False | 1.070 | 0.0885 |
| twist_torso_left | PASS | False | 1.073 | 0.0692 |

说明:

* 合成 PICO 录制由 `tools/make_synthetic_pico_recording.py` 从 SMPL-X clip 经官方
  rest skeleton 的 FK 生成(局部关节坐标与真实发送端一致)。验证的是链路与坐标/单位
  约定,不是 PICO 硬件。
* `apps/record_pico.py --synthetic` 的内置示例只是占位 body(单点),不能用于
  retarget;真实链路请用上面的工具或真实录制。
* 真实 PICO 头显接入后按 README 的 Mode B / Mode D 执行,链路本身已验证。
