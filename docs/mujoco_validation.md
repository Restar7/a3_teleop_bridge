# mujoco_validation.md — MuJoCo 验证证据

所有结论均来自实际运行,原始日志在 `~/a3_teleop_ws/logs/`。

## M1 — 官方 A3-fast baseline

命令(即官方 `docs/a3_training2sim2deploy.md` §4):

```bash
cd ~/a3_teleop_ws/sonic_for_a3
.venv_sim/bin/python gear_sonic/scripts/sim2sim_a3_mujoco.py \
  --checkpoint checkpoints/035_step200000/model_step_200000.pt \
  --motion a3_data/agibot_a3/001_walk_front_slow.csv \
  --encoder-mode a3_fast \
  --mjcf gear_sonic/data/assets/robot_description/mjcf/a3_t2d5_loop_passive_foot_twostage_fit_optimized.xml \
  --batch-once
```

结果(`logs/baseline/full_metadata.json`):

```text
checkpoint sha256  9cf33be2…(与发布一致)
policy steps       1652 @ 50 Hz(25.1 s ≈ 66x realtime)
fall               false
non-finite frames  0
root height        mean 1.0519 m, min 1.0294 m
root roll/pitch    max 9.62°
all_29 tracking    RMSE 0.0662 rad
encoder            640D = 10 × 64('a3_fast')
actor obs dim      930(proprio)→ 930 + 640 = 1570
29DoF mapping      OK(logical_policy=29,total_model_joints=48)
```

## M3b/M4 — SMPL-X → UMR → A3 → CSV → A3-fast → MuJoCo

9 个验收动作(方案 §21),脚本 `tools/run_a3_validation_suite.py`,
报告 `logs/a3_validation/report.json`:

| clip | MuJoCo | fall | root z | roll/pitch | RMSE(29) | 数值验收 |
| --- | --- | --- | --- | --- | --- | --- |
| stand | ok | false | 1.073 m | 1.54° | 0.0537 | 14/14 |
| raise_left_arm | ok | false | 1.070 | 3.03° | 0.0935 | 14/14 |
| raise_right_arm | ok | false | 1.072 | 1.95° | 0.0810 | 14/14 |
| bend_knees | ok | false | 1.056 | 11.85° | 0.1333 | 14/14 |
| twist_torso_left | ok | false | 1.073 | 1.48° | 0.0696 | 14/14 |
| twist_torso_right | ok | false | 1.073 | 3.72° | 0.0582 | 14/14 |
| lift_left_foot | ok | false | 1.072 | 1.33° | 0.0944 | 14/14 |
| lift_right_foot | ok | false | 1.073 | 1.39° | 0.0903 | 14/14 |
| step_forward_slow | ok | false | 1.070 | 3.04° | 0.0894 | 14/14 |

14 项数值检查:有限性、root 高度带、无 teleport、四元数单位化、关节限位、
左右膝方向、左右镜像、关节速度/加速度、双脚不交叉(heading 参考系)、
双脚高于地面、CSV round-trip(位 5e-9 m / 角 9e-9 rad)。

## M6 — streaming 参考链路

短测(300 步)与 CSV 模式逐项一致:

```text
                stream        csv(control)
fall            false         false
root z          1.0733 m      1.0733 m
RMSE(29)        0.05365       0.05365
```

耐久(`logs/m6_endurance_5min/`):18000 policy step(= 360 s 参考),
`fall=false`,root z 1.0696,RMSE 0.1558,publisher 7004 帧 @49.9 Hz,
`rejected=0`。

方案 §40 回归:官方 sample CSV 的 86 帧,旧路径 vs provider 路径的 encoder input
(10×64)最大绝对差 **0.0**。

## 已知失败/边界

| 场景 | 现象 | 结论 |
| --- | --- | --- |
| `dance1_subject2` 全片 | tick 1504 倒地 | 含跳跃/旋转,方案 §21 明确排除;不作为验收 |
| 自研 CSV 用默认 `stride 4` | tick 337 倒地 | 采样率错误(1/4),必须 `--csv-source-fps 30 --csv-frame-stride 1` |
| streaming 用 policy 顺序发布关节 | `|action|=20` 饱和、走飞倒地 | 线上必须是 encoder(il)顺序,已在协议层强制 |
| 合成 SMPL-X `trans_z = 0` | root z ≈ 0.05 m(趴在地上) | rest skeleton 是 Y-up,需要 base 全局朝向 + 站立高度 |
| 右腿复用左腿旋转轴 | 右腿完全不动 | 每条腿的轴必须独立实测 |
| 膝限位用模型原值 | 膝被压到 −0.1222(反折) | UMR 配置把膝下界裁到 0 |
