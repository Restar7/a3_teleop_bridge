# examples — 可复现的示例输入/输出(方案 §86)

这些是**真实跑出来的**小样本,用来在不动硬件(甚至不动 PICO)的情况下复现整条链路。
全部按“产生它的命令”标注,不要手工编辑。

## 1. PICO 录制(`../recordings/`)

```text
recordings/m5_stand/                站立
recordings/m5_step_forward_slow/    慢速迈步
recordings/m5_twist_torso_left/     左转体
recordings/m5_lift_left_foot/       抬左脚
```

每个目录含 `smpl.npz`(`smpl_joints [N,24,3]`、`smpl_pose [N,21,3]`、
`root_translation`、`root_quat_wxyz`、`seq`、`timestamp_ns`)+ `metadata.json` + `stats.json`。

产生方式(无头显时用合成录制;有头显时用 `apps/record_pico.py`):

```bash
python tools/make_synthetic_pico_recording.py \
    --clip ../data/smplx_validation/lift_left_foot.npz \
    --out ../recordings/m5_lift_left_foot
```

## 2. UMR → A3 结果(`../UMR/output/a3_pico/`,A3 CSV 在 `../logs/`)

```bash
python tools/run_umr_a3_batch.py --data-dir ../data/pico_smplx/lift_left_foot \
    --out-dir ../UMR/output/a3_pico --force
# => ../UMR/output/a3_pico/m5_lift_left_foot_smplx_agibot_a3.npz  (qpos [N,72])

python tools/run_a3_validation_suite.py --data-dir ../UMR/output/a3_pico \
    --out-dir ../logs/m5/lift_left_foot --only m5_lift_left_foot
# => ../logs/m5/lift_left_foot/m5_lift_left_foot/m5_lift_left_foot.csv  (A3 flat CSV)
```

## 3. A3 参考流录制(`reference_recording/`)

`tools/record_reference.py` 订阅 `A3_REFERENCE_V1` 并存盘;`apps/replay_reference.py --windows`
可以把它**原样重播**,这就是方案 §79 要求的“一级 replay 能力”:真机出问题时不需要机器人、
不需要 PICO、不需要 UMR 就能复现。

```bash
# 先有一个 publisher(任意来源:CSV / UMR 结果 / online UMR)
python -m a3_teleop_bridge.apps.replay_reference \
    --csv ../logs/a3_validation/stand/stand.csv --fps 30 --publish-hz 50 --loop \
    --endpoint tcp://127.0.0.1:15680 --duration 25 &
python tools/record_reference.py --endpoint tcp://127.0.0.1:15680 \
    --duration 12 --out ../examples/reference_recording/stand_12s
# => stand_12s/windows.npz + metadata.json(601 窗口,约 50.0 Hz,rejected=0)

# 重播录制的参考流,并让 SONIC 消费(端到端验证过:fall=false,roll/pitch 1.54°)
python -m a3_teleop_bridge.apps.replay_reference \
    --windows ../examples/reference_recording/stand_12s --publish-hz 50 --loop \
    --endpoint tcp://127.0.0.1:15682 --duration 60 &
cd $SONIC_A3_ROOT && .venv_sim/bin/python gear_sonic/scripts/sim2sim_a3_mujoco.py \
    --checkpoint checkpoints/035_step200000/model_step_200000.pt \
    --motion ../logs/a3_validation/stand/stand.csv \
    --encoder-mode a3_fast --csv-source-fps 30 --csv-frame-stride 1 \
    --reference-source stream --reference-endpoint tcp://127.0.0.1:15682 \
    --batch-once --realtime --max-policy-steps 500 --metrics-out /tmp/replay_win_metrics.json
```

`windows.npz` 字段:`seq`、`timestamp_ns`、`receive_timestamp_ns`、`dt`、
`root_pos_m [N,10,3]`、`root_quat_wxyz [N,10,4]`、`joint_pos_rad [N,10,29]`、
`joint_vel_rad_s [N,10,29]`、`source_age_ms`、`valid`;
`metadata.json` 记录窗口数、seq 区间、实测发布频率、网络延迟 p50/p95 与拒收计数。
