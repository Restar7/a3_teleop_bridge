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

耐久(连续 cross-fade 参考,`logs/m6_endurance_7min/`,**最终版本**):

```text
policy steps   18000(= 360 s policy 时间 / 288 s 墙钟)
fall           false
root z         mean 1.0729 m
RMSE(29)       0.1262
publisher      14253 帧 @ 49.0 Hz,rejected=0,seq 单调
receiver       received 11928,rejected 0,interpolated 11998
```

为什么需要「有界插值」:参考流是 latest-only,消费者比发布者慢时会被直接塞进一个
向前跳了若干 slot 的窗口(实测 6 分钟内 279 次跳变、最大 200 ms),policy 会在
不连续参考处摔倒。现在 SONIC 侧 provider 保留上一拍发出的窗口并朝最新窗口插值
(root 四元数用带半球修正的 nlerp),**每拍最多前进一个 20 ms slot**;
`interpolated / max_gap_ms` 计入统计,卡顿可见而不是静默产生跳变。

耐久测试中定位并修复的 3 个问题(详见 `progress.md`):
`--batch-once` 无实时基 → 新增 `--realtime`;循环播放重置 seq 且发占位窗口 →
不再重置、不发 DISCONNECTED 窗口;直接拼接动作造成参考跳变 → 用
`tools/make_endurance_clip.py` 做 20 帧 cross-fade(最大关节步进 0.087 rad,
root 0.0014 m)。

方案 §40 回归:官方 sample CSV 的 86 帧,旧路径 vs provider 路径的 encoder input
(10×64)最大绝对差 **0.0**。

## M5b — 5060 复验:验收动作集 9/9(全量 MuJoCo)

`logs/a3_validation_mujoco_full/`(2026-09-28,conda `a3_bridge`;数据
`$A3WS/UMR/output/a3_pico_all/*.npz`,即 PICO 回环那一批,未重新生成):

```text
9/9 clips PASS(每个 clip 249 policy steps @50 Hz)
fall            false(全部)
root z          1.057 – 1.073 m
roll/pitch max  ≤ 7.41°
RMSE(29)        0.0568 – 0.1436
CSV round-trip  ≤ 8.7e-09 rad
```

同轮其它门禁:`pytest tests integration -q` → 180 passed / 13 skipped;
`check_orin_ready.sh` → 16 ok / 0 failed;A3-fast 官方 baseline 1652 步 fall=false;
`run_live_chain.py --recording`(recorded PICO → online UMR → 参考流 → MuJoCo)→ ACCEPTED,
published 857 帧、solver p50 34.9 ms。详见 `progress.md` 阶段 M5b。

## 已知失败/边界

| 场景 | 现象 | 结论 |
| --- | --- | --- |
| `dance1_subject2` 全片 | tick 1504 倒地 | 含跳跃/旋转,方案 §21 明确排除;不作为验收 |
| 自研 CSV 用默认 `stride 4` | tick 337 倒地 | 采样率错误(1/4),必须 `--csv-source-fps 30 --csv-frame-stride 1` |
| streaming 用 policy 顺序发布关节 | `|action|=20` 饱和、走飞倒地 | 线上必须是 encoder(il)顺序,已在协议层强制 |
| 合成 SMPL-X `trans_z = 0` | root z ≈ 0.05 m(趴在地上) | rest skeleton 是 Y-up,需要 base 全局朝向 + 站立高度 |
| 右腿复用左腿旋转轴 | 右腿完全不动 | 每条腿的轴必须独立实测 |
| 膝限位用模型原值 | 膝被压到 −0.1222(反折) | UMR 配置把膝下界裁到 0 |
| 膝下界裁到 0 之后 | 求解器**饱和在 0**:9 个 clip 的膝行程 ≈ 0,源动作 36–43° 屈膝未被转移,下蹲由 ankle/hip pitch 代偿 | **已修复**(2026-09-28,见下节):根因是目标函数里膝完全无约束,已实现 `solver.joint_map_cost` 膝姿态先验。validator 仍以诊断形式报出 `knee_excursion_rad` |

## 膝屈曲调查与修复(2026-09-28)— **已修复**

**结论:这是 retarget 求解器侧的缺陷,不是 joint limit / 轴向约定 / 动作质量 / 迭代预算问题。**
下面的每一条都是实测,不是推断;全部探针都跑在同一套 UMR + 同一份 A3 配置上。

| # | 假设 | 实验 | 结果 |
| --- | --- | --- | --- |
| 1 | A3 膝轴向反了 | 标准 `a3_t2d5.xml` FK:膝角 vs 大腿-小腿夹角 | **不成立**。膝=0 → 165.6°;+0.3 → 148.4°;+0.6 → 131.2°。正 = 屈曲,正确 |
| 2 | 膝被限位夹住 | 把 config 膝下界依次设为 −0.1222 / 0 / 0.15 / 0.25 | **是症状不是病因**。膝**总是**停在给定范围的下界(−0.1222→−0.1222,0→0,0.15→0.15,0.25→0.25) |
| 3 | 迭代不够 | `solver.iters` 1 → 30 | **不成立**。hip 行程 0.7064 vs 0.7065,膝都是 0 |
| 4 | 源动作不物理(膝弯了但脚离地、骨盆不降) | 重新合成"脚踩实、骨盆下沉 13.4 cm"的真深蹲 | **不成立**。膝仍 0;再加深到骨盆降 20 cm / 屈膝 87°,膝仍是 0 |
| 5 | 膝 DOF 在并联脚(loop)模型里无效 | `left_knee_joint` +0.3 的 FK 灵敏度 | **不成立**。踝移动 12 cm(有效)。且把 config 指向串连模型 `a3_t2d5.xml` 结果完全相同 |
| 6 | loop 模型的树式 FK 折不了腿 | 串连 vs loop 模型:膝 +0.3/+0.6/+0.9 的 hip→踝距离 | **不成立**。两者完全一致:0.8287 → 0.8194 / 0.7917 / 0.7462 m |
| 7 | 初值停在下界上 | `tpose_qpos` 膝 = 0.8 再 retarget | **关键线索**。膝从 0.7505 一路衰减到 0(约 75 帧);同期 hip 行程 0.706→**0.073**、骨盆下沉 0.081→**0.118 m**,而且求解器**自己的 cost 更低**(0.0250→0.0225) |
| 8 | 给个合理站姿就能好转 | `tpose_qpos` 膝=0.35 + 下界 0.15 | **方向是反的**:站立 clip 膝恒为 0.356(合理),但下蹲时膝 0.357→0.15,**越蹲越伸直** |
| 9 | 有 joint-space 先验可调 | `--joint-map-cost` | 该开关在 UMR 当前版本里**声明了但从未被使用**(dead knob) |

**读法**:第 7 条最关键 —— 膝在初值被抬起来时是**可用**的,而且用了之后 cost 更低、骨盆下沉更接近源动作;但存在一个持续的、方向恒定向下的梯度,把膝拉回下界。第 8 条进一步说明这个梯度是**反向**的:本该屈膝的下蹲,膝反而伸直。
所以 `hip/ankle pitch` 代偿、`bend_knees` 名不副实、骨盆下沉只有源动作的 ~60%,都是同一个根因的下游表现。

**影响面**:只影响"下蹲/屈膝"这类需要膝折叠的动作;站立、抬臂、转体、抬单脚的 MuJoCo 跟踪都正常(fall=false,RMSE 0.057–0.144),物理安全检查全过。因此**不影响当前 9/9 验收的有效性**,但 `bend_knees` 这个动作名不副实。

**已排除、因此不要再试的修法**:改膝轴/限位、加迭代、换串连模型、重新合成更物理的源动作、调 `tpose_qpos`。
**已排除、因此不要再试的修法**:改膝轴/限位、加迭代、换串连模型、重新合成更物理的源动作、调 `tpose_qpos`。

### 根因(2026-09-28 第二轮,已在 UMR 内部定位)

在 UMR 的 solver 里临时插桩(`A3_KNEE_DEBUG`,已完全回退,脚本与备份逐字节相同)后测到:

**1) 代价函数本身偏爱伸膝 —— 不是优化器的问题。**
用 `joint_limits` 把膝钉死在不同角度,读求解器自己报告的 cost:

```text
膝 0.00 → cost mean 0.0192      膝 0.45 → 0.0364
膝 0.15 → 0.0239                膝 0.60 → 0.0443
膝 0.30 → 0.0296                膝 0.90 → 0.0650
```

单调递增 ⇒ 膝=0 就是该目标的约束最优,求解器没有做错事。

**2) 是谁把膝顶住的:foot 的 point term。**
在 `m5_bend_knees` 最深帧(第 97 帧)对左膝做 ±扫描,按身体部位分解加权 point cost:

| Δ膝 | 总 point cost | leftFoot | leftLeg(shank) |
| --- | --- | --- | --- |
| −0.20 | 19.36 | 1.06 | 2.43 |
| 0.00 | 20.89 | 3.17 | 1.83 |
| +0.20 | 26.91 | **9.28** | 1.72 |
| +0.60 | 51.30 | **32.35** | 2.97 |
| +1.20 | 110.44 | **86.34** | 8.00 |

屈膝**改善**小腿(shank)匹配,却让**脚**的匹配急剧变差 ⇒ 是脚的对应点在拉着整条腿不许折叠。

**3) 为什么"屈膝反而让脚更远":**
第 97 帧目标的左脚中心在机器人脚的 **上方 90 mm、后方 87 mm**,而目标到髋的距离(0.7867 m)比机器人当前(完全伸展)的腿(0.8286 m)**短 4.2 cm** —— 也就是说目标确实要求"折叠变短"。
但求解器收敛到的姿态是"整条腿向后扫 ~22° + 膝伸直";在这个姿态下屈膝会把脚扫向**前下方**,正好背离"后上方"的目标。于是 point 目标的最优解就是**用 hip pitch 把整条腿向后扫、膝一直停在伸展限位**。
无约束 IK 可以精确命中该目标位置,但需要膝 **−0.64 rad(超伸)** 且 `hip_yaw` **+0.59 rad(34° 扭转)**、`hip_roll` −0.31 —— 模型不允许前者,后者的异常扭转说明**脚的对应点横向位置本身是偏的**。

**结论**:膝不是坏了,而是**目标函数里完全没有膝的姿态信息**,foot-slot 项占主导且存在"伸膝 + 扫腿"这个更低代价的退化解。UMR 里唯一的关节空间先验开关 `--joint-map-cost` 是 **dead code**(声明了但从未被使用),所以没有任何项来打破这个平局。
**correspondence 本身基本是正常的**(目标确实抬起并在后方),因此**不需要重建 500-epoch correspondence** —— 这与上一轮"需要重建"的猜测不同。

### 修复(P1,已实施并验证)

UMR 里实现 `solver.joint_map_cost`(此前是 dead code):
**逐帧**测量源骨架的膝屈曲(大腿-小腿夹角),并在模型上**实测**机器人自己的
`大腿-小腿夹角 = intercept + slope × 膝角` 线性映射(实测得
`180.00 − 57.30×knee` 度),再把源膝角映射成机器人膝角目标,每膝加一行
`sqrt(cost)·(q_knee − target)` 的单位残差。

**标定依据**:A3 自己的 MJCF keyframe(标称站姿)膝角是 **+0.2515 rad(14.4° 屈曲)**,
而源 `stand` 测到 165.3° 内角 vs keyframe 的 165.59° —— 两者几乎相同。所以正确的
站姿膝角就是 ~0.25 rad;修复前 retarget 给出的 0.0000 意味着**腿比机器人自己的
标称站姿还直**。

权重从 0/300/600/1200/2400 扫描(膝跟踪 vs 点云拟合),取 **600**:

| clip | 膝角(修前) | 膝角(修后) | 源目标 | RMSE vs 源 | corr |
| --- | --- | --- | --- | --- | --- |
| stand | [0.00, 0.00] | [0.24, 0.24] | [0.26, 0.26] | 0.012 | — |
| bend_knees | [0.00, 0.00] | [0.24, 0.80] | [0.25, 0.88] | 0.060 | **+1.00** |
| lift_left_foot | [0.00, 0.00] | [0.24, 0.81] | [0.25, 0.88] | 0.055 | **+1.00** |
| step_forward_slow | [0.00, 0.00] | [0.24, 0.47] | [0.25, 0.51] | 0.035 | **+1.00** |

**全套验收复跑(9 clip,数值 + CSV + 全量 MuJoCo)**:9/9 PASS,全部 `fall=false`。
`bend_knees` 明显改善(RMSE 0.1436 → **0.1219**,roll/pitch 7.27° → **3.46°**),
而且 root **真的沉下去了**(1.0571 → **1.0044 m**,之前只有 ~6 mm 的假蹲);
5 个 clip 改善,3 个变化 ≤ +0.033 RMSE,无一摔倒。

**在线路径同步**:online session 有自己的逐帧求解循环,最初没接上先验 —— 那会导致
"离线数据弯膝、现场遥操仍塌膝"。已在 `umr_session.py` 里用**同一套**
`source_knee_interior_deg` / `robot_knee_interior_calibration` 接上并验证:
online 膝角 0.244–0.274 rad(与离线 0.24 一致),solver p50 42.8 ms / p95 75.6 ms
(与修前 40.4 / 69.9 基本持平),rejected=0。

**没有动**:correspondence、joint limits、joint mapping、部署模型。
