# Upstream versions (pinned)

Recorded at bootstrap time (2026-09-27). Do not silently move these.

```text
sonic_for_a3:
repo      = https://github.com/Restar7/sonic_for_a3  (fork of AgibotTech/sonic_for_a3)
path      = ~/a3_teleop_ws/sonic_for_a3
commit    = fe6868ba37034f89b912f0fb851bce19f120266d
branch    = main (feature branch: feat/a3-streaming-reference)
checkpoint= 035_step200000/model_step_200000.pt (sha256 9cf33be2f4e602858b68ce31d5824113ab1dda250acaab5842b6d3bf88b70f2d)

UMR:
repo      = https://github.com/hanyang9/UMR  (fork: Restar7/UMR)
path      = ~/a3_teleop_ws/UMR
commit    = c56b6301ded02a187a30cc6aafa4f535735104d2
branch    = main (feature branch: feat/a3-online-retarget)
A3 branch = feat/a3-online-retarget @ 086d46c
            (84d3650 implements solver.joint_map_cost as the knee posture prior;
             086d46c splits out the shared helpers the online session reuses)
            The retarget WILL collapse the A3 knee without this: the objective
            has no knee posture term, so the solver leaves it on the extension
            stop.  See docs/mujoco_validation.md.

GR00T-WholeBodyControl:
repo      = https://github.com/Restar7/GR00T-WholeBodyControl (fork of NVlabs/GR00T-WholeBodyControl)
path      = ~/a3_teleop_ws/GR00T-WholeBodyControl  -> symlink to existing clone, NOT re-cloned
commit    = 3c2dbe1c302e62db56291914934b6d8f5460f341
branch    = main (unmodified; PICO / XRoboToolkit stage only)
```

## Notes

- `sonic_for_a3` had to be cloned even though `GR00T-WholeBodyControl` was
  already present: the A3 assets (`gear_sonic/scripts/sim2sim_a3_mujoco.py`,
  `gear_sonic/data/assets/robot_description/mjcf/a3_t2d5_*.xml`,
  `a3_data/agibot_a3/*.csv`, `gear_sonic_deploy/assets/a3_runtime/**`) exist only
  in the A3 fork. The pre-existing `GR00T-WholeBodyControl` checkout does not
  contain them.
- The pre-existing `GR00T-WholeBodyControl` clone was reused (per instruction
  "不需要重新 clone sonic"), so no second copy of that repo exists.

## 本项目的 feature branch 提交(截至本次会话)

```text
sonic_for_a3  feat/a3-streaming-reference
  301d4f1  ReferenceProvider 抽象(Csv / Streaming)
  db3110e  joint-order 校验(il 顺序)
  ef07aac  --realtime 发布
  e9d3a11  C++ A3ReferenceStream + a3_reference_limits.hpp + 独立单测
  2f244dc  streaming 统计输出
  e735bba  参考推进上界 + nlerp 插值(每 tick 一个槽位)

UMR  feat/a3-online-retarget
  91895f3  env var 路径展开 + A3 robot config
  d892361  嵌套相对 mesh 路径按 meshdir 解析
  9341764  膝关节下界裁到 0(禁止反折)
```

上游 main/master 未被修改;`sonic_for_a3` 只做 §16 允许的最小 streaming 接口改动,
`UMR` 只暴露 online 装配所需的公开函数(未重写算法)。
