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
repo      = https://github.com/hanyang9/UMR
path      = ~/a3_teleop_ws/UMR
commit    = c56b6301ded02a187a30cc6aafa4f535735104d2
branch    = main (feature branch: feat/a3-online-retarget)

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
