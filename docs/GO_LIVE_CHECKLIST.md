# GO_LIVE_CHECKLIST.md — 从打包到真机遥操的单页清单

> 每一步都有:命令 + 通过判据。**任一项不通过就停**,不要跳到下一步(方案 §13/§63/§85)。
> 详细说明见 `DEPLOY_ORIN.md`(Orin)、`A3_ONBOARD.md`(机载)、`pico_setup.md`(PICO)。

图例:🖥 开发机(4090) · 🟩 Orin · 🟥 A3 MDU

---

## A. 上机前(全部在 🖥 完成)

- [ ] **A1 打包** 🖥
  ```bash
  cd $A3WS/a3_teleop_bridge && bash scripts/package_bundle.sh --with-umr
  ```
  判据:`[package] size : 14M` + 打印 `sha256`;`MANIFEST.txt` 里三个仓库 SHA 与本地一致。

- [ ] **A2 回归测试** 🖥
  ```bash
  .venv_bridge/bin/python -m pytest tests integration -q
  ```
  判据:`167 passed, 1 skipped`。

- [ ] **A3 MuJoCo 端到端** 🖥(不接任何硬件)
  ```bash
  bash scripts/run_a3_streaming.py --csv $A3WS/logs/a3_validation/endurance_loop.csv \
       --policy-steps 3000 --duration 300
  ```
  判据:`ACCEPTED`、`fall=false`、root z ≈1.07 m、rejected=0。

- [ ] **A4 在线 UMR 单独跑通** 🖥
  ```bash
  $PY_UMR -m a3_teleop_bridge.apps.retarget_live --source recording --backend umr-online \
       --recording $A3WS/recordings/m5_twist_torso_left --duration 8 --no-publish --stats /tmp/on.json
  ```
  判据:`solved>0`、solver ≈40 ms 量级、rejected=0。

- [ ] **A5 参考流录制/重播能力可用** 🖥
  判据:`examples/reference_recording/stand_12s/` 能被 `replay_reference --windows` 重播。

---

## B. Orin 部署(🟩)

- [ ] **B1 传输 + 解包**
  ```bash
  bash scripts/sync_to_orin.sh --bundle $A3WS/dist/a3_teleop_orin_<stamp>.tar.gz --host <ORIN_IP>
  ```
- [ ] **B2 环境变量** 🟩 `source scripts/env_orin.sh` → 四个路径都打印出来
- [ ] **B3 系统预检** 🟩 `bash scripts/orin_preflight.sh` → 生成 `orin_system_info/` 三份报告;
      缺 x86-only 依赖就写 `ORIN_BLOCKER.md`
- [ ] **B4 重建环境** 🟩 `bash scripts/orin_bootstrap.sh --with-umr` → 测试通过
- [ ] **B5 SMPL-X 手工拷贝(许可证资产,包里没有)** 🟩
  ```bash
  rsync -avP <保存处>/SMPLX_NEUTRAL.pkl <USER>@<ORIN_IP>:~/a3_teleop_ws/UMR/smpl/
  ```
- [ ] **B6 checkpoint / ONNX / RKNN** 🟩 `python download_from_hf.py --component pt onnx rknn`
- [ ] **B7 就绪门禁** 🟩 `bash scripts/check_orin_ready.sh`
      判据:**`16 ok, 0 failed`**(含 online UMR 真解一帧)
- [ ] **B8 网络** 🟩 `configs/network.yaml` 里 `reference.bind_host: 0.0.0.0`、`pico.port: 5556`;
      `ping <A3_IP>` 通;防火墙放行 5560/tcp

---

## C. PICO 接入(🟩,细节见 `pico_setup.md`)

- [ ] **C1 头显** 开发者模式 + XRoboToolkit app + 填 PC IP + 全身追踪可用
- [ ] **C2 PC Service** 在 Orin(或 4090)启动,头显能连上
- [ ] **C3 SDK** `bash setup_orin.sh && python setup.py install` → `import xrobotoolkit_sdk` 成功
- [ ] **C4 发送端** 🟩
  ```bash
  .venv_sim/bin/python gear_sonic/scripts/pico_pose_zmq_minimal.py \
      --port 5556 --target_fps 50 --start_unpaused     # 或按手柄 A 键
  ```
  判据:日志 `Stream state: RUNNING` + `sent` 递增(默认是 PAUSED,**不加参数或按键就没有数据**)
- [ ] **C5 bridge 收帧** 🟩 §5 的订阅脚本 → `received > 0`,形状 `(24,3)/(21,3)`,rejected=0

---

## D. 参考流 + MuJoCo 复核(🟩 + 🖥)

- [ ] **D1 Orin 起参考流** 🟩
  ```bash
  bash scripts/run_orin_live.sh --duration 600 --endpoint tcp://0.0.0.0:5560 \
       --save-calibration calibration.json
  ```
  判据:首个窗口约 15 s 后出现;状态机 `DISCONNECTED → CALIBRATION`;`--save-calibration` 后
  重新载入能进 **TRACKING**;`solver p50` 稳定(4090 实测 37.5 ms)
- [ ] **D2 MuJoCo 消费同一条流** 🖥
  ```bash
  bash scripts/run_mujoco_consumer.sh --endpoint tcp://<ORIN_IP>:5560 --steps 3000
  ```
  判据:`fall=false`、root z ≈1.07 m、无 rejected
- [ ] **D3 长跑** ≥10 分钟不出现 SAFE_STOP,`frames_published` 线性增长,内存稳定
- [ ] **D4 故障注入** 拔网线 / kill PICO → 进 HOLD/SAFE_STOP;发 NaN/越界 → reject

---

## E. A3 机载(🟥,细节见 `A3_ONBOARD.md`)

- [ ] **E1 交叉编译** 🖥 `build_a3_deploy_pkg.sh --arch rockchip ...`(需 sysroot + aarch64 ONNXRuntime)
- [ ] **E2 传输到 MDU**(经 HDU 跳板)
- [ ] **E3 服务配置** 🟥 备份 `sm_config.yaml` → Motion 功能组只留 `agent` → `systemctl restart agibot_pm`
      → `systemctl is-active --quiet agibot_pm` 成功且 `ps aux | grep '[m]otion_control'` 无输出
- [ ] **E4 包预检** 🟥 `file ./a3_deploy_onnx_ref` 为 ARM aarch64;`backend: rknn`;两个 `.rknn` 存在
- [ ] **E5 receive-only probe** 🟥
  ```bash
  A3_TRANSPORT=iceoryx A3_PROBE_SOURCE=g1 A3_LATENCY_LOG=verbose taskset -c 4-5 ./run_a3_probe.sh
  ```
  判据:六路状态齐全、freshness/watchdog 正常、**不发布任何电机命令**
- [ ] **E6 参考接入** 🟥 适配节点发布 `/ta/whole_body_command` →
      runtime 日志出现 `[teleop] first whole_body_command received`
      (关节顺序:wire 是 il 顺序,该通道是 policy 顺序,**必须用 contract 的置换表**)
- [ ] **E7 正式启动(悬吊)** 🟥 `taskset -c 4-5 ./run_a3.sh`
      现场:安全吊带/防坠、物理急停、安全员、清场

---

## F. 悬吊 10 级动作(🟥,任一级失败即停)

- [ ] 1 stand &nbsp;&nbsp; [ ] 2 shoulder &nbsp;&nbsp; [ ] 3 elbow &nbsp;&nbsp; [ ] 4 wrist &nbsp;&nbsp; [ ] 5 torso small rotation
- [ ] 6 knee small bend &nbsp;&nbsp; [ ] 7 weight shift &nbsp;&nbsp; [ ] 8 foot unload &nbsp;&nbsp; [ ] 9 foot lift &nbsp;&nbsp; [ ] 10 slow single step

每级:观察 ≥30 s → 记录 → 无异常才继续。禁止跳、跑、深蹲到底、跪、躺、快速 180° 旋转(方案 §45)。

---

## G. 每级都要记录(方案 §0/§78)

```text
阶段 / commit SHA(a3_teleop_bridge + sonic_for_a3 + UMR)/ 执行命令 / 输入 / 输出 /
结果 / 已知问题 / 下一阶段
+ orin_system_info/ 三份报告 + benchmarks/orin_umr.json + checkpoint sha256
+ 参考流录制文件(便于事后 replay 复现,不需要机器人)
```

追加到 `a3_teleop_bridge/docs/progress.md`。
