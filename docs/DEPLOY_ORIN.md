# DEPLOY_ORIN.md — 部署到 Jetson Orin(完整可复制命令)

> **V1 架构(方案 §52/§71/§88)**:Orin 只负责 `PICO → SMPL → UMR online → predictor → A3_REFERENCE_V1`,
> **不接管电机**。A3 原装 RK3588 + RKNN + MDU + safety 全部沿用官方。
> 本文所有命令都可直接复制;凡本机未实测的步骤都标注了 **[需 Orin 硬件]**。
>
> 相关文档:`A3_ONBOARD.md`(机载侧)、`DELIVERY.md`(交付物/仓库/**部署公钥**/推送命令)、
> `docs/safety.md`、`docs/pico_setup.md`、`ORIN_BLOCKER.md`。

---

## 0. 交付物与两类机器

| 机器 | 角色 | 需要的仓库 |
| --- | --- | --- |
| 4090 工作站 | 开发/打包/MuJoCo 验证,可选 fallback 跑 UMR | `a3_teleop_bridge` `sonic_for_a3` `UMR` |
| **Jetson Orin** | 真机 V1 的参考生成端 | `a3_teleop_bridge` `sonic_for_a3`(代码+资产) `UMR`(代码) |
| **A3 机载 RK3588/MDU** | policy + safety + 电机,保持官方 | 官方 deploy 包 + 我们的 `A3ReferenceStream` |

端口约定(见 `configs/network.yaml`):

```text
5556  PICO pose 流(GR00T / sonic_for_a3 的 sender 发布,Orin 订阅)
5560  A3_REFERENCE_V1(Orin 发布,A3 / MuJoCo 订阅)
```

> ⚠️ PICO 端口是 **5556**:`sonic_for_a3/gear_sonic/scripts/pico_pose_zmq_minimal.py`
> 与 GR00T 的 `pico_manager_thread_server.py` 默认都是 5556。本工程早期文档写 5561,
> 已全部改为 5556(commit 见 `docs/progress.md`)。

---

## 1. 在 4090 上打包

```bash
cd $A3WS/a3_teleop_bridge
bash scripts/package_bundle.sh --with-umr
# => $A3WS/dist/a3_teleop_orin_<stamp>.tar.gz   (16 MB,含 SHA256 与 MANIFEST.txt)
```

包里有什么 / 没有什么(实测):

```text
有:  a3_teleop_bridge 全量(src/tools/tests/configs/docs/examples/scripts)
     sonic_for_a3 的 gear_sonic + gear_sonic_deploy(A3 MJCF/URDF/mesh、a3_loop solver、
     rknn_runtime 2.3.2 aarch64、C++ a3_reference_stream)
     UMR 的 scripts + robot_configs + assets/smplx_parts_segm.pkl
没有: .venv* / .git / logs / recordings / checkpoints(.pt/.onnx/.rknn)
     UMR 其它机器人 mesh(312 MB)与 demo 数据(57 MB)
     SMPL-X 人体模型(**许可证资产,必须手工拷贝**,见 §3)
     UMR 生成的 *.floating_mjcf.xml(内嵌 4090 绝对路径)
```

---

## 2. 传到 Orin

```bash
cd $A3WS/a3_teleop_bridge
bash scripts/sync_to_orin.sh --bundle $A3WS/dist/a3_teleop_orin_<stamp>.tar.gz \
    --host <ORIN_IP> --user <USER> --dest '~/a3_teleop_ws'
# 内部就是 mkdir + rsync -avP + 远端 tar -xzf
```

没有网络/离线时,用 U 盘或 `scp` 传同一个 tar.gz 即可,然后在 Orin 上:

```bash
mkdir -p ~/a3_teleop_ws && cd ~/a3_teleop_ws && tar -xzf /path/to/a3_teleop_orin_<stamp>.tar.gz
ls   # => a3_teleop_orin/
```

---

## 3. Orin 上第一次配置(逐条执行)

```bash
ssh <USER>@<ORIN_IP>
cd ~/a3_teleop_ws/a3_teleop_orin
mv a3_teleop_bridge sonic_for_a3 UMR ~/a3_teleop_ws/ 2>/dev/null || true
cd ~/a3_teleop_ws/a3_teleop_bridge
source scripts/env_orin.sh            # 导出 A3WS/SONIC_A3_ROOT/UMR_ROOT/PY_*
```

**3.1 系统信息(不要假设 JetPack 版本)**

```bash
bash scripts/orin_preflight.sh
# => orin_system_info/{system_info,python_deps,zmq_check}.txt
# 把这三个文件保留下来(上机记录要写进 docs/progress.md)
```

**3.2 依赖环境(重建,不要拷贝 4090 的 venv)**

```bash
bash scripts/orin_bootstrap.sh --check-only     # 先只做前置检查
bash scripts/orin_bootstrap.sh --with-umr       # 建 .venv_bridge 与 .venv_umr 并跑测试
```

要点:

- bridge 是纯 python(numpy/scipy/pyzmq/PyYAML/msgpack),aarch64 直接装;
- UMR 需要 `torch`(按 **当前 JetPack** 选 wheel)、`trimesh`、`clarabel`、`smplx`;
  若某个依赖只有 x86 wheel(例如某些 embree/pytorch3d 扩展),**记录到 `ORIN_BLOCKER.md`**
  并走 §6 的 fallback,不要硬改 UMR 算法。

**3.3 许可证资产(必须手工拷贝,包里没有)**

```bash
# 从 4090(或你保存 SMPL-X 的地方)拷到 Orin
rsync -avP ~/a3_teleop_ws/UMR/smpl/SMPLX_NEUTRAL.pkl  <USER>@<ORIN_IP>:~/a3_teleop_ws/UMR/smpl/
# 校验一致
sha256sum ~/a3_teleop_ws/UMR/smpl/SMPLX_NEUTRAL.pkl   # 两端应一致
```

**A3 checkpoint / ONNX / RKNN** 走官方 HuggingFace 脚本(需要时就地下载):

```bash
cd ~/a3_teleop_ws/sonic_for_a3
.venv_sim/bin/python download_from_hf.py --component pt onnx rknn   # 需要 .venv_sim
find checkpoints/035_step200000 -maxdepth 1 -type f | sort
```

**3.4 就绪门禁(必须 16/16 通过才继续)**

```bash
bash scripts/check_orin_ready.sh
# 检查:解释器 / MJCF+URDF+checkpoint+SMPL-X+分割图+robot config+contract /
#       python 依赖 / zmq loopback / 端口 / **online UMR 能否真的解一帧**
```

> 本机(4090)实测输出:`[ready] 16 ok, 0 failed / ready for the live chain`。

---

## 4. 网络配置(Orin ↔ A3)

```bash
cd ~/a3_teleop_ws/a3_teleop_bridge
$EDITOR configs/network.yaml
```

```yaml
reference:
  bind_host: 0.0.0.0      # Orin 侧:接受局域网订阅
  port: 5560
pico:
  connect_host: 127.0.0.1 # PICO sender 就跑在 Orin 上
  port: 5556
```

建议用直连网线或专用交换机,固定 IP:

```bash
# Orin(示例网段,按现场实际改)
sudo nmcli con mod <conn> ipv4.addresses 192.168.10.10/24 ipv4.method manual
sudo nmcli con up <conn>
ip -4 addr show | grep inet
# A3 侧配置见 A3_ONBOARD.md §3;互 ping 通再继续
ping -c 3 192.168.10.20
```

防火墙(若开了 ufw):`sudo ufw allow 5560/tcp`。

---

## 5. 上真机前:先用 MuJoCo 走一遍 Orin 的流

这是**不动机器人**验证 Orin 全链路的方式(方案 §42/§49):

```bash
# --- Orin:启动 PICO 发送端(XRoboToolkit PC service 必须已连上头显) ---
cd ~/a3_teleop_ws/sonic_for_a3
.venv_sim/bin/python gear_sonic/scripts/pico_pose_zmq_minimal.py --port 5556
# 看到持续输出即 PICO → SMPL 通了;先别连 A3 policy

# --- Orin:第二个终端,启动参考生成(online UMR + predictor + publisher) ---
cd ~/a3_teleop_ws/a3_teleop_bridge && source scripts/env_orin.sh
bash scripts/run_orin_live.sh --duration 600 --endpoint tcp://0.0.0.0:5560 \
     --save-calibration calibration.json
# 首次约 15 s 初始化;状态机 DISCONNECTED → CALIBRATION →(标定后)TRACKING

# --- 4090(或任何能连到 Orin 的机器):MuJoCo 消费这条流 ---
cd ~/a3_teleop_ws/a3_teleop_bridge && source scripts/env_orin.sh
bash scripts/run_mujoco_consumer.sh --endpoint tcp://<ORIN_IP>:5560 \
     --motion ~/a3_teleop_ws/logs/a3_validation/endurance_loop.csv --steps 3000
# 判据:fall=false、root z ≈1.07 m、日志里没有 rejected
```

**没有头显也能验证 Orin**:把 `--source pico` 换成录制文件(源侧不依赖硬件):

```bash
$PY_UMR -m a3_teleop_bridge.apps.retarget_live \
    --source recording --backend umr-online \
    --recording $A3WS/recordings/m5_twist_torso_left \
    --duration 8 --no-publish --playback-hz 30 --stats /tmp/orin_smoke.json
# 实测(4090):frames in=240 solved=182, solver 38.7 ms, rejected=0
```

---

## 6. 实时性门槛与 fallback(方案 §47/§48/§56)

| 指标 | 4090 实测 | Orin 上要重新测(§55) |
| --- | --- | --- |
| online UMR 单帧 | prepare 15.5 ms + solve 34.3 ms ≈ **48 ms(20 Hz)** | 用 `--stats` 的 `solver_latency_ms` |
| 参考发布 | 50 Hz | 同上 |
| SONIC 侧 | 50 Hz 消费,缺失由有界插值补齐,rejected=0 | 同一套代码 |

Orin 实测写入 `benchmarks/orin_umr.json`(同一段录制动作跑 4090/Orin 对比:
逐关节 q 误差 max/mean、p50/p95 延迟、内存与 GPU 占用)。

**若 Orin 上 UMR 达不到实时**,不要改算法,直接用网络解耦的 fallback:

```bash
# UMR 留在 4090(或外部 PC),只把参考流发给 A3
# 4090:
source scripts/env_orin.sh
bash scripts/run_orin_live.sh --endpoint tcp://0.0.0.0:5560 --duration 3600
# A3 侧的 --reference-endpoint 写 4090 的 IP:5560(见 A3_ONBOARD.md)
# Orin 则只做 PICO/SMPL 转发或相机应用
```

这条路径在第一版就是保留能力:协议不变,只改 `connect_host`。

---

## 7. 长跑与故障注入(方案 §43/§49/§50)

```bash
# 30 分钟连续(Orin 侧)
bash scripts/run_orin_live.sh --duration 1800 --stats logs/orin_live/stats.json
# 观察:state_history 不出现 SAFE_STOP;frames_published 随时间线性增长;
#       solver p95 不随时间恶化;内存稳定(htop)
```

故障注入(方案 §80 的 13 项已在 bridge 测试里覆盖,Orin 上复跑):

```bash
$PY_BRIDGE -m pytest tests/test_fault_injection.py tests/test_online_pipeline.py -q
# 手动:Ethernet 拔线 → A3 侧进 HOLD/SAFE_STOP;kill PICO sender → 同上;发 NaN → reject
```

---

## 8. 常见问题

| 现象 | 原因 / 处理 |
| --- | --- |
| `robot config not found` | `UMR_ROOT` 不对,或 `robot_configs/humanoid_retarget_agibot_a3.json` 没拷;`source scripts/env_orin.sh` |
| `${SONIC_A3_ROOT}` 字面量出现在路径报错里 | 没 `source scripts/env_orin.sh`,UMR 无法展开环境变量 |
| 状态机一直 `CALIBRATION` | 没有 session 标定:用 `--save-calibration` 生成后 `--calibration` 载入;或首帧没有真实人体 |
| PICO 连不上 | sender 与 `configs/teleop.yaml` 端口必须一致(**5556**);`ss -ltnp | grep 5556` |
| A3 收不到参考 | `bind_host` 仍是 127.0.0.1;改成 0.0.0.0 并放行防火墙 5560/tcp |
| UMR 报 `trimesh/smplx` 缺失 | 用 `$PY_UMR`(UMR venv)而不是 `$PY_BRIDGE` 跑 online 会话 |
| 单帧从 48 ms 涨到 200 ms | 同一台机器上 SONIC 策略与 UMR 抢 CPU:把 UMR 放 Orin、policy 放 4090(§88 架构) |

---

## 9. 上机记录(每次必写)

```text
阶段 / commit SHA / 执行命令 / 输入 / 输出 / 结果 / 已知问题 / 下一阶段
+ orin_system_info/ 三个文件 + benchmarks/orin_umr.json + sha256(checkpoint)
```

追加到 `a3_teleop_bridge/docs/progress.md`(方案 §0 要求)。
