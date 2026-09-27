# a3_bringup.md — A3 真机 bring-up(严格按序,任一级失败即停)

> 前置条件:`docs/safety.md` 全部满足;MuJoCo 全链路已连续 10 分钟稳定。

## 0. 上机前

```text
[ ] 035 step-200000 checkpoint / ONNX / RKNN 校验通过
[ ] RK3588 部署包在 MDU 上通过 aarch64 预检
[ ] 机载 motion_control 已停止(官方 §6.1 流程,先备份 sm_config.yaml)
[ ] 安全吊带 / 防坠 / 物理急停 / 现场安全员到位
[ ] reference publisher(Orin 或 4090)已在跑,`seq` 持续增长
```

## 1. receive-only(§62)

```text
A3 状态接收 + 参考处理,但**不发布任何电机命令**
检查:joint order / IMU / state freshness / policy inference /
      reference freshness / 网络
```

使用官方 no-command probe(`A3_PROBE_SOURCE=g1`)。

## 2. 悬吊 + 10 级动作(§63/§64)

```text
1  stand
2  shoulder
3  elbow
4  wrist
5  torso small rotation
6  knee small bend
7  weight shift
8  foot unload
9  foot lift
10 slow single step
```

每一级:

```text
观察 30 s → 记录(日志见 §78 字段) → 无异常才进入下一级
任何一级失败:立即 P(passive)/ 物理急停,回到上一级并复查
```

## 3. 参考来源切换

真机验收阶段用 Mode C(离线轨迹 → ZMQ)先跑通网络与安全降级,
再切到 Mode D(PICO live)。切换只改 publisher 的上游,协议与 A3 侧不变。

## 4. 与软件版本绑定

每次上机记录(`docs/progress.md` 追加一条):

```text
阶段 / commit SHA / 执行命令 / 输入 / 输出 / 结果 / 已知问题 / 下一阶段
以及:sonic_for_a3 与 UMR 的 commit、checkpoint sha256、reference 录制文件
```
