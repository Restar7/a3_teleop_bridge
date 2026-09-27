# coordinate_frames.md — 坐标系、单位与关节顺序(全部来自源码实测)

> 方案 §0.5/§0.6/§0.7:坐标系、单位、joint order 必须从现有源码验证,禁止猜测;
> 禁止用数组下标假设 joint order。本文件记录**实测结论**与验证方式。

## 1. A3 关节顺序(三种顺序,必须区分!)

| 名称 | 含义 | 来源 | 长度 |
| --- | --- | --- | --- |
| **policy order** | MuJoCo policy 视图(CSV 列序去掉 head) | `A3_CSV_JOINT_NAMES` + `A3_POLICY_TO_SDK_IDX` | 29 |
| **csv order** | flat CSV 列序(含 head) | `A3_CSV_JOINT_NAMES` | 31 |
| **encoder order (`dof_il`)** | SONIC encoder 消费的顺序(= URDF/IsaacLab 顺序) | `load_urdf_actuated_joints` 的 BFS 遍历 | 29 |

三者关系(由 `tools/inspect_a3_contract.py` 自动提取并断言):

```text
policy  = csv 去掉 head 两列(顺序不变)
encoder = policy 的一个真置换(同名关节,顺序完全不同)
```

policy 顺序:

```text
waist_yaw, waist_roll, waist_pitch,
left_shoulder_pitch/roll/yaw, left_elbow, left_wrist_roll/pitch/yaw,
right_shoulder_pitch/roll/yaw, right_elbow, right_wrist_roll/pitch/yaw,
left_hip_pitch/roll/yaw, left_knee, left_ankle_pitch/roll,
right_hip_pitch/roll/yaw, right_knee, right_ankle_pitch/roll
```

encoder 顺序(前 6 个):

```text
left_hip_pitch, right_hip_pitch, waist_yaw, left_hip_roll, right_hip_roll, waist_roll, ...
```

**这是本项目最容易踩的坑**:第一版 streaming 直接按 policy 顺序发布,A3-fast 的
输入被置换,policy 输出饱和到 `|action|=20`,机器人走飞倒地。现在:

* `A3_REFERENCE_V1` 的线上数据 **必须是 encoder 顺序**;
* header 带 `joint_order: a3_il_v1` 与 29 个关节名,接收端校验并逆置换;
* 桥接内部一律用 policy 顺序,只有 `transport/protocol.py` 做一次显式置换。

## 2. 单位

| 量 | 单位 | 证据 |
| --- | --- | --- |
| CSV `root_translateX/Y/Z` | **厘米** | `load_a3_flat_csv` 里 `* 0.01` |
| CSV `root_rotateX/Y/Z` | **度**,外旋 XYZ 欧拉 | `Rotation.from_euler("xyz", deg, degrees=True)` |
| CSV 关节列 | **度** | `np.deg2rad(...)` |
| `A3CanonicalState.root_pos_m` | 米 | 命名 + MJCF |
| 关节角/角速度 | rad / rad·s⁻¹ | — |
| `A3_REFERENCE_V1` | m, rad, s | 协议头 `dtype=<f4` |

## 3. 四元数约定

* MuJoCo free joint:`qpos[3:7] = (w, x, y, z)`;本项目内部统一 **wxyz**。
* CSV 存的是欧拉角,读入时转成 wxyz;写回时用外旋 XYZ 欧拉角。
* 连续性:每帧与上一帧做半球检查(`dot(q_prev, q_new) < 0` 则取反),
  见 `umr/state_converter.py:quaternion_continuity`。
* 预测**不使用欧拉线性外推**:`omega = log(q_prev⁻¹ q_now)/dt`,再用四元数指数
  映射推进(`a3/predictor.py`)。

## 4. root / anchor

* A3 的 root body 与 anchor body 都是 `pelvis_link`(`sim2sim` 启动日志会打印)。
* MJCF 里 free joint 在 `pelvis_link` 上,UML/UMR 结果的 `qpos[0:3]` 是 pelvis 世界位置。
* UMR 的 +Z 向上;SMPL-X rest skeleton 是 **Y-up**,LaFan1 数据在 `poses[:,0]` 里
  带 ~+90° X 旋转把它立到 Z-up 世界(合成验收动作必须照做,否则机器人被重定向到地面,
  见 `progress.md` M3b 记录 1)。

## 5. 双脚/朝向判据

"双脚交叉"必须在 **heading 参考系**里判断:用 root 四元数投影到地面的偏航角,
把双脚位置转进去再比较左右。世界 Y 轴在转身动作里没有意义(这是 M3 阶段
heading 判据修正的原因)。

## 6. 关节限位与方向

* 限位来自 MJCF + URDF + `a3_policy_parameters.hpp`(`tools/extract_a3_joint_limits.py`)。
* 膝:模型允许 `[-0.1222, 2.4958]`,但人类膝盖不反折,因此 UMR 配置把下界裁到
  **0**(`NON_NEGATIVE_JOINTS`);否则 retargeter 会把膝盖压到负下界。
* 左右对称关节(shoulder_roll / hip_roll / ankle_roll)在模型里轴向相同、符号相反,
  任何镜像判据都要用"左右均值反号"而不是"绝对值相等"。
