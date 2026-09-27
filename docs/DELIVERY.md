# DELIVERY.md — 交付清单、GitHub 推送与部署密钥

> 配套文档:[`DEPLOY_ORIN.md`](DEPLOY_ORIN.md)(Orin 部署)、[`A3_ONBOARD.md`](A3_ONBOARD.md)(A3 机载)。

## 1. 交付物

| 文件(本机 `$A3WS/dist/`) | 大小 | 内容 |
| --- | --- | --- |
| `a3_teleop_orin_<stamp>.tar.gz` | 14 MB | 上机包:`a3_teleop_bridge` 全量 + `sonic_for_a3` 代码与 A3 资产 + `UMR` 脚本与配置 + `MANIFEST.txt`(三仓库 SHA) |
| `a3_teleop_bridge.bundle` | 358 KB | bridge 仓库完整 git bundle |
| `sonic_for_a3_feat_incremental.bundle` | 21 KB | 基于 `origin/main` 的增量 bundle(6 个 streaming 接口提交) |
| `UMR_feat_incremental.bundle` | 4 KB | 基于 `origin/main` 的增量 bundle(3 个 A3/online 提交) |

包内自带 `scripts/`(部署脚本),解包即可在 Orin 上直接跑。

重新生成:

```bash
cd $A3WS/a3_teleop_bridge
bash scripts/package_bundle.sh --with-umr
cd $A3WS/a3_teleop_bridge && git bundle create ../dist/a3_teleop_bridge.bundle main
cd $A3WS/sonic_for_a3      && git bundle create ../dist/sonic_for_a3_feat_incremental.bundle \
                                ^origin/main feat/a3-streaming-reference
cd $A3WS/UMR               && git bundle create ../dist/UMR_feat_incremental.bundle \
                                ^origin/main feat/a3-online-retarget
```

包内**故意不含**:`.venv*`、`.git`、checkpoint 权重(`.pt/.onnx/.rknn`)、SMPL-X 人体模型
(许可证资产,必须手工拷贝,见 `DEPLOY_ORIN.md` §3.3)、UMR 其它机器人 mesh(312 MB)、
demo 数据(57 MB)、UMR 生成的含开发机绝对路径的 `*.floating_mjcf.xml`。

## 2. 目标仓库

```text
https://github.com/Restar7/sonic_for_a3        feat/a3-streaming-reference  (已存在,6 commits)
https://github.com/Restar7/a3_teleop_bridge    main                         (需新建空仓库)
https://github.com/hanyang9/UMR                feat/a3-online-retarget      (上游,需你有写权限)
```

上游 `main` / `master` 一律不动(方案 §0 第 1 条)。

## 3. 部署密钥(公钥)

本机(开发/打包机)已生成专用密钥对,用于向上述仓库推送:

```text
私钥:~/.ssh/id_ed25519_github_a3         (仅本机,权限 600,切勿提交或外传)
公钥:~/.ssh/id_ed25519_github_a3.pub
指纹:SHA256:WJ5l9QV0feolYc1LTRAOn0Tjd4G44CBCi/M0E2NQn0o
注释:a3-teleop-orin-deploy
```

**公钥全文(复制这一整行):**

```text
ssh-ed25519 AAAAC3NzaC1lZDI1NTE5AAAAIDrJ4lUXNPn69rNCjgRhkAieGpu3AMAnOzjeAaeTCXPi a3-teleop-orin-deploy
```

添加到 GitHub:**右上头像 → Settings → SSH and GPG keys → New SSH key → 粘贴 → Add SSH key**。

- 加到**账号级 SSH keys**(不要加单个仓库的 Deploy key):因为要推两个(或三个)仓库,
  而同一个公钥只能作为**一个**仓库的 deploy key。
- 加完后本机验证:`ssh -T git@github.com` 应回 `Hi <你的账号>! You've successfully authenticated`。
  当前状态:**已添加并验证通过** —— `Hi Restar7! You've successfully authenticated`。
- 本机已写入 `~/.ssh/config`(`Host github.com` → `IdentityFile ~/.ssh/id_ed25519_github_a3`,
  `IdentitiesOnly yes`),所以 `git push` 无需额外参数。

重新生成/轮换(如需):

```bash
ssh-keygen -t ed25519 -N "" -C "a3-teleop-orin-deploy" -f ~/.ssh/id_ed25519_github_a3
cat ~/.ssh/id_ed25519_github_a3.pub      # 把新的公钥重新贴到 GitHub
```

## 4. 推送命令

密钥加好、且 `Restar7/a3_teleop_bridge` 空仓库建好后:

```bash
# 1) sonic_for_a3:只推我们的 feature branch,不碰 upstream main
cd $A3WS/sonic_for_a3
git push git@github.com:Restar7/sonic_for_a3.git feat/a3-streaming-reference

# 2) a3_teleop_bridge:新建仓库后首推
cd $A3WS/a3_teleop_bridge
git remote add origin git@github.com:Restar7/a3_teleop_bridge.git   # 若已存在则 set-url
git push -u origin main

# 3) UMR(可选,需 hanyang9/UMR 写权限)
cd $A3WS/UMR
git push git@github.com:hanyang9/UMR.git feat/a3-online-retarget
```

## 5. 没有认证时的替代路径(用 bundle)

在任意能访问 GitHub 的机器上:

```bash
# a3_teleop_bridge(新仓库)
git clone -b main a3_teleop_bridge.bundle a3_teleop_bridge   # -b main: bundle 的 HEAD 不指向分支
cd a3_teleop_bridge
git remote set-url origin git@github.com:Restar7/a3_teleop_bridge.git
git push -u origin main

# sonic_for_a3(需要一个已含 origin/main 的克隆)
cd <sonic_for_a3 克隆>
git fetch /path/to/sonic_for_a3_feat_incremental.bundle \
    feat/a3-streaming-reference:feat/a3-streaming-reference
git push origin feat/a3-streaming-reference
```

或者直接用上机包 `tar.gz` 拷到 Orin,不经过 GitHub(见 `DEPLOY_ORIN.md` §2)。

## 6. 推送记录(已完成)

| 仓库 | 分支 | 远端 SHA | 状态 |
| --- | --- | --- | --- |
| https://github.com/Restar7/sonic_for_a3 | `feat/a3-streaming-reference` | `1fcdf7f` | ✅ 已推送(含 LFS 108 objects / 73 MB) |
| https://github.com/Restar7/a3_teleop_bridge | `main` | `109d8b0` | ✅ 已推送(新仓库首推) |
| https://github.com/Restar7/UMR | `feat/a3-online-retarget` | `9341764` | ✅ 已推送(LFS 内容已校验为真实文件);`origin` 仍指向 hanyang9/UMR,新增远端 `fork` |

校验方式(本机实测):

```bash
cd /tmp && git clone --depth 1 -b feat/a3-streaming-reference \
    git@github.com:Restar7/sonic_for_a3.git gh_check
grep -h "ssh-ed25519 AAAA" gh_check/docs/a3_teleop_deployment.md   # 公钥在线可见
git clone --depth 1 git@github.com:Restar7/a3_teleop_bridge.git gh_bridge
ls gh_bridge/docs gh_bridge/scripts                               # 16 份文档 + 8 个脚本
```

`sonic_for_a3` 的 `main` 未被改动(仍为 `fe6868b`,方案 §0 第 1 条)。

**关于 UMR 的两条历史线**(重要):`Restar7/UMR` 与 `hanyang9/UMR` 是**分叉的两条历史**
(`Restar7/UMR:main = d6bb761`,基线是 `8c4db7d Initial public release`;
`hanyang9/UMR:main = c56b630 Add MIT license`,我们的分支基于它)。我们的 3 个提交只碰
`robot_configs/humanoid_retarget_agibot_a3.json`、`scripts/humanoid_retarget_config.py`、
`scripts/retarget_smpl_to_humanoid_surface_vector.py`,**没有引入 LFS 文件**。
因此分支是"基于 hanyang9 线、推到 Restar7 仓库",直接开 PR 到 `Restar7/UMR:main`
会因为两条线的差异而显得很大;若要干净的 PR,需要先把 3 个提交 rebase 到 `d6bb761`
(那会得到一个**未在本机验证过**的树,故本工程默认不做)。
