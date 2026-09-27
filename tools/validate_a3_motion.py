#!/usr/bin/env python3
"""Numeric validation of a retargeted A3 motion (plan section 17).

The acceptance list in the plan is visual, but every item has a numeric proxy
that can be checked headlessly:

  * no NaN / no non-finite frames
  * root stays in a plausible height band and never teleports between frames
  * knees bend in the correct direction (no reverse knee)
  * left/right limbs keep their own side (no mirroring, no swapped arms)
  * feet do not cross the body midline and stay above the ground plane
  * joint limits are respected (hard requirement)
  * joint velocity and acceleration stay inside the configured bounds

Usage:
    python tools/validate_a3_motion.py --umr-result PATH.npz [--json OUT]
    python tools/validate_a3_motion.py --csv PATH.csv --fps 30
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

BRIDGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(BRIDGE_ROOT / "src"))

from a3_teleop_bridge.a3.limits import load_limits  # noqa: E402
from a3_teleop_bridge.contract import load_contract  # noqa: E402


def foot_and_body_series(robot_xml: Path, qpos: np.ndarray, body_names: list[str]) -> dict[str, np.ndarray]:
    import mujoco

    model = mujoco.MjModel.from_xml_path(str(robot_xml))
    data = mujoco.MjData(model)
    ids = {}
    for name in body_names:
        bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
        if bid < 0:
            raise SystemExit(f"body {name!r} not found in {robot_xml}")
        ids[name] = bid
    if qpos.shape[1] != model.nq:
        raise SystemExit(f"qpos width {qpos.shape[1]} != model nq {model.nq}")
    out = {name: np.zeros((qpos.shape[0], 3)) for name in body_names}
    for i in range(qpos.shape[0]):
        data.qpos[:] = qpos[i]
        mujoco.mj_forward(model, data)
        for name, bid in ids.items():
            out[name][i] = data.xpos[bid]
    return out


def heading_yaw(quat_wxyz: np.ndarray) -> np.ndarray:
    """Yaw of the root *heading* (projection of the body x-axis on the ground)."""
    q = np.asarray(quat_wxyz, dtype=np.float64)
    norms = np.linalg.norm(q, axis=1, keepdims=True)
    q = q / np.where(norms > 1e-12, norms, 1.0)
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    # body x-axis in world coordinates
    fx = 1.0 - 2.0 * (y * y + z * z)
    fy = 2.0 * (x * y + w * z)
    return np.arctan2(fy, fx)


def rotate_into_heading_frame(
    points: np.ndarray, root_quat_wxyz: np.ndarray
) -> np.ndarray:
    """Rotate world-frame offsets into the root *heading* (yaw-only) frame.

    A turning or leaning motion makes world Y meaningless and a full body-frame
    projection noisy, so "feet crossed" is evaluated against the robot's own
    lateral axis projected on the ground plane.
    """
    yaw = heading_yaw(root_quat_wxyz)
    c, s = np.cos(-yaw), np.sin(-yaw)
    out = np.empty_like(points)
    out[:, 0] = c * points[:, 0] - s * points[:, 1]
    out[:, 1] = s * points[:, 0] + c * points[:, 1]
    out[:, 2] = points[:, 2]
    return out


def validate(
    joint_pos: np.ndarray,
    joint_vel: np.ndarray,
    root_pos: np.ndarray,
    root_quat: np.ndarray,
    dt: float,
    bodies: dict[str, np.ndarray] | None = None,
    ground_z: float = 0.0,
) -> dict:
    contract = load_contract()
    limits = load_limits()
    names = list(contract.policy_joint_names)
    idx = {name: i for i, name in enumerate(names)}

    report: dict = {"frames": int(joint_pos.shape[0]), "dt": dt, "problems": [], "checks": {}}

    def check(name: str, ok: bool, detail: str) -> None:
        report["checks"][name] = {"ok": bool(ok), "detail": detail}
        if not ok:
            report["problems"].append(f"{name}: {detail}")

    finite = bool(
        np.isfinite(joint_pos).all()
        and np.isfinite(joint_vel).all()
        and np.isfinite(root_pos).all()
        and np.isfinite(root_quat).all()
    )
    check("finite", finite, "all arrays finite" if finite else "non-finite values present")

    # ---- root ---------------------------------------------------------
    heights = root_pos[:, 2]
    check(
        "root_height_band",
        bool(heights.min() > 0.4 and heights.max() < 1.6),
        f"root z in [{heights.min():.3f}, {heights.max():.3f}] m",
    )
    steps = np.linalg.norm(np.diff(root_pos, axis=0), axis=1) if root_pos.shape[0] > 1 else np.zeros(0)
    max_step = float(steps.max()) if steps.size else 0.0
    check("root_no_teleport", max_step < 0.25, f"max per-frame root step {max_step:.3f} m")

    quat_norms = np.linalg.norm(root_quat, axis=1)
    check(
        "root_quat_unit",
        bool(np.all(np.abs(quat_norms - 1.0) < 1e-3)),
        f"|quat| in [{quat_norms.min():.6f}, {quat_norms.max():.6f}]",
    )

    # ---- limits -------------------------------------------------------
    lower = limits.lower
    upper = limits.upper
    excess = np.maximum(lower - joint_pos.min(axis=0), joint_pos.max(axis=0) - upper)
    worst = float(excess.max()) if excess.size else 0.0
    check("joint_limits", worst <= 1e-6, f"worst limit excess {worst:.3e} rad")

    # ---- knees --------------------------------------------------------
    for side in ("left", "right"):
        key = f"{side}_knee_joint"
        if key in idx:
            values = joint_pos[:, idx[key]]
            check(
                f"{key}_direction",
                bool(values.min() >= -1e-6),
                f"knee range [{values.min():+.3f}, {values.max():+.3f}] rad (negative = reverse knee)",
            )

    # ---- left/right sanity (no mirroring) ------------------------------
    for suffix in ("shoulder_roll_joint", "hip_roll_joint", "ankle_roll_joint"):
        left, right = f"left_{suffix}", f"right_{suffix}"
        if left in idx and right in idx:
            lv = joint_pos[:, idx[left]]
            rv = joint_pos[:, idx[right]]
            # roll axes point the same way in the model, so the mirror invariant is
            # that the mean signs are opposite
            check(
                f"{suffix}_mirror",
                bool(np.sign(np.mean(lv)) != np.sign(np.mean(rv)) or abs(np.mean(lv)) < 1e-6),
                f"mean left {np.mean(lv):+.3f} vs right {np.mean(rv):+.3f} rad",
            )

    # ---- velocity / acceleration --------------------------------------
    if joint_pos.shape[0] > 2 and dt > 0:
        vel = np.diff(joint_pos, axis=0) / dt
        acc = np.diff(vel, axis=0) / dt
        vel_max = float(np.abs(vel).max())
        acc_max = float(np.abs(acc).max())
        check(
            "joint_velocity",
            vel_max <= float(limits.velocity.max()) * 1.05,
            f"max |dq| {vel_max:.2f} rad/s (hardware max {limits.velocity.max():.2f})",
        )
        check(
            "joint_acceleration",
            acc_max <= 5000.0,
            f"max |ddq| {acc_max:.1f} rad/s^2",
        )
        report["velocity_max_rad_s"] = vel_max
        report["acceleration_max_rad_s2"] = acc_max

    # ---- feet (needs FK) ----------------------------------------------
    if bodies:
        left_foot = bodies.get("left_ankle_roll_Link")
        right_foot = bodies.get("right_ankle_roll_Link")
        pelvis = bodies.get("pelvis_link")
        if left_foot is not None and right_foot is not None and pelvis is not None:
            # Express the feet in the *root heading frame*: a turning motion makes
            # world-frame Y meaningless, while the robot's own lateral axis is what
            # "feet crossed" actually means.
            local_left = rotate_into_heading_frame(left_foot - pelvis, root_quat)
            local_right = rotate_into_heading_frame(right_foot - pelvis, root_quat)
            separation = local_left[:, 1] - local_right[:, 1]
            worst = int(np.argmin(separation))
            crossing_frames = int(np.count_nonzero(separation < -0.02))
            check(
                "feet_not_crossed",
                crossing_frames == 0,
                f"min heading-frame separation {separation.min():+.3f} m at frame {worst}; "
                f"{crossing_frames} frames below -0.02 m",
            )
            report["foot_separation_series_min_frame"] = worst
            above = np.minimum(left_foot[:, 2], right_foot[:, 2]) - ground_z
            check(
                "feet_above_ground",
                bool(above.min() > -0.05),
                f"lowest ankle above ground {above.min():+.3f} m",
            )
            report["foot_separation_min_m"] = float(separation.min())
            report["foot_height_min_m"] = float(above.min())
            report["foot_height_max_m"] = float(above.max())

    report["acceptable"] = not report["problems"]
    return report


def load_inputs(args) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, float, Path | None]:
    if args.umr_result:
        from a3_teleop_bridge.umr.offline import load_umr_result

        result = load_umr_result(args.umr_result)
        contract = load_contract()
        joint_pos = np.asarray(
            [[result.joint_value(i, n) for n in contract.policy_joint_names] for i in range(result.n_frames)]
        )
        dt = result.dt
        joint_vel = np.zeros_like(joint_pos)
        if result.n_frames > 1:
            joint_vel[1:] = np.diff(joint_pos, axis=0) / dt
            joint_vel[0] = joint_vel[1]
        return joint_pos, joint_vel, result.root_pos, result.root_quat_wxyz, dt, result.robot_xml

    from a3_teleop_bridge.a3.csv_export import read_a3_flat_csv

    data = read_a3_flat_csv(args.csv, source_fps=args.fps, frame_stride=1)
    dt = 1.0 / data.effective_fps
    joint_vel = np.zeros_like(data.dof29_rad)
    if data.n_frames > 1:
        joint_vel[1:] = np.diff(data.dof29_rad, axis=0) / dt
        joint_vel[0] = joint_vel[1]
    return data.dof29_rad, joint_vel, data.root_pos_m, data.root_quat_wxyz, dt, None


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--umr-result", default=None)
    parser.add_argument("--csv", default=None)
    parser.add_argument("--fps", type=float, default=30.0, help="csv fps (stride is always 1)")
    parser.add_argument("--json", dest="json_out", default=None)
    parser.add_argument("--no-fk", action="store_true", help="skip the MuJoCo foot checks")
    args = parser.parse_args(argv)
    if not args.umr_result and not args.csv:
        parser.error("pass --umr-result or --csv")

    joint_pos, joint_vel, root_pos, root_quat, dt, robot_xml = load_inputs(args)
    bodies = None
    if not args.no_fk:
        xml = robot_xml
        if xml is None:
            xml = load_contract().mjcf_path
        # FK needs a full qpos; only the UMR result carries one (the CSV holds the
        # 29-DoF policy view, which is not enough to place the passive foot bodies)
        if args.umr_result:
            from a3_teleop_bridge.umr.offline import load_umr_result

            result = load_umr_result(args.umr_result)
            bodies = foot_and_body_series(
                Path(xml),
                result.qpos,
                ["pelvis_link", "left_ankle_roll_Link", "right_ankle_roll_Link"],
            )

    report = validate(joint_pos, joint_vel, root_pos, root_quat, dt, bodies=bodies)
    report["source"] = args.umr_result or args.csv
    if args.json_out:
        out = Path(args.json_out).expanduser()
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")

    print(f"source : {report['source']}")
    print(f"frames : {report['frames']} @ dt {dt:.6f} s")
    for name, entry in report["checks"].items():
        flag = "ok  " if entry["ok"] else "FAIL"
        print(f"  [{flag}] {name:28s} {entry['detail']}")
    if args.json_out:
        print(f"report : {args.json_out}")
    print("RESULT:", "OK" if report["acceptable"] else "PROBLEMS FOUND")
    return 0 if report["acceptable"] else 1


if __name__ == "__main__":
    sys.exit(main())
