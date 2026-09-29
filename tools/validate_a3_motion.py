#!/usr/bin/env python3
"""Numeric validation of a retargeted A3 motion (plan section 17).

The acceptance list in the plan is visual, but every item has a numeric proxy
that can be checked headlessly:

  * no NaN / no non-finite frames
  * root stays in a plausible height band and never teleports between frames
  * knees bend in the correct direction (no reverse knee)
  * left/right limbs keep their own side: the mirrored roll pairs are checked
    against their *reference-relative* excursion (a joint sign/convention check,
    not a posture-symmetry requirement -- see the note in ``validate``)
  * feet do not cross the body midline and stay above the ground plane
  * joint limits are respected (hard requirement)
  * joint velocity and acceleration stay inside the configured bounds

Usage:
    python tools/validate_a3_motion.py --umr-result PATH.npz [--json OUT]
    python tools/validate_a3_motion.py --umr-result PATH.npz --neutral-npz STAND.npz
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
    neutral: dict[str, float] | None = None,
    neutral_label: str = "clip frame 0 (retarget pose-init)",
) -> dict:
    contract = load_contract()
    limits = load_limits()
    names = list(contract.policy_joint_names)
    idx = {name: i for i, name in enumerate(names)}

    report: dict = {
        "frames": int(joint_pos.shape[0]),
        "dt": dt,
        "problems": [],
        "diagnostics": [],
        "checks": {},
    }

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
    # The A3 knee model itself allows a small negative range (lower limit
    # -0.1222 rad), so "reverse knee" is judged against a physical tolerance
    # rather than against exactly zero.
    knee_tolerance = 0.02  # rad (~1.1 deg)
    for side in ("left", "right"):
        key = f"{side}_knee_joint"
        if key in idx:
            values = joint_pos[:, idx[key]]
            lower_limit = limits.lower[idx[key]]
            check(
                f"{key}_direction",
                bool(values.min() >= -knee_tolerance),
                f"knee range [{values.min():+.4f}, {values.max():+.4f}] rad "
                f"(model lower limit {lower_limit:+.4f}, tolerance {-knee_tolerance:+.2f})",
            )

    # ---- how much of each joint's range the clip actually uses ----------
    # Range utilisation is reported, never failed: a clip is allowed to leave a
    # joint alone.  It is what makes a saturated solver visible instead of
    # silently green -- a knee that sits at exactly one value for the whole clip
    # is the signature of a retargeter whose preferred direction is on the far
    # side of a clipped lower bound, not of a motion that chose to keep the knee
    # straight.
    excursion = joint_pos.max(axis=0) - joint_pos.min(axis=0)
    report["joint_excursion_rad"] = {
        name: round(float(excursion[i]), 5) for i, name in enumerate(names)
    }
    if joint_pos.shape[0] > 2:
        pinned_atol = 1e-6
        pinned_low = (np.abs(joint_pos - lower) <= pinned_atol).mean(axis=0) > 0.95
        pinned_high = (np.abs(joint_pos - upper) <= pinned_atol).mean(axis=0) > 0.95
        for i, name in enumerate(names):
            if pinned_low[i] or pinned_high[i]:
                bound = "lower" if pinned_low[i] else "upper"
                value = float(lower[i] if pinned_low[i] else upper[i])
                report["diagnostics"].append(
                    f"{name}: pinned on its {bound} limit ({value:+.4f} rad) for "
                    f">95% of the clip (excursion {excursion[i]:.5f} rad) -- check "
                    f"whether the retarget solver is saturated against a clipped "
                    f"bound rather than tracking the motion"
                )
    knee_usage = {}
    for side in ("left", "right"):
        key = f"{side}_knee_joint"
        if key not in idx:
            continue
        used = float(excursion[idx[key]])
        knee_usage[key] = round(used, 5)
        if used < 0.02:
            report["diagnostics"].append(
                f"{key}: flexes by only {used:.5f} rad over the whole clip "
                f"(held at {joint_pos[0, idx[key]]:+.4f} rad of a "
                f"[{limits.lower[idx[key]]:+.4f}, {limits.upper[idx[key]]:+.4f}] rad range) "
                f"-- verify the source motion really is knee-neutral before accepting it"
            )
    report["knee_excursion_rad"] = knee_usage

    # ---- left/right sanity (no mirroring) ------------------------------
    # "mirror" here is a *joint sign convention* check, not a posture-symmetry
    # requirement.  Two facts about the A3 make a raw whole-clip mean unusable
    # for it:
    #
    #   * the shipped neutral pose is not roll-symmetric.  The retarget
    #     pose-init sits at left_hip_roll -0.0469 rad / right_hip_roll
    #     -0.0179 rad -- both negative -- and the ``stand`` clip is constant at
    #     exactly those values, so every raw clip mean is dominated by a static
    #     offset that carries no information about the motion at all.  With a
    #     0.05 rad significance threshold the left offset alone consumes 94% of
    #     the budget: 3 mrad of drift used to trip the test.
    #   * inherently one-sided motions (lift one foot, raise one arm, torso
    #     twist, slow step) add a common-mode lateral shift on top of any
    #     mirror-symmetric component, and a trajectory cannot be asked to
    #     separate the two on its own.
    #
    # So the test is evaluated on the *reference-relative* excursion
    # ``d(t) = q(t) - q(reference)`` -- the reference is the clip's own neutral
    # pose-init frame unless ``--neutral-npz`` supplies an explicit neutral --
    # and it only hard-fails on the signature a real sign/mirror bug leaves:
    # both limbs of a mirrored pair sweeping *grossly* in the *same* world
    # direction.  Note the A3 roll ranges are themselves mirrored and strongly
    # asymmetric (hip: left [-0.524, +1.606] / right [-1.606, +0.524]), so a
    # sign swap on any motion with real amplitude also breaks ``joint_limits``
    # independently.  Below the gross bound the left/right sign reading is
    # reported as a non-fatal diagnostic instead.  Everything genuinely
    # physical (limits, velocity, acceleration, feet) stays a hard failure.
    mirror_significance = 0.05  # rad: below this the sign test carries no information
    mirror_gross = 0.25  # rad (~14 deg): past what a balance shift can explain
    neutral = neutral or {}
    for suffix in ("shoulder_roll_joint", "hip_roll_joint", "ankle_roll_joint"):
        left, right = f"left_{suffix}", f"right_{suffix}"
        if left not in idx or right not in idx:
            continue
        left_series = joint_pos[:, idx[left]]
        right_series = joint_pos[:, idx[right]]
        ref_left = float(neutral[left]) if left in neutral else float(left_series[0])
        ref_right = float(neutral[right]) if right in neutral else float(right_series[0])
        d_left = left_series - ref_left
        d_right = right_series - ref_right
        amp_left = float(np.abs(d_left).max())
        amp_right = float(np.abs(d_right).max())
        amp = max(amp_left, amp_right)
        peak_left = float(d_left[int(np.argmax(np.abs(d_left)))])
        peak_right = float(d_right[int(np.argmax(np.abs(d_right)))])
        # split the motion into the common-mode (lateral shift) and the
        # anti-symmetric (scissor) component so the numbers are auditable
        common = 0.5 * (float(d_left.mean()) + float(d_right.mean()))
        scissor = 0.5 * (float(d_left.mean()) - float(d_right.mean()))
        detail = (
            f"neutral-relative excursion {amp_left:.4f}/{amp_right:.4f} rad "
            f"(reference {ref_left:+.4f}/{ref_right:+.4f} from {neutral_label}); "
            f"common-mode {common:+.4f} scissor {scissor:+.4f} rad"
        )
        report.setdefault("mirror_reference", {})[suffix] = {
            "reference": neutral_label,
            "left": ref_left,
            "right": ref_right,
            "excursion_left_rad": amp_left,
            "excursion_right_rad": amp_right,
            "common_mode_rad": common,
            "scissor_rad": scissor,
        }
        same_direction = bool(np.sign(peak_left) == np.sign(peak_right))
        gross_swap = (
            amp > mirror_gross
            and min(abs(peak_left), abs(peak_right)) > mirror_gross
            and same_direction
        )
        if amp < mirror_significance:
            check(
                f"{suffix}_mirror",
                True,
                f"n/a: {detail}; below the {mirror_significance} rad significance threshold",
            )
        elif gross_swap:
            check(
                f"{suffix}_mirror",
                False,
                f"both limbs sweep {peak_left:+.3f}/{peak_right:+.3f} rad in the SAME "
                f"world direction (> {mirror_gross} rad): the left/right sign "
                f"convention is broken, not a balance shift; {detail}",
            )
        else:
            check(f"{suffix}_mirror", True, detail)
            if same_direction:
                report["diagnostics"].append(
                    f"{suffix}_mirror: left/right excursions share a world direction "
                    f"({peak_left:+.4f}/{peak_right:+.4f} rad) but stay inside the "
                    f"{mirror_gross} rad gross bound -- consistent with the common-mode "
                    f"({common:+.4f} rad) lateral shift of an inherently one-sided "
                    f"motion, not a mirror error"
                )

    # ---- bound saturation -------------------------------------------------
    # An existing check above only fires when a joint is pinned for >95% of the
    # clip.  The ankle fails in a wider band than that: the posture prior
    # (``solver.joint_map_cost``) covers the knee and NOT the ankle, so the
    # surface objective parks it against whichever stop is nearest while still
    # showing a large excursion.  Measured 2026-09-29, the left ankle sits on its
    # -52 deg stop for 43-75% of every clip that bends the left leg (bend_knees
    # 64/150, lift_left_foot 73/150, squat_deep 112/150, press_pedal 125/249)
    # against an official reference that never goes below -38.6 deg -- and the
    # suite still reported 11/11 PASS.  Reported, not failed: the point is to
    # stop "all PASS" from reading as "the ankle is fine".
    for i, name in enumerate(names):
        span = float(limits.upper[i]) - float(limits.lower[i])
        if span <= 0.0:
            continue
        column = joint_pos[:, i]
        at_low = float(((column - float(limits.lower[i])) <= 0.02 * span).mean())
        at_high = float(((float(limits.upper[i]) - column) <= 0.02 * span).mean())
        fraction, bound, value = max(
            (at_low, "lower", float(limits.lower[i])),
            (at_high, "upper", float(limits.upper[i])),
        )
        if 0.20 <= fraction <= 0.95:
            report["diagnostics"].append(
                f"{name}: spends {fraction * 100:.0f}% of the clip against its {bound} "
                f"stop ({value:+.3f} rad) but still moves "
                f"{float(column.max() - column.min()):.3f} rad -- saturation, not a "
                f"pinned joint; the retarget is riding the bound instead of tracking"
                + (
                    " (the ankle has no posture prior; see runbook 17.8.6)"
                    if "ankle" in name
                    else ""
                )
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


def load_neutral_reference(path: str | Path, names: list[str]) -> dict[str, float]:
    """Mean joint pose of an explicit neutral clip (e.g. the ``stand`` motion)."""
    from a3_teleop_bridge.umr.offline import load_umr_result

    result = load_umr_result(path)
    series = np.asarray(
        [[result.joint_value(i, n) for n in names] for i in range(result.n_frames)]
    )
    return {name: float(series[:, i].mean()) for i, name in enumerate(names)}


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--umr-result", default=None)
    parser.add_argument("--csv", default=None)
    parser.add_argument("--fps", type=float, default=30.0, help="csv fps (stride is always 1)")
    parser.add_argument("--json", dest="json_out", default=None)
    parser.add_argument("--no-fk", action="store_true", help="skip the MuJoCo foot checks")
    parser.add_argument(
        "--neutral-npz",
        default=None,
        help="explicit neutral clip for the left/right mirror reference "
        "(default: the validated clip's own frame 0)",
    )
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

    neutral = None
    neutral_label = "clip frame 0 (retarget pose-init)"
    if args.neutral_npz:
        neutral = load_neutral_reference(args.neutral_npz, list(load_contract().policy_joint_names))
        neutral_label = f"mean of {Path(args.neutral_npz).name}"

    report = validate(
        joint_pos,
        joint_vel,
        root_pos,
        root_quat,
        dt,
        bodies=bodies,
        neutral=neutral,
        neutral_label=neutral_label,
    )
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
    for note in report.get("diagnostics", []):
        print(f"  [warn] {note}")
    if args.json_out:
        print(f"report : {args.json_out}")
    print("RESULT:", "OK" if report["acceptable"] else "PROBLEMS FOUND")
    return 0 if report["acceptable"] else 1



if __name__ == "__main__":
    sys.exit(main())
