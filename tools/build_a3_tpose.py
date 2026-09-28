#!/usr/bin/env python3
"""Derive the A3 T-pose used as UMR's robot sample pose (plan section 15).

The pose is *computed from the MJCF*, never typed in by hand:

  * pelvis upright, legs straight/natural, head neutral;
  * both arms in a T-pose: upper arm horizontal along ±Y, elbow straight,
    forearm continuing the upper-arm direction.

The arm solution is found with a bounded least-squares fit over the four arm
joints per side (shoulder pitch/roll/yaw + elbow), starting from the measured
zero pose, and rejected if it leaves the joint limits or misses the T-pose
tolerances.

Passive foot joints (forefoot/toe) are deliberately **not** part of the result:
they must never become optimisation free variables (plan section 15).

Output: ``generated/a3_tpose.json``

Usage:
    python tools/build_a3_tpose.py [--mjcf PATH] [--out PATH]
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import mujoco
import numpy as np
from scipy.optimize import least_squares

BRIDGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(BRIDGE_ROOT / "src"))

from a3_teleop_bridge.a3.limits import load_limits  # noqa: E402
from a3_teleop_bridge.contract import load_contract  # noqa: E402

DEFAULT_OUT = BRIDGE_ROOT / "generated" / "a3_tpose.json"

ARM_JOINTS = {
    "left": (
        "left_shoulder_pitch_joint",
        "left_shoulder_roll_joint",
        "left_shoulder_yaw_joint",
        "left_elbow_joint",
    ),
    "right": (
        "right_shoulder_pitch_joint",
        "right_shoulder_roll_joint",
        "right_shoulder_yaw_joint",
        "right_elbow_joint",
    ),
}
ARM_BODIES = {
    "left": ("left_shoulder_roll_Link", "left_elbow_Link", "left_wrist_roll_Link"),
    "right": ("right_shoulder_roll_Link", "right_elbow_Link", "right_wrist_roll_Link"),
}
SIDE_SIGN = {"left": +1.0, "right": -1.0}

# T-pose acceptance tolerances (metres / radians)
TOL_HORIZONTAL_M = 0.02
TOL_LATERAL_FRACTION = 0.98
TOL_STRAIGHTNESS_RAD = np.deg2rad(3.0)


class Model:
    def __init__(self, mjcf: Path, limits) -> None:
        self.model = mujoco.MjModel.from_xml_path(str(mjcf))
        self.data = mujoco.MjData(self.model)
        self.limits = limits
        self.qpos_adr: dict[str, int] = {}
        hinge = int(mujoco.mjtJoint.mjJNT_HINGE)
        for jid in range(self.model.njnt):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, jid)
            if int(self.model.jnt_type[jid]) == hinge:
                self.qpos_adr[name] = int(self.model.jnt_qposadr[jid])
        self.body_id = {
            mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, bid): bid
            for bid in range(self.model.nbody)
        }

    def forward(self, q: dict[str, float]) -> None:
        self.data.qpos[:] = 0.0
        self.data.qpos[0:7] = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]  # free joint identity
        for name, value in q.items():
            self.data.qpos[self.qpos_adr[name]] = value
        mujoco.mj_forward(self.model, self.data)

    def pos(self, body: str) -> np.ndarray:
        return np.array(self.data.xpos[self.body_id[body]], dtype=np.float64)


def _arm_residual_factory(model: "Model", side: str, names):
    shoulder_b, elbow_b, wrist_b = ARM_BODIES[side]
    sign = SIDE_SIGN[side]
    target = np.array([0.0, sign, 0.0])

    def residuals(x: np.ndarray) -> np.ndarray:
        model.forward(dict(zip(names, x)))
        shoulder = model.pos(shoulder_b)
        elbow = model.pos(elbow_b)
        wrist = model.pos(wrist_b)
        upper_vec = elbow - shoulder
        fore_vec = wrist - elbow
        total = wrist - shoulder
        total_len = np.linalg.norm(total) + 1e-9
        upper_len = np.linalg.norm(upper_vec) + 1e-9
        fore_len = np.linalg.norm(fore_vec) + 1e-9
        # 20x: whole arm sideways; 20x: each segment sideways.
        # 0.02x: only a tie-breaker between kinematically equivalent solutions
        #        (shoulder pitch/yaw and elbow are redundant for a pure T-pose);
        #        it must stay small enough not to pull the arm off the pose.
        return np.concatenate(
            [
                20.0 * (total / total_len - target),
                20.0 * (upper_vec / upper_len - target),
                20.0 * (fore_vec / fore_len - target),
                0.02 * x,
            ]
        )

    return residuals


def solve_arm(model: "Model", side: str, x0: np.ndarray | None = None) -> np.ndarray:
    """Fit the arm into a horizontal T-pose.

    Preferred solution: the arms stay in the **frontal plane** (shoulder pitch
    and yaw untouched; only roll + elbow move), which is what a human T-pose
    looks like and keeps the shoulder chain unrotated.  Only when that family
    cannot reach the T-pose tolerances do we fall back to a full 4-DoF
    shoulder solve (which can trade yaw against elbow).
    """
    names = ARM_JOINTS[side]
    lower = np.array([model.limits.lower[model.limits.joint_names.index(n)] for n in names])
    upper = np.array([model.limits.upper[model.limits.joint_names.index(n)] for n in names])
    lo = np.maximum(lower, -np.pi + 1e-6)
    hi = np.minimum(upper, np.pi - 1e-6)
    sign = SIDE_SIGN[side]
    residuals = _arm_residual_factory(model, side, names)

    def frontal_x(roll: float, elbow: float) -> np.ndarray:
        return np.array([0.0, roll, 0.0, elbow])

    # ---- 1) dense scan of the frontal-plane family -----------------------
    # Only roll + elbow move; the T-pose tolerances are checked on every grid
    # point and the cheapest *valid* point wins.  A plain least-squares solve is
    # not enough here: it happily trades straightness against horizontality and
    # can land just outside the acceptance region.
    scan_roll = np.arange(80.0, 96.0 + 1e-9, 0.25)
    scan_elbow = np.arange(65.0, 100.0 + 1e-9, 0.25)
    candidates: list[tuple[float, np.ndarray, dict]] = []
    for roll_deg in scan_roll:
        roll = sign * np.deg2rad(roll_deg)
        if not (lo[1] <= roll <= hi[1]):
            continue
        for elbow_deg in scan_elbow:
            elbow = np.deg2rad(elbow_deg)
            if not (lo[3] <= elbow <= hi[3]):
                continue
            x = frontal_x(roll, elbow)
            report = verify_arm(model, side, dict(zip(names, x)))
            if not report["ok"]:
                continue
            cost = float(np.sum(residuals(x) ** 2))
            candidates.append((cost, x, report))

    if candidates:
        candidates.sort(key=lambda item: item[0])
        best = candidates[0]
        # local polish around the winning grid point (still frontal-plane only)
        def frontal_residuals(x2: np.ndarray) -> np.ndarray:
            return residuals(np.array([0.0, x2[0], 0.0, x2[1]]))

        delta = np.deg2rad(0.3)
        result = least_squares(
            frontal_residuals,
            x0=np.array([best[1][1], best[1][3]]),
            bounds=(
                np.array([best[1][1] - delta, best[1][3] - delta]),
                np.array([best[1][1] + delta, best[1][3] + delta]),
            ),
            xtol=1e-15,
            ftol=1e-15,
            gtol=1e-15,
            max_nfev=1000,
        )
        polished = np.array([0.0, result.x[0], 0.0, result.x[1]], dtype=np.float64)
        polished_report = verify_arm(model, side, dict(zip(names, polished)))
        if polished_report["ok"] and float(np.sum(residuals(polished) ** 2)) <= best[0]:
            return polished
        return best[1]

    # ---- 2) fallback: full shoulder solve --------------------------------
    starts = [
        np.array([0.0, sign * np.pi / 2, 0.0, np.pi / 2]),
        np.zeros(4),
        np.array([0.0, sign * 0.75, 0.0, 0.9]),
    ]
    if x0 is not None:
        starts.insert(0, np.asarray(x0, dtype=np.float64))
    best_x = None
    best_key = None
    for start in starts:
        result = least_squares(
            residuals,
            x0=np.clip(start, lo, hi),
            bounds=(lo, hi),
            xtol=1e-15,
            ftol=1e-15,
            gtol=1e-15,
            max_nfev=4000,
        )
        x = np.asarray(result.x, dtype=np.float64)
        report = verify_arm(model, side, dict(zip(names, x)))
        # prefer valid solutions, then the smallest joint excursion
        key = (0 if report["ok"] else 1, float(np.linalg.norm(x)))
        if best_key is None or key < best_key:
            best_key = key
            best_x = x
    assert best_x is not None
    return best_x


def verify_arm(model: Model, side: str, angles: dict[str, float]) -> dict:
    shoulder_b, elbow_b, wrist_b = ARM_BODIES[side]
    model.forward(angles)
    shoulder = model.pos(shoulder_b)
    elbow = model.pos(elbow_b)
    wrist = model.pos(wrist_b)
    upper_vec = elbow - shoulder
    fore_vec = wrist - elbow
    total = wrist - shoulder
    total_len = float(np.linalg.norm(total))
    sign = SIDE_SIGN[side]
    lateral = float(total[1] * sign) / (total_len + 1e-9)
    vertical_err = float(abs(total[2]))
    forward_err = float(abs(total[0]))
    straightness = float(
        np.arccos(
            np.clip(
                np.dot(upper_vec, fore_vec) / (np.linalg.norm(upper_vec) * np.linalg.norm(fore_vec) + 1e-12),
                -1.0,
                1.0,
            )
        )
    )
    return {
        "lateral_fraction": lateral,
        "vertical_error_m": vertical_err,
        "forward_error_m": forward_err,
        "straightness_deg": float(np.rad2deg(straightness)),
        "arm_length_m": total_len,
        "shoulder_pos": shoulder.tolist(),
        "elbow_pos": elbow.tolist(),
        "wrist_pos": wrist.tolist(),
        "ok": bool(
            lateral >= TOL_LATERAL_FRACTION
            and vertical_err <= TOL_HORIZONTAL_M
            and forward_err <= TOL_HORIZONTAL_M
            and straightness <= TOL_STRAIGHTNESS_RAD
        ),
    }


def _portable_source(value: Path, sonic_root: Path) -> str:
    """Render a source path relative to the sonic checkout (portable provenance)."""
    try:
        return str(Path(value).relative_to(sonic_root))
    except ValueError:
        return str(value)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mjcf", default=None)
    parser.add_argument("--out", default=str(DEFAULT_OUT))
    args = parser.parse_args(argv)

    contract = load_contract()
    limits = load_limits()
    mjcf = Path(args.mjcf).expanduser() if args.mjcf else contract.mjcf_path
    if not mjcf.is_file():
        raise SystemExit(f"MJCF not found: {mjcf}")

    model = Model(mjcf, limits)

    # ---- zero pose reference -------------------------------------------
    model.forward({})
    zero_report = {}
    for side in ("left", "right"):
        shoulder_b, elbow_b, wrist_b = ARM_BODIES[side]
        zero_report[side] = {
            "shoulder": model.pos(shoulder_b).tolist(),
            "elbow": model.pos(elbow_b).tolist(),
            "wrist": model.pos(wrist_b).tolist(),
        }
    pelvis = model.pos("pelvis_link")
    knee = model.pos("left_knee_Link")
    ankle = model.pos("left_ankle_roll_Link")
    hip = model.pos("left_hip_roll_Link")
    leg_straightness_deg = float(
        np.rad2deg(
            np.arccos(
                np.clip(
                    np.dot(hip - knee, ankle - knee)
                    / (np.linalg.norm(hip - knee) * np.linalg.norm(ankle - knee) + 1e-12),
                    -1.0,
                    1.0,
                )
            )
        )
    )

    tpose: dict[str, float] = {}
    arms: dict[str, dict] = {}
    for side in ("left", "right"):
        names = ARM_JOINTS[side]
        solution = solve_arm(model, side, x0=None)
        angles = {name: float(value) for name, value in zip(names, solution)}
        arms[side] = {"angles": angles, **verify_arm(model, side, angles)}
        tpose.update(angles)

    # legs natural (zero = straight), head neutral, passive feet untouched
    for name in contract.policy_joint_names:
        if name.startswith("left_") or name.startswith("right_"):
            if name.split("_", 1)[1].startswith(("hip", "knee", "ankle")):
                tpose.setdefault(name, 0.0)
    tpose["head_yaw_joint"] = 0.0
    tpose["head_pitch_joint"] = 0.0

    # ---- joint limit check ---------------------------------------------
    limit_violations = []
    for name, value in tpose.items():
        if name not in limits.joint_names:
            continue
        idx = limits.joint_names.index(name)
        if value < limits.lower[idx] - 1e-9 or value > limits.upper[idx] + 1e-9:
            limit_violations.append(
                f"{name}={value:.6f} outside [{limits.lower[idx]:.6f}, {limits.upper[idx]:.6f}]"
            )

    forbidden = set(contract.passive_foot_joint_names)
    leaked = sorted(forbidden & set(tpose))
    problems = list(limit_violations)
    if leaked:
        problems.append(f"passive foot joints must not appear in tpose_qpos: {leaked}")
    for side, report in arms.items():
        if not report["ok"]:
            problems.append(f"{side} arm failed the T-pose tolerances: {report}")

    # T-pose must reproduce exactly when reloaded
    model.forward(tpose)
    roundtrip = {side: verify_arm(model, side, tpose) for side in ("left", "right")}

    doc = {
        "schema": "a3_tpose/v1",
        "generated_by": "tools/build_a3_tpose.py",
        # portable provenance: relative to the sonic checkout, never an absolute path
        "mjcf": _portable_source(mjcf, contract.sonic_root),
        "method": (
            "bounded least-squares over shoulder pitch/roll/yaw + elbow per side, "
            "objective = arm axis along ±Y with straight elbow; legs/head fixed at "
            "the model neutral pose"
        ),
        "tpose_qpos": {k: round(v, 9) for k, v in sorted(tpose.items())},
        "zero_pose_bodies": zero_report,
        "arms": arms,
        "legs": {
            "pose": "model zero (straight)",
            "hip_knee_ankle_deg": leg_straightness_deg,
            "pelvis_z_m": float(pelvis[2]),
            "hip_z_m": float(hip[2]),
            "knee_z_m": float(knee[2]),
            "ankle_z_m": float(ankle[2]),
        },
        "tolerances": {
            "horizontal_m": TOL_HORIZONTAL_M,
            "lateral_fraction": TOL_LATERAL_FRACTION,
            "straightness_deg": float(np.rad2deg(TOL_STRAIGHTNESS_RAD)),
        },
        "roundtrip": roundtrip,
        "excluded_joints": {
            "head": list(contract.head_joint_names),
            "passive_foot": list(contract.passive_foot_joint_names),
            "note": "head kept neutral in UMR qpos but stripped from the A3 policy reference",
        },
        "acceptable": not problems,
        "problems": problems,
    }

    out_path = Path(args.out).expanduser()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(doc, indent=2) + "\n", encoding="utf-8")

    print(f"mjcf: {mjcf}")
    print("zero-pose arm geometry:")
    for side in ("left", "right"):
        z = zero_report[side]
        print(f"  {side:5s} shoulder={np.round(z['shoulder'],4)} elbow={np.round(z['elbow'],4)} wrist={np.round(z['wrist'],4)}")
    print(f"legs: hip-knee-ankle angle {leg_straightness_deg:.3f} deg (0 = straight)")
    print("solved T-pose:")
    for side in ("left", "right"):
        a = arms[side]
        print(f"  {side:5s} lateral={a['lateral_fraction']:.4f} vert_err={a['vertical_error_m']*1000:.2f} mm "
              f"fwd_err={a['forward_error_m']*1000:.2f} mm straight={a['straightness_deg']:.3f} deg "
              f"arm_len={a['arm_length_m']:.4f} m ok={a['ok']}")
        for name, value in a["angles"].items():
            print(f"        {name:30s} = {value:+.6f} rad ({np.rad2deg(value):+.2f} deg)")
    print(f"\nwrote {out_path}")
    if problems:
        print("FAILED:")
        for line in problems:
            print(f"  ! {line}")
        return 1
    print("A3 T-pose ACCEPTED")
    return 0


if __name__ == "__main__":
    sys.exit(main())
