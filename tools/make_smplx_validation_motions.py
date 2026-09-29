#!/usr/bin/env python3
"""Build the plan's M3/M4 validation motions as SMPL-X sequences (plan §21).

The plan requires these motions (and explicitly *not* running/jumping/spinning):

    stand, raise left arm, raise right arm, bend knees, twist torso,
    lift left foot, lift right foot, slow forward step

LaFan1 dance clips are a useful stress test but they contain spins and jumps, so
they cannot decide whether the retarget preserves limb sidedness.  This tool
therefore synthesises the required motions directly from the SMPL-X rest skeleton
that ships with sonic_for_a3 (``gear_sonic/data/human/human_joints_info.npz``):

  * every rotation axis is *found empirically*: apply a small rotation to the
    candidate joint, run forward kinematics, and keep the axis that actually
    moves the intended limb the intended way (never guessed by hand);
  * each generated clip is verified afterwards (e.g. "left wrist is above the
    shoulder by more than X while the right wrist is not") and the verification
    result is written next to the npz.

Output (default ``~/a3_teleop_ws/data/smplx_validation/``):
    <name>.npz        poses (T,55,3), trans (T,3), betas, gender, mocap_frame_rate
    <name>.json       verification + provenance

Usage:
    python tools/make_smplx_validation_motions.py
    python tools/make_smplx_validation_motions.py --out DIR --fps 30 --duration 3
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

BRIDGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(BRIDGE_ROOT / "src"))

from a3_teleop_bridge import paths  # noqa: E402
from a3_teleop_bridge.contract import load_contract  # noqa: E402

DEFAULT_OUT = paths.workspace_root() / "data" / "smplx_validation"

# SMPL-22 joint indices
PELVIS, L_HIP, R_HIP, SPINE1, L_KNEE, R_KNEE, SPINE2, L_ANKLE, R_ANKLE, SPINE3, L_FOOT, R_FOOT = range(12)
NECK, L_COLLAR, R_COLLAR, HEAD = 12, 13, 14, 15
L_SHOULDER, R_SHOULDER, L_ELBOW, R_ELBOW, L_WRIST, R_WRIST = 16, 17, 18, 19, 20, 21


#: The shipped rest skeleton is Y-up (head at +Y, ankles at -Y).  The LaFan1
#: sample stores a ~+90 deg rotation about X in ``poses[:, 0]`` to stand the
#: character up in a Z-up world; every FK call below uses the same base so the
#: measured axes and verification metrics are world-frame quantities.
BASE_ROOT_ROTATION = np.array([np.pi / 2.0, 0.0, 0.0])


class SmplFk:
    """Minimal SMPL-X body forward kinematics from the shipped rest joints."""

    def __init__(self, npz_path: Path) -> None:
        data = np.load(npz_path)
        self.rest = np.asarray(data["J"], dtype=np.float64)
        self.parents = np.asarray(data["parents_list"], dtype=np.int64)
        if self.rest.shape != (55, 3) or self.parents.shape != (55,):
            raise SystemExit(f"unexpected rest skeleton in {npz_path}: {self.rest.shape}")
        self.offsets = self.rest.copy()
        self.offsets[1:] -= self.rest[self.parents[1:]]

    def joints(self, body21: np.ndarray, root: np.ndarray) -> np.ndarray:
        pose = np.zeros((55, 3))
        pose[0] = root
        pose[1:22] = np.asarray(body21, dtype=np.float64).reshape(21, 3)
        rot = Rotation.from_rotvec(pose.reshape(-1, 3)).as_matrix()
        out = np.zeros((55, 3))
        world_rot = [None] * 55
        for j in range(55):
            p = int(self.parents[j])
            if p < 0:
                world_rot[j] = Rotation.from_matrix(rot[j])
                out[j] = self.offsets[j]
            else:
                world_rot[j] = world_rot[p] * Rotation.from_matrix(rot[j])
                out[j] = out[p] + world_rot[p].apply(self.offsets[j])
        return out


def find_axis(
    fk: SmplFk,
    joint: int,
    effector: int,
    goal: np.ndarray,
    angle: float = 0.6,
    root: np.ndarray | None = None,
) -> tuple[np.ndarray, float]:
    """Pick the axis (and sign) that best moves ``effector`` along ``goal``.

    Both the pose and the *displacement* are measured with the base global
    orientation applied, so ``goal`` is an honest world-frame direction
    (``+Z`` = up, ``+Y`` = the direction the standing character faces).
    """
    root = BASE_ROOT_ROTATION if root is None else root
    goal = np.asarray(goal, dtype=np.float64)
    goal = goal / max(np.linalg.norm(goal), 1e-12)
    best = (np.zeros(3), -np.inf)
    for axis in np.eye(3):
        for sign in (1.0, -1.0):
            body = np.zeros((21, 3))
            body[joint - 1] = sign * angle * axis
            joints = fk.joints(body, root)
            base = fk.joints(np.zeros(21 * 3), root)
            delta = joints[effector] - base[effector]
            score = float(np.dot(delta, goal))
            if score > best[1]:
                best = (sign * axis, score)
    if best[1] <= 1e-4:
        raise SystemExit(f"no axis moves joint {joint} effector {effector} along {goal}")
    return best


def shoulder_line_azimuth(joints: np.ndarray) -> float:
    return float(
        np.arctan2(
            joints[L_SHOULDER][1] - joints[R_SHOULDER][1],
            joints[L_SHOULDER][0] - joints[R_SHOULDER][0],
        )
    )


def find_twist_axis(fk: SmplFk, joint: int, angle: float = 0.5) -> tuple[np.ndarray, float]:
    """Pick the axis that actually twists the shoulder line about the body axis."""
    base = shoulder_line_azimuth(fk.joints(np.zeros(21 * 3), BASE_ROOT_ROTATION))
    best = (np.zeros(3), -np.inf)
    for axis in np.eye(3):
        for sign in (1.0, -1.0):
            body = np.zeros((21, 3))
            body[joint - 1] = sign * angle * axis
            joints = fk.joints(body, BASE_ROOT_ROTATION)
            delta = abs(float(np.angle(np.exp(1j * (shoulder_line_azimuth(joints) - base)))))
            if delta > best[1]:
                best = (sign * axis, delta)
    if best[1] <= 1e-3:
        raise SystemExit(f"no axis twists the torso at joint {joint}")
    return best


def arm_angle_from_vertical(fk: SmplFk, body: np.ndarray) -> np.ndarray:
    """Angle (deg) between each upper arm and straight down; 0 = hanging."""
    joints = fk.joints(body, BASE_ROOT_ROTATION)
    down = np.array([0.0, 0.0, -1.0])
    out = []
    for shoulder, elbow in ((L_SHOULDER, L_ELBOW), (R_SHOULDER, R_ELBOW)):
        v = joints[elbow] - joints[shoulder]
        out.append(
            float(np.degrees(np.arccos(np.clip(np.dot(v / np.linalg.norm(v), down), -1.0, 1.0))))
        )
    return np.asarray(out)


def natural_stance(fk: SmplFk, left_arm_down: np.ndarray, right_arm_down: np.ndarray,
                   target_deg: float = 12.0) -> tuple[np.ndarray, float]:
    """The all-clip base pose: arms hanging by the sides, not out at the rest pose.

    The SMPL-X rest skeleton stands with the arms out at 74-82 deg from vertical,
    and every clip used to interpolate from exactly that.  The retarget faithfully
    reproduced it (the A3 came out at shoulder_roll 1.545 rad, its T-pose), so the
    whole validation set was "a human holding their arms out" -- nothing like an
    operator standing in front of a headset.  Search the angle that actually puts
    the arms down rather than guessing it.
    """
    best = (None, None, np.inf)
    for angle in np.linspace(0.6, 1.8, 25):
        body = np.zeros((21, 3))
        body[L_SHOULDER - 1] = left_arm_down * angle
        body[R_SHOULDER - 1] = right_arm_down * angle
        measured = arm_angle_from_vertical(fk, body)
        error = float(np.abs(measured - target_deg).max())
        if error < best[2]:
            best = (body, angle, error)
    return best[0], float(best[1])


def smoothstep(t: np.ndarray) -> np.ndarray:
    t = np.clip(t, 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


def _ankle_world_z(fk: SmplFk, poses: np.ndarray, trans: np.ndarray) -> np.ndarray:
    """World-frame z of every joint for the whole clip (``trans`` per frame)."""
    joints = np.stack([fk.joints(frame[1:22], frame[0]) for frame in poses])
    return joints[:, :, 2] + trans[:, 2][:, None]


def build_clip(
    fk: SmplFk,
    frames: int,
    specs: list[tuple[int, np.ndarray, float]],
    hold: float = 0.35,
) -> np.ndarray:
    """Interpolate from neutral to a pose built from (joint, axis, angle) specs."""
    target = np.zeros((21, 3))
    for joint, axis, angle in specs:
        target[joint - 1] += axis * angle
    t = np.linspace(0.0, 1.0, frames)
    ramp = smoothstep((t - (1.0 - hold)) / max(hold, 1e-6)) if hold < 1 else smoothstep(t)
    ramp = smoothstep(t / max(1.0 - hold, 1e-6))
    return target[None, :, :] * ramp[:, None, None]


#: Distance (m) over which a foot stops counting as "planted".  Used to blend
#: between the two feet continuously so the root never jumps when the support
#: foot changes.
SUPPORT_BLEND_M = 0.05

#: Deep-squat joint angles, chosen to land inside the envelope the A3 policy was
#: validated on: the shipped ``043_squat_deep_repeated`` reference reaches a knee
#: of 131 deg and a hip pitch of -103 deg, so a clip that never bends that far
#: cannot claim to have tested a squat.
SQUAT_KNEE_RAD = 2.35
SQUAT_HIP_RAD = 1.15


def support_anchored_root(
    fk: SmplFk,
    body: np.ndarray,
    base_root: np.ndarray,
    root_height: float,
    stance_body: np.ndarray,
    feet: tuple[int, int] = (L_ANKLE, R_ANKLE),
    floor_blend_m: float = SUPPORT_BLEND_M,
) -> np.ndarray:
    """Root translation that keeps the support foot planted on the floor.

    ``trans`` used to be the constant ``[0, 0, root_height]`` for every clip,
    which *pins the pelvis in space*.  Bending the knees then pushed the feet
    down through the floor instead of lowering the hips, so no clip could ever
    express a squat, a jump or a weight shift no matter what the joints did --
    the whole lower-body half of the acceptance set was measuring a frozen root.
    (``data/smplx_validation/*.npz`` had ``trans[:, 2]`` constant to the last
    digit, and so did every converted recording.)

    The root is derived from the pose instead, under the constraint a real body
    satisfies: the lowest foot stays at the height it has in the natural stance.
    Lifting a leg therefore drops nothing, bending both knees lowers the pelvis
    (a squat), and extending the legs raises it (the launch half of a jump).
    The horizontal part keeps the *support* foot where it was, blended smoothly
    between the feet so switching support does not step the root.
    """
    base_root = np.asarray(base_root, dtype=np.float64).reshape(3)
    joints = np.stack([fk.joints(frame, base_root) for frame in np.asarray(body, dtype=np.float64)])
    stance_joints = fk.joints(np.asarray(stance_body, dtype=np.float64), base_root)

    foot_z = joints[:, list(feet), 2]  # (T, 2)
    stance_foot_z = stance_joints[list(feet), 2]

    # vertical: put the lowest foot back on the stance floor height
    trans_z = root_height + (stance_foot_z.min() - foot_z.min(axis=1))

    # horizontal: keep the planted foot under the same spot, soft-weighting the
    # two feet by how close each is to the floor
    scale = max(float(floor_blend_m), 1e-9)
    weights = np.exp(-(foot_z - foot_z.min(axis=1, keepdims=True)) / scale)
    weights = weights / weights.sum(axis=1, keepdims=True)
    stance_weights = np.exp(-(stance_foot_z - stance_foot_z.min()) / scale)
    stance_weights = stance_weights / stance_weights.sum()
    foot_xy = joints[:, list(feet), :2]
    trans_xy = (
        (stance_weights[None, :, None] * stance_joints[list(feet), :2][None]).sum(axis=1)
        - (weights[:, :, None] * foot_xy).sum(axis=1)
    )

    trans = np.zeros((len(joints), 3), dtype=np.float32)
    trans[:, :2] = trans_xy
    trans[:, 2] = trans_z
    return trans


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", default=str(DEFAULT_OUT))
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--duration", type=float, default=3.0, help="seconds per clip")
    parser.add_argument("--sonic-root", default=None)
    args = parser.parse_args(argv)

    contract = load_contract()
    sonic_root = paths.resolve_sonic_root(
        explicit=args.sonic_root, contract_value=contract.raw.get("sonic_root")
    )
    rest_npz = sonic_root / "gear_sonic/data/human/human_joints_info.npz"
    if not rest_npz.is_file():
        raise SystemExit(f"missing rest skeleton: {rest_npz}")
    fk = SmplFk(rest_npz)

    out_dir = Path(args.out).expanduser()
    out_dir.mkdir(parents=True, exist_ok=True)
    frames = int(round(args.duration * args.fps))

    # --- discover the axes we need (never hand-guessed) --------------------
    up = np.array([0.0, 0.0, 1.0])
    forward = np.array([0.0, 1.0, 0.0])  # resolved empirically by the search
    left_arm_up = find_axis(fk, L_SHOULDER, L_WRIST, up)
    right_arm_up = find_axis(fk, R_SHOULDER, R_WRIST, up)
    left_arm_fwd = find_axis(fk, L_SHOULDER, L_WRIST, forward)
    right_arm_fwd = find_axis(fk, R_SHOULDER, R_WRIST, forward)
    # knee flexion swings the ankle backwards (-Y) in a standing pose
    knee_bend = find_axis(fk, L_KNEE, L_ANKLE, -forward)
    # hip flexion lifts the knee forwards/up
    hip_raise = find_axis(fk, L_HIP, L_KNEE, forward)
    # the right leg is a mirror: its axes must be discovered separately, never
    # copied from the left side
    knee_bend_r = find_axis(fk, R_KNEE, R_ANKLE, -forward)
    hip_raise_r = find_axis(fk, R_HIP, R_KNEE, forward)
    twist = find_twist_axis(fk, SPINE2)
    # standing root height: put the ankle at the same world height the LaFan1
    # sample uses (~0.10 m), so UMR's ground alignment sees a standing human
    rest_ankle_z = float(fk.joints(np.zeros(21 * 3), BASE_ROOT_ROTATION)[L_ANKLE][2])
    root_height = 0.10 - rest_ankle_z

    # every clip now departs from a natural stance instead of the T-posed rest
    left_arm_down = -left_arm_up[0]
    right_arm_down = -right_arm_up[0]
    stance, stance_angle = natural_stance(fk, left_arm_down, right_arm_down)
    print(
        f"[motions] natural stance: arm-down rotation {stance_angle:.3f} rad -> "
        f"arm angle {np.round(arm_angle_from_vertical(fk, stance), 1)} deg from vertical "
        f"(rest pose was {np.round(arm_angle_from_vertical(fk, np.zeros((21, 3))), 1)})"
    )

    clips: dict[str, list[tuple[int, np.ndarray, float]]] = {
        "stand": [],
        "raise_left_arm": [(L_SHOULDER, left_arm_up[0], 1.2)],
        "raise_right_arm": [(R_SHOULDER, right_arm_up[0], 1.2)],
        "bend_knees": [(L_KNEE, knee_bend[0], 0.9), (R_KNEE, knee_bend_r[0], 0.9)],
        "squat_deep": [
            (L_HIP, hip_raise[0], SQUAT_HIP_RAD),
            (R_HIP, hip_raise_r[0], SQUAT_HIP_RAD),
            (L_KNEE, knee_bend[0], SQUAT_KNEE_RAD),
            (R_KNEE, knee_bend_r[0], SQUAT_KNEE_RAD),
        ],
        "lift_left_foot": [(L_HIP, hip_raise[0], 0.5), (L_KNEE, knee_bend[0], 0.9)],
        "lift_right_foot": [(R_HIP, hip_raise_r[0], 0.5), (R_KNEE, knee_bend_r[0], 0.9)],
        "twist_torso_left": [(SPINE2, twist[0], 0.5)],
        "twist_torso_right": [(SPINE2, twist[0], -0.5)],
        "step_forward_slow": [
            (L_HIP, hip_raise[0], 0.6),
            (L_KNEE, knee_bend[0], 0.5),
            (R_SHOULDER, right_arm_fwd[0], 0.3),
        ],
    }

    metadata: dict[str, dict] = {}
    for name, specs in clips.items():
        # motion on top of the natural stance (so `stand` really is standing)
        body = build_clip(fk, frames, specs) + stance[None, :, :]
        poses = np.zeros((frames, 55, 3), dtype=np.float32)
        poses[:, 0] = BASE_ROOT_ROTATION.astype(np.float32)
        poses[:, 1:22] = body.astype(np.float32)
        trans = support_anchored_root(fk, body, BASE_ROOT_ROTATION, root_height, stance)

        npz_path = out_dir / f"{name}.npz"
        np.savez(
            npz_path,
            poses=poses,
            trans=trans,
            betas=np.zeros(10, dtype=np.float32),
            gender="neutral",
            mocap_frame_rate=np.float32(args.fps),
        )

        # ---- verification -------------------------------------------------
        joints_end = fk.joints(poses[-1, 1:22], poses[-1, 0])
        joints_start = fk.joints(poses[0, 1:22], poses[0, 0])
        world_end = joints_end + trans[-1]
        world_start = joints_start + trans[0]
        checks = {
            "finite": bool(np.isfinite(poses).all()),
            "frames": int(frames),
            "fps": float(args.fps),
            "root_z_min": float(trans[:, 2].min()),
            "root_z_max": float(trans[:, 2].max()),
            "root_z_span_m": float(trans[:, 2].max() - trans[:, 2].min()),
            "root_xy_range_m": float(np.abs(trans[:, :2] - trans[0, :2]).max()),
            "left_wrist_z_gain": float(world_end[L_WRIST][2] - world_start[L_WRIST][2]),
            "right_wrist_z_gain": float(world_end[R_WRIST][2] - world_start[R_WRIST][2]),
            "left_ankle_forward_gain": float(world_end[L_ANKLE][1] - world_start[L_ANKLE][1]),
            "left_ankle_z_gain": float(world_end[L_ANKLE][2] - world_start[L_ANKLE][2]),
            "right_ankle_forward_gain": float(world_end[R_ANKLE][1] - world_start[R_ANKLE][1]),
            "right_ankle_z_gain": float(world_end[R_ANKLE][2] - world_start[R_ANKLE][2]),
            "left_knee_z_gain": float(world_end[L_KNEE][2] - world_start[L_KNEE][2]),
            "left_ankle_world_z": float(world_start[L_ANKLE][2]),
            "left_ankle_floor_drift_m": float(
                np.abs(_ankle_world_z(fk, poses, trans)[:, L_ANKLE] - _ankle_world_z(fk, poses, trans)[0, L_ANKLE]).max()
            ),
            "torso_twist_deg": float(
                np.rad2deg(
                    np.arctan2(
                        joints_end[L_SHOULDER][1] - joints_end[R_SHOULDER][1],
                        joints_end[L_SHOULDER][0] - joints_end[R_SHOULDER][0],
                    )
                    - np.arctan2(
                        joints_start[L_SHOULDER][1] - joints_start[R_SHOULDER][1],
                        joints_start[L_SHOULDER][0] - joints_start[R_SHOULDER][0],
                    )
                )
            ),
        }
        expectations = {
            "stand": lambda c: abs(c["left_wrist_z_gain"]) < 1e-6 and abs(c["right_wrist_z_gain"]) < 1e-6,
            "raise_left_arm": lambda c: c["left_wrist_z_gain"] > 0.2 and abs(c["right_wrist_z_gain"]) < 1e-6,
            "raise_right_arm": lambda c: c["right_wrist_z_gain"] > 0.2 and abs(c["left_wrist_z_gain"]) < 1e-6,
            "bend_knees": lambda c: c["left_knee_z_gain"] < -0.02
            or c["left_ankle_forward_gain"] < -0.02,
            # a squat is *defined* by the pelvis going down while the feet stay
            # planted; without both, the clip tests nothing about squatting
            "squat_deep": lambda c: c["root_z_span_m"] > 0.25
            and c["left_ankle_floor_drift_m"] < 0.02,
            "lift_left_foot": lambda c: c["left_ankle_z_gain"] > 0.05 or c["left_ankle_forward_gain"] > 0.1,
            "lift_right_foot": lambda c: c["right_ankle_z_gain"] > 0.05
            or c["right_ankle_forward_gain"] > 0.1,
            "twist_torso_left": lambda c: abs(c["torso_twist_deg"]) > 5.0,
            "twist_torso_right": lambda c: abs(c["torso_twist_deg"]) > 5.0,
            "step_forward_slow": lambda c: c["left_ankle_forward_gain"] > 0.05,
        }
        checks["expected_behaviour"] = bool(expectations[name](checks))
        metadata[name] = checks

        (out_dir / f"{name}.json").write_text(
            json.dumps(
                {
                    "name": name,
                    "npz": str(npz_path),
                    "rest_skeleton": str(rest_npz),
                    "specs": [
                        {"joint": int(j), "axis": [float(v) for v in a], "angle_rad": float(ang)}
                        for j, a, ang in specs
                    ],
                    "axes_discovered": {
                        "left_arm_up": [float(v) for v in left_arm_up[0]],
                        "right_arm_up": [float(v) for v in right_arm_up[0]],
                        "knee_bend": [float(v) for v in knee_bend[0]],
                        "knee_bend_right": [float(v) for v in knee_bend_r[0]],
                        "hip_raise": [float(v) for v in hip_raise[0]],
                        "hip_raise_right": [float(v) for v in hip_raise_r[0]],
                        "spine_twist": [float(v) for v in twist[0]],
                    },
                    "checks": checks,
                },
                indent=2,
            )
            + "\n",
            encoding="utf-8",
        )
        print(
            f"{name:20s} frames={frames:4d}  rootz_span={checks['root_z_span_m']:.3f}m "
            f"Lwrist_dz={checks['left_wrist_z_gain']:+.3f} "
            f"Rwrist_dz={checks['right_wrist_z_gain']:+.3f} "
            f"Lankle_dy={checks['left_ankle_forward_gain']:+.3f} "
            f"twist={checks['torso_twist_deg']:+.1f}deg  ok={checks['expected_behaviour']}"
        )

    ok = all(entry["expected_behaviour"] for entry in metadata.values())
    (out_dir / "manifest.json").write_text(
        json.dumps({"clips": metadata, "all_ok": ok}, indent=2) + "\n", encoding="utf-8"
    )
    print(f"\nwrote {len(metadata)} clips to {out_dir}")
    print("RESULT:", "OK" if ok else "SOME CLIPS DID NOT MATCH THEIR EXPECTATION")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
