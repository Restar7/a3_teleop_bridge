#!/usr/bin/env python3
"""Build a long, *continuous* A3 clip for endurance testing (plan section 43).

Concatenating the validation clips directly creates reference jumps (each clip
ends in a different pose than the next one starts), and a jumping reference is a
fault condition, not a valid endurance input.  This tool cross-fades consecutive
clips so the trajectory stays continuous, then repeats the sequence.

Usage:
    python tools/make_endurance_clip.py --out ~/a3_teleop_ws/logs/a3_validation/endurance_loop.csv
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

BRIDGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(BRIDGE_ROOT / "src"))

from a3_teleop_bridge.a3.csv_export import A3FlatCsvCodec, write_a3_flat_csv  # noqa: E402

CLIP_ORDER = (
    "stand",
    "raise_left_arm",
    "raise_right_arm",
    "bend_knees",
    "twist_torso_left",
    "twist_torso_right",
    "lift_left_foot",
    "lift_right_foot",
    "step_forward_slow",
    "stand",
)


def smoothstep(t: np.ndarray) -> np.ndarray:
    t = np.clip(t, 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--clips-dir", default=str(BRIDGE_ROOT.parent / "logs" / "a3_validation"))
    parser.add_argument("--out", default=str(BRIDGE_ROOT.parent / "logs" / "a3_validation" / "endurance_loop.csv"))
    parser.add_argument("--repeats", type=int, default=8)
    parser.add_argument("--blend-frames", type=int, default=20)
    parser.add_argument("--fps", type=float, default=30.0)
    args = parser.parse_args(argv)

    codec = A3FlatCsvCodec()
    clips_dir = Path(args.clips_dir).expanduser()
    clips = []
    for name in CLIP_ORDER:
        path = clips_dir / name / f"{name}.csv"
        if not path.is_file():
            raise SystemExit(f"missing clip: {path}")
        clips.append(codec.read(path, source_fps=args.fps, frame_stride=1))

    blend = max(1, int(args.blend_frames))
    root_parts, quat_parts, dof_parts = [], [], []
    from scipy.spatial.transform import Rotation, Slerp

    for rep in range(args.repeats):
        for idx, clip in enumerate(clips):
            root = clip.root_pos_m.copy()
            quat = clip.root_quat_wxyz.copy()
            dof = clip.dof29_rad.copy()
            if root_parts and blend > 1:
                # cross-fade the first `blend` frames with the running tail
                prev_root = root_parts[-1][-1]
                prev_quat = quat_parts[-1][-1]
                prev_dof = dof_parts[-1][-1]
                w = smoothstep(np.linspace(0.0, 1.0, blend))[:, None]
                root[:blend] = (1.0 - w) * prev_root + w * root[:blend]
                dof[:blend] = (1.0 - w) * prev_dof + w * dof[:blend]
                q_prev = Rotation.from_quat(np.tile(prev_quat[[1, 2, 3, 0]], (blend, 1)))
                q_new = Rotation.from_quat(quat[:blend][:, [1, 2, 3, 0]])
                slerp = Slerp([0.0, 1.0], Rotation.concatenate([q_prev[:1], q_new[-1:]]))
                quat[:blend] = slerp(np.linspace(0.0, 1.0, blend)).as_quat()[:, [3, 0, 1, 2]]
            root_parts.append(root)
            quat_parts.append(quat)
            dof_parts.append(dof)

    root = np.concatenate(root_parts)
    quat = np.concatenate(quat_parts)
    dof = np.concatenate(dof_parts)
    root[:, 0] -= root[0, 0]
    root[:, 1] -= root[0, 1]

    out = Path(args.out).expanduser()
    write_a3_flat_csv(out, root, quat, dof)

    # report the largest reference jumps so the continuity claim is checkable
    dq = np.abs(np.diff(dof, axis=0)).max(axis=1)
    dpos = np.linalg.norm(np.diff(root, axis=0), axis=1)
    print(f"wrote {out}")
    print(f"  frames        : {root.shape[0]} ({root.shape[0] / args.fps:.1f} s, "
          f"{root.shape[0] / args.fps * 50:.0f} policy steps)")
    print(f"  max joint step: {dq.max():.4f} rad (p99 {np.percentile(dq, 99):.4f})")
    print(f"  max root step : {dpos.max():.4f} m  (p99 {np.percentile(dpos, 99):.4f})")
    print(f"  blend frames  : {blend} per boundary")
    return 0


if __name__ == "__main__":
    sys.exit(main())
