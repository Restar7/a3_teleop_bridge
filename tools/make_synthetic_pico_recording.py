#!/usr/bin/env python3
"""Build a realistic synthetic PICO recording from a SMPL-X clip (plan §27/M5).

``apps/record_pico.py --synthetic`` produces a placeholder body (a single moving
point), which is fine for exercising the recorder but useless for retargeting:
UMR needs a real body surface.  This tool instead replays an existing SMPL-X clip
through the shipped rest skeleton and packs the result the way the SONIC PICO
sender does:

    smpl_joints : 24x3, root-relative (the sender publishes local joints)
    smpl_pose   : 21x3 body axis-angle
    body_quat_w : the root orientation
    root_translation: the world root position (so the motion is faithful)

Usage:
    python tools/make_synthetic_pico_recording.py \
        --clip ~/a3_teleop_ws/data/smplx_validation/lift_left_foot.npz \
        --out  ~/a3_teleop_ws/recordings/m5_lift_left_foot
"""

from __future__ import annotations

import argparse
import json
import sys
from datetime import datetime
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

BRIDGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(BRIDGE_ROOT / "src"))
sys.path.insert(0, str(BRIDGE_ROOT / "tools"))

from a3_teleop_bridge.contract import load_contract  # noqa: E402
from a3_teleop_bridge.pico.recorder import PicoRecording  # noqa: E402
from a3_teleop_bridge.types import HumanSmplFrame  # noqa: E402
from make_smplx_validation_motions import SmplFk  # noqa: E402


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--clip", required=True, help="SMPL-X .npz (poses/trans)")
    parser.add_argument("--out", required=True, help="recording directory")
    parser.add_argument("--stride", type=int, default=1)
    args = parser.parse_args(argv)

    contract = load_contract()
    rest = contract.sonic_root / "gear_sonic/data/human/human_joints_info.npz"
    if not rest.is_file():
        raise SystemExit(f"missing rest skeleton: {rest}")
    fk = SmplFk(rest)

    clip = np.load(Path(args.clip).expanduser(), allow_pickle=True)
    poses = np.asarray(clip["poses"], dtype=np.float64)
    trans = np.asarray(clip["trans"], dtype=np.float64).reshape(-1, 3)
    fps = float(np.asarray(clip["mocap_frame_rate"]).reshape(-1)[0]) if "mocap_frame_rate" in clip else 30.0
    if poses.shape[0] != trans.shape[0]:
        raise SystemExit("poses and trans disagree on the frame count")

    recording = PicoRecording(source=f"synthetic_from:{Path(args.clip).name}")
    frames = range(0, poses.shape[0], max(1, args.stride))
    t0 = 1_700_000_000_000_000_000
    previous_quat = None
    for out_idx, i in enumerate(frames):
        joints_world = fk.joints(poses[i, 1:22], poses[i, 0]) + trans[i]
        root = joints_world[0]
        quat = Rotation.from_rotvec(poses[i, 0]).as_quat()[[3, 0, 1, 2]]
        quat = quat / max(float(np.linalg.norm(quat)), 1e-12)
        if previous_quat is not None and float(np.dot(previous_quat, quat)) < 0.0:
            quat = -quat
        previous_quat = quat
        rot = Rotation.from_quat(quat[[1, 2, 3, 0]])
        local = rot.inv().apply(joints_world[:24] - root)
        recording.frames.append(
            HumanSmplFrame(
                seq=out_idx,
                timestamp_ns=t0 + int(round(out_idx / fps * 1e9)),
                smpl_joints=local,
                smpl_pose=poses[i, 1:22].copy(),
                root_translation=root.copy(),
                root_quat_wxyz=quat.copy(),
                body_quat_w=quat.copy(),
                receive_timestamp_ns=t0 + int(round(out_idx / fps * 1e9)),
            ).validate()
        )

    recording.started_ns = recording.frames[0].receive_timestamp_ns
    recording.finished_ns = recording.frames[-1].receive_timestamp_ns
    recording.seq_stats = {
        "received": len(recording.frames),
        "dropped": 0,
        "duplicates": 0,
        "reordered": 0,
    }
    out = Path(args.out).expanduser()
    directory = recording.save(out, source=recording.source)
    stats = recording.stats()
    (directory / "source.json").write_text(
        json.dumps(
            {
                "created": datetime.now().isoformat(timespec="seconds"),
                "clip": str(Path(args.clip).expanduser()),
                "fps": fps,
                "frames": len(recording.frames),
                "note": "synthetic PICO recording derived from a SMPL-X clip (M5 chain test)",
            },
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    print(f"wrote {directory}: {len(recording)} frames @ {fps:g} fps")
    print(f"stats: hz={stats['source_hz_mean']:.2f} finite={stats['all_finite']}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
