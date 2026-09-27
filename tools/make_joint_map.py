#!/usr/bin/env python3
"""Generate ``configs/a3_joint_map.yaml`` (plan section 18).

The policy side always comes from the A3 contract.  When a UMR result file is
available its ``robot_joint_names`` are read as the *source* side and the two
name sets are compared: names that match exactly need no alias, the rest must be
covered by an explicit alias entry.  Without a UMR result the tool still emits a
valid map (identity aliases plus the known naming variants), so the bridge can be
built and tested before UMR's A3 run exists.

Usage:
    python tools/make_joint_map.py                      # contract-only
    python tools/make_joint_map.py --umr-result PATH.npz  # verify against UMR
"""

from __future__ import annotations

import argparse
import json
import sys
from collections import OrderedDict
from pathlib import Path

import numpy as np
import yaml

BRIDGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(BRIDGE_ROOT / "src"))

from a3_teleop_bridge.contract import load_contract  # noqa: E402

DEFAULT_OUT = BRIDGE_ROOT / "configs" / "a3_joint_map.yaml"

# Naming variants accepted for the same A3 joint.  Values are policy names.
KNOWN_ALIASES = {
    # MJCF/URDF style without the trailing "_joint"
    "waist_yaw": "waist_yaw_joint",
    "waist_roll": "waist_roll_joint",
    "waist_pitch": "waist_pitch_joint",
    "left_shoulder_pitch": "left_shoulder_pitch_joint",
    "left_shoulder_roll": "left_shoulder_roll_joint",
    "left_shoulder_yaw": "left_shoulder_yaw_joint",
    "left_elbow": "left_elbow_joint",
    "left_wrist_roll": "left_wrist_roll_joint",
    "left_wrist_pitch": "left_wrist_pitch_joint",
    "left_wrist_yaw": "left_wrist_yaw_joint",
    "right_shoulder_pitch": "right_shoulder_pitch_joint",
    "right_shoulder_roll": "right_shoulder_roll_joint",
    "right_shoulder_yaw": "right_shoulder_yaw_joint",
    "right_elbow": "right_elbow_joint",
    "right_wrist_roll": "right_wrist_roll_joint",
    "right_wrist_pitch": "right_wrist_pitch_joint",
    "right_wrist_yaw": "right_wrist_yaw_joint",
    "left_hip_pitch": "left_hip_pitch_joint",
    "left_hip_roll": "left_hip_roll_joint",
    "left_hip_yaw": "left_hip_yaw_joint",
    "left_knee": "left_knee_joint",
    "left_ankle_pitch": "left_ankle_pitch_joint",
    "left_ankle_roll": "left_ankle_roll_joint",
    "right_hip_pitch": "right_hip_pitch_joint",
    "right_hip_roll": "right_hip_roll_joint",
    "right_hip_yaw": "right_hip_yaw_joint",
    "right_knee": "right_knee_joint",
    "right_ankle_pitch": "right_ankle_pitch_joint",
    "right_ankle_roll": "right_ankle_roll_joint",
    # link-style names used by the MJCF bodies
    "waist_yaw_Link": "waist_yaw_joint",
    "waist_roll_Link": "waist_roll_joint",
    "waist_pitch_Link": "waist_pitch_joint",
    "left_elbow_Link": "left_elbow_joint",
    "right_elbow_Link": "right_elbow_joint",
    "left_knee_Link": "left_knee_joint",
    "right_knee_Link": "right_knee_joint",
}


def read_umr_joint_names(path: Path) -> list[str] | None:
    data = np.load(path, allow_pickle=True)
    for key in ("robot_joint_names", "joint_names"):
        if key in data:
            names = data[key]
            return [str(n) for n in np.asarray(names).reshape(-1)]
    return None


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--umr-result", default=None, help="UMR retarget .npz to verify against")
    parser.add_argument("--out", default=str(DEFAULT_OUT))
    args = parser.parse_args(argv)

    contract = load_contract()
    policy = list(contract.policy_joint_names)

    source_names: list[str] | None = None
    if args.umr_result:
        source_names = read_umr_joint_names(Path(args.umr_result).expanduser())
        if source_names is None:
            print(f"[warn] {args.umr_result} has no robot_joint_names field")
    else:
        # look for a UMR A3 result produced earlier in the workspace
        for cand in sorted((BRIDGE_ROOT.parent / "UMR" / "output").glob("agibot_a3*/**/*.npz")):
            names = read_umr_joint_names(cand)
            if names:
                source_names = names
                print(f"[info] using UMR result {cand}")
                break

    aliases: dict[str, str] = {}
    unmatched: list[str] = []
    if source_names:
        for name in source_names:
            if name in policy:
                continue
            target = KNOWN_ALIASES.get(name)
            if target is not None and target in policy:
                aliases[name] = target
            else:
                unmatched.append(name)
        covered = {n for n in source_names if n in policy or n in aliases}
        missing = [n for n in policy if n not in covered]
        if missing:
            print(f"[warn] source does not cover policy joints: {missing}")
    else:
        print("[info] no UMR result available; emitting the contract-only map")

    doc = OrderedDict(
        [
            ("schema", "a3_joint_map/v1"),
            ("generated_by", "tools/make_joint_map.py"),
            ("policy_joint_names", policy),
            (
                "aliases",
                OrderedDict(sorted(aliases.items())) if aliases else OrderedDict(),
            ),
            (
                "forbidden_source_names",
                list(contract.head_joint_names) + list(contract.passive_foot_joint_names),
            ),
            (
                "known_alias_candidates",
                sorted(set(KNOWN_ALIASES) - set(aliases)),
            ),
            (
                "notes",
                [
                    "policy order is the 29-DoF MuJoCo policy view from generated/a3_contract.json",
                    "aliases map alternative source names onto policy joint names",
                    "forbidden_source_names must never appear in a policy vector",
                    "no code may slice qpos by index; always resolve names through this map",
                ],
            ),
        ]
    )
    if source_names:
        doc["umr_source_joint_names"] = source_names
        doc["umr_unmatched_joint_names"] = unmatched

    out_path = Path(args.out).expanduser()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with out_path.open("w", encoding="utf-8") as handle:
        yaml.safe_dump(json.loads(json.dumps(doc)), handle, sort_keys=False)

    print(f"wrote {out_path}")
    print(f"  policy joints : {len(policy)}")
    print(f"  aliases       : {len(aliases)}")
    if source_names:
        print(f"  source joints : {len(source_names)}")
        print(f"  unmatched     : {len(unmatched)}")
        for name in unmatched[:20]:
            print(f"    ! {name}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
