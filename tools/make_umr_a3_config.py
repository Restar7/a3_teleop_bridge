#!/usr/bin/env python3
"""Generate ``UMR/robot_configs/humanoid_retarget_agibot_a3.json`` (plan §13-16).

Everything in the generated config is derived from the live A3 assets:

  * MJCF / URDF            -> joint limits        (generated/a3_joint_limits.yaml)
  * MJCF forward kinematics-> T-pose              (generated/a3_tpose.json)
  * A3 contract            -> policy joint set    (generated/a3_contract.json)

The MJCF path is written with ``${SONIC_A3_ROOT}`` so the UMR config never
hard-codes a home directory (plan section 14; the loader expands env vars).

Locked joints: the passive foot hinges and the parallel-mechanism motor joints
are pinned to ``[0, 0]`` so the retargeter cannot turn them into free variables.
The A3 ankle/waist pitch+roll coordinates themselves remain fully optimisable --
they are what actually drives the foot and torso surfaces.

Usage:
    python tools/make_umr_a3_config.py [--umr-root PATH] [--check]
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import yaml

BRIDGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(BRIDGE_ROOT / "src"))

from a3_teleop_bridge.contract import load_contract  # noqa: E402

TPOSE_JSON = BRIDGE_ROOT / "generated" / "a3_tpose.json"
LIMITS_YAML = BRIDGE_ROOT / "generated" / "a3_joint_limits.yaml"

# Hinge joints that must never be optimised by the retargeter.
LOCKED_JOINTS = (
    # two-stage passive foot
    "left_foot_forefoot_joint",
    "left_foot_toe_joint",
    "right_foot_forefoot_joint",
    "right_foot_toe_joint",
    # parallel ankle mechanism motors
    "left_ankle_motor_up_joint",
    "left_ankle_motor_down_joint",
    "right_ankle_motor_up_joint",
    "right_ankle_motor_down_joint",
    # parallel waist mechanism motors
    "left_waist_motor_joint",
    "right_waist_motor_joint",
    # head (kept neutral; stripped from the A3 policy reference anyway)
    "head_yaw_joint",
    "head_pitch_joint",
)


def find_umr_root(explicit: str | None) -> Path:
    candidates = []
    if explicit:
        candidates.append(Path(explicit))
    candidates.append(Path.home() / "a3_teleop_ws" / "UMR")
    candidates.append(BRIDGE_ROOT.parent / "UMR")
    for cand in candidates:
        if (cand / "robot_configs").is_dir():
            return cand.expanduser().resolve()
    raise SystemExit("Could not locate the UMR checkout; pass --umr-root")


def build_config(contract, tpose_doc: dict, limits_doc: dict) -> dict:
    mjcf_rel = tpose_doc["mjcf"]
    sonic_root = contract.sonic_root
    try:
        mjcf_rel_from_root = str(Path(mjcf_rel).relative_to(sonic_root))
    except ValueError:
        raise SystemExit(f"MJCF {mjcf_rel} is not inside SONIC_A3_ROOT {sonic_root}")

    tpose = {k: float(v) for k, v in tpose_doc["tpose_qpos"].items()}

    joint_limits: dict[str, list] = {}
    for name in contract.policy_joint_names:
        entry = limits_doc["joints"][name]
        joint_limits[name] = [
            round(float(entry["position_lower"]), 9),
            round(float(entry["position_upper"]), 9),
        ]
    for name in LOCKED_JOINTS:
        joint_limits[name] = [0.0, 0.0]

    return {
        "robot": {
            "name": "agibot_a3",
            "xml": "${SONIC_A3_ROOT}/" + mjcf_rel_from_root,
            "point_cloud_center": "body:waist_yaw_Link",
            "sample_pose": "tpose",
            "tpose_qpos": dict(sorted(tpose.items())),
            "joint_limits": dict(sorted(joint_limits.items())),
        },
        "_bridge": {
            "generated_by": "a3_teleop_bridge/tools/make_umr_a3_config.py",
            "sources": {
                "mjcf": mjcf_rel,
                "urdf": str(contract.urdf_path),
                "tpose": "generated/a3_tpose.json",
                "joint_limits": "generated/a3_joint_limits.yaml",
                "contract": "generated/a3_contract.json",
            },
            "notes": [
                "robot.xml uses ${SONIC_A3_ROOT}; the UMR config loader expands ~ and ${VAR}.",
                "tpose_qpos comes from a bounded FK fit of the MJCF, not from hand entry.",
                "passive foot + parallel-mechanism motor joints are locked to [0, 0].",
                "head joints stay neutral in UMR qpos but are excluded from the A3 policy view.",
            ],
            "locked_joints": list(LOCKED_JOINTS),
            "policy_joint_count": contract.n_policy_joints,
        },
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--umr-root", default=None)
    parser.add_argument("--check", action="store_true", help="fail if the file would change")
    args = parser.parse_args(argv)

    contract = load_contract()
    tpose_doc = json.loads(TPOSE_JSON.read_text(encoding="utf-8"))
    limits_doc = yaml.safe_load(LIMITS_YAML.read_text(encoding="utf-8"))
    if not tpose_doc.get("acceptable"):
        raise SystemExit("generated/a3_tpose.json is not acceptable; run tools/build_a3_tpose.py")

    cfg = build_config(contract, tpose_doc, limits_doc)
    umr_root = find_umr_root(args.umr_root)
    out_path = umr_root / "robot_configs" / "humanoid_retarget_agibot_a3.json"
    text = json.dumps(cfg, indent=2) + "\n"

    if args.check:
        if not out_path.is_file() or out_path.read_text(encoding="utf-8") != text:
            print(f"[check] {out_path} is out of date")
            return 1
        print(f"[check] {out_path} up to date")
        return 0

    out_path.write_text(text, encoding="utf-8")
    print(f"wrote {out_path}")
    print(f"  xml            : {cfg['robot']['xml']}")
    print(f"  policy joints  : {contract.n_policy_joints}")
    print(f"  locked joints  : {len(LOCKED_JOINTS)}")
    print(f"  tpose entries  : {len(cfg['robot']['tpose_qpos'])}")
    for name, value in cfg["robot"]["tpose_qpos"].items():
        print(f"    {name:32s} {value:+.6f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
