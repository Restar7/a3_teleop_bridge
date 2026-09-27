#!/usr/bin/env python3
"""Inspect a UMR retarget result (plan section 12).

Reports qpos shape, joint names, fps, root representation, NaN count and joint
ranges -- and, when the result targets the A3, cross-checks it against the A3
contract (policy joints present, head/passive joints excluded, limits respected).

Usage:
    python tools/inspect_umr_result.py PATH.npz [--json OUT] [--no-images]
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

BRIDGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(BRIDGE_ROOT / "src"))

from a3_teleop_bridge.umr.offline import load_umr_result  # noqa: E402


def describe(result, expect_a3: bool) -> dict:
    qpos = result.qpos
    report: dict = {
        "path": str(result.path),
        "qpos_shape": list(qpos.shape),
        "n_frames": result.n_frames,
        "nq": result.nq,
        "fps": result.fps,
        "dt": result.dt,
        "duration_s": result.n_frames * result.dt if result.dt == result.dt else None,
        "robot_name": result.robot_name,
        "robot_xml": str(result.robot_xml),
        "source_sequence_key": result.source_sequence_key,
        "source_format": result.source_format,
        "smpl_scale": result.smpl_scale,
        "ground_z": result.ground_z,
        "nan_count": int(np.size(qpos) - np.count_nonzero(np.isfinite(qpos))),
        "frame_ids": {
            "first": int(result.frame_ids[0]) if result.frame_ids.size else None,
            "last": int(result.frame_ids[-1]) if result.frame_ids.size else None,
            "monotonic": bool(np.all(np.diff(result.frame_ids) > 0)) if result.frame_ids.size > 1 else True,
        },
        "root": {
            "representation": "free joint: qpos[0:3] = xyz (m), qpos[3:7] = quaternion (w, x, y, z)",
            "position_min": qpos[:, :3].min(axis=0).tolist(),
            "position_max": qpos[:, :3].max(axis=0).tolist(),
            "height_mean_m": float(qpos[:, 2].mean()),
            "quat_norm_min": float(np.linalg.norm(qpos[:, 3:7], axis=1).min()),
            "quat_norm_max": float(np.linalg.norm(qpos[:, 3:7], axis=1).max()),
        },
        "joint_count": len(result.joint_names),
        "joint_names": list(result.joint_names),
        "joint_ranges": {},
    }

    for name in result.joint_names:
        series = result.joint_series(name)
        report["joint_ranges"][name] = {
            "min": float(series.min()),
            "max": float(series.max()),
            "mean": float(series.mean()),
            "qpos_addr": result.joint_qpos_addr[name],
            "nonfinite": int(np.count_nonzero(~np.isfinite(series))),
        }

    if expect_a3:
        from a3_teleop_bridge.a3.limits import load_limits
        from a3_teleop_bridge.a3.joint_map import load_joint_map
        from a3_teleop_bridge.contract import load_contract

        contract = load_contract()
        limits = load_limits()
        jmap = load_joint_map()
        try:
            index, resolved, _ = jmap.build_index_map(result.joint_names)
            report["a3_mapping"] = {
                "ok": True,
                "policy_joints": contract.n_policy_joints,
                "resolved_source_names": resolved,
                "index": [int(i) for i in index],
            }
        except Exception as exc:
            report["a3_mapping"] = {"ok": False, "error": str(exc)}

        violations = []
        for i, name in enumerate(contract.policy_joint_names):
            series = result.joint_series(name)
            lo, hi = limits.lower[i], limits.upper[i]
            worst = max(float(lo - series.min()), float(series.max() - hi))
            if worst > 1e-6:
                violations.append({"joint": name, "excess_rad": worst})
        report["a3_limit_violations"] = violations

        excluded_present = [
            n
            for n in list(contract.head_joint_names) + list(contract.passive_foot_joint_names)
            if n in result.joint_qpos_addr
        ]
        report["excluded_joints_in_model"] = excluded_present
    return report


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("result", help="UMR result .npz")
    parser.add_argument("--robot-xml", default=None)
    parser.add_argument("--json", dest="json_out", default=None)
    parser.add_argument("--expect-a3", action="store_true", default=None)
    parser.add_argument("--no-a3", dest="expect_a3", action="store_false")
    args = parser.parse_args(argv)

    result = load_umr_result(args.result, robot_xml=args.robot_xml)
    expect_a3 = args.expect_a3
    if expect_a3 is None:
        expect_a3 = "a3" in Path(str(result.robot_xml)).name.lower() or "a3" in result.robot_name.lower()

    report = describe(result, expect_a3)

    print(f"file        : {report['path']}")
    print(f"qpos        : {report['qpos_shape']}  ({report['n_frames']} frames)")
    print(f"fps / dt    : {report['fps']:.4f} / {report['dt']:.6f} s")
    print(f"robot       : {report['robot_name']}  ({report['robot_xml']})")
    print(f"sequence    : {report['source_sequence_key']} [{report['source_format']}]")
    print(f"NaN count   : {report['nan_count']}")
    print(f"root        : {report['root']['representation']}")
    print(f"  height    : mean {report['root']['height_mean_m']:.4f} m")
    print(f"  pos range : min {np.round(report['root']['position_min'], 4)} max {np.round(report['root']['position_max'], 4)}")
    print(f"  |quat|    : [{report['root']['quat_norm_min']:.6f}, {report['root']['quat_norm_max']:.6f}]")
    print(f"joints      : {report['joint_count']}")
    head = list(report["joint_ranges"].items())[:6]
    for name, stats in head:
        print(f"  {name:34s} qpos[{stats['qpos_addr']:2d}] range [{stats['min']:+.4f}, {stats['max']:+.4f}]")
    if report["joint_count"] > 6:
        print(f"  ... {report['joint_count'] - 6} more")

    if expect_a3:
        mapping = report.get("a3_mapping", {})
        print(f"A3 mapping  : ok={mapping.get('ok')}")
        if not mapping.get("ok"):
            print(f"  error     : {mapping.get('error')}")
        violations = report.get("a3_limit_violations", [])
        print(f"A3 limits   : {len(violations)} violations")
        for entry in violations[:10]:
            print(f"  ! {entry['joint']:32s} exceeds by {entry['excess_rad']:.4f} rad")
        print(f"excluded    : {report['excluded_joints_in_model']}")

    if args.json_out:
        out = Path(args.json_out).expanduser()
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        print(f"\nwrote {out}")

    ok = report["nan_count"] == 0
    if expect_a3:
        ok = ok and report.get("a3_mapping", {}).get("ok", False)
    print("\nRESULT:", "OK" if ok else "PROBLEMS FOUND")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
