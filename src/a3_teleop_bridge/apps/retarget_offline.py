"""Offline UMR result -> A3 canonical -> flat CSV (plan sections 20, 21).

Usage::

    python -m a3_teleop_bridge.apps.retarget_offline \
        --umr-result ~/a3_teleop_ws/UMR/output/agibot_a3_retarget/<seq>.npz \
        --out-csv ~/a3_teleop_ws/logs/a3_csv/<seq>.csv \
        --report ~/a3_teleop_ws/logs/a3_csv/<seq>.report.json

The exported CSV is a **native source-fps** flat CSV, so sim2sim must be run with
``--csv-source-fps <fps> --csv-frame-stride 1`` (the repository default of
30 fps / stride 4 is meant for the legacy 120 fps sample files).  The report
records the fps so the MuJoCo run cannot silently use the wrong stride.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

from ..a3.csv_export import quat_angle_deg, read_a3_flat_csv, write_a3_flat_csv
from ..a3.limits import load_limits
from ..contract import load_contract
from ..umr.offline import load_umr_result
from ..umr.state_converter import UmrStateConverter


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--umr-result", required=True, help="UMR retarget .npz")
    parser.add_argument("--out-csv", required=True, help="destination flat CSV")
    parser.add_argument("--report", default=None, help="optional JSON report path")
    parser.add_argument("--robot-xml", default=None, help="override the robot XML")
    parser.add_argument("--max-frames", type=int, default=0, help="0 = all frames")
    parser.add_argument("--start", type=int, default=0)
    parser.add_argument("--verbose", action="store_true")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)

    contract = load_contract()
    limits = load_limits()
    converter = UmrStateConverter()

    result = load_umr_result(args.umr_result, robot_xml=args.robot_xml)
    stop = result.n_frames if args.max_frames <= 0 else min(result.n_frames, args.start + args.max_frames)
    frames = range(args.start, stop)

    joint_pos = np.zeros((len(frames), contract.n_policy_joints))
    joint_vel = np.zeros_like(joint_pos)
    root_pos = np.zeros((len(frames), 3))
    root_quat = np.zeros((len(frames), 4))
    timestamps = np.zeros(len(frames), dtype=np.int64)
    invalid = 0
    clamped_frames = 0
    reasons: dict[str, int] = {}

    dt = result.dt
    previous = None
    for out_idx, frame_idx in enumerate(frames):
        state = converter.convert_frame(
            result.frame_joint_dict(frame_idx),
            seq=out_idx,
            timestamp_ns=int(round(frame_idx * dt * 1e9)),
            root_pos_m=result.root_pos[frame_idx],
            root_quat_wxyz=result.root_quat_wxyz[frame_idx],
            previous=previous,
            dt=dt,
        )
        if not state.valid:
            invalid += 1
            reasons[state.reason] = reasons.get(state.reason, 0) + 1
        if state.reason.startswith("clamped") or "clamped" in state.reason:
            clamped_frames += 1
        if state.valid:
            previous = state
        joint_pos[out_idx] = state.joint_pos_rad
        joint_vel[out_idx] = state.joint_vel_rad_s
        root_pos[out_idx] = state.root_pos_m
        root_quat[out_idx] = state.root_quat_wxyz
        timestamps[out_idx] = state.timestamp_ns

    out_csv = Path(args.out_csv).expanduser()
    write_a3_flat_csv(out_csv, root_pos, root_quat, joint_pos, frame_ids=np.arange(len(frames)))

    # ---- verify the CSV we just wrote ------------------------------------
    back = read_a3_flat_csv(out_csv, source_fps=result.fps, frame_stride=1)
    pos_err = float(np.max(np.abs(back.root_pos_m - root_pos)))
    joint_err = float(np.max(np.abs(back.dof29_rad - joint_pos)))
    quat_err = float(np.max(quat_angle_deg(back.root_quat_wxyz, root_quat)))

    velocity_norm = np.linalg.norm(joint_vel, axis=1)
    report = {
        "source_npz": str(result.path),
        "robot_xml": str(result.robot_xml),
        "sequence": result.source_sequence_key,
        "out_csv": str(out_csv),
        "frames": len(frames),
        "fps": result.fps,
        "frame_stride_for_sim2sim": 1,
        "sim2sim_hint": f"--csv-source-fps {result.fps:g} --csv-frame-stride 1",
        "invalid_frames": invalid,
        "invalid_reasons": reasons,
        "clamped_frames": clamped_frames,
        "joint_pos_min": float(joint_pos.min()),
        "joint_pos_max": float(joint_pos.max()),
        "joint_vel_max_norm": float(velocity_norm.max()) if velocity_norm.size else 0.0,
        "root_height_mean": float(root_pos[:, 2].mean()),
        "root_height_min": float(root_pos[:, 2].min()),
        "root_height_max": float(root_pos[:, 2].max()),
        "nan_joint_pos": int(np.count_nonzero(~np.isfinite(joint_pos))),
        "nan_root": int(np.count_nonzero(~np.isfinite(root_pos))),
        "limit_violations": [],
    }
    violations = []
    for i, name in enumerate(contract.policy_joint_names):
        lo, hi = limits.lower[i], limits.upper[i]
        excess = max(float(lo - joint_pos[:, i].min()), float(joint_pos[:, i].max() - hi))
        if excess > 1e-6:
            violations.append({"joint": name, "excess_rad": excess})
    report["limit_violations"] = violations
    report["csv_roundtrip"] = {
        "root_pos_max_err_m": pos_err,
        "joint_pos_max_err_rad": joint_err,
        "root_quat_max_err_deg": quat_err,
    }

    acceptable = (
        report["nan_joint_pos"] == 0
        and report["nan_root"] == 0
        and invalid == 0
        and pos_err < 1e-9
        and joint_err < 1e-9
        and not violations
    )
    report["acceptable"] = acceptable

    if args.report:
        report_path = Path(args.report).expanduser()
        report_path.parent.mkdir(parents=True, exist_ok=True)
        report_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")

    print(f"UMR result : {result.path}  ({result.n_frames} frames @ {result.fps:g} fps)")
    print(f"robot      : {result.robot_name}  ({result.robot_xml})")
    print(f"CSV        : {out_csv}  ({len(frames)} rows)")
    print(f"sim2sim    : {report['sim2sim_hint']}")
    print(f"invalid    : {invalid} frames  (clamped in {clamped_frames})")
    print(f"joint range: [{joint_pos.min():+.4f}, {joint_pos.max():+.4f}] rad")
    print(f"root height: mean {report['root_height_mean']:.4f} m "
          f"[{report['root_height_min']:.4f}, {report['root_height_max']:.4f}]")
    print(f"roundtrip  : pos {pos_err:.2e} m, joints {joint_err:.2e} rad, quat {quat_err:.2e} deg")
    print(f"violations : {len(violations)}")
    for entry in violations[:10]:
        print(f"  ! {entry['joint']:32s} by {entry['excess_rad']:.4f} rad")
    if args.report:
        print(f"report     : {args.report}")
    print("RESULT:", "OK" if acceptable else "PROBLEMS FOUND")
    return 0 if acceptable else 1


if __name__ == "__main__":
    sys.exit(main())
