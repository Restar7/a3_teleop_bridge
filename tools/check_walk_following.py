#!/usr/bin/env python3
"""Measure how far the A3 policy actually travels against a walking reference.

Answers "the operator walks but the robot will not move" with numbers instead of
opinion, by running the *official* walk references through sim2sim and comparing
the robot's horizontal travel with the reference's.

Why the robot under-travels is not a pipeline bug.  The A3-fast observation is
built from exactly two terms (``sim2sim_a3_mujoco.ENCODER_TERMS``):

    ENCODER_FRAME_DIM = NUM_POLICY_DOFS * 2 + 6      # 29*2 joints + 6D orientation
    ENCODER_INPUT_DIM = NUM_FUTURE_FRAMES * 64       # 10 x 64 = 640

i.e. future joint positions/velocities plus the *orientation* difference between
the robot's anchor and the reference anchor.  There is **no position term**: the
reference's horizontal position never enters the observation, and
``anchor_pos_error_m`` is only computed for the metrics report.  So the robot has
no signal that it is falling behind and no means to correct it -- it translates
only as a side effect of stepping, which saturates near 0.02 m/s.

Consequences this tool demonstrates:

    reference                        ref travel   robot travel   ratio
    001_walk_front_slow                  1.085 m        0.546 m    50%
    ...same reference, root x2           2.170 m        0.452 m    21%
    009_walk_left_fast                   1.630 m        0.143 m     9%

Scaling the reference up makes it *worse*, so "command it further" is not a fix.
Every run stays fall=False: the robot balances fine, it just does not locomote.

Usage:
    python tools/check_walk_following.py
    python tools/check_walk_following.py --clips 001_walk_front_slow 050_march_place_slow
"""

from __future__ import annotations

import argparse
import csv
import json
import subprocess
import sys
import tempfile
from pathlib import Path

BRIDGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(BRIDGE_ROOT / "src"))

from a3_teleop_bridge import paths  # noqa: E402
from a3_teleop_bridge.contract import load_contract  # noqa: E402

DEFAULT_CLIPS = ("001_walk_front_slow", "009_walk_left_fast", "050_march_place_slow")
#: Below this fraction of the reference's travel the robot is not really walking.
FOLLOWING_FLOOR = 0.80


def travel(series, start: int = 0, end: int = -1) -> float:
    first = series[start]
    last = series[end]
    return float(((last[0] - first[0]) ** 2 + (last[1] - first[1]) ** 2) ** 0.5)


def scale_root(csv_path: Path, factor: float, out_path: Path) -> None:
    """Multiply the reference's horizontal travel, leaving everything else alone."""
    rows = list(csv.DictReader(csv_path.open()))
    x0, y0 = float(rows[0]["root_translateX"]), float(rows[0]["root_translateY"])
    for row in rows:
        row["root_translateX"] = str(x0 + factor * (float(row["root_translateX"]) - x0))
        row["root_translateY"] = str(y0 + factor * (float(row["root_translateY"]) - y0))
    with out_path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def run_clip(clip_csv: Path, out_dir: Path, sim_python: Path, steps: int) -> dict:
    contract = load_contract()
    out_dir.mkdir(parents=True, exist_ok=True)
    cmd = [
        str(sim_python),
        "gear_sonic/scripts/sim2sim_a3_mujoco.py",
        "--checkpoint",
        str(contract.sonic_root / "checkpoints" / "035_step200000" / "model_step_200000.pt"),
        "--motion",
        str(clip_csv),
        "--encoder-mode",
        "a3_fast",
        "--mjcf",
        str(contract.mjcf_path),
        "--csv-source-fps",
        "30",
        "--csv-frame-stride",
        "1",
        "--batch-once",
        "--max-policy-steps",
        str(steps),
        "--metrics-out",
        str(out_dir / "metrics.json"),
        "--timeseries-out",
        str(out_dir / "timeseries.json"),
    ]
    proc = subprocess.run(
        cmd, cwd=str(contract.sonic_root), capture_output=True, text=True, timeout=1800
    )
    if proc.returncode != 0:
        return {"error": (proc.stderr or proc.stdout or "")[-400:]}
    motion = json.loads((out_dir / "timeseries.json").read_text())["motions"][0]
    metrics = json.loads((out_dir / "metrics.json").read_text())
    return {
        "reference_travel_m": travel(motion["ref_root_pos_w"]),
        "robot_travel_m": travel(motion["sim_root_pos_w"]),
        "fall": bool(metrics.get("fall")),
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--clips", nargs="*", default=list(DEFAULT_CLIPS))
    parser.add_argument("--steps", type=int, default=1200, help="policy steps (50 Hz)")
    parser.add_argument("--scale", type=float, default=None,
                        help="also try the reference with its root travel multiplied by this")
    parser.add_argument("--sim-python", default=None)
    parser.add_argument("--out-dir", default=None)
    args = parser.parse_args(argv)

    contract = load_contract()
    clips_dir = contract.sonic_root / "a3_data" / "agibot_a3"
    if not clips_dir.is_dir():
        print(f"no official motion set at {clips_dir}")
        return 2
    sim_python = Path(args.sim_python) if args.sim_python else Path(sys.executable)
    out_root = Path(args.out_dir) if args.out_dir else Path(tempfile.mkdtemp(prefix="walk_follow_"))

    print(f"sim interpreter: {sim_python}")
    print(f"{args.steps} policy steps = {args.steps / 50.0:.1f}s of motion\n")
    print(f"{'clip':30s} {'ref travel':>11s} {'robot travel':>13s} {'ratio':>7s} {'fall':>6s}")

    weak = []
    for name in args.clips:
        csv_path = clips_dir / f"{name}.csv"
        if not csv_path.is_file():
            print(f"{name:30s}   (missing)")
            continue
        result = run_clip(csv_path, out_root / name, sim_python, args.steps)
        if "error" in result:
            print(f"{name:30s}   FAILED: {result['error']}")
            continue
        ratio = result["robot_travel_m"] / max(result["reference_travel_m"], 1e-9)
        print(
            f"{name:30s} {result['reference_travel_m']:10.3f}m "
            f"{result['robot_travel_m']:12.3f}m {ratio * 100:6.0f}% {str(result['fall']):>6s}"
        )
        if ratio < FOLLOWING_FLOOR:
            weak.append((name, ratio))

        if args.scale and args.scale > 0:
            scaled = out_root / f"{name}_x{args.scale:g}.csv"
            scale_root(csv_path, args.scale, scaled)
            scaled_result = run_clip(scaled, out_root / f"{name}_x{args.scale:g}", sim_python, args.steps)
            if "error" not in scaled_result:
                sratio = scaled_result["robot_travel_m"] / max(scaled_result["reference_travel_m"], 1e-9)
                print(
                    f"{'  same, root x' + f'{args.scale:g}':30s} "
                    f"{scaled_result['reference_travel_m']:10.3f}m "
                    f"{scaled_result['robot_travel_m']:12.3f}m {sratio * 100:6.0f}% "
                    f"{str(scaled_result['fall']):>6s}"
                )

    print()
    if weak:
        print(
            f"VERDICT: the robot follows only {min(r for _, r in weak) * 100:.0f}-"
            f"{max(r for _, r in weak) * 100:.0f}% of the reference's travel, so it is left behind\n"
            "         no matter how healthy the input is.  The A3-fast observation has no\n"
            "         position term (ENCODER_TERMS), so the robot cannot see or correct the\n"
            "         gap; forward travel is a side effect of stepping and saturates near\n"
            "         0.02 m/s.  This is a policy property, not a bridge or retarget defect."
        )
        return 1
    print("VERDICT: the robot tracks the reference's travel; the input is not the limit.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
