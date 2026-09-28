#!/usr/bin/env python3
"""Turn a `retarget_live --dump-frames` JSONL into a source-vs-reference verdict.

The live chain has four layers and a leg that "does not move" can die at any of
them:

    headset XR joints -> sender conversion -> UMR source -> reference -> policy

`probe_pico_body.py` covers the first two.  This covers the middle: for every
solved frame the session records the source-side leg angles and the reference
joint values it produced, so comparing their excursions says whether the
operator's motion reached the reference -- without guessing.

Usage:
    python tools/report_live_dump.py PATH.jsonl
    # or the newest dump under the workspace logs:
    python tools/report_live_dump.py
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

BRIDGE_ROOT = Path(__file__).resolve().parents[1]
WS = BRIDGE_ROOT.parent

#: source angle (degrees) -> the reference joint (radians) it drives.
#: ``exact`` marks the pairs whose mapping really is 1:1, so a ratio far from 1
#: is a finding.  A thigh tilt also contains the pelvis pitch, so the hip pair is
#: informational only -- comparing it numerically would be meaningless.
PAIRS = [
    ("L_knee_interior_deg", "ref_left_knee_joint", True),
    ("R_knee_interior_deg", "ref_right_knee_joint", True),
    # thigh tilt is deliberately NOT compared: it is measured on the source
    # skeleton, whose frame in the online session is not the one a "tilt from
    # vertical" needs, and reading it the wrong way round produced a warning on a
    # perfectly standing clip. The reference's own hip pitch is used instead.
]


def newest_dump() -> Path | None:
    candidates = sorted((WS / "logs").glob("**/live_frames.jsonl"), key=lambda p: p.stat().st_mtime)
    return candidates[-1] if candidates else None


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("path", nargs="?", default=None)
    parser.add_argument("--min-move-deg", type=float, default=5.0,
                        help="source motion below this counts as 'the operator did not move it'")
    args = parser.parse_args(argv)

    path = Path(args.path).expanduser() if args.path else newest_dump()
    if path is None or not path.is_file():
        print("no frame dump found -- run the live chain with --dump-frames PATH")
        return 2

    rows = []
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            rows.append(json.loads(line))
        except json.JSONDecodeError:
            continue
    if len(rows) < 5:
        print(f"{path}: only {len(rows)} frames -- too few to judge")
        return 2

    def span(key):
        vals = [r[key] for r in rows if key in r and r[key] == r[key]]
        return (max(vals) - min(vals)) if len(vals) > 1 else 0.0

    print(f"dump : {path}  ({len(rows)} frames)")
    print(f"cost : mean {sum(r.get('cost', 0) for r in rows) / len(rows):.6f}")
    print(f"root z: [{min(r.get('root_z', 0) for r in rows):.4f}, "
          f"{max(r.get('root_z', 0) for r in rows):.4f}]")
    print()
    print(f"{'source signal':24s} {'source':>11s} {'reference':>12s} {'ratio':>7s}  note")
    verdict = []
    for src, ref, exact in PAIRS:
        s_span_deg = span(src)
        r_span = span(ref)
        # degrees -> radians: comparing raw numbers here is how a working pipeline
        # gets reported as "0.02x, the reference barely moves"
        s_span = float(np.radians(s_span_deg))
        ratio = (r_span / s_span) if s_span > 1e-9 else float("nan")
        print(f"{src:24s} {s_span_deg:10.2f}d {r_span:11.4f}r {ratio:7.2f}  "
              f"{'1:1 expected' if exact else 'informational (tilt includes pelvis)'}")
        verdict.append((src, ref, s_span_deg, r_span, ratio, exact))

    # ---- does the reference stay inside what the policy can actually hold? ----
    # A reference can be a perfect copy of the operator and still put the robot on
    # the floor: A3-fast cannot balance on one leg, so a high leg lift (the
    # runbook forbids "large single-leg standing") ends in a fall with root_err
    # growing past a metre while every ratio above reads 1.00.
    # Use the *reference's own* hip pitch: it is in the robot's frame, so unlike any
    # angle measured on the source skeleton it cannot be read the wrong way round.
    swings = []
    for side in ("left", "right"):
        vals = [r.get(f"ref_{side}_hip_pitch_joint") for r in rows
                if r.get(f"ref_{side}_hip_pitch_joint") is not None]
        if vals:
            swings.append((side, max(abs(v) for v in vals)))
    print()
    print("reference leg swing (the policy's envelope, not the pipeline's):")
    for side, peak in swings:
        flag = "  <-- swung far past a balanced stance" if peak > 0.9 else ""
        print(f"  {side} hip pitch: peak |q| {peak:5.2f} rad ({peak * 57.2958:5.1f} deg){flag}")
    if any(peak > 0.9 for _side, peak in swings):
        print("  WARNING: the reference swings a hip past ~50 deg. A3-fast cannot balance")
        print("           through that (the runbook forbids large single-leg standing), so")
        print("           expect a fall even though every ratio below reads 1.00.")

    print()
    moved = [v for v in verdict if v[2] >= args.min_move_deg]
    if not moved:
        print("VERDICT: SOURCE_STATIC -- the operator's leg motion never reached the "
              "retarget. The headset (or the sender conversion) is not delivering "
              "legs; run tools/probe_pico_body.py --expect legs to tell which.")
        return 4
    exact_moved = [v for v in moved if v[5]]
    dead = [v for v in exact_moved if not (v[3] > 0.02)]
    if dead:
        print("VERDICT: REFERENCE_STATIC -- the source moved but the reference did not "
              "(" + ", ".join(v[0] for v in dead) + "). This is a retarget bug.")
        return 5
    weak = [v for v in exact_moved if v[4] < 0.6]
    if weak:
        print("VERDICT: REFERENCE_WEAK -- the reference follows the source but with much "
              "less amplitude (" + ", ".join(f"{v[0]} {v[4]:.2f}x" for v in weak) + ").")
        return 6
    print("VERDICT: OK -- the source's leg motion reaches the reference at comparable "
          "amplitude (" + ", ".join(f"{v[0]} {v[4]:.2f}x" for v in exact_moved) + "). "
          "Anything left is the headset input or the policy's tracking, not the reference.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
