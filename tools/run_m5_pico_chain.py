#!/usr/bin/env python3
"""M5: recorded PICO -> adapter -> UMR -> A3 -> CSV -> A3-fast -> MuJoCo.

Chains the four existing tools so the whole offline PICO path is one command:

    recordings/<session>            (PICO packed stream, recorded)
      -> umr/source_adapter         (SMPL-X sequence; neutral betas)
      -> run_umr_a3_batch           (UMR surface retargeting to A3)
      -> run_a3_validation_suite    (numeric checks + CSV + MuJoCo)

Usage:
    python tools/run_m5_pico_chain.py --clips stand,lift_left_foot --out ~/a3_teleop_ws/logs/m5
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path

BRIDGE_ROOT = Path(__file__).resolve().parents[1]
WS = BRIDGE_ROOT.parent
PY = sys.executable


def run(cmd: list[str], timeout: int = 3600) -> tuple[int, str]:
    proc = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
    return proc.returncode, proc.stdout + proc.stderr


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--clips", required=True, help="comma separated validation clip names")
    parser.add_argument("--smplx-dir", default=str(WS / "data" / "smplx_validation"))
    parser.add_argument("--recordings-dir", default=str(WS / "recordings"))
    parser.add_argument("--smplx-out", default=str(WS / "data" / "pico_smplx"))
    parser.add_argument("--umr-out", default=str(WS / "UMR" / "output" / "a3_pico"))
    parser.add_argument("--out", default=str(WS / "logs" / "m5"))
    args = parser.parse_args(argv)

    out_dir = Path(args.out).expanduser()
    out_dir.mkdir(parents=True, exist_ok=True)
    clips = [c.strip() for c in args.clips.split(",") if c.strip()]
    results = {}

    for clip in clips:
        entry: dict = {"clip": clip}
        recording = Path(args.recordings_dir) / f"m5_{clip}"
        smplx_clip = Path(args.smplx_dir) / f"{clip}.npz"

        code, text = run(
            [
                PY,
                str(BRIDGE_ROOT / "tools" / "make_synthetic_pico_recording.py"),
                "--clip",
                str(smplx_clip),
                "--out",
                str(recording),
            ]
        )
        entry["recording"] = {"ok": code == 0, "path": str(recording), "log": text[-400:]}
        if code != 0:
            results[clip] = entry
            continue

        code, text = run(
            [
                PY,
                str(BRIDGE_ROOT / "tools" / "convert_pico_recording.py"),
                "--recording",
                str(recording),
                "--out",
                str(Path(args.smplx_out) / clip),
            ]
        )
        entry["adapter"] = {"ok": code == 0, "log": text[-400:]}
        if code != 0:
            results[clip] = entry
            continue

        code, text = run(
            [
                PY,
                str(BRIDGE_ROOT / "tools" / "run_umr_a3_batch.py"),
                "--data-dir",
                str(Path(args.smplx_out) / clip),
                "--out-dir",
                str(args.umr_out),
                "--force",
            ]
        )
        entry["umr"] = {"ok": code == 0, "log": text[-400:]}
        if code != 0:
            results[clip] = entry
            continue

        code, text = run(
            [
                PY,
                str(BRIDGE_ROOT / "tools" / "run_a3_validation_suite.py"),
                "--data-dir",
                str(args.umr_out),
                "--out-dir",
                str(out_dir / clip),
                "--only",
                f"m5_{clip}",
            ]
        )
        entry["validation"] = {"ok": code == 0, "log": text[-600:]}
        entry["pass"] = code == 0
        results[clip] = entry
        print(f"{clip:22s} {'PASS' if entry.get('pass') else 'FAIL'}")

    summary = {
        "clips": results,
        "passed": sum(1 for e in results.values() if e.get("pass")),
        "total": len(results),
    }
    summary["all_pass"] = summary["passed"] == summary["total"]
    (out_dir / "m5_report.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    print(f"\n{summary['passed']}/{summary['total']} clips PASS")
    print(f"report: {out_dir / 'm5_report.json'}")
    return 0 if summary["all_pass"] else 1


if __name__ == "__main__":
    sys.exit(main())
