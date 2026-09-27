#!/usr/bin/env python3
"""End-to-end A3 validation suite (M3b/M4 acceptance, plan sections 17 and 21).

For every retargeted clip in ``--data-dir`` this runs the whole chain and records
the result:

    UMR A3 npz  -> numeric validation (tools/validate_a3_motion.py)
                -> A3 canonical -> flat CSV (apps/retarget_offline)
                -> A3-fast + MuJoCo (tools/run_a3_baseline.py)

Output: ``--out-dir/report.json`` + one subdirectory per clip with the CSV,
the sim2sim metrics and both logs.  A clip only counts as PASS when the numeric
validation, the CSV round-trip and the MuJoCo rollout all pass.

Usage:
    python tools/run_a3_validation_suite.py \
        --data-dir ~/a3_teleop_ws/UMR/output/a3_validation \
        --out-dir ~/a3_teleop_ws/logs/a3_validation
"""

from __future__ import annotations

import argparse
import json
import re
import subprocess
import sys
from pathlib import Path

BRIDGE_ROOT = Path(__file__).resolve().parents[1]
PYTHON = sys.executable


def run(cmd: list[str], timeout: int = 1800) -> tuple[int, str]:
    proc = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
    return proc.returncode, proc.stdout + proc.stderr


def parse_roundtrip(text: str) -> dict:
    match = re.search(
        r"roundtrip\s*:\s*pos ([0-9.eE+-]+) m, joints ([0-9.eE+-]+) rad, quat ([0-9.eE+-]+) deg",
        text,
    )
    if not match:
        return {}
    return {
        "root_pos_max_err_m": float(match.group(1)),
        "joint_pos_max_err_rad": float(match.group(2)),
        "root_quat_max_err_deg": float(match.group(3)),
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data-dir", required=True, help="directory of UMR A3 .npz results")
    parser.add_argument("--out-dir", required=True)
    parser.add_argument("--only", default=None)
    parser.add_argument("--skip-mujoco", action="store_true")
    parser.add_argument("--policy-steps", type=int, default=0, help="0 = full clip")
    args = parser.parse_args(argv)

    data_dir = Path(args.data_dir).expanduser().resolve()
    out_dir = Path(args.out_dir).expanduser().resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    clips = sorted(data_dir.glob("*.npz"))
    if args.only:
        wanted = {n.strip() for n in args.only.split(",")}
        clips = [c for c in clips if c.stem.split("_smplx_")[0] in wanted]
    if not clips:
        raise SystemExit(f"no UMR A3 results in {data_dir}")

    results = {}
    for clip_path in clips:
        name = clip_path.stem.split("_smplx_")[0]
        clip_dir = out_dir / name
        clip_dir.mkdir(parents=True, exist_ok=True)
        entry: dict = {"npz": str(clip_path)}

        # ---- numeric validation ------------------------------------------
        code, text = run(
            [
                PYTHON,
                str(BRIDGE_ROOT / "tools" / "validate_a3_motion.py"),
                "--umr-result",
                str(clip_path),
                "--json",
                str(clip_dir / "validation.json"),
            ]
        )
        (clip_dir / "validate.log").write_text(text, encoding="utf-8")
        entry["numeric_validation"] = {
            "returncode": code,
            "ok": code == 0,
            "checks": {},
        }
        report_path = clip_dir / "validation.json"
        if report_path.is_file():
            report = json.loads(report_path.read_text(encoding="utf-8"))
            entry["numeric_validation"]["checks"] = {
                k: v["ok"] for k, v in report.get("checks", {}).items()
            }
            entry["numeric_validation"]["problems"] = report.get("problems", [])

        # ---- CSV export ---------------------------------------------------
        csv_path = clip_dir / f"{name}.csv"
        code, text = run(
            [
                PYTHON,
                "-m",
                "a3_teleop_bridge.apps.retarget_offline",
                "--umr-result",
                str(clip_path),
                "--out-csv",
                str(csv_path),
                "--report",
                str(clip_dir / "export_report.json"),
            ]
        )
        (clip_dir / "export.log").write_text(text, encoding="utf-8")
        entry["csv_export"] = {
            "returncode": code,
            "ok": code == 0,
            "csv": str(csv_path),
            "roundtrip": parse_roundtrip(text),
        }

        # ---- MuJoCo -------------------------------------------------------
        if not args.skip_mujoco and csv_path.is_file():
            cmd = [
                PYTHON,
                str(BRIDGE_ROOT / "tools" / "run_a3_baseline.py"),
                "--motion",
                str(csv_path),
                "--csv-source-fps",
                "30",
                "--csv-frame-stride",
                "1",
                "--log-dir",
                str(clip_dir / "mujoco"),
            ]
            if args.policy_steps:
                cmd += ["--policy-steps", str(args.policy_steps)]
            code, text = run(cmd, timeout=3600)
            (clip_dir / "mujoco.log").write_text(text, encoding="utf-8")
            metrics_path = clip_dir / "mujoco" / "full_metrics.json"
            metrics = (
                json.loads(metrics_path.read_text(encoding="utf-8"))
                if metrics_path.is_file()
                else {}
            )
            entry["mujoco"] = {
                "returncode": code,
                "ok": code == 0,
                "fall": metrics.get("fall"),
                "policy_steps": metrics.get("num_policy_steps"),
                "root_height_mean": (metrics.get("root_height") or {}).get("mean"),
                "root_roll_pitch_abs_max_deg": metrics.get("root_roll_pitch_abs_max_deg"),
                "all_29_rmse": (metrics.get("tracking") or {}).get("all_29_rmse"),
            }

        entry["pass"] = bool(
            entry["numeric_validation"]["ok"]
            and entry["csv_export"]["ok"]
            and entry.get("mujoco", {}).get("ok", True)
        )
        results[name] = entry
        flags = " ".join(
            f"{key}={'ok' if value.get('ok') else 'FAIL'}"
            for key, value in entry.items()
            if isinstance(value, dict) and "ok" in value
        )
        print(f"{name:22s} {'PASS' if entry['pass'] else 'FAIL'}  {flags}")

    summary = {
        "clips": results,
        "passed": sum(1 for e in results.values() if e["pass"]),
        "total": len(results),
    }
    summary["all_pass"] = summary["passed"] == summary["total"]
    (out_dir / "report.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    print(f"\n{summary['passed']}/{summary['total']} clips PASS")
    print(f"report: {out_dir / 'report.json'}")
    return 0 if summary["all_pass"] else 1


if __name__ == "__main__":
    sys.exit(main())
