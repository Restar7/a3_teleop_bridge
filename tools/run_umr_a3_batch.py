#!/usr/bin/env python3
"""Run the UMR A3 retarget for many SMPL-X clips (plan sections 17, 21).

The pipeline caches results, so this tool asks it for its exact retarget command
with ``--dry-run`` and then re-executes that same command with ``--data``,
``--seq-key`` and ``--out`` swapped per clip.  Nothing about the retargeting
algorithm or its flags is invented here.

Usage:
    python tools/run_umr_a3_batch.py --data-dir ~/a3_teleop_ws/data/smplx_validation \
        --out-dir ~/a3_teleop_ws/UMR/output/a3_validation
"""

from __future__ import annotations

import argparse
import re
import shlex
import subprocess
import sys
from pathlib import Path

BRIDGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(BRIDGE_ROOT / "src"))

from a3_teleop_bridge.contract import load_contract  # noqa: E402

DEFAULT_CONFIG = "robot_configs/humanoid_retarget_agibot_a3.json"


def find_umr_root(explicit: str | None) -> Path:
    candidates = []
    if explicit:
        candidates.append(Path(explicit))
    candidates.append(Path.home() / "a3_teleop_ws" / "UMR")
    candidates.append(BRIDGE_ROOT.parent / "UMR")
    for cand in candidates:
        if (cand / "scripts" / "humanoid_retarget_pipeline.py").is_file():
            return cand.expanduser().resolve()
    raise SystemExit("could not locate the UMR checkout")


def base_command(umr_root: Path, python: Path, config: str, env: dict) -> list[str]:
    """Ask the pipeline for the retarget command it would run."""
    proc = subprocess.run(
        [
            str(python),
            "scripts/humanoid_retarget_pipeline.py",
            "--config",
            config,
            "--skip-view",
            "--force-retarget",  # always emit the retarget command, never the cache hit
            "--dry-run",
        ],
        cwd=str(umr_root),
        env=env,
        capture_output=True,
        text=True,
        timeout=900,
    )
    if proc.returncode != 0:
        raise SystemExit(f"dry-run failed:\n{proc.stdout[-2000:]}\n{proc.stderr[-2000:]}")
    for line in proc.stdout.splitlines():
        if "retarget_smpl_to_humanoid_surface_vector.py" in line:
            # the pipeline prefixes its delegated commands with "[HumanoidPipeline] "
            text = line.split("[HumanoidPipeline]", 1)[-1].strip()
            return shlex.split(text)
    raise SystemExit("could not find the retarget command in the pipeline dry-run output")


def swap(argv: list[str], flag: str, value: str) -> list[str]:
    out = list(argv)
    if flag in out:
        idx = out.index(flag)
        if idx + 1 < len(out):
            out[idx + 1] = value
            return out
    out.extend([flag, value])
    return out


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--umr-root", default=None)
    parser.add_argument("--python", default=None)
    parser.add_argument("--config", default=DEFAULT_CONFIG)
    parser.add_argument("--data-dir", required=True, help="directory of SMPL-X .npz clips")
    parser.add_argument("--out-dir", default=None, help="default <umr>/output/a3_validation")
    parser.add_argument("--only", default=None, help="comma separated clip names")
    parser.add_argument("--force", action="store_true", help="redo clips that already exist")
    args = parser.parse_args(argv)

    contract = load_contract()
    umr_root = find_umr_root(args.umr_root)
    python = Path(args.python).expanduser() if args.python else umr_root / ".venv_umr" / "bin" / "python"
    if not python.is_file():
        raise SystemExit(f"UMR python not found: {python}")

    data_dir = Path(args.data_dir).expanduser().resolve()
    if not data_dir.is_dir():
        raise SystemExit(f"data dir not found: {data_dir}")
    out_dir = Path(args.out_dir).expanduser() if args.out_dir else umr_root / "output" / "a3_validation"
    out_dir.mkdir(parents=True, exist_ok=True)

    clips = sorted(p.stem for p in data_dir.glob("*.npz"))
    if args.only:
        wanted = {name.strip() for name in args.only.split(",")}
        clips = [c for c in clips if c in wanted]
    if not clips:
        raise SystemExit(f"no .npz clips in {data_dir}")

    import os

    env = dict(os.environ)
    env.setdefault("SONIC_A3_ROOT", str(contract.sonic_root))

    base = base_command(umr_root, python, args.config, env)
    print(f"[batch] base retarget command has {len(base)} tokens")

    failures = []
    for clip in clips:
        out_path = out_dir / f"{clip}_smplx_agibot_a3.npz"
        if out_path.is_file() and not args.force:
            print(f"[batch] {clip:22s} cached")
            continue
        cmd = swap(base, "--data", str(data_dir))
        cmd = swap(cmd, "--seq-key", clip)
        cmd = swap(cmd, "--out", str(out_path))
        print(f"[batch] {clip:22s} running …", flush=True)
        proc = subprocess.run(cmd, cwd=str(umr_root), env=env, capture_output=True, text=True)
        if proc.returncode != 0:
            failures.append(clip)
            print(f"[batch] {clip:22s} FAILED")
            print(proc.stdout[-1500:])
            print(proc.stderr[-1500:])
            continue
        cost = ""
        match = re.search(r"cost mean=([0-9.eE+-]+) max=([0-9.eE+-]+)", proc.stdout)
        if match:
            cost = f"cost mean={float(match.group(1)):.4f} max={float(match.group(2)):.4f}"
        print(f"[batch] {clip:22s} OK  {cost}  -> {out_path.name}")

    print(f"\n[batch] {len(clips) - len(failures)}/{len(clips)} clips retargeted into {out_dir}")
    if failures:
        print(f"[batch] failures: {failures}")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
