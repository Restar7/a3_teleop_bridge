#!/usr/bin/env python3
"""Run the UMR A3 retarget for many SMPL-X clips (plan sections 17, 21).

The UMR pipeline caches by sequence and resolves its motion source through a
*defaults* file, so this tool writes a per-clip defaults file (a copy of the
pipeline defaults with ``motion.data``/``motion.seq_key`` overridden) and lets the
pipeline do the work.  Nothing about the retargeting algorithm or its flags is
invented here.

Usage:
    python tools/run_umr_a3_batch.py --data-dir ~/a3_teleop_ws/data/smplx_validation
"""

from __future__ import annotations

import argparse
import json
import os
import re
import shutil
import subprocess
import sys
from pathlib import Path

BRIDGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(BRIDGE_ROOT / "src"))

from a3_teleop_bridge.contract import load_contract  # noqa: E402

DEFAULT_CONFIG = "robot_configs/humanoid_retarget_agibot_a3.json"
DEFAULTS_NAME = "humanoid_retarget_defaults_a3_validation.json"


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


def write_defaults(umr_root: Path, data_dir: Path, seq_key: str) -> Path:
    """Copy the pipeline defaults and point the motion at one clip.

    The file is written inside the UMR root so the pipeline's relative asset
    paths keep resolving exactly as they do for its own defaults file.
    """
    source = umr_root / "humanoid_retarget_defaults.json"
    doc = json.loads(source.read_text(encoding="utf-8"))
    doc.setdefault("motion", {})
    doc["motion"].update(
        {
            "data": str(data_dir),
            "seq_key": seq_key,
            "seq_index": 0,
            "start": 0,
            "end": -1,
            "stride": 1,
            "max_frames": 0,
        }
    )
    target = umr_root / DEFAULTS_NAME
    target.write_text(json.dumps(doc, indent=2) + "\n", encoding="utf-8")
    return target


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--umr-root", default=None)
    parser.add_argument("--python", default=None)
    parser.add_argument("--config", default=DEFAULT_CONFIG)
    parser.add_argument("--data-dir", required=True, help="directory of SMPL-X .npz clips")
    parser.add_argument("--out-dir", default=None, help="default <umr>/output/a3_validation")
    parser.add_argument("--only", default=None, help="comma separated clip names")
    parser.add_argument("--force", action="store_true", help="redo clips that already exist")
    parser.add_argument("--robot-name", default="agibot_a3")
    args = parser.parse_args(argv)

    contract = load_contract()
    umr_root = find_umr_root(args.umr_root)
    python = (
        Path(args.python).expanduser()
        if args.python
        else umr_root / ".venv_umr" / "bin" / "python"
    )
    if not python.is_file():
        raise SystemExit(f"UMR python not found: {python}")

    data_dir = Path(args.data_dir).expanduser().resolve()
    if not data_dir.is_dir():
        raise SystemExit(f"data dir not found: {data_dir}")
    out_dir = (
        Path(args.out_dir).expanduser()
        if args.out_dir
        else umr_root / "output" / "a3_validation"
    )
    out_dir.mkdir(parents=True, exist_ok=True)

    clips = sorted(p.stem for p in data_dir.glob("*.npz"))
    if args.only:
        wanted = {name.strip() for name in args.only.split(",")}
        clips = [c for c in clips if c in wanted]
    if not clips:
        raise SystemExit(f"no .npz clips in {data_dir}")

    env = dict(os.environ)
    env.setdefault("SONIC_A3_ROOT", str(contract.sonic_root))

    failures = []
    for clip in clips:
        out_path = out_dir / f"{clip}_smplx_{args.robot_name}.npz"
        if out_path.is_file() and not args.force:
            print(f"[batch] {clip:22s} cached")
            continue

        defaults_path = write_defaults(umr_root, data_dir, clip)
        cmd = [
            str(python),
            "scripts/humanoid_retarget_pipeline.py",
            "--config",
            args.config,
            "--defaults",
            defaults_path.name,
            "--skip-view",
            "--stage",
            "retarget",
            # the pipeline caches by output path; ALWAYS redo the retarget so a
            # regenerated source clip cannot silently reuse a stale result
            "--force-retarget",
        ]
        print(f"[batch] {clip:22s} running …", flush=True)
        proc = subprocess.run(cmd, cwd=str(umr_root), env=env, capture_output=True, text=True)
        produced = (
            umr_root
            / "output"
            / f"{args.robot_name}_retarget"
            / f"{clip}_smplx_{args.robot_name}.npz"
        )
        if proc.returncode != 0 or not produced.is_file():
            failures.append(clip)
            print(f"[batch] {clip:22s} FAILED")
            print("\n".join(proc.stdout.splitlines()[-12:]))
            print("\n".join(proc.stderr.splitlines()[-12:]))
            continue
        shutil.copy2(produced, out_path)
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
