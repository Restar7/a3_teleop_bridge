#!/usr/bin/env python3
"""Run the official A3-fast MuJoCo sim2sim baseline and record evidence (M1).

This wraps the *repository-official* command documented in
``docs/a3_training2sim2deploy.md`` section 4 -- it does not invent flags:

    python gear_sonic/scripts/sim2sim_a3_mujoco.py \
      --checkpoint "$MODEL_035_PT" \
      --motion a3_data/agibot_a3/001_walk_front_slow.csv \
      --encoder-mode a3_fast \
      --mjcf gear_sonic/data/assets/robot_description/mjcf/a3_t2d5_loop_passive_foot_twostage_fit_optimized.xml \
      --batch-once --metrics-out ... --output-video ...

Acceptance (plan section 6): runs without error, no NaN, 29-DoF mapping intact,
robot does not fall.  Evidence goes to ``logs/baseline/``.

Usage:
    python tools/run_a3_baseline.py --motion a3_data/agibot_a3/001_walk_front_slow.csv
    python tools/run_a3_baseline.py --smoke          # 10 policy steps only
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import subprocess
import sys
import time
from pathlib import Path

BRIDGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(BRIDGE_ROOT / "src"))

from a3_teleop_bridge.contract import load_contract  # noqa: E402

DEFAULT_LOG_DIR = BRIDGE_ROOT.parent / "logs" / "baseline"
CHECKPOINT_REL = "checkpoints/035_step200000/model_step_200000.pt"
EXPECTED_CHECKPOINT_SHA256 = (
    "9cf33be2f4e602858b68ce31d5824113ab1dda250acaab5842b6d3bf88b70f2d"
)


def sha256_file(path: Path, chunk: int = 1 << 20) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(chunk), b""):
            digest.update(block)
    return digest.hexdigest()


def find_python(sonic_root: Path, explicit: str | None) -> Path:
    if explicit:
        return Path(explicit).expanduser()
    candidate = sonic_root / ".venv_sim" / "bin" / "python"
    if candidate.is_file():
        return candidate
    return Path(sys.executable)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sonic-root", default=None)
    parser.add_argument("--python", default=None, help="interpreter with mujoco+torch")
    parser.add_argument("--motion", default="a3_data/agibot_a3/001_walk_front_slow.csv")
    parser.add_argument("--encoder-mode", default="a3_fast")
    parser.add_argument("--checkpoint", default=None)
    parser.add_argument("--mjcf", default=None)
    parser.add_argument("--policy-steps", type=int, default=None, help="limit policy steps")
    parser.add_argument("--action-delay-ms", type=float, default=0.0)
    parser.add_argument("--output-video", default=None)
    parser.add_argument("--log-dir", default=str(DEFAULT_LOG_DIR))
    parser.add_argument(
        "--smoke",
        action="store_true",
        help="10 policy steps, no video (fast regression gate)",
    )
    args = parser.parse_args(argv)

    contract = load_contract()
    sonic_root = Path(args.sonic_root).expanduser().resolve() if args.sonic_root else contract.sonic_root
    python = find_python(sonic_root, args.python)

    checkpoint = Path(args.checkpoint).expanduser() if args.checkpoint else sonic_root / CHECKPOINT_REL
    mjcf = Path(args.mjcf).expanduser() if args.mjcf else contract.mjcf_path
    motion = Path(args.motion)
    if not motion.is_absolute():
        motion = sonic_root / motion

    for label, path in (("checkpoint", checkpoint), ("mjcf", mjcf), ("motion", motion)):
        if not path.is_file():
            raise SystemExit(f"{label} not found: {path}")

    policy_steps = 10 if args.smoke else args.policy_steps
    log_dir = Path(args.log_dir).expanduser()
    log_dir.mkdir(parents=True, exist_ok=True)
    tag = "smoke" if args.smoke else "full"
    metrics_path = log_dir / f"{tag}_metrics.json"
    timeseries_path = log_dir / f"{tag}_timeseries.json"
    log_path = log_dir / f"{tag}_run.log"
    meta_path = log_dir / f"{tag}_metadata.json"

    cmd = [
        str(python),
        "gear_sonic/scripts/sim2sim_a3_mujoco.py",
        "--checkpoint",
        str(checkpoint),
        "--motion",
        str(motion),
        "--encoder-mode",
        args.encoder_mode,
        "--mjcf",
        str(mjcf),
        "--batch-once",
        "--metrics-out",
        str(metrics_path),
        "--timeseries-out",
        str(timeseries_path),
    ]
    if policy_steps:
        cmd += ["--max-policy-steps", str(policy_steps)]
    if args.action_delay_ms:
        cmd += ["--action-delay-ms", str(args.action_delay_ms)]
    if args.output_video:
        cmd += ["--output-video", str(args.output_video)]

    env = dict(os.environ)
    env["MODEL_035_PT"] = str(checkpoint)
    # NOTE: leave MUJOCO_GL alone.  This host has no EGL platform and forcing
    # MUJOCO_GL=egl makes `import mujoco` fail; the default backend works and
    # --batch-once never opens a viewer.

    print("[baseline] " + " ".join(cmd))
    started = time.time()
    with log_path.open("w", encoding="utf-8") as log:
        proc = subprocess.run(cmd, cwd=str(sonic_root), env=env, stdout=log, stderr=subprocess.STDOUT)
    elapsed = time.time() - started

    checkpoint_sha = sha256_file(checkpoint)
    metrics = json.loads(metrics_path.read_text(encoding="utf-8")) if metrics_path.is_file() else {}

    # ---- acceptance -----------------------------------------------------
    problems: list[str] = []
    if proc.returncode != 0:
        problems.append(f"sim2sim exited with code {proc.returncode}")
    if checkpoint_sha != EXPECTED_CHECKPOINT_SHA256:
        problems.append(
            f"checkpoint sha256 {checkpoint_sha} != released {EXPECTED_CHECKPOINT_SHA256}"
        )
    if metrics.get("fall"):
        problems.append(f"fall detected at tick {metrics.get('fall_tick')}")
    if not timeseries_path.is_file():
        problems.append("timeseries output missing")

    nan_frames = 0
    q_min = q_max = None
    samples = 0
    if timeseries_path.is_file():
        import math

        try:
            doc = json.loads(timeseries_path.read_text(encoding="utf-8"))
            motions = doc.get("motions", [])
        except json.JSONDecodeError:
            motions = []
        for motion_entry in motions:
            for values in motion_entry.get("q_des_29", []):
                samples += 1
                if any(v is None or not math.isfinite(float(v)) for v in values):
                    nan_frames += 1
                else:
                    q_min = min(values) if q_min is None else min(q_min, min(values))
                    q_max = max(values) if q_max is None else max(q_max, max(values))
    if nan_frames:
        problems.append(f"{nan_frames} non-finite policy frames in the timeseries")
    if samples == 0:
        problems.append("no policy frames found in the timeseries output")

    metadata = {
        "stage": "M1",
        "tag": tag,
        "command": cmd,
        "cwd": str(sonic_root),
        "elapsed_s": elapsed,
        "returncode": proc.returncode,
        "checkpoint": {
            "path": str(checkpoint),
            "sha256": checkpoint_sha,
            "sha256_matches_release": checkpoint_sha == EXPECTED_CHECKPOINT_SHA256,
            "config_yaml": str(checkpoint.parent / "config.yaml"),
        },
        "mjcf": str(mjcf),
        "urdf": str(contract.urdf_path),
        "motion": str(motion),
        "encoder_mode": args.encoder_mode,
        "policy_hz": contract.policy_hz,
        "policy_dt": contract.policy_dt,
        "reference_window": {
            "frames": contract.window_frames,
            "dt": contract.window_dt,
            "future_horizon_s": contract.future_horizon_s,
            "frame_skip": contract.window_frame_skip,
        },
        "shapes": {
            "policy_joints": contract.n_policy_joints,
            "encoder_input_dim": contract.encoder_input_dim,
            "encoder_frame_dim": contract.encoder_frame_dim,
            "observation_dim": contract.observation_dim,
            "action_dim": contract.action_dim,
        },
        "metrics": metrics,
        "timeseries": {
            "path": str(timeseries_path),
            "samples": samples,
            "non_finite_frames": nan_frames,
            "q_min": q_min,
            "q_max": q_max,
        },
        "log": str(log_path),
        "acceptable": not problems,
        "problems": problems,
    }
    meta_path.write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")

    print(f"[baseline] returncode={proc.returncode} elapsed={elapsed:.1f}s")
    print(f"[baseline] metrics: fall={metrics.get('fall')} fps={metrics.get('fps')}")
    print(f"[baseline] timeseries: {samples} samples, {nan_frames} non-finite")
    if q_min is not None:
        print(f"[baseline] policy q range: [{q_min:.4f}, {q_max:.4f}] rad")
    print(f"[baseline] wrote {meta_path}")
    if problems:
        print("[baseline] FAILED:")
        for line in problems:
            print(f"  ! {line}")
        return 1
    print("[baseline] ACCEPTED (M1)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
