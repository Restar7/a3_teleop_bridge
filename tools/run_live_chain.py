#!/usr/bin/env python3
"""Live-architecture chain test (plan sections 44/49), hardware-free.

Runs the *live* process split -- a bridge publisher driven by the online pipeline
(4-stage: receiver -> solver -> predictor -> publisher) plus SONIC sim2sim
consuming A3_REFERENCE_V1 over ZMQ -- with a recorded trajectory standing in for
PICO+UMR.  This exercises exactly the code M7 will use; only the headset is
missing.

Usage:
    python tools/run_live_chain.py --csv <clip>.csv --policy-steps 3000 --duration 120
"""

from __future__ import annotations

import argparse
import json
import signal
import subprocess
import sys
import time
from pathlib import Path

BRIDGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(BRIDGE_ROOT / "src"))

from a3_teleop_bridge.contract import load_contract  # noqa: E402


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--csv", required=True)
    parser.add_argument("--csv-fps", type=float, default=30.0)
    parser.add_argument("--policy-steps", type=int, default=3000)
    parser.add_argument("--duration", type=float, default=120.0, help="pipeline seconds")
    parser.add_argument("--port", type=int, default=15640)
    parser.add_argument(
        "--recording",
        default=None,
        help="recorded PICO session dir: drives the bridge with the *online UMR* "
        "backend (--source recording --backend umr-online) instead of the replay "
        "backend.  This is the hardware-free M7 chain: only the headset is missing.",
    )
    parser.add_argument(
        "--startup-wait",
        type=float,
        default=None,
        help="seconds to wait for retarget_live before starting the policy "
        "(default 2.5, or 25 with --recording: the online UMR session needs a "
        "~15 s one-off initialisation)",
    )
    parser.add_argument("--out-dir", default=str(BRIDGE_ROOT.parent / "logs" / "live_chain"))
    args = parser.parse_args(argv)

    contract = load_contract()
    out_dir = Path(args.out_dir).expanduser()
    out_dir.mkdir(parents=True, exist_ok=True)
    endpoint = f"tcp://127.0.0.1:{args.port}"

    live_cmd = [
        sys.executable,
        "-m",
        "a3_teleop_bridge.apps.retarget_live",
        "--source",
        "recording" if args.recording else "trajectory",
        "--backend",
        "umr-online" if args.recording else "offline",
        "--csv",
        str(Path(args.csv).expanduser()),
        "--csv-fps",
        str(args.csv_fps),
        "--duration",
        str(args.duration + 30.0),
        "--endpoint",
        endpoint,
        "--stats",
        str(out_dir / "pipeline_stats.json"),
    ]
    if args.recording:
        live_cmd += ["--recording", str(Path(args.recording).expanduser())]
    live_log = (out_dir / "retarget_live.log").open("w", encoding="utf-8")
    live = subprocess.Popen(live_cmd, cwd=str(BRIDGE_ROOT), stdout=live_log, stderr=subprocess.STDOUT)
    print(f"[live-chain] retarget_live started (pid {live.pid}) on {endpoint}")
    wait = args.startup_wait if args.startup_wait is not None else (25.0 if args.recording else 2.5)
    time.sleep(wait)

    sim_cmd = [
        str(contract.sonic_root / ".venv_sim" / "bin" / "python"),
        "gear_sonic/scripts/sim2sim_a3_mujoco.py",
        "--checkpoint",
        str(contract.sonic_root / "checkpoints" / "035_step200000" / "model_step_200000.pt"),
        "--motion",
        str(Path(args.csv).expanduser()),
        "--encoder-mode",
        "a3_fast",
        "--mjcf",
        str(contract.mjcf_path),
        "--csv-source-fps",
        str(args.csv_fps),
        "--csv-frame-stride",
        "1",
        "--reference-source",
        "stream",
        "--reference-endpoint",
        endpoint,
        "--batch-once",
        "--realtime",
        "--max-policy-steps",
        str(args.policy_steps),
        "--metrics-out",
        str(out_dir / "metrics.json"),
        "--timeseries-out",
        str(out_dir / "timeseries.json"),
    ]
    started = time.time()
    with (out_dir / "sim2sim.log").open("w", encoding="utf-8") as log:
        proc = subprocess.run(sim_cmd, cwd=str(contract.sonic_root), stdout=log, stderr=subprocess.STDOUT)
    elapsed = time.time() - started

    live.send_signal(signal.SIGINT)
    try:
        live.wait(timeout=25)
    except subprocess.TimeoutExpired:
        live.kill()

    metrics = json.loads((out_dir / "metrics.json").read_text()) if (out_dir / "metrics.json").is_file() else {}
    stats = (
        json.loads((out_dir / "pipeline_stats.json").read_text())
        if (out_dir / "pipeline_stats.json").is_file()
        else {}
    )
    sim_log = (out_dir / "sim2sim.log").read_text(encoding="utf-8", errors="replace")
    import re

    match = re.search(r"\[reference-stream\] (\{.*\})", sim_log)
    stream_stats = json.loads(match.group(1).replace("'", '"')) if match else {}

    problems = []
    if proc.returncode != 0:
        problems.append(f"sim2sim exited with {proc.returncode}")
    if metrics.get("fall"):
        problems.append(f"fall at tick {metrics.get('fall_tick')}")
    if not metrics:
        problems.append("no metrics produced")
    if stats.get("rejected"):
        problems.append(f"pipeline rejected {stats['rejected']} frames")
    if not stats.get("frames_published"):
        problems.append("pipeline published nothing")

    report = {
        "csv": str(Path(args.csv).expanduser()),
        "endpoint": endpoint,
        "policy_steps": args.policy_steps,
        "elapsed_s": elapsed,
        "fall": metrics.get("fall"),
        "root_height": metrics.get("root_height"),
        "all_29_rmse": (metrics.get("tracking") or {}).get("all_29_rmse"),
        "pipeline_stats": stats,
        "stream_stats": stream_stats,
        "acceptable": not problems,
        "problems": problems,
    }
    (out_dir / "live_chain_report.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    # report the steps the sim *actually* executed: the metrics file is written by
    # the policy loop, so a motion CSV that runs out early shows up here instead
    # of being masked by the requested --policy-steps
    executed = metrics.get("num_policy_steps")
    print(
        f"[live-chain] returncode={proc.returncode} elapsed={elapsed:.1f}s "
        f"steps={executed} (requested {args.policy_steps})"
    )
    print(f"[live-chain] fall={metrics.get('fall')} root_z={(metrics.get('root_height') or {}).get('mean')}")
    print(f"[live-chain] states={stats.get('state_history')} published={stats.get('frames_published')}")
    print(f"[live-chain] solver p50={((stats.get('solver_latency_ms') or {}).get('p50'))} ms, "
          f"e2e p95={((stats.get('end_to_end_ms') or {}).get('p95'))} ms")
    print(f"[live-chain] report: {out_dir / 'live_chain_report.json'}")
    if problems:
        print("[live-chain] FAILED:")
        for line in problems:
            print(f"  ! {line}")
        return 1
    print("[live-chain] ACCEPTED")
    return 0


if __name__ == "__main__":
    sys.exit(main())
