#!/usr/bin/env python3
"""Live-architecture chain (plan sections 44/49).

Runs the *live* process split -- a bridge publisher driven by the online pipeline
(4-stage: receiver -> solver -> predictor -> publisher) plus SONIC sim2sim
consuming A3_REFERENCE_V1 over ZMQ.  Three reference sources:

    --pico                LIVE PICO headset (needs the XRoboToolkit PC service and
                          a pose sender on port 5556): real teleoperation in MuJoCo
    --recording <dir>     recorded PICO session replayed through online UMR
    (neither)             recorded A3 trajectory replayed through the predictor

Usage:
    # PICO teleoperation in simulation -- see docs/SIM_TELEOP.md
    python tools/run_live_chain.py --pico \\
        --csv $A3WS/logs/a3_validation/stand/stand.csv --policy-steps 3000 --duration 150
    # recorded session (no headset needed)
    python tools/run_live_chain.py --recording $A3WS/recordings/m5_twist_torso_left \\
        --csv $A3WS/logs/a3_validation/endurance_loop.csv --policy-steps 1500
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


def find_sim_python(sonic_root: Path, explicit: str | None) -> Path:
    """Interpreter that has mujoco+torch for the SONIC sim2sim consumer.

    Same resolution order as ``tools/run_a3_baseline.py`` so a machine that runs
    the sim out of a conda env (no ``.venv_sim``) works without a special case.
    """
    if explicit:
        return Path(explicit).expanduser()
    import os

    env_python = os.environ.get("PY_SIM")
    if env_python and Path(env_python).expanduser().is_file():
        return Path(env_python).expanduser()
    candidate = sonic_root / ".venv_sim" / "bin" / "python"
    if candidate.is_file():
        return candidate
    return Path(sys.executable)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--csv",
        required=True,
        help="motion CSV for the policy's own state input; with --pico/--recording "
        "the *reference* comes from the stream instead",
    )
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
        "--pico",
        action="store_true",
        help="drive the bridge from a LIVE PICO stream (--source pico "
        "--backend umr-online): the operator's motion becomes the reference. "
        "Requires the XRoboToolkit PC service and the pose sender on port 5556.",
    )
    parser.add_argument(
        "--startup-wait",
        type=float,
        default=None,
        help="seconds to wait for retarget_live before starting the policy "
        "(default 2.5, 25 with --recording/--pico: the online UMR session needs a "
        "~15 s one-off initialisation)",
    )
    parser.add_argument("--out-dir", default=str(BRIDGE_ROOT.parent / "logs" / "live_chain"))
    parser.add_argument(
        "--sim-python",
        default=None,
        help="interpreter with mujoco+torch for the SONIC sim2sim consumer "
        "(default: $PY_SIM, else $SONIC_A3_ROOT/.venv_sim/bin/python, else this "
        "interpreter -- so a conda env works the same way it does for "
        "tools/run_a3_baseline.py)",
    )
    args = parser.parse_args(argv)

    contract = load_contract()
    out_dir = Path(args.out_dir).expanduser()
    out_dir.mkdir(parents=True, exist_ok=True)
    endpoint = f"tcp://127.0.0.1:{args.port}"
    sim_python = find_sim_python(contract.sonic_root, args.sim_python)
    print(f"[live-chain] sim interpreter: {sim_python}")

    live_cmd = [
        sys.executable,
        "-m",
        "a3_teleop_bridge.apps.retarget_live",
        "--source",
        "pico" if args.pico else ("recording" if args.recording else "trajectory"),
        "--backend",
        "umr-online" if (args.pico or args.recording) else "offline",
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
    online_umr = bool(args.pico or args.recording)
    wait = args.startup_wait if args.startup_wait is not None else (25.0 if online_umr else 2.5)
    time.sleep(wait)

    sim_cmd = [
        str(sim_python),
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
    if args.pico and not stats.get("frames_published"):
        problems.append(
            "no reference was ever published: is the PICO sender RUNNING "
            "(--start_unpaused or the controller's A button) on port 5556?"
        )

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
    if problems:
        print(f"[live-chain] NOT ACCEPTED -- {len(problems)} problem(s):")
        for item in problems:
            print(f"[live-chain]   - {item}")
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
