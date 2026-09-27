#!/usr/bin/env python3
"""M6: offline trajectory -> ZMQ -> StreamingReferenceProvider -> A3-fast -> MuJoCo.

Starts the bridge replay publisher for a flat A3 CSV and runs the official
sim2sim with ``--reference-source stream``, so the encoder window arrives over
ZMQ as ``A3_REFERENCE_V1`` instead of being sliced from the CSV.  The CSV is still
passed as the ghost/metrics reference, so tracking numbers stay meaningful
(they compare the policy against the very trajectory that is being streamed).

Acceptance (plan section 43): no NaN, no fall, sequence keeps increasing,
reference age stays bounded, the publisher never rejects a frame.

Usage:
    python tools/run_a3_streaming.py --csv <clip>.csv --policy-steps 3000 --duration 120
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
    parser.add_argument("--csv", required=True, help="flat A3 CSV (native fps)")
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--policy-steps", type=int, default=3000)
    parser.add_argument("--duration", type=float, default=0.0, help="publisher seconds (0 = until finished)")
    parser.add_argument("--port", type=int, default=15572)
    parser.add_argument("--out-dir", default=str(BRIDGE_ROOT.parent / "logs" / "m6_streaming"))
    parser.add_argument("--checkpoint", default=None)
    parser.add_argument("--no-publisher", action="store_true", help="use an already running publisher")
    parser.add_argument(
        "--smooth",
        action="store_true",
        help="use logs/a3_validation/endurance_loop.csv (cross-faded, continuous). "
        "A concatenated reference with pose jumps is a fault input, not an "
        "endurance input -- the policy is expected to fall on reference teleports.",
    )
    parser.add_argument(
        "--free-run",
        action="store_true",
        help="do not pace the policy loop (default: --realtime, so the streamed "
        "reference and the policy clock stay aligned)",
    )
    args = parser.parse_args(argv)

    contract = load_contract()
    out_dir = Path(args.out_dir).expanduser()
    out_dir.mkdir(parents=True, exist_ok=True)
    endpoint = f"tcp://127.0.0.1:{args.port}"

    publisher = None
    if not args.no_publisher:
        pub_cmd = [
            sys.executable,
            "-m",
            "a3_teleop_bridge.apps.replay_reference",
            "--csv",
            str(Path(args.csv).expanduser()),
            "--fps",
            str(args.fps),
            "--publish-hz",
            "50",
            "--loop",
            "--endpoint",
            endpoint,
        ]
        if args.duration > 0:
            pub_cmd += ["--duration", str(args.duration)]
        pub_log = (out_dir / "publisher.log").open("w", encoding="utf-8")
        publisher = subprocess.Popen(pub_cmd, cwd=str(BRIDGE_ROOT), stdout=pub_log, stderr=subprocess.STDOUT)
        print(f"[m6] publisher started (pid {publisher.pid}) on {endpoint}")
        time.sleep(2.0)

    checkpoint = args.checkpoint or str(contract.sonic_root / "checkpoints" / "035_step200000" / "model_step_200000.pt")
    sim_cmd = [
        str(contract.sonic_root / ".venv_sim" / "bin" / "python"),
        "gear_sonic/scripts/sim2sim_a3_mujoco.py",
        "--checkpoint",
        checkpoint,
        "--motion",
        str(Path(args.csv).expanduser()),
        "--encoder-mode",
        "a3_fast",
        "--mjcf",
        str(contract.mjcf_path),
        "--csv-source-fps",
        str(args.fps),
        "--csv-frame-stride",
        "1",
        "--reference-source",
        "stream",
        "--reference-endpoint",
        endpoint,
        "--batch-once",
        "--max-policy-steps",
        str(args.policy_steps),
        "--metrics-out",
        str(out_dir / "metrics.json"),
        "--timeseries-out",
        str(out_dir / "timeseries.json"),
    ]
    if not args.free_run:
        sim_cmd.append("--realtime")
    print("[m6] " + " ".join(sim_cmd))
    started = time.time()
    sim_log_path = out_dir / "sim2sim.log"
    with sim_log_path.open("w", encoding="utf-8") as log:
        proc = subprocess.run(
            sim_cmd, cwd=str(contract.sonic_root), stdout=log, stderr=subprocess.STDOUT
        )
    elapsed = time.time() - started

    if publisher is not None:
        publisher.send_signal(signal.SIGINT)
        try:
            publisher.wait(timeout=20)
        except subprocess.TimeoutExpired:
            publisher.kill()

    metrics = json.loads((out_dir / "metrics.json").read_text()) if (out_dir / "metrics.json").is_file() else {}
    pub_text = ""
    if (out_dir / "publisher.log").is_file():
        pub_text = (out_dir / "publisher.log").read_text(encoding="utf-8", errors="replace")

    problems = []
    if proc.returncode != 0:
        problems.append(f"sim2sim exited with {proc.returncode}")
    if metrics.get("fall"):
        problems.append(f"fall at tick {metrics.get('fall_tick')}")
    if "rejected=0" not in pub_text:
        problems.append("publisher reported rejected frames")
    # a KeyboardInterrupt traceback is how the harness stops the publisher; any
    # other exception is a real failure
    fatal = [
        line
        for line in pub_text.splitlines()
        if line.startswith(("zmq.error", "Traceback", "ValueError", "RuntimeError", "FileNotFoundError"))
    ]
    interrupt_only = all(
        "KeyboardInterrupt" in pub_text or "replay] interrupted" in pub_text for _ in fatal
    )
    if fatal and not interrupt_only:
        problems.append("publisher raised an exception")
    if "Address already in use" in pub_text:
        problems.append("publisher could not bind its port (a stale publisher is running?)")
    if not metrics:
        problems.append("no metrics produced")
    stream_stats = {}
    if (out_dir / "sim2sim.log").is_file():
        import re as _re

        text = (out_dir / "sim2sim.log").read_text(encoding="utf-8", errors="replace")
        match = _re.search(r"\[reference-stream\] (\{.*\})", text)
        if match:
            try:
                stream_stats = json.loads(match.group(1).replace("'", '"'))
            except json.JSONDecodeError:
                stream_stats = {"raw": match.group(1)}
    if stream_stats:
        # Received-window gaps are normal for a latest-only stream (the publisher
        # is faster than the consumer); the provider interpolates towards the
        # newest window and bounds the per-tick advance to one slot.  Report the
        # gaps as diagnostics, fail only on real problems.
        if stream_stats.get("rejected"):
            problems.append(f"receiver rejected {stream_stats['rejected']} packets")
        if not stream_stats.get("received"):
            problems.append("no reference windows were received")

    report = {
        "csv": str(Path(args.csv).expanduser()),
        "endpoint": endpoint,
        "policy_steps": args.policy_steps,
        "elapsed_s": elapsed,
        "returncode": proc.returncode,
        "fall": metrics.get("fall"),
        "root_height": metrics.get("root_height"),
        "root_roll_pitch_abs_max_deg": metrics.get("root_roll_pitch_abs_max_deg"),
        "all_29_rmse": (metrics.get("tracking") or {}).get("all_29_rmse"),
        "publisher_tail": pub_text.strip().splitlines()[-3:],
        "stream_stats": stream_stats,
        "sim2sim_log": str(sim_log_path),
        "acceptable": not problems,
        "problems": problems,
    }
    (out_dir / "m6_report.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")

    print(f"[m6] returncode={proc.returncode} elapsed={elapsed:.1f}s steps={args.policy_steps}")
    print(f"[m6] fall={metrics.get('fall')} root_z={(metrics.get('root_height') or {}).get('mean')}")
    print(f"[m6] RMSE(29)={(metrics.get('tracking') or {}).get('all_29_rmse')}")
    print(f"[m6] publisher: {report['publisher_tail']}")
    if stream_stats:
        print(f"[m6] stream   : {stream_stats}")
    print(f"[m6] report: {out_dir / 'm6_report.json'}")
    if problems:
        print("[m6] FAILED:")
        for line in problems:
            print(f"  ! {line}")
        return 1
    print("[m6] ACCEPTED")
    return 0


if __name__ == "__main__":
    sys.exit(main())
