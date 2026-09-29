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
        --csv $A3WS/logs/a3_validation/endurance_loop.csv --policy-steps 1500   # or bound it
"""

from __future__ import annotations

import argparse
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

BRIDGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(BRIDGE_ROOT / "src"))

from a3_teleop_bridge.contract import load_contract  # noqa: E402


def parse_stream_stats(sim_log: str) -> dict:
    """The ``[reference-stream] {...}`` dict the SONIC consumer prints.

    It is a *Python* dict repr, so it can contain ``None`` (``last_seq`` is None
    until the first packet arrives).  ``json.loads`` with the usual quote swap
    cannot parse that, which used to turn the most common operator mistake --
    starting the chain while the PICO sender is still PAUSED -- into a
    JSONDecodeError traceback instead of the "is the PICO sender RUNNING?"
    message.  Parse it as the Python literal it is, and never raise.
    """
    import ast
    import re

    if not sim_log:
        return {}
    # last match wins: earlier lines may be the startup-pose warning
    matches = re.findall(r"\[reference-stream\] (\{.*\})", sim_log)
    for candidate in reversed(matches):
        try:
            value = ast.literal_eval(candidate)
        except (ValueError, SyntaxError):
            continue
        if isinstance(value, dict):
            return value
    return {}


def bridge_environment() -> dict:
    """Environment for the bridge process, with the math runtimes pinned to 1 thread.

    ``torch.set_num_threads(1)`` inside the session is not enough: torch's
    OpenMP pool is created at *import* time, sized to the whole machine, and its
    threads spin between parallel regions.  Measured on this 16-core box, the
    bridge sat at **~700% CPU across 35 threads** while its actual work is under
    one core -- four to five cores burned on barrier waits, taken straight from
    the policy running in the sim next to it.  Naming the thread count in the
    *environment* means the pool is built with one worker and never spins:

        solver p50   30.0 ms -> 21.6 ms   (1.39x, same clip, same machine)

    Only the bridge is affected.  The simulator keeps its own defaults, because
    the A3 policy there is a much larger workload that does benefit from
    threads.  ``A3_OMP_THREADS`` overrides, ``0``/``off`` leaves it alone.
    """
    env = dict(os.environ)
    raw = env.get("A3_OMP_THREADS", "1").strip().lower()
    if raw in ("0", "off", "default", "none"):
        return env
    threads = raw or "1"
    for name in ("OMP_NUM_THREADS", "MKL_NUM_THREADS", "OPENBLAS_NUM_THREADS", "NUMEXPR_NUM_THREADS"):
        env.setdefault(name, threads)
    return env


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
    parser.add_argument(
        "--policy-steps",
        type=int,
        default=None,
        help="simulator step budget; omit (or pass 0) to run until the window is closed",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=None,
        help="bridge seconds; omit (or pass 0) to publish until interrupted",
    )
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
        "--dump-frames",
        default=None,
        help="write a per-frame JSONL of the online session's diagnostics "
        "(source leg angles vs the reference joints it produced); analyse with "
        "tools/report_live_dump.py",
    )
    parser.add_argument(
        "--reference-startup-wait-s",
        type=float,
        default=90.0,
        help="how long the simulator may hold its startup pose before giving up "
        "on the first reference packet (default 90 s; the bridge needs ~15 s to "
        "assemble plus its first calibration frames)",
    )
    parser.add_argument(
        "--viewer",
        action="store_true",
        help="open the MuJoCo passive viewer window instead of running headless. "
        "Needs a display (X11/Wayland); Space pauses, './,' step, ',' rewinds, "
        "'R' resets, closing the window stops the run.  Metrics are written the "
        "same way either way.",
    )
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
        # The bridge should outlive the simulator it feeds, so it gets the sim's
        # budget plus a margin.  With no budget (open-ended session) it must not
        # have a deadline either -- 0 means "publish until interrupted".
        str(0.0 if not args.duration else args.duration + 30.0),
        "--endpoint",
        endpoint,
        "--stats",
        str(out_dir / "pipeline_stats.json"),
    ]
    online_umr = bool(args.pico or args.recording)
    if args.recording:
        live_cmd += ["--recording", str(Path(args.recording).expanduser())]
    dump_path = args.dump_frames
    if dump_path is None and os.environ.get("A3_LIVE_DUMP"):
        dump_path = os.environ["A3_LIVE_DUMP"]
    if dump_path is None and online_umr:
        dump_path = str(out_dir / "live_frames.jsonl")
    if dump_path:
        live_cmd += ["--dump-frames", str(dump_path)]
    live_log = (out_dir / "retarget_live.log").open("w", encoding="utf-8")
    live = subprocess.Popen(
        live_cmd,
        cwd=str(BRIDGE_ROOT),
        stdout=live_log,
        stderr=subprocess.STDOUT,
        env=bridge_environment(),
    )
    print(f"[live-chain] retarget_live started (pid {live.pid}) on {endpoint}")
    # Wait until the bridge is really publishing instead of guessing a delay.
    # A fixed 25 s was not enough on a real headset: the online UMR session needs
    # ~15 s to assemble and the auto-calibration then needs its first frames, so
    # the simulator could start after the bridge had already gone quiet and then
    # spend its whole 30 s startup window waiting for packets that never came.
    stats_path = out_dir / "pipeline_stats.json"
    if args.startup_wait is not None:
        wait = float(args.startup_wait)
        print(f"[live-chain] waiting {wait:.0f}s for the bridge (explicit --startup-wait)")
        time.sleep(wait)
    elif online_umr:
        budget = float(os.environ.get("A3_STARTUP_BUDGET_S", "150"))
        print(f"[live-chain] waiting for the first published reference (up to {budget:.0f}s)")
        waited, published = 0.0, 0
        while waited < budget:
            time.sleep(1.0)
            waited += 1.0
            if live.poll() is not None:
                break
            try:
                published = int(json.loads(stats_path.read_text()).get("frames_published", 0))
            except Exception:
                published = 0
            if published > 0:
                break
            if int(waited) % 10 == 0:
                print(f"[live-chain]   still waiting ({waited:.0f}s, published={published})")
        if published > 0:
            print(f"[live-chain] bridge is publishing ({published} frames after {waited:.0f}s)")
        else:
            print(f"[live-chain] WARNING: nothing published after {waited:.0f}s -- starting the "
                  f"simulator anyway; it will hold its startup pose and say why")
    else:
        time.sleep(2.5)

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
        "--reference-startup-wait-s",
        str(args.reference_startup_wait_s),
        "--realtime",
    ]
    if args.policy_steps:
        sim_cmd += ["--max-policy-steps", str(int(args.policy_steps))]
    sim_cmd += [
        "--metrics-out",
        str(out_dir / "metrics.json"),
        "--timeseries-out",
        str(out_dir / "timeseries.json"),
    ]
    # --batch-once explicitly turns the passive viewer off ("batch mode: run each
    # CSV once without the passive viewer"), so the interactive path simply omits
    # it.  Both paths stop at the end of the reference and both write metrics, so
    # the acceptance numbers stay comparable; the viewer just adds Space/./,/R
    # controls and a window you can close to stop.
    if not args.viewer:
        sim_cmd.append("--batch-once")
    else:
        # A viewer session is interactive: leaving the robot on the floor after the
        # first fall makes the rest of the run useless, so stand it back up on the
        # live reference pose.  Batch/acceptance runs must NOT do this -- they score
        # the fall and would be measuring the recovery instead.
        sim_cmd.append("--reset-on-fall")
    viewer = bool(args.viewer)
    if viewer and not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
        print("[live-chain] --viewer requested but no DISPLAY/WAYLAND_DISPLAY; running headless")
        viewer = False
    print(f"[live-chain] sim mode: {'MuJoCo viewer window' if viewer else 'headless (batch-once)'}")
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
    stream_stats = parse_stream_stats(sim_log)

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
