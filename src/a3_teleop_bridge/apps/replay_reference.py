"""Replay an offline A3 trajectory as a live A3_REFERENCE_V1 stream (plan §42).

    A3 trajectory file (flat CSV or UMR npz)
      -> A3CanonicalState
      -> A3ReferencePredictor (10-slot window)
      -> ReferencePublisher (ZMQ, latest-only)

This validates the network / streaming path without any PICO or online-UMR
dependency, and is also the "recorded PICO replay" building block for later.

Usage::

    python -m a3_teleop_bridge.apps.replay_reference \
        --csv ~/a3_teleop_ws/logs/a3_validation/stand/stand.csv \
        --fps 30 --publish-hz 50 --loop --duration 60
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

from ..a3.csv_export import A3FlatCsvCodec
from ..a3.predictor import A3ReferencePredictor
from ..clocks import LatencyStats, now_ns
from ..contract import load_contract
from ..types import A3ReferenceWindow, BridgeState
from ..transport.publisher import NetworkConfig, ReferencePublisher
from ..umr.offline import load_umr_result


def load_states_from_csv(path: Path, fps: float, stride: int = 1):
    """Yield ``A3CanonicalState``-like tuples from a flat CSV."""
    codec = A3FlatCsvCodec()
    data = codec.read(path, source_fps=fps, frame_stride=stride)
    return data


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--csv", default=None, help="flat A3 CSV (native fps)")
    parser.add_argument("--umr-result", default=None, help="UMR .npz result instead of a CSV")
    parser.add_argument("--fps", type=float, default=30.0, help="frame rate of the source file")
    parser.add_argument("--publish-hz", type=float, default=50.0)
    parser.add_argument("--duration", type=float, default=0.0, help="0 = whole trajectory once")
    parser.add_argument("--loop", action="store_true")
    parser.add_argument("--speed", type=float, default=1.0, help="playback speed multiplier")
    parser.add_argument("--endpoint", default=None, help="override the bind endpoint")
    parser.add_argument("--realtime", action="store_true", default=True)
    parser.add_argument("--no-realtime", dest="realtime", action="store_false")
    parser.add_argument("--stats-every", type=float, default=2.0)
    parser.add_argument(
        "--windows",
        default=None,
        help="directory written by tools/record_reference.py: republish a recorded "
        "A3_REFERENCE_V1 stream (plan section 79 -- replay is first class)",
    )
    args = parser.parse_args(argv)

    if args.windows:
        if args.csv or args.umr_result:
            parser.error("--windows cannot be combined with --csv/--umr-result")
    elif not args.csv and not args.umr_result:
        parser.error("pass --csv, --umr-result or --windows")

    contract = load_contract()
    predictor = A3ReferencePredictor()
    network = NetworkConfig.from_yaml()
    publisher = ReferencePublisher(network, contract, bind=args.endpoint)

    if args.windows:
        return replay_recorded_windows(args, publisher)

    # ---- load the source trajectory --------------------------------------
    if args.umr_result:
        from ..umr.state_converter import UmrStateConverter

        result = load_umr_result(args.umr_result)
        converter = UmrStateConverter()
        states = converter.convert_umr_result(result)
        source_dt = result.dt
        label = str(result.path)
    else:
        from ..types import A3CanonicalState

        data = load_states_from_csv(Path(args.csv).expanduser(), args.fps)
        source_dt = 1.0 / data.effective_fps
        states = []
        for i in range(data.n_frames):
            states.append(
                A3CanonicalState(
                    seq=i,
                    timestamp_ns=int(round(i * source_dt * 1e9)),
                    root_pos_m=data.root_pos_m[i],
                    root_quat_wxyz=data.root_quat_wxyz[i],
                    joint_pos_rad=data.dof29_rad[i],
                    joint_vel_rad_s=np.zeros(contract.n_policy_joints),
                    valid=True,
                )
            )
        label = str(data.path)

    if not states:
        print("[replay] the source trajectory is empty")
        return 1
    print(f"[replay] {label}: {len(states)} frames @ {1.0 / source_dt:.1f} fps")
    print(f"[replay] publishing A3_REFERENCE_V1 on {publisher.bind} at {args.publish_hz:g} Hz")

    # ---- publish ----------------------------------------------------------
    period = 1.0 / max(args.publish_hz, 1e-6)
    start = time.perf_counter()
    next_tick = start
    # Playback follows SOURCE time, not the publish rate: a 30 fps clip is held
    # for ~1.67 publish ticks per frame instead of being sped up 1.67x.
    source_time = 0.0
    index = 0
    published = 0
    rejected = 0
    skipped = 0
    age_stats = LatencyStats("publish_period")
    last_report = start
    end_time = start + args.duration if args.duration > 0 else None

    try:
        while True:
            now = time.perf_counter()
            if end_time is not None and now >= end_time:
                break
            index = int(source_time / source_dt)
            if index >= len(states):
                if not args.loop:
                    break
                source_time -= len(states) * source_dt
                index = int(source_time / source_dt)
                # NOTE: never reset the predictor here. A reset would drop the
                # last safe state and make the next window an all-default
                # placeholder, and it would restart the sequence number -- both
                # are protocol/safety violations (seq must be monotonic).

            state = states[index]
            # re-stamp the frame onto the wall clock so the window carries a live
            # timestamp while playing back a file
            state.timestamp_ns = now_ns()
            accepted = predictor.push(state)
            if not accepted:
                rejected += 1
            window = predictor.window(timestamp_ns=now_ns(), source_age_ms=0.0)
            if window.state is BridgeState.DISCONNECTED:
                # no accepted state yet: publishing a default-pose window would
                # look like a valid reference to the policy
                skipped += 1
                source_time += source_dt
                continue
            publisher.send(window)
            published += 1
            source_time += period * max(args.speed, 1e-6)

            if args.realtime:
                next_tick += period
                sleep = next_tick - time.perf_counter()
                if sleep > 0:
                    time.sleep(sleep)
                else:
                    next_tick = time.perf_counter()
                age_stats.add(abs(sleep) * 1e3 if sleep > 0 else -sleep * 1e3)

            if now - last_report >= args.stats_every:
                fps = published / max(now - start, 1e-9)
                print(
                    f"[replay] t={now - start:6.1f}s frames={published:6d} "
                    f"({fps:5.1f} Hz) idx={index}/{len(states)} rejected={rejected} "
                    f"seq={window.seq} lag_ms={age_stats.summary()['p95']:.2f}",
                    flush=True,
                )
                last_report = now
    except KeyboardInterrupt:
        print("\n[replay] interrupted")
    finally:
        elapsed = time.perf_counter() - start
        summary = {
            "frames_published": published,
            "source_frames": len(states),
            "elapsed_s": elapsed,
            "effective_hz": published / max(elapsed, 1e-9),
            "rejected": rejected,
            "skipped_no_state": skipped,
            "last_seq": predictor.seq,
        }
        publisher.close()
        print(f"[replay] done: {summary}")
    return 0


def load_recorded_windows(directory: Path) -> tuple[list[A3ReferenceWindow], dict]:
    """Load a window stream written by ``tools/record_reference.py``.

    Returns the windows in stored order plus the recording metadata (``{}`` when
    the recorder did not write any).  Kept separate from the publish loop so the
    file format can be tested without a live socket.
    """
    data = np.load(directory / "windows.npz")
    windows: list[A3ReferenceWindow] = []
    for i in range(data["seq"].shape[0]):
        windows.append(
            A3ReferenceWindow(
                seq=int(data["seq"][i]),
                timestamp_ns=int(data["timestamp_ns"][i]),
                dt=float(data["dt"][i]),
                root_pos_m=np.asarray(data["root_pos_m"][i], dtype=np.float64),
                root_quat_wxyz=np.asarray(data["root_quat_wxyz"][i], dtype=np.float64),
                joint_pos_rad=np.asarray(data["joint_pos_rad"][i], dtype=np.float64),
                joint_vel_rad_s=np.asarray(data["joint_vel_rad_s"][i], dtype=np.float64),
                source_age_ms=float(data["source_age_ms"][i]),
                valid=bool(data["valid"][i]),
            )
        )
    metadata: dict = {}
    meta_path = directory / "metadata.json"
    if meta_path.is_file():
        metadata = json.loads(meta_path.read_text(encoding="utf-8"))
    return windows, metadata


def replay_recorded_windows(args, publisher) -> int:
    """Publish the windows stored by ``tools/record_reference.py``.

    The stored payload (root/joints/dt) is sent unchanged -- only ``timestamp_ns``
    is re-stamped onto the live clock, because a replayed window must look fresh
    to the consumer's freshness watchdog.  Sequence numbers stay monotonic.
    """
    directory = Path(args.windows).expanduser()
    windows, metadata = load_recorded_windows(directory)
    if not windows:
        print("[replay] the recorded window stream is empty")
        return 1
    print(
        f"[replay] {directory}: {len(windows)} recorded windows "
        f"(recorded publish_hz={metadata.get('publish_hz')})"
    )
    print(f"[replay] republishing A3_REFERENCE_V1 on {publisher.bind} at {args.publish_hz:g} Hz")

    period = 1.0 / max(args.publish_hz, 1e-6)
    started = time.perf_counter()
    end_time = started + args.duration if args.duration > 0 else None
    published = 0
    next_tick = started
    index = 0
    try:
        while True:
            now = time.perf_counter()
            if end_time is not None and now >= end_time:
                break
            if index >= len(windows):
                if not args.loop:
                    break
                index = 0
            window = windows[index]
            window.timestamp_ns = now_ns()  # freshness watchdog needs a live stamp
            publisher.send(window)
            published += 1
            index += 1
            next_tick += period
            if next_tick > time.perf_counter() + 0.05:  # fell behind: resynchronise
                next_tick = time.perf_counter() + period
            if args.realtime:
                delay = next_tick - time.perf_counter()
                if delay > 0:
                    time.sleep(delay)
    except KeyboardInterrupt:
        pass
    finally:
        publisher.close()
    print(f"[replay] published {published} windows")
    return 0


if __name__ == "__main__":
    sys.exit(main())
