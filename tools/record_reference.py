#!/usr/bin/env python3
"""Record an ``A3_REFERENCE_V1`` stream to disk (plan sections 78/79/86).

Subscribes to a reference publisher and stores every distinct window as an npz,
so a real-robot problem can be replayed without the robot (and without PICO or
UMR): ``apps/replay_reference.py --windows <dir>`` publishes the recording again.

Usage:
    python tools/record_reference.py --endpoint tcp://127.0.0.1:5560 \\
        --duration 20 --out examples/reference_recording/stand_20s
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

BRIDGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(BRIDGE_ROOT / "src"))

from a3_teleop_bridge.contract import load_contract  # noqa: E402
from a3_teleop_bridge.transport.publisher import NetworkConfig, ReferenceSubscriber  # noqa: E402


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--endpoint", default=None, help="connect endpoint (default: network.yaml)")
    parser.add_argument("--duration", type=float, default=20.0, help="seconds to record")
    parser.add_argument("--out", required=True, help="output directory")
    parser.add_argument("--idle-timeout", type=float, default=5.0, help="stop after this silence")
    args = parser.parse_args(argv)

    contract = load_contract()
    network = NetworkConfig.from_yaml()
    out_dir = Path(args.out).expanduser()
    out_dir.mkdir(parents=True, exist_ok=True)

    windows: list = []
    seqs: set[int] = set()
    started = time.perf_counter()
    last_rx = started

    with ReferenceSubscriber(network, contract, connect=args.endpoint) as subscriber:
        print(f"[record-reference] listening on {subscriber.connect} for {args.duration:g}s")
        while True:
            now = time.perf_counter()
            if now - started >= args.duration:
                break
            if windows and now - last_rx >= args.idle_timeout:
                print("[record-reference] publisher went quiet; stopping")
                break
            received = subscriber.poll(timeout_ms=50)
            if received is None or received.rejected:
                continue
            # latest-only socket: the same window can be seen twice, keep it once
            if received.window.seq in seqs:
                continue
            seqs.add(int(received.window.seq))
            windows.append((received))
            last_rx = time.perf_counter()

    if not windows:
        print("[record-reference] nothing received; is a publisher running?")
        return 1

    windows_sorted = sorted(windows, key=lambda w: int(w.window.seq))
    np.savez_compressed(
        out_dir / "windows.npz",
        seq=np.asarray([int(w.window.seq) for w in windows_sorted], dtype=np.int64),
        timestamp_ns=np.asarray([int(w.window.timestamp_ns) for w in windows_sorted], dtype=np.int64),
        receive_timestamp_ns=np.asarray(
            [int(w.receive_timestamp_ns) for w in windows_sorted], dtype=np.int64
        ),
        dt=np.asarray([float(w.window.dt) for w in windows_sorted], dtype=np.float32),
        root_pos_m=np.stack([np.asarray(w.window.root_pos_m) for w in windows_sorted]),
        root_quat_wxyz=np.stack([np.asarray(w.window.root_quat_wxyz) for w in windows_sorted]),
        joint_pos_rad=np.stack([np.asarray(w.window.joint_pos_rad) for w in windows_sorted]),
        joint_vel_rad_s=np.stack([np.asarray(w.window.joint_vel_rad_s) for w in windows_sorted]),
        source_age_ms=np.asarray([float(w.window.source_age_ms) for w in windows_sorted]),
        valid=np.asarray([bool(w.window.valid) for w in windows_sorted], dtype=bool),
    )

    latencies = np.asarray([w.network_latency_ms for w in windows_sorted], dtype=np.float64)
    latencies = latencies[np.isfinite(latencies)]
    span_s = (windows_sorted[-1].receive_timestamp_ns - windows_sorted[0].receive_timestamp_ns) / 1e9
    metadata = {
        "endpoint": subscriber.connect,
        "windows": len(windows_sorted),
        "first_seq": int(windows_sorted[0].window.seq),
        "last_seq": int(windows_sorted[-1].window.seq),
        "span_s": float(span_s),
        "publish_hz": float((len(windows_sorted) - 1) / span_s) if span_s > 0 else None,
        "dt": float(np.asarray([w.window.dt for w in windows_sorted]).mean()),
        "horizon": int(np.asarray(windows_sorted[0].window.joint_pos_rad).shape[0]),
        "n_policy_joints": int(np.asarray(windows_sorted[0].window.joint_pos_rad).shape[1]),
        "network_latency_ms": {
            "p50": float(np.percentile(latencies, 50)) if latencies.size else None,
            "p95": float(np.percentile(latencies, 95)) if latencies.size else None,
        },
        "rejected": int(subscriber.rejected),
        "stats": subscriber.stats(),
    }
    (out_dir / "metadata.json").write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
    print(
        f"[record-reference] {metadata['windows']} windows "
        f"(seq {metadata['first_seq']}..{metadata['last_seq']}) over {span_s:.1f}s "
        f"-> {out_dir}"
    )
    print(f"[record-reference] publish_hz={metadata['publish_hz']}, rejected={metadata['rejected']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
