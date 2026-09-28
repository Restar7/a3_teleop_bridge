#!/usr/bin/env python3
"""Stage latency benchmark (plan sections 46, 72).

Measures the bridge-side stages that sit between PICO and the A3 policy and
writes ``benchmarks/4090_live.json``:

  * UMR result -> A3CanonicalState conversion (state converter)
  * predictor window construction (10 slots)
  * A3_REFERENCE_V1 encode / decode
  * ZMQ publish -> receive round trip (localhost)
  * CSV codec read/write

The UMR solver itself is not re-run here; its throughput is recorded separately
(``umr_retarget`` section) from the retarget log, because it depends on the GPU
and the correspondence cache.

Usage:
    python tools/benchmark_latency.py [--umr-result PATH.npz] [--out benchmarks/4090_live.json]
"""

from __future__ import annotations

import argparse
import json
import re
import statistics
import sys
import time
import zmq
from pathlib import Path

import numpy as np

BRIDGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(BRIDGE_ROOT / "src"))

from a3_teleop_bridge import paths  # noqa: E402

from a3_teleop_bridge.a3.csv_export import A3FlatCsvCodec  # noqa: E402
from a3_teleop_bridge.a3.predictor import A3ReferencePredictor  # noqa: E402
from a3_teleop_bridge.clocks import LatencyStats  # noqa: E402
from a3_teleop_bridge.contract import load_contract  # noqa: E402
from a3_teleop_bridge.transport.protocol import decode_packet, encode_packet  # noqa: E402
from a3_teleop_bridge.transport.publisher import (  # noqa: E402
    NetworkConfig,
    ReferencePublisher,
    ReferenceSubscriber,
)
from a3_teleop_bridge.types import A3CanonicalState  # noqa: E402
from a3_teleop_bridge.umr.offline import load_umr_result  # noqa: E402
from a3_teleop_bridge.umr.state_converter import UmrStateConverter  # noqa: E402


def timed(fn, iterations: int, warmup: int = 5) -> LatencyStats:
    for _ in range(warmup):
        fn()
    stats = LatencyStats("stage")
    for _ in range(iterations):
        start = time.perf_counter()
        fn()
        stats.add((time.perf_counter() - start) * 1e3)
    return stats


def find_default_umr_result() -> Path | None:
    candidates = [
        paths.workspace_root() / "UMR" / "output" / "a3_validation" / "stand_smplx_agibot_a3.npz",
        paths.workspace_root() / "UMR" / "output" / "agibot_a3_retarget" / "dance1_subject2_smplx_agibot_a3.npz",
    ]
    for cand in candidates:
        if cand.is_file():
            return cand
    return None


def umr_throughput_from_log() -> dict | None:
    """Read the retarget throughput out of the UMR log if it is available."""
    log = paths.workspace_root() / "logs" / "umr" / "a3_retarget.log"
    if not log.is_file():
        return None
    text = log.read_text(encoding="utf-8", errors="replace")
    rates = [float(m) for m in re.findall(r"retarget forward:.*?([0-9.]+)it/s", text)]
    costs = re.findall(r"cost mean=([0-9.eE+-]+) max=([0-9.eE+-]+)", text)
    if not rates:
        return None
    median_rate = statistics.median(rates)
    return {
        "frames_per_second": median_rate,
        "ms_per_frame": 1000.0 / median_rate if median_rate else None,
        "samples": len(rates),
        "min_hz": min(rates),
        "max_hz": max(rates),
        "cost_mean": float(costs[-1][0]) if costs else None,
        "cost_max": float(costs[-1][1]) if costs else None,
        "note": "UMR correspondence-guided retargeting, 4090, correspondence cached",
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--umr-result", default=None)
    parser.add_argument("--csv", default=None, help="flat CSV for the codec benchmark")
    parser.add_argument("--iterations", type=int, default=400)
    parser.add_argument("--zmq-iterations", type=int, default=200)
    parser.add_argument("--port", type=int, default=15590)
    parser.add_argument("--out", default=str(BRIDGE_ROOT / "benchmarks" / "4090_live.json"))
    args = parser.parse_args(argv)

    contract = load_contract()
    limits_path = BRIDGE_ROOT / "generated" / "a3_joint_limits.yaml"
    from a3_teleop_bridge.a3.limits import load_limits

    limits = load_limits()
    results: dict = {
        "contract": {
            "policy_joints": contract.n_policy_joints,
            "window_frames": contract.window_frames,
            "window_dt": contract.window_dt,
            "future_horizon_s": contract.future_horizon_s,
            "policy_hz": contract.policy_hz,
            "observation_dim": contract.observation_dim,
        }
    }

    # ---- 1) state converter on a real UMR result -------------------------
    umr_path = Path(args.umr_result).expanduser() if args.umr_result else find_default_umr_result()
    converter = UmrStateConverter()
    if umr_path and umr_path.is_file():
        result = load_umr_result(umr_path)
        frame_dicts = [result.frame_joint_dict(i) for i in range(min(400, result.n_frames))]
        roots = result.root_pos[: len(frame_dicts)]
        quats = result.root_quat_wxyz[: len(frame_dicts)]
        index = {"i": 0}

        def convert_one():
            i = index["i"] % len(frame_dicts)
            converter.convert_frame(
                frame_dicts[i],
                seq=i,
                timestamp_ns=int(i * 20e6),
                root_pos_m=roots[i],
                root_quat_wxyz=quats[i],
            )
            index["i"] += 1

        results["umr_state_converter"] = timed(convert_one, args.iterations).to_dict()
        results["umr_result"] = str(umr_path)
    else:
        results["umr_state_converter"] = {"note": "no UMR result available"}

    # ---- 2) predictor ----------------------------------------------------
    predictor = A3ReferencePredictor()
    state_index = {"i": 0}

    def push_and_window():
        i = state_index["i"]
        state = A3CanonicalState(
            seq=i,
            timestamp_ns=int(i * 20e6),
            root_pos_m=np.array([0.0, 0.0, 1.0]),
            root_quat_wxyz=np.array([1.0, 0.0, 0.0, 0.0]),
            joint_pos_rad=limits.default_angle,
            joint_vel_rad_s=np.zeros(contract.n_policy_joints),
        )
        predictor.push(state)
        predictor.window(timestamp_ns=int(i * 20e6), source_age_ms=1.0)
        state_index["i"] += 1

    results["predictor_push_and_window"] = timed(push_and_window, args.iterations).to_dict()

    # ---- 3) protocol encode / decode -------------------------------------
    window = predictor.window(timestamp_ns=0, source_age_ms=1.0)
    packet_holder = {"p": encode_packet(window, contract)}
    results["protocol_encode"] = timed(lambda: packet_holder.__setitem__("p", encode_packet(window, contract)), args.iterations).to_dict()
    results["protocol_decode"] = timed(lambda: decode_packet(packet_holder["p"], contract), args.iterations).to_dict()
    results["packet_bytes"] = len(packet_holder["p"])

    # ---- 4) ZMQ round trip ------------------------------------------------
    context = zmq.Context()
    config = NetworkConfig.from_yaml()
    publisher = ReferencePublisher(config, contract, context=context, bind=f"tcp://127.0.0.1:{args.port}")
    subscriber = ReferenceSubscriber(config, contract, context=context, connect=f"tcp://127.0.0.1:{args.port}")
    try:
        round_trip = LatencyStats("network")
        connected = False
        for _ in range(400):
            publisher.send(window)
            received = subscriber.poll(timeout_ms=20)
            if received is not None and not received.rejected:
                connected = True
                break
        if connected:
            for i in range(args.zmq_iterations):
                publisher.send(window)
                start = time.perf_counter()
                received = subscriber.poll(timeout_ms=50)
                if received is not None and not received.rejected:
                    round_trip.add((time.perf_counter() - start) * 1e3)
            results["zmq_localhost_round_trip"] = round_trip.to_dict()
            results["zmq_stats"] = subscriber.stats()
        else:
            results["zmq_localhost_round_trip"] = {"note": "could not establish pub/sub handshake"}
    finally:
        subscriber.close()
        publisher.close()
        context.term()

    # ---- 5) CSV codec ----------------------------------------------------
    csv_path = Path(args.csv).expanduser() if args.csv else None
    if csv_path is None:
        candidate = BRIDGE_ROOT.parent / "logs" / "a3_validation" / "stand" / "stand.csv"
        csv_path = candidate if candidate.is_file() else None
    if csv_path and csv_path.is_file():
        codec = A3FlatCsvCodec()
        results["csv_read"] = timed(lambda: codec.read(csv_path, source_fps=30.0, frame_stride=1), 20, warmup=2).to_dict()
        data = codec.read(csv_path, source_fps=30.0, frame_stride=1)
        results["csv_frames"] = data.n_frames

    # ---- 6) UMR solver throughput (from the retarget log) ----------------
    results["umr_retarget"] = umr_throughput_from_log()

    # ---- 7) budgets ------------------------------------------------------
    budget = {
        "policy_period_ms": 1000.0 / contract.policy_hz,
        "predictor_target_ms": 1.0,
        "comment": (
            "The 50 Hz policy period is 20 ms. Everything the bridge adds between "
            "the UMR solution and the policy (predictor + encode + network) must stay "
            "well below that; the UMR solver itself may run at 25-50 Hz and is "
            "completed by the predictor (plan sections 47/48)."
        ),
    }
    results["budget"] = budget

    out = Path(args.out).expanduser()
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(results, indent=2) + "\n", encoding="utf-8")

    print(f"{'stage':34s} {'mean':>8s} {'p50':>8s} {'p95':>8s} {'p99':>8s} {'max':>8s}  (ms)")
    for key, value in results.items():
        if isinstance(value, dict) and "p95" in value:
            print(
                f"{key:34s} {value['mean']:8.3f} {value['p50']:8.3f} {value['p95']:8.3f} "
                f"{value['p99']:8.3f} {value['max']:8.3f}"
            )
    if results.get("umr_retarget"):
        u = results["umr_retarget"]
        print(f"{'umr_retarget (log)':34s} {u['ms_per_frame']:8.3f} "
              f"{u['frames_per_second']:8.2f} Hz")
    print(f"\nwrote {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
