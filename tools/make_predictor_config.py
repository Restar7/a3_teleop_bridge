#!/usr/bin/env python3
"""Generate ``configs/predictor.yaml`` from the extracted A3 limits (plan §35).

Every real limit is taken from the model/config data (plan section 35: "所有真实
limit 优先从 A3 config 提取"); only the filter and watchdog constants are
engineering choices and are marked as such.

Usage:
    python tools/make_predictor_config.py [--out PATH]
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import yaml

BRIDGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(BRIDGE_ROOT / "src"))

from a3_teleop_bridge.a3.limits import load_limits  # noqa: E402
from a3_teleop_bridge.contract import load_contract  # noqa: E402

DEFAULT_OUT = BRIDGE_ROOT / "configs" / "predictor.yaml"


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", default=str(DEFAULT_OUT))
    args = parser.parse_args(argv)

    contract = load_contract()
    limits = load_limits()

    doc = {
        "schema": "a3_predictor/v1",
        "generated_by": "tools/make_predictor_config.py",
        "source": "generated/a3_joint_limits.yaml + generated/a3_contract.json",
        # ---- real limits (from the A3 model / deploy config) --------------
        "joint_velocity_limit": [round(float(v), 9) for v in limits.velocity],
        "joint_acceleration_limit": [round(float(v) * 20.0, 9) for v in limits.velocity],
        "joint_position_lower": [round(float(v), 9) for v in limits.lower],
        "joint_position_upper": [round(float(v), 9) for v in limits.upper],
        "root_velocity_limit": 2.0,
        "root_angular_velocity_limit": 4.0,
        # ---- smoothing (engineering choice, low lag) ---------------------
        "filter_min_cutoff": 1.5,
        "filter_beta": 0.05,
        "filter_d_cutoff": 1.0,
        # ---- watchdog (plan section 36) ----------------------------------
        "hold_after_ms": 50.0,
        "invalid_after_ms": 250.0,
        "max_source_gap_s": 0.5,
        "notes": [
            "joint_velocity_limit is the hardware velocity limit from the A3 URDF",
            "joint_acceleration_limit defaults to 20x the velocity limit as a sanity bound",
            "root_velocity_limit [m/s] and root_angular_velocity_limit [rad/s] are reference bounds",
            f"window: {contract.window_frames} frames x {contract.window_dt:g} s = {contract.future_horizon_s:g} s horizon",
        ],
    }

    out = Path(args.out).expanduser()
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("w", encoding="utf-8") as handle:
        yaml.safe_dump(doc, handle, sort_keys=False)

    print(f"wrote {out}")
    print(f"  joints            : {limits.n_joints}")
    print(f"  velocity range    : [{limits.velocity.min():.3f}, {limits.velocity.max():.3f}] rad/s")
    print(f"  horizon           : {contract.future_horizon_s:g} s "
          f"({contract.window_frames} x {contract.window_dt:g} s)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
