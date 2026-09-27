"""Network configuration for the bridge (ports live here, not in code)."""

from __future__ import annotations

import sys
from pathlib import Path

import yaml

BRIDGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(BRIDGE_ROOT / "src"))

from a3_teleop_bridge.contract import load_contract  # noqa: E402

OUT = BRIDGE_ROOT / "configs" / "network.yaml"

# The SONIC PICO streamer publishes SMPL frames on its own ZMQ port; keep the
# bridge reference port separate so both can run side by side.
REFERENCE_PORT = 5560
PICO_PORT = 5556

doc = {
    "schema": "a3_network/v1",
    "protocol": "A3_REFERENCE_V1",
    "reference": {
        "bind_host": "0.0.0.0",
        "connect_host": "127.0.0.1",
        "port": REFERENCE_PORT,
        "comment": "publisher binds, MuJoCo/A3 runtime connects; use the LAN IP for Orin -> A3",
    },
    "pico": {
        "bind_host": "0.0.0.0",
        "connect_host": "127.0.0.1",
        "port": PICO_PORT,
        "comment": "upstream SONIC PICO streamer (configs/teleop.yaml holds its topic contract)",
    },
    "zmq": {
        "high_water_mark": 1,
        "conflate": True,
        "recv_timeout_ms": 50,
        "comment": "latest-only: never process a backlog of motion frames",
    },
    "frequencies_hz": {
        "pico": "as delivered (~60-90)",
        "umr": "25-50 (target)",
        "predictor": 50,
        "reference_publish": 50,
        "a3_policy": 50,
    },
    "watchdog": {
        "hold_after_ms": 50.0,
        "invalid_after_ms": 250.0,
        "comment": "aligned with the official runtime frame-age watchdog (50 ms)",
    },
    "reference_window": {
        "frames": load_contract().window_frames,
        "dt": load_contract().window_dt,
        "horizon_s": load_contract().future_horizon_s,
    },
}


def main() -> int:
    OUT.parent.mkdir(parents=True, exist_ok=True)
    with OUT.open("w", encoding="utf-8") as handle:
        yaml.safe_dump(doc, handle, sort_keys=False)
    print(f"wrote {OUT}")
    print(f"  reference port : {REFERENCE_PORT}")
    print(f"  pico port      : {PICO_PORT}")
    print(f"  window         : {doc['reference_window']['frames']} x "
          f"{doc['reference_window']['dt']} s = {doc['reference_window']['horizon_s']} s")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
