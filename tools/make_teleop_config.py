#!/usr/bin/env python3
"""Generate ``configs/teleop.yaml`` (PICO topic/port contract, plan section 23)."""

from __future__ import annotations

import sys
from pathlib import Path

import yaml

BRIDGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(BRIDGE_ROOT / "src"))

from a3_teleop_bridge.contract import load_contract  # noqa: E402

OUT = BRIDGE_ROOT / "configs" / "teleop.yaml"

doc = {
    "schema": "a3_teleop/v1",
    "pico": {
        "topic": "pose",
        "connect_host": "127.0.0.1",
        "port": 5556,
        "source": "gear_sonic/scripts/pico_pose_zmq_minimal.py",
        "format": "topic + 1280-byte JSON header + concatenated binary fields",
        "version": 3,
        "required_fields": ["smpl_pose[*,21,3]", "smpl_joints[*,24,3]"],
        "optional_fields": ["body_quat_w[*,4]", "root_translation[*,3]", "timestamp_realtime[1]"],
        "ignored_fields": {
            "joint_pos": "G1-flavoured wrist estimate, NOT A3 joints",
            "joint_vel": "same",
        },
    },
    "zmq": {"high_water_mark": 1, "conflate": True, "recv_timeout_ms": 50},
    "states": ["DISCONNECTED", "CALIBRATION", "READY", "TRACKING", "HOLD", "SAFE_STOP"],
    "recordings_dir": "recordings",
    "calibration": {
        "stand_still_s": 3.0,
        "height_scale_source": "A3 pelvis height from the robot MJCF at the default pose",
        "save": "calibration.json",
    },
    "reference_window": {
        "frames": load_contract().window_frames,
        "dt": load_contract().window_dt,
    },
}


def main() -> int:
    OUT.parent.mkdir(parents=True, exist_ok=True)
    with OUT.open("w", encoding="utf-8") as handle:
        yaml.safe_dump(doc, handle, sort_keys=False)
    print(f"wrote {OUT}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
