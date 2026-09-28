#!/usr/bin/env python3
"""Sample the live PICO body tracking and report which joints actually move.

"the legs do not move at all" has two very different causes and this tells them
apart in one minute:

  * the headset is not tracking the lower body (the joints stay frozen no matter
    what the operator does) -- a headset/PC-Service problem, nothing downstream
    can fix it;
  * the headset is tracking fine and the retarget is not following -- a bridge
    problem.

It prints, per joint, the position range and the total orientation change over
the sampling window.  Move the part you care about while it runs.

Usage:
    # while the headset is streaming:
    python tools/probe_pico_body.py --seconds 15
    python tools/probe_pico_body.py --seconds 15 --expect legs   # focus the report
"""

from __future__ import annotations

import argparse
import os
import sys
import time

import numpy as np

#: the XR body layout the sender consumes: 24 poses of [x, y, z, qx, qy, qz, qw]
XR_JOINT_NAMES = [
    "Hips", "Spine", "Spine1", "Spine2", "Spine3", "Neck", "Head",
    "LeftShoulder", "LeftArm", "LeftForeArm", "LeftHand",
    "RightShoulder", "RightArm", "RightForeArm", "RightHand",
    "LeftUpLeg", "LeftLeg", "LeftFoot", "LeftToe",
    "RightUpLeg", "RightLeg", "RightFoot", "RightToe", "Root",
]
LEG_IDS = (15, 16, 17, 18, 19, 20, 21, 22)
ARM_IDS = (7, 8, 9, 10, 11, 12, 13, 14)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=float, default=15.0)
    parser.add_argument("--rate", type=float, default=20.0, help="samples per second")
    parser.add_argument("--expect", choices=("legs", "arms", "all"), default="all")
    args = parser.parse_args(argv)

    try:
        import xrobotoolkit_sdk as xrt
    except Exception as exc:  # pragma: no cover - environment dependent
        print(f"SDK_MISSING {exc}")
        return 2
    xrt.init()

    deadline = time.time() + 10.0
    while time.time() < deadline and not xrt.is_body_data_available():
        time.sleep(0.2)
    if not xrt.is_body_data_available():
        print("NO_BODY_DATA -- the headset is not streaming (see run_pico_sim.sh --check)")
        sys.stdout.flush()
        os._exit(3)

    focus = {"legs": LEG_IDS, "arms": ARM_IDS, "all": tuple(range(len(XR_JOINT_NAMES)))}[args.expect]
    label = {
        "legs": ">>> MOVE YOUR LEGS (lift a knee, take a step) <<<",
        "arms": ">>> MOVE YOUR ARMS <<<",
        "all": ">>> MOVE EVERYTHING (arms, legs, torso) <<<",
    }[args.expect]
    print(f"sampling {args.seconds:g}s at {args.rate:g} Hz -- {label}")
    sys.stdout.flush()

    positions, quats = [], []
    period = 1.0 / max(args.rate, 1e-6)
    t0 = time.time()
    while time.time() - t0 < args.seconds:
        pose = np.asarray(xrt.get_body_joints_pose(), dtype=np.float64)
        if pose.ndim >= 2 and pose.shape[0] >= 24:
            positions.append(pose[:24, :3].copy())
            quats.append(pose[:24, 3:7].copy())
        nxt = t0 + (len(positions) + 1) * period
        if nxt > time.time():
            time.sleep(nxt - time.time())

    if len(positions) < 5:
        print(f"TOO_FEW_SAMPLES ({len(positions)})")
        os._exit(3)

    pos = np.stack(positions)          # (T, 24, 3)
    quat = np.stack(quats)             # (T, 24, 4)
    ranges = np.linalg.norm(pos.max(axis=0) - pos.min(axis=0), axis=1)
    # orientation change: angle between the first and the most distant sample
    ref = quat[0] / np.maximum(np.linalg.norm(quat[0], axis=1, keepdims=True), 1e-12)
    dots = np.abs(np.einsum("tjc,jc->tj", quat / np.maximum(
        np.linalg.norm(quat, axis=2, keepdims=True), 1e-12), ref))
    angles = np.degrees(2.0 * np.arccos(np.clip(dots.min(axis=0), -1.0, 1.0)))

    print(f"\n{'joint':16s} {'pos range (m)':>14s} {'orient change (deg)':>20s}")
    for i, name in enumerate(XR_JOINT_NAMES):
        mark = "  <-- focused" if i in focus else ""
        print(f"{name:16s} {ranges[i]:14.4f} {angles[i]:20.2f}{mark}")

    moving = [i for i in focus if ranges[i] > 0.02 or angles[i] > 5.0]
    print()
    if not moving:
        print(f"VERDICT: NO_MOTION -- none of the focused joints moved. The headset is not "
              f"tracking them; this is a headset/PC-Service problem, not the bridge.")
        os._exit(4)
    print(f"VERDICT: TRACKING -- {len(moving)}/{len(focus)} focused joints moved "
          f"(pos>{0.02} m or orient>{5.0} deg). The headset is fine; look downstream.")
    sys.stdout.flush()
    os._exit(0)


if __name__ == "__main__":
    sys.exit(main())
