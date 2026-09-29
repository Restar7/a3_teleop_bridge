#!/usr/bin/env python3
"""Report whether the PICO has dedicated motion trackers, and how well the legs track.

Without waist/ankle motion trackers the headset *estimates* the lower body from
the headset and controllers.  That estimate does move -- a 100 s walk produced 70
leg-lift events -- but at roughly a third of a real walk's amplitude (knee
flexion p50 17 deg, p90 35 deg, against the 66-79 deg the validated A3 walk
references use), so the retargeted reference never asks the robot to step and it
cannot walk no matter how healthy the pipeline is.

Run this WITH THE HEADSET ON AND STREAMING, then march in place for ~15 s.

Usage:
    $A3WS/sonic_for_a3/.venv_pico_minimal/bin/python \
        tools/check_pico_trackers.py --seconds 15
"""

from __future__ import annotations

import argparse
import time

import numpy as np

JOINTS = ["LeftUpLeg", "LeftLeg", "LeftFoot", "RightUpLeg", "RightLeg", "RightFoot"]
#: A real walk flexes the knee to 66-79 deg (official 001_walk_front_slow /
#: 050_march_place_slow).  Anything much below this cannot drive a step.
WALK_GRADE_ORIENT_DEG = 40.0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=float, default=15.0)
    parser.add_argument("--rate", type=float, default=30.0)
    args = parser.parse_args(argv)

    import xrobotoolkit_sdk as xrt

    xrt.init()
    time.sleep(1.0)
    try:
        serials = list(xrt.get_motion_tracker_serial_numbers() or [])
        print(f"motion trackers paired : {len(serials)}  {serials}")
        if not serials:
            print("  -> 0 means the lower body is ESTIMATED, not measured.")
            print("     Expect leg amplitude well below a real walk (see the table below).")
        print(f"body data available    : {xrt.is_body_data_available()}")

        samples = []
        period = 1.0 / max(args.rate, 1e-6)
        end = time.perf_counter() + max(args.seconds, 0.1)
        while time.perf_counter() < end:
            if xrt.is_body_data_available():
                pose = np.asarray(xrt.get_body_joints_pose(), dtype=np.float64)
                if pose.ndim == 2 and pose.shape[0] >= 24:
                    samples.append(pose[: len(JOINTS)])
            time.sleep(period)

        if len(samples) < 10:
            print(f"TOO_FEW_SAMPLES ({len(samples)}) -- is the headset streaming?")
            return 3
        arr = np.stack(samples)  # (T, n, 7) = pos xyz + quat xyzw
        print(f"\nsampled {len(samples)} frames over {args.seconds:g}s -- march in place next time\n")
        print(f"{'joint':12s} {'pos range (m)':>14s} {'orient change (deg)':>20s}   verdict")
        weak = []
        for i, name in enumerate(JOINTS):
            pos = arr[:, i, :3]
            quat = arr[:, i, 3:7]
            span = float(np.linalg.norm(pos.max(axis=0) - pos.min(axis=0)))
            q = quat / np.maximum(np.linalg.norm(quat, axis=1, keepdims=True), 1e-12)
            dots = np.abs(q @ q[0])
            angle = float(np.rad2deg(2.0 * np.arccos(np.clip(dots.min(), -1.0, 1.0))))
            grade = "walk-grade" if angle >= WALK_GRADE_ORIENT_DEG else "too small for a step"
            if angle < WALK_GRADE_ORIENT_DEG:
                weak.append(name)
            print(f"{name:12s} {span:14.4f} {angle:20.2f}   {grade}")
        print()
        if weak:
            print(
                f"VERDICT: {len(weak)}/{len(JOINTS)} leg joints never reach "
                f"{WALK_GRADE_ORIENT_DEG:g} deg of rotation. The reference cannot ask for a "
                "step, so the robot will shuffle and drift instead of walking."
            )
            return 1
        print("VERDICT: leg amplitude is walk-grade; the input is not the limit.")
        return 0
    finally:
        try:
            xrt.close()
        except Exception:
            pass


if __name__ == "__main__":
    raise SystemExit(main())
