"""Record a PICO session to ``recordings/<timestamp>/`` (plan section 24).

Usage::

    # live PICO stream
    python -m a3_teleop_bridge.apps.record_pico --duration 20

    # synthetic self-test (no headset required)
    python -m a3_teleop_bridge.apps.record_pico --synthetic --duration 5

The plan's 20 s script (stand / left arm / right arm / knees / left foot /
right foot / stand) is printed as an operator prompt while recording.
"""

from __future__ import annotations

import argparse
import sys
import time
from datetime import datetime
from pathlib import Path

import numpy as np

from ..clocks import now_ns
from ..pico.recorder import PicoRecorder
from ..pico.zmq_subscriber import PicoPoseSubscriber, TeleopConfig, wait_for_pico

# plan section 24: 20-second operator script
DEFAULT_SCRIPT = (
    (3.0, "stand still"),
    (3.0, "raise LEFT hand"),
    (3.0, "raise RIGHT hand"),
    (3.0, "bend knees"),
    (3.0, "lift LEFT foot"),
    (3.0, "lift RIGHT foot"),
    (2.0, "stand still"),
)


def synthetic_frames(duration_s: float, hz: float = 60.0):
    """Generate a scripted synthetic SMPL stream (no hardware needed)."""
    steps = int(duration_s * hz)
    for i in range(steps):
        t = i / hz
        pose = np.zeros((21, 3))
        joints = np.zeros((24, 3))
        phase = t % 20.0
        if 3.0 <= phase < 6.0:  # left arm
            pose[15, 0] = -0.8
        elif 6.0 <= phase < 9.0:  # right arm
            pose[16, 0] = -0.8
        elif 9.0 <= phase < 12.0:  # knees
            pose[4, 0] = pose[5, 0] = 0.6
        elif 12.0 <= phase < 15.0:  # left foot
            pose[1, 0] = 0.4
        elif 15.0 <= phase < 18.0:  # right foot
            pose[2, 0] = 0.4
        joints[0] = [0.0, 0.0, 0.95 + 0.01 * np.sin(2 * np.pi * t)]
        yield pose.astype(np.float32), joints.astype(np.float32), t


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=20.0, help="seconds to record")
    parser.add_argument("--out", default=None, help="output directory (default recordings/<ts>)")
    parser.add_argument("--synthetic", action="store_true", help="no PICO hardware")
    parser.add_argument("--connect", default=None, help="override tcp endpoint")
    parser.add_argument("--wait-s", type=float, default=10.0, help="seconds to wait for the first frame")
    parser.add_argument("--quiet", action="store_true")
    args = parser.parse_args(argv)

    out_dir = (
        Path(args.out).expanduser()
        if args.out
        else Path(__file__).resolve().parents[3] / "recordings" / datetime.now().strftime("%Y%m%d_%H%M%S")
    )
    recorder = PicoRecorder(source="synthetic" if args.synthetic else "pico_zmq")

    if args.synthetic:
        from ..types import HumanSmplFrame

        seq = 0
        t0 = now_ns()
        for pose, joints, t in synthetic_frames(args.duration):
            frame = HumanSmplFrame(
                seq=seq,
                timestamp_ns=t0 + int(t * 1e9),
                smpl_joints=joints.astype(np.float64),
                smpl_pose=pose.astype(np.float64),
                root_translation=joints[0].astype(np.float64),
                root_quat_wxyz=np.array([1.0, 0.0, 0.0, 0.0]),
                receive_timestamp_ns=now_ns(),
            ).validate()
            recorder.recording.frames.append(frame)
            recorder.recording.started_ns = recorder.recording.started_ns or frame.receive_timestamp_ns
            recorder.recording.finished_ns = frame.receive_timestamp_ns
            seq += 1
        seq_stats = {"received": seq, "dropped": 0, "duplicates": 0, "reordered": 0}
    else:
        config = TeleopConfig.from_yaml()
        if args.connect:
            host, port = args.connect.rsplit(":", 1)
            config = TeleopConfig(
                topic=config.topic,
                connect_host=host.replace("tcp://", ""),
                port=int(port),
                recv_timeout_ms=config.recv_timeout_ms,
                conflate=config.conflate,
                high_water_mark=config.high_water_mark,
            )
        with PicoPoseSubscriber(config) as subscriber:
            print(f"[record] waiting for PICO on {subscriber.connect} (topic '{config.topic}') …")
            if not wait_for_pico(subscriber, timeout_s=args.wait_s):
                print("[record] no PICO frames received; aborting")
                return 1
            start = time.time()
            next_prompt = 0
            while time.time() - start < args.duration:
                frame = subscriber.poll(timeout_ms=50)
                recorder.add(frame)
                elapsed = time.time() - start
                while next_prompt < len(DEFAULT_SCRIPT) and elapsed >= sum(
                    d for d, _ in DEFAULT_SCRIPT[: next_prompt + 1]
                ):
                    label = DEFAULT_SCRIPT[next_prompt][1]
                    if not args.quiet:
                        print(f"[record] t={elapsed:5.1f}s  -> {label}")
                    next_prompt += 1
            seq_stats = subscriber.stats()

    recording = recorder.finalize(seq_stats)
    directory = recording.save(out_dir, source=recorder.recording.source)
    stats = recording.stats()

    print(f"[record] saved {len(recording)} frames to {directory}")
    print(f"[record] duration {stats.get('duration_s', 0):.2f} s, "
          f"mean rate {stats.get('source_hz_mean', 0):.1f} Hz")
    print(f"[record] rejected {stats.get('rejected', 0)}, finite={stats.get('all_finite')}")
    return 0 if len(recording) > 0 and stats.get("all_finite") else 1


if __name__ == "__main__":
    sys.exit(main())
