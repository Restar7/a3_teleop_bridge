"""Live teleop driver: PICO (or a recording) -> A3 reference -> publisher.

Plan sections 31/32/44.  The heavy chain is split across threads with
latest-only hand-offs so no stage can build a backlog:

    receiver thread  : PICO ZMQ / recorded replay  -> human_slot (maxsize 1)
    solver thread    : backend.step(frame)         -> state_slot (maxsize 1)
    predictor thread : predictor.window()          -> reference publisher

Usage::

    # hardware-free: drive the live plumbing from a recorded trajectory
    python -m a3_teleop_bridge.apps.retarget_live \
        --source trajectory --csv ~/a3_teleop_ws/logs/a3_validation/stand/stand.csv

    # real PICO stream (needs the SONIC streamer running)
    python -m a3_teleop_bridge.apps.retarget_live --source pico --duration 60

    # online UMR (needs the UMR venv; see docs/architecture.md)
    python -m a3_teleop_bridge.apps.retarget_live --source pico --backend umr-online
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

from ..a3.predictor import A3ReferencePredictor
from ..clocks import now_ns
from ..pico.calibration import CalibrationError, SessionCalibration
from ..pico.recorder import PicoRecording
from ..pico.zmq_subscriber import PicoPoseSubscriber, TeleopConfig
from ..transport.publisher import NetworkConfig, ReferencePublisher
from ..types import HumanSmplFrame
from ..umr.backends import TrajectoryReplayBackend, UmrOnlineBackend
from ..umr.online import OnlineTeleopPipeline


def make_frame_provider(args, recording=None):
    """Return a callable producing the next :class:`HumanSmplFrame` (or None)."""
    if args.source in ("trajectory", "recording"):
        if recording is None:
            raise SystemExit(f"--source {args.source} needs --csv or --recording")
        state = {"i": 0, "next_time": time.perf_counter()}
        frames = recording.frames
        # replay at the recorded rate: an unthrottled producer would starve the
        # solver thread with millions of frames/s and make freshness meaningless
        period = 1.0 / max(args.playback_hz, 1e-6)

        def provider():
            now = time.perf_counter()
            if now < state["next_time"]:
                time.sleep(min(0.005, state["next_time"] - now))
                return None
            state["next_time"] += period
            if now - state["next_time"] > 0.25:  # fell behind: resynchronise
                state["next_time"] = now + period
            if state["i"] >= len(frames):
                if not args.loop:
                    return None
                state["i"] = 0
            frame = frames[state["i"]]
            state["i"] += 1
            # re-stamp on the live clock so the pipeline sees a live stream
            return HumanSmplFrame(
                seq=frame.seq,
                timestamp_ns=now_ns(),
                smpl_joints=frame.smpl_joints,
                smpl_pose=frame.smpl_pose,
                root_translation=frame.root_translation,
                root_quat_wxyz=frame.root_quat_wxyz,
                body_quat_w=frame.body_quat_w,
                receive_timestamp_ns=now_ns(),
            )

        return provider, None

    config = TeleopConfig.from_yaml()
    subscriber = PicoPoseSubscriber(config)

    def provider():
        frame = subscriber.poll(timeout_ms=2)
        if frame is None or frame.rejected or frame.smpl_frame is None:
            return None
        return frame.smpl_frame

    return provider, subscriber


def build_backend(args):
    if args.backend == "umr-online":
        backend = UmrOnlineBackend(
            robot_config=args.robot_config,
            verbose=args.verbose,
            dump_frames=getattr(args, "dump_frames", None),
        )
        backend.initialize()
        return backend
    if args.umr_result:
        return TrajectoryReplayBackend(umr_result=args.umr_result)
    if args.csv:
        return TrajectoryReplayBackend(csv_path=args.csv, csv_fps=args.csv_fps)
    raise SystemExit("--backend offline needs --umr-result or --csv")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", choices=("pico", "trajectory", "recording"), default="pico")
    parser.add_argument("--backend", choices=("offline", "umr-online"), default="offline")
    parser.add_argument("--csv", default=None, help="A3 flat CSV (trajectory source/backend)")
    parser.add_argument("--csv-fps", type=float, default=30.0)
    parser.add_argument(
        "--recording",
        default=None,
        help="recorded PICO session directory; with --backend umr-online this is the "
        "hardware-free M7 path (everything but the headset)",
    )
    parser.add_argument(
        "--playback-hz",
        type=float,
        default=50.0,
        help="rate at which a recorded source is replayed (real PICO arrives at its own rate)",
    )
    parser.add_argument(
        "--auto-calibrate",
        action="store_true",
        default=True,
        help="derive a session calibration from the first frames when none is loaded",
    )
    parser.add_argument("--no-auto-calibrate", dest="auto_calibrate", action="store_false")
    parser.add_argument("--umr-result", default=None)
    parser.add_argument("--robot-config", default=None)
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--loop", action="store_true", default=True)
    parser.add_argument("--no-loop", dest="loop", action="store_false")
    parser.add_argument("--publish", action="store_true", default=True)
    parser.add_argument("--no-publish", dest="publish", action="store_false")
    parser.add_argument("--endpoint", default=None)
    parser.add_argument("--calibration", default=None, help="calibration.json to load")
    parser.add_argument("--save-calibration", default=None)
    parser.add_argument(
        "--dump-frames",
        default=None,
        help="write a JSONL of per-frame diagnostics (source leg angles + the "
        "reference joint values it produced); see tools/report_live_dump.py",
    )
    parser.add_argument("--stats", default=None, help="write pipeline stats JSON here")
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args(argv)

    recording = None
    if args.source == "trajectory":
        if args.csv:
            # a flat CSV is a robot trajectory, not a human recording: convert it
            # into a stand-in human stream so the plumbing can be exercised
            recording = _csv_to_human_stream(Path(args.csv).expanduser(), args.csv_fps)
        else:
            raise SystemExit("--source trajectory needs --csv (or use --source pico)")
    elif args.source == "recording":
        if not args.recording:
            raise SystemExit("--source recording needs --recording <dir>")
        # real recorded SMPL-X body frames, replayed on the live clock: this is
        # the source M7 uses, minus the headset
        recording = PicoRecording.load(Path(args.recording).expanduser())

    provider, subscriber = make_frame_provider(args, recording)
    backend = build_backend(args)
    predictor = A3ReferencePredictor()

    publisher = None
    if args.publish:
        network = NetworkConfig.from_yaml()
        publisher = ReferencePublisher(network, bind=args.endpoint) if args.endpoint else ReferencePublisher(network)

    pipeline = OnlineTeleopPipeline(backend, predictor, publisher)
    if args.calibration:
        pipeline.calibration = SessionCalibration.load(args.calibration)
        print(f"[live] loaded calibration {args.calibration}")

    # ---- automatic session calibration (plan section 74) -----------------
    # Both live-capable sources are covered: a recorded PICO session carries a
    # real body, and the CSV stand-in is rejected by the body_present check below
    # (previously only "trajectory" was wired, so an online-UMR run driven by a
    # recording could never leave the CALIBRATION state).
    if args.auto_calibrate and args.source in ("trajectory", "recording") and recording is not None:
        sample = recording.frames[: max(30, int(1.0 * args.playback_hz))]
        body_present = bool(
            sample and np.abs(np.asarray([f.smpl_joints for f in sample])).max() > 1e-3
        )
        if not body_present:
            print("[live] auto-calibration skipped: the stream carries no body (stand-in trajectory)")
            sample = []
        try:
            if not sample:
                raise CalibrationError("no body in the stream")
            pipeline.calibration = SessionCalibration.from_frames(sample, robot_height_m=1.07)
            print(f"[live] auto-calibrated: yaw_off={pipeline.calibration.root_yaw_offset_rad:+.3f} rad "
                  f"scale={pipeline.calibration.body_scale:.3f} "
                  f"left_right_ok={pipeline.calibration.left_right_ok}")
            if args.save_calibration:
                pipeline.calibration.save(args.save_calibration)
        except CalibrationError as exc:
            print(f"[live] auto-calibration skipped: {exc}")

    # ---- live PICO auto-calibration (plan section 74) --------------------
    # The block above only covers sources that already have a recording to
    # sample.  A live headset has none, and the pico source has no calibration
    # path at all -- so `pipeline.calibration` stayed None, the state machine sat
    # in CALIBRATION forever, and nothing could ever reach TRACKING (which the
    # real robot needs before it will follow a reference).  Collect the first
    # frames from the live stream instead and calibrate from those: the operator
    # stands still for the first second.
    if (
        args.auto_calibrate
        and args.source == "pico"
        and pipeline.calibration is None
        and provider is not None
    ):
        live_inner = provider
        pending = []
        state = {"need": max(10, int(1.5 * max(args.playback_hz, 1.0))), "tries": 0}

        def provider():  # type: ignore[misc]
            frame = live_inner()
            if frame is not None and pipeline.calibration is None:
                pending.append(frame)
                if len(pending) >= state["need"]:
                    try:
                        pipeline.calibration = SessionCalibration.from_frames(
                            list(pending), robot_height_m=1.07
                        )
                        print(
                            f"[live] auto-calibrated from {len(pending)} live frames: "
                            f"yaw_off={pipeline.calibration.root_yaw_offset_rad:+.3f} rad "
                            f"scale={pipeline.calibration.body_scale:.3f} "
                            f"left_right_ok={pipeline.calibration.left_right_ok}"
                        )
                        if args.save_calibration:
                            pipeline.calibration.save(args.save_calibration)
                            print(f"[live] saved calibration {args.save_calibration}")
                    except CalibrationError as exc:
                        state["tries"] += 1
                        pending.clear()
                        if state["tries"] >= 3:
                            print(f"[live] auto-calibration gave up after 3 tries: {exc}")
                            state["need"] = 1 << 30  # stop retrying
                        else:
                            print(f"[live] auto-calibration retry {state['tries']}: {exc}")
            return frame

    print(f"[live] source={args.source} backend={args.backend} "
          f"duration={'unlimited' if float(args.duration) <= 0.0 else f'{args.duration:g}s'} "
          f"playback={args.playback_hz:g} Hz publish={bool(publisher)}")
    def write_stats() -> None:
        if not args.stats:
            return
        out = Path(args.stats).expanduser()
        out.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "frames_in": pipeline.stats.frames_in,
            "frames_solved": pipeline.stats.frames_solved,
            "frames_published": pipeline.stats.frames_published,
            "rejected": pipeline.stats.rejected,
            "state_history": [name for name, _ in pipeline.state_history],
            "wall_s": time.time() - started,
        }
        out.write_text(json.dumps(payload, indent=2, default=float) + "\n", encoding="utf-8")

    def run_with_progress() -> None:
        """``run_for`` plus a periodic stats flush.

        The MuJoCo side starts by *reading this file* to know when the bridge is
        really publishing, instead of guessing a fixed delay -- a guess that made
        the simulator wait 30 s for packets that were never coming.
        """
        period = 1.0
        # A non-positive --duration means "no deadline": keep publishing until the
        # process is interrupted.  An interactive session should not end on a timer
        # while the operator is still wearing the headset.
        _duration = float(args.duration)
        deadline = float("inf") if _duration <= 0.0 else time.perf_counter() + _duration
        last = 0.0
        while time.perf_counter() < deadline and not pipeline._stop.is_set():
            frame = provider()
            if frame is None:
                time.sleep(0.001)
            else:
                pipeline.submit(frame)
            now = time.time()
            if now - last >= period:
                write_stats()
                last = now

    started = time.time()
    try:
        pipeline.start()
        run_with_progress()
        time.sleep(0.3)
    except KeyboardInterrupt:
        print("\n[live] interrupted")
    finally:
        pipeline.stop()
        _backend = getattr(pipeline, "backend", None)
        _session = getattr(_backend, "session", None)
        if _session is not None and hasattr(_session, "close"):
            _session.close()
        if subscriber is not None:
            subscriber.close()
        if publisher is not None:
            publisher.close()

    stats = pipeline.stats.as_dict()
    stats["state_history"] = [name for name, _ in pipeline.state_history]
    stats["wall_s"] = time.time() - started
    stats["queue_dropped"] = {
        "human": pipeline.human_slot.dropped,
        "state": pipeline.state_slot.dropped,
    }
    print(f"[live] frames in={stats['frames_in']} solved={stats['frames_solved']} "
          f"published={stats['frames_published']} rejected={stats['rejected']}")
    print(f"[live] solver {stats['solver_latency_ms']['p50']:.2f} ms p95 "
          f"{stats['solver_latency_ms']['p95']:.2f} ms; e2e p95 "
          f"{stats['end_to_end_ms']['p95']:.2f} ms")
    print(f"[live] states {stats['state_history']} dropped {stats['queue_dropped']}")
    if args.stats:
        out = Path(args.stats).expanduser()
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps(stats, indent=2, default=float) + "\n", encoding="utf-8")
        print(f"[live] wrote {out}")
    return 0 if stats["frames_solved"] > 0 else 1


def _csv_to_human_stream(csv_path: Path, fps: float) -> PicoRecording:
    """Wrap a robot trajectory as a stand-in human stream (plumbing test only)."""
    from ..a3.csv_export import A3FlatCsvCodec

    codec = A3FlatCsvCodec()
    data = codec.read(csv_path, source_fps=fps, frame_stride=1)
    recording = PicoRecording(source=f"{csv_path} (robot trajectory stand-in)")
    for i in range(data.n_frames):
        recording.frames.append(
            HumanSmplFrame(
                seq=i,
                timestamp_ns=int(round(i / data.effective_fps * 1e9)),
                smpl_joints=np.zeros((24, 3)),
                smpl_pose=np.zeros((21, 3)),
                root_translation=data.root_pos_m[i],
                root_quat_wxyz=data.root_quat_wxyz[i],
                receive_timestamp_ns=now_ns(),
            )
        )
    return recording


if __name__ == "__main__":
    sys.exit(main())
