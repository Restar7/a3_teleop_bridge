"""Online pipeline architecture: latest-only queues, state machine, backends."""

from __future__ import annotations

import threading
import time
from pathlib import Path

import numpy as np
import pytest

from a3_teleop_bridge.a3.limits import load_limits
from a3_teleop_bridge.a3.predictor import A3ReferencePredictor
from a3_teleop_bridge.clocks import now_ns
from a3_teleop_bridge.contract import load_contract
from a3_teleop_bridge.pico.calibration import CalibrationError, SessionCalibration
from a3_teleop_bridge.types import A3CanonicalState, BridgeState, HumanSmplFrame
from a3_teleop_bridge.umr.backends import TrajectoryReplayBackend
from a3_teleop_bridge.umr.online import LatestSlot, OnlineTeleopPipeline, StageStats

BRIDGE_ROOT = Path(__file__).resolve().parents[1]
WS = BRIDGE_ROOT.parent
A3_CSV = WS / "logs" / "a3_validation" / "stand" / "stand.csv"


# --------------------------------------------------------------------------
# solver throughput
# --------------------------------------------------------------------------
def test_online_session_caps_torch_threads(monkeypatch):
    """The online SMPL-X forward must not be split across every core.

    Torch's default thread count made the retarget several times slower than a
    single thread (measured p50 120.9 ms/frame at 14 threads vs 31.1 ms at 1),
    which is what pinned the live pipeline near 20 Hz.  ``initialize`` applies
    the cap, and ``A3_TORCH_THREADS`` is the documented escape hatch.
    """
    torch = pytest.importorskip("torch")
    from a3_teleop_bridge.umr.umr_session import (
        DEFAULT_TORCH_THREADS,
        configure_torch_threads,
    )

    assert DEFAULT_TORCH_THREADS == 1

    original = torch.get_num_threads()
    try:
        monkeypatch.delenv("A3_TORCH_THREADS", raising=False)
        assert configure_torch_threads() == 1

        assert configure_torch_threads(3) == 3

        monkeypatch.setenv("A3_TORCH_THREADS", "2")
        assert configure_torch_threads() == 2
        assert torch.get_num_threads() == 2

        # "off" means "leave torch's own default alone", not "use zero threads"
        monkeypatch.setenv("A3_TORCH_THREADS", "off")
        assert configure_torch_threads() == torch.get_num_threads()
        assert torch.get_num_threads() > 0
    finally:
        torch.set_num_threads(original)


def test_run_pico_sim_documents_the_thread_knob():
    """The knob has to be discoverable from the runbook, not only the source."""
    runbook = (BRIDGE_ROOT / "docs" / "5060_FULL_RUNBOOK.md").read_text(encoding="utf-8")
    assert "A3_TORCH_THREADS" in runbook


# --------------------------------------------------------------------------
# latest-only mailbox
# --------------------------------------------------------------------------
def test_latest_slot_keeps_only_the_newest():
    slot = LatestSlot()
    for i in range(100):
        slot.put(i)
    assert slot.dropped == 99
    assert slot.peek() == 99
    assert slot.get(timeout=0.01) == 99
    assert slot.get(timeout=0.01) is None


def test_latest_slot_does_not_block_producers():
    slot = LatestSlot()
    started = time.perf_counter()
    for i in range(10_000):
        slot.put(i)
    assert time.perf_counter() - started < 1.0
    assert slot.dropped == 9_999


def test_latest_slot_wakes_a_waiter():
    slot = LatestSlot()
    result = {}

    def waiter():
        result["value"] = slot.get(timeout=1.0)

    thread = threading.Thread(target=waiter)
    thread.start()
    time.sleep(0.05)
    slot.put("frame")
    thread.join(timeout=1.0)
    assert result.get("value") == "frame"


# --------------------------------------------------------------------------
# calibration
# --------------------------------------------------------------------------
def make_frames(n=30, yaw=0.0, height=1.7, swap_lr=False):
    from a3_teleop_bridge.a3.predictor import quat_exp_map

    frames = []
    for i in range(n):
        joints = np.zeros((24, 3))
        joints[0] = [0.0, 0.0, 0.0]           # pelvis
        joints[1] = [0.10, -0.9, 0.0] if not swap_lr else [-0.10, -0.9, 0.0]
        joints[2] = [-0.10, -0.9, 0.0] if not swap_lr else [0.10, -0.9, 0.0]
        joints[16] = [0.18, 0.3, 0.0]
        joints[17] = [-0.18, 0.3, 0.0]
        joints[15] = [0.0, height - 0.9, 0.0]  # head sets the body height
        quat = quat_exp_map(np.array([0.0, 0.0, yaw]))
        frames.append(
            HumanSmplFrame(
                seq=i,
                timestamp_ns=int(i * 20e6),
                smpl_joints=joints,
                smpl_pose=np.zeros((21, 3)),
                root_translation=np.zeros(3),
                root_quat_wxyz=quat,
                body_quat_w=quat,
            )
        )
    return frames


def test_calibration_roundtrip(tmp_path):
    calibration = SessionCalibration.from_frames(make_frames(yaw=0.3), robot_height_m=1.07)
    assert calibration.left_right_ok is True
    assert calibration.frames_used == 30
    path = calibration.save(tmp_path / "calibration.json")
    loaded = SessionCalibration.load(path)
    assert loaded.body_scale == pytest.approx(calibration.body_scale)
    assert loaded.root_yaw_offset_rad == pytest.approx(calibration.root_yaw_offset_rad)


def test_calibration_detects_swapped_sides():
    calibration = SessionCalibration.from_frames(make_frames(swap_lr=True), robot_height_m=1.07)
    assert calibration.left_right_ok is False


def test_calibration_needs_enough_frames():
    with pytest.raises(CalibrationError):
        SessionCalibration.from_frames(make_frames(n=3))


def test_calibration_applies_heading_offset():
    calibration = SessionCalibration(
        root_yaw_offset_rad=0.5, pelvis_height_offset_m=0.0, robot_height_m=1.0
    )
    translation, quat = calibration.apply_root(np.zeros(3), np.array([1.0, 0.0, 0.0, 0.0]))
    assert np.linalg.norm(quat) == pytest.approx(1.0)
    # a -0.5 rad yaw about z: w = cos(0.25)
    assert quat[0] == pytest.approx(np.cos(0.25), abs=1e-9)
    assert quat[3] == pytest.approx(-np.sin(0.25), abs=1e-9)


# --------------------------------------------------------------------------
# pipeline
# --------------------------------------------------------------------------
class CountingBackend:
    """Minimal backend: returns the default standing pose every step."""

    def __init__(self, delay_s: float = 0.0) -> None:
        self.limits = load_limits()
        self.calls = 0
        self.delay_s = delay_s

    def reset(self, initial_frame=None) -> None:
        self.calls = 0

    def step(self, frame, timestamp_ns):
        if self.delay_s:
            time.sleep(self.delay_s)
        self.calls += 1
        return A3CanonicalState(
            seq=self.calls,
            timestamp_ns=timestamp_ns,
            root_pos_m=np.array([0.0, 0.0, 1.07]),
            root_quat_wxyz=np.array([1.0, 0.0, 0.0, 0.0]),
            joint_pos_rad=self.limits.default_angle,
            joint_vel_rad_s=np.zeros(29),
            valid=True,
        )


class FailingBackend(CountingBackend):
    def step(self, frame, timestamp_ns):
        raise RuntimeError("simulated solver failure")


def test_pipeline_runs_and_counts_frames():
    backend = CountingBackend()
    pipeline = OnlineTeleopPipeline(backend, A3ReferencePredictor())
    frames = make_frames()
    index = {"i": 0}

    def producer():
        if index["i"] >= len(frames):
            return None
        frame = frames[index["i"]]
        index["i"] += 1
        frame.timestamp_ns = now_ns()
        return frame

    pipeline.start()
    try:
        pipeline.run_for(producer, duration_s=2.0)
        time.sleep(0.3)
    finally:
        pipeline.stop()

    stats = pipeline.stats
    assert stats.frames_in == len(frames)
    assert stats.frames_solved >= 1
    assert stats.rejected == 0
    summary = stats.as_dict()
    assert summary["solver_latency_ms"]["count"] >= 1
    assert summary["end_to_end_ms"]["count"] >= 1


def test_pipeline_drops_frames_instead_of_queueing():
    """A slow solver must drop frames, never build a backlog (plan section 31)."""
    backend = CountingBackend(delay_s=0.03)  # ~33 Hz solver
    pipeline = OnlineTeleopPipeline(backend, A3ReferencePredictor())
    frames = make_frames(n=200)
    index = {"i": 0}

    def producer():
        if index["i"] >= len(frames):
            return None
        frame = frames[index["i"]]
        index["i"] += 1
        frame.timestamp_ns = now_ns()
        return frame

    pipeline.start()
    try:
        pipeline.run_for(producer, duration_s=0.6)  # 200 frames pushed fast
        time.sleep(0.4)
    finally:
        pipeline.stop()

    assert pipeline.stats.frames_in == 200
    assert pipeline.human_slot.dropped > 50, pipeline.stats.as_dict()
    assert pipeline.stats.frames_solved < 200


def test_pipeline_survives_solver_failure():
    pipeline = OnlineTeleopPipeline(FailingBackend(), A3ReferencePredictor())
    frames = make_frames(n=5)
    index = {"i": 0}

    def producer():
        if index["i"] >= len(frames):
            return None
        frame = frames[index["i"]]
        index["i"] += 1
        frame.timestamp_ns = now_ns()
        return frame

    pipeline.start()
    try:
        pipeline.run_for(producer, duration_s=0.8)
        time.sleep(0.3)
    finally:
        pipeline.stop()
    assert pipeline.stats.rejected >= 1
    assert pipeline.stats.frames_solved >= 1  # the loop kept running


def test_pipeline_state_machine_transitions():
    """DISCONNECTED -> CALIBRATION -> TRACKING -> HOLD -> SAFE_STOP."""
    pipeline = OnlineTeleopPipeline(CountingBackend(), A3ReferencePredictor())
    pipeline._maybe_transition()
    assert pipeline.state is BridgeState.DISCONNECTED

    pipeline.submit(make_frames(n=1)[0])
    pipeline._maybe_transition()
    assert pipeline.state is BridgeState.CALIBRATION

    pipeline.calibration = SessionCalibration()
    pipeline.last_frame_timestamp_ns = now_ns()
    pipeline._maybe_transition()
    assert pipeline.state is BridgeState.TRACKING

    pipeline.last_frame_timestamp_ns = now_ns() - int(100e6)
    pipeline._maybe_transition()
    assert pipeline.state is BridgeState.HOLD

    pipeline.last_frame_timestamp_ns = now_ns() - int(1000e6)
    pipeline._maybe_transition()
    assert pipeline.state is BridgeState.SAFE_STOP

    values = [name for name, _ in pipeline.state_history]
    assert values == ["DISCONNECTED", "CALIBRATION", "TRACKING", "HOLD", "SAFE_STOP"]


def test_pipeline_publishes_reference_windows():
    contract = load_contract()
    import zmq

    from a3_teleop_bridge.transport.publisher import (
        NetworkConfig,
        ReferencePublisher,
        ReferenceSubscriber,
    )

    context = zmq.Context()
    config = NetworkConfig.from_yaml()
    port = 15620
    publisher = ReferencePublisher(config, contract, context=context, bind=f"tcp://127.0.0.1:{port}")
    subscriber = ReferenceSubscriber(config, contract, context=context, connect=f"tcp://127.0.0.1:{port}")
    pipeline = OnlineTeleopPipeline(CountingBackend(), A3ReferencePredictor(), publisher)
    frames = make_frames(n=40)
    index = {"i": 0}

    def producer():
        if index["i"] >= len(frames):
            return None
        frame = frames[index["i"]]
        index["i"] += 1
        frame.timestamp_ns = now_ns()
        return frame

    try:
        pipeline.start()
        pipeline.run_for(producer, duration_s=1.0)
        received = None
        for _ in range(200):
            received = subscriber.poll(timeout_ms=20)
            if received is not None and not received.rejected:
                break
        assert received is not None and not received.rejected
        assert received.window.frames == contract.window_frames
        assert received.window.is_finite
        assert pipeline.stats.frames_published >= 1
    finally:
        pipeline.stop()
        subscriber.close()
        publisher.close()
        context.term()


# --------------------------------------------------------------------------
# replay backend
# --------------------------------------------------------------------------
@pytest.mark.skipif(not A3_CSV.is_file(), reason="no A3 CSV available")
def test_trajectory_replay_backend_advances():
    backend = TrajectoryReplayBackend(csv_path=A3_CSV, csv_fps=30.0)
    assert backend.frames > 0
    backend.reset()
    frame = make_frames(n=1)[0]
    first = backend.step(frame, now_ns())
    second = backend.step(frame, now_ns())
    assert first.valid and second.valid
    assert first.seq == 1 and second.seq == 2
    assert first.timestamp_ns != second.timestamp_ns
