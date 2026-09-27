"""Core data type behaviour (plan section 9)."""

from __future__ import annotations

import numpy as np
import pytest

from a3_teleop_bridge.clocks import LatencyStats, RateMeter, SeqTracker, StaleWatchdog
from a3_teleop_bridge.types import (
    A3CanonicalState,
    A3ReferenceWindow,
    BridgeState,
    HumanSmplFrame,
)


def make_smpl_frame(seq: int = 0) -> HumanSmplFrame:
    return HumanSmplFrame(
        seq=seq,
        timestamp_ns=1_000_000 * seq,
        smpl_joints=np.zeros((24, 3)),
        smpl_pose=np.zeros((21, 3)),
        root_translation=np.array([0.0, 0.0, 0.9]),
        root_quat_wxyz=np.array([1.0, 0.0, 0.0, 0.0]),
    )


def make_state(seq: int = 0, n: int = 29) -> A3CanonicalState:
    return A3CanonicalState(
        seq=seq,
        timestamp_ns=1_000_000 * seq,
        root_pos_m=np.array([0.1, 0.0, 0.7]),
        root_quat_wxyz=np.array([1.0, 0.0, 0.0, 0.0]),
        joint_pos_rad=np.linspace(-0.2, 0.2, n),
        joint_vel_rad_s=np.full(n, 0.5),
    )


def test_smpl_frame_shapes():
    frame = make_smpl_frame().validate()
    assert frame.smpl_joints.shape == (24, 3)
    assert frame.smpl_pose.shape == (21, 3)
    assert frame.body_quat_w is None
    assert frame.quat_norm() == pytest.approx(1.0)


def test_smpl_frame_rejects_bad_shape():
    frame = make_smpl_frame()
    frame.smpl_joints = np.zeros((23, 3))
    with pytest.raises(ValueError):
        frame.validate()


def test_smpl_frame_rejects_nan():
    frame = make_smpl_frame()
    frame.smpl_joints[0, 0] = np.nan
    with pytest.raises(ValueError):
        frame.validate()


def test_canonical_state_shapes():
    state = make_state().validate()
    assert state.joint_pos_rad.shape == (29,)
    assert state.is_finite
    assert state.valid


def test_canonical_state_invalid_factory():
    state = A3CanonicalState.invalid(seq=3, timestamp_ns=42, reason="solver fail")
    assert not state.valid
    assert state.reason == "solver fail"
    assert state.joint_vel_rad_s.shape == (29,)
    assert state.is_finite  # zeros are finite; validity is a separate flag


def test_canonical_state_detects_nan():
    state = make_state()
    state.joint_vel_rad_s[5] = np.inf
    assert not state.is_finite


def test_reference_window_hold_freezes_state():
    state = make_state()
    window = A3ReferenceWindow.hold(state, seq=1, timestamp_ns=10, dt=0.02)
    assert window.frames == 10
    assert window.n_joints == 29
    assert window.state is BridgeState.HOLD
    assert not window.valid
    np.testing.assert_allclose(window.joint_pos_rad[0], state.joint_pos_rad)
    np.testing.assert_allclose(window.joint_pos_rad[-1], state.joint_pos_rad)
    np.testing.assert_allclose(window.joint_vel_rad_s, 0.0)
    np.testing.assert_allclose(window.root_quat_wxyz[5], state.root_quat_wxyz)
    assert window.horizon_s == pytest.approx(0.18)


def test_reference_window_validate_shapes():
    state = make_state()
    window = A3ReferenceWindow.hold(state, seq=0, timestamp_ns=0, dt=0.02)
    window.validate()
    bad = A3ReferenceWindow.hold(state, seq=0, timestamp_ns=0, dt=0.02, frames=5)
    with pytest.raises(ValueError):
        bad.validate()


def test_bridge_state_values():
    assert [s.value for s in BridgeState] == [
        "DISCONNECTED",
        "CALIBRATION",
        "READY",
        "TRACKING",
        "HOLD",
        "SAFE_STOP",
    ]


def test_seq_tracker_counts_losses():
    tracker = SeqTracker()
    for seq in (0, 1, 2, 2, 5, 4):
        tracker.update(seq)
    stats = tracker.stats()
    assert stats["received"] == 6
    assert stats["duplicates"] == 1
    assert stats["dropped"] == 2  # seq 3 and 4 (4 arrives late)
    assert stats["reordered"] == 1


def test_latency_stats_percentiles():
    stats = LatencyStats("umr")
    stats.extend([1.0, 2.0, 3.0, 4.0, 100.0])
    summary = stats.summary()
    assert summary["count"] == 5
    assert summary["max"] == pytest.approx(100.0)
    assert summary["p50"] == pytest.approx(3.0)
    assert summary["p50"] <= summary["p90"] <= summary["p95"] <= summary["p99"] <= summary["max"]


def test_latency_stats_empty_is_nan():
    summary = LatencyStats().summary()
    assert summary["count"] == 0
    assert np.isnan(summary["p50"])


def test_rate_meter():
    meter = RateMeter(window=10)
    for i in range(11):
        meter.tick(i * 20_000_000)  # 20 ms apart -> 50 Hz
    assert meter.hz == pytest.approx(50.0, rel=1e-6)
    assert meter.period_ms_mean == pytest.approx(20.0)


def test_stale_watchdog_thresholds():
    dog = StaleWatchdog(hold_after_ms=50.0, invalid_after_ms=250.0)
    assert dog.classify(10.0) == "OK"
    assert dog.classify(60.0) == "HOLD"
    assert dog.classify(300.0) == "INVALID"
    assert dog.classify(float("nan")) == "INVALID"
