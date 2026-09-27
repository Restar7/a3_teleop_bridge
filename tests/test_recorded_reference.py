"""Recorded reference stream: file format + replay loader (plan sections 78/79)."""

from __future__ import annotations

import json

import numpy as np
import pytest

from a3_teleop_bridge.apps.replay_reference import load_recorded_windows
from a3_teleop_bridge.contract import load_contract


@pytest.fixture(scope="module")
def contract():
    return load_contract()


def _write_recording(directory, windows):
    directory.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        directory / "windows.npz",
        seq=np.asarray([w["seq"] for w in windows], dtype=np.int64),
        timestamp_ns=np.asarray([w["timestamp_ns"] for w in windows], dtype=np.int64),
        receive_timestamp_ns=np.asarray([w["timestamp_ns"] for w in windows], dtype=np.int64),
        dt=np.asarray([w["dt"] for w in windows], dtype=np.float32),
        root_pos_m=np.stack([w["root_pos_m"] for w in windows]),
        root_quat_wxyz=np.stack([w["root_quat_wxyz"] for w in windows]),
        joint_pos_rad=np.stack([w["joint_pos_rad"] for w in windows]),
        joint_vel_rad_s=np.stack([w["joint_vel_rad_s"] for w in windows]),
        source_age_ms=np.asarray([w["source_age_ms"] for w in windows]),
        valid=np.asarray([w["valid"] for w in windows], dtype=bool),
    )
    (directory / "metadata.json").write_text(json.dumps({"publish_hz": 50.0}), encoding="utf-8")


def test_recorded_windows_roundtrip(tmp_path, contract):
    """The loader must reproduce the stored payload field by field."""
    horizon = int(contract.window_frames)
    n = 5
    windows = [
        {
            "seq": 100 + i,
            "timestamp_ns": 1_000_000 * i,
            "dt": 0.02,
            "root_pos_m": np.full((horizon, 3), float(i), dtype=np.float64),
            "root_quat_wxyz": np.tile(np.array([1.0, 0.0, 0.0, 0.0]), (horizon, 1)),
            "joint_pos_rad": np.full((horizon, contract.n_policy_joints), 0.1 * i),
            "joint_vel_rad_s": np.full((horizon, contract.n_policy_joints), 0.01 * i),
            "source_age_ms": float(i),
            "valid": True,
        }
        for i in range(n)
    ]
    _write_recording(tmp_path / "rec", windows)

    loaded, metadata = load_recorded_windows(tmp_path / "rec")
    assert metadata["publish_hz"] == 50.0
    assert [w.seq for w in loaded] == [100 + i for i in range(n)]
    assert loaded[0].joint_pos_rad.shape == (horizon, contract.n_policy_joints)
    for stored, restored in zip(windows, loaded):
        assert restored.timestamp_ns == stored["timestamp_ns"]
        assert restored.dt == pytest.approx(stored["dt"])
        assert restored.valid is True
        np.testing.assert_allclose(restored.root_pos_m, stored["root_pos_m"])
        np.testing.assert_allclose(restored.joint_pos_rad, stored["joint_pos_rad"])
        np.testing.assert_allclose(restored.joint_vel_rad_s, stored["joint_vel_rad_s"])
        np.testing.assert_allclose(restored.root_quat_wxyz, stored["root_quat_wxyz"])


def test_replayed_sequences_stay_monotonic(tmp_path):
    """Replay must not renumber: a consumer's seq tracker has to stay happy."""
    windows = [
        {
            "seq": 7 + i,
            "timestamp_ns": i,
            "dt": 0.02,
            "root_pos_m": np.zeros((2, 3)),
            "root_quat_wxyz": np.tile(np.array([1.0, 0.0, 0.0, 0.0]), (2, 1)),
            "joint_pos_rad": np.zeros((2, 3)),
            "joint_vel_rad_s": np.zeros((2, 3)),
            "source_age_ms": 0.0,
            "valid": True,
        }
        for i in range(4)
    ]
    _write_recording(tmp_path / "rec", windows)
    loaded, _ = load_recorded_windows(tmp_path / "rec")
    seqs = [w.seq for w in loaded]
    assert seqs == sorted(seqs)
    assert len(set(seqs)) == len(seqs)
