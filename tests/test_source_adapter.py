"""PICO recording -> SMPL-X sequence (plan sections 25, 26)."""

from __future__ import annotations

import numpy as np
import pytest

from a3_teleop_bridge.pico.recorder import PicoRecording
from a3_teleop_bridge.types import HumanSmplFrame
from a3_teleop_bridge.umr.source_adapter import (
    PicoToSmplxAdapter,
    RootOrientationSource,
    _quat_from_axes,
)


def make_recording(n=20, with_body_quat=True) -> PicoRecording:
    recording = PicoRecording(source="unit-test")
    for i in range(n):
        t = i * 0.02
        pose = np.zeros((21, 3))
        pose[15, 0] = 0.5 * np.sin(t * 3.0)
        # realistic lateral separation so the hips+shoulders heading is defined
        joints = np.zeros((24, 3))
        joints[1] = [0.10, -0.90, 0.0]
        joints[2] = [-0.10, -0.90, 0.0]
        joints[16] = [0.18, 0.30, 0.0]
        joints[17] = [-0.18, 0.30, 0.0]
        recording.frames.append(
            HumanSmplFrame(
                seq=i,
                timestamp_ns=int(t * 1e9),
                smpl_joints=joints,
                smpl_pose=pose,
                root_translation=np.zeros(3),  # PICO does not publish it
                root_quat_wxyz=np.array([1.0, 0.0, 0.0, 0.0]),
                body_quat_w=np.array([1.0, 0.0, 0.0, 0.0]) if with_body_quat else None,
                receive_timestamp_ns=int(t * 1e9),
            )
        )
    return recording


def test_adapter_produces_umr_ready_sequence():
    adapter = PicoToSmplxAdapter()
    sequence = adapter.convert(make_recording())
    assert sequence.poses.shape == (20, 55, 3)
    assert sequence.trans.shape == (20, 3)
    assert sequence.betas.shape == (10,)
    assert np.isfinite(sequence.poses).all()
    assert np.allclose(sequence.poses[:, 22:], 0.0), "hands/face must stay neutral in v1"
    assert sequence.metadata["stream_quaternion_frames"] == 20


def test_adapter_falls_back_to_fixed_root_height():
    sequence = PicoToSmplxAdapter(root_height_m=0.9).convert(make_recording())
    np.testing.assert_allclose(sequence.trans[:, 2], 0.9)


def test_adapter_uses_hips_and_shoulders_when_no_quaternion():
    recording = make_recording(with_body_quat=False)
    adapter = PicoToSmplxAdapter(root_orientation_source=RootOrientationSource.HIPS_SHOULDERS)
    sequence = adapter.convert(recording)
    assert sequence.metadata["stream_quaternion_frames"] == 0
    assert np.isfinite(sequence.poses[:, 0]).all()
    assert not np.allclose(sequence.poses[:, 0], 0.0), "fallback heading must not be identity"


def test_quat_from_axes_is_orthonormal():
    quat = _quat_from_axes(np.array([1.0, 0.2, 0.0]), np.array([0.0, 0.0, 1.0]))
    assert np.linalg.norm(quat) == pytest.approx(1.0)
    # parallel vectors must not produce NaNs
    quat_parallel = _quat_from_axes(np.array([0.0, 0.0, 1.0]), np.array([0.0, 0.0, 1.0]))
    assert np.isfinite(quat_parallel).all()


def test_quaternion_continuity_is_preserved():
    recording = make_recording()
    for i, frame in enumerate(recording.frames):
        # flip the sign every other frame: the adapter must undo it
        if i % 2:
            frame.body_quat_w = -frame.body_quat_w
    sequence = PicoToSmplxAdapter().convert(recording)
    rotvec = sequence.poses[:, 0]
    assert np.abs(np.diff(rotvec, axis=0)).max() < 1e-6


def test_empty_recording_raises():
    with pytest.raises(ValueError):
        PicoToSmplxAdapter().convert(PicoRecording())


def test_save_roundtrip(tmp_path):
    sequence = PicoToSmplxAdapter().convert(make_recording())
    path = sequence.save(tmp_path / "clip.npz")
    data = np.load(path, allow_pickle=True)
    assert data["poses"].shape == (20, 55, 3)
    assert data["trans"].shape == (20, 3)
    assert float(data["mocap_frame_rate"]) > 0
    assert (tmp_path / "clip.json").is_file()


def test_example_recording_converts():
    from pathlib import Path

    example = Path(__file__).resolve().parents[1].parent / "recordings" / "example_synthetic"
    if not (example / "smpl.npz").is_file():
        pytest.skip("no example recording")
    recording = PicoRecording.load(example)
    sequence = PicoToSmplxAdapter().convert(recording)
    assert sequence.n_frames == len(recording)
    assert np.isfinite(sequence.poses).all()
    assert np.isfinite(sequence.trans).all()
