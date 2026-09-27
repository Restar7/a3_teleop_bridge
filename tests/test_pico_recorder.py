"""PICO recorder round-trip (plan section 24)."""

from __future__ import annotations

import json

import numpy as np
import pytest

from a3_teleop_bridge.pico.recorder import PicoRecorder, PicoRecording
from a3_teleop_bridge.pico.zmq_subscriber import PicoFrame, decode_packed_message
from a3_teleop_bridge.types import SMPL_JOINT_COUNT, SMPL_POSE_COUNT, HumanSmplFrame
from tests.test_pico_subscriber import pack_like_official, sample_payload


def make_frame(seq: int, t_ns: int) -> HumanSmplFrame:
    return HumanSmplFrame(
        seq=seq,
        timestamp_ns=t_ns,
        smpl_joints=np.full((SMPL_JOINT_COUNT, 3), 0.01 * seq),
        smpl_pose=np.zeros((SMPL_POSE_COUNT, 3)),
        root_translation=np.array([0.0, 0.0, 0.95]),
        root_quat_wxyz=np.array([1.0, 0.0, 0.0, 0.0]),
        receive_timestamp_ns=t_ns + 2_000_000,
    )


def pico_frame_wrapper(seq: int, t_ns: int, rejected: bool = False) -> PicoFrame:
    message = decode_packed_message(pack_like_official(sample_payload(seq)))
    return PicoFrame(
        seq=seq,
        message=message,
        receive_timestamp_ns=t_ns + 2_000_000,
        source_timestamp_ns=t_ns,
        receive_latency_ms=2.0,
        smpl_frame=None if rejected else make_frame(seq, t_ns),
        rejected=rejected,
        reject_reason="stub rejection" if rejected else "",
    )


def test_recorder_collects_frames():
    recorder = PicoRecorder(source="unit-test")
    for i in range(50):
        assert recorder.add(pico_frame_wrapper(i, int(i * 0.02 * 1e9)))
    recording = recorder.finalize({"received": 50})
    assert len(recording) == 50
    stats = recording.stats()
    assert stats["frames"] == 50
    assert stats["all_finite"] is True
    assert stats["source_hz_mean"] == pytest.approx(50.0, rel=1e-3)
    assert stats["seq"]["received"] == 50


def test_recorder_counts_rejections():
    recorder = PicoRecorder()
    recorder.add(pico_frame_wrapper(0, 0))
    recorder.add(pico_frame_wrapper(1, 1, rejected=True))
    recorder.add(None)
    recording = recorder.finalize()
    assert len(recording) == 1
    assert recording.rejected == 1
    assert "stub rejection" in recording.reject_reasons


def test_recording_roundtrip(tmp_path):
    recorder = PicoRecorder(source="roundtrip")
    for i in range(25):
        recorder.add(pico_frame_wrapper(i, int(i * 0.02 * 1e9)))
    recording = recorder.finalize({"received": 25, "dropped": 0, "duplicates": 0, "reordered": 0})
    directory = recording.save(tmp_path / "session")

    assert (directory / "smpl.npz").is_file()
    assert (directory / "metadata.json").is_file()
    assert (directory / "stats.json").is_file()

    loaded = PicoRecording.load(directory)
    assert len(loaded) == len(recording)
    np.testing.assert_allclose(
        loaded.arrays()["smpl_joints"], recording.arrays()["smpl_joints"], atol=1e-6
    )
    np.testing.assert_array_equal(loaded.timestamps_ns, recording.timestamps_ns)
    metadata = json.loads((directory / "metadata.json").read_text())
    assert metadata["frames"] == 25
    assert metadata["fields"]


def test_empty_recording_is_safe(tmp_path):
    recording = PicoRecording()
    assert recording.stats() == {"frames": 0}
    directory = recording.save(tmp_path / "empty")
    loaded = PicoRecording.load(directory)
    assert len(loaded) == 0
    assert loaded.arrays()["smpl_joints"].shape == (0, SMPL_JOINT_COUNT, 3)


def test_body_quat_saved_when_available(tmp_path):
    recorder = PicoRecorder()
    for i in range(3):
        frame = make_frame(i, i * 1_000_000)
        frame.body_quat_w = np.array([1.0, 0.0, 0.0, 0.0])
        wrapper = pico_frame_wrapper(i, i * 1_000_000)
        wrapper.smpl_frame = frame
        recorder.add(wrapper)
    recording = recorder.finalize()
    arrays = recording.arrays()
    assert "body_quat_w" in arrays
    assert arrays["body_quat_w"].shape == (3, 4)
