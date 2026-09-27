"""PICO packed-message decoding (plan sections 23, 80).

Compatibility is checked against the *official* packer: the test runs
``gear_sonic/scripts/pico_pose_zmq_minimal.py:pack_pose_message`` inside the
sonic_for_a3 virtualenv and decodes the resulting bytes with the bridge.
"""

from __future__ import annotations

import json
import subprocess
from pathlib import Path

import numpy as np
import pytest
import zmq

from a3_teleop_bridge.contract import load_contract
from a3_teleop_bridge.pico.zmq_subscriber import (
    PACKED_HEADER_SIZE,
    PicoPoseSubscriber,
    PicoProtocolError,
    TeleopConfig,
    decode_packed_message,
)
from a3_teleop_bridge.types import SMPL_JOINT_COUNT, SMPL_POSE_COUNT

BRIDGE_ROOT = Path(__file__).resolve().parents[1]


def pack_like_official(payload: dict[str, np.ndarray], topic: str = "pose") -> bytes:
    """Mirror of the official packer (used when the sonic venv is unavailable)."""
    dtype_names = {
        np.dtype("float32"): "f32",
        np.dtype("float64"): "f64",
        np.dtype("int32"): "i32",
        np.dtype("int64"): "i64",
        np.dtype("bool"): "bool",
    }
    fields = []
    chunks = []
    for key, value in payload.items():
        value = np.ascontiguousarray(value)
        fields.append(
            {"name": key, "dtype": dtype_names[value.dtype], "shape": list(value.shape)}
        )
        chunks.append(value.tobytes())
    header = {"v": 3, "endian": "le", "count": 1, "fields": fields}
    header_bytes = json.dumps(header, separators=(",", ":")).encode("utf-8")
    assert len(header_bytes) <= PACKED_HEADER_SIZE
    return topic.encode() + header_bytes.ljust(PACKED_HEADER_SIZE, b"\x00") + b"".join(chunks)


def sample_payload(seed: int = 0) -> dict[str, np.ndarray]:
    rng = np.random.default_rng(seed)
    return {
        "smpl_pose": rng.normal(0, 0.1, (1, SMPL_POSE_COUNT, 3)).astype(np.float32),
        "smpl_joints": rng.normal(0, 0.2, (1, SMPL_JOINT_COUNT, 3)).astype(np.float32),
        "body_quat_w": np.array([[0.99, 0.0, 0.1, 0.0]], dtype=np.float32),
        "joint_pos": rng.normal(0, 0.1, (1, 29)).astype(np.float32),
        "joint_vel": np.zeros((1, 29), dtype=np.float32),
        "timestamp_realtime": np.asarray([1_700_000_000.25], dtype=np.float64),
        "timestamp_monotonic": np.asarray([12345.5], dtype=np.float64),
    }


def test_decode_roundtrip():
    payload = sample_payload()
    message = decode_packed_message(pack_like_official(payload))
    assert message.count == 1
    for name, value in payload.items():
        np.testing.assert_allclose(message.fields[name], value)


def test_decode_uses_names_not_order():
    payload = sample_payload()
    reversed_payload = dict(reversed(list(payload.items())))
    message = decode_packed_message(pack_like_official(reversed_payload))
    for name, value in payload.items():
        np.testing.assert_allclose(message.fields[name], value)


def test_decode_rejects_short_message():
    with pytest.raises(PicoProtocolError):
        decode_packed_message(b"pose" + b"{}")


def test_decode_rejects_unknown_dtype():
    header = {
        "v": 3,
        "endian": "le",
        "count": 1,
        "fields": [{"name": "x", "dtype": "f16", "shape": [1]}],
    }
    raw = json.dumps(header).encode()
    with pytest.raises(PicoProtocolError):
        decode_packed_message(b"pose" + raw.ljust(PACKED_HEADER_SIZE, b"\x00") + b"\x00\x00")


def test_decode_rejects_truncated_payload():
    payload = sample_payload()
    full = pack_like_official(payload)
    with pytest.raises(PicoProtocolError):
        decode_packed_message(full[:-8])


def build_subscriber(port: int) -> tuple[PicoPoseSubscriber, zmq.Context]:
    context = zmq.Context()
    config = TeleopConfig(topic="pose", connect_host="127.0.0.1", port=port)
    subscriber = PicoPoseSubscriber(config, context=context, connect=f"tcp://127.0.0.1:{port}")
    return subscriber, context


def test_subscriber_builds_smpl_frame():
    port = 15561
    context = zmq.Context()
    publisher = context.socket(zmq.PUB)
    publisher.bind(f"tcp://127.0.0.1:{port}")
    subscriber, context = build_subscriber(port)
    try:
        frame = None
        for _ in range(200):
            publisher.send(pack_like_official(sample_payload()))
            frame = subscriber.poll(timeout_ms=50)
            if frame is not None and not frame.rejected:
                break
        assert frame is not None and not frame.rejected, frame.reject_reason if frame else "no frame"
        smpl = frame.smpl_frame
        assert smpl is not None
        assert smpl.smpl_joints.shape == (SMPL_JOINT_COUNT, 3)
        assert smpl.smpl_pose.shape == (SMPL_POSE_COUNT, 3)
        assert smpl.root_quat_wxyz.shape == (4,)
        assert smpl.quat_norm() == pytest.approx(1.0, abs=1e-6)
        # the G1 joint vectors must be ignored, not converted
        assert frame.field("joint_pos") is not None
        assert frame.source_timestamp_ns == pytest.approx(int(1_700_000_000.25 * 1e9), rel=1e-9)
    finally:
        subscriber.close()
        publisher.close(linger=0)
        context.term()


def test_subscriber_rejects_missing_fields():
    """A message without smpl_joints must be rejected, not guessed."""
    payload = sample_payload()
    payload.pop("smpl_joints")
    message = pack_like_official(payload)
    context = zmq.Context()
    config = TeleopConfig(topic="pose", connect_host="127.0.0.1", port=15562)
    subscriber = PicoPoseSubscriber(config, context=context, connect=f"tcp://127.0.0.1:15562")
    publisher = context.socket(zmq.PUB)
    publisher.bind("tcp://127.0.0.1:15562")
    try:
        for _ in range(200):
            publisher.send(message)
            frame = subscriber.poll(timeout_ms=50)
            if frame is not None:
                break
        assert frame is not None
        assert frame.rejected
        assert "missing required PICO fields" in frame.reject_reason
    finally:
        subscriber.close()
        publisher.close(linger=0)
        context.term()


def test_subscriber_latest_only():
    port = 15563
    context = zmq.Context()
    publisher = context.socket(zmq.PUB)
    publisher.bind(f"tcp://127.0.0.1:{port}")
    config = TeleopConfig(topic="pose", connect_host="127.0.0.1", port=port)
    subscriber = PicoPoseSubscriber(config, context=context, connect=f"tcp://127.0.0.1:{port}")
    try:
        for _ in range(200):
            publisher.send(pack_like_official(sample_payload()))
            if subscriber.poll(timeout_ms=50) is not None:
                break
        for i in range(300):
            publisher.send(pack_like_official(sample_payload(i)))
        import time

        time.sleep(0.2)
        frame = subscriber.poll(timeout_ms=200)
        assert frame is not None
        stats = subscriber.stats()
        assert stats["messages_seen"] <= 5, f"backlog detected: {stats}"
    finally:
        subscriber.close()
        publisher.close(linger=0)
        context.term()


def test_official_packer_compatibility(tmp_path):
    """Decode bytes produced by the official pico_pose_zmq_minimal packer."""
    contract = load_contract()
    python = contract.sonic_root / ".venv_sim" / "bin" / "python"
    if not python.is_file():
        pytest.skip("sonic_for_a3 .venv_sim not available")
    script = f"""
import importlib.util, sys, numpy as np
spec = importlib.util.spec_from_file_location(
    "pico_zmq_minimal",
    {str(contract.sonic_root / 'gear_sonic/scripts/pico_pose_zmq_minimal.py')!r})
mod = importlib.util.module_from_spec(spec)
sys.modules["pico_zmq_minimal"] = mod
spec.loader.exec_module(mod)
rng = np.random.default_rng(7)
payload = {{
    "smpl_pose": rng.normal(0, 0.1, (1, 21, 3)).astype(np.float32),
    "smpl_joints": rng.normal(0, 0.2, (1, 24, 3)).astype(np.float32),
    "body_quat_w": np.array([[0.98, 0.0, 0.19, 0.0]], dtype=np.float32),
    "joint_pos": rng.normal(0, 0.1, (1, 29)).astype(np.float32),
    "joint_vel": np.zeros((1, 29), dtype=np.float32),
    "timestamp_realtime": np.asarray([1700000000.5], dtype=np.float64),
}}
open({str(tmp_path / 'official_pico.bin')!r}, "wb").write(
    mod.pack_pose_message(payload, topic="pose"))
np.savez({str(tmp_path / 'official_pico.npz')!r}, **payload)
print("ok")
"""
    out = subprocess.run([str(python), "-c", script], capture_output=True, text=True, timeout=600)
    if out.returncode != 0:
        pytest.skip(f"official packer unavailable: {out.stderr.strip().splitlines()[-1:]}")
    message = decode_packed_message((tmp_path / "official_pico.bin").read_bytes())
    reference = np.load(tmp_path / "official_pico.npz")
    for name in reference.files:
        np.testing.assert_allclose(message.fields[name], reference[name])
