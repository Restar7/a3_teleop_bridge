"""A3_REFERENCE_V1 protocol and transport tests (plan sections 37, 38, 80)."""

from __future__ import annotations

import time

import numpy as np
import pytest
import zmq

from a3_teleop_bridge.a3.limits import load_limits
from a3_teleop_bridge.contract import load_contract
from a3_teleop_bridge.transport.protocol import (
    PROTOCOL_VERSION,
    ProtocolError,
    decode_frames,
    decode_packet,
    encode_frames,
    encode_packet,
)
from a3_teleop_bridge.transport.publisher import (
    NetworkConfig,
    ReferencePublisher,
    ReferenceSubscriber,
)
from a3_teleop_bridge.types import A3CanonicalState, A3ReferenceWindow, BridgeState


def make_window(contract, seq: int = 1, scale: float = 0.0) -> A3ReferenceWindow:
    n = contract.n_policy_joints
    frames = contract.window_frames
    return A3ReferenceWindow(
        seq=seq,
        timestamp_ns=1_700_000_000_000 + seq,
        dt=contract.window_dt,
        root_pos_m=np.linspace(0, 0.1, frames * 3).reshape(frames, 3) + scale,
        root_quat_wxyz=np.tile(np.array([1.0, 0.0, 0.0, 0.0]), (frames, 1)),
        joint_pos_rad=np.linspace(-0.2, 0.2, frames * n).reshape(frames, n) + scale,
        joint_vel_rad_s=np.zeros((frames, n)),
        source_age_ms=3.5,
        solver_latency_ms=7.25,
        valid=True,
        state=BridgeState.TRACKING,
    )


@pytest.fixture(scope="module")
def contract():
    return load_contract()


def test_protocol_roundtrip(contract):
    window = make_window(contract)
    frames = encode_frames(window, contract)
    assert len(frames) == 2
    back = decode_frames(frames, contract)
    assert back.seq == window.seq
    assert back.timestamp_ns == window.timestamp_ns
    assert back.dt == pytest.approx(window.dt)
    assert back.source_age_ms == pytest.approx(window.source_age_ms)
    assert back.solver_latency_ms == pytest.approx(window.solver_latency_ms)
    assert back.valid is True
    assert back.state is BridgeState.TRACKING
    for name in ("root_pos_m", "root_quat_wxyz", "joint_pos_rad", "joint_vel_rad_s"):
        np.testing.assert_allclose(
            getattr(back, name), getattr(window, name), rtol=1e-6, atol=1e-6
        )


def test_protocol_payload_is_compact(contract):
    window = make_window(contract)
    _, payload = encode_frames(window, contract)
    expected = (10 * 3 + 10 * 4 + 10 * 29 + 10 * 29) * 4
    assert len(payload) == expected
    assert expected < 3000  # far below a JSON encoding


def test_protocol_rejects_wrong_version(contract):
    window = make_window(contract)
    header, payload = encode_frames(window, contract)
    import msgpack

    doc = msgpack.unpackb(header, raw=False)
    doc["version"] = "A3_REFERENCE_V0"
    bad = [msgpack.packb(doc, use_bin_type=True), payload]
    with pytest.raises(ProtocolError):
        decode_frames(bad, contract)


def test_protocol_rejects_layout_mismatch(contract):
    window = make_window(contract)
    header, payload = encode_frames(window, contract)
    import msgpack

    doc = msgpack.unpackb(header, raw=False)
    doc["shapes"]["joint_pos_rad"] = [10, 28]
    bad = [msgpack.packb(doc, use_bin_type=True), payload]
    with pytest.raises(ProtocolError):
        decode_frames(bad, contract)


def test_protocol_rejects_nan_payload(contract):
    window = make_window(contract)
    header, payload = encode_frames(window, contract)
    arr = np.frombuffer(payload, dtype="<f4").copy()
    arr[0] = np.nan
    with pytest.raises(ProtocolError):
        decode_frames([header, arr.tobytes()], contract)


def test_protocol_rejects_truncated_payload(contract):
    window = make_window(contract)
    header, payload = encode_frames(window, contract)
    with pytest.raises(ProtocolError):
        decode_frames([header, payload[:64]], contract)


def test_protocol_rejects_garbage(contract):
    with pytest.raises(ProtocolError):
        decode_frames([b"not-a-header", b"x"], contract)
    with pytest.raises(ProtocolError):
        decode_frames([b"only-one-frame"], contract)


def test_network_config_ports():
    config = NetworkConfig.from_yaml()
    assert config.reference_port != config.pico_port
    assert config.conflate is True
    assert config.high_water_mark == 1
    assert config.reference_bind.endswith(str(config.reference_port))


def test_pubsub_latest_only(contract):
    """Publishing faster than we consume must never build a backlog."""
    context = zmq.Context()
    config = NetworkConfig.from_yaml()
    port = 15560  # keep the test off the default port
    publisher = ReferencePublisher(
        config, contract, context=context, bind=f"tcp://127.0.0.1:{port}"
    )
    subscriber = ReferenceSubscriber(
        config, contract, context=context, connect=f"tcp://127.0.0.1:{port}"
    )
    try:
        # give the SUB socket time to finish the handshake
        deadline = time.time() + 5.0
        while time.time() < deadline:
            publisher.send(make_window(contract, seq=0))
            time.sleep(0.01)
            got = subscriber.poll(timeout_ms=50)
            if got is not None and not got.rejected:
                break
        assert got is not None and not got.rejected, "no window received"

        for seq in range(1, 200):
            publisher.send(make_window(contract, seq=seq, scale=seq * 1e-6))
        time.sleep(0.2)
        latest = subscriber.poll(timeout_ms=200)
        assert latest is not None and not latest.rejected
        # latest-only: the window we see must be one of the newest published
        assert latest.window.seq >= 190
        stats = subscriber.stats()
        assert stats["received"] <= 5, f"backlog detected: {stats}"
    finally:
        subscriber.close()
        publisher.close()
        context.term()


def test_hold_window_roundtrip(contract):
    state = A3CanonicalState(
        seq=7,
        timestamp_ns=12345,
        root_pos_m=np.array([0.1, 0.2, 0.9]),
        root_quat_wxyz=np.array([0.7071, 0.0, 0.7071, 0.0]),
        joint_pos_rad=load_limits().default_angle,
        joint_vel_rad_s=np.zeros(29),
    )
    window = A3ReferenceWindow.hold(state, seq=8, timestamp_ns=999, dt=contract.window_dt)
    back = decode_frames(encode_frames(window, contract), contract)
    assert back.valid is False
    assert back.state is BridgeState.HOLD
    np.testing.assert_allclose(back.joint_vel_rad_s, 0.0)
    np.testing.assert_allclose(back.joint_pos_rad[0], state.joint_pos_rad, atol=1e-6)


def test_version_constant_is_frozen():
    assert PROTOCOL_VERSION == "A3_REFERENCE_V1"


def test_wire_joint_order_is_encoder_order(contract):
    """joint_pos on the wire must be permuted into the encoder (il) order."""
    import msgpack

    n = contract.n_policy_joints
    values = np.arange(n, dtype=np.float64)  # policy joint i carries value i
    window = A3ReferenceWindow(
        seq=1,
        timestamp_ns=1,
        dt=contract.window_dt,
        root_pos_m=np.zeros((contract.window_frames, 3)),
        root_quat_wxyz=np.tile(np.array([1.0, 0, 0, 0]), (contract.window_frames, 1)),
        joint_pos_rad=np.tile(values, (contract.window_frames, 1)),
        joint_vel_rad_s=np.zeros((contract.window_frames, n)),
    )
    packet = encode_packet(window, contract)
    header_len = int.from_bytes(packet[4:8], "little")
    header = msgpack.unpackb(packet[8 : 8 + header_len], raw=False)
    assert header["joint_order"] == "a3_il_v1"
    assert header["joint_names"] == list(contract.il_joint_names)

    payload = np.frombuffer(packet[8 + header_len :], dtype="<f4")
    offset = contract.window_frames * 3 + contract.window_frames * 4
    wire_row = payload[offset : offset + n]
    for k in range(n):
        assert wire_row[k] == pytest.approx(float(values[contract.il_to_policy_index[k]]))

    # and the decoder must undo the permutation exactly
    back = decode_packet(packet, contract)
    np.testing.assert_allclose(back.joint_pos_rad, np.tile(values, (contract.window_frames, 1)))


def test_decoder_rejects_wrong_joint_order(contract):
    import msgpack

    window = make_window(contract)
    packet = bytearray(encode_packet(window, contract))
    header_len = int.from_bytes(packet[4:8], "little")
    header = msgpack.unpackb(bytes(packet[8 : 8 + header_len]), raw=False)
    header["joint_order"] = "a3_policy_v1"
    new_header = msgpack.packb(header, use_bin_type=True)
    tampered = bytes(packet[:4]) + len(new_header).to_bytes(4, "little") + new_header + bytes(packet[8 + header_len :])
    with pytest.raises(ProtocolError):
        decode_packet(tampered, contract)
