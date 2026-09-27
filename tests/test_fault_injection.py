"""Fault injection (plan sections 50 and 80).

Every fault must end in reject / HOLD / INVALID / safe-stop -- never in a silent
extrapolation.  The tests drive the real components (protocol, subscriber,
predictor, converter) with the failure modes the plan lists:

  * PICO / publisher disappears            -> HOLD then SAFE_STOP, no extrapolation
  * NaN in a packet                        -> rejected, last good window kept
  * joint values beyond the limits         -> clamped, never passed through
  * 500 ms delayed packet                  -> aged out, HOLD
  * out-of-order and duplicate sequence    -> counted, never mistaken for new data
  * wrong version / layout / joint order   -> rejected
  * solver failure / non-finite state      -> frame marked invalid, held downstream
"""

from __future__ import annotations

import time

import numpy as np
import pytest
import zmq

from a3_teleop_bridge.a3.limits import load_limits
from a3_teleop_bridge.a3.predictor import A3ReferencePredictor
from a3_teleop_bridge.contract import load_contract
from a3_teleop_bridge.transport.protocol import (
    ProtocolError,
    decode_packet,
    encode_packet,
)
from a3_teleop_bridge.transport.publisher import (
    NetworkConfig,
    ReferencePublisher,
    ReferenceSubscriber,
)
from a3_teleop_bridge.types import A3CanonicalState, A3ReferenceWindow, BridgeState
from a3_teleop_bridge.umr.state_converter import UmrStateConverter


@pytest.fixture(scope="module")
def contract():
    return load_contract()


def make_window(contract, seq: int = 1) -> A3ReferenceWindow:
    n = contract.n_policy_joints
    frames = contract.window_frames
    return A3ReferenceWindow(
        seq=seq,
        timestamp_ns=1_700_000_000_000 + seq,
        dt=contract.window_dt,
        root_pos_m=np.zeros((frames, 3)),
        root_quat_wxyz=np.tile(np.array([1.0, 0.0, 0.0, 0.0]), (frames, 1)),
        joint_pos_rad=np.tile(load_limits().default_angle, (frames, 1)),
        joint_vel_rad_s=np.zeros((frames, n)),
        valid=True,
    )


# --------------------------------------------------------------------------
# packet level
# --------------------------------------------------------------------------
def _retag(packet: bytearray, mutate) -> bytes:
    import msgpack

    header_len = int.from_bytes(packet[4:8], "little")
    header = msgpack.unpackb(bytes(packet[8 : 8 + header_len]), raw=False)
    payload = bytes(packet[8 + header_len :])
    mutate(header)
    new_header = msgpack.packb(header, use_bin_type=True)
    return bytes(packet[:4]) + len(new_header).to_bytes(4, "little") + new_header + payload


def test_nan_joint_rejected(contract):
    window = make_window(contract)
    packet = bytearray(encode_packet(window, contract))
    header_len = int.from_bytes(packet[4:8], "little")
    payload = np.frombuffer(bytes(packet[8 + header_len :]), dtype="<f4").copy()
    payload[10 * 3 + 10 * 4 + 3] = np.nan  # first joint of slot 0
    tampered = bytes(packet[: 8 + header_len]) + payload.tobytes()
    with pytest.raises(ProtocolError, match="non-finite"):
        decode_packet(tampered, contract)


def test_inf_root_rejected(contract):
    window = make_window(contract)
    packet = bytearray(encode_packet(window, contract))
    header_len = int.from_bytes(packet[4:8], "little")
    payload = np.frombuffer(bytes(packet[8 + header_len :]), dtype="<f4").copy()
    payload[2] = np.inf  # root z of slot 0
    tampered = bytes(packet[: 8 + header_len]) + payload.tobytes()
    with pytest.raises(ProtocolError, match="non-finite"):
        decode_packet(tampered, contract)


def test_wrong_version_rejected(contract):
    window = make_window(contract)
    packet = bytearray(encode_packet(window, contract))
    tampered = _retag(packet, lambda h: h.update({"version": "A3_REFERENCE_V0"}))
    with pytest.raises(ProtocolError, match="version"):
        decode_packet(tampered, contract)


def test_wrong_layout_rejected(contract):
    window = make_window(contract)
    packet = bytearray(encode_packet(window, contract))

    def mutate(header):
        header["shapes"] = dict(header["shapes"])
        header["shapes"]["joint_pos_rad"] = [10, 28]

    with pytest.raises(ProtocolError, match="layout"):
        decode_packet(_retag(packet, mutate), contract)


def test_out_of_order_joints_rejected(contract):
    window = make_window(contract)
    packet = bytearray(encode_packet(window, contract))
    tampered = _retag(packet, lambda h: h.update({"joint_order": "a3_policy_v1"}))
    with pytest.raises(ProtocolError, match="joint order"):
        decode_packet(tampered, contract)


def test_truncated_packet_rejected(contract):
    window = make_window(contract)
    packet = encode_packet(window, contract)
    with pytest.raises(ProtocolError):
        decode_packet(packet[: len(packet) // 2], contract)


# --------------------------------------------------------------------------
# transport level
# --------------------------------------------------------------------------
def test_publisher_stop_leads_to_hold(contract):
    """When the publisher stops, the subscriber must not invent new windows."""
    context = zmq.Context()
    config = NetworkConfig.from_yaml()
    port = 15600
    publisher = ReferencePublisher(config, contract, context=context, bind=f"tcp://127.0.0.1:{port}")
    subscriber = ReferenceSubscriber(config, contract, context=context, connect=f"tcp://127.0.0.1:{port}")
    try:
        got = None
        for _ in range(400):
            publisher.send(make_window(contract, seq=1))
            got = subscriber.poll(timeout_ms=20)
            if got is not None and not got.rejected:
                break
        assert got is not None and not got.rejected

        publisher.close()  # simulate the publisher disappearing
        time.sleep(0.15)
        assert subscriber.poll(timeout_ms=100) is None, "subscriber invented a window"
        stats = subscriber.stats()
        assert stats["received"] == 1
    finally:
        subscriber.close()
        context.term()


def test_out_of_order_and_duplicate_sequences(contract):
    """Sequence regressions must be counted, never treated as new data."""
    context = zmq.Context()
    config = NetworkConfig.from_yaml()
    port = 15601
    publisher = ReferencePublisher(config, contract, context=context, bind=f"tcp://127.0.0.1:{port}")
    subscriber = ReferenceSubscriber(config, contract, context=context, connect=f"tcp://127.0.0.1:{port}")
    try:
        for seq in (1, 2):
            for _ in range(200):
                publisher.send(make_window(contract, seq=seq))
                if subscriber.poll(timeout_ms=20) is not None:
                    break
        for seq in (2, 1, 5):  # duplicate, rewind, jump
            publisher.send(make_window(contract, seq=seq))
            time.sleep(0.02)
            subscriber.poll(timeout_ms=50)
        stats = subscriber.stats()
        assert stats["duplicates"] >= 1
        assert stats["reordered"] >= 1
        assert stats["dropped"] >= 2
    finally:
        subscriber.close()
        publisher.close()
        context.term()


def test_delayed_packet_becomes_stale(contract):
    """A 500 ms old reference must classify as SAFE_STOP, not TRACKING."""
    predictor = A3ReferencePredictor()
    limits = load_limits()
    for i in range(10):
        predictor.push(
            A3CanonicalState(
                seq=i,
                timestamp_ns=int(i * 20e6),
                root_pos_m=np.array([0.0, 0.0, 1.0]),
                root_quat_wxyz=np.array([1.0, 0.0, 0.0, 0.0]),
                joint_pos_rad=limits.default_angle,
                joint_vel_rad_s=np.zeros(29),
            )
        )
    assert predictor.window(timestamp_ns=0, source_age_ms=0.0).state is BridgeState.TRACKING
    held = predictor.window(timestamp_ns=0, source_age_ms=500.0)
    assert held.state is BridgeState.SAFE_STOP
    assert held.valid is False
    np.testing.assert_allclose(held.joint_vel_rad_s, 0.0)
    # the window is frozen, not extrapolated
    np.testing.assert_allclose(held.joint_pos_rad[0], held.joint_pos_rad[-1])


# --------------------------------------------------------------------------
# solver / converter level
# --------------------------------------------------------------------------
def test_solver_failure_is_held_downstream():
    predictor = A3ReferencePredictor()
    limits = load_limits()
    good = A3CanonicalState(
        seq=0,
        timestamp_ns=0,
        root_pos_m=np.array([0.0, 0.0, 1.0]),
        root_quat_wxyz=np.array([1.0, 0.0, 0.0, 0.0]),
        joint_pos_rad=limits.default_angle,
        joint_vel_rad_s=np.zeros(29),
    )
    assert predictor.push(good)
    for bad in (
        A3CanonicalState.invalid(seq=1, timestamp_ns=int(20e6), reason="solver fail"),
        A3CanonicalState(
            seq=2,
            timestamp_ns=int(40e6),
            root_pos_m=np.array([np.nan, 0.0, 1.0]),
            root_quat_wxyz=np.array([1.0, 0.0, 0.0, 0.0]),
            joint_pos_rad=limits.default_angle,
            joint_vel_rad_s=np.zeros(29),
        ),
    ):
        assert not predictor.push(bad)
        window = predictor.window(timestamp_ns=bad.timestamp_ns, source_age_ms=1.0)
        assert window.is_finite
        np.testing.assert_allclose(window.joint_pos_rad[0], limits.default_angle, atol=1e-9)


def test_out_of_limit_joints_are_clamped():
    converter = UmrStateConverter()
    limits = load_limits()
    contract = load_contract()
    values = {name: 99.0 for name in contract.policy_joint_names}
    state = converter.convert_frame(
        values, seq=0, timestamp_ns=0, root_pos_m=np.zeros(3), root_quat_wxyz=np.array([1.0, 0, 0, 0])
    )
    assert state.valid
    assert "clamped" in state.reason
    assert np.all(state.joint_pos_rad <= limits.upper + 1e-12)


def test_velocity_explosion_is_clamped():
    converter = UmrStateConverter()
    limits = load_limits()
    contract = load_contract()
    dt = 0.02
    first = converter.convert_frame(
        dict.fromkeys(contract.policy_joint_names, 0.0),
        seq=0,
        timestamp_ns=0,
        root_pos_m=np.zeros(3),
        root_quat_wxyz=np.array([1.0, 0, 0, 0]),
    )
    second = converter.convert_frame(
        dict.fromkeys(contract.policy_joint_names, 5.0),
        seq=1,
        timestamp_ns=int(dt * 1e9),
        root_pos_m=np.zeros(3),
        root_quat_wxyz=np.array([1.0, 0, 0, 0]),
        previous=first,
        dt=dt,
    )
    assert second.valid
    assert np.all(np.abs(second.joint_vel_rad_s) <= limits.velocity + 1e-9)


def test_bad_dt_marks_invalid_and_downstream_holds():
    converter = UmrStateConverter()
    contract = load_contract()
    first = converter.convert_frame(
        dict.fromkeys(contract.policy_joint_names, 0.0),
        seq=0,
        timestamp_ns=0,
        root_pos_m=np.zeros(3),
        root_quat_wxyz=np.array([1.0, 0, 0, 0]),
    )
    stale = converter.convert_frame(
        dict.fromkeys(contract.policy_joint_names, 0.1),
        seq=1,
        timestamp_ns=1,
        root_pos_m=np.zeros(3),
        root_quat_wxyz=np.array([1.0, 0, 0, 0]),
        previous=first,
        dt=1.5,  # solver stalled for 1.5 s
    )
    assert not stale.valid
    assert "dt" in stale.reason

    predictor = A3ReferencePredictor()
    assert predictor.push(first)
    assert not predictor.push(stale)
    window = predictor.window(timestamp_ns=0, source_age_ms=1.0)
    np.testing.assert_allclose(window.joint_pos_rad[0], first.joint_pos_rad)
