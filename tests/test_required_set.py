"""The plan's mandatory test set (plan section 80), by name.

These are the items the plan requires explicitly.  Most behaviours are also
covered in the topical test modules; this file pins the exact names so the
acceptance checklist can be read off `pytest --collect-only`.
"""

from __future__ import annotations

import numpy as np
import pytest

from a3_teleop_bridge.a3.csv_export import A3FlatCsvCodec, read_a3_flat_csv, write_a3_flat_csv
from a3_teleop_bridge.a3.limits import load_limits
from a3_teleop_bridge.a3.predictor import A3ReferencePredictor
from a3_teleop_bridge.contract import load_contract
from a3_teleop_bridge.transport.protocol import ProtocolError, decode_packet, encode_packet
from a3_teleop_bridge.types import A3CanonicalState, A3ReferenceWindow


@pytest.fixture(scope="module")
def contract():
    return load_contract()


def _state(seq: int, timestamp_ns: int, value: float = 0.0) -> A3CanonicalState:
    limits = load_limits()
    return A3CanonicalState(
        seq=seq,
        timestamp_ns=timestamp_ns,
        root_pos_m=np.array([0.0, 0.0, 1.07]),
        root_quat_wxyz=np.array([1.0, 0.0, 0.0, 0.0]),
        joint_pos_rad=limits.default_angle + value,
        joint_vel_rad_s=np.zeros(29),
    )


def test_timestamp_monotonic(contract):
    """Timestamps must increase; a repeated or rewound stamp is rejected."""
    predictor = A3ReferencePredictor()
    assert predictor.push(_state(0, 0))
    assert predictor.push(_state(1, int(20e6)))
    assert not predictor.push(_state(2, int(20e6))), "duplicate timestamp accepted"
    assert not predictor.push(_state(3, int(10e6))), "rewound timestamp accepted"
    assert "dt" in predictor.last_reject_reason

    # a window carries the timestamp it was built for, and they increase
    stamps = [
        predictor.window(timestamp_ns=t, source_age_ms=1.0).timestamp_ns
        for t in (int(40e6), int(60e6), int(80e6))
    ]
    assert stamps == sorted(stamps) and len(set(stamps)) == 3

    # the protocol preserves the timestamp exactly
    window = A3ReferenceWindow.hold(_state(9, 123456789), seq=1, timestamp_ns=123456789, dt=0.02)
    back = decode_packet(encode_packet(window, contract), contract)
    assert back.timestamp_ns == 123456789


def test_nan_rejection(contract):
    """NaN / Inf anywhere in a reference window must be rejected, not clamped."""
    window = A3ReferenceWindow.hold(_state(1, 1), seq=1, timestamp_ns=1, dt=0.02)
    packet = bytearray(encode_packet(window, contract))
    header_len = int.from_bytes(packet[4:8], "little")
    payload = np.frombuffer(bytes(packet[8 + header_len :]), dtype="<f4").copy()
    payload[0] = np.nan  # root position of slot 0
    with pytest.raises(ProtocolError, match="non-finite"):
        decode_packet(bytes(packet[: 8 + header_len]) + payload.tobytes(), contract)

    payload[0] = np.inf
    with pytest.raises(ProtocolError, match="non-finite"):
        decode_packet(bytes(packet[: 8 + header_len]) + payload.tobytes(), contract)

    # and the predictor refuses a non-finite canonical state outright
    predictor = A3ReferencePredictor()
    bad = _state(0, 0)
    bad.root_pos_m[0] = np.nan
    assert not predictor.push(bad)


@pytest.mark.skipif(
    not (load_contract().sample_csv_path).is_file(), reason="official sample CSV missing"
)
def test_csv_roundtrip(tmp_path):
    """load -> canonical -> export -> reload keeps the numbers."""
    codec = A3FlatCsvCodec()
    source = codec.read(load_contract().sample_csv_path)
    out = tmp_path / "roundtrip.csv"
    write_a3_flat_csv(
        out, source.root_pos_m, source.root_quat_wxyz, source.dof29_rad, frame_ids=source.frame_ids
    )
    back = read_a3_flat_csv(out, source_fps=source.source_fps, frame_stride=1)
    assert back.n_frames == source.n_frames
    np.testing.assert_allclose(back.root_pos_m, source.root_pos_m, atol=1e-9)
    np.testing.assert_allclose(back.dof29_rad, source.dof29_rad, atol=1e-9)


def test_stream_latest_only(contract):
    """A subscriber must expose only the newest window, never a backlog."""
    import time

    import zmq

    from a3_teleop_bridge.transport.publisher import (
        NetworkConfig,
        ReferencePublisher,
        ReferenceSubscriber,
    )

    context = zmq.Context()
    config = NetworkConfig.from_yaml()
    port = 15660
    publisher = ReferencePublisher(config, contract, context=context, bind=f"tcp://127.0.0.1:{port}")
    subscriber = ReferenceSubscriber(config, contract, context=context, connect=f"tcp://127.0.0.1:{port}")
    try:
        window = A3ReferenceWindow.hold(_state(0, 1), seq=0, timestamp_ns=1, dt=contract.window_dt)
        connected = False
        for _ in range(300):
            publisher.send(window)
            got = subscriber.poll(timeout_ms=20)
            if got is not None and not got.rejected:
                connected = True
                break
        assert connected, "no window received"

        for seq in range(1, 500):
            window.seq = seq
            publisher.send(window)
        time.sleep(0.2)
        latest = subscriber.poll(timeout_ms=200)
        assert latest is not None and not latest.rejected
        assert latest.window.seq > 480, f"backlog: got seq {latest.window.seq}"
        assert subscriber.stats()["received"] < 10
    finally:
        subscriber.close()
        publisher.close()
        context.term()
