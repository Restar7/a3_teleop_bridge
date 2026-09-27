"""Integration suite (plan section 81).

End-to-end chains that must keep working across commits:

  * UMR sample  -> A3 canonical -> flat CSV
  * A3 CSV      -> replay publisher -> subscriber -> predictor -> window
  * stream      -> disconnect    -> HOLD / SAFE_STOP (no extrapolation)

The heavy GPU steps (UMR retargeting, MuJoCo) are exercised by the tools in
``tools/`` and their evidence lives in ``docs/mujoco_validation.md``; these tests
cover the bridge-side chains that can run in seconds.
"""

from __future__ import annotations

import time
from pathlib import Path

import numpy as np
import pytest
import zmq

from a3_teleop_bridge.a3.csv_export import A3FlatCsvCodec, read_a3_flat_csv, write_a3_flat_csv
from a3_teleop_bridge.a3.predictor import A3ReferencePredictor
from a3_teleop_bridge.contract import load_contract
from a3_teleop_bridge.pico.recorder import PicoRecording
from a3_teleop_bridge.types import A3CanonicalState, BridgeState
from a3_teleop_bridge.umr.offline import load_umr_result
from a3_teleop_bridge.umr.state_converter import UmrStateConverter

BRIDGE_ROOT = Path(__file__).resolve().parents[1]
WS = BRIDGE_ROOT.parent


def first_existing(*candidates: Path) -> Path | None:
    for cand in candidates:
        if cand.is_file():
            return cand
    return None


UMR_A3_RESULT = first_existing(
    WS / "UMR" / "output" / "a3_validation" / "stand_smplx_agibot_a3.npz",
    WS / "UMR" / "output" / "agibot_a3_retarget" / "dance1_subject2_smplx_agibot_a3.npz",
)
A3_CSV = first_existing(
    WS / "logs" / "a3_validation" / "stand" / "stand.csv",
    WS / "logs" / "a3_csv" / "dance1_subject2_a3.csv",
)


@pytest.mark.skipif(UMR_A3_RESULT is None, reason="no UMR A3 result available")
def test_umr_result_to_canonical_to_csv(tmp_path):
    """UMR sample -> A3CanonicalState -> flat CSV -> reload."""
    result = load_umr_result(UMR_A3_RESULT)
    converter = UmrStateConverter()
    states = converter.convert_umr_result(result)
    assert len(states) == result.n_frames
    assert all(state.is_finite for state in states)
    # only the first frame may lack a velocity reference
    assert sum(1 for s in states if not s.valid) <= 1

    contract = load_contract()
    n = contract.n_policy_joints
    joint_pos = np.asarray([s.joint_pos_rad for s in states])
    root_pos = np.asarray([s.root_pos_m for s in states])
    root_quat = np.asarray([s.root_quat_wxyz for s in states])
    out = tmp_path / "chain.csv"
    write_a3_flat_csv(out, root_pos, root_quat, joint_pos, frame_ids=np.arange(len(states)))

    back = read_a3_flat_csv(out, source_fps=result.fps, frame_stride=1)
    assert back.n_frames == len(states)
    np.testing.assert_allclose(back.dof29_rad, joint_pos, atol=1e-6)
    np.testing.assert_allclose(back.root_pos_m, root_pos, atol=1e-6)
    assert back.dof29_rad.shape[1] == n


@pytest.mark.skipif(A3_CSV is None, reason="no A3 CSV available")
def test_csv_to_predictor_windows():
    """A3 CSV -> canonical states -> predictor windows stay finite and bounded."""
    contract = load_contract()
    codec = A3FlatCsvCodec()
    data = codec.read(A3_CSV, source_fps=30.0, frame_stride=1)
    predictor = A3ReferencePredictor()
    dt = 1.0 / data.effective_fps
    accepted = 0
    for i in range(data.n_frames):
        state = A3CanonicalState(
            seq=i,
            timestamp_ns=int(round(i * dt * 1e9)),
            root_pos_m=data.root_pos_m[i],
            root_quat_wxyz=data.root_quat_wxyz[i],
            joint_pos_rad=data.dof29_rad[i],
            joint_vel_rad_s=np.zeros(contract.n_policy_joints),
        )
        accepted += int(predictor.push(state))
    assert accepted >= data.n_frames - 1
    window = predictor.window(timestamp_ns=0, source_age_ms=1.0)
    assert window.frames == contract.window_frames
    assert window.is_finite
    assert window.horizon_s == pytest.approx(contract.future_horizon_s)


@pytest.mark.skipif(A3_CSV is None, reason="no A3 CSV available")
def test_csv_to_stream_to_window_and_disconnect():
    """CSV -> publisher -> subscriber -> predictor, then disconnect -> no data."""
    from a3_teleop_bridge.transport.publisher import (
        NetworkConfig,
        ReferencePublisher,
        ReferenceSubscriber,
    )

    contract = load_contract()
    codec = A3FlatCsvCodec()
    data = codec.read(A3_CSV, source_fps=30.0, frame_stride=1)
    predictor = A3ReferencePredictor()
    dt = 1.0 / data.effective_fps
    for i in range(min(60, data.n_frames)):
        predictor.push(
            A3CanonicalState(
                seq=i,
                timestamp_ns=int(round(i * dt * 1e9)),
                root_pos_m=data.root_pos_m[i],
                root_quat_wxyz=data.root_quat_wxyz[i],
                joint_pos_rad=data.dof29_rad[i],
                joint_vel_rad_s=np.zeros(contract.n_policy_joints),
            )
        )

    context = zmq.Context()
    config = NetworkConfig.from_yaml()
    port = 15610
    publisher = ReferencePublisher(config, contract, context=context, bind=f"tcp://127.0.0.1:{port}")
    subscriber = ReferenceSubscriber(config, contract, context=context, connect=f"tcp://127.0.0.1:{port}")
    try:
        got = None
        for _ in range(300):
            publisher.send(predictor.window(timestamp_ns=0, source_age_ms=1.0))
            got = subscriber.poll(timeout_ms=20)
            if got is not None and not got.rejected:
                break
        assert got is not None and not got.rejected
        np.testing.assert_allclose(got.window.joint_pos_rad, predictor.window(timestamp_ns=0, source_age_ms=1.0).joint_pos_rad, atol=1e-6)

        publisher.close()
        time.sleep(0.1)
        assert subscriber.poll(timeout_ms=80) is None
        held = predictor.window(timestamp_ns=0, source_age_ms=400.0)
        assert held.state is BridgeState.SAFE_STOP
        assert not held.valid
    finally:
        subscriber.close()
        context.term()


def test_recorded_pico_shapes_feed_the_adapter_contract(tmp_path):
    """A recording must satisfy the shapes the UMR source adapter requires."""
    recorder_dir = WS / "recordings" / "example_synthetic"
    if not (recorder_dir / "smpl.npz").is_file():
        pytest.skip("no example recording available")
    recording = PicoRecording.load(recorder_dir)
    assert len(recording) > 0
    arrays = recording.arrays()
    assert arrays["smpl_joints"].shape[1:] == (24, 3)
    assert arrays["smpl_pose"].shape[1:] == (21, 3)
    assert arrays["root_quat_wxyz"].shape[1:] == (4,)
    assert np.isfinite(arrays["smpl_joints"]).all()
    stats = recording.stats()
    assert stats["all_finite"] is True
