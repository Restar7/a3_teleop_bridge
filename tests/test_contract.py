"""Contract-level invariants (plan sections 7, 18, 80)."""

from __future__ import annotations

import numpy as np
import pytest

from a3_teleop_bridge import contract as contract_mod


@pytest.fixture(scope="module")
def contract():
    return contract_mod.load_contract()


def test_contract_loads(contract):
    assert contract.raw["schema"] == "a3_contract/v1"
    assert contract.sonic_root.is_dir()


def test_joint_mapping_complete(contract):
    assert contract.n_policy_joints == 29
    assert len(contract.policy_joint_names) == 29
    assert len(set(contract.policy_joint_names)) == 29
    # csv joints minus the head joints, in csv order
    expected = [n for n in contract.csv_joint_names if n not in set(contract.head_joint_names)]
    assert list(contract.policy_joint_names) == expected
    # the index mapping must select exactly those columns
    assert [contract.csv_joint_names[i] for i in contract.policy_to_csv_index] == expected


def test_no_head_in_policy(contract):
    assert set(contract.head_joint_names) == {"head_yaw_joint", "head_pitch_joint"}
    assert not set(contract.head_joint_names) & set(contract.policy_joint_names)


def test_no_passive_joint_in_policy(contract):
    passive = set(contract.passive_foot_joint_names)
    assert passive, "passive foot joints must be declared by the contract"
    assert not passive & set(contract.policy_joint_names)


def test_timing_and_window(contract):
    assert contract.policy_dt == pytest.approx(0.02)
    assert contract.policy_hz == pytest.approx(50.0)
    assert contract.window_frames == 10
    assert contract.window_dt == pytest.approx(0.02)
    assert contract.future_horizon_s == pytest.approx(0.18)
    offsets = contract.window_offsets_s
    assert offsets[0] == 0.0
    assert offsets[-1] == pytest.approx(0.18)


def test_dimensions(contract):
    assert contract.observation_dim == 1570
    assert contract.action_dim == 29
    assert contract.encoder_frame_dim == 64
    assert contract.encoder_input_dim == 640


def test_assets_exist(contract):
    assert contract.mjcf_path.is_file()
    assert contract.urdf_path.is_file()
    assert contract.sample_csv_path.is_file()


def test_contract_is_cached_per_path(contract):
    again = contract_mod.load_contract()
    assert again is contract


def test_missing_contract_raises(tmp_path):
    with pytest.raises(contract_mod.ContractError):
        contract_mod.load_contract(tmp_path / "nope.json")


def test_sample_csv_header_matches_contract(contract):
    header = contract.sample_csv_path.read_text(encoding="utf-8").splitlines()[0].split(",")
    assert header[0] == contract.csv_frame_column
    assert tuple(header[1:7]) == contract.csv_root_columns
    assert tuple(header[7:]) == contract.csv_joint_names


def test_policy_view_has_no_mirroring_bug(contract):
    """Left/right joints must appear in matching order, never swapped."""
    names = list(contract.policy_joint_names)
    left = [n for n in names if n.startswith("left_")]
    right = [n for n in names if n.startswith("right_")]
    # 7 arm joints + 6 leg joints per side
    assert len(left) == len(right) == 13
    assert [n[len("left_") :] for n in left] == [n[len("right_") :] for n in right]
    # limb groups stay contiguous and order within each group is legible
    assert names[:3] == ["waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint"]
    assert names[3:10] == [
        "left_shoulder_pitch_joint",
        "left_shoulder_roll_joint",
        "left_shoulder_yaw_joint",
        "left_elbow_joint",
        "left_wrist_roll_joint",
        "left_wrist_pitch_joint",
        "left_wrist_yaw_joint",
    ]
    assert names[10:17] == [n.replace("left_", "right_") for n in names[3:10]]
    assert names.index("left_shoulder_pitch_joint") < names.index("right_shoulder_pitch_joint")
    assert names.index("left_hip_pitch_joint") < names.index("right_hip_pitch_joint")
    assert set(names[17:23]) == {
        "left_hip_pitch_joint",
        "left_hip_roll_joint",
        "left_hip_yaw_joint",
        "left_knee_joint",
        "left_ankle_pitch_joint",
        "left_ankle_roll_joint",
    }
    assert set(names[23:29]) == {
        "right_hip_pitch_joint",
        "right_hip_roll_joint",
        "right_hip_yaw_joint",
        "right_knee_joint",
        "right_ankle_pitch_joint",
        "right_ankle_roll_joint",
    }


def test_contract_numpy_shapes(contract):
    from a3_teleop_bridge.types import A3CanonicalState, A3ReferenceWindow

    n = contract.n_policy_joints
    window = A3ReferenceWindow(
        seq=0,
        timestamp_ns=0,
        dt=contract.window_dt,
        root_pos_m=np.zeros((contract.window_frames, 3)),
        root_quat_wxyz=np.tile(np.array([1.0, 0, 0, 0]), (contract.window_frames, 1)),
        joint_pos_rad=np.zeros((contract.window_frames, n)),
        joint_vel_rad_s=np.zeros((contract.window_frames, n)),
    )
    window.validate()
    assert window.horizon_s == pytest.approx(contract.future_horizon_s)

    state = A3CanonicalState.invalid(seq=0, timestamp_ns=0)
    assert not state.valid
    assert state.joint_pos_rad.shape == (n,)
