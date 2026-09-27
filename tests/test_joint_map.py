"""Joint-name mapping behaviour (plan sections 18, 80, 85)."""

from __future__ import annotations

import numpy as np
import pytest

from a3_teleop_bridge.a3.joint_map import JointMapError, load_joint_map
from a3_teleop_bridge.contract import load_contract


@pytest.fixture(scope="module")
def jmap():
    return load_joint_map()


def test_map_matches_contract(jmap):
    contract = load_contract()
    assert jmap.policy_joint_names == contract.policy_joint_names
    assert jmap.n_joints == 29


def test_forbidden_names_are_recorded(jmap):
    contract = load_contract()
    for name in list(contract.head_joint_names) + list(contract.passive_foot_joint_names):
        assert name in jmap.forbidden_source_names


def test_identity_mapping_with_exact_names(jmap):
    index, resolved, missing = jmap.build_index_map(jmap.policy_joint_names)
    assert missing == []
    np.testing.assert_array_equal(index, np.arange(29))
    assert tuple(resolved) == jmap.policy_joint_names


def test_mapping_uses_names_not_positions(jmap):
    """A shuffled source vector must still map onto the same policy joints."""
    names = list(reversed(jmap.policy_joint_names))
    values = np.arange(29, dtype=np.float64)
    mapped = jmap.map_vector(values, names)
    # policy joint i lives at position 28 - i in the reversed vector
    np.testing.assert_allclose(mapped, np.arange(29)[::-1])


def test_mapping_with_extra_source_joints(jmap):
    """Extra (head/passive) columns are allowed in a *source* vector."""
    contract = load_contract()
    names = list(contract.passive_foot_joint_names) + list(jmap.policy_joint_names) + ["head_yaw_joint"]
    values = np.zeros(len(names))
    values[len(contract.passive_foot_joint_names) + 6] = 1.0  # left_elbow_joint (policy idx 6)
    mapped = jmap.map_vector(values, names)
    assert mapped[6] == pytest.approx(1.0)


def test_alias_resolution(jmap):
    alias_source = "left_hip_pitch"
    canonical = jmap.canonical_source_name(alias_source)
    if canonical is None:
        pytest.skip("no alias configured in this build")
    assert canonical == "left_hip_pitch_joint"


def test_missing_policy_joint_raises(jmap):
    names = list(jmap.policy_joint_names)[:-1]
    with pytest.raises(JointMapError):
        jmap.build_index_map(names)


def test_duplicate_source_names_raise(jmap):
    names = list(jmap.policy_joint_names) + [jmap.policy_joint_names[0]]
    with pytest.raises(JointMapError):
        jmap.build_index_map(names)


def test_ambiguous_alias_raises():
    """Two source names mapping to the same policy joint must be rejected."""
    from pathlib import Path

    import yaml

    from a3_teleop_bridge.a3 import joint_map as jm

    contract = load_contract()
    doc = {
        "policy_joint_names": list(contract.policy_joint_names),
        "aliases": {"left_elbow": "left_elbow_joint", "left_elbow_joint": "left_elbow_joint"},
        "forbidden_source_names": [],
    }
    path = Path("/tmp/_a3_joint_map_alias_test.yaml")
    path.write_text(yaml.safe_dump(doc), encoding="utf-8")
    try:
        with pytest.raises(JointMapError):
            jm.load_joint_map(path)
    finally:
        path.unlink(missing_ok=True)
        jm._load_cached.cache_clear()


def test_forbidden_check_on_policy_vector(jmap):
    contract = load_contract()
    with pytest.raises(JointMapError):
        jmap.assert_no_forbidden(list(jmap.policy_joint_names) + [contract.head_joint_names[0]])
    jmap.assert_no_forbidden(jmap.policy_joint_names)


def test_vector_length_mismatch_raises(jmap):
    with pytest.raises(JointMapError):
        jmap.map_vector(np.zeros(5), jmap.policy_joint_names)
