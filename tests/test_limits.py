"""Joint limit / gain extraction invariants (plan section 16)."""

from __future__ import annotations

import numpy as np
import pytest

from a3_teleop_bridge.a3.limits import clamp_to_limits, load_limits
from a3_teleop_bridge.contract import load_contract


@pytest.fixture(scope="module")
def limits():
    return load_limits()


def test_limits_cover_policy_joints(limits):
    contract = load_contract()
    assert limits.joint_names == contract.policy_joint_names
    assert limits.n_joints == 29
    assert limits.lower.shape == limits.upper.shape == limits.velocity.shape == (29,)


def test_limits_are_ordered_and_positive(limits):
    assert np.all(limits.lower <= limits.upper)
    assert np.all(limits.velocity > 0)
    assert np.all(limits.effort > 0)


def test_default_pose_inside_limits(limits):
    assert not limits.exceeds(limits.default_angle).any()
    assert np.all(limits.lower <= limits.default_angle)
    assert np.all(limits.default_angle <= limits.upper)


def test_excluded_joints_not_in_limits(limits):
    contract = load_contract()
    for name in list(contract.head_joint_names) + list(contract.passive_foot_joint_names):
        assert name not in limits.joint_names
        assert name in limits.raw["excluded_joints"]


def test_known_hardware_values(limits):
    """Cross-check a few values documented in the 035 material."""
    idx = {name: i for i, name in enumerate(limits.joint_names)}
    assert limits.velocity[idx["left_ankle_pitch_joint"]] == pytest.approx(10.8, abs=1e-6)
    assert limits.velocity[idx["left_ankle_roll_joint"]] == pytest.approx(19.37, abs=1e-6)
    assert limits.velocity[idx["waist_pitch_joint"]] == pytest.approx(9.24785, abs=1e-6)
    assert limits.velocity[idx["waist_roll_joint"]] == pytest.approx(22.7, abs=1e-6)
    assert limits.effort[idx["left_knee_joint"]] == pytest.approx(320.0)
    assert limits.kp[idx["left_knee_joint"]] == pytest.approx(250.0)


def test_clamp_reports_changed_joints(limits):
    q = limits.lower - 1.0
    clamped, changed = clamp_to_limits(q, limits)
    assert changed == 29
    np.testing.assert_allclose(clamped, limits.lower)

    q = limits.default_angle.copy()
    clamped, changed = clamp_to_limits(q, limits)
    assert changed == 0
    np.testing.assert_allclose(clamped, q)


def test_clamp_margin(limits):
    q = limits.upper.copy()
    clamped, changed = clamp_to_limits(q, limits, margin_rad=0.1)
    assert changed == 29
    np.testing.assert_allclose(clamped, limits.upper - 0.1)
