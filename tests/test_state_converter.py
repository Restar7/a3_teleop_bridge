"""UMR -> A3CanonicalState conversion (plan section 19)."""

from __future__ import annotations

import numpy as np
import pytest

from a3_teleop_bridge.a3.limits import load_limits
from a3_teleop_bridge.contract import load_contract
from a3_teleop_bridge.types import A3CanonicalState
from a3_teleop_bridge.umr.state_converter import (
    UmrStateConverter,
    quaternion_continuity,
    unwrap_delta,
)


@pytest.fixture(scope="module")
def converter():
    return UmrStateConverter()


def make_joint_dict(converter, offset: float = 0.0):
    return {name: offset for name in converter.contract.policy_joint_names}


def test_convert_frame_basic(converter):
    state = converter.convert_frame(
        make_joint_dict(converter),
        seq=0,
        timestamp_ns=0,
        root_pos_m=np.array([0.0, 0.0, 0.9]),
        root_quat_wxyz=np.array([1.0, 0.0, 0.0, 0.0]),
    )
    state.validate()
    assert state.valid
    assert state.joint_pos_rad.shape == (29,)
    np.testing.assert_allclose(state.joint_vel_rad_s, 0.0)


def _interior_pose(converter, delta: float):
    """A pose `delta` away from the default angles, guaranteed inside limits."""
    limits = load_limits()
    values = np.clip(
        limits.default_angle + delta, limits.lower + 1e-6, limits.upper - 1e-6
    )
    return dict(zip(converter.contract.policy_joint_names, values))


def test_velocities_are_finite_differences(converter):
    dt = 0.02
    delta = 0.05
    first = converter.convert_frame(
        _interior_pose(converter, 0.0), 0, 0, np.zeros(3), np.array([1.0, 0, 0, 0])
    )
    assert "clamped" not in first.reason
    second = converter.convert_frame(
        _interior_pose(converter, delta), 1, int(dt * 1e9), np.zeros(3), np.array([1.0, 0, 0, 0]),
        previous=first, dt=dt,
    )
    assert second.valid
    np.testing.assert_allclose(second.joint_vel_rad_s, delta / dt, rtol=1e-9)


def test_invalid_dt_marks_frame_invalid(converter):
    first = converter.convert_frame(
        make_joint_dict(converter), 0, 0, np.zeros(3), np.array([1.0, 0, 0, 0])
    )
    for bad_dt in (0.0, -0.02, float("nan"), 5.0):
        state = converter.convert_frame(
            make_joint_dict(converter), 1, 0, np.zeros(3), np.array([1.0, 0, 0, 0]),
            previous=first, dt=bad_dt,
        )
        assert not state.valid
        assert "dt" in state.reason


def test_joint_limits_are_clamped(converter):
    limits = load_limits()
    values = {name: 10.0 for name in converter.contract.policy_joint_names}
    state = converter.convert_frame(
        values, 0, 0, np.zeros(3), np.array([1.0, 0, 0, 0])
    )
    assert "clamped" in state.reason
    assert np.all(state.joint_pos_rad <= limits.upper + 1e-12)
    assert np.all(state.joint_pos_rad >= limits.lower - 1e-12)


def test_nan_root_is_rejected(converter):
    state = converter.convert_frame(
        make_joint_dict(converter), 0, 0, np.array([np.nan, 0.0, 0.0]), np.array([1.0, 0, 0, 0])
    )
    assert not state.valid
    assert "non-finite" in state.reason


def test_degenerate_quaternion_is_rejected(converter):
    state = converter.convert_frame(
        make_joint_dict(converter), 0, 0, np.zeros(3), np.zeros(4)
    )
    assert not state.valid


def test_missing_joint_is_rejected(converter):
    values = make_joint_dict(converter)
    values.pop("left_elbow_joint")
    state = converter.convert_frame(values, 0, 0, np.zeros(3), np.array([1.0, 0, 0, 0]))
    assert not state.valid
    assert "joint map failed" in state.reason


def test_quaternion_continuity_flips_sign():
    prev = np.array([0.0, 0.0, 0.0, 1.0])
    out = quaternion_continuity(-prev, prev)
    np.testing.assert_allclose(out, prev)
    assert float(np.dot(out, prev)) > 0.0

    # already continuous stays untouched
    np.testing.assert_allclose(quaternion_continuity(prev, prev), prev)


def test_unwrap_delta_wraps():
    assert unwrap_delta(0.1) == pytest.approx(0.1)
    assert unwrap_delta(2 * np.pi - 0.1) == pytest.approx(-0.1, abs=1e-9)
    assert abs(unwrap_delta(3 * np.pi)) <= np.pi


def test_convert_sequence_keeps_previous_valid(converter):
    dt = 0.02
    frames = []
    for i in range(5):
        frames.append((make_joint_dict(converter, 0.01 * i), np.zeros(3), np.array([1.0, 0, 0, 0])))
    states = converter.convert_sequence(frames, dt=dt)
    assert len(states) == 5
    assert all(s.valid for s in states)
    assert states[0].timestamp_ns == 0
    assert states[-1].timestamp_ns == int(round(4 * dt * 1e9))


def test_minimal_motion_dataset_has_no_nan(converter):
    """A short synthetic sway must stay finite through the whole chain."""
    dt = 0.02
    frames = []
    for i in range(50):
        values = {
            name: 0.2 * np.sin(2 * np.pi * i * dt) * (1 if "left" in name else -1)
            for name in converter.contract.policy_joint_names
        }
        frames.append((values, np.array([0.0, 0.0, 0.9]), np.array([1.0, 0.0, 0.0, 0.0])))
    states = converter.convert_sequence(frames, dt=dt)
    for state in states:
        assert state.is_finite
        assert np.all(np.abs(state.joint_vel_rad_s) < 1e4)
