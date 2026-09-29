"""Future-reference predictor behaviour (plan sections 34-36, 80)."""

from __future__ import annotations

import numpy as np
import pytest

from a3_teleop_bridge.a3.limits import load_limits
from a3_teleop_bridge.a3.predictor import (
    A3ReferencePredictor,
    OneEuroFilter,
    PredictorConfig,
    quat_exp_map,
    quat_log_map,
    quat_multiply,
)
from a3_teleop_bridge.contract import load_contract
from a3_teleop_bridge.types import A3CanonicalState, BridgeState


@pytest.fixture(scope="module")
def contract():
    return load_contract()


@pytest.fixture()
def predictor():
    return A3ReferencePredictor()


def feed(predictor, poses, dt=0.02, start_ns=0, quats=None):
    accepted = 0
    for i, pose in enumerate(poses):
        quat = np.array([1.0, 0.0, 0.0, 0.0]) if quats is None else quats[i]
        state = A3CanonicalState(
            seq=i,
            timestamp_ns=start_ns + int(round(i * dt * 1e9)),
            root_pos_m=pose[:3],
            root_quat_wxyz=quat,
            joint_pos_rad=pose[3:],
            joint_vel_rad_s=np.zeros(29),
        )
        accepted += int(predictor.push(state))
    return accepted


# --------------------------------------------------------------------------
# quaternion math
# --------------------------------------------------------------------------
def test_quat_exp_log_roundtrip():
    for rotvec in (np.zeros(3), np.array([0.1, 0.0, 0.0]), np.array([0.3, -0.2, 0.5])):
        quat = quat_exp_map(rotvec)
        assert np.linalg.norm(quat) == pytest.approx(1.0)
        np.testing.assert_allclose(quat_log_map(quat), rotvec, atol=1e-12)


def test_quat_multiply_identity():
    q = quat_exp_map(np.array([0.2, 0.3, -0.1]))
    np.testing.assert_allclose(quat_multiply(q, np.array([1.0, 0, 0, 0])), q, atol=1e-12)


# --------------------------------------------------------------------------
# One Euro filter
# --------------------------------------------------------------------------
def test_one_euro_tracks_constant():
    filt = OneEuroFilter(size=3, freq_hz=50.0)
    for _ in range(50):
        out = filt.filter(np.array([1.0, 2.0, 3.0]))
    np.testing.assert_allclose(out, [1.0, 2.0, 3.0], atol=1e-9)


def test_one_euro_smooths_noise():
    rng = np.random.default_rng(0)
    filt = OneEuroFilter(size=1, freq_hz=50.0, min_cutoff=1.0, beta=0.0)
    signal = np.ones((200, 1))
    noisy = signal + rng.normal(0, 0.05, size=signal.shape)
    outs = np.array([filt.filter(x)[0] for x in noisy])
    assert outs[10:].std() < noisy[10:].std()
    assert abs(outs[-1] - 1.0) < 0.02


# --------------------------------------------------------------------------
# stationary / constant velocity
# --------------------------------------------------------------------------
def test_predictor_stationary(predictor, contract):
    """A stationary source must produce a constant window (no drift)."""
    pose = np.zeros(3 + 29)
    pose[2] = 0.9
    pose[3:] = load_limits().default_angle
    assert feed(predictor, [pose] * 20) == 20
    window = predictor.window(timestamp_ns=int(20 * 0.02 * 1e9), source_age_ms=5.0)
    assert window.valid
    assert window.frames == contract.window_frames
    np.testing.assert_allclose(
        window.joint_pos_rad, np.tile(pose[3:], (contract.window_frames, 1)), atol=1e-6
    )
    np.testing.assert_allclose(
        window.root_pos_m, np.tile(pose[:3], (contract.window_frames, 1)), atol=1e-6
    )
    np.testing.assert_allclose(window.joint_vel_rad_s, 0.0, atol=1e-6)
    for i in range(window.frames):
        assert window.root_quat_wxyz[i][0] == pytest.approx(1.0)


def test_predictor_constant_velocity(predictor, contract):
    """Constant joint velocity must extrapolate linearly to +180 ms.

    Only the *slope inside the window* is checked: the One Euro filter adds a
    deliberate small lag (~tau * rate) to the absolute level, which is exactly
    the trade-off we want (smooth input, no extra window delay).
    """
    limits = load_limits()
    rate = 0.05  # rad/s -- small enough that no joint reaches its limit
    dt = 0.02
    poses = []
    for i in range(60):
        q = limits.clamp_position(limits.default_angle + rate * (i * dt))
        poses.append(np.concatenate([[0.0, 0.0, 0.9], q]))
    assert feed(predictor, poses, dt=dt) == 60

    # the filtered baseline may lag, but it must still move forward
    assert np.all(predictor.joint_vel > 0)

    window = predictor.window(timestamp_ns=int(60 * dt * 1e9), source_age_ms=1.0)
    assert window.valid
    baseline = window.joint_pos_rad[0]
    deltas = window.joint_pos_rad - baseline
    horizons = np.asarray(contract.window_offsets_s)
    for i, h in enumerate(horizons):
        np.testing.assert_allclose(
            deltas[i], rate * h, atol=5e-4, err_msg=f"slot {i} at {h}s"
        )
    assert window.joint_pos_rad[-1].min() > baseline.min()


def test_predictor_root_translation_extrapolates(predictor):
    dt = 0.02
    speed = 0.4  # m/s
    poses = []
    for i in range(30):
        pose = np.zeros(3 + 29)
        pose[0] = speed * i * dt
        pose[2] = 0.9
        pose[3:] = load_limits().default_angle
        poses.append(pose)
    feed(predictor, poses, dt=dt)
    window = predictor.window(timestamp_ns=int(30 * dt * 1e9), source_age_ms=1.0)
    span = window.root_pos_m[-1, 0] - window.root_pos_m[0, 0]
    assert span == pytest.approx(speed * 0.18, rel=0.05)


def test_predictor_orientation_uses_exp_map(predictor):
    """Rotating about z at a constant rate must advance yaw linearly, not shear."""
    dt = 0.02
    rate = 1.0  # rad/s
    quats = []
    poses = []
    for i in range(30):
        angle = rate * i * dt
        quats.append(quat_exp_map(np.array([0.0, 0.0, angle])))
        pose = np.zeros(3 + 29)
        pose[2] = 0.9
        pose[3:] = load_limits().default_angle
        poses.append(pose)
    feed(predictor, poses, dt=dt, quats=quats)
    window = predictor.window(timestamp_ns=int(30 * dt * 1e9), source_age_ms=1.0)
    yaws = [2.0 * np.arctan2(q[3], q[0]) for q in window.root_quat_wxyz]
    deltas = np.diff(yaws)
    assert np.all(deltas > 0)
    assert yaws[-1] - yaws[0] == pytest.approx(rate * 0.18, rel=0.05)


def test_predictor_limits_joint_velocity(predictor):
    """An absurd velocity spike must be clamped to the hardware limit."""
    dt = 0.02
    limits = load_limits()
    poses = [np.concatenate([[0, 0, 0.9], limits.default_angle]) for _ in range(10)]
    feed(predictor, poses, dt=dt)
    spike = limits.default_angle + 1.0
    spike = limits.clamp_position(spike)
    poses.append(np.concatenate([[0, 0, 0.9], spike]))
    predictor.push(
        A3CanonicalState(
            seq=99,
            timestamp_ns=int(10 * dt * 1e9),
            root_pos_m=np.array([0.0, 0.0, 0.9]),
            root_quat_wxyz=np.array([1.0, 0, 0, 0]),
            joint_pos_rad=spike,
            joint_vel_rad_s=np.zeros(29),
        )
    )
    assert np.all(np.abs(predictor.joint_vel) <= limits.velocity + 1e-9)


def test_predictor_rejects_nonfinite(predictor):
    bad = A3CanonicalState.invalid(seq=0, timestamp_ns=0, reason="solver fail")
    assert not predictor.push(bad)
    assert "solver fail" in predictor.last_reject_reason

    pose = np.zeros(3 + 29)
    pose[2] = 0.9
    pose[3:] = load_limits().default_angle
    assert predictor.push(
        A3CanonicalState(
            seq=1,
            timestamp_ns=0,
            root_pos_m=pose[:3],
            root_quat_wxyz=np.array([1.0, 0, 0, 0]),
            joint_pos_rad=pose[3:],
            joint_vel_rad_s=np.zeros(29),
        )
    )


def test_predictor_rejects_bad_dt(predictor):
    pose = np.zeros(3 + 29)
    pose[2] = 0.9
    pose[3:] = load_limits().default_angle
    for seq, ts in ((0, 0), (1, int(0.02e9))):
        assert predictor.push(
            A3CanonicalState(
                seq=seq,
                timestamp_ns=ts,
                root_pos_m=pose[:3],
                root_quat_wxyz=np.array([1.0, 0, 0, 0]),
                joint_pos_rad=pose[3:],
                joint_vel_rad_s=np.zeros(29),
            )
        )
    # same timestamp -> dt == 0 -> rejected
    assert not predictor.push(
        A3CanonicalState(
            seq=2,
            timestamp_ns=int(0.02e9),
            root_pos_m=pose[:3],
            root_quat_wxyz=np.array([1.0, 0, 0, 0]),
            joint_pos_rad=pose[3:],
            joint_vel_rad_s=np.zeros(29),
        )
    )


# --------------------------------------------------------------------------
# stale handling (plan section 36)
# --------------------------------------------------------------------------
def test_stale_reference_holds(predictor):
    pose = np.zeros(3 + 29)
    pose[2] = 0.9
    pose[3:] = load_limits().default_angle
    feed(predictor, [pose] * 5)
    # derive the age from the configured thresholds instead of hardcoding one:
    # hold_after_ms was raised 50 -> 150 so a 30 Hz headset stream stops
    # flapping TRACKING/HOLD, and a magic 100.0 silently stopped being "stale"
    stale_ms = predictor.config.hold_after_ms + 1.0
    assert stale_ms < predictor.config.invalid_after_ms
    window = predictor.window(timestamp_ns=0, source_age_ms=stale_ms)
    assert window.state is BridgeState.HOLD
    assert not window.valid
    np.testing.assert_allclose(window.joint_vel_rad_s, 0.0)


def test_very_stale_reference_is_invalid(predictor):
    pose = np.zeros(3 + 29)
    pose[2] = 0.9
    pose[3:] = load_limits().default_angle
    feed(predictor, [pose] * 5)
    window = predictor.window(
        timestamp_ns=0, source_age_ms=predictor.config.invalid_after_ms + 1.0
    )
    assert not window.valid
    assert window.state is BridgeState.SAFE_STOP


def test_no_source_is_disconnected(predictor):
    window = predictor.window(timestamp_ns=0, source_age_ms=0.0)
    assert not window.valid
    assert window.state is BridgeState.DISCONNECTED


def test_predictor_never_produces_nan(predictor):
    rng = np.random.default_rng(3)
    limits = load_limits()
    dt = 0.02
    for i in range(200):
        q = limits.default_angle + 0.3 * np.sin(2 * np.pi * i * dt)
        pose = np.concatenate([rng.normal(0, 0.01, 3), q])
        predictor.push(
            A3CanonicalState(
                seq=i,
                timestamp_ns=int(i * dt * 1e9),
                root_pos_m=pose[:3],
                root_quat_wxyz=quat_exp_map(rng.normal(0, 0.05, 3)),
                joint_pos_rad=pose[3:],
                joint_vel_rad_s=np.zeros(29),
            )
        )
        window = predictor.window(timestamp_ns=int(i * dt * 1e9), source_age_ms=1.0)
        assert window.is_finite, f"non-finite window at frame {i}"


def test_predictor_config_from_yaml():
    config = PredictorConfig.from_yaml()
    limits = load_limits()
    assert config.joint_velocity_limit.shape == (29,)
    np.testing.assert_allclose(config.joint_velocity_limit, limits.velocity)
    assert config.hold_after_ms < config.invalid_after_ms


def test_published_reference_obeys_the_validated_velocity_envelope():
    """A fast operator move must not be published faster than the policy has seen.

    Every one of the 20 shipped references is clamped at exactly 2.5 rad/s (620
    joint samples, p99 equal to max, none above), so 2.5 rad/s is the envelope the
    policy was trained and validated on.  The predictor's ``joint_velocity_limit``
    is the *hardware* limit (12-23 rad/s) and only bounds the extrapolation
    velocity, so the base frame went out unbounded: a real session published
    876 deg/s of elbow, 6.1x the validated ceiling, and the robot fell.
    """
    import numpy as np

    from a3_teleop_bridge.a3.predictor import A3ReferencePredictor, PredictorConfig
    from a3_teleop_bridge.contract import load_contract
    from a3_teleop_bridge.types import A3CanonicalState

    config = PredictorConfig.from_yaml()
    assert abs(config.reference_velocity_limit - 2.5) < 1e-9
    # it must be well below the hardware limit, or it would not be doing anything
    assert config.reference_velocity_limit < float(np.min(config.joint_velocity_limit))

    predictor = A3ReferencePredictor()
    n = load_contract().n_policy_joints
    dt = 0.02
    previous = np.zeros(n)
    for i in range(6):
        target = previous + 0.5  # 25 rad/s at dt=0.02, 10x the ceiling
        predictor.push(
            A3CanonicalState(
                seq=i,
                timestamp_ns=int(i * dt * 1e9),
                root_pos_m=np.array([0.0, 0.0, 1.07]),
                root_quat_wxyz=np.array([1.0, 0.0, 0.0, 0.0]),
                joint_pos_rad=target.copy(),
                joint_vel_rad_s=np.zeros(n),
                valid=True,
                solver_latency_ms=1.0,
            )
        )
        step = predictor.last_safe.joint_pos_rad[0] - previous[0]
        if i > 0:  # the first frame has no predecessor to limit against
            assert abs(step) <= config.reference_velocity_limit * dt + 1e-9
        previous = np.asarray(predictor.last_safe.joint_pos_rad).copy()
