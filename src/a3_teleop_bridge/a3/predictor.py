"""Future reference predictor (plan sections 33-36).

A3-fast does not consume a single pose: it consumes a 10-slot window at 20 ms
spacing (0 .. 180 ms).  The bridge therefore extrapolates the canonical state:

  * joints      : constant velocity, ``q(t+h) = q_t + dq_t * h``
  * root position: ``p(t+h) = p_t + v_root * h``
  * root rotation: angular velocity from ``q_prev^-1 * q_now``, advanced with the
    quaternion exponential map -- **never** Euler linear extrapolation

Inputs are lightly smoothed with a One Euro filter (low lag at speed, strong
smoothing at rest), and every extrapolation is bounded by joint/root velocity and
acceleration limits.  Stale or non-finite input never extrapolates: the window
freezes (HOLD) and, past the invalid threshold, is flagged INVALID so downstream
code can go to a safe state.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import yaml

from ..a3.limits import A3Limits, load_limits
from ..clocks import StaleWatchdog
from ..contract import A3Contract, load_contract
from ..types import A3CanonicalState, A3ReferenceWindow, BridgeState

BRIDGE_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_CONFIG_PATH = BRIDGE_ROOT / "configs" / "predictor.yaml"

__all__ = [
    "OneEuroFilter",
    "PredictorConfig",
    "A3ReferencePredictor",
    "quat_multiply",
    "quat_conjugate",
    "quat_exp_map",
    "quat_log_map",
]


# --------------------------------------------------------------------------
# quaternion helpers (w, x, y, z)
# --------------------------------------------------------------------------
def quat_multiply(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array(
        [
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ],
        dtype=np.float64,
    )


def quat_conjugate(q: np.ndarray) -> np.ndarray:
    return np.array([q[0], -q[1], -q[2], -q[3]], dtype=np.float64)


def quat_exp_map(rotvec: np.ndarray) -> np.ndarray:
    """Exponential map: rotation vector (rad) -> unit quaternion (w, x, y, z)."""
    rotvec = np.asarray(rotvec, dtype=np.float64).reshape(3)
    theta = float(np.linalg.norm(rotvec))
    if theta < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0])
    axis = rotvec / theta
    half = 0.5 * theta
    return np.concatenate([[math.cos(half)], math.sin(half) * axis])


def quat_log_map(q: np.ndarray) -> np.ndarray:
    """Logarithm map: unit quaternion -> rotation vector (rad)."""
    q = np.asarray(q, dtype=np.float64).reshape(4)
    if q[0] < 0.0:
        q = -q
    w = float(np.clip(q[0], -1.0, 1.0))
    v = q[1:]
    vnorm = float(np.linalg.norm(v))
    if vnorm < 1e-12:
        return np.zeros(3)
    theta = 2.0 * math.atan2(vnorm, w)
    return v / vnorm * theta


# --------------------------------------------------------------------------
# One Euro filter
# --------------------------------------------------------------------------
@dataclass
class OneEuroFilter:
    """One Euro filter over the last axis of a fixed-size vector."""

    size: int
    freq_hz: float = 50.0
    min_cutoff: float = 1.5
    beta: float = 0.05
    d_cutoff: float = 1.0
    _x_prev: np.ndarray | None = field(default=None, repr=False)
    _dx_prev: np.ndarray | None = field(default=None, repr=False)

    def reset(self) -> None:
        self._x_prev = None
        self._dx_prev = None

    @staticmethod
    def _alpha(cutoff: float, freq: float) -> float:
        tau = 1.0 / (2.0 * math.pi * max(cutoff, 1e-6))
        te = 1.0 / max(freq, 1e-6)
        return 1.0 / (1.0 + tau / te)

    def filter(self, x: np.ndarray, freq_hz: float | None = None) -> np.ndarray:
        x = np.asarray(x, dtype=np.float64).reshape(self.size)
        freq = float(freq_hz) if freq_hz else float(self.freq_hz)
        if self._x_prev is None:
            self._x_prev = x.copy()
            self._dx_prev = np.zeros(self.size)
            return x.copy()
        dx = (x - self._x_prev) * freq
        a_d = self._alpha(self.d_cutoff, freq)
        dx_hat = a_d * dx + (1.0 - a_d) * self._dx_prev
        cutoff = self.min_cutoff + self.beta * np.abs(dx_hat)
        a = np.asarray([self._alpha(c, freq) for c in np.atleast_1d(cutoff)])
        x_hat = a * x + (1.0 - a) * self._x_prev
        self._x_prev = x_hat
        self._dx_prev = dx_hat
        return x_hat


# --------------------------------------------------------------------------
# configuration
# --------------------------------------------------------------------------
@dataclass
class PredictorConfig:
    joint_velocity_limit: np.ndarray
    joint_acceleration_limit: np.ndarray
    root_velocity_limit: float
    root_angular_velocity_limit: float
    filter_min_cutoff: float
    filter_beta: float
    filter_d_cutoff: float
    hold_after_ms: float
    invalid_after_ms: float
    max_source_gap_s: float
    raw: dict = field(default_factory=dict)

    @classmethod
    def from_yaml(cls, path: Path | str | None = None, limits: A3Limits | None = None) -> "PredictorConfig":
        path = Path(path).expanduser() if path else DEFAULT_CONFIG_PATH
        if not path.is_file():
            raise FileNotFoundError(
                f"predictor config not found: {path}. Run: python tools/make_predictor_config.py"
            )
        with open(path, "r", encoding="utf-8") as handle:
            doc = yaml.safe_load(handle)
        limits = limits or load_limits()
        n = limits.n_joints

        def vector(key: str, fallback: np.ndarray) -> np.ndarray:
            value = doc.get(key)
            if value is None:
                return fallback
            arr = np.asarray(value, dtype=np.float64).reshape(-1)
            if arr.size == 1:
                return np.full(n, float(arr[0]))
            if arr.size != n:
                raise ValueError(f"predictor config {key} must have 1 or {n} entries, got {arr.size}")
            return arr

        # joint velocity limits default to the *hardware* limits from the model
        velocity = vector("joint_velocity_limit", limits.velocity.copy())
        accel = vector("joint_acceleration_limit", velocity * 20.0)

        return cls(
            joint_velocity_limit=velocity,
            joint_acceleration_limit=accel,
            root_velocity_limit=float(doc.get("root_velocity_limit", 2.0)),
            root_angular_velocity_limit=float(doc.get("root_angular_velocity_limit", 4.0)),
            filter_min_cutoff=float(doc.get("filter_min_cutoff", 1.5)),
            filter_beta=float(doc.get("filter_beta", 0.05)),
            filter_d_cutoff=float(doc.get("filter_d_cutoff", 1.0)),
            hold_after_ms=float(doc.get("hold_after_ms", 50.0)),
            invalid_after_ms=float(doc.get("invalid_after_ms", 250.0)),
            max_source_gap_s=float(doc.get("max_source_gap_s", 0.5)),
            raw=doc,
        )


# --------------------------------------------------------------------------
# predictor
# --------------------------------------------------------------------------
class A3ReferencePredictor:
    """Turns a stream of :class:`A3CanonicalState` into reference windows."""

    def __init__(
        self,
        config: PredictorConfig | None = None,
        contract: A3Contract | None = None,
        limits: A3Limits | None = None,
    ) -> None:
        self.contract = contract or load_contract()
        self.limits = limits or load_limits()
        self.config = config or PredictorConfig.from_yaml(limits=self.limits)
        n = self.contract.n_policy_joints
        self._joint_filter = OneEuroFilter(
            size=n,
            freq_hz=self.contract.policy_hz,
            min_cutoff=self.config.filter_min_cutoff,
            beta=self.config.filter_beta,
            d_cutoff=self.config.filter_d_cutoff,
        )
        self._root_pos_filter = OneEuroFilter(
            size=3,
            freq_hz=self.contract.policy_hz,
            min_cutoff=self.config.filter_min_cutoff,
            beta=self.config.filter_beta,
            d_cutoff=self.config.filter_d_cutoff,
        )
        self._watchdog = StaleWatchdog(
            hold_after_ms=self.config.hold_after_ms,
            invalid_after_ms=self.config.invalid_after_ms,
        )
        self.reset()

    # ------------------------------------------------------------------
    def reset(self) -> None:
        self._joint_filter.reset()
        self._root_pos_filter.reset()
        self.last_state: A3CanonicalState | None = None
        self.last_safe: A3CanonicalState | None = None
        self.prev_state: A3CanonicalState | None = None
        self.joint_vel = np.zeros(self.contract.n_policy_joints)
        self.root_vel = np.zeros(3)
        self.root_omega = np.zeros(3)
        self.seq = 0
        self.state = BridgeState.DISCONNECTED
        self.last_reject_reason = ""

    # ------------------------------------------------------------------
    def push(self, state: A3CanonicalState) -> bool:
        """Feed one canonical state; returns True if it was accepted."""
        if not state.valid or not state.is_finite:
            self.last_reject_reason = state.reason or "invalid or non-finite state"
            return False

        dt = None
        if self.last_state is not None:
            dt = (state.timestamp_ns - self.last_state.timestamp_ns) / 1e9
            if not math.isfinite(dt) or dt <= 0.0 or dt > self.config.max_source_gap_s:
                self.last_reject_reason = f"bad dt {dt}"
                return False

        joint_pos = self._joint_filter.filter(state.joint_pos_rad)
        root_pos = self._root_pos_filter.filter(state.root_pos_m)
        quat = np.asarray(state.root_quat_wxyz, dtype=np.float64)
        quat = quat / max(float(np.linalg.norm(quat)), 1e-12)

        if dt is not None and self.last_safe is not None:
            dq = np.asarray(
                [
                    float(np.angle(np.exp(1j * (a - b))))
                    for a, b in zip(joint_pos, self.last_safe.joint_pos_rad)
                ]
            )
            vel = dq / dt
            self.joint_vel = np.clip(vel, -self.config.joint_velocity_limit, self.config.joint_velocity_limit)

            root_vel = (root_pos - self.last_safe.root_pos_m) / dt
            speed = float(np.linalg.norm(root_vel))
            if speed > self.config.root_velocity_limit:
                root_vel = root_vel * (self.config.root_velocity_limit / speed)
            self.root_vel = root_vel

            prev_quat = self.last_safe.root_quat_wxyz
            delta = quat_multiply(quat, quat_conjugate(prev_quat))
            omega = quat_log_map(delta) / dt
            omega_norm = float(np.linalg.norm(omega))
            if omega_norm > self.config.root_angular_velocity_limit:
                omega = omega * (self.config.root_angular_velocity_limit / omega_norm)
            self.root_omega = omega

        filtered = A3CanonicalState(
            seq=state.seq,
            timestamp_ns=state.timestamp_ns,
            root_pos_m=root_pos,
            root_quat_wxyz=quat,
            joint_pos_rad=joint_pos,
            joint_vel_rad_s=self.joint_vel.copy(),
            valid=True,
            solver_latency_ms=state.solver_latency_ms,
        )
        finite = filtered.is_finite
        if not finite:
            self.last_reject_reason = "filtered state is non-finite"
            return False

        self.prev_state = self.last_state
        self.last_state = filtered
        self.last_safe = filtered
        self.state = BridgeState.TRACKING
        self.last_reject_reason = ""
        return True

    # ------------------------------------------------------------------
    def window(
        self,
        timestamp_ns: int | None = None,
        source_age_ms: float = 0.0,
    ) -> A3ReferenceWindow:
        """Build the 10-slot window; HOLD/INVALID when the source is stale."""
        self.seq += 1
        if self.last_safe is None:
            self.state = BridgeState.DISCONNECTED
            return self._empty_window(timestamp_ns or 0, source_age_ms)

        classification = self._watchdog.classify(source_age_ms)
        if classification != "OK":
            self.state = BridgeState.HOLD if classification == "HOLD" else BridgeState.SAFE_STOP
            window = A3ReferenceWindow.hold(
                self.last_safe,
                seq=self.seq,
                timestamp_ns=timestamp_ns or self.last_safe.timestamp_ns,
                dt=self.contract.window_dt,
                source_age_ms=source_age_ms,
            )
            window.solver_latency_ms = self.last_safe.solver_latency_ms
            if classification == "INVALID":
                window.valid = False
                window.state = BridgeState.SAFE_STOP
            return window

        offsets = np.asarray(self.contract.window_offsets_s, dtype=np.float64)
        n_joints = self.contract.n_policy_joints
        joint_pos = np.empty((offsets.size, n_joints))
        joint_vel = np.empty((offsets.size, n_joints))
        root_pos = np.empty((offsets.size, 3))
        root_quat = np.empty((offsets.size, 4))

        base_joint = np.asarray(self.last_safe.joint_pos_rad)
        for i, h in enumerate(offsets):
            q = base_joint + self.joint_vel * h
            q = self.limits.clamp_position(q)
            joint_pos[i] = q
            joint_vel[i] = self.joint_vel
            root_pos[i] = self.last_safe.root_pos_m + self.root_vel * h
            root_quat[i] = quat_multiply(
                quat_exp_map(self.root_omega * h), self.last_safe.root_quat_wxyz
            )

        window = A3ReferenceWindow(
            seq=self.seq,
            timestamp_ns=timestamp_ns or self.last_safe.timestamp_ns,
            dt=self.contract.window_dt,
            root_pos_m=root_pos,
            root_quat_wxyz=root_quat,
            joint_pos_rad=joint_pos,
            joint_vel_rad_s=joint_vel,
            source_age_ms=source_age_ms,
            solver_latency_ms=self.last_safe.solver_latency_ms,
            valid=True,
            state=BridgeState.TRACKING,
        )
        return window

    # ------------------------------------------------------------------
    def _empty_window(self, timestamp_ns: int, source_age_ms: float) -> A3ReferenceWindow:
        n = self.contract.n_policy_joints
        frames = self.contract.window_frames
        window = A3ReferenceWindow(
            seq=self.seq,
            timestamp_ns=timestamp_ns,
            dt=self.contract.window_dt,
            root_pos_m=np.zeros((frames, 3)),
            root_quat_wxyz=np.tile(np.array([1.0, 0.0, 0.0, 0.0]), (frames, 1)),
            joint_pos_rad=np.tile(self.limits.default_angle, (frames, 1)),
            joint_vel_rad_s=np.zeros((frames, n)),
            source_age_ms=source_age_ms,
            valid=False,
            state=BridgeState.DISCONNECTED,
        )
        return window
