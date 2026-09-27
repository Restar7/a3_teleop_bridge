"""UMR qpos -> :class:`A3CanonicalState` (plan section 19).

Pipeline::

    UMR qpos
      -> root position / root orientation (free joint)
      -> joint-name mapping to the 29-DoF policy view
      -> joint-limit clamp
      -> finite-difference velocities
      -> A3CanonicalState

Velocities are ``v_t = (q_t - q_{t-1}) / dt`` with angle unwrapping; frames with
``dt <= 0`` or ``dt > timeout`` are marked invalid instead of being silently
extrapolated (plan sections 19 and 36).
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ..a3.joint_map import A3JointMap, load_joint_map
from ..a3.limits import A3Limits, load_limits
from ..contract import A3Contract, load_contract
from ..types import A3CanonicalState

__all__ = ["UmrStateConverter", "quaternion_continuity"]


def quaternion_continuity(quat: np.ndarray, previous: np.ndarray | None = None) -> np.ndarray:
    """Normalise ``quat`` and flip its sign if it left the previous hemisphere."""
    quat = np.asarray(quat, dtype=np.float64).reshape(4)
    norm = float(np.linalg.norm(quat))
    if norm < 1e-12:
        raise ValueError("degenerate quaternion (norm ~ 0)")
    quat = quat / norm
    if previous is not None and float(np.dot(np.asarray(previous).reshape(4), quat)) < 0.0:
        quat = -quat
    return quat


def unwrap_delta(delta: float) -> float:
    """Wrap an angle difference into (-pi, pi]."""
    return float(np.angle(np.exp(1j * float(delta))))


@dataclass
class UmrStateConverter:
    """Converts UMR qpos frames into policy-view canonical states."""

    joint_map: A3JointMap | None = None
    limits: A3Limits | None = None
    contract: A3Contract | None = None
    clamp_positions: bool = True
    clamp_velocities: bool = True
    max_dt_s: float = 0.5

    def __post_init__(self) -> None:
        self.contract = self.contract or load_contract()
        self.joint_map = self.joint_map or load_joint_map()
        self.limits = self.limits or load_limits()

    # ------------------------------------------------------------------
    @property
    def n_joints(self) -> int:
        return self.contract.n_policy_joints

    def policy_joint_values(self, joint_values: dict[str, float]) -> tuple[np.ndarray, int]:
        """Resolve a ``{joint_name: value}`` mapping onto the policy view."""
        names = list(joint_values.keys())
        values = np.asarray([joint_values[n] for n in names], dtype=np.float64)
        mapped = self.joint_map.map_vector(values, names)
        clamped = 0
        if self.clamp_positions:
            before = mapped.copy()
            mapped = self.limits.clamp_position(mapped)
            clamped = int(np.count_nonzero(np.abs(mapped - before) > 1e-12))
        return mapped, clamped

    def convert_frame(
        self,
        joint_values: dict[str, float],
        seq: int,
        timestamp_ns: int,
        root_pos_m: np.ndarray,
        root_quat_wxyz: np.ndarray,
        previous: A3CanonicalState | None = None,
        dt: float | None = None,
        solver_latency_ms: float = 0.0,
    ) -> A3CanonicalState:
        """Convert one frame; ``dt`` enables finite-difference velocity."""
        root_pos = np.asarray(root_pos_m, dtype=np.float64).reshape(3)
        if not np.isfinite(root_pos).all():
            return A3CanonicalState.invalid(seq, timestamp_ns, self.n_joints, "non-finite root position")

        previous_quat = previous.root_quat_wxyz if previous is not None else None
        try:
            root_quat = quaternion_continuity(root_quat_wxyz, previous_quat)
        except ValueError as exc:
            return A3CanonicalState.invalid(seq, timestamp_ns, self.n_joints, f"bad quaternion: {exc}")

        try:
            joint_pos, clamped = self.policy_joint_values(joint_values)
        except Exception as exc:  # JointMapError and friends
            return A3CanonicalState.invalid(seq, timestamp_ns, self.n_joints, f"joint map failed: {exc}")

        if not np.isfinite(joint_pos).all():
            return A3CanonicalState.invalid(seq, timestamp_ns, self.n_joints, "non-finite joint position")

        joint_vel = np.zeros(self.n_joints, dtype=np.float64)
        valid = True
        reason = ""
        if previous is not None and dt is not None:
            if not np.isfinite(dt) or dt <= 0.0:
                valid, reason = False, f"invalid dt {dt}"
            elif dt > self.max_dt_s:
                valid, reason = False, f"dt {dt:.4f}s exceeds timeout {self.max_dt_s}s"
            else:
                deltas = np.asarray(
                    [unwrap_delta(a - b) for a, b in zip(joint_pos, previous.joint_pos_rad)],
                    dtype=np.float64,
                )
                joint_vel = deltas / dt
                if self.clamp_velocities:
                    joint_vel = self.limits.clamp_velocity(joint_vel)

        state = A3CanonicalState(
            seq=seq,
            timestamp_ns=timestamp_ns,
            root_pos_m=root_pos,
            root_quat_wxyz=root_quat,
            joint_pos_rad=joint_pos,
            joint_vel_rad_s=joint_vel,
            valid=valid,
            solver_latency_ms=solver_latency_ms,
            reason=reason,
        )
        if clamped:
            state.reason = (state.reason + f"; clamped {clamped} joints").strip("; ")
        return state

    # ------------------------------------------------------------------
    def convert_sequence(
        self,
        frames,
        dt: float | None = None,
        start_timestamp_ns: int = 0,
    ) -> list[A3CanonicalState]:
        """Convert an iterable of ``(joint_values, root_pos, root_quat)`` frames."""
        states: list[A3CanonicalState] = []
        previous: A3CanonicalState | None = None
        for seq, (joint_values, root_pos, root_quat) in enumerate(frames):
            timestamp_ns = start_timestamp_ns + int(round(seq * (dt or 0.0) * 1e9))
            state = self.convert_frame(
                joint_values,
                seq=seq,
                timestamp_ns=timestamp_ns,
                root_pos_m=root_pos,
                root_quat_wxyz=root_quat,
                previous=previous,
                dt=dt,
            )
            states.append(state)
            if state.valid:
                previous = state
        return states

    def convert_umr_result(self, result, start_timestamp_ns: int = 0) -> list[A3CanonicalState]:
        """Convert a loaded :class:`~a3_teleop_bridge.umr.offline.UmrResult`."""
        dt = result.dt
        frames = (
            (result.frame_joint_dict(i), result.root_pos[i], result.root_quat_wxyz[i])
            for i in range(result.n_frames)
        )
        return self.convert_sequence(frames, dt=dt, start_timestamp_ns=start_timestamp_ns)
