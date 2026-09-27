"""Frozen internal data types for the whole bridge (plan sections 9 and 51).

These four structures are the *only* interfaces between stages:

    PICO --HumanSmplFrame--> UMR --A3CanonicalState--> predictor
         --A3ReferenceWindow--> SONIC A3-fast reference builder

Do not mix up A3 motor SDK order, UMR qpos order, the policy order and the
flat-CSV order: every conversion between them must go through an explicit
joint-name mapping (``a3/joint_map.py``).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any

import numpy as np

from . import contract as _contract

__all__ = [
    "BridgeState",
    "HumanSmplFrame",
    "A3CanonicalState",
    "A3ReferenceWindow",
    "SMPL_JOINT_COUNT",
    "SMPL_POSE_COUNT",
]

# SMPL body model shape used by the SONIC PICO streamer: 24 body joints and
# 21 body axis-angle entries (the root rotation is carried separately).
SMPL_JOINT_COUNT = 24
SMPL_POSE_COUNT = 21


class BridgeState(str, Enum):
    """Teleop state machine (plan section 73)."""

    DISCONNECTED = "DISCONNECTED"
    CALIBRATION = "CALIBRATION"
    READY = "READY"
    TRACKING = "TRACKING"
    HOLD = "HOLD"
    SAFE_STOP = "SAFE_STOP"


def _as_array(value: Any, shape: tuple[int, ...], name: str, dtype=np.float64) -> np.ndarray:
    arr = np.asarray(value, dtype=dtype)
    if arr.shape != shape:
        raise ValueError(f"{name}: expected shape {shape}, got {arr.shape}")
    return arr


@dataclass
class HumanSmplFrame:
    """One SMPL body frame coming from the PICO / XRoboToolkit streamer."""

    seq: int
    timestamp_ns: int

    smpl_joints: np.ndarray  # [24, 3] metres, world frame
    smpl_pose: np.ndarray  # [21, 3] axis-angle, body joints (root excluded)
    root_translation: np.ndarray  # [3] metres
    root_quat_wxyz: np.ndarray  # [4] w,x,y,z

    body_quat_w: np.ndarray | None = None  # [4] optional, from the streamer
    wrist_joint_pos: np.ndarray | None = None  # [6] optional

    # bookkeeping filled by the subscriber (plan section 23)
    receive_timestamp_ns: int = 0
    source_hz: float = 0.0

    def validate(self) -> "HumanSmplFrame":
        _as_array(self.smpl_joints, (SMPL_JOINT_COUNT, 3), "smpl_joints")
        _as_array(self.smpl_pose, (SMPL_POSE_COUNT, 3), "smpl_pose")
        _as_array(self.root_translation, (3,), "root_translation")
        _as_array(self.root_quat_wxyz, (4,), "root_quat_wxyz")
        if not np.isfinite(self.smpl_joints).all():
            raise ValueError("smpl_joints contains non-finite values")
        if not np.isfinite(self.smpl_pose).all():
            raise ValueError("smpl_pose contains non-finite values")
        if not np.isfinite(self.root_translation).all():
            raise ValueError("root_translation contains non-finite values")
        if self.body_quat_w is not None:
            _as_array(self.body_quat_w, (4,), "body_quat_w")
        if self.wrist_joint_pos is not None:
            _as_array(self.wrist_joint_pos, (6,), "wrist_joint_pos")
        return self

    def quat_norm(self) -> float:
        return float(np.linalg.norm(self.root_quat_wxyz))


@dataclass
class A3CanonicalState:
    """A3 whole-body reference state in the policy (29-DoF) joint view."""

    seq: int
    timestamp_ns: int

    root_pos_m: np.ndarray  # [3] metres, world frame
    root_quat_wxyz: np.ndarray  # [4] w,x,y,z
    joint_pos_rad: np.ndarray  # [29] policy order
    joint_vel_rad_s: np.ndarray  # [29] policy order

    valid: bool = True
    solver_latency_ms: float = 0.0

    # provenance / diagnostics
    solve_start_ns: int = 0
    solve_end_ns: int = 0
    solver_iterations: int = 0
    solver_cost: float = float("nan")
    constraint_violation: float = float("nan")
    dropped: bool = False
    reason: str = ""

    def __post_init__(self) -> None:
        n = 29
        try:
            n = len(self.joint_pos_rad)
        except TypeError:  # pragma: no cover - defensive
            n = 29
        self.joint_pos_rad = _as_array(self.joint_pos_rad, (n,), "joint_pos_rad")
        self.joint_vel_rad_s = _as_array(self.joint_vel_rad_s, (n,), "joint_vel_rad_s")
        self.root_pos_m = _as_array(self.root_pos_m, (3,), "root_pos_m")
        self.root_quat_wxyz = _as_array(self.root_quat_wxyz, (4,), "root_quat_wxyz")

    def validate(self, n_joints: int | None = None, check_finite: bool = True) -> "A3CanonicalState":
        expected = n_joints or _contract.load_contract().n_policy_joints
        _as_array(self.joint_pos_rad, (expected,), "joint_pos_rad")
        _as_array(self.joint_vel_rad_s, (expected,), "joint_vel_rad_s")
        if check_finite:
            for name in ("root_pos_m", "root_quat_wxyz", "joint_pos_rad", "joint_vel_rad_s"):
                if not np.isfinite(getattr(self, name)).all():
                    raise ValueError(f"{name} contains non-finite values")
        return self

    @property
    def is_finite(self) -> bool:
        return bool(
            np.isfinite(self.root_pos_m).all()
            and np.isfinite(self.root_quat_wxyz).all()
            and np.isfinite(self.joint_pos_rad).all()
            and np.isfinite(self.joint_vel_rad_s).all()
        )

    @classmethod
    def invalid(
        cls,
        seq: int,
        timestamp_ns: int,
        n_joints: int | None = None,
        reason: str = "",
    ) -> "A3CanonicalState":
        n = n_joints or _contract.load_contract().n_policy_joints
        return cls(
            seq=seq,
            timestamp_ns=timestamp_ns,
            root_pos_m=np.zeros(3),
            root_quat_wxyz=np.array([1.0, 0.0, 0.0, 0.0]),
            joint_pos_rad=np.zeros(n),
            joint_vel_rad_s=np.zeros(n),
            valid=False,
            reason=reason,
        )


@dataclass
class A3ReferenceWindow:
    """The 10-slot future reference window handed to the A3-fast encoder."""

    seq: int
    timestamp_ns: int

    dt: float  # spacing between two slots [s]

    root_pos_m: np.ndarray  # [10, 3]
    root_quat_wxyz: np.ndarray  # [10, 4] w,x,y,z
    joint_pos_rad: np.ndarray  # [10, 29]
    joint_vel_rad_s: np.ndarray  # [10, 29]

    source_age_ms: float = 0.0
    solver_latency_ms: float = 0.0
    valid: bool = True
    state: BridgeState = BridgeState.TRACKING
    prediction_ms: float = 0.0
    extra: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        frames = None
        try:
            frames = int(np.asarray(self.joint_pos_rad).shape[0])
        except Exception:  # pragma: no cover - defensive
            frames = None
        if frames is None:
            return
        n = int(np.asarray(self.joint_pos_rad).shape[1]) if np.asarray(self.joint_pos_rad).ndim == 2 else 29
        self.joint_pos_rad = _as_array(self.joint_pos_rad, (frames, n), "joint_pos_rad")
        self.joint_vel_rad_s = _as_array(self.joint_vel_rad_s, (frames, n), "joint_vel_rad_s")
        self.root_pos_m = _as_array(self.root_pos_m, (frames, 3), "root_pos_m")
        self.root_quat_wxyz = _as_array(self.root_quat_wxyz, (frames, 4), "root_quat_wxyz")

    @property
    def frames(self) -> int:
        return int(self.joint_pos_rad.shape[0])

    @property
    def n_joints(self) -> int:
        return int(self.joint_pos_rad.shape[1])

    @property
    def horizon_s(self) -> float:
        return self.dt * max(self.frames - 1, 0)

    def validate(self, n_joints: int | None = None) -> "A3ReferenceWindow":
        contract = _contract.load_contract()
        expected_joints = n_joints or contract.n_policy_joints
        expected_frames = contract.window_frames
        _as_array(self.joint_pos_rad, (expected_frames, expected_joints), "joint_pos_rad")
        _as_array(self.joint_vel_rad_s, (expected_frames, expected_joints), "joint_vel_rad_s")
        _as_array(self.root_pos_m, (expected_frames, 3), "root_pos_m")
        _as_array(self.root_quat_wxyz, (expected_frames, 4), "root_quat_wxyz")
        return self

    @property
    def is_finite(self) -> bool:
        return bool(
            np.isfinite(self.root_pos_m).all()
            and np.isfinite(self.root_quat_wxyz).all()
            and np.isfinite(self.joint_pos_rad).all()
            and np.isfinite(self.joint_vel_rad_s).all()
        )

    @classmethod
    def hold(
        cls,
        state: A3CanonicalState,
        seq: int,
        timestamp_ns: int,
        dt: float,
        frames: int | None = None,
        source_age_ms: float = 0.0,
    ) -> "A3ReferenceWindow":
        """Repeat ``state`` across the whole window, velocities zeroed.

        This is the HOLD behaviour of plan section 36: never extrapolate a
        stale, failed or non-finite reference -- freeze it instead.
        """
        contract = _contract.load_contract()
        n_frames = frames or contract.window_frames
        root_pos = np.tile(np.asarray(state.root_pos_m, dtype=np.float64), (n_frames, 1))
        root_quat = np.tile(np.asarray(state.root_quat_wxyz, dtype=np.float64), (n_frames, 1))
        joint_pos = np.tile(np.asarray(state.joint_pos_rad, dtype=np.float64), (n_frames, 1))
        joint_vel = np.zeros_like(joint_pos)
        return cls(
            seq=seq,
            timestamp_ns=timestamp_ns,
            dt=dt,
            root_pos_m=root_pos,
            root_quat_wxyz=root_quat,
            joint_pos_rad=joint_pos,
            joint_vel_rad_s=joint_vel,
            source_age_ms=source_age_ms,
            valid=False,
            state=BridgeState.HOLD,
        )
