"""Retarget backends for the online pipeline (plan sections 28-30).

The pipeline itself (:mod:`a3_teleop_bridge.umr.online`) only needs something
that implements ``reset()`` / ``step(frame, timestamp_ns)``.  Two backends ship
here:

``TrajectoryReplayBackend``
    Walks a pre-computed A3 trajectory (a UMR result or a flat CSV) one frame per
    call.  It is the hardware-free way to exercise the live plumbing end to end:
    receiver thread -> solver thread -> predictor thread -> publisher, with the
    real latest-only queues and the real state machine.

``UmrOnlineBackend``
    The real online UMR driver.  It loads the SMPL-X model, the A3 MJCF, the
    cached correspondence slots and the per-frame solver state **once** (plan
    section 29), then solves each incoming frame with the previous solution as the
    warm start (plan section 30).  It runs inside the UMR virtualenv, so it is
    imported lazily; ``available()`` reports whether that environment is usable.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from ..a3.csv_export import A3FlatCsvCodec
from ..a3.limits import load_limits
from ..contract import load_contract
from ..types import A3CanonicalState, HumanSmplFrame
from .offline import load_umr_result
from .state_converter import UmrStateConverter

__all__ = ["TrajectoryReplayBackend", "UmrOnlineBackend", "OnlineBackendError"]


class OnlineBackendError(RuntimeError):
    """Raised when a backend cannot be initialised."""


@dataclass
class TrajectoryReplayBackend:
    """Replay a pre-computed A3 trajectory through the live pipeline."""

    umr_result: Path | str | None = None
    csv_path: Path | str | None = None
    csv_fps: float = 30.0
    source: str = ""
    _states: list = field(default_factory=list, repr=False)
    _index: int = 0
    _position = None
    _converter = None

    def __post_init__(self) -> None:
        if self.umr_result is None and self.csv_path is None:
            raise OnlineBackendError("give either umr_result or csv_path")
        if self.umr_result is not None:
            result = load_umr_result(self.umr_result)
            self._converter = UmrStateConverter()
            self._states = self._converter.convert_umr_result(result)
            self.source = str(result.path)
        else:
            codec = A3FlatCsvCodec()
            data = codec.read(Path(self.csv_path).expanduser(), source_fps=self.csv_fps, frame_stride=1)
            contract = load_contract()
            dt = 1.0 / data.effective_fps
            states = []
            for i in range(data.n_frames):
                velocity = (
                    (data.dof29_rad[i] - data.dof29_rad[i - 1]) / dt if i else np.zeros(contract.n_policy_joints)
                )
                states.append(
                    A3CanonicalState(
                        seq=i,
                        timestamp_ns=int(round(i * dt * 1e9)),
                        root_pos_m=data.root_pos_m[i],
                        root_quat_wxyz=data.root_quat_wxyz[i],
                        joint_pos_rad=data.dof29_rad[i],
                        joint_vel_rad_s=velocity,
                        valid=True,
                    )
                )
            self._states = states
            self.source = str(data.path)

    def reset(self, initial_frame: HumanSmplFrame | None = None) -> None:
        self._index = 0

    def step(self, frame: HumanSmplFrame, timestamp_ns: int) -> A3CanonicalState:
        if not self._states:
            raise OnlineBackendError("the trajectory is empty")
        template = self._states[self._index % len(self._states)]
        self._index += 1
        # re-stamp onto the live clock while keeping the trajectory content
        state = A3CanonicalState(
            seq=self._index,
            timestamp_ns=timestamp_ns,
            root_pos_m=np.asarray(template.root_pos_m).copy(),
            root_quat_wxyz=np.asarray(template.root_quat_wxyz).copy(),
            joint_pos_rad=np.asarray(template.joint_pos_rad).copy(),
            joint_vel_rad_s=np.asarray(template.joint_vel_rad_s).copy(),
            valid=True,
        )
        return state

    @property
    def frames(self) -> int:
        return len(self._states)


class UmrOnlineBackend:
    """Per-frame UMR retargeting with everything heavy loaded once.

    ``initialize()`` performs the one-off setup the plan calls Stage I
    (correspondence + solver + CUDA warm-up); ``step()`` then only updates the
    pose, solves and returns the new state (Stage II).  The implementation
    delegates to the UMR retargeting module rather than re-deriving it:

    * ``humanoid_retarget_config.load_config`` for the robot/motion config,
    * ``retarget_smpl_to_humanoid_surface_vector.fill_args_from_config`` for the
      solver arguments,
    * ``smpl_surface_retarget_common`` for the source-side geometry,
    * ``solve_frame_body_segment_qp`` for the per-frame solve.
    """

    def __init__(
        self,
        robot_config: Path | str | None = None,
        umr_root: Path | str | None = None,
        python: Path | str | None = None,
        device: str = "cuda",
        verbose: bool = False,
        dump_frames: Path | str | None = None,
    ) -> None:
        self.dump_frames = Path(dump_frames).expanduser() if dump_frames else None
        self.robot_config = Path(robot_config).expanduser() if robot_config else None
        self.umr_root = Path(umr_root).expanduser() if umr_root else None
        self.python = Path(python).expanduser() if python else None
        self.device = device
        self.verbose = verbose
        self.session = None
        self.converter = UmrStateConverter()
        self._previous: A3CanonicalState | None = None
        self._stats = {"solves": 0, "total_ms": 0.0, "iterations": 0}

    # ------------------------------------------------------------------
    @staticmethod
    def available(umr_root: Path | str | None = None) -> bool:
        from .. import paths

        root = Path(umr_root).expanduser() if umr_root else paths.resolve_umr_root()
        return (root / "scripts" / "retarget_smpl_to_humanoid_surface_vector.py").is_file()

    def initialize(self, robot_config: Path | str | None = None) -> None:
        """Load SMPL-X + MJCF + correspondence + solver once (Stage I)."""
        from .umr_session import UmrRetargetSession  # local import: uses the UMR venv

        config = robot_config or self.robot_config
        self.session = UmrRetargetSession(
            robot_config=config,
            umr_root=self.umr_root,
            device=self.device,
            verbose=self.verbose,
            dump_frames=self.dump_frames,
        )
        self.session.initialize()

    def reset(self, initial_frame: HumanSmplFrame | None = None) -> None:
        self._previous = None
        if self.session is not None:
            self.session.reset(initial_frame)

    def step(self, frame: HumanSmplFrame, timestamp_ns: int) -> A3CanonicalState:
        if self.session is None:
            raise OnlineBackendError("call initialize() before step()")
        started = time.perf_counter()
        qpos, info = self.session.solve_frame(frame)
        elapsed_ms = (time.perf_counter() - started) * 1e3
        self._stats["solves"] += 1
        self._stats["total_ms"] += elapsed_ms

        joint_values = self.session.joint_values(qpos)
        state = self.converter.convert_frame(
            joint_values,
            seq=self._stats["solves"],
            timestamp_ns=timestamp_ns,
            root_pos_m=self.session.root_position(qpos),
            root_quat_wxyz=self.session.root_quaternion(qpos),
            previous=self._previous,
            dt=info.get("dt", 0.02),
            solver_latency_ms=elapsed_ms,
        )
        state.solver_iterations = int(info.get("iterations", 0))
        state.solver_cost = float(info.get("cost", float("nan")))
        state.constraint_violation = float(info.get("violation", float("nan")))
        if state.valid:
            self._previous = state
        return state

    # ------------------------------------------------------------------
    def stats(self) -> dict:
        solves = max(self._stats["solves"], 1)
        return {
            "solves": self._stats["solves"],
            "mean_solver_ms": self._stats["total_ms"] / solves,
        }
