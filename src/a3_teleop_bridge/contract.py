"""Load the machine-generated A3 contract.

``generated/a3_contract.json`` is produced by ``tools/inspect_a3_contract.py``
from the live sonic_for_a3 sources.  Every other module in this package MUST
read joint names, dimensions and timing from here -- never hard-code them
(plan sections 7 and 18).
"""

from __future__ import annotations

import functools
import json
import os
from dataclasses import dataclass
from pathlib import Path

BRIDGE_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONTRACT_PATH = BRIDGE_ROOT / "generated" / "a3_contract.json"


class ContractError(RuntimeError):
    """Raised when the contract file is missing, stale or inconsistent."""


@dataclass(frozen=True)
class A3Contract:
    """Typed view over ``generated/a3_contract.json``."""

    raw: dict
    path: Path

    # ---- joints ---------------------------------------------------------
    @property
    def policy_joint_names(self) -> tuple[str, ...]:
        return tuple(self.raw["policy_joint_names"])

    @property
    def csv_joint_names(self) -> tuple[str, ...]:
        return tuple(self.raw["csv_joint_names"])

    @property
    def head_joint_names(self) -> tuple[str, ...]:
        return tuple(self.raw["head_joint_names"])

    @property
    def passive_foot_joint_names(self) -> tuple[str, ...]:
        return tuple(self.raw["passive_foot_joint_names"])

    @property
    def n_policy_joints(self) -> int:
        return int(self.raw["policy_joint_count"])

    @property
    def policy_to_csv_index(self) -> tuple[int, ...]:
        return tuple(int(i) for i in self.raw["policy_to_csv_index"])

    # ---- timing ---------------------------------------------------------
    @property
    def policy_hz(self) -> float:
        return float(self.raw["timing"]["policy_hz"])

    @property
    def policy_dt(self) -> float:
        return float(self.raw["timing"]["policy_dt"])

    @property
    def reference_hz(self) -> float:
        """Rate of the reference stream the policy window is sampled from."""
        return float(self.raw["timing"]["reference_stream_hz"])

    # ---- window ---------------------------------------------------------
    @property
    def window_frames(self) -> int:
        return int(self.raw["reference_window"]["frame_count"])

    @property
    def window_frame_skip(self) -> int:
        return int(self.raw["reference_window"]["a3_fast_preset"]["frame_skip"])

    @property
    def window_dt(self) -> float:
        """Time between two consecutive window slots [s]."""
        return self.policy_dt * self.window_frame_skip

    @property
    def future_horizon_s(self) -> float:
        return float(self.raw["reference_window"]["future_horizon_s"])

    @property
    def window_offsets_s(self) -> tuple[float, ...]:
        """Offsets of each window slot relative to the current state [s]."""
        return tuple(i * self.window_dt for i in range(self.window_frames))

    # ---- dims -----------------------------------------------------------
    @property
    def action_dim(self) -> int:
        return int(self.raw["dims"]["action_dim"])

    @property
    def observation_dim(self) -> int:
        return int(self.raw["dims"]["observation_dim"])

    @property
    def encoder_frame_dim(self) -> int:
        return int(self.raw["dims"]["encoder_frame_dim"])

    @property
    def encoder_input_dim(self) -> int:
        return int(self.raw["dims"]["encoder_input_dim"])

    # ---- assets ---------------------------------------------------------
    @property
    def mjcf_path(self) -> Path:
        return Path(self.raw["assets"]["mjcf"])

    @property
    def urdf_path(self) -> Path:
        return Path(self.raw["assets"]["urdf"])

    @property
    def sample_csv_path(self) -> Path:
        return Path(self.raw["assets"]["sample_csv"])

    @property
    def sonic_root(self) -> Path:
        return Path(self.raw["sonic_root"])

    # ---- csv layout -----------------------------------------------------
    @property
    def csv_root_columns(self) -> tuple[str, ...]:
        return tuple(self.raw["csv_columns"]["root"])

    @property
    def csv_frame_column(self) -> str:
        return str(self.raw["csv_columns"]["frame"])

    def validate(self) -> None:
        """Cheap self-consistency checks; raises ContractError."""
        if len(self.policy_joint_names) != self.n_policy_joints:
            raise ContractError("policy_joint_names length != policy_joint_count")
        if len(set(self.policy_joint_names)) != self.n_policy_joints:
            raise ContractError("policy joint names are not unique")
        overlap = set(self.policy_joint_names) & set(self.head_joint_names)
        if overlap:
            raise ContractError(f"head joints leaked into the policy view: {sorted(overlap)}")
        overlap = set(self.policy_joint_names) & set(self.passive_foot_joint_names)
        if overlap:
            raise ContractError(f"passive joints leaked into the policy view: {sorted(overlap)}")
        if self.action_dim != self.n_policy_joints:
            raise ContractError("action_dim != number of policy joints")


def _contract_path(path: Path | str | None) -> Path:
    if path is not None:
        return Path(path).expanduser()
    env = os.environ.get("A3_CONTRACT_PATH")
    if env:
        return Path(env).expanduser()
    return DEFAULT_CONTRACT_PATH


@functools.lru_cache(maxsize=8)
def _load_cached(path_str: str, mtime_ns: int) -> A3Contract:
    del mtime_ns  # cache key only
    with open(path_str, "r", encoding="utf-8") as handle:
        raw = json.load(handle)
    contract = A3Contract(raw=raw, path=Path(path_str))
    contract.validate()
    return contract


def load_contract(path: Path | str | None = None) -> A3Contract:
    """Load (and validate) the A3 contract; cached per path+mtime."""
    resolved = _contract_path(path)
    if not resolved.is_file():
        raise ContractError(
            f"A3 contract not found at {resolved}. "
            "Run: python tools/inspect_a3_contract.py"
        )
    return _load_cached(str(resolved), resolved.stat().st_mtime_ns)


def policy_joint_names(path: Path | str | None = None) -> tuple[str, ...]:
    return load_contract(path).policy_joint_names
