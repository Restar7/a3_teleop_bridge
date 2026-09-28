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


from . import paths


class ContractError(RuntimeError):
    """Raised when the contract file is missing, stale or inconsistent."""


@dataclass(frozen=True)
class A3Contract:
    """Typed view over ``generated/a3_contract.json``."""

    raw: dict
    path: Path
    _sonic_root: Path | None = None

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

    # ---- encoder (IsaacLab/URDF) joint order ----------------------------
    @property
    def il_joint_names(self) -> tuple[str, ...]:
        """Joint order the SONIC encoder consumes (``dof_il``).

        This is a genuine permutation of :attr:`policy_joint_names`; publishing
        raw policy-order joints without permuting them silently corrupts the
        policy input (found during M6).
        """
        return tuple(self.raw["il_joint_names"])

    @property
    def policy_to_il_index(self) -> tuple[int, ...]:
        """``policy_to_il_index[i]`` = position of policy joint *i* in il order."""
        return tuple(int(i) for i in self.raw["policy_to_il_index"])

    @property
    def il_to_policy_index(self) -> tuple[int, ...]:
        """``il_to_policy_index[k]`` = policy index of the *k*-th encoder joint."""
        return tuple(int(i) for i in self.raw["il_to_policy_index"])

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
    def sonic_root(self) -> Path:
        """Resolved on *this* machine; the recorded value is only a hint.

        ``generated/a3_contract.json`` is committed, so its ``sonic_root`` was
        written on whichever machine generated it.  Resolution order:
        ``SONIC_A3_ROOT`` -> recorded value (only if it exists here) -> sibling
        checkout -> error.  No developer path is ever hard-coded.
        """
        if self._sonic_root is None:
            object.__setattr__(
                self,
                "_sonic_root",
                paths.resolve_sonic_root(contract_value=self.raw.get("sonic_root")),
            )
        return self._sonic_root

    def _asset(self, key: str) -> Path:
        """Resolve an asset on this machine.

        The committed contract stores these paths *relative* to ``sonic_root``; a
        legacy/absolute value is rebased onto the resolved root instead, so a
        contract generated on another machine still works here.
        """
        assets = self.raw["assets"]
        stored = assets[key]
        candidate = Path(stored)
        if not candidate.is_absolute():
            resolved = self.sonic_root / candidate
            if resolved.exists():
                return resolved
            # fall through to the informational absolute field if present
            stored = assets.get(f"{key}_abs", stored)
        rebased = paths.rebase_asset(stored, self.raw.get("sonic_root"), self.sonic_root)
        assert rebased is not None  # keys are required by validate()
        return rebased

    @property
    def mjcf_path(self) -> Path:
        return self._asset("mjcf")

    @property
    def urdf_path(self) -> Path:
        return self._asset("urdf")

    @property
    def sample_csv_path(self) -> Path:
        return self._asset("sample_csv")

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
