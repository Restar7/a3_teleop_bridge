"""A3 joint limits / gains, loaded from ``generated/a3_joint_limits.yaml``.

The YAML is produced by ``tools/extract_a3_joint_limits.py`` from the live MJCF,
URDF and deploy policy-parameter header.  Nothing here may be filled in by hand
(plan sections 16 and 19).
"""

from __future__ import annotations

import functools
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml

from ..contract import load_contract

BRIDGE_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_LIMITS_PATH = BRIDGE_ROOT / "generated" / "a3_joint_limits.yaml"

__all__ = ["A3Limits", "load_limits", "clamp_to_limits"]


@dataclass(frozen=True)
class A3Limits:
    joint_names: tuple[str, ...]
    lower: np.ndarray  # [29]
    upper: np.ndarray  # [29]
    velocity: np.ndarray  # [29]
    effort: np.ndarray  # [29]
    default_angle: np.ndarray  # [29]
    kp: np.ndarray  # [29]
    kd: np.ndarray  # [29]
    raw: dict
    path: Path

    @property
    def n_joints(self) -> int:
        return len(self.joint_names)

    def clamp_position(self, q: np.ndarray) -> np.ndarray:
        return np.clip(np.asarray(q, dtype=np.float64), self.lower, self.upper)

    def clamp_velocity(self, dq: np.ndarray) -> np.ndarray:
        return np.clip(np.asarray(dq, dtype=np.float64), -self.velocity, self.velocity)

    def exceeds(self, q: np.ndarray, tol: float = 0.0) -> np.ndarray:
        """Boolean mask of joints outside [lower - tol, upper + tol]."""
        q = np.asarray(q, dtype=np.float64)
        return (q < self.lower - tol) | (q > self.upper + tol)

    def as_dict(self) -> dict:
        return {
            "joint_names": list(self.joint_names),
            "lower": self.lower.tolist(),
            "upper": self.upper.tolist(),
            "velocity": self.velocity.tolist(),
            "effort": self.effort.tolist(),
            "default_angle": self.default_angle.tolist(),
            "kp": self.kp.tolist(),
            "kd": self.kd.tolist(),
        }


@functools.lru_cache(maxsize=4)
def _load_cached(path_str: str, mtime_ns: int) -> A3Limits:
    del mtime_ns
    with open(path_str, "r", encoding="utf-8") as handle:
        doc = yaml.safe_load(handle)
    contract = load_contract()
    policy_names = contract.policy_joint_names
    if list(doc["policy_joint_order"]) != list(policy_names):
        raise ValueError("a3_joint_limits.yaml joint order does not match the A3 contract")
    joints = doc["joints"]
    lower, upper, velocity, effort, default, kp, kd = ([] for _ in range(7))
    for name in policy_names:
        entry = joints[name]
        lower.append(float(entry["position_lower"]))
        upper.append(float(entry["position_upper"]))
        velocity.append(float(entry["velocity_limit"]))
        effort.append(float(entry["effort_limit"]))
        default.append(float(entry["default_angle"]))
        kp.append(float(entry["kp"]))
        kd.append(float(entry["kd"]))
    limits = A3Limits(
        joint_names=tuple(policy_names),
        lower=np.asarray(lower, dtype=np.float64),
        upper=np.asarray(upper, dtype=np.float64),
        velocity=np.asarray(velocity, dtype=np.float64),
        effort=np.asarray(effort, dtype=np.float64),
        default_angle=np.asarray(default, dtype=np.float64),
        kp=np.asarray(kp, dtype=np.float64),
        kd=np.asarray(kd, dtype=np.float64),
        raw=doc,
        path=Path(path_str),
    )
    if np.any(limits.lower > limits.upper):
        raise ValueError("inverted joint limits in generated/a3_joint_limits.yaml")
    if np.any(limits.velocity <= 0):
        raise ValueError("non-positive joint velocity limits")
    return limits


def load_limits(path: Path | str | None = None) -> A3Limits:
    resolved = Path(path).expanduser() if path else DEFAULT_LIMITS_PATH
    if not resolved.is_file():
        raise FileNotFoundError(
            f"A3 joint limits not found at {resolved}. "
            "Run: python tools/extract_a3_joint_limits.py"
        )
    return _load_cached(str(resolved), resolved.stat().st_mtime_ns)


def clamp_to_limits(
    q: np.ndarray,
    limits: A3Limits | None = None,
    margin_rad: float = 0.0,
) -> tuple[np.ndarray, int]:
    """Clamp a 29-DoF vector; returns ``(clamped, number_of_clamped_joints)``."""
    limits = limits or load_limits()
    q = np.asarray(q, dtype=np.float64)
    lower = limits.lower + margin_rad
    upper = limits.upper - margin_rad
    clamped = np.clip(q, lower, upper)
    changed = int(np.count_nonzero(np.abs(clamped - q) > 1e-12))
    return clamped, changed
