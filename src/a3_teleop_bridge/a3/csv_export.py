"""A3 flat-CSV codec matching ``sim2sim_a3_mujoco.py`` exactly.

Semantics were read out of ``load_a3_flat_csv`` in the live sonic_for_a3 source
(plan section 20 -- do not trust older documentation):

  * ``root_translateX/Y/Z`` are **centimetres** (multiplied by 0.01 -> metres)
  * ``root_rotateX/Y/Z`` are **degrees** and form an *extrinsic* XYZ Euler
    sequence (``scipy.spatial.transform.Rotation.from_euler("xyz", deg, degrees=True)``)
  * joint columns are **degrees**, ordered like ``A3_CSV_JOINT_NAMES`` (31 joints
    including the two head joints); the 29-DoF policy view selects
    ``A3_POLICY_TO_SDK_IDX``
  * joint columns may be spelled ``<joint>`` or ``<joint>_dof`` (exactly one)
  * a leading ``Frame`` column carries the frame index
"""

from __future__ import annotations

import csv
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from ..contract import A3Contract, load_contract

__all__ = ["A3CsvData", "read_a3_flat_csv", "write_a3_flat_csv", "A3FlatCsvCodec"]

DEFAULT_SOURCE_FPS = 30.0
DEFAULT_FRAME_STRIDE = 4


@dataclass
class A3CsvData:
    root_pos_m: np.ndarray  # [T, 3]
    root_quat_wxyz: np.ndarray  # [T, 4]
    dof29_rad: np.ndarray  # [T, 29] policy order
    frame_ids: np.ndarray  # [T]
    raw_row_count: int
    path: Path | None = None
    source_fps: float = DEFAULT_SOURCE_FPS
    frame_stride: int = DEFAULT_FRAME_STRIDE

    @property
    def n_frames(self) -> int:
        return int(self.root_pos_m.shape[0])

    @property
    def effective_fps(self) -> float:
        return self.source_fps / float(self.frame_stride)

    @property
    def duration_s(self) -> float:
        return self.n_frames / self.effective_fps if self.effective_fps > 0 else float("nan")


def _joint_column_map(fieldnames: list[str], csv_joint_names: tuple[str, ...], path) -> dict[str, str]:
    fields = set(fieldnames)
    resolved: dict[str, str] = {}
    missing: list[str] = []
    ambiguous: list[str] = []
    for name in csv_joint_names:
        bare = name in fields
        alias = f"{name}_dof" in fields
        if bare and alias:
            ambiguous.append(name)
        elif bare:
            resolved[name] = name
        elif alias:
            resolved[name] = alias
        else:
            missing.append(name)
    if ambiguous:
        raise ValueError(f"{path}: ambiguous joint columns (both spellings present): {ambiguous}")
    if missing:
        raise ValueError(f"{path}: missing joint columns: {missing}")
    return resolved


def read_a3_flat_csv(
    path: Path | str,
    source_fps: float = DEFAULT_SOURCE_FPS,
    frame_stride: int = DEFAULT_FRAME_STRIDE,
    contract: A3Contract | None = None,
) -> A3CsvData:
    """Read a flat A3 CSV with the same conventions as the official loader."""
    contract = contract or load_contract()
    path = Path(path).expanduser()
    if source_fps <= 0.0:
        raise ValueError("source_fps must be positive")
    if frame_stride < 1:
        raise ValueError("frame_stride must be >= 1")

    rows: list[dict[str, str]] = []
    frame_ids: list[int] = []
    raw_row_count = 0
    with path.open(newline="") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames is None:
            raise ValueError(f"empty CSV: {path}")
        missing_root = [c for c in contract.csv_root_columns if c not in reader.fieldnames]
        if missing_root:
            raise ValueError(f"{path}: missing root columns {missing_root}")
        joint_columns = _joint_column_map(list(reader.fieldnames), contract.csv_joint_names, path)
        for raw_idx, row in enumerate(reader):
            raw_row_count += 1
            if raw_idx % frame_stride != 0:
                continue
            rows.append(row)
            frame_raw = row.get(contract.csv_frame_column, raw_idx)
            try:
                frame_ids.append(int(float(frame_raw)))
            except (TypeError, ValueError):
                frame_ids.append(raw_idx)
    if not rows:
        raise ValueError(f"{path}: no rows selected with stride {frame_stride}")

    root_pos = (
        np.asarray(
            [[float(r["root_translateX"]), float(r["root_translateY"]), float(r["root_translateZ"])] for r in rows],
            dtype=np.float64,
        )
        * 0.01
    )
    euler_deg = np.asarray(
        [[float(r["root_rotateX"]), float(r["root_rotateY"]), float(r["root_rotateZ"])] for r in rows],
        dtype=np.float64,
    )
    quat_xyzw = Rotation.from_euler("xyz", euler_deg, degrees=True).as_quat()
    root_quat_wxyz = quat_xyzw[:, [3, 0, 1, 2]]

    csv_names = contract.csv_joint_names
    policy_to_csv = contract.policy_to_csv_index
    dof_deg = np.asarray(
        [
            [float(row[joint_columns[csv_names[sdk_idx]]]) for sdk_idx in policy_to_csv]
            for row in rows
        ],
        dtype=np.float64,
    )
    return A3CsvData(
        root_pos_m=root_pos,
        root_quat_wxyz=root_quat_wxyz,
        dof29_rad=np.deg2rad(dof_deg),
        frame_ids=np.asarray(frame_ids, dtype=np.int64),
        raw_row_count=raw_row_count,
        path=path,
        source_fps=float(source_fps),
        frame_stride=int(frame_stride),
    )


def write_a3_flat_csv(
    path: Path | str,
    root_pos_m: np.ndarray,
    root_quat_wxyz: np.ndarray,
    dof29_rad: np.ndarray,
    *,
    head_dof_rad: np.ndarray | None = None,
    frame_ids: np.ndarray | None = None,
    contract: A3Contract | None = None,
    float_format: str = "%.9g",
) -> Path:
    """Write a flat A3 CSV; head joints default to neutral (0 rad).

    The inverse of :func:`read_a3_flat_csv`: metres -> centimetres, quaternion ->
    extrinsic XYZ Euler degrees, radians -> degrees.
    """
    contract = contract or load_contract()
    path = Path(path).expanduser()
    path.parent.mkdir(parents=True, exist_ok=True)

    root_pos_m = np.asarray(root_pos_m, dtype=np.float64)
    root_quat_wxyz = np.asarray(root_quat_wxyz, dtype=np.float64)
    dof29_rad = np.asarray(dof29_rad, dtype=np.float64)
    if root_pos_m.ndim != 2 or root_pos_m.shape[1] != 3:
        raise ValueError(f"root_pos_m must be [T, 3], got {root_pos_m.shape}")
    if root_quat_wxyz.shape != (root_pos_m.shape[0], 4):
        raise ValueError(f"root_quat_wxyz must be [T, 4], got {root_quat_wxyz.shape}")
    if dof29_rad.shape != (root_pos_m.shape[0], contract.n_policy_joints):
        raise ValueError(
            f"dof29_rad must be [T, {contract.n_policy_joints}], got {dof29_rad.shape}"
        )

    n_frames = root_pos_m.shape[0]
    if frame_ids is None:
        frame_ids = np.arange(n_frames, dtype=np.int64)
    frame_ids = np.asarray(frame_ids).reshape(-1)
    if frame_ids.size != n_frames:
        raise ValueError("frame_ids length must match the number of frames")

    csv_names = contract.csv_joint_names
    policy_to_csv = contract.policy_to_csv_index
    dof31 = np.zeros((n_frames, len(csv_names)), dtype=np.float64)
    for policy_idx, csv_idx in enumerate(policy_to_csv):
        dof31[:, csv_idx] = np.rad2deg(dof29_rad[:, policy_idx])
    if head_dof_rad is not None:
        head_dof_rad = np.asarray(head_dof_rad, dtype=np.float64)
        if head_dof_rad.shape != (n_frames, len(contract.head_joint_names)):
            raise ValueError(
                f"head_dof_rad must be [T, {len(contract.head_joint_names)}], got {head_dof_rad.shape}"
            )
        for i, name in enumerate(contract.head_joint_names):
            dof31[:, csv_names.index(name)] = np.rad2deg(head_dof_rad[:, i])

    quat_xyzw = root_quat_wxyz[:, [1, 2, 3, 0]]
    norms = np.linalg.norm(quat_xyzw, axis=1, keepdims=True)
    quat_xyzw = quat_xyzw / np.where(norms > 1e-12, norms, 1.0)
    euler_deg = Rotation.from_quat(quat_xyzw).as_euler("xyz", degrees=True)
    root_cm = root_pos_m * 100.0

    header = [contract.csv_frame_column, *contract.csv_root_columns, *csv_names]
    with path.open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(header)
        for i in range(n_frames):
            row = [
                int(frame_ids[i]),
                float_format % root_cm[i, 0],
                float_format % root_cm[i, 1],
                float_format % root_cm[i, 2],
                float_format % euler_deg[i, 0],
                float_format % euler_deg[i, 1],
                float_format % euler_deg[i, 2],
            ]
            row.extend(float_format % v for v in dof31[i])
            writer.writerow(row)
    return path


class A3FlatCsvCodec:
    """Thin object wrapper around the module-level functions."""

    def __init__(self, contract: A3Contract | None = None) -> None:
        self.contract = contract or load_contract()

    def read(self, path, source_fps: float = DEFAULT_SOURCE_FPS, frame_stride: int = DEFAULT_FRAME_STRIDE):
        return read_a3_flat_csv(path, source_fps, frame_stride, contract=self.contract)

    def write(self, path, root_pos_m, root_quat_wxyz, dof29_rad, **kwargs):
        return write_a3_flat_csv(
            path,
            root_pos_m,
            root_quat_wxyz,
            dof29_rad,
            contract=self.contract,
            **kwargs,
        )

    @staticmethod
    def finite_or_raise(data: A3CsvData) -> None:
        for name in ("root_pos_m", "root_quat_wxyz", "dof29_rad"):
            arr = getattr(data, name)
            if not np.isfinite(arr).all():
                raise ValueError(f"{name} contains non-finite values from {data.path}")


def quat_angle_deg(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Per-frame rotation angle between two wxyz quaternion sequences, degrees."""
    a = np.asarray(a, dtype=np.float64)
    b = np.asarray(b, dtype=np.float64)
    dots = np.abs(np.sum(a * b, axis=-1))
    dots = np.clip(dots, -1.0, 1.0)
    return np.rad2deg(2.0 * np.arccos(dots))


def assert_same_rotation(a: np.ndarray, b: np.ndarray, tol_deg: float = 1e-6) -> None:
    angles = quat_angle_deg(a, b)
    if not np.all(np.isfinite(angles)):
        raise AssertionError("non-finite quaternion angle")
    worst = float(np.max(angles)) if angles.size else 0.0
    if worst > tol_deg:
        raise AssertionError(f"quaternion mismatch: worst {worst:.3e} deg > {tol_deg}")


_ = math  # kept for potential numeric formatting hooks
