"""Read UMR retarget results (``*.npz``) without index assumptions.

A UMR result stores ``qpos`` plus ``robot_joint_names``.  Those names list the
model's **scalar** (hinge/slide) joints in model order -- they are *not*
contiguous in ``qpos``, because ball joints occupy four slots each (the A3
parallel-mechanism rods are ball joints).  The qpos address of every joint is
therefore resolved through the robot XML that produced the result, and the
resolution is asserted against the stored names.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

__all__ = ["UmrResult", "load_umr_result"]


@dataclass
class UmrResult:
    path: Path
    qpos: np.ndarray  # [T, nq]
    fps: float
    joint_names: tuple[str, ...]  # scalar joints, model order
    joint_qpos_addr: dict[str, int]  # joint name -> qpos address
    robot_xml: Path | None
    robot_name: str
    frame_ids: np.ndarray
    source_sequence_key: str
    source_format: str
    smpl_scale: float
    ground_z: float
    metadata: dict = field(default_factory=dict)

    @property
    def n_frames(self) -> int:
        return int(self.qpos.shape[0])

    @property
    def nq(self) -> int:
        return int(self.qpos.shape[1])

    @property
    def dt(self) -> float:
        return 1.0 / float(self.fps) if self.fps > 0 else float("nan")

    @property
    def root_pos(self) -> np.ndarray:
        """[T, 3] free-joint translation."""
        return self.qpos[:, :3]

    @property
    def root_quat_wxyz(self) -> np.ndarray:
        """[T, 4] free-joint rotation, normalised and sign-continuous."""
        quat = np.array(self.qpos[:, 3:7], dtype=np.float64, copy=True)
        norms = np.linalg.norm(quat, axis=1, keepdims=True)
        quat = quat / np.where(norms > 1e-12, norms, 1.0)
        for i in range(1, quat.shape[0]):
            if float(np.dot(quat[i - 1], quat[i])) < 0.0:
                quat[i] = -quat[i]
        return quat

    def joint_value(self, frame: int, name: str) -> float:
        return float(self.qpos[frame, self.joint_qpos_addr[name]])

    def frame_joint_dict(self, frame: int) -> dict[str, float]:
        """All scalar joint values of one frame, keyed by joint name."""
        return {name: float(self.qpos[frame, addr]) for name, addr in self.joint_qpos_addr.items()}

    def joint_series(self, name: str) -> np.ndarray:
        return np.asarray(self.qpos[:, self.joint_qpos_addr[name]], dtype=np.float64)


def _scalar_joint_addrs(robot_xml: Path) -> dict[str, int]:
    import mujoco

    model = mujoco.MjModel.from_xml_path(str(robot_xml))
    # NOTE: mujoco's mjtJoint members do not compare symmetrically against numpy
    # scalars, so `x in (mjJNT_HINGE, mjJNT_SLIDE)` silently yields False.  Always
    # compare plain ints (UMR itself does the same).
    scalar_types = (int(mujoco.mjtJoint.mjJNT_HINGE), int(mujoco.mjtJoint.mjJNT_SLIDE))
    addrs: dict[str, int] = {}
    for jid in range(model.njnt):
        if int(model.jnt_type[jid]) not in scalar_types:
            continue
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, jid)
        if name:
            addrs[name] = int(model.jnt_qposadr[jid])
    return addrs


def _resolve_robot_xml(raw: str, npz_path: Path) -> Path | None:
    """Locate the robot XML recorded in a UMR result.

    UMR stores an absolute path, which is stale as soon as the result is read on
    another machine.  Resolution: the stored path -> the path relative to the
    result -> the same path re-anchored on *this* machine's sonic checkout (the
    part below ``sonic_for_a3/`` is machine independent) -> a floating MJCF with
    the same file name inside the sonic checkout.
    """
    import os

    from .. import paths

    text = os.path.expandvars(os.path.expanduser(str(raw)))
    path = Path(text)
    if path.is_file():
        return path
    if not path.is_absolute():
        candidate = npz_path.parent / path
        if candidate.is_file():
            return candidate

    # re-anchor a foreign absolute path on this machine's checkout
    suffix = _suffix_after_sonic_root(path)
    if suffix is not None:
        try:
            root = paths.resolve_sonic_root()
        except paths.PathResolutionError:
            root = None
        if root is not None:
            candidate = root / suffix
            if candidate.is_file():
                return candidate
            # the floating MJCF is generated next to the source MJCF: try the same
            # file name in the checkout's mjcf directory
            for folder in ("gear_sonic/data/assets/robot_description/mjcf",):
                candidate = root / folder / path.name
                if candidate.is_file():
                    return candidate
    return None


def _suffix_after_sonic_root(path: Path) -> Path | None:
    """``/any/prefix/sonic_for_a3/gear_sonic/...`` -> ``gear_sonic/...``"""
    parts = path.parts
    for index in range(len(parts) - 1, -1, -1):
        if parts[index] == "sonic_for_a3" and index + 1 < len(parts):
            return Path(*parts[index + 1 :])
    return None


def load_umr_result(path: Path | str, robot_xml: Path | str | None = None) -> UmrResult:
    """Load and validate a UMR retarget result."""
    path = Path(path).expanduser()
    if not path.is_file():
        raise FileNotFoundError(f"UMR result not found: {path}")
    with np.load(path, allow_pickle=True) as data:
        if "qpos" not in data:
            raise ValueError(f"{path} has no 'qpos' field (keys: {sorted(data.files)})")
        qpos = np.asarray(data["qpos"], dtype=np.float64)
        fps_arr = np.asarray(data["fps"]).reshape(-1) if "fps" in data else np.asarray([30.0])
        names = tuple(str(n) for n in np.asarray(data["robot_joint_names"]).reshape(-1))
        xml_raw = str(np.asarray(data["robot_xml"]).reshape(-1)[0]) if "robot_xml" in data else ""
        robot_name = str(np.asarray(data["robot_name"]).reshape(-1)[0]) if "robot_name" in data else ""
        frame_ids = (
            np.asarray(data["frame_ids"]).reshape(-1)
            if "frame_ids" in data
            else np.arange(qpos.shape[0])
        )
        seq_key = (
            str(np.asarray(data["source_sequence_key"]).reshape(-1)[0])
            if "source_sequence_key" in data
            else ""
        )
        source_format = (
            str(np.asarray(data["source_format"]).reshape(-1)[0]) if "source_format" in data else ""
        )
        smpl_scale = (
            float(np.asarray(data["smpl_scale"]).reshape(-1)[0]) if "smpl_scale" in data else 1.0
        )
        ground_z = (
            float(np.asarray(data["ground_z"]).reshape(-1)[0]) if "ground_z" in data else 0.0
        )

    if qpos.ndim != 2 or qpos.shape[1] < 7:
        raise ValueError(f"qpos must be [T, nq>=7], got {qpos.shape}")
    if not np.isfinite(qpos).all():
        bad = np.argwhere(~np.isfinite(qpos))
        raise ValueError(f"{path}: qpos contains non-finite values (first at {bad[0].tolist()})")

    resolved_xml = Path(robot_xml).expanduser() if robot_xml else None
    if resolved_xml is None and xml_raw:
        resolved_xml = _resolve_robot_xml(xml_raw, path)
    if resolved_xml is None or not resolved_xml.is_file():
        raise FileNotFoundError(
            f"cannot resolve the robot XML for {path} (stored value: {xml_raw!r}).\n"
            "The result records the path of the UMR-generated floating MJCF, which lives "
            "inside the sonic_for_a3 checkout.\n"
            "Fix it by either:\n"
            "  * setting SONIC_A3_ROOT to this machine's sonic_for_a3 checkout, or\n"
            "  * regenerating the result here: tools/run_umr_a3_batch.py --force "
            "(recreates the floating MJCF), or\n"
            "  * passing robot_xml= explicitly."
        )

    addrs = _scalar_joint_addrs(resolved_xml)
    model_names = list(addrs.keys())
    if names and list(names) != model_names:
        # tolerate ordering differences but require the same set
        if set(names) != set(model_names):
            missing = sorted(set(model_names) - set(names))[:8]
            extra = sorted(set(names) - set(model_names))[:8]
            raise ValueError(
                f"{path}: robot_joint_names do not match {resolved_xml} "
                f"(missing from npz: {missing}, unknown: {extra})"
            )
        names = tuple(model_names)

    max_addr = max(addrs.values()) if addrs else 0
    if max_addr >= qpos.shape[1]:
        raise ValueError(
            f"{path}: qpos width {qpos.shape[1]} is smaller than the joint addresses "
            f"of {resolved_xml} (max {max_addr})"
        )

    return UmrResult(
        path=path,
        qpos=qpos,
        fps=float(fps_arr[0]),
        joint_names=tuple(names),
        joint_qpos_addr=addrs,
        robot_xml=resolved_xml,
        robot_name=robot_name,
        frame_ids=frame_ids,
        source_sequence_key=seq_key,
        source_format=source_format,
        smpl_scale=smpl_scale,
        ground_z=ground_z,
        metadata={"xml_stored": xml_raw, "n_scalar_joints": len(addrs)},
    )
