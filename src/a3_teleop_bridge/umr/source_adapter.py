"""PICO SMPL recording -> UMR SMPL-X sequence (plan sections 25, 26).

PICO does **not** deliver a full SMPL-X sequence: the SONIC streamer publishes
24×3 body joints (root-relative), 21×3 body axis-angle and a body quaternion.
UMR's surface retargeting needs a body *surface*, so this adapter rebuilds a
SMPL-X shaped sequence from that material:

    PICO smpl_pose / smpl_joints / body_quat_w
      -> neutral SMPL-X template (fixed betas, neutral hands/face)
      -> root orientation from the stream, or from hips + shoulders
      -> root translation from the stream, or reconstructed
      -> UMR-ready .npz (poses[55,3], trans, betas, gender, mocap_frame_rate)

Version 1 deliberately uses ``betas = neutral``, ``hands = 0``, ``face = 0`` and
matches the template UMR uses for the neutral correspondence
(``smplx_neutral``) -- the same template the PICO path needs at runtime.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from ..pico.recorder import PicoRecording

__all__ = ["SmplxSequence", "PicoToSmplxAdapter", "RootOrientationSource"]

#: SMPL-22 joint indices used for the fallback heading estimate
PELVIS, L_HIP, R_HIP = 0, 1, 2
L_SHOULDER, R_SHOULDER = 16, 17


class RootOrientationSource:
    STREAM = "stream"
    HIPS_SHOULDERS = "hips_shoulders"
    IDENTITY = "identity"


def _quat_from_axes(lateral: np.ndarray, up: np.ndarray) -> np.ndarray:
    """Build a wxyz quaternion whose +Y is ``lateral`` and +Z is ``up``.

    The SMPL-X rest skeleton used by UMR is Y-up, so the body frame is defined by
    the lateral axis (left hip -> right hip, or shoulders) and the world up.
    """
    y = np.asarray(lateral, dtype=np.float64)
    z = np.asarray(up, dtype=np.float64)
    y = y - z * float(np.dot(y, z))
    ny = float(np.linalg.norm(y))
    if ny < 1e-9:
        return np.array([1.0, 0.0, 0.0, 0.0])
    y /= ny
    z = z / max(float(np.linalg.norm(z)), 1e-12)
    x = np.cross(y, z)
    x /= max(float(np.linalg.norm(x)), 1e-12)
    z = np.cross(x, y)
    rot = np.stack([x, y, z], axis=1)
    # rotation matrix -> quaternion (w, x, y, z), numerically stable branch
    trace = float(np.trace(rot))
    if trace > 0.0:
        s = np.sqrt(trace + 1.0) * 2.0
        quat = np.array(
            [0.25 * s, (rot[2, 1] - rot[1, 2]) / s, (rot[0, 2] - rot[2, 0]) / s, (rot[1, 0] - rot[0, 1]) / s]
        )
    elif rot[0, 0] > rot[1, 1] and rot[0, 0] > rot[2, 2]:
        s = np.sqrt(1.0 + rot[0, 0] - rot[1, 1] - rot[2, 2]) * 2.0
        quat = np.array(
            [(rot[2, 1] - rot[1, 2]) / s, 0.25 * s, (rot[0, 1] + rot[1, 0]) / s, (rot[0, 2] + rot[2, 0]) / s]
        )
    elif rot[1, 1] > rot[2, 2]:
        s = np.sqrt(1.0 + rot[1, 1] - rot[0, 0] - rot[2, 2]) * 2.0
        quat = np.array(
            [(rot[0, 2] - rot[2, 0]) / s, (rot[0, 1] + rot[1, 0]) / s, 0.25 * s, (rot[1, 2] + rot[2, 1]) / s]
        )
    else:
        s = np.sqrt(1.0 + rot[2, 2] - rot[0, 0] - rot[1, 1]) * 2.0
        quat = np.array(
            [(rot[1, 0] - rot[0, 1]) / s, (rot[0, 2] + rot[2, 0]) / s, (rot[1, 2] + rot[2, 1]) / s, 0.25 * s]
        )
    quat = quat / max(float(np.linalg.norm(quat)), 1e-12)
    return quat


@dataclass
class SmplxSequence:
    """A UMR-ready SMPL-X sequence."""

    poses: np.ndarray  # [T, 55, 3] axis-angle, [:, 0] = global orientation
    trans: np.ndarray  # [T, 3] root translation
    betas: np.ndarray  # [10]
    gender: str = "neutral"
    fps: float = 30.0
    metadata: dict = field(default_factory=dict)

    def save(self, path: Path | str) -> Path:
        path = Path(path).expanduser()
        path.parent.mkdir(parents=True, exist_ok=True)
        np.savez(
            path,
            poses=self.poses.astype(np.float32),
            trans=self.trans.astype(np.float32),
            betas=self.betas.astype(np.float32),
            gender=self.gender,
            mocap_frame_rate=np.float32(self.fps),
        )
        sidecar = path.with_suffix(".json")
        sidecar.write_text(json.dumps(self.metadata, indent=2, default=float) + "\n", encoding="utf-8")
        return path

    @property
    def n_frames(self) -> int:
        return int(self.poses.shape[0])


class PicoToSmplxAdapter:
    """Rebuild a SMPL-X sequence from a :class:`PicoRecording`."""

    def __init__(
        self,
        betas: np.ndarray | None = None,
        gender: str = "neutral",
        root_orientation_source: str = RootOrientationSource.STREAM,
        root_height_m: float = 0.85,
        quaternion_continuity: bool = True,
    ) -> None:
        self.betas = np.zeros(10) if betas is None else np.asarray(betas, dtype=np.float64).reshape(-1)[:10]
        self.gender = gender
        self.root_orientation_source = root_orientation_source
        self.root_height_m = float(root_height_m)
        self.quaternion_continuity = quaternion_continuity

    # ------------------------------------------------------------------
    def convert(self, recording: PicoRecording) -> SmplxSequence:
        if len(recording) == 0:
            raise ValueError("the recording is empty")
        frames = recording.frames
        n = len(frames)
        poses = np.zeros((n, 55, 3), dtype=np.float64)
        trans = np.zeros((n, 3), dtype=np.float64)

        previous_quat: np.ndarray | None = None
        used_stream_quat = 0
        for i, frame in enumerate(frames):
            poses[i, 1:22] = np.asarray(frame.smpl_pose, dtype=np.float64).reshape(21, 3)
            # hands (22:55) stay neutral in v1

            quat = None
            if self.root_orientation_source == RootOrientationSource.STREAM:
                candidate = frame.body_quat_w if frame.body_quat_w is not None else frame.root_quat_wxyz
                if candidate is not None and float(np.linalg.norm(candidate)) > 1e-9:
                    quat = np.asarray(candidate, dtype=np.float64).reshape(4)
                    used_stream_quat += 1
            if quat is None and self.root_orientation_source != RootOrientationSource.IDENTITY:
                quat = self._quat_from_body(frame)
            if quat is None:
                quat = np.array([1.0, 0.0, 0.0, 0.0])

            quat = quat / max(float(np.linalg.norm(quat)), 1e-12)
            if self.quaternion_continuity and previous_quat is not None:
                if float(np.dot(previous_quat, quat)) < 0.0:
                    quat = -quat
            previous_quat = quat
            poses[i, 0] = self._rotvec_from_quat_wxyz(quat)

            translation = np.asarray(frame.root_translation, dtype=np.float64).reshape(3)
            if not np.isfinite(translation).all() or float(np.linalg.norm(translation)) < 1e-9:
                translation = np.array([0.0, 0.0, self.root_height_m])
            trans[i] = translation

        meta = {
            "source": recording.source,
            "frames": n,
            "fps": self._fps(recording),
            "root_orientation_source": self.root_orientation_source,
            "stream_quaternion_frames": used_stream_quat,
            "betas": self.betas.tolist(),
            "gender": self.gender,
            "root_height_mean_m": float(trans[:, 2].mean()),
            "notes": [
                "hands (22:55) and face are neutral in v1",
                "betas default to neutral so the UMR 'smplx_neutral' correspondence applies",
                "root translation falls back to a fixed standing height when the stream "
                "does not publish it",
            ],
        }
        return SmplxSequence(
            poses=poses,
            trans=trans,
            betas=self.betas,
            gender=self.gender,
            fps=round(self._fps(recording), 6),
            metadata=meta,
        )

    # ------------------------------------------------------------------
    @staticmethod
    def _fps(recording: PicoRecording) -> float:
        stats = recording.stats()
        hz = float(stats.get("source_hz_mean") or 0.0)
        return hz if hz > 1.0 else 30.0

    @staticmethod
    def _rotvec_from_quat_wxyz(quat: np.ndarray) -> np.ndarray:
        from scipy.spatial.transform import Rotation

        return Rotation.from_quat(np.asarray(quat)[[1, 2, 3, 0]]).as_rotvec()

    def _quat_from_body(self, frame) -> np.ndarray | None:
        """Fallback heading from hips + shoulders (never a single hip vector)."""
        joints = np.asarray(frame.smpl_joints, dtype=np.float64)
        if joints.shape[0] <= R_SHOULDER:
            return None
        hip_lateral = joints[L_HIP] - joints[R_HIP]
        shoulder_lateral = joints[L_SHOULDER] - joints[R_SHOULDER]
        lateral = hip_lateral + shoulder_lateral
        if float(np.linalg.norm(lateral)) < 1e-9:
            lateral = hip_lateral
        up = np.array([0.0, 1.0, 0.0])  # SMPL-X rest skeleton is Y-up
        return _quat_from_axes(lateral, up)
