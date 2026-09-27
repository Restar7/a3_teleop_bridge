"""Teleop session calibration (plan sections 74, 75).

Every session starts with the operator standing still in a known pose.  From a
few seconds of frames this module derives:

  * ``root_yaw_offset``   -- human heading vs the A3 heading at the same moment
  * ``pelvis_height_offset`` -- human pelvis height vs the A3 default standing height
  * ``body_scale``        -- human height / A3 height, from the same sample
  * ``left_right_ok``     -- a sanity check that the left/right joints are not swapped
  * ``timestamp``         -- when the calibration was taken

The scale is applied to the *root translation* only (plan section 75: foot motion
must not be scaled blindly -- that stays UMR's job through its contact
constraints).
"""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path

import numpy as np

from ..a3.limits import load_limits
from ..types import HumanSmplFrame

__all__ = ["SessionCalibration", "CalibrationError"]

#: SMPL-22 indices used for the sanity checks
PELVIS, L_HIP, R_HIP = 0, 1, 2
L_SHOULDER, R_SHOULDER = 16, 17


class CalibrationError(RuntimeError):
    """Raised when a calibration sample is unusable."""


@dataclass
class SessionCalibration:
    root_yaw_offset_rad: float = 0.0
    pelvis_height_offset_m: float = 0.0
    body_scale: float = 1.0
    human_height_m: float = 0.0
    robot_height_m: float = 0.0
    left_right_ok: bool = True
    frames_used: int = 0
    timestamp: str = ""

    # ------------------------------------------------------------------
    @classmethod
    def from_frames(
        cls,
        frames: list[HumanSmplFrame],
        robot_heading_rad: float = 0.0,
        robot_height_m: float | None = None,
        min_frames: int = 10,
    ) -> "SessionCalibration":
        if len(frames) < min_frames:
            raise CalibrationError(
                f"calibration needs at least {min_frames} frames, got {len(frames)}"
            )
        joints = np.asarray([f.smpl_joints for f in frames], dtype=np.float64)
        quats = np.asarray(
            [
                f.body_quat_w if f.body_quat_w is not None else f.root_quat_wxyz
                for f in frames
            ],
            dtype=np.float64,
        )
        if not np.isfinite(joints).all() or not np.isfinite(quats).all():
            raise CalibrationError("calibration frames contain non-finite values")

        # heading from the body quaternion: the SMPL rest skeleton faces +Z after
        # the +90 deg X base rotation, so yaw is read off the body x-axis.
        mean_quat = quats.mean(axis=0)
        mean_quat /= max(float(np.linalg.norm(mean_quat)), 1e-12)
        w, x, y, z = mean_quat
        fx = 1.0 - 2.0 * (y * y + z * z)
        fy = 2.0 * (x * y + w * z)
        human_heading = float(np.arctan2(fy, fx))
        yaw_offset = float(np.angle(np.exp(1j * (human_heading - robot_heading_rad))))

        pelvis_height = float(np.mean(joints[:, PELVIS, 1]))  # SMPL is Y-up
        if robot_height_m is None:
            limits = load_limits()
            robot_height_m = float(np.mean(limits.raw["joints"][
                "left_ankle_roll_joint"
            ]["default_angle"]))  # placeholder; refined by the caller
            robot_height_m = 1.07
        height = float(joints[:, :, 1].max() - joints[:, :, 1].min())
        body_scale = float(robot_height_m) / max(height, 1e-6)

        # left/right sanity: the left hip must sit on the opposite side of the right
        lateral = float(np.mean(joints[:, L_HIP, 0] - joints[:, R_HIP, 0]))
        shoulder_lateral = float(
            np.mean(joints[:, L_SHOULDER, 0] - joints[:, R_SHOULDER, 0])
        )
        left_right_ok = bool(lateral * shoulder_lateral > 0.0)

        return cls(
            root_yaw_offset_rad=yaw_offset,
            pelvis_height_offset_m=float(pelvis_height),
            body_scale=body_scale,
            human_height_m=height,
            robot_height_m=float(robot_height_m),
            left_right_ok=left_right_ok,
            frames_used=len(frames),
            timestamp=datetime.now().isoformat(timespec="seconds"),
        )

    # ------------------------------------------------------------------
    def apply_root(self, root_translation: np.ndarray, root_quat_wxyz: np.ndarray):
        """Apply the heading offset and the height offset to a root pose."""
        from ..a3.predictor import quat_exp_map, quat_multiply

        yaw = quat_exp_map(np.array([0.0, 0.0, -self.root_yaw_offset_rad]))
        quat = quat_multiply(yaw, np.asarray(root_quat_wxyz, dtype=np.float64))
        translation = np.asarray(root_translation, dtype=np.float64).copy()
        translation[2] -= self.pelvis_height_offset_m - self.robot_height_m
        return translation, quat

    def save(self, path: Path | str) -> Path:
        path = Path(path).expanduser()
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(asdict(self), indent=2) + "\n", encoding="utf-8")
        return path

    @classmethod
    def load(cls, path: Path | str) -> "SessionCalibration":
        data = json.loads(Path(path).expanduser().read_text(encoding="utf-8"))
        return cls(**data)
