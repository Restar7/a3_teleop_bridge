"""PICO recording: SMPL frames -> npz + metadata + stats (plan section 24)."""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path

import numpy as np

from ..clocks import LatencyStats
from ..types import SMPL_JOINT_COUNT, SMPL_POSE_COUNT, HumanSmplFrame

__all__ = ["PicoRecording", "PicoRecorder"]


@dataclass
class PicoRecording:
    """In-memory recording of a PICO session."""

    frames: list[HumanSmplFrame] = field(default_factory=list)
    rejected: int = 0
    reject_reasons: dict[str, int] = field(default_factory=dict)
    seq_stats: dict[str, int] = field(default_factory=dict)
    started_ns: int = 0
    finished_ns: int = 0
    source: str = ""

    def __len__(self) -> int:
        return len(self.frames)

    # ------------------------------------------------------------------
    @property
    def timestamps_ns(self) -> np.ndarray:
        return np.asarray([f.timestamp_ns for f in self.frames], dtype=np.int64)

    @property
    def receive_timestamps_ns(self) -> np.ndarray:
        return np.asarray([f.receive_timestamp_ns for f in self.frames], dtype=np.int64)

    @property
    def duration_s(self) -> float:
        if len(self.frames) < 2:
            return 0.0
        return float((self.timestamps_ns[-1] - self.timestamps_ns[0]) / 1e9)

    def arrays(self) -> dict[str, np.ndarray]:
        if not self.frames:
            return {
                "smpl_joints": np.zeros((0, SMPL_JOINT_COUNT, 3), dtype=np.float32),
                "smpl_pose": np.zeros((0, SMPL_POSE_COUNT, 3), dtype=np.float32),
                "root_translation": np.zeros((0, 3), dtype=np.float32),
                "root_quat_wxyz": np.zeros((0, 4), dtype=np.float32),
                "timestamps_ns": np.zeros(0, dtype=np.int64),
                "receive_timestamps_ns": np.zeros(0, dtype=np.int64),
                "seq": np.zeros(0, dtype=np.int64),
            }
        has_body_quat = all(f.body_quat_w is not None for f in self.frames)
        out = {
            "smpl_joints": np.asarray([f.smpl_joints for f in self.frames], dtype=np.float32),
            "smpl_pose": np.asarray([f.smpl_pose for f in self.frames], dtype=np.float32),
            "root_translation": np.asarray(
                [f.root_translation for f in self.frames], dtype=np.float32
            ),
            "root_quat_wxyz": np.asarray(
                [f.root_quat_wxyz for f in self.frames], dtype=np.float32
            ),
            "timestamps_ns": self.timestamps_ns,
            "receive_timestamps_ns": self.receive_timestamps_ns,
            "seq": np.asarray([f.seq for f in self.frames], dtype=np.int64),
        }
        if has_body_quat:
            out["body_quat_w"] = np.asarray(
                [f.body_quat_w for f in self.frames], dtype=np.float32
            )
        return out

    def stats(self) -> dict:
        if not self.frames:
            return {"frames": 0}
        ts = self.timestamps_ns
        dts = np.diff(ts) / 1e9 if ts.size > 1 else np.zeros(0)
        latency = LatencyStats("pico_receive")
        latency.extend((self.receive_timestamps_ns - ts) / 1e6)
        finite = all(np.isfinite(f.smpl_joints).all() and np.isfinite(f.smpl_pose).all() for f in self.frames)
        return {
            "frames": len(self.frames),
            "rejected": self.rejected,
            "reject_reasons": self.reject_reasons,
            "seq": self.seq_stats,
            "duration_s": self.duration_s,
            "source_hz_mean": (len(self.frames) - 1) / self.duration_s if self.duration_s > 0 else 0.0,
            "dt_ms": {
                "mean": float(np.mean(dts) * 1e3) if dts.size else 0.0,
                "p95": float(np.percentile(dts, 95) * 1e3) if dts.size else 0.0,
                "max": float(np.max(dts) * 1e3) if dts.size else 0.0,
            },
            "receive_latency_ms": latency.summary(),
            "all_finite": bool(finite),
            "root_height_mean_m": float(np.mean([f.root_translation[2] for f in self.frames])),
        }

    # ------------------------------------------------------------------
    def save(self, directory: Path | str, source: str = "") -> Path:
        directory = Path(directory).expanduser()
        directory.mkdir(parents=True, exist_ok=True)
        arrays = self.arrays()
        np.savez_compressed(directory / "smpl.npz", **arrays)

        metadata = {
            "created": datetime.now().isoformat(timespec="seconds"),
            "source": source or self.source,
            "frames": len(self.frames),
            "duration_s": self.duration_s,
            "started_ns": self.started_ns,
            "finished_ns": self.finished_ns,
            "fields": sorted(arrays.keys()),
            "smpl_joints_shape": list(arrays["smpl_joints"].shape),
            "smpl_pose_shape": list(arrays["smpl_pose"].shape),
            "units": {"smpl_joints": "metres (body frame, SONIC convention)", "smpl_pose": "axis-angle rad"},
        }
        (directory / "metadata.json").write_text(
            json.dumps(metadata, indent=2) + "\n", encoding="utf-8"
        )
        (directory / "stats.json").write_text(
            json.dumps(self.stats(), indent=2, default=float) + "\n", encoding="utf-8"
        )
        return directory

    @classmethod
    def load(cls, directory: Path | str) -> "PicoRecording":
        directory = Path(directory).expanduser()
        data = np.load(directory / "smpl.npz")
        recording = cls(source=str(directory))
        has_body_quat = "body_quat_w" in data.files
        for i in range(data["smpl_joints"].shape[0]):
            recording.frames.append(
                HumanSmplFrame(
                    seq=int(data["seq"][i]) if "seq" in data.files else i,
                    timestamp_ns=int(data["timestamps_ns"][i]),
                    smpl_joints=np.asarray(data["smpl_joints"][i], dtype=np.float64),
                    smpl_pose=np.asarray(data["smpl_pose"][i], dtype=np.float64),
                    root_translation=np.asarray(data["root_translation"][i], dtype=np.float64),
                    root_quat_wxyz=np.asarray(data["root_quat_wxyz"][i], dtype=np.float64),
                    body_quat_w=np.asarray(data["body_quat_w"][i], dtype=np.float64)
                    if has_body_quat
                    else None,
                    receive_timestamp_ns=int(data["receive_timestamps_ns"][i])
                    if "receive_timestamps_ns" in data.files
                    else 0,
                )
            )
        meta_path = directory / "metadata.json"
        if meta_path.is_file():
            recording.source = json.loads(meta_path.read_text(encoding="utf-8")).get("source", "")
        return recording


class PicoRecorder:
    """Collects :class:`PicoFrame` objects into a :class:`PicoRecording`."""

    def __init__(self, source: str = "") -> None:
        self.recording = PicoRecording(source=source)

    def add(self, frame) -> bool:
        """Add one :class:`~a3_teleop_bridge.pico.zmq_subscriber.PicoFrame`."""
        if frame is None:
            return False
        if frame.rejected or frame.smpl_frame is None:
            self.recording.rejected += 1
            reason = frame.reject_reason or "unknown"
            self.recording.reject_reasons[reason] = (
                self.recording.reject_reasons.get(reason, 0) + 1
            )
            return False
        if not self.recording.started_ns:
            self.recording.started_ns = frame.receive_timestamp_ns
        self.recording.finished_ns = frame.receive_timestamp_ns
        self.recording.frames.append(frame.smpl_frame)
        return True

    def finalize(self, seq_stats: dict | None = None) -> PicoRecording:
        if seq_stats:
            self.recording.seq_stats = dict(seq_stats)
        return self.recording
