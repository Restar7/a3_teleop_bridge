"""PICO / XRoboToolkit SMPL subscriber (plan sections 23, 26).

The SONIC PICO streamer publishes the packed binary format documented in
``zmq_packed_message_subscriber.hpp`` (and produced by
``gear_sonic/scripts/pico_pose_zmq_minimal.py``)::

    [topic prefix]["pose"] [1280-byte JSON header, NUL padded] [binary fields]

Header: ``{"v": 3, "endian": "le", "count": N, "fields": [{name, dtype, shape}...]}``

The A3 sender publishes ``smpl_pose`` (N,21,3), ``smpl_joints`` (N,24,3) *local to
the body frame*, ``body_quat_w`` (N,4), a G1-flavoured ``joint_pos``/``joint_vel``
(ignored: those are not A3 joints) and the two timestamps.  Nothing here assumes
field order or a fixed layout beyond the documented 1280-byte header.
"""

from __future__ import annotations

import json
import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import yaml
import zmq

from ..clocks import SeqTracker, now_ns
from ..types import SMPL_JOINT_COUNT, SMPL_POSE_COUNT, HumanSmplFrame

BRIDGE_ROOT = Path(__file__).resolve().parents[3]
DEFAULT_TELEOP_CONFIG = BRIDGE_ROOT / "configs" / "teleop.yaml"

PACKED_HEADER_SIZE = 1280
DEFAULT_TOPIC = b"pose"

#: Pelvis height (m) used when a packet carries no absolute body position.
#: The A3 sender publishes root-relative joints + orientation only, and this is
#: the SMPL-X neutral standing pelvis height the whole offline/recorded path
#: uses (`recordings/*/smpl.npz` root_translation z = 0.975).  Without it the
#: retarget is handed a human lying on the floor and the reference drives the
#: robot down.  Override with A3_PICO_STANDING_PELVIS_M or `standing_pelvis_m`.
DEFAULT_STANDING_PELVIS_HEIGHT_M = 0.975

# ---------------------------------------------------------------------------
# Root-orientation convention bridge.
#
# pico_pose_zmq_minimal.py publishes body_quat_w in the *deploy runtime's*
# adjusted root-local frame:
#
#     body_quat_w = (Y_TO_Z_UP  x  xr_root  x  R_y(180 deg))  x  SMPL_BASE_ROT_CONJ
#
# (see _compute_local_smpl_from_xrt: global_rots = xr_root * R_y(180), then
#  root_quat_zup = Y_TO_Z_UP * root, then body = root_quat_zup * BASE_CONJ).
#
# What the rest of this project means by "the root orientation" -- and what the
# offline SMPL-X path, make_synthetic_pico_recording.py and every validated
# recording contain -- is
#
#     root_quat = Y_TO_Z_UP  x  xr_root
#
# i.e. the +90 deg X rotation that stands the Y-up SMPL skeleton up in a Z-up
# world, with the operator's heading on top.  Feeding the deploy-frame value
# straight into UMR rotates the whole body by the two trailing factors, which is
# what tipped the robot over 0.8 s into the first real headset run: the live
# reference root sat 1.24 m from the robot while the joints tracked fine.
#
# Undoing the two trailing factors is exact for any root pose (verified to 1e-16
# against the sender's own _quat_multiply_wxyz over random orientations); for a
# standing operator it reproduces the recording's [0.7071, 0.7071, 0, 0].
# ---------------------------------------------------------------------------
Y_TO_Z_UP_QUAT_WXYZ = np.array([0.7071067811865476, 0.7071067811865476, 0.0, 0.0])
SMPL_BASE_ROT_CONJ_WXYZ = np.array([0.5, -0.5, -0.5, -0.5])
SMPL_BASE_ROT_CONJ_INV = np.array([0.5, 0.5, 0.5, 0.5])
R_Y_180_INV_WXYZ = np.array([0.0, 0.0, -1.0, 0.0])


def quat_mul_wxyz(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Hamilton product in (w, x, y, z) order."""
    w1, x1, y1, z1 = (float(v) for v in np.asarray(a).reshape(4))
    w2, x2, y2, z2 = (float(v) for v in np.asarray(b).reshape(4))
    return np.array(
        [
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        ],
        dtype=np.float64,
    )


def sender_root_quat_to_smplx(body_quat_w: np.ndarray) -> np.ndarray:
    """Convert the A3 sender's body_quat_w into the SMPL-X root orientation."""
    q = quat_mul_wxyz(body_quat_w, SMPL_BASE_ROT_CONJ_INV)
    q = quat_mul_wxyz(q, R_Y_180_INV_WXYZ)
    return q / max(float(np.linalg.norm(q)), 1e-12)
DTYPE_MAP = {"f32": "<f4", "f64": "<f8", "i32": "<i4", "i64": "<i8", "bool": "?"}

__all__ = [
    "PicoProtocolError",
    "PackedMessage",
    "decode_packed_message",
    "PicoFrame",
    "PicoPoseSubscriber",
    "TeleopConfig",
]


class PicoProtocolError(RuntimeError):
    """Malformed, unsupported or incomplete PICO packet."""


@dataclass
class PackedMessage:
    header: dict
    fields: dict[str, np.ndarray]
    topic: bytes
    count: int
    size_bytes: int


def decode_packed_message(message: bytes, topic: bytes | str = DEFAULT_TOPIC) -> PackedMessage:
    """Decode ``[topic][1280-byte header][payload]`` into named arrays."""
    if isinstance(topic, str):
        topic = topic.encode("utf-8")
    if len(message) < PACKED_HEADER_SIZE:
        raise PicoProtocolError(f"message too short: {len(message)} bytes")
    body = message
    if topic and body.startswith(topic):
        body = body[len(topic) :]
    if len(body) < PACKED_HEADER_SIZE:
        raise PicoProtocolError("message has no room for the packed header")

    header_bytes = body[:PACKED_HEADER_SIZE]
    header = json.loads(header_bytes.split(b"\x00", 1)[0].decode("utf-8"))
    if not isinstance(header, dict) or "fields" not in header:
        raise PicoProtocolError("packed header lacks a 'fields' list")
    endian = str(header.get("endian", "le")).lower()
    if endian != "le":
        raise PicoProtocolError(f"unsupported endianness {endian!r}")

    payload = memoryview(body)[PACKED_HEADER_SIZE:]
    offset = 0
    fields: dict[str, np.ndarray] = {}
    for spec in header["fields"]:
        name = str(spec["name"])
        dtype_str = str(spec["dtype"])
        if dtype_str not in DTYPE_MAP:
            raise PicoProtocolError(f"unknown dtype {dtype_str!r} for field {name!r}")
        shape = tuple(int(v) for v in spec["shape"])
        np_dtype = np.dtype(DTYPE_MAP[dtype_str])
        count = int(np.prod(shape)) if shape else 1
        nbytes = count * np_dtype.itemsize
        if offset + nbytes > len(payload):
            raise PicoProtocolError(f"payload truncated while reading field {name!r}")
        arr = np.frombuffer(payload[offset : offset + nbytes], dtype=np_dtype).reshape(shape)
        fields[name] = np.array(arr)  # copy out of the buffer
        offset += nbytes

    return PackedMessage(
        header=header,
        fields=fields,
        topic=topic,
        count=int(header.get("count", 1)),
        size_bytes=len(message),
    )


@dataclass
class PicoFrame:
    """One decoded PICO frame plus bridge bookkeeping."""

    seq: int
    message: PackedMessage
    receive_timestamp_ns: int
    source_timestamp_ns: int
    receive_latency_ms: float
    smpl_frame: HumanSmplFrame | None
    rejected: bool = False
    reject_reason: str = ""

    def field(self, name: str) -> np.ndarray | None:
        return self.message.fields.get(name)


@dataclass
class TeleopConfig:
    topic: str = "pose"
    connect_host: str = "127.0.0.1"
    port: int = 5556
    recv_timeout_ms: int = 50
    conflate: bool = True
    high_water_mark: int = 1
    raw: dict = field(default_factory=dict)

    @classmethod
    def from_yaml(cls, path: Path | str | None = None) -> "TeleopConfig":
        path = Path(path).expanduser() if path else DEFAULT_TELEOP_CONFIG
        if not path.is_file():
            raise FileNotFoundError(f"teleop config not found: {path}")
        with open(path, "r", encoding="utf-8") as handle:
            doc = yaml.safe_load(handle) or {}
        pico = doc.get("pico", {}) or {}
        zmq_doc = doc.get("zmq", {}) or {}
        return cls(
            topic=str(pico.get("topic", "pose")),
            connect_host=str(pico.get("connect_host", "127.0.0.1")),
            port=int(pico.get("port", 5556)),
            recv_timeout_ms=int(zmq_doc.get("recv_timeout_ms", 50)),
            conflate=bool(zmq_doc.get("conflate", True)),
            high_water_mark=int(zmq_doc.get("high_water_mark", 1)),
            raw=doc,
        )

    @property
    def endpoint(self) -> str:
        return f"tcp://{self.connect_host}:{self.port}"


class PicoPoseSubscriber:
    """SUB socket -> :class:`PicoFrame` with latest-only draining."""

    #: Pelvis height used when the packet has no absolute root (see the module
    #: constant); instance attribute so a caller can override per session.
    standing_pelvis_m = DEFAULT_STANDING_PELVIS_HEIGHT_M

    #: field names the A3 sender publishes; the G1 joint vectors are ignored
    REQUIRED_FIELDS = ("smpl_pose", "smpl_joints")
    OPTIONAL_FIELDS = (
        "body_quat_w",
        "root_translation",
        "root_quat_w",
        "smpl_joints_world",
        "timestamp_realtime",
        "timestamp_monotonic",
    )
    IGNORED_FIELDS = ("joint_pos", "joint_vel")

    def __init__(
        self,
        config: TeleopConfig | None = None,
        context: zmq.Context | None = None,
        connect: str | None = None,
        subscribe: bool = True,
    ) -> None:
        self.config = config or TeleopConfig.from_yaml()
        self.context = context or zmq.Context.instance()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.setsockopt(zmq.RCVHWM, self.config.high_water_mark)
        if self.config.conflate:
            self.socket.setsockopt(zmq.CONFLATE, 1)
        self.socket.setsockopt(zmq.RCVTIMEO, self.config.recv_timeout_ms)
        self.connect = connect or self.config.endpoint
        if subscribe:
            self.socket.setsockopt(zmq.SUBSCRIBE, self.config.topic.encode("utf-8"))
            self.socket.connect(self.connect)
        self.seq_tracker = SeqTracker()
        self.received = 0
        self.rejected = 0
        self.messages_seen = 0
        self.last: PicoFrame | None = None

    # ------------------------------------------------------------------
    def poll(self, timeout_ms: int | None = None) -> PicoFrame | None:
        timeout = self.config.recv_timeout_ms if timeout_ms is None else timeout_ms
        packet = None
        try:
            packet = self.socket.recv(zmq.NOBLOCK)
            while True:  # latest-only
                try:
                    packet = self.socket.recv(zmq.NOBLOCK)
                except zmq.Again:
                    break
        except zmq.Again:
            if timeout and timeout > 0 and self.socket.poll(timeout):
                try:
                    packet = self.socket.recv(zmq.NOBLOCK)
                except zmq.Again:
                    return None
            if packet is None:
                return None

        receive_ns = now_ns()
        try:
            message = decode_packed_message(packet, self.config.topic)
        except (PicoProtocolError, json.JSONDecodeError) as exc:
            self.rejected += 1
            self.last = PicoFrame(
                seq=-1,
                message=None,  # type: ignore[arg-type]
                receive_timestamp_ns=receive_ns,
                source_timestamp_ns=0,
                receive_latency_ms=float("nan"),
                smpl_frame=None,
                rejected=True,
                reject_reason=str(exc),
            )
            return self.last

        self.messages_seen += 1
        source_ns = self._source_timestamp_ns(message)
        seq = int(message.header.get("seq", self.messages_seen))
        self.seq_tracker.update(seq if isinstance(seq, int) else self.messages_seen)
        latency_ms = (receive_ns - source_ns) / 1e6 if source_ns else float("nan")

        smpl_frame = None
        rejected = False
        reason = ""
        try:
            smpl_frame = self._build_smpl_frame(message, seq, source_ns, receive_ns)
        except PicoProtocolError as exc:
            rejected = True
            reason = str(exc)
            self.rejected += 1
        if not rejected:
            self.received += 1

        self.last = PicoFrame(
            seq=seq,
            message=message,
            receive_timestamp_ns=receive_ns,
            source_timestamp_ns=source_ns,
            receive_latency_ms=latency_ms,
            smpl_frame=smpl_frame,
            rejected=rejected,
            reject_reason=reason,
        )
        return self.last

    # ------------------------------------------------------------------
    @staticmethod
    def _source_timestamp_ns(message: PackedMessage) -> int:
        for key in ("timestamp_realtime", "timestamp_monotonic"):
            value = message.fields.get(key)
            if value is not None and value.size:
                stamp = float(np.asarray(value).reshape(-1)[-1])
                # `timestamp_realtime` is seconds since the epoch (~1.7e9) while a
                # nanosecond stamp is ~1.7e18; anything below 1e14 is seconds.
                if stamp >= 1e14:
                    return int(stamp)
                return int(round(stamp * 1e9))
        return 0

    def _build_smpl_frame(
        self, message: PackedMessage, seq: int, source_ns: int, receive_ns: int
    ) -> HumanSmplFrame:
        fields = message.fields
        missing = [name for name in self.REQUIRED_FIELDS if name not in fields]
        if missing:
            raise PicoProtocolError(f"missing required PICO fields: {missing}")

        joints = np.asarray(fields["smpl_joints"], dtype=np.float64)
        pose = np.asarray(fields["smpl_pose"], dtype=np.float64)
        if joints.ndim < 3 or joints.shape[-2:] != (SMPL_JOINT_COUNT, 3):
            raise PicoProtocolError(
                f"smpl_joints must end in ({SMPL_JOINT_COUNT}, 3), got {joints.shape}"
            )
        if pose.ndim < 3 or pose.shape[-2:] != (SMPL_POSE_COUNT, 3):
            raise PicoProtocolError(
                f"smpl_pose must end in ({SMPL_POSE_COUNT}, 3), got {pose.shape}"
            )
        joints = joints.reshape(-1, SMPL_JOINT_COUNT, 3)[-1]
        pose = pose.reshape(-1, SMPL_POSE_COUNT, 3)[-1]

        root_quat = np.array([1.0, 0.0, 0.0, 0.0])
        body_quat = fields.get("body_quat_w")
        if body_quat is not None and np.asarray(body_quat).size >= 4:
            candidate = np.asarray(body_quat, dtype=np.float64).reshape(-1, 4)[-1]
            norm = float(np.linalg.norm(candidate))
            if norm < 1e-9:
                raise PicoProtocolError("body_quat_w is degenerate")
            root_quat = candidate / norm
        elif "root_quat_w" in fields and np.asarray(fields["root_quat_w"]).size >= 4:
            candidate = np.asarray(fields["root_quat_w"], dtype=np.float64).reshape(-1, 4)[-1]
            root_quat = candidate / max(float(np.linalg.norm(candidate)), 1e-12)

        # The A3 sender publishes only root-relative joints + orientation: no
        # root_translation and no smpl_joints_world, so the packet carries no
        # absolute body position at all.  Defaulting to the origin is what made
        # the robot collapse on the first live PICO run -- the retarget saw a
        # human whose pelvis sat on the floor, so the reference put the A3's
        # pelvis ~0.98 m below where it stands, root_err grew to 1.37 m and the
        # robot tipped over backwards within 0.8 s.  Reconstruct a standing root
        # instead: teleoperation is in-place, and this is exactly what the
        # recorded path carries (root_translation z = 0.975).
        root_translation = np.zeros(3)
        root_source = "none"
        for key in ("root_translation", "root_pos", "root_pos_w"):
            value = fields.get(key)
            if value is not None and np.asarray(value).size >= 3:
                root_translation = np.asarray(value, dtype=np.float64).reshape(-1, 3)[-1]
                root_source = key
                break
        else:
            world = fields.get("smpl_joints_world")
            if world is not None and np.asarray(world).size >= 3:
                root_translation = (
                    np.asarray(world, dtype=np.float64).reshape(-1, SMPL_JOINT_COUNT, 3)[-1, 0]
                )
                root_source = "smpl_joints_world"
            else:
                root_translation = np.asarray(
                    [0.0, 0.0, float(getattr(self, "standing_pelvis_m", DEFAULT_STANDING_PELVIS_HEIGHT_M))],
                    dtype=np.float64,
                )
                root_source = "synthesized_standing"
        self.last_root_source = root_source

        # The packet carries the deploy-runtime frame; convert it to the SMPL-X
        # root orientation the rest of the pipeline (and UMR) expects.
        body_quat_for_pipeline = sender_root_quat_to_smplx(root_quat)

        wrist = fields.get("wrist_joint_pos")
        frame = HumanSmplFrame(
            seq=seq,
            timestamp_ns=source_ns,
            smpl_joints=joints,
            smpl_pose=pose,
            root_translation=root_translation,
            root_quat_wxyz=root_quat,
            body_quat_w=body_quat_for_pipeline if "body_quat_w" in fields else None,
            wrist_joint_pos=np.asarray(wrist, dtype=np.float64).reshape(-1)[:6]
            if wrist is not None
            else None,
            receive_timestamp_ns=receive_ns,
        )
        return frame.validate()

    # ------------------------------------------------------------------
    def stats(self) -> dict:
        return {
            "received": self.received,
            "rejected": self.rejected,
            "messages_seen": self.messages_seen,
            **self.seq_tracker.stats(),
        }

    def close(self) -> None:
        self.socket.close(linger=0)

    def __enter__(self) -> "PicoPoseSubscriber":
        return self

    def __exit__(self, *exc) -> None:
        self.close()


def wait_for_pico(subscriber: PicoPoseSubscriber, timeout_s: float = 5.0) -> bool:
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        frame = subscriber.poll(timeout_ms=100)
        if frame is not None and not frame.rejected:
            return True
    return False
