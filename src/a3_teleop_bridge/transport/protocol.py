"""``A3_REFERENCE_V1`` wire protocol (plan sections 37, 38, 51).

On-the-wire layout -- a **single** ZMQ frame so ``ZMQ_CONFLATE`` (latest-only)
keeps working; libzmq conflate is not defined for multipart messages::

    magic   : b"A3R1"                      (4 bytes)
    hdr_len : uint32 little endian          (4 bytes)
    header  : msgpack map                   (hdr_len bytes)
    payload : float32 array, C-contiguous   (rest)

Header fields: ``version, seq, timestamp_ns, dt, source_age_ms,
solver_latency_ms, valid, state, dtype, shapes, order``.  The header is
self-describing (dtype + shapes), so a receiver rejects a mismatched bridge build
instead of mis-reading memory.  A two-frame multipart form is also provided for
transports that cannot conflate; both decode paths share the same validation.

This protocol is frozen after the 4090 stage (plan section 51): do not redesign
it for Orin or for the C++ runtime.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass

import msgpack
import numpy as np

from ..contract import A3Contract, load_contract
from ..types import A3ReferenceWindow, BridgeState

__all__ = [
    "PROTOCOL_VERSION",
    "MAGIC",
    "HEADER_KEY",
    "PAYLOAD_KEY",
    "ProtocolError",
    "encode_packet",
    "decode_packet",
    "encode_frames",
    "decode_frames",
    "decode_any",
]

PROTOCOL_VERSION = "A3_REFERENCE_V1"
MAGIC = b"A3R1"
_MAGIC_STRUCT = struct.Struct("<4sI")
HEADER_KEY = b"h"
PAYLOAD_KEY = b"p"

ARRAY_ORDER = ("root_pos_m", "root_quat_wxyz", "joint_pos_rad", "joint_vel_rad_s")

#: Joint order carried by ``joint_pos_rad`` / ``joint_vel_rad_s``: the order the
#: SONIC A3-fast encoder consumes (URDF/IsaacLab ``dof_il`` order), NOT the
#: CSV/MJCF policy order.  The two are different permutations of the same 29
#: joints, so the header is tagged and the names are shipped for verification.
JOINT_ORDER_TAG = "a3_il_v1"


class ProtocolError(RuntimeError):
    """Raised when a packet is malformed, stale-versioned or inconsistent."""


@dataclass(frozen=True)
class _Layout:
    dtype: str
    shapes: dict[str, list[int]]
    offsets: dict[str, int]
    n_floats: int


def _layout(contract: A3Contract) -> _Layout:
    frames = contract.window_frames
    n = contract.n_policy_joints
    shapes = {
        "root_pos_m": [frames, 3],
        "root_quat_wxyz": [frames, 4],
        "joint_pos_rad": [frames, n],
        "joint_vel_rad_s": [frames, n],
    }
    offsets: dict[str, int] = {}
    cursor = 0
    for name in ARRAY_ORDER:
        offsets[name] = cursor
        cursor += int(np.prod(shapes[name]))
    return _Layout(dtype="<f4", shapes=shapes, offsets=offsets, n_floats=cursor)


def _to_il_order(window: A3ReferenceWindow, contract: A3Contract) -> tuple[np.ndarray, np.ndarray]:
    """Permute policy-order joint arrays into the encoder (il) order.

    ``il[k] = policy[il_to_policy_index[k]]`` -- the encoder joint *k* is looked
    up by name in the policy view.
    """
    perm = np.asarray(contract.il_to_policy_index, dtype=np.int64)
    joint_pos = np.asarray(window.joint_pos_rad, dtype=np.float64)[:, perm]
    joint_vel = np.asarray(window.joint_vel_rad_s, dtype=np.float64)[:, perm]
    return joint_pos, joint_vel


def _from_il_order(joint_pos: np.ndarray, joint_vel: np.ndarray, contract: A3Contract):
    """Inverse permutation: encoder order -> policy order."""
    inverse = np.asarray(contract.policy_to_il_index, dtype=np.int64)
    return np.asarray(joint_pos, dtype=np.float64)[:, inverse], np.asarray(
        joint_vel, dtype=np.float64
    )[:, inverse]


def _header_for(window: A3ReferenceWindow, layout: _Layout, contract: A3Contract) -> dict:
    return {
        "version": PROTOCOL_VERSION,
        "seq": int(window.seq),
        "timestamp_ns": int(window.timestamp_ns),
        "dt": float(window.dt),
        "source_age_ms": float(window.source_age_ms),
        "solver_latency_ms": float(window.solver_latency_ms),
        "valid": bool(window.valid),
        "state": str(getattr(window.state, "value", window.state)),
        "dtype": layout.dtype,
        "shapes": layout.shapes,
        "order": list(ARRAY_ORDER),
        "joint_order": JOINT_ORDER_TAG,
        "joint_names": list(contract.il_joint_names),
    }


def _payload_for(window: A3ReferenceWindow, layout: _Layout, contract: A3Contract) -> bytes:
    joint_pos_il, joint_vel_il = _to_il_order(window, contract)
    arrays = {
        "root_pos_m": np.asarray(window.root_pos_m, dtype=np.float32),
        "root_quat_wxyz": np.asarray(window.root_quat_wxyz, dtype=np.float32),
        "joint_pos_rad": joint_pos_il.astype(np.float32),
        "joint_vel_rad_s": joint_vel_il.astype(np.float32),
    }
    for name in ARRAY_ORDER:
        arr = arrays[name]
        expected = tuple(layout.shapes[name])
        if arr.shape != expected:
            raise ProtocolError(f"{name}: expected shape {expected}, got {arr.shape}")
    payload = np.concatenate(
        [np.ascontiguousarray(arrays[name]).reshape(-1) for name in ARRAY_ORDER]
    ).astype(layout.dtype, copy=False)
    return payload.tobytes()


def encode_packet(window: A3ReferenceWindow, contract: A3Contract | None = None) -> bytes:
    """Single-frame encoding (use this with ZMQ_CONFLATE)."""
    contract = contract or load_contract()
    layout = _layout(contract)
    header = msgpack.packb(_header_for(window, layout, contract), use_bin_type=True)
    return (
        _MAGIC_STRUCT.pack(MAGIC, len(header))
        + header
        + _payload_for(window, layout, contract)
    )


def decode_packet(packet: bytes, contract: A3Contract | None = None) -> A3ReferenceWindow:
    """Decode a single-frame packet."""
    if len(packet) < _MAGIC_STRUCT.size:
        raise ProtocolError(f"packet too short ({len(packet)} bytes)")
    magic, header_len = _MAGIC_STRUCT.unpack_from(packet, 0)
    if magic != MAGIC:
        raise ProtocolError(f"bad magic {magic!r}, expected {MAGIC!r}")
    start = _MAGIC_STRUCT.size
    end = start + header_len
    if end > len(packet):
        raise ProtocolError("header length exceeds packet size")
    return _decode_header_payload(packet[start:end], packet[end:], contract)


def encode_frames(window: A3ReferenceWindow, contract: A3Contract | None = None):
    """Two-frame multipart encoding (only for transports without conflate)."""
    contract = contract or load_contract()
    layout = _layout(contract)
    header = msgpack.packb(_header_for(window, layout, contract), use_bin_type=True)
    return [header, _payload_for(window, layout, contract)]


def decode_frames(frames, contract: A3Contract | None = None) -> A3ReferenceWindow:
    """Decode ``[header, payload]`` into a window; raises ProtocolError."""
    if isinstance(frames, (bytes, bytearray)):
        frames = [frames]
    if len(frames) != 2:
        raise ProtocolError(f"expected 2 frames (header, payload), got {len(frames)}")
    return _decode_header_payload(frames[0], frames[1], contract)


def decode_any(frames, contract: A3Contract | None = None) -> A3ReferenceWindow:
    """Decode either a single-frame packet or a ``[header, payload]`` pair."""
    if isinstance(frames, (bytes, bytearray)):
        if bytes(frames[:4]) == MAGIC:
            return decode_packet(bytes(frames), contract)
        raise ProtocolError("single frame is not an A3_REFERENCE_V1 packet (bad magic)")
    if len(frames) == 1:
        return decode_packet(bytes(frames[0]), contract)
    return decode_frames(frames, contract)


def _decode_header_payload(
    header_bytes, payload_bytes, contract: A3Contract | None = None
) -> A3ReferenceWindow:
    contract = contract or load_contract()
    try:
        header = msgpack.unpackb(bytes(header_bytes), raw=False)
    except Exception as exc:  # msgpack raises several types
        raise ProtocolError(f"cannot unpack header: {exc}") from exc
    if not isinstance(header, dict):
        raise ProtocolError("header is not a mapping")
    if header.get("version") != PROTOCOL_VERSION:
        raise ProtocolError(
            f"protocol version mismatch: got {header.get('version')!r}, want {PROTOCOL_VERSION!r}"
        )

    layout = _layout(contract)
    if header.get("dtype") != layout.dtype or header.get("shapes") != layout.shapes:
        raise ProtocolError(
            "layout mismatch: sender "
            f"{header.get('dtype')} {header.get('shapes')} vs receiver {layout.dtype} {layout.shapes}"
        )

    payload = np.frombuffer(bytes(payload_bytes), dtype=layout.dtype)
    if payload.size != layout.n_floats:
        raise ProtocolError(
            f"payload holds {payload.size} floats, expected {layout.n_floats}"
        )

    if header.get("joint_order") != JOINT_ORDER_TAG:
        raise ProtocolError(
            f"joint order mismatch: got {header.get('joint_order')!r}, want {JOINT_ORDER_TAG!r}"
        )
    names = header.get("joint_names")
    if names is not None and list(names) != list(contract.il_joint_names):
        raise ProtocolError("joint_names in the packet do not match this build's encoder order")

    arrays: dict[str, np.ndarray] = {}
    for name in ARRAY_ORDER:
        shape = tuple(layout.shapes[name])
        count = int(np.prod(shape))
        chunk = payload[layout.offsets[name] : layout.offsets[name] + count]
        arrays[name] = np.array(chunk, dtype=np.float64).reshape(shape)

    for name in ARRAY_ORDER:
        if not np.isfinite(arrays[name]).all():
            raise ProtocolError(f"{name} contains non-finite values")

    joint_pos_policy, joint_vel_policy = _from_il_order(
        arrays["joint_pos_rad"], arrays["joint_vel_rad_s"], contract
    )
    arrays["joint_pos_rad"] = joint_pos_policy
    arrays["joint_vel_rad_s"] = joint_vel_policy

    try:
        state = BridgeState(str(header.get("state", BridgeState.TRACKING.value)))
    except ValueError:
        state = BridgeState.TRACKING

    return A3ReferenceWindow(
        seq=int(header.get("seq", 0)),
        timestamp_ns=int(header.get("timestamp_ns", 0)),
        dt=float(header.get("dt", contract.window_dt)),
        root_pos_m=arrays["root_pos_m"],
        root_quat_wxyz=arrays["root_quat_wxyz"],
        joint_pos_rad=arrays["joint_pos_rad"],
        joint_vel_rad_s=arrays["joint_vel_rad_s"],
        source_age_ms=float(header.get("source_age_ms", 0.0)),
        solver_latency_ms=float(header.get("solver_latency_ms", 0.0)),
        valid=bool(header.get("valid", True)),
        state=state,
    )


# Aliases kept for call sites that predate the single-frame form.
encode_window = encode_packet
decode_window = decode_packet
