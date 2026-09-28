#!/usr/bin/env python3
"""Publish PICO packets shaped exactly like the real A3 sender, without a headset.

``run_pico_sim.sh --replay`` does NOT exercise the live path: it drives the
pipeline from the recording loader and never touches the ZMQ subscriber.  This
does, so the code that turns a real packet into a frame (including the root
reconstruction, which is what made the first real headset run fall over in
0.8 s) is testable on a desk.

Payload is byte-compatible with ``pico_pose_zmq_minimal.py``: root-relative
joints + orientation, i.e. ``smpl_pose`` / ``smpl_joints`` / ``body_quat_w`` and
deliberately **no** ``root_translation``.

Usage:
    python tools/fake_pico_sender.py --recording $A3WS/recordings/all/m5_stand --duration 60 &
    python -m a3_teleop_bridge.apps.retarget_live --source pico --backend umr-online \
        --no-publish --duration 20
    # expect: states [..., 'TRACKING', ...]  (it used to stay in CALIBRATION)
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

BRIDGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(BRIDGE_ROOT / "tests"))

PACKED_HEADER_SIZE = 1280
DTYPES = {np.dtype("float32"): "f32", np.dtype("float64"): "f64", np.dtype("int32"): "i32"}


def pack(payload: dict[str, np.ndarray], topic: str = "pose") -> bytes:
    """Mirror of the official packer: [topic][1280-byte header][payload]."""
    fields, chunks = [], []
    for name, value in payload.items():
        arr = np.ascontiguousarray(value)
        fields.append({"name": name, "dtype": DTYPES[arr.dtype], "shape": list(arr.shape)})
        chunks.append(arr.tobytes())
    header = {"v": 3, "endian": "le", "count": 1, "fields": fields}
    header_bytes = json.dumps(header, separators=(",", ":")).encode("utf-8")
    if len(header_bytes) > PACKED_HEADER_SIZE:
        raise SystemExit("header too large for the packed format")
    return topic.encode() + header_bytes.ljust(PACKED_HEADER_SIZE, b"\x00") + b"".join(chunks)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--recording", required=True, help="recording dir with smpl.npz")
    parser.add_argument("--port", type=int, default=5556)
    parser.add_argument("--duration", type=float, default=60.0)
    parser.add_argument("--fps", type=float, default=50.0)
    parser.add_argument(
        "--with-root",
        action="store_true",
        help="also publish root_translation (the real sender does not; useful to "
        "prove an explicitly published root still wins)",
    )
    args = parser.parse_args(argv)

    import zmq

    data = np.load(Path(args.recording).expanduser() / "smpl.npz", allow_pickle=True)
    joints = np.asarray(data["smpl_joints"], dtype=np.float32)
    pose = np.asarray(data["smpl_pose"], dtype=np.float32)
    quat = np.asarray(data["body_quat_w"], dtype=np.float32)
    root = np.asarray(data["root_translation"], dtype=np.float32) if "root_translation" in data else None

    # The recording stores body_quat_w in the *bridge* convention (it is what
    # the pipeline consumes).  A real sender puts the deploy-runtime frame on the
    # wire, and zmq_subscriber converts it back -- so to be a faithful stand-in
    # this tool has to apply the inverse transform, otherwise the frame gets
    # converted twice.
    def to_sender_frame(q):
        q = q / max(float(np.linalg.norm(q)), 1e-12)
        return _quat_mul(_quat_mul(q, R_Y_180_WXYZ), SMPL_BASE_ROT_CONJ_WXYZ)

    from a3_teleop_bridge.pico.zmq_subscriber import (
        R_Y_180_INV_WXYZ,
        SMPL_BASE_ROT_CONJ_INV,
        quat_mul_wxyz as _quat_mul,
    )

    R_Y_180_WXYZ = -R_Y_180_INV_WXYZ
    SMPL_BASE_ROT_CONJ_WXYZ = np.array([0.5, -0.5, -0.5, -0.5])

    ctx = zmq.Context()
    socket = ctx.socket(zmq.PUB)
    socket.bind(f"tcp://*:{args.port}")
    time.sleep(0.5)
    keys = "smpl_pose/smpl_joints/body_quat_w" + ("/root_translation" if args.with_root else "")
    print(f"[fake-pico] bound tcp://*:{args.port}  {joints.shape[0]} frames @{args.fps:g} Hz  keys={keys}")

    period = 1.0 / max(args.fps, 1e-6)
    sent, i, t0 = 0, 0, time.perf_counter()
    while time.perf_counter() - t0 < args.duration:
        k = i % joints.shape[0]
        payload = {
            "smpl_pose": pose[k][None],
            "smpl_joints": joints[k][None],
            "body_quat_w": to_sender_frame(quat[k]),
        }
        if args.with_root and root is not None:
            payload["root_translation"] = root[k]
        socket.send(pack(payload))
        sent += 1
        i += 1
        nxt = t0 + i * period
        if nxt > time.perf_counter():
            time.sleep(nxt - time.perf_counter())
    print(f"[fake-pico] sent {sent} frames")
    return 0


if __name__ == "__main__":
    sys.exit(main())
