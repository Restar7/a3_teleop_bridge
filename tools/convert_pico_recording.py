#!/usr/bin/env python3
"""Convert a PICO recording into a UMR-ready SMPL-X sequence (plan sections 25-27).

Usage:
    python tools/convert_pico_recording.py --recording ~/a3_teleop_ws/recordings/example_synthetic \
        --out ~/a3_teleop_ws/data/pico_smplx/example
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

BRIDGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(BRIDGE_ROOT / "src"))

from a3_teleop_bridge.pico.recorder import PicoRecording  # noqa: E402
from a3_teleop_bridge.umr.source_adapter import (  # noqa: E402
    PicoToSmplxAdapter,
    RootOrientationSource,
)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--recording", required=True, help="recordings/<session> directory")
    parser.add_argument("--out", required=True, help="output directory (or .npz path)")
    parser.add_argument(
        "--root-orientation",
        choices=[RootOrientationSource.STREAM, RootOrientationSource.HIPS_SHOULDERS, RootOrientationSource.IDENTITY],
        default=RootOrientationSource.STREAM,
    )
    parser.add_argument("--root-height", type=float, default=0.85, help="fallback standing height [m]")
    parser.add_argument("--stride", type=int, default=1, help="keep every Nth frame")
    args = parser.parse_args(argv)

    recording = PicoRecording.load(Path(args.recording).expanduser())
    if args.stride > 1:
        recording.frames = recording.frames[:: args.stride]
    sequence = PicoToSmplxAdapter(
        root_orientation_source=args.root_orientation,
        root_height_m=args.root_height,
    ).convert(recording)

    out = Path(args.out).expanduser()
    if out.suffix == ".npz":
        path = sequence.save(out)
    else:
        out.mkdir(parents=True, exist_ok=True)
        path = sequence.save(out / f"{Path(args.recording).name}.npz")

    print(f"recording : {args.recording} ({len(recording)} frames)")
    print(f"fps       : {sequence.fps:.3f}")
    print(f"wrote     : {path}")
    print(f"metadata  : {json.dumps(sequence.metadata, ensure_ascii=False)[:400]}")
    print("next      : tools/run_umr_a3_batch.py --data-dir <dir with this npz> --force")
    return 0


if __name__ == "__main__":
    sys.exit(main())
