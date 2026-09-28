#!/usr/bin/env python3
"""Ask the PICO SDK whether body-tracking data is actually arriving.

A listening PC Service is not the same thing as a streaming headset: the service
accepts the headset's TCP connection (port 63901) and the SDK still sees
``is_body_data_available() == False`` until the headset app is connected *and*
publishing body tracking.  Checking only the port produces a green light for a
run that is guaranteed to time out, which is exactly what happened on
2026-09-28: the device had gone offline 7 minutes before the run.

Exit codes / output are meant for a shell preflight:
    0  BODY_DATA_OK        body tracking is flowing right now
    2  SDK_MISSING         xrobotoolkit_sdk not importable by this interpreter
    3  BODY_DATA_TIMEOUT   service reachable but no body data within --timeout

Usage:
    python tools/probe_pico_sdk.py [--timeout 8]
"""

from __future__ import annotations

import argparse
import os
import sys
import time


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--timeout",
        type=float,
        default=8.0,
        help="seconds to wait for body data before giving up (default 8)",
    )
    args = parser.parse_args(argv)

    try:
        import xrobotoolkit_sdk as xrt
    except Exception as exc:  # pragma: no cover - environment dependent
        print(f"SDK_MISSING {exc}")
        return 2

    try:
        xrt.init()
    except Exception as exc:
        print(f"SDK_INIT_FAILED {exc}")
        return 2

    deadline = time.time() + max(0.0, float(args.timeout))
    while time.time() < deadline:
        try:
            if xrt.is_body_data_available():
                print("BODY_DATA_OK")
                sys.stdout.flush()
                # The SDK can abort in its static destructor ("terminate called
                # without an active exception"); that is noise, not a verdict.
                os._exit(0)
        except Exception as exc:
            print(f"SDK_QUERY_FAILED {exc}")
            return 2
        time.sleep(0.25)

    print("BODY_DATA_TIMEOUT")
    sys.stdout.flush()
    os._exit(3)


if __name__ == "__main__":
    sys.exit(main())
