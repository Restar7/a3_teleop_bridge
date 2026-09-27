"""Build and run the C++ A3ReferenceStream test (plan sections 60/61).

The C++ runtime must read exactly the bytes the Python bridge publishes, so this
test compiles the provider standalone (libzmq + msgpack only, no ROS/ONNX) and
runs its assertions against a packet produced by the Python encoder.
"""

from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

import pytest

from a3_teleop_bridge.contract import load_contract

BRIDGE_ROOT = Path(__file__).resolve().parents[1]


def test_cpp_reference_stream_accepts_bridge_packets():
    if shutil.which("g++") is None:
        pytest.skip("no C++ compiler")
    if not Path("/usr/include/zmq.hpp").is_file():
        pytest.skip("libzmq/cppzmq headers not installed")
    if not Path("/usr/include/msgpack.hpp").is_file():
        pytest.skip("msgpack-cxx headers not installed")

    contract = load_contract()
    deploy = (
        contract.sonic_root
        / "gear_sonic_deploy/src/g1/g1_deploy_onnx_ref"
    )
    if not (deploy / "src/a3_deploy/a3_reference_stream.cpp").is_file():
        pytest.skip("sonic_for_a3 does not carry the C++ reference stream yet")

    proc = subprocess.run(
        ["bash", str(BRIDGE_ROOT / "tools" / "run_cpp_reference_test.sh")],
        capture_output=True,
        text=True,
        timeout=900,
        cwd=str(BRIDGE_ROOT),
    )
    report = proc.stdout + proc.stderr
    assert proc.returncode == 0, report[-4000:]
    assert "ALL C++ REFERENCE-STREAM TESTS PASSED" in report
    assert "[FAIL]" not in report
