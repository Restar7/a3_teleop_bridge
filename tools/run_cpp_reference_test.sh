#!/usr/bin/env bash
# Build and run the C++ A3ReferenceStream test (plan sections 60/61).
#
# The standalone test only needs libzmq + msgpack + a C++17 compiler, so it can
# run on the workstation and on the A3/RK3588 target without ROS or ONNX Runtime.
#
# Usage: bash tools/run_cpp_reference_test.sh [packet.bin]
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BRIDGE_ROOT="$(cd "$HERE/.." && pwd)"
SONIC_ROOT="${SONIC_A3_ROOT:-$(cd "$BRIDGE_ROOT/../sonic_for_a3" && pwd)}"
DEPLOY="$SONIC_ROOT/gear_sonic_deploy/src/g1/g1_deploy_onnx_ref"
PY="${PYTHON:-$BRIDGE_ROOT/.venv_bridge/bin/python}"

PACKET="${1:-/tmp/a3_reference_packet.bin}"
if [[ ! -f "$PACKET" ]]; then
  echo "[cpp-test] generating a reference packet with the bridge encoder -> $PACKET"
  "$PY" - "$PACKET" <<'PYEOF'
import sys
sys.path.insert(0, "src")
import numpy as np
from a3_teleop_bridge.contract import load_contract
from a3_teleop_bridge.a3.limits import load_limits
from a3_teleop_bridge.types import A3ReferenceWindow
from a3_teleop_bridge.transport.protocol import encode_packet

contract = load_contract()
limits = load_limits()
n = contract.n_policy_joints
window = A3ReferenceWindow(
    seq=7,
    timestamp_ns=1_700_000_000_000_000_000,
    dt=contract.window_dt,
    root_pos_m=np.tile([0.0, 0.0, 1.07], (contract.window_frames, 1)),
    root_quat_wxyz=np.tile([1.0, 0.0, 0.0, 0.0], (contract.window_frames, 1)),
    joint_pos_rad=np.tile(limits.default_angle, (contract.window_frames, 1)),
    joint_vel_rad_s=np.zeros((contract.window_frames, n)),
    source_age_ms=3.0,
    valid=True,
)
open(sys.argv[1], "wb").write(encode_packet(window, contract))
print(f"wrote {sys.argv[1]} ({len(encode_packet(window, contract))} bytes)")
PYEOF
fi

echo "[cpp-test] building"
g++ -std=c++17 -O2 -w -I "$DEPLOY/include" \
    "$DEPLOY/unit_tests/test_a3_reference_stream_standalone.cpp" \
    "$DEPLOY/src/a3_deploy/a3_reference_stream.cpp" \
    -lzmq -o /tmp/a3_reference_stream_test
echo "[cpp-test] running"
/tmp/a3_reference_stream_test "$PACKET"
