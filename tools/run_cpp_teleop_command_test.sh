#!/usr/bin/env bash
# Build and run the C++ teleop-command bridge test (plan sections 59/60).
#
# Pure C++17, no ZMQ / AimRT / ONNX Runtime: verifies the A3_REFERENCE_V1 ->
# /ta/whole_body_command field conversion, the joint-order permutation, and the
# pump's monotonicity / staleness / teleport guards.
#
# Usage: bash tools/run_cpp_teleop_command_test.sh
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BRIDGE_ROOT="$(cd "$HERE/.." && pwd)"
SONIC_ROOT="${SONIC_A3_ROOT:-$(cd "$BRIDGE_ROOT/../sonic_for_a3" && pwd)}"
DEPLOY="$SONIC_ROOT/gear_sonic_deploy/src/g1/g1_deploy_onnx_ref"
CXX="${CXX:-g++}"
OUT="${OUT:-/tmp/test_a3_teleop_command_source}"

echo "[cpp-teleop] source : $DEPLOY"
echo "[cpp-teleop] output : $OUT"
"$CXX" -std=c++17 -O1 -Wall -Wextra -I "$DEPLOY/include" \
  "$DEPLOY/unit_tests/test_a3_teleop_command_source.cpp" \
  "$DEPLOY/src/a3_deploy/a3_teleop_command_source.cpp" \
  -o "$OUT"
"$OUT"
