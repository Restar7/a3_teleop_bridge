#!/usr/bin/env bash
# Build and run the proto-level teleop channel test (plan sections 59/60).
#
# Needs protoc + libprotobuf (apt: protobuf-compiler libprotobuf-dev); does NOT
# need AimRT, ROS or ONNX Runtime, so it runs on the workstation and on the robot.
#
# Usage: bash tools/run_cpp_channel_message_test.sh
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BRIDGE_ROOT="$(cd "$HERE/.." && pwd)"
SONIC_ROOT="${SONIC_A3_ROOT:-$(cd "$BRIDGE_ROOT/../sonic_for_a3" && pwd)}"
DEPLOY="$SONIC_ROOT/gear_sonic_deploy/src/g1/g1_deploy_onnx_ref"
PROTO="$DEPLOY/proto"
GEN="${GEN:-/tmp/a3_ta_proto_gen}"
OUT="${OUT:-/tmp/test_a3_teleop_channel_message}"
CXX="${CXX:-g++}"

command -v protoc >/dev/null || {
  echo "[cpp-channel] protoc not found: apt-get install -y protobuf-compiler libprotobuf-dev" >&2
  exit 2
}

rm -rf "$GEN" && mkdir -p "$GEN"
echo "[cpp-channel] generating TA protobuf -> $GEN"
protoc -I "$PROTO" --cpp_out="$GEN" \
  "$PROTO/aimdk/protocol/common/timestamp.proto" \
  "$PROTO/aimdk/protocol/common/control_source.proto" \
  "$PROTO/aimdk/protocol/common/header.proto" \
  "$PROTO/aimdk/protocol/ta/ta_whole_body_command.proto" \
  "$PROTO/aimdk/protocol/ta/ta_channel.proto"

echo "[cpp-channel] building $OUT"
"$CXX" -std=c++17 -O1 -Wall -Wextra -DHAS_A3_TA_PROTO \
  -I "$DEPLOY/include" -I "$GEN" \
  "$DEPLOY/unit_tests/test_a3_teleop_channel_message.cpp" \
  "$DEPLOY/src/a3_deploy/a3_teleop_command_source.cpp" \
  "$DEPLOY/src/a3_deploy/a3_teleop_channel_message.cpp" \
  "$GEN"/aimdk/protocol/common/*.pb.cc "$GEN"/aimdk/protocol/ta/*.pb.cc \
  -lprotobuf -o "$OUT"
"$OUT"
