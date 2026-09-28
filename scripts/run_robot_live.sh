#!/usr/bin/env bash
# One command: publish A3_REFERENCE_V1 from the PICO headset to the REAL robot.
#
#   bash scripts/run_robot_live.sh --check                      # preflight only
#   bash scripts/run_robot_live.sh --a3-host 10.42.10.12 --duration 1800 --confirm-live
#
# This is the same reference chain as the simulation
# (PICO -> online UMR -> predictor -> A3_REFERENCE_V1), with the MuJoCo consumer
# replaced by the robot: we BIND tcp://0.0.0.0:<port> and the A3 side subscribes
# to this machine's LAN address.
#
# Publishing motion to a real robot is gated: --confirm-live is required, and the
# go-live checklist is printed for an explicit acknowledgement first.
#
# A3-side prerequisites (build once on this machine, copy to the robot) are
# printed by --check; they are the ones that need "downloading things onto the
# robot" (rockchip deploy package + aarch64 ONNX Runtime + RKNN models).
set -euo pipefail

BRIDGE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# env_orin.sh is the single source of truth for interpreter resolution:
# explicit PY_BRIDGE/PY_UMR/PY_SIM > project venv > conda env > python3.
# shellcheck source=/dev/null
source "$BRIDGE_ROOT/scripts/env_orin.sh" >/dev/null
WS_ROOT="$A3WS"
SONIC_ROOT="$SONIC_A3_ROOT"
UMR_ROOT="$UMR_ROOT"

PORT=5560
DURATION=1800
A3_HOST=""
CONFIRM_LIVE=0
CHECK_ONLY=0
ASSUME_YES=0
STATS=""
CALIB=""

usage() { sed -n '2,18p' "$0"; }

while [ $# -gt 0 ]; do
  case "$1" in
    --check) CHECK_ONLY=1; shift ;;
    --a3-host) A3_HOST="$2"; shift 2 ;;
    --duration) DURATION="$2"; shift 2 ;;
    --port) PORT="$2"; shift 2 ;;
    --calibration) CALIB="$2"; shift 2 ;;
    --stats) STATS="$2"; shift 2 ;;
    --confirm-live) CONFIRM_LIVE=1; shift ;;
    --yes) ASSUME_YES=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
done

# interpreters resolved by env_orin.sh above

LAN_IP="$(ip -4 route get 1.1.1.1 2>/dev/null | awk '{for(i=1;i<=NF;i++) if($i=="src") print $(i+1)}' | head -1)"
[ -n "$LAN_IP" ] || LAN_IP="$(hostname -I 2>/dev/null | awk '{print $1}')"

ok=0; bad=0
pass() { printf '  [ OK ] %s\n' "$1"; ok=$((ok+1)); }
fail() { printf '  [FAIL] %s\n' "$1"; bad=$((bad+1)); }
warn() { printf '  [warn] %s\n' "$1"; }

echo "=============================================================="
echo " A3_REFERENCE_V1 -> REAL ROBOT   (PICO -> online UMR -> publish)"
echo " bridge      : $BRIDGE_ROOT"
echo " PY_UMR      : $PY_UMR"
echo " this host   : ${LAN_IP:-<unknown>}"
echo " bind        : tcp://0.0.0.0:$PORT   duration: ${DURATION}s"
echo " A3 host     : ${A3_HOST:-<not given>}"
echo "--------------------------------------------------------------"

echo "[1/5] reference side prerequisites"
CONTRACT="$BRIDGE_ROOT/generated/a3_contract.json"
[ -f "$CONTRACT" ] && pass "contract JSON" || fail "contract JSON -- tools/inspect_a3_contract.py"
[ -f "$UMR_ROOT/smpl/SMPLX_NEUTRAL.pkl" ] || [ -f "$UMR_ROOT/smpl/SMPLX_NEUTRAL.npz" ] \
  && pass "SMPL-X model" || fail "SMPL-X model (licensed) -- runbook 5.2"
if [ -x "$PY_UMR" ] && "$PY_UMR" -c "import torch,trimesh,clarabel,smplx,zmq,numpy" >/dev/null 2>&1; then
  pass "UMR imports (torch/trimesh/clarabel/smplx/zmq)"
else
  fail "UMR imports for $PY_UMR -- runbook §4.1, or export PY_UMR=<your python>"
fi
if "$PY_BRIDGE" -c "
import socket
s=socket.socket(); s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR,1)
s.bind(('0.0.0.0', $PORT)); s.close()
" >/dev/null 2>&1; then pass "port $PORT free to bind"; else fail "port $PORT already in use"; fi

echo "[2/5] PICO side (the reference source)"
if "$PY_BRIDGE" -c "
import socket
s=socket.socket(); s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR,1)
s.bind(('127.0.0.1', 5556)); s.close()
" >/dev/null 2>&1; then
  pass "port 5556 free"
else
  warn "port 5556 busy -- a PICO sender is already running (that is fine if it is yours)"
fi
echo "         start the sender in another terminal:"
echo "           cd $SONIC_ROOT && .venv_pico_minimal/bin/python \\"
echo "             gear_sonic/scripts/pico_pose_zmq_minimal.py --port 5556 --start_unpaused"
echo "         (or use: bash scripts/run_pico_sim.sh, which does it for you in sim)"

echo "[3/5] network to the robot"
if [ -n "$A3_HOST" ]; then
  if ping -c 2 -W 2 "$A3_HOST" >/dev/null 2>&1; then
    pass "ping $A3_HOST"
  else
    fail "cannot ping $A3_HOST -- check cabling / HDU / subnet"
  fi
  echo "         the A3 side must subscribe to: tcp://${LAN_IP:-<this-host-ip>}:$PORT"
else
  warn "no --a3-host given; skipping the reachability probe"
fi

echo "[4/5] A3-side readiness (things that live ON the robot)"
echo "         robot-side package must already be built and copied:"
echo "           cd $SONIC_ROOT"
echo "           python download_from_hf.py --component sysroot        # build input"
echo "           export A3_ONNXRUNTIME_AARCH64_TARBALL=/abs/path/onnxruntime-aarch64-*.tar.gz"
echo "           gear_sonic_deploy/scripts/build_a3_deploy_pkg.sh --arch rockchip --jobs 20 \\"
echo "             --onnxruntime-aarch64-tarball \"\$A3_ONNXRUNTIME_AARCH64_TARBALL\" \\"
echo "             --runtime-cfg gear_sonic_deploy/src/g1/g1_deploy_onnx_ref/config/a3_runtime_config.yaml"
echo "           rsync -avP <pkg-dir>/ <user>@<hdu>:/tmp/a3_pkg/"
echo "         then on the MDU: switch sm_config.yaml Motion group to [ \"agent\" ],"
echo "           restart agibot_pm, and run the reference->/ta/whole_body_command"
echo "           adapter node pointed at ${LAN_IP:-<this-host-ip>}:$PORT"
echo "         full sequence: docs/A3_ONBOARD.md sections 4-8"

echo "[5/5] go-live checklist (A->G, docs/GO_LIVE_CHECKLIST.md)"
echo "         [ ] harness / fall arrest rigged, physical e-stop in hand, spotter present"
echo "         [ ] area clear, nobody inside the robot's reach"
echo "         [ ] MuJoCo + AimSim already passed for these motions"
echo "         [ ] A3 runtime up, adapter node up, receive-only probe OK"
echo "         [ ] first motion is 'stand', then the 10 graded motions in order"
echo "--------------------------------------------------------------"
if [ "$bad" -gt 0 ]; then
  echo "[preflight] $ok ok, $bad failed -- fix the FAIL lines first."
  exit 1
fi
echo "[preflight] $ok ok, 0 failed."
if [ "$CHECK_ONLY" = 1 ]; then
  echo "[preflight] --check only; nothing started."
  exit 0
fi

if [ "$CONFIRM_LIVE" != 1 ]; then
  echo
  echo "[gate] refusing to publish motion to a real robot without --confirm-live."
  echo "       re-run with --confirm-live once the checklist above and the A3-side"
  echo "       receive-only probe are done."
  exit 3
fi
if [ "$ASSUME_YES" != 1 ]; then
  echo
  printf '[gate] type GO (uppercase) to start publishing to %s: ' "${A3_HOST:-the robot}"
  read -r answer
  if [ "$answer" != "GO" ]; then
    echo "[gate] aborted."
    exit 3
  fi
fi

# ---- run ------------------------------------------------------------------
OUT_DIR="$WS_ROOT/logs/robot_live/$(date +%Y%m%d_%H%M%S)"
mkdir -p "$OUT_DIR"
STATS="${STATS:-$OUT_DIR/pipeline_stats.json}"
CALIB="${CALIB:-$OUT_DIR/calibration.json}"
echo "[run] out-dir: $OUT_DIR"
echo "[run] A3 side connects to tcp://${LAN_IP:-<this-host-ip>}:$PORT"
echo "[run] Ctrl-C stops publishing; the A3 watchdog then holds and safe-stops."

exec bash "$BRIDGE_ROOT/scripts/run_orin_live.sh" \
  --duration "$DURATION" \
  --endpoint "tcp://0.0.0.0:$PORT" \
  --save-calibration "$CALIB" \
  --stats "$STATS"
