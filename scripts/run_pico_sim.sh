#!/usr/bin/env bash
# One command: PICO headset -> online UMR -> A3_REFERENCE_V1 -> A3-fast in MuJoCo.
#
#   bash scripts/run_pico_sim.sh                 # live headset teleoperation in sim
#   bash scripts/run_pico_sim.sh --check         # preflight only, start nothing
#   bash scripts/run_pico_sim.sh --replay DIR    # no headset: drive from a recording
#   bash scripts/run_pico_sim.sh --duration 300 --policy-steps 6000
#
# This starts BOTH halves and tears them down together:
#   * the PICO sender  (sonic_for_a3/gear_sonic/scripts/pico_pose_zmq_minimal.py, port 5556)
#   * the bridge + MuJoCo (tools/run_live_chain.py --pico, i.e. retarget_live + sim2sim)
#
# Ctrl-C stops everything.  If the sender never reaches RUNNING this exits with a
# clear message instead of silently publishing nothing.
#
# Prerequisites are checked (and listed on failure) by --check:
#   PICO PC Service on THIS machine + headset on the same Wi-Fi  (live mode only)
#   xrobotoolkit_sdk importable by the sender interpreter
#   A3 assets/checkpoint, contract, SMPL-X model, live-chain deps
set -euo pipefail

BRIDGE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# env_orin.sh is the single source of truth for interpreter resolution:
# explicit PY_BRIDGE/PY_UMR/PY_SIM > project venv > conda env > python3.
# shellcheck source=/dev/null
source "$BRIDGE_ROOT/scripts/env_orin.sh" >/dev/null
WS_ROOT="$A3WS"
SONIC_ROOT="$SONIC_A3_ROOT"
UMR_ROOT="$UMR_ROOT"

DURATION=150
POLICY_STEPS=3000
PORT=5560
PICO_PORT=5556
REPLAY=""
CHECK_ONLY=0
OUT_DIR=""

usage() { sed -n '2,24p' "$0"; }

while [ $# -gt 0 ]; do
  case "$1" in
    --check) CHECK_ONLY=1; shift ;;
    --replay) REPLAY="$2"; shift 2 ;;
    --duration) DURATION="$2"; shift 2 ;;
    --policy-steps) POLICY_STEPS="$2"; shift 2 ;;
    --port) PORT="$2"; shift 2 ;;
    --pico-port) PICO_PORT="$2"; shift 2 ;;
    --out-dir) OUT_DIR="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
done

# ---- interpreters ----------------------------------------------------------
# Already resolved by env_orin.sh above; PY_PICO is not needed by the replay mode
# so fall back silently when the PICO env is absent.

ok=0; bad=0
pass() { printf '  [ OK ] %s\n' "$1"; ok=$((ok+1)); }
fail() { printf '  [FAIL] %s\n' "$1"; bad=$((bad+1)); }
note() { printf '         %s\n' "$1"; }

MODE="live PICO headset"
[ -n "$REPLAY" ] && MODE="replay (no headset): $REPLAY"

echo "=============================================================="
echo " PICO -> online UMR -> A3 reference -> A3-fast in MuJoCo"
echo " mode        : $MODE"
echo " bridge      : $BRIDGE_ROOT"
echo " PY_UMR      : $PY_UMR"
echo " PY_SIM      : $PY_SIM"
echo " PY_PICO     : $PY_PICO"
echo " reference   : tcp://0.0.0.0:$PORT     pico in: tcp://127.0.0.1:$PICO_PORT"
echo "--------------------------------------------------------------"

echo "[1/4] A3 assets + contract"
CONTRACT="$BRIDGE_ROOT/generated/a3_contract.json"
MJCF="$SONIC_ROOT/gear_sonic/data/assets/robot_description/mjcf/a3_t2d5_loop_passive_foot_twostage_fit_optimized.xml"
CKPT="$SONIC_ROOT/checkpoints/035_step200000/model_step_200000.pt"
[ -f "$CONTRACT" ] && pass "contract JSON" || fail "contract JSON ($CONTRACT) -- run tools/inspect_a3_contract.py"
[ -f "$MJCF" ]     && pass "A3 MJCF"       || fail "A3 MJCF ($MJCF) -- see runbook 5.1"
[ -f "$CKPT" ]     && pass "A3-fast checkpoint" || fail "checkpoint ($CKPT) -- .venv_sim/bin/python download_from_hf.py --component pt onnx"
[ -f "$UMR_ROOT/smpl/SMPLX_NEUTRAL.pkl" ] || [ -f "$UMR_ROOT/smpl/SMPLX_NEUTRAL.npz" ] \
  && pass "SMPL-X model" || fail "SMPL-X model -- licensed asset, see runbook 5.2"

echo "[2/4] interpreters and imports"
for spec in "UMR (torch/trimesh/clarabel/smplx/zmq):$PY_UMR:torch,trimesh,clarabel,smplx,zmq,numpy" \
            "sim (mujoco/torch):$PY_SIM:mujoco,torch,numpy"; do
  label="${spec%%:*}"; rest="${spec#*:}"; py="${rest%%:*}"; mods="${rest#*:}"
  if [ -x "$py" ] && "$py" -c "import ${mods//,/; import }" >/dev/null 2>&1; then
    pass "$label"
  else
    fail "$label -> $py (missing: $("$py" -c "
import importlib,sys
mods='$mods'.split(',')
missing=[m for m in mods if not importlib.util.find_spec(m)]
print(','.join(missing) or 'interpreter not executable')
" 2>/dev/null || echo 'interpreter not executable'))"
  fi
done

echo "[3/4] reference port"
if "$PY_BRIDGE" -c "
import socket,sys
s=socket.socket(); s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR,1)
try:
    s.bind(('0.0.0.0', $PORT))
except OSError as e:
    sys.exit(f'port $PORT busy: {e}')
finally:
    s.close()
" >/dev/null 2>&1; then pass "port $PORT free to bind"; else fail "port $PORT already in use (another run still alive?)"; fi

echo "[4/4] PICO side"
if [ -n "$REPLAY" ]; then
  [ -d "$REPLAY" ] && pass "recording dir $REPLAY" || fail "recording dir not found: $REPLAY"
else
  if "$PY_PICO" -c "import xrobotoolkit_sdk" >/dev/null 2>&1; then
    pass "xrobotoolkit_sdk importable by $PY_PICO"
  else
    fail "xrobotoolkit_sdk missing for $PY_PICO"
    note "fix: cd $SONIC_ROOT && bash install_scripts/install_pico_minimal.sh"
    note "     (PYTHON_BIN=python3 bash install_scripts/install_pico_minimal.sh if python3.10 is absent)"
  fi
  if "$PY_BRIDGE" -c "
import socket,sys
s=socket.socket(); s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR,1)
try: s.bind(('127.0.0.1', $PICO_PORT))
except OSError: sys.exit(1)
finally: s.close()
" >/dev/null 2>&1; then
    pass "port $PICO_PORT free (no stray sender)"
  else
    fail "port $PICO_PORT already in use -- a PICO sender is probably still running"
  fi
  note "PICO PC Service must be RUNNING ON THIS MACHINE, headset on the same Wi-Fi."
  note "If the sender starts but never reaches RUNNING, press A on the controller"
  note "(the stream is PAUSED by default; --start-unpaused is passed for you)."
fi

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

OUT_DIR="${OUT_DIR:-$WS_ROOT/logs/sim_teleop/$(date +%Y%m%d_%H%M%S)}"
mkdir -p "$OUT_DIR"
echo "[run] out-dir: $OUT_DIR"

SENDER_PID=""
cleanup() {
  if [ -n "$SENDER_PID" ] && kill -0 "$SENDER_PID" 2>/dev/null; then
    echo "[teardown] stopping PICO sender (pid $SENDER_PID)"
    kill -INT "$SENDER_PID" 2>/dev/null || true
    sleep 1
    kill -0 "$SENDER_PID" 2>/dev/null && kill -TERM "$SENDER_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

if [ -z "$REPLAY" ]; then
  echo "[run] starting PICO sender (port $PICO_PORT), log: $OUT_DIR/pico_sender.log"
  # The SDK's native lib is linked with an rpath, but the official installer also
  # asks for LD_LIBRARY_PATH; setting it costs nothing and covers other builds.
  XRT_LIB="$SONIC_ROOT/external_dependencies/XRoboToolkit-PC-Service-Pybind_X86_and_ARM64/lib"
  [ "$(uname -m)" = "aarch64" ] && XRT_LIB="$XRT_LIB/aarch64"
  ( cd "$SONIC_ROOT" && LD_LIBRARY_PATH="$XRT_LIB:${LD_LIBRARY_PATH:-}" \
      "$PY_PICO" gear_sonic/scripts/pico_pose_zmq_minimal.py \
      --port "$PICO_PORT" --target_fps 50 --start_unpaused ) \
      >"$OUT_DIR/pico_sender.log" 2>&1 &
  SENDER_PID=$!

  echo -n "[run] waiting for the sender to reach RUNNING "
  for _ in $(seq 1 40); do
    if grep -q "Stream state: RUNNING" "$OUT_DIR/pico_sender.log" 2>/dev/null; then break; fi
    if ! kill -0 "$SENDER_PID" 2>/dev/null; then
      echo; echo "[run] the PICO sender exited immediately:"; tail -25 "$OUT_DIR/pico_sender.log"
      exit 1
    fi
    echo -n "."; sleep 0.5
  done
  echo
  if grep -q "Stream state: RUNNING" "$OUT_DIR/pico_sender.log" 2>/dev/null; then
    echo "[run] sender RUNNING"
  else
    echo "[run] sender has not reported RUNNING yet -- the chain will hold the startup"
    echo "      pose for up to 30 s.  Press A on the controller if it stays paused."
    echo "      (sender log: $OUT_DIR/pico_sender.log)"
  fi
fi

if [ -n "$REPLAY" ]; then
  echo "[run] replaying $REPLAY through online UMR (no headset needed)"
  CHAIN=(--recording "$REPLAY")
else
  CHAIN=(--pico)
fi

set +e
"$PY_SIM" "$BRIDGE_ROOT/tools/run_live_chain.py" "${CHAIN[@]}" \
    --csv "$WS_ROOT/logs/a3_validation_all_nomj/m5_stand/m5_stand.csv" \
    --csv-fps 30 \
    --policy-steps "$POLICY_STEPS" \
    --duration "$DURATION" \
    --port "$PORT" \
    --sim-python "$PY_SIM" \
    --out-dir "$OUT_DIR"
rc=$?
set -e

echo "--------------------------------------------------------------"
echo "[done] exit=$rc  logs: $OUT_DIR"
echo "       pico sender : $OUT_DIR/pico_sender.log"
echo "       reference   : $OUT_DIR/retarget_live.log"
echo "       MuJoCo      : $OUT_DIR/sim2sim.log"
echo "       metrics     : $OUT_DIR/metrics.json"
exit "$rc"
