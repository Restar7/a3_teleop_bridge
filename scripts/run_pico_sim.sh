#!/usr/bin/env bash
# One command: PICO headset -> online UMR -> A3_REFERENCE_V1 -> A3-fast in MuJoCo.
#
#   bash scripts/run_pico_sim.sh                 # live headset teleoperation in sim
#   bash scripts/run_pico_sim.sh --check         # preflight only, start nothing
#   bash scripts/run_pico_sim.sh --replay DIR    # no headset: drive from a recording
#   bash scripts/run_pico_sim.sh --no-viewer     # headless (CI / no display)
#   bash scripts/run_pico_sim.sh --skip-pico-probe   # do not pre-check headset streaming
#   bash scripts/run_pico_sim.sh --pico-fps 30       # override the source rate (default 50)
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
# 50 Hz: the online solve now runs at p50 ~16 ms (p99 18 ms), i.e. a ~62 Hz
# ceiling, so 50 Hz fits inside the 20 ms budget with no frame drops.  It was
# lowered to 30 only because the solver could not keep up (see runbook 17.9).
PICO_FPS=50
REPLAY=""
CHECK_ONLY=0
OUT_DIR=""
VIEWER=1
SKIP_PICO_PROBE=0

usage() { sed -n '2,24p' "$0"; }

while [ $# -gt 0 ]; do
  case "$1" in
    --check) CHECK_ONLY=1; shift ;;
    --viewer) VIEWER=1; shift ;;
    --skip-pico-probe) SKIP_PICO_PROBE=1; shift ;;
    --no-viewer) VIEWER=0; shift ;;
    --replay) REPLAY="$2"; shift 2 ;;
    --duration) DURATION="$2"; shift 2 ;;
    --policy-steps) POLICY_STEPS="$2"; shift 2 ;;
    --port) PORT="$2"; shift 2 ;;
    --pico-port) PICO_PORT="$2"; shift 2 ;;
    --pico-fps) PICO_FPS="$2"; shift 2 ;;
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

LAN_IP="$(ip -4 route get 1.1.1.1 2>/dev/null | awk '{for(i=1;i<=NF;i++) if($i=="src") print $(i+1)}' | head -1)"
[ -n "$LAN_IP" ] || LAN_IP="$(hostname -I 2>/dev/null | awk '{print $1}')"

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
  # The SDK talks to the PC Service on localhost:60061 (that is the address the
  # sender logs right after "initialize sdk").  It is the one prerequisite the
  # sender cannot work around: it only auto-starts the service when the official
  # install sits at /opt/apps/roboticsservice/runService.sh.
  PC_SERVICE_PORT="${PICO_SERVICE_PORT:-60061}"
  if ss -ltn 2>/dev/null | grep -q ":${PC_SERVICE_PORT} "; then
    pass "PC Service listening on 127.0.0.1:${PC_SERVICE_PORT}"
  elif [ -x /opt/apps/roboticsservice/runService.sh ]; then
    pass "PC Service installed (sender will auto-start /opt/apps/roboticsservice/runService.sh)"
  else
    fail "no PC Service on port ${PC_SERVICE_PORT} and none installed at /opt/apps/roboticsservice/runService.sh"
    note "the headset alone is not enough: the sender talks to the PC Service over localhost."
    note "install the official XRoboToolkit PC Service (Linux x86_64) on THIS machine, then"
    note "either keep it at /opt/apps/roboticsservice/runService.sh (sender auto-starts it)"
    note "or start it yourself before this script."
  fi
  # A listening PC Service is NOT a streaming headset: the service accepts the
  # headset's TCP connection on 63901 while the SDK still sees no body data.
  # Ask the SDK directly so a doomed run fails here instead of after 30 s.
  if [ -x "$PY_PICO" ] && [ "$SKIP_PICO_PROBE" = 0 ]; then
    # the probe reports through its exit code too, so swallow it here: under
    # `set -e`/pipefail a non-zero probe would abort the preflight mid-section
    _probe="$("$PY_PICO" "$BRIDGE_ROOT/tools/probe_pico_sdk.py" --timeout "${PICO_PROBE_TIMEOUT:-8}" 2>/dev/null | tail -1)" || true
    case "$_probe" in
      BODY_DATA_OK)
        pass "headset is streaming body tracking (SDK sees body data)" ;;
      BODY_DATA_TIMEOUT)
        fail "PC Service is up but NO body data is arriving from the headset"
        note "the device is connected at the TCP level but not publishing body tracking."
        note "check, in order:"
        note "  1. XRoboToolkit app on the headset: open / restart it, body tracking ON"
        note "  2. headset PC IP = ${LAN_IP:-<this host>} (it changed if you switched Wi-Fi)"
        note "  3. the app may have gone offline mid-session -- reopen it, then re-run"
        note "  4. service-side device log:"
        note "     grep -E 'device' ~/.local/share/PICOBusinessSuitData/log/\$(date +%Y%m%d).txt | tail"
        note "skip this probe with --skip-pico-probe (start the chain, then put the headset on)." ;;
      *)
        fail "PICO SDK probe did not answer (${_probe:-no output})"
        note "run it by hand: $PY_PICO $BRIDGE_ROOT/tools/probe_pico_sdk.py" ;;
    esac
  elif [ "$SKIP_PICO_PROBE" = 1 ]; then
    note "--skip-pico-probe: not checking whether the headset is streaming"
  fi
  note "headset side: XRoboToolkit app open, PC IP = ${LAN_IP:-<this host>}, body tracking on,"
  note "same Wi-Fi. Operator does one straight-stand calibration pose. Controller A toggles pause."
  note "(--start-unpaused is already passed, so it begins RUNNING.)"
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

# ---- startup snapshot: everything needed to reproduce a diagnosis later ----
{
  echo "=== when ==="; date -Is
  echo "=== host ==="; hostname; uname -a; echo "LAN_IP=$LAN_IP"
  echo "=== git ==="
  for r in "$BRIDGE_ROOT" "$UMR_ROOT" "$SONIC_ROOT"; do
    printf '%-46s %s  %s  dirty=%s\n' "$r" \
      "$(git -C "$r" rev-parse --abbrev-ref HEAD 2>/dev/null)" \
      "$(git -C "$r" rev-parse --short HEAD 2>/dev/null)" \
      "$(git -C "$r" status --porcelain 2>/dev/null | wc -l)"
  done
  echo "=== interpreters ==="
  for py in "$PY_BRIDGE" "$PY_UMR" "$PY_SIM" "$PY_PICO"; do
    printf '%-52s ' "$py"
    "$py" -c "import sys;print(sys.version.split()[0])" 2>/dev/null || echo MISSING
  done
  echo "=== mode ==="; echo "mode=$MODE viewer=$VIEWER pico_fps=$PICO_FPS port=$PORT pico_port=$PICO_PORT"
} > "$OUT_DIR/startup_info.txt" 2>&1
echo "[run] startup snapshot: $OUT_DIR/startup_info.txt"

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
      PYTHONUNBUFFERED=1 "$PY_PICO" -u gear_sonic/scripts/pico_pose_zmq_minimal.py \
      --port "$PICO_PORT" --target_fps "$PICO_FPS" --start_unpaused ) \
      >"$OUT_DIR/pico_sender.log" 2>&1 &
  SENDER_PID=$!

  echo -n "[run] waiting for body tracking + RUNNING "
  _body_note_shown=0
  for _ in $(seq 1 60); do
    if grep -q "Stream state: RUNNING" "$OUT_DIR/pico_sender.log" 2>/dev/null; then break; fi
    if ! kill -0 "$SENDER_PID" 2>/dev/null; then
      echo; echo "[run] the PICO sender exited immediately:"; tail -25 "$OUT_DIR/pico_sender.log"
      exit 1
    fi
    # still stuck before the first body frame: say which side is missing
    if [ "$_body_note_shown" = 0 ] && grep -q "waiting for body data" "$OUT_DIR/pico_sender.log" 2>/dev/null; then
      _waited=$(grep -c "waiting for body data" "$OUT_DIR/pico_sender.log" 2>/dev/null || echo 0)
      if [ "${_waited:-0}" -ge 15 ]; then
        _body_note_shown=1
        echo
        echo "[run] sender is stuck BEFORE any body frame ('waiting for body data...')."
        echo "      This is the headset/PC-Service side, not UMR/SONIC.  Check, in order:"
        echo "        1. PC Service running on THIS machine?    ss -ltn | grep ${PICO_SERVICE_PORT:-60061}"
        echo "        2. headset: XRoboToolkit app open, PC IP = ${LAN_IP:-<this host>}, same Wi-Fi?"
        echo "        3. headset: body tracking available?  (app shows body data)"
        echo "        4. operator standing straight, arms down, one calibration pose"
        echo -n "      still waiting "
      fi
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
if [ "$VIEWER" = 1 ]; then
  CHAIN+=(--viewer)
  echo "[run] MuJoCo window: ON   (Space pause, '.' step, ',' rewind, 'R' reset, close window to stop)"
  echo "      headless: add --no-viewer"
else
  echo "[run] MuJoCo window: OFF (headless)"
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
DUMP="$OUT_DIR/live_frames.jsonl"
if [ -f "$DUMP" ] && [ -f "$BRIDGE_ROOT/tools/report_live_dump.py" ]; then
  echo "[report] 源动作 → 参考(哪一层丢的动作,看这里)"
  "$PY_BRIDGE" "$BRIDGE_ROOT/tools/report_live_dump.py" "$DUMP" 2>&1 | sed 's/^/  /' || true
  echo
  echo "[report] 头显原始数据(腿不动时,先看是不是头显没跟踪)"
  echo "  戴着眼镜跑: \$A3WS/sonic_for_a3/.venv_pico_minimal/bin/python \\"
  echo "      $BRIDGE_ROOT/tools/probe_pico_body.py --seconds 15 --expect legs"
  echo
fi
echo "[done] exit=$rc  logs: $OUT_DIR"
echo "       pico sender : $OUT_DIR/pico_sender.log"
echo "       reference   : $OUT_DIR/retarget_live.log"
echo "       MuJoCo      : $OUT_DIR/sim2sim.log"
echo "       metrics     : $OUT_DIR/metrics.json"
exit "$rc"
