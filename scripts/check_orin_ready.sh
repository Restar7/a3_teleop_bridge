#!/usr/bin/env bash
# Readiness gate before touching the robot (plan sections 49/62): everything the
# live PICO -> UMR -> reference stream chain needs must be present and working.
#
# Usage: bash scripts/check_orin_ready.sh [--sonic-root DIR] [--umr-root DIR]
set -euo pipefail

BRIDGE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS_ROOT="${WS_ROOT:-$(dirname "$BRIDGE_ROOT")}"
# env_orin.sh already resolves PY_BRIDGE / PY_UMR with the full precedence
# (explicit override > project venv > conda env > python3), so just keep what it
# picked.  Re-deriving the venv path here is what used to make this gate silently
# skip its import / zmq / online-UMR sections on a conda-only machine.
PY_UMR_WAS_SET="${PY_UMR:-}"
# shellcheck source=/dev/null
[ -f "$BRIDGE_ROOT/scripts/env_orin.sh" ] && source "$BRIDGE_ROOT/scripts/env_orin.sh" >/dev/null

SONIC_ROOT="${SONIC_A3_ROOT:-$WS_ROOT/sonic_for_a3}"
UMR_ROOT="${UMR_ROOT:-$WS_ROOT/UMR}"

while [ $# -gt 0 ]; do
  case "$1" in
    --sonic-root) SONIC_ROOT="$2"; shift 2 ;;
    --umr-root) UMR_ROOT="$2"; shift 2 ;;
    -h|--help) sed -n '2,6p' "$0"; exit 0 ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
done
# honour --umr-root without clobbering an explicit/conda interpreter
if [ -z "$PY_UMR_WAS_SET" ] && [ -x "$UMR_ROOT/.venv_umr/bin/python" ]; then
  PY_UMR="$UMR_ROOT/.venv_umr/bin/python"
fi

ok=0; bad=0
check() { # check <label> <cmd...>
  local label="$1"; shift
  if "$@" >/dev/null 2>&1; then echo "  [ OK ] $label"; ok=$((ok+1));
  else echo "  [FAIL] $label"; bad=$((bad+1)); fi
}

echo "[ready] 1/5 interpreters"
check "bridge venv ($PY_BRIDGE)" test -x "$PY_BRIDGE"
check "umr venv ($PY_UMR)" test -x "$PY_UMR"

echo "[ready] 2/5 model + contract files"
check "A3 MJCF" test -f "$SONIC_ROOT/gear_sonic/data/assets/robot_description/mjcf/a3_t2d5_loop_passive_foot_twostage_fit_optimized.xml"
check "A3 URDF" test -d "$SONIC_ROOT/gear_sonic/data/assets/robot_description/urdf/a3"
check "checkpoint dir" test -d "$SONIC_ROOT/checkpoints/035_step200000"
check "SMPL-X model (licensed, copy by hand)" bash -c "ls $UMR_ROOT/smpl/SMPLX_NEUTRAL.pkl $UMR_ROOT/smpl/SMPLX_NEUTRAL.npz 2>/dev/null | grep -q ."
check "SMPL-X segmentation map" test -f "$UMR_ROOT/assets/smplx_parts_segm.pkl"
check "A3 robot config" test -f "$UMR_ROOT/robot_configs/humanoid_retarget_agibot_a3.json"
check "contract JSON" test -f "$BRIDGE_ROOT/generated/a3_contract.json"

echo "[ready] 3/5 python imports"
if [ -x "$PY_BRIDGE" ]; then
  check "bridge imports (numpy/scipy/zmq/yaml/msgpack)" "$PY_BRIDGE" -c "import numpy,scipy,zmq,yaml,msgpack"
  check "inspect_a3_contract" "$PY_BRIDGE" "$BRIDGE_ROOT/tools/inspect_a3_contract.py" --sonic-root "$SONIC_ROOT"
fi
if [ -x "$PY_UMR" ]; then
  check "umr imports (torch/trimesh/clarabel/smplx)" "$PY_UMR" -c "import torch,trimesh,clarabel,smplx"
  check "UMR retarget entry point" test -f "$UMR_ROOT/scripts/retarget_smpl_to_humanoid_surface_vector.py"
fi

echo "[ready] 4/5 sockets (loops are private, no port clash)"
if [ -x "$PY_BRIDGE" ]; then
  check "zmq loopback" "$PY_BRIDGE" -c "
import time, zmq
c = zmq.Context.instance(); p = c.socket(zmq.PUB); s = c.socket(zmq.SUB)
port = p.bind_to_random_port('tcp://127.0.0.1'); s.setsockopt(zmq.SUBSCRIBE, b''); s.connect(f'tcp://127.0.0.1:{port}')
time.sleep(0.2); p.send(b'x'); time.sleep(0.2); assert s.poll(500), 'no message'
"
fi
check "pico stream port free (5556)" bash -c "! (ss -ltn 2>/dev/null || netstat -ltn 2>/dev/null) | grep -q ':5556 ' || true"

echo "[ready] 5/5 live PICO pipeline (reference-only smoke, no robot)"
if [ -x "$PY_UMR" ] && [ -x "$PY_BRIDGE" ]; then
  check "online UMR single frame" bash -c "cd $BRIDGE_ROOT && timeout 300 $PY_UMR -c \"
import sys; sys.path.insert(0, '$BRIDGE_ROOT/src')
from a3_teleop_bridge.umr.umr_session import UmrRetargetSession
s = UmrRetargetSession(robot_config='$UMR_ROOT/robot_configs/humanoid_retarget_agibot_a3.json', verbose=False)
s.initialize(); print('online session ready')
\""
fi

echo
echo "[ready] $ok ok, $bad failed"
[ "$bad" = 0 ] || { echo "[ready] NOT ready -- fix the FAIL lines (or record them in ORIN_BLOCKER.md)"; exit 1; }
echo "[ready] ready for the live chain"
