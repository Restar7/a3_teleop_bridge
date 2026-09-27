#!/usr/bin/env bash
# Orin side of the live chain (plan sections 44/52/71):
#   PICO (XRoboToolkit) -> SMPL -> online UMR -> predictor -> A3_REFERENCE_V1 publisher
#
# Usage:
#   source scripts/env_orin.sh
#   bash scripts/run_orin_live.sh --duration 1800            # publish for 30 min
#   bash scripts/run_orin_live.sh --no-publish --duration 60 # reference only (no socket)
#
# The A3 runtime / MuJoCo consumer connects to $A3_REF_ENDPOINT (see configs/network.yaml).
set -euo pipefail

BRIDGE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=/dev/null
source "$BRIDGE_ROOT/scripts/env_orin.sh"

DURATION=1800
PUBLISH=1
ENDPOINT="$A3_REF_ENDPOINT"
EXTRA=()

while [ $# -gt 0 ]; do
  case "$1" in
    --duration) DURATION="$2"; shift 2 ;;
    --endpoint) ENDPOINT="$2"; shift 2 ;;
    --no-publish) PUBLISH=0; shift ;;
    --calibration) EXTRA+=(--calibration "$2"); shift 2 ;;
    --save-calibration) EXTRA+=(--save-calibration "$2"); shift 2 ;;
    --no-auto-calibrate) EXTRA+=(--no-auto-calibrate); shift ;;
    --robot-config) EXTRA+=(--robot-config "$2"); shift 2 ;;
    -h|--help) sed -n '2,12p' "$0"; exit 0 ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
done

[ -x "$PY_UMR" ] || { echo "[orin] UMR interpreter not found: $PY_UMR (run scripts/orin_bootstrap.sh --with-umr)" >&2; exit 1; }

echo "[orin] PICO in  : $A3_PICO_ENDPOINT   (sender: sonic_for_a3/gear_sonic/scripts/pico_pose_zmq_minimal.py --port 5556)"
echo "[orin] reference : $ENDPOINT"
echo "[orin] duration  : ${DURATION}s"
echo "[orin] NOTE: the first frame needs a ~15 s UMR one-off initialisation; the state"
echo "[orin]       machine stays in CALIBRATION until a session calibration exists."

cd "$BRIDGE_ROOT"
ARGS=(--source pico --backend umr-online --duration "$DURATION"
      --stats "$BRIDGE_ROOT/logs/orin_live/pipeline_stats.json")
mkdir -p "$BRIDGE_ROOT/logs/orin_live"
if [ "$PUBLISH" = 1 ]; then
  ARGS+=(--publish --endpoint "$ENDPOINT")
else
  ARGS+=(--no-publish)
fi
ARGS+=("${EXTRA[@]}")

exec "$PY_UMR" -m a3_teleop_bridge.apps.retarget_live "${ARGS[@]}"
