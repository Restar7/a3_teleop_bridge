#!/usr/bin/env bash
# 4090 / workstation side: consume a reference stream (from the Orin or from a
# local publisher) in SONIC sim2sim.  This is how you verify the Orin end to end
# *before* the robot is involved (plan sections 42/49).
#
# Usage:
#   bash scripts/run_mujoco_consumer.sh --endpoint tcp://<orin-ip>:5560 \
#        --motion logs/a3_validation/endurance_loop.csv --steps 3000
set -euo pipefail

BRIDGE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=/dev/null
source "$BRIDGE_ROOT/scripts/env_orin.sh"

ENDPOINT=""
MOTION="$A3WS/logs/a3_validation/endurance_loop.csv"
STEPS=3000
OUT="$A3WS/logs/mujoco_consumer"

while [ $# -gt 0 ]; do
  case "$1" in
    --endpoint) ENDPOINT="$2"; shift 2 ;;
    --motion) MOTION="$2"; shift 2 ;;
    --steps) STEPS="$2"; shift 2 ;;
    --out) OUT="$2"; shift 2 ;;
    -h|--help) sed -n '2,10p' "$0"; exit 0 ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
done
[ -n "$ENDPOINT" ] || { echo "need --endpoint tcp://<host>:5560" >&2; exit 2; }
mkdir -p "$OUT"

echo "[consumer] stream   : $ENDPOINT"
echo "[consumer] motion   : $MOTION"
echo "[consumer] steps    : $STEPS"

cd "$SONIC_A3_ROOT"
exec "$PY_SIM" gear_sonic/scripts/sim2sim_a3_mujoco.py \
  --checkpoint checkpoints/035_step200000/model_step_200000.pt \
  --motion "$MOTION" \
  --csv-source-fps 30 --csv-frame-stride 1 \
  --encoder-mode a3_fast \
  --mjcf gear_sonic/data/assets/robot_description/mjcf/a3_t2d5_loop_passive_foot_twostage_fit_optimized.xml \
  --reference-source stream --reference-endpoint "$ENDPOINT" \
  --batch-once --realtime --max-policy-steps "$STEPS" \
  --metrics-out "$OUT/metrics.json" --timeseries-out "$OUT/timeseries.json"
