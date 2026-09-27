# Source this before running anything on the Orin (or on the workstation):
#   source scripts/env_orin.sh
#
# It derives every path from this file's own location, so the same checkout works
# on the 4090 (~/a3_teleop_ws) and on the Orin (~/a3_teleop_ws).
#
#   A3WS           workspace root (holds a3_teleop_bridge / sonic_for_a3 / UMR)
#   BRIDGE_ROOT    this repo
#   SONIC_A3_ROOT  A3 MJCF + checkpoint + deploy package   (UMR config expands ${SONIC_A3_ROOT})
#   UMR_ROOT       UMR checkout (robot configs + online session)
#   PY_BRIDGE      bridge interpreter
#   PY_UMR         UMR interpreter (torch/trimesh/clarabel)
#   A3_REF_ENDPOINT  what the bridge binds; point it at the LAN IP for the A3
#   A3_PICO_ENDPOINT what the bridge subscribes to (GR00T/sonic_for_a3 PICO sender)

_A3_ENV_DIR="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")/.." && pwd)"
export BRIDGE_ROOT="${BRIDGE_ROOT:-$_A3_ENV_DIR}"
export A3WS="${A3WS:-$(dirname "$BRIDGE_ROOT")}"
export SONIC_A3_ROOT="${SONIC_A3_ROOT:-$A3WS/sonic_for_a3}"
export UMR_ROOT="${UMR_ROOT:-$A3WS/UMR}"

export PY_BRIDGE="${PY_BRIDGE:-$BRIDGE_ROOT/.venv_bridge/bin/python}"
export PY_UMR="${PY_UMR:-$UMR_ROOT/.venv_umr/bin/python}"
export PY_SIM="${PY_SIM:-$SONIC_A3_ROOT/.venv_sim/bin/python}"

# 0.0.0.0 = accept the A3 / MuJoCo consumer over the LAN; the A3 runtime connects
# to <orin-ip>:5560 unless configs/network.yaml says otherwise.
export A3_REF_ENDPOINT="${A3_REF_ENDPOINT:-tcp://0.0.0.0:5560}"
export A3_PICO_ENDPOINT="${A3_PICO_ENDPOINT:-tcp://127.0.0.1:5556}"

export PATH="$BRIDGE_ROOT/scripts:$PATH"
echo "[env] A3WS=$A3WS"
echo "[env] SONIC_A3_ROOT=$SONIC_A3_ROOT"
echo "[env] UMR_ROOT=$UMR_ROOT"
echo "[env] PY_BRIDGE=$PY_BRIDGE"
echo "[env] reference bind=$A3_REF_ENDPOINT  pico subscribe=$A3_PICO_ENDPOINT"
