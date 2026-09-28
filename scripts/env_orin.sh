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

# Interpreter resolution, in order:
#   1. an explicit PY_BRIDGE / PY_UMR / PY_SIM already in the environment
#   2. the per-project venv this runbook builds (scripts/orin_bootstrap.sh)
#   3. a conda env (default name: a3_bridge, override with A3_CONDA_ENV)
#   4. this shell's python3
# Step 3 exists because some machines already have numpy/scipy/zmq/torch/mujoco
# in one conda env, and building three ~3 GB venvs next to it buys nothing.
# Whichever one is chosen gets printed below, so it is never a silent guess.
_a3_pick_python() {  # <override> <venv-path>
  if [ -n "${1:-}" ] && [ -x "$1" ]; then printf '%s' "$1"; return 0; fi
  if [ -x "$2" ]; then printf '%s' "$2"; return 0; fi
  _env_name="${A3_CONDA_ENV:-a3_bridge}"
  for _base in "${CONDA_PREFIX:-}/.." "$HOME/miniconda3/envs" "$HOME/anaconda3/envs" \
               "$HOME/miniforge3/envs" "/opt/conda/envs"; do
    case "$_base" in /..|"") continue ;; esac
    if [ -x "$_base/$_env_name/bin/python" ]; then
      printf '%s' "$_base/$_env_name/bin/python"; return 0
    fi
  done
  command -v python3
}
_A3_PY_BRIDGE_OVERRIDE="${PY_BRIDGE:-}"
_A3_PY_UMR_OVERRIDE="${PY_UMR:-}"
_A3_PY_SIM_OVERRIDE="${PY_SIM:-}"
export PY_BRIDGE="$(_a3_pick_python "$_A3_PY_BRIDGE_OVERRIDE" "$BRIDGE_ROOT/.venv_bridge/bin/python")"
export PY_UMR="$(_a3_pick_python "$_A3_PY_UMR_OVERRIDE" "$UMR_ROOT/.venv_umr/bin/python")"
export PY_SIM="$(_a3_pick_python "$_A3_PY_SIM_OVERRIDE" "$SONIC_A3_ROOT/.venv_sim/bin/python")"
# The PICO sender lives in its own env (SDK pinning); prefer it when present.
export PY_PICO="${PY_PICO:-$SONIC_A3_ROOT/.venv_pico_minimal/bin/python}"

# 0.0.0.0 = accept the A3 / MuJoCo consumer over the LAN; the A3 runtime connects
# to <orin-ip>:5560 unless configs/network.yaml says otherwise.
export A3_REF_ENDPOINT="${A3_REF_ENDPOINT:-tcp://0.0.0.0:5560}"
export A3_PICO_ENDPOINT="${A3_PICO_ENDPOINT:-tcp://127.0.0.1:5556}"

export PATH="$BRIDGE_ROOT/scripts:$PATH"
echo "[env] A3WS=$A3WS"
echo "[env] SONIC_A3_ROOT=$SONIC_A3_ROOT"
echo "[env] UMR_ROOT=$UMR_ROOT"
echo "[env] PY_BRIDGE=$PY_BRIDGE"
echo "[env] PY_UMR=$PY_UMR"
echo "[env] PY_SIM=$PY_SIM"
if [ ! -x "$BRIDGE_ROOT/.venv_bridge/bin/python" ] && [ "$PY_BRIDGE" = "$HOME/miniconda3/envs/${A3_CONDA_ENV:-a3_bridge}/bin/python" ]; then
  echo "[env] note: project venvs absent; using conda env '${A3_CONDA_ENV:-a3_bridge}'"
  echo "[env]       (override with PY_BRIDGE/PY_UMR/PY_SIM, or A3_CONDA_ENV=<name>)"
fi
echo "[env] reference bind=$A3_REF_ENDPOINT  pico subscribe=$A3_PICO_ENDPOINT"
