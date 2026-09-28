#!/usr/bin/env bash
# Orin environment bootstrap (plan sections 22/54): recreate aarch64 environments
# from scratch -- never copy the workstation's venvs/conda dirs.
#
# Usage:
#   bash scripts/orin_bootstrap.sh --check-only        # verify preconditions only
#   bash scripts/orin_bootstrap.sh                     # create bridge venv + run tests
#   bash scripts/orin_bootstrap.sh --with-umr          # also prepare the UMR venv
#
# Environment overrides:
#   WS_ROOT     workspace root that holds UMR/ and sonic_for_a3/ (default: parent of this repo)
#   PYTHON      interpreter for the venvs (default: python3)
#   TORCH_INDEX pip index for torch (default: the JetPack-matched NVIDIA index; see docs)
set -euo pipefail

BRIDGE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS_ROOT="${WS_ROOT:-$(dirname "$BRIDGE_ROOT")}"
# shellcheck source=/dev/null
[ -f "$BRIDGE_ROOT/scripts/env_orin.sh" ] && source "$BRIDGE_ROOT/scripts/env_orin.sh" >/dev/null


PYTHON="${PYTHON:-python3}"
WITH_UMR=0
CHECK_ONLY=0
# Default index.  NOTE: an RTX 50-series (Blackwell, compute capability 12.0) card
# needs a CUDA >= 12.8 build (cu128/cu129/cu130); the legacy cu121 wheels have no
# sm_120 kernels.  This only matters for GPU work (the MuJoCo policy simulation) --
# the reference chain itself runs on CPU.
TORCH_INDEX="${TORCH_INDEX:-https://download.pytorch.org/whl/cu121}"

while [ $# -gt 0 ]; do
  case "$1" in
    --with-umr) WITH_UMR=1; shift ;;
    --check-only) CHECK_ONLY=1; shift ;;
    -h|--help) sed -n '2,12p' "$0"; exit 0 ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
done

echo "[bootstrap] bridge : $BRIDGE_ROOT"
echo "[bootstrap] ws     : $WS_ROOT"
echo "[bootstrap] python : $($PYTHON --version 2>&1) ($(uname -m))"

fail=0
need() { command -v "$1" >/dev/null 2>&1 || { echo "  MISSING: $1"; fail=1; }; }
echo "[bootstrap] checking base tooling"
for tool in git tar rsync curl; do need "$tool"; done
"$PYTHON" -c "import venv" 2>/dev/null || { echo "  MISSING: python3-venv"; fail=1; }
case "$(uname -m)" in
  aarch64) echo "  arch: aarch64 (Jetson Orin)" ;;
  x86_64)  echo "  arch: x86_64 -- fine for a 5060/4090 deployment machine (same runbook)" ;;
  *)       echo "  WARNING: unexpected arch $(uname -m)" ;;
esac

if [ "$fail" != 0 ]; then
  echo "[bootstrap] install the missing base packages first (apt-get install ...)" >&2
  exit 1
fi
[ "$CHECK_ONLY" = 1 ] && { echo "[bootstrap] --check-only: preconditions OK"; exit 0; }

# ------------------------------------------------------------- bridge venv --
# The bridge is pure python (numpy/scipy/zmq/yaml/msgpack); mujoco is optional
# and only needed for offline replay of the MuJoCo validations.
echo "[bootstrap] creating $BRIDGE_ROOT/.venv_bridge"
"$PYTHON" -m venv "$BRIDGE_ROOT/.venv_bridge"
"$BRIDGE_ROOT/.venv_bridge/bin/python" -m pip install --upgrade pip wheel >/dev/null
"$BRIDGE_ROOT/.venv_bridge/bin/python" -m pip install -r "$BRIDGE_ROOT/requirements-bridge.txt"
"$BRIDGE_ROOT/.venv_bridge/bin/python" -m pip install -e "$BRIDGE_ROOT"

echo "[bootstrap] running the test suite on the Orin"
"$BRIDGE_ROOT/.venv_bridge/bin/python" -m pytest "$BRIDGE_ROOT/tests" "$BRIDGE_ROOT/integration" -q \
  || echo "[bootstrap] WARNING: tests failed -- record it in ORIN_BLOCKER.md"

# ---------------------------------------------------------------- UMR venv --
if [ "$WITH_UMR" = 1 ]; then
  UMR_ROOT="${UMR_ROOT:-$WS_ROOT/UMR}"
  [ -d "$UMR_ROOT" ] || { echo "[bootstrap] UMR not found at $UMR_ROOT" >&2; exit 1; }
  echo "[bootstrap] creating $UMR_ROOT/.venv_umr"
  "$PYTHON" -m venv "$UMR_ROOT/.venv_umr"
  "$UMR_ROOT/.venv_umr/bin/python" -m pip install --upgrade pip wheel >/dev/null
  CAP="$(nvidia-smi --query-gpu=compute_cap --format=csv,noheader 2>/dev/null | head -1 | tr -d ' ')"
  if [ -n "$CAP" ] && awk "BEGIN{exit !($CAP >= 12.0)}" && [ "$TORCH_INDEX" = "https://download.pytorch.org/whl/cu121" ]; then
    echo "[bootstrap] WARNING: this GPU reports compute capability $CAP (Blackwell)."
    echo "[bootstrap]          cu121 wheels have no sm_120 kernels -- for the MuJoCo policy"
    echo "[bootstrap]          simulation re-run with e.g.:"
    echo "[bootstrap]            TORCH_INDEX=https://download.pytorch.org/whl/cu130 bash scripts/orin_bootstrap.sh --with-umr"
    echo "[bootstrap]          (the reference chain itself does not need CUDA)"
  fi
  echo "[bootstrap] torch from $TORCH_INDEX (JetPack: match it; Blackwell/x86: cu128+)"
  "$UMR_ROOT/.venv_umr/bin/python" -m pip install torch --index-url "$TORCH_INDEX"
  "$UMR_ROOT/.venv_umr/bin/python" -m pip install -r "$UMR_ROOT/requirements-umr.txt"
  echo "[bootstrap] installing the bridge into the UMR venv (the online session imports it)"
  "$UMR_ROOT/.venv_umr/bin/python" -m pip install -e "$BRIDGE_ROOT"

  echo "[bootstrap] UMR import check"
  "$UMR_ROOT/.venv_umr/bin/python" - <<'PYEOF' || echo "[bootstrap] WARNING: UMR import check failed"
import importlib, sys
bad = []
for mod in ("torch", "trimesh", "numpy", "scipy", "clarabel", "smplx"):
    try:
        importlib.import_module(mod)
    except Exception as exc:  # noqa: BLE001
        bad.append(f"{mod}: {type(exc).__name__}")
import torch
print("torch", torch.__version__, "cuda", torch.cuda.is_available())
print("missing:", bad or "none (except SMPL-X model data, see below)")
sys.exit(1 if bad else 0)
PYEOF
fi

# ------------------------------------------------------------ model assets --
echo
echo "[bootstrap] SMPL-X body models are NOT in the bundle (licence)."
echo "            copy them by hand into \$UMR_ROOT/smpl/ :"
echo "              UMR/smpl/SMPLX_NEUTRAL.pkl  (or .npz)"
echo
echo "[bootstrap] A3 checkpoint/ONNX/RKNN come from HuggingFace on the A3 side:"
echo "              python download_from_hf.py --component pt onnx rknn"
echo
echo "[bootstrap] contract check"
"$BRIDGE_ROOT/.venv_bridge/bin/python" "$BRIDGE_ROOT/tools/inspect_a3_contract.py" \
  --sonic-root "${SONIC_A3_ROOT:-$WS_ROOT/sonic_for_a3}" || true
echo
echo "[bootstrap] done. next: bash scripts/orin_preflight.sh && bash scripts/check_orin_ready.sh"
echo "[bootstrap] (script names say \"orin\" for historical reasons: they are machine-agnostic --"
echo "[bootstrap]  the same commands run on a 5060/4090 deployment machine, see docs/ORIN_FULL_RUNBOOK.md section 13)"
