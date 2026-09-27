#!/usr/bin/env bash
# Jetson Orin preflight (plan sections 22/53): collect what the robot actually is
# instead of assuming a JetPack version.
#
# Usage: bash scripts/orin_preflight.sh [--out DIR]
# Writes system_info.txt / python_deps.txt / zmq_check.txt and prints a summary.
set -euo pipefail

BRIDGE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT_DIR="${BRIDGE_ROOT}/orin_system_info"
# shellcheck source=/dev/null
[ -f "$BRIDGE_ROOT/scripts/env_orin.sh" ] && source "$BRIDGE_ROOT/scripts/env_orin.sh" >/dev/null


while [ $# -gt 0 ]; do
  case "$1" in
    --out) OUT_DIR="$2"; shift 2 ;;
    -h|--help) sed -n '2,7p' "$0"; exit 0 ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
done
mkdir -p "$OUT_DIR"

echo "[preflight] writing to $OUT_DIR"

{
  echo "== identity =="
  uname -a
  uname -m
  echo
  echo "== jetson =="
  cat /etc/nv_tegra_release 2>/dev/null || echo "no /etc/nv_tegra_release (not a Jetson?)"
  cat /proc/device-tree/model 2>/dev/null | tr -d '\0' || true
  echo
  echo "== toolchain =="
  (nvcc --version 2>/dev/null || echo "no nvcc") | tail -4
  (cmake --version 2>/dev/null || echo "no cmake") | head -1
  (g++ --version 2>/dev/null || echo "no g++") | head -1
  python3 --version 2>&1
  echo
  echo "== tensorrt / cuda =="
  dpkg -l 2>/dev/null | grep -iE "tensorrt|nvinfer|cuda-toolkit" | awk '{print $2, $3}' || true
  ls -d /usr/local/cuda* 2>/dev/null || true
  echo
  echo "== resources =="
  free -h || true
  df -h / "$HOME" 2>/dev/null || true
  echo
  echo "== thermal / power (orin) =="
  (cat /sys/devices/virtual/thermal/thermal_zone*/temp 2>/dev/null | head -8) || true
  (nvpmodel -q 2>/dev/null || echo "no nvpmodel") | head -5
  (jetson_clocks --show 2>/dev/null | head -5 || true)
} | tee "$OUT_DIR/system_info.txt"

# which interpreter has what: the bridge and UMR use separate environments
{
  for PY in python3 "${BRIDGE_ROOT}/.venv_bridge/bin/python" "$HOME/a3_teleop_ws/UMR/.venv_umr/bin/python" "$HOME/a3_teleop_ws/sonic_for_a3/.venv_sim/bin/python"; do
    [ -x "$(command -v "$PY" 2>/dev/null || echo "$PY")" ] || continue
    echo "== $PY =="
    "$PY" - <<'PYEOF' 2>&1 || true
import importlib, platform, sys
print("python", sys.version.split()[0], platform.machine())
for mod in ("numpy", "scipy", "zmq", "yaml", "msgpack", "torch", "trimesh", "mujoco", "warp", "embreex", "clarabel"):
    try:
        m = importlib.import_module(mod)
        print(f"  {mod:10s} {getattr(m, '__version__', 'ok')}")
    except Exception as exc:  # noqa: BLE001
        print(f"  {mod:10s} MISSING ({type(exc).__name__})")
try:
    import torch
    print("  cuda      ", torch.cuda.is_available(), torch.cuda.get_device_name(0) if torch.cuda.is_available() else "-")
except Exception:
    pass
PYEOF
    echo
  done
} | tee "$OUT_DIR/python_deps.txt"

# the two sockets the live chain depends on must be usable on this box
{
  python3 - <<'PYEOF' 2>&1 || true
try:
    import zmq
    ctx = zmq.Context.instance()
    pub = ctx.socket(zmq.PUB)
    sub = ctx.socket(zmq.SUB)
    port = pub.bind_to_random_port("tcp://127.0.0.1")
    sub.setsockopt(zmq.SUBSCRIBE, b"")
    sub.connect(f"tcp://127.0.0.1:{port}")
    import time
    time.sleep(0.2)
    pub.send(b"ping")
    time.sleep(0.2)
    print("zmq", zmq.__version__, "loopback", "OK" if sub.poll(500) else "NO MESSAGE")
except Exception as exc:  # noqa: BLE001
    print("zmq loopback FAILED:", type(exc).__name__, exc)
PYEOF
} | tee "$OUT_DIR/zmq_check.txt"

echo
echo "[preflight] summary"
grep -E "^(Linux|aarch64|x86_64)" "$OUT_DIR/system_info.txt" | head -2 || true
grep -c "MISSING" "$OUT_DIR/python_deps.txt" | sed 's/^/[preflight] missing python deps: /'
grep -E "zmq" "$OUT_DIR/zmq_check.txt" || true
echo "[preflight] anything missing that is x86-only -> write it into ORIN_BLOCKER.md"
