#!/usr/bin/env bash
# Build the deployment bundle for Jetson Orin + A3 onboard (plan sections 22-24, 52-56).
#
# Produces, in one tarball:
#   * a3_teleop_bridge  (our工程: src/tools/tests/configs/docs/examples)
#   * sonic_for_a3      (code only: the deploy package + streaming reference interface)
#   * UMR               (code only: the A3 robot config + online-step entry points)
#   * MANIFEST.txt      (git SHAs, dirty flags, sizes, sha256 of the tarball)
#
# NOT included (by design):
#   .venv*/ .git/ logs/ recordings/ data/ checkpoints/ *.pt *.onnx *.rknn
#   SMPL-X body models  -> licensed asset, copy manually (see docs/orin_deployment.md §2.3)
#
# Usage:
#   bash scripts/package_bundle.sh [--out DIR] [--name NAME] [--with-umr]
set -euo pipefail

BRIDGE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS_ROOT="$(dirname "$BRIDGE_ROOT")"
SONIC_ROOT="${SONIC_A3_ROOT:-$WS_ROOT/sonic_for_a3}"
UMR_ROOT="${UMR_ROOT:-$WS_ROOT/UMR}"

OUT_DIR="$WS_ROOT/dist"
NAME="a3_teleop_orin"
WITH_UMR=0

while [ $# -gt 0 ]; do
  case "$1" in
    --out) OUT_DIR="$2"; shift 2 ;;
    --name) NAME="$2"; shift 2 ;;
    --with-umr) WITH_UMR=1; shift ;;
    -h|--help) sed -n '2,20p' "$0"; exit 0 ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
done

STAMP="$(date +%Y%m%d_%H%M%S)"
PKG="$NAME"
STAGE="$OUT_DIR/$PKG"
TARBALL="$OUT_DIR/${PKG}_${STAMP}.tar.gz"

echo "[package] bridge : $BRIDGE_ROOT"
echo "[package] sonic  : $SONIC_ROOT"
[ "$WITH_UMR" = 1 ] && echo "[package] umr    : $UMR_ROOT"
echo "[package] out    : $TARBALL"

rm -rf "$STAGE"
mkdir -p "$STAGE"

# ---------------------------------------------------------------- bridge ----
mkdir -p "$STAGE/a3_teleop_bridge"
for item in src tools tests integration configs docs examples benchmarks scripts pyproject.toml README.md requirements-bridge.txt; do
  if [ -e "$BRIDGE_ROOT/$item" ]; then
    cp -a "$BRIDGE_ROOT/$item" "$STAGE/a3_teleop_bridge/"
  fi
done

# ------------------------------------------------------------- sonic_for_a3 --
# code only: the MuJoCo/deploy sources plus the streaming reference interface.
mkdir -p "$STAGE/sonic_for_a3"
for item in gear_sonic gear_sonic_deploy docs README.md download_from_hf.py check_environment.py; do
  if [ -e "$SONIC_ROOT/$item" ]; then
    cp -a "$SONIC_ROOT/$item" "$STAGE/sonic_for_a3/" 2>/dev/null || true
  fi
done
# UMR writes these next to the source MJCF and they embed the *workstation* paths
find "$STAGE/sonic_for_a3" -name "*.floating_mjcf.xml" -delete 2>/dev/null || true

# 38 MB of docs/media is not needed on the robot; the runbooks are in the bridge,
# but keep this repository's own delivery document
if [ -f "$SONIC_ROOT/docs/a3_teleop_deployment.md" ]; then
  mkdir -p "$STAGE/sonic_for_a3/docs"
  cp -a "$SONIC_ROOT/docs/a3_teleop_deployment.md" "$STAGE/sonic_for_a3/docs/"
fi
rm -rf "$STAGE/sonic_for_a3/docs/a3_024_sim2real_materials" "$STAGE/sonic_for_a3/docs/licenses" 2>/dev/null || true
find "$STAGE/sonic_for_a3/docs" -maxdepth 1 -name "*.md" ! -name "a3_teleop_deployment.md" -delete 2>/dev/null || true
find "$STAGE/sonic_for_a3" -type d -name "media" -prune -exec rm -rf {} + 2>/dev/null || true
find "$STAGE/sonic_for_a3" -type d -name "node_modules" -prune -exec rm -rf {} + 2>/dev/null || true

# never ship weights or virtualenvs, even if the copy above picked something up
find "$STAGE/sonic_for_a3" \( -name "*.pt" -o -name "*.onnx" -o -name "*.rknn" -o -name "*.engine" \) \
  -delete 2>/dev/null || true
rm -rf "$STAGE/sonic_for_a3"/.venv* 2>/dev/null || true

# ----------------------------------------------------------------- UMR ------
if [ "$WITH_UMR" = 1 ]; then
  mkdir -p "$STAGE/UMR/assets"
  for item in scripts robot_configs README.md requirements-umr.txt; do
    [ -e "$UMR_ROOT/$item" ] && cp -a "$UMR_ROOT/$item" "$STAGE/UMR/" 2>/dev/null || true
  done
  # only the SMPL-X segmentation map is needed for the A3 retarget; the other
  # robots' meshes (312 MB) and the demo motions (57 MB) stay on the workstation
  for item in "$UMR_ROOT"/assets/*.pkl; do
    [ -e "$item" ] && cp -a "$item" "$STAGE/UMR/assets/" 2>/dev/null || true
  done
  [ -d "$UMR_ROOT/assets/a3" ] && cp -a "$UMR_ROOT/assets/a3" "$STAGE/UMR/assets/" 2>/dev/null || true
  # SMPL-X body models are licensed: never bundle them
  rm -rf "$STAGE/UMR/smpl" 2>/dev/null || true
fi

# ------------------------------------------------------------- manifest -----
{
  echo "bundle        : $PKG"
  echo "built_at      : $(date -u +%Y-%m-%dT%H:%M:%SZ)"
  echo "built_on      : $(uname -a)"
  echo
  for repo_path in "$BRIDGE_ROOT" "$SONIC_ROOT" "$UMR_ROOT"; do
    [ -d "$repo_path/.git" ] || continue
    name="$(basename "$repo_path")"
    sha="$(git -C "$repo_path" rev-parse HEAD 2>/dev/null || echo unknown)"
    branch="$(git -C "$repo_path" branch --show-current 2>/dev/null || echo unknown)"
    dirty="clean"
    [ -n "$(git -C "$repo_path" status --porcelain 2>/dev/null)" ] && dirty="DIRTY"
    echo "$name: $sha ($branch, $dirty)"
  done
  echo
  echo "excluded: .venv* .git logs recordings data checkpoints weights SMPL-X models"
  echo "SMPL-X must be copied by hand (see docs/orin_deployment.md section 2.3)"
} | tee "$STAGE/MANIFEST.txt"

tar -czf "$TARBALL" -C "$OUT_DIR" "$PKG"
rm -rf "$STAGE"

echo
# NOTE: `du` reports 0 allocated blocks on some networked filesystems, so size is
# taken from the file length instead.
BYTES="$(stat -c %s "$TARBALL" 2>/dev/null || wc -c < "$TARBALL")"
echo "[package] size : $(numfmt --to=iec "$BYTES" 2>/dev/null || echo "${BYTES} bytes")"
echo "[package] sha256: $(sha256sum "$TARBALL" | cut -d' ' -f1)"
echo "[package] done : $TARBALL"
echo
echo "next: bash scripts/sync_to_orin.sh --bundle $TARBALL --host <orin-ip>"
