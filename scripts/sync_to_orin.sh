#!/usr/bin/env bash
# Copy the deployment bundle to the Orin and unpack it (plan sections 22-24).
#
# Usage:
#   bash scripts/sync_to_orin.sh --bundle dist/a3_teleop_orin_*.tar.gz --host 192.168.1.50 [--user agibot] [--dest ~/a3_teleop_ws]
set -euo pipefail

BUNDLE=""
HOST=""
USER_NAME="${ORIN_USER:-agibot}"
DEST="~/a3_teleop_ws"

while [ $# -gt 0 ]; do
  case "$1" in
    --bundle) BUNDLE="$2"; shift 2 ;;
    --host) HOST="$2"; shift 2 ;;
    --user) USER_NAME="$2"; shift 2 ;;
    --dest) DEST="$2"; shift 2 ;;
    -h|--help) sed -n '2,8p' "$0"; exit 0 ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
done

[ -n "$BUNDLE" ] || { echo "need --bundle" >&2; exit 2; }
[ -n "$HOST" ] || { echo "need --host <orin-ip>" >&2; exit 2; }
[ -f "$BUNDLE" ] || { echo "bundle not found: $BUNDLE" >&2; exit 2; }

echo "[sync] $BUNDLE -> ${USER_NAME}@${HOST}:${DEST}"
ssh "${USER_NAME}@${HOST}" "mkdir -p ${DEST}"
rsync -avP --partial "$BUNDLE" "${USER_NAME}@${HOST}:${DEST}/"
BASE="$(basename "$BUNDLE")"

echo "[sync] unpacking on the Orin"
ssh "${USER_NAME}@${HOST}" "cd ${DEST} && tar -xzf ${BASE} && ls -la ${DEST}"
echo
echo "[sync] done. next, on the Orin:"
echo "  ssh ${USER_NAME}@${HOST}"
echo "  cd ${DEST}/a3_teleop_orin/a3_teleop_bridge && bash scripts/orin_preflight.sh"
