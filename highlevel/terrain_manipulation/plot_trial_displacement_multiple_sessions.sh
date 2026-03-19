#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

SESSIONS=(
  "$SCRIPT_DIR/data/session_20260317_153944"
  "$SCRIPT_DIR/data/session_20260317_154742"
  "$SCRIPT_DIR/data/session_20260317_155128"
  "$SCRIPT_DIR/data/session_20260317_155550"
  "$SCRIPT_DIR/data/session_20260317_160026"
  "$SCRIPT_DIR/data/session_20260317_160410"
)


# Default MPLCONFIGDIR avoids matplotlib cache permission issues.
export MPLCONFIGDIR="${MPLCONFIGDIR:-/tmp}"

for session_dir in "${SESSIONS[@]}"; do
  echo "Rendering: ${session_dir}"
  python3 "$SCRIPT_DIR/render_rgbd_force_mocap_sync_video.py" \
    "$session_dir" \
    "$@"
done
