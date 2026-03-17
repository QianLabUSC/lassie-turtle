#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

SESSIONS=(
  "$SCRIPT_DIR/data/session_20260316_132543"
  "$SCRIPT_DIR/data/session_20260316_142942"
  "$SCRIPT_DIR/data/session_20260316_144047"
  "$SCRIPT_DIR/data/session_20260316_145733"
  "$SCRIPT_DIR/data/session_20260316_150341"
  "$SCRIPT_DIR/data/session_20260316_152118"
  "$SCRIPT_DIR/data/session_20260316_152712"
)

# Default MPLCONFIGDIR avoids matplotlib cache permission issues.
export MPLCONFIGDIR="${MPLCONFIGDIR:-/tmp}"

for session_dir in "${SESSIONS[@]}"; do
  echo "Rendering: ${session_dir}"
  python3 "$SCRIPT_DIR/render_rgbd_force_mocap_sync_video.py" \
    "$session_dir" \
    "$@"
done
