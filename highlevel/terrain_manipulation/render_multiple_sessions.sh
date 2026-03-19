#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# NOTE:
# Steel sessions from 2026-03-14 and resin sessions from 2026-03-16
# used an incorrect Motive pivot/COM definition and require correction.
COM_OFFSET_Y_M="-0.00375"

STEEL_SESSIONS=(
  "$SCRIPT_DIR/data/session_20260314_122949"
  "$SCRIPT_DIR/data/session_20260314_123416"
  "$SCRIPT_DIR/data/session_20260314_123744"
  "$SCRIPT_DIR/data/session_20260314_124126"
  "$SCRIPT_DIR/data/session_20260314_124658"
  "$SCRIPT_DIR/data/session_20260314_125646"
)

RESIN_SESSIONS=(
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

for session_dir in "${STEEL_SESSIONS[@]}"; do
  echo "Rendering steel: ${session_dir}"
  python3 "$SCRIPT_DIR/render_rgbd_force_mocap_sync_video.py" \
    "$session_dir" \
    --mode experiment \
    --half-sphere steel \
    --com-offset-y-m "$COM_OFFSET_Y_M" \
    --depth-min-m 0.4 --depth-max-m 0.7
    --compare-output \
    "$@"
done

for session_dir in "${RESIN_SESSIONS[@]}"; do
  echo "Rendering resin: ${session_dir}"
  python3 "$SCRIPT_DIR/render_rgbd_force_mocap_sync_video.py" \
    "$session_dir" \
    --mode experiment \
    --half-sphere resin \
    --com-offset-y-m "$COM_OFFSET_Y_M" \
    --depth-min-m 0.4 --depth-max-m 0.7
    --compare-output \
    "$@"
done
