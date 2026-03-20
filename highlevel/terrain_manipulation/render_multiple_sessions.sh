#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# For recollected sessions, keep COM correction off by default.
# Override at runtime if needed, e.g.:
#   COM_OFFSET_Y_M=-0.00375 bash render_multiple_sessions.sh
COM_OFFSET_Y_M="${COM_OFFSET_Y_M:-0.0}"

STEEL_SESSIONS=(
  "$SCRIPT_DIR/data/session_20260319_134621"
  "$SCRIPT_DIR/data/session_20260319_135104"
  "$SCRIPT_DIR/data/session_20260319_140118"
  "$SCRIPT_DIR/data/session_20260319_140705"
  "$SCRIPT_DIR/data/session_20260319_141232"
  "$SCRIPT_DIR/data/session_20260319_141641"
)

RESIN_SESSIONS=(
  "$SCRIPT_DIR/data/session_20260319_142653"
  "$SCRIPT_DIR/data/session_20260319_145057"
  "$SCRIPT_DIR/data/session_20260319_145447"
  "$SCRIPT_DIR/data/session_20260319_145955"
  "$SCRIPT_DIR/data/session_20260319_151418"
  "$SCRIPT_DIR/data/session_20260319_151845"
  "$SCRIPT_DIR/data/session_20260319_152329"
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
    --depth-min-m 0.4 --depth-max-m 0.7 \
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
    --depth-min-m 0.4 --depth-max-m 0.7 \
    --compare-output \
    "$@"
done
