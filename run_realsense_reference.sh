#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REFERENCE_SCRIPT="$SCRIPT_DIR/highlevel/terrain_manipulation/src/data_collectors/record_realsense_reference.py"
DEFAULT_PYTHON="/home/parnia/anaconda3/envs/Turtle_TM/bin/python"
PYTHON_BIN="${PYTHON_BIN:-$DEFAULT_PYTHON}"

if [[ ! -x "$PYTHON_BIN" ]]; then
  echo "Error: Python interpreter is not executable: $PYTHON_BIN" >&2
  echo "Set PYTHON_BIN=/path/to/python to use a different environment." >&2
  exit 1
fi

echo "Recording RealSense reference"
echo "Python: ${PYTHON_BIN}"

"$PYTHON_BIN" "$REFERENCE_SCRIPT" "$@"
