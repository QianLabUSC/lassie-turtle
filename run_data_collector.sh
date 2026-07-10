#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
COLLECTOR_SCRIPT="$SCRIPT_DIR/highlevel/terrain_manipulation/src/data_collectors/data_collector.py"
DEFAULT_PYTHON="/home/parnia/anaconda3/envs/Turtle_TM/bin/python"
PYTHON_BIN="${PYTHON_BIN:-$DEFAULT_PYTHON}"
TRAJECTORY_SEQUENCE=("90_30_2_back:10")

if [[ ! -x "$PYTHON_BIN" ]]; then
  echo "Error: Python interpreter is not executable: $PYTHON_BIN" >&2
  echo "Set PYTHON_BIN=/path/to/python to use a different environment." >&2
  exit 1
fi

echo "Running data collector"
echo "Python: ${PYTHON_BIN}"

"$PYTHON_BIN" "$COLLECTOR_SCRIPT" --trajectory-sequence "${TRAJECTORY_SEQUENCE[@]}" "$@"
