#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
COLLECTOR_SCRIPT="$SCRIPT_DIR/highlevel/terrain_manipulation/src/data_collectors/data_collector.py"
DEFAULT_PYTHON="/home/parnia/anaconda3/envs/Turtle_TM/bin/python"
PYTHON_BIN="${PYTHON_BIN:-$DEFAULT_PYTHON}"

SCHEDULE_SEED=20260713

if [[ ! -x "$PYTHON_BIN" ]]; then
  echo "Error: Python interpreter is not executable: $PYTHON_BIN" >&2
  echo "Set PYTHON_BIN=/path/to/python to use a different environment." >&2
  exit 1
fi

if [[ $# -ne 2 ]]; then
  echo "Usage: $0 <height-cm> <schedule-trial-index 0..7>" >&2
  echo "Example: $0 5 0" >&2
  exit 1
fi

HEIGHT_CM="$1"
TRIAL_INDEX="$2"
if ! [[ "$HEIGHT_CM" =~ ^-?[0-9]+([.][0-9]+)?$ ]]; then
  echo "Error: height must be a number in cm." >&2
  exit 1
fi

if ! [[ "$TRIAL_INDEX" =~ ^[0-7]$ ]]; then
  echo "Error: schedule trial index must be an integer from 0 through 7." >&2
  exit 1
fi

echo "Running data collector"
echo "Python: ${PYTHON_BIN}"
echo "Height: ${HEIGHT_CM} cm"
echo "Schedule seed: ${SCHEDULE_SEED}"
echo "Schedule trial index: ${TRIAL_INDEX}"

"$PYTHON_BIN" "$COLLECTOR_SCRIPT" \
  --trials 1 \
  --height-cm "$HEIGHT_CM" \
  --schedule-seed "$SCHEDULE_SEED" \
  --schedule-trial-index "$TRIAL_INDEX"
