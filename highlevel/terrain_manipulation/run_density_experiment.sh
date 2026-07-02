#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
COLLECTOR_SCRIPT="$SCRIPT_DIR/distributed_data_collector_highrate_density_experiment.py"
DEFAULT_HEIGHT_CM="$(awk -F= '/^HEIGHT_CM[[:space:]]*=/{gsub(/[[:space:]]/, "", $2); print $2; exit}' "$COLLECTOR_SCRIPT")"

if [[ -z "$DEFAULT_HEIGHT_CM" ]]; then
  echo "Error: could not read HEIGHT_CM from $COLLECTOR_SCRIPT" >&2
  exit 1
fi

HEIGHT_CM="${1:-${HEIGHT_CM:-$DEFAULT_HEIGHT_CM}}"
TRIALS="${2:-${TRIALS:-5}}"
DEFAULT_PYTHON="/home/parnia/anaconda3/envs/Turtle_TM/bin/python"
PYTHON_BIN="${PYTHON_BIN:-$DEFAULT_PYTHON}"

if [[ ! -x "$PYTHON_BIN" ]]; then
  echo "Error: Python interpreter is not executable: $PYTHON_BIN" >&2
  echo "Set PYTHON_BIN=/path/to/python to use a different environment." >&2
  exit 1
fi

echo "Running density experiment: height=${HEIGHT_CM} cm, trials=${TRIALS}"
echo "Python: ${PYTHON_BIN}"

"$PYTHON_BIN" "$COLLECTOR_SCRIPT" \
  --height-cm "$HEIGHT_CM" \
  --trials "$TRIALS" \
  "${@:3}"
