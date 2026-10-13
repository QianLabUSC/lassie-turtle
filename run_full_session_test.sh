#!/usr/bin/env bash
# Run a full session: reset the sand with the trackway blower before every
# trial, record the RealSense camera reference once (after the first reset),
# and run the turtle data collector on one or all eight trials of the
# deterministic sweep schedule (same schedule as lassie-turtle's
# run_data_collector.sh).
#
# The robot must be lifted off the trackway while the blower runs, so the
# script waits for the operator to press Enter before each sand reset.
set -euo pipefail

# This script lives in the lassie-turtle checkout that holds the data collector.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TURTLE_WORKSPACE="${TURTLE_WORKSPACE:-$SCRIPT_DIR}"
TRACKWAY_SETUP="${TRACKWAY_SETUP:-$HOME/Projects/trackway/highlevel/install/setup.bash}"
TRACKWAY_DOMAIN_ID=32
TURTLE_DOMAIN_ID="${TURTLE_DOMAIN_ID:-2}"
PYTHON_BIN="${PYTHON_BIN:-/home/parnia/anaconda3/envs/Turtle_TM/bin/python}"
COLLECTOR_SCRIPT="$TURTLE_WORKSPACE/highlevel/terrain_manipulation/src/data_collectors/data_collector.py"
REFERENCE_SCRIPT="$TURTLE_WORKSPACE/highlevel/terrain_manipulation/src/data_collectors/record_realsense_reference.py"
# Same seed as run_data_collector.sh; schedule trial indices are 0..7.
SCHEDULE_SEED="${SCHEDULE_SEED:-20260713}"
NUM_SCHEDULE_TRIALS=8
PAUSE_BETWEEN_TRIALS_S=3

# Same parameters as blower_sand_reset in trackway's CLI_controller.py:
# run at 60 Hz for 10 s, then ramp down to 0 Hz over 20 s.
BLOWER_HZ=60.0
BLOWER_DURATION_S=10.0
BLOWER_RAMP_S=20.0
BLOWER_EXTRA_WAIT_S=30   # how long to keep polling for 0 Hz after the nominal end

usage() {
  echo "Usage: $0 <height-cm> <schedule-trial-index 0..7 | all>" >&2
  echo "Example: $0 4 0      # schedule trial 0 only" >&2
  echo "         $0 4 all    # schedule trials 0 through 7" >&2
  exit 1
}

[[ $# -eq 2 ]] || usage
HEIGHT_CM="$1"
[[ "$HEIGHT_CM" =~ ^-?[0-9]+([.][0-9]+)?$ ]] || usage
if [[ "$2" == all ]]; then
  TRIAL_INDICES=($(seq 0 $((NUM_SCHEDULE_TRIALS - 1))))
elif [[ "$2" =~ ^[0-7]$ ]]; then
  TRIAL_INDICES=("$2")
else
  usage
fi

if [[ ! -f "$COLLECTOR_SCRIPT" ]]; then
  echo "Error: data collector not found at $COLLECTOR_SCRIPT" >&2
  echo "Set TURTLE_WORKSPACE=/path/to/lassie-turtle checkout." >&2
  exit 1
fi

if [[ ! -x "$PYTHON_BIN" ]]; then
  echo "Error: Python interpreter is not executable: $PYTHON_BIN" >&2
  echo "Set PYTHON_BIN=/path/to/python to use a different environment." >&2
  exit 1
fi

if [[ ! -f "$TRACKWAY_SETUP" ]]; then
  echo "Error: trackway workspace not found at $TRACKWAY_SETUP" >&2
  echo "Build it with ~/Projects/trackway/highlevel/build.sh or set TRACKWAY_SETUP." >&2
  exit 1
fi

# The ROS setup scripts reference unset variables, so relax -u while sourcing.
set +u
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
source "$TRACKWAY_SETUP"
set -u

blower_call() {
  local method="$1" data="$2"
  ROS_DOMAIN_ID=$TRACKWAY_DOMAIN_ID timeout 15 ros2 service call \
    /trackway/blower_control trackway_interface/srv/BlowerControl \
    "{method: '$method', data: $data}" >/dev/null
}

blower_speed_hz() {
  ROS_DOMAIN_ID=$TRACKWAY_DOMAIN_ID timeout 5 ros2 topic echo --once \
    /trackway/blower_status 2>/dev/null | awk '/blower_speed_hz/ {print $2}'
}

blower_stop() {
  echo "Stopping blower..."
  blower_call stop "[0.0, 0.0, 0.0]" || echo "Warning: blower stop request failed." >&2
}

BLOWER_ACTIVE=0
on_interrupt() {
  if (( BLOWER_ACTIVE )); then
    blower_stop
  fi
  exit 130
}
trap on_interrupt INT TERM

reset_sand() {
  BLOWER_ACTIVE=1
  if ! blower_call user_defined "[$BLOWER_HZ, $BLOWER_DURATION_S, $BLOWER_RAMP_S]"; then
    echo "Error: blower service call failed. Is the ebox on and the PC on the EEB network?" >&2
    exit 1
  fi
  local nominal_s
  nominal_s=$(printf '%.0f' "$(echo "$BLOWER_DURATION_S + $BLOWER_RAMP_S" | bc)")
  echo "Blower running (${BLOWER_HZ} Hz for ${BLOWER_DURATION_S} s, ramp ${BLOWER_RAMP_S} s)..."
  sleep "$nominal_s"

  # The service returns immediately, so confirm the blower actually reached 0 Hz.
  local deadline=$((SECONDS + BLOWER_EXTRA_WAIT_S)) speed
  while true; do
    speed="$(blower_speed_hz)"
    if [[ -n "$speed" ]] && awk -v s="$speed" 'BEGIN { exit !(s <= 0.0) }'; then
      break
    fi
    if (( SECONDS >= deadline )); then
      echo "Error: blower did not report 0 Hz (last reading: '${speed:-none}')." >&2
      blower_stop
      exit 1
    fi
    sleep 1
  done
  BLOWER_ACTIVE=0
  echo "Sand reset done; blower stopped."
}

echo "Checking that the trackway blower is reachable (ROS_DOMAIN_ID=$TRACKWAY_DOMAIN_ID)..."
if ! ROS_DOMAIN_ID=$TRACKWAY_DOMAIN_ID timeout 10 ros2 service list 2>/dev/null \
    | grep -qx /trackway/blower_control; then
  echo "Error: /trackway/blower_control not found. Turn on the ebox and wait ~1 minute." >&2
  exit 1
fi

echo "Checking that the turtle controller is running (ROS_DOMAIN_ID=$TURTLE_DOMAIN_ID)..."
if ! ROS_DOMAIN_ID=$TURTLE_DOMAIN_ID timeout 10 ros2 topic info /trajectory_points 2>/dev/null \
    | grep -Eq "Subscription count: [1-9]"; then
  echo "Error: nothing subscribes to /trajectory_points. Start turtle_high_controller on the Pi" >&2
  echo "and make sure its ROS_DOMAIN_ID matches TURTLE_DOMAIN_ID=$TURTLE_DOMAIN_ID." >&2
  exit 1
fi

NUM_TRIALS=${#TRIAL_INDICES[@]}
for (( n = 0; n < NUM_TRIALS; n++ )); do
  trial_index=${TRIAL_INDICES[n]}
  if (( n > 0 )); then
    echo "Pausing ${PAUSE_BETWEEN_TRIALS_S} s before the next trial..."
    sleep "$PAUSE_BETWEEN_TRIALS_S"
  fi

  echo
  echo "=== Trial $((n + 1))/$NUM_TRIALS: schedule trial $trial_index (height ${HEIGHT_CM} cm, seed $SCHEDULE_SEED) ==="
  read -r -p "Lift the robot off the trackway, then press Enter to reset the sand... "
  reset_sand

  if (( n == 0 )); then
    echo
    echo "=== Recording RealSense camera reference ==="
    echo "Keep the robot out of the camera view."
    "$PYTHON_BIN" "$REFERENCE_SCRIPT" --preview
  fi

  echo
  echo "Put the robot in place. The data collector will ask you to press Enter to start."
  ROS_DOMAIN_ID=$TURTLE_DOMAIN_ID "$PYTHON_BIN" "$COLLECTOR_SCRIPT" \
    --trials 1 \
    --height-cm "$HEIGHT_CM" \
    --schedule-seed "$SCHEDULE_SEED" \
    --schedule-trial-index "$trial_index"
done

echo
echo "Session complete: schedule trial(s) ${TRIAL_INDICES[*]} at ${HEIGHT_CM} cm."
