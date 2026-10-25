# Full sand-bed session guide

[`run_full_session_test.sh`](../../run_full_session_test.sh) runs the same collector and sweep
schedule as [`run_data_collector.sh`](../../run_data_collector.sh) (seed `20260713`, schedule
trials 0–7, five sweeps each), and also resets the sand with the trackway blower and records the
camera reference. See [`DATA_COLLECTION_GUIDE.md`](DATA_COLLECTION_GUIDE.md) for what the collector
records and how the reference is used.

```bash
./run_full_session_test.sh 2 0      # height 2 cm, schedule trial 0 only
./run_full_session_test.sh 2 all    # schedule trials 0 through 7
```

For each trial the script:

1. asks you to lift the robot off the sand bed and out of the camera view, then press Enter;
2. runs the blower (60 Hz for 10 s, ramp to 0 over 20 s) and waits until it reports 0 Hz;
3. on the first trial only, records the camera reference while the robot is still out of view;
4. asks you to put the robot back; the collector then asks for Enter and runs the five sweeps.

With `all`, the script pauses 3 s between trials. Each trial is saved in its own session folder.

Between trials the motors stay on and hold the leg at the home pose, so the next trial starts from
there. At the end of the session the script asks you to support the leg, then switches the
right-leg motors to idle (the same `/Gui_information` stop the collector sends on exit, sent three
times) and checks in `/robot_state` that both right-leg currents are below 0.1 A. If they are not,
it prints the `cansend` commands to idle the motors from the Pi (see below). Idle motors have no
torque, so the leg can drop.
Pressing Ctrl+C while the blower is running sends a stop command before exiting.

Set `SCHEDULE_SEED` to use a different schedule, and `PYTHON_BIN`, `TRACKWAY_SETUP` or
`TURTLE_DOMAIN_ID` to override the defaults at the top of the script. Print the sweeps in each
schedule trial with:

```bash
"$PYTHON_BIN" highlevel/terrain_manipulation/src/data_collectors/data_collector.py --print-schedule
```

## Before every session

### 1. Power on

- Turn on the trackway ebox (regular switch on, e-stop released) and wait about one minute. The
  blower services start automatically on `ROS_DOMAIN_ID=32`.
- Turn on the turtle robot, including motor power.

### 2. Set up CAN and start the controller (on the turtle Pi)

The controller reads and commands the right leg (ODrive nodes 2 and 3) on **`can1`**, at
1 Mbit/s. After a Pi reboot, both CAN interfaces are down and the port the motors are wired to is
named `can0`. If you skip this step, the collector still runs and every trajectory reports
`complete`, but the leg does not move and every position and current in `/robot_state` is 0.

On the Pi (`ssh ubuntu@192.168.10.20`), stop the controller and bring both interfaces up at
1 Mbit/s:

```bash
pkill -f turtle_high_controller
sudo ip link set can0 down
sudo ip link set can1 down
sudo ip link set can0 up type can bitrate 1000000
sudo ip link set can1 up type can bitrate 1000000
timeout 3 candump can1 | head -5
```

If the last command prints lines such as `can1  049  [8] ...` and `can1  069  [8] ...`, the
motors are already on `can1`. Otherwise swap the two names and check again:

```bash
sudo ip link set can0 down
sudo ip link set can1 down
sudo ip link set can0 name cantmp
sudo ip link set can1 name can0
sudo ip link set cantmp name can1
sudo ip link set can0 up type can bitrate 1000000
sudo ip link set can1 up type can bitrate 1000000
timeout 3 candump can1 | head -5
```

If neither interface shows `049`/`069` messages, the motor drivers have no power or a CAN cable is
unplugged.

Then start the controller, and leave this terminal open. It opens the CAN interfaces when it
starts, so always start it after the interfaces are up:

```bash
source /opt/ros/humble/setup.bash
source ~/roboland/dependencies/install/setup.bash
export ROS_DOMAIN_ID=2
ros2 run turtle_high_controller turtle_high_controller
```

### 3. Check both devices (on the lab PC)

```bash
ROS_DOMAIN_ID=2 ros2 topic echo /robot_state --once   # entries 4 and 5 under data: must not be 0.0
ROS_DOMAIN_ID=2 ros2 topic hz /robot_state            # expect several hundred Hz
ROS_DOMAIN_ID=32 ros2 service list | grep blower      # expect /trackway/blower_control
```

Entries 4 and 5 are the right adduction and right sweeping positions. If they are exactly `0.0`,
go back to step 2.

### 4. Run the session

```bash
./run_full_session_test.sh <height-cm> <0..7 | all>
```

Before lifting the robot for the blower, also clear anything else from the sand, such as cables
lying on it or tools.

## After the session

If the script warned that the motors may still be on, or the leg is still quivering, idle the
right-leg motors (ODrive nodes 2 and 3) on the Pi. Support the leg first:

```bash
cansend can1 047#01000000
cansend can1 067#01000000
```

Stopping the controller with Ctrl+C does not do this: the ODrives keep holding their last position
until they are told to go idle.

Other experiments on this robot expect the default CAN names. If you swapped the names in step 2,
swap them back on the Pi (or reboot it):

```bash
pkill -f turtle_high_controller
sudo ip link set can0 down
sudo ip link set can1 down
sudo ip link set can0 name cantmp
sudo ip link set can1 name can0
sudo ip link set cantmp name can1
```

## Troubleshooting

| Symptom | Cause / fix |
|---|---|
| Leg does not move, trajectories `complete`, `/robot_state` all 0 | CAN is down or the motors are on `can0`. Redo step 2. |
| `candump: read: Network is down` | That interface is down. Bring it up at 1 Mbit/s as in step 2. |
| Script stops at the blower check | The ebox is off or still booting, or the PC is not on the EEB network. |
| Script stops at the turtle check | The controller is not running, or not on `ROS_DOMAIN_ID=2`. |
| Leg quivers while holding a position | The ODrive position loop hunts at roughly 13–17 Hz with about ±0.5 A. This is present in earlier data too; it needs the gains of nodes 2 and 3 re-tuned. |
| Leg still quivers after the session | The motors were not set to idle. Use the `cansend` commands above. |
| `mocap` arrays empty | OptiTrack UDP forwarding to port 8000 is not running. |
