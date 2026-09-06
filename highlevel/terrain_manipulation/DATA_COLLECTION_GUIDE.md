# Terrain-manipulation data collection guide

This guide describes the current sweep-data collection pipeline for the LASSIE turtle. It is intended for the experiment in which the right leg is placed at a selected penetration/depth configuration and then executes one or more controlled sweeps while RGB-D, robot telemetry, and motion-capture data are recorded.

The active entry point is [`run_data_collector.sh`](../../../run_data_collector.sh). It runs [`src/data_collectors/data_collector.py`](src/data_collectors/data_collector.py), which is the current reference-corrected, sequence-aware collector. Do not start collection with `distributed_data_collector_highrate.py`, `distributed_data_collector_highrate_density_experiment.py`, or `deprecated_distributed_data_collector.py` unless reproducing their older fixed-trajectory experiments. `run_density_experiment.sh` belongs specifically to the density experiment and invokes the older density collector.

## What the collector records

For every trial, the collector:

1. sends each requested right-leg trajectory to `/trajectory_points`;
2. records both RealSense RGB streams and depth streams at 848 x 480 and 30 Hz;
3. logs every received `/robot_state` sample, then saves a nearest-time copy aligned to camera 0 frames;
4. listens for OptiTrack UDP data on port 8000 and saves both its native-rate and camera-aligned forms;
5. waits three seconds after the final requested trajectory completes, so the sand can settle; and
6. writes one `trial_N.npy` payload plus session metadata.

The low-level controller must already be running and publishing `/robot_state` and `/trajectory_complete`. The collector is a recorder and trajectory publisher; it does not launch or configure the robot controller. OptiTrack forwarding must also already be running if its measurements are required. The collector will still start if the OptiTrack listener receives no packets, but the resulting mocap arrays will be empty, so confirm its data stream before treating a run as valid.

## Required camera reference: do this before a collection session

The saved depth arrays are not absolute camera-to-sand distances. Before collecting a new session, create a reference after the camera positions are final and the sand bed has been flattened. Raise or remove the robot/tool from the camera view first.

From `lassie-turtle`, using the same Python environment as collection:

```bash
PYTHON_BIN=/path/to/Turtle_TM/bin/python
"$PYTHON_BIN" highlevel/terrain_manipulation/src/data_collectors/record_realsense_reference.py --preview
```

By default, the reference program expects two connected RealSense cameras. It discards 30 warm-up frames, takes the median of 120 subsequent depth frames, fits the flat sand surface with RANSAC, and stores one plane per camera under `highlevel/terrain_manipulation/output/references/reference_<timestamp>/`. It also updates `output/references/latest_reference_session.json`.

During collection, `data_collector.py` loads that latest reference unless `--reference-session` names another reference folder or its metadata JSON. For every live depth pixel it projects the measured depth into camera coordinates and saves the signed distance to the fitted reference sand plane in millimetres. Thus zero means the original nominal sand plane; positive and negative values represent surface change relative to that plane. The camera pose, serial number, resolution, and reference must match the collection setup. Re-record the reference whenever a camera moves, the stream configuration changes, or the initial flat-sand condition is re-established differently.

## Sweep representation

Trajectory names have this form:

```text
<adduction_deg>_<sweep_deg>_<speed_rad_s>_<front|back>
```

For example, `90_30_2_back` means: begin at the centered sweeping angle, pre-position on the front side, add 90 degrees to the adduction home coordinate to lower the right leg, sweep backward through 30 degrees at 2 rad/s, lift, and return to the centered home pose. `front` does the symmetric operation: it pre-positions to the back endpoint, then sweeps toward the front.

The nominal controller coordinates are defined in [`src/data_collectors/trajectory.py`](src/data_collectors/trajectory.py): adduction home is 0.5235987756 rad and sweep center is -0.53 rad. A trajectory always performs the full safe sequence

```text
home/center -> start side -> lower -> center -> end side -> lift -> home/center
```

so the named adduction displacement defines the inserted configuration and the named sweep displacement defines the angular sweep width. The module predefines combinations of adduction `{0, 45, 90}` degrees, sweep `{30, 45, 90}` degrees, and directions `{front, back}` at 2 rad/s, but the parser accepts any numeric values in the same name format. A decimal speed is written with `p`, such as `1p5` for 1.5 rad/s.

## Running the normal collector

From the `lassie-turtle` directory:

```bash
./run_data_collector.sh
```

The launcher currently uses `/home/parnia/anaconda3/envs/Turtle_TM/bin/python`. Override it if the environment is elsewhere:

```bash
PYTHON_BIN=/path/to/Turtle_TM/bin/python ./run_data_collector.sh
```

The script's default experiment is:

```text
90_30_2_back:10
```

That is one trial containing ten sequential executions of the same 90-degree adduction, 30-degree backward sweep. The collector's own defaults, when `data_collector.py` is run directly, differ: one trial at height `3` cm with the single trajectory `45_30_2_front:1`. The launcher intentionally replaces that trajectory default with the ten-sweep sequence above.

The program prints the chosen reference, session path, height, and trajectory sequence, then waits for Enter. Verify the robot is clear to move and press Enter once. Use Ctrl-C to abort; the collector then publishes a final stop message to `/Gui_information` and writes the trial payload accumulated up to the interruption, so mark interrupted trials during later curation.

### Common commands

One trial with the launcher default:

```bash
./run_data_collector.sh
```

Three independent trials, each with the same ten backward sweeps:

```bash
./run_data_collector.sh --trials 3
```

Set the physical experiment height only for naming and metadata:

```bash
./run_data_collector.sh --height-cm 4
```

Use a particular reference rather than the latest one:

```bash
./run_data_collector.sh --reference-session highlevel/terrain_manipulation/output/references/reference_YYYYMMDD_HHMMSS
```

Record RGB MP4 previews in addition to the arrays:

```bash
./run_data_collector.sh --save-rgb-mp4
```

Apply a clockwise incline correction to saved OptiTrack y-z coordinates and orientations:

```bash
./run_data_collector.sh --incline-deg 5
```

## Designing sweep sequences

Pass `--trajectory-sequence` followed by one or more `NAME:COUNT` blocks. A missing `:COUNT` means one execution. All blocks run consecutively inside *each* trial; camera and telemetry recording is continuous across those blocks, and each command has its own timestamp and completion status in `trajectory_runs`.

For example, a trial with one shallow forward sweep followed by two deeper backward sweeps is:

```bash
PYTHON_BIN=/path/to/Turtle_TM/bin/python \
  ./run_data_collector.sh \
  --trajectory-sequence 45_30_2_front:1 90_45_1p5_back:2
```

The present collector does **not** randomly sample or shuffle trajectories. It executes the blocks in exactly the typed order and records that request. Therefore, for a randomized-following-sweep design, create the randomized order before launching and pass the resulting explicit list. Preserve that exact launch command with the session notes; the ordered request is also stored in each trial and session metadata. Do not claim a run was random merely because it contains several sweep types.

The current trajectory API varies penetration/adduction, sweep width, direction, and speed. It does not independently expose a separate starting-position parameter: each trajectory's initial side is determined by `front` or `back`, and its insertion configuration by the adduction displacement. For new kinds of initial conditions, add an explicit, named trajectory generator in `trajectory.py` rather than silently changing a current label's meaning.

## Output layout and provenance

A reference-linked collection is written below:

```text
highlevel/terrain_manipulation/data/
  <reference_session_name>/
    reference/                         # copied reference package, created once
    sessions/
      session_<timestamp>_height_<X>cm/
        metadata.json
        trial_1.npy
        trial_2.npy                    # when --trials is greater than one
        trial_1_rgb_0.mp4              # only with --save-rgb-mp4
        trial_1_rgb_1.mp4
```

Each trial payload contains `rgb_0`, `depth_0`, `camera_time_0`, and the equivalent camera-1 arrays; `robot_state_raw` and camera-aligned `robot_state`; `mocap_raw` and camera-aligned `mocap`; the reference paths and camera serials; and complete trajectory provenance. In particular, use `trajectory_sequence_request` to see the requested blocks and `trajectory_runs` to see every individual command, its controller points, command time, completion time, and status. `trajectory_name` and `trajectory_points` are retained for compatibility and refer only to the first sequence element, not the whole trial.

`metadata.json` records planned and completed trial counts, height, camera information, reference location, dwell duration, OptiTrack settings, and the requested sequence. Inspect a saved session with:

```bash
PYTHON_BIN=/path/to/Turtle_TM/bin/python \
  "$PYTHON_BIN" highlevel/terrain_manipulation/src/utils/read_trial_data.py \
  highlevel/terrain_manipulation/data/<reference>/sessions/<session>
```

## Pre-flight checklist

- Low-level robot controller is running; ROS 2 networking/domain configuration matches the collection computer.
- Two RealSense cameras are connected, recognized, stable at the reference resolution, and have not moved since the reference was recorded.
- Bed is flat and the tool is out of view while creating the reference; `latest_reference_session.json` points to that reference.
- The physical height and any incline correction passed at launch match the experimental setup.
- OptiTrack UDP forwarding is active if displacement/orientation data are required.
- The requested sweep names, order, and repeat counts have been reviewed before pressing Enter.
- Sufficient disk space is available for full RGB-D arrays; use `--save-rgb-mp4` only when the additional video is needed.

## Files to modify or avoid

- Modify [`src/data_collectors/trajectory.py`](src/data_collectors/trajectory.py) to add or change the current named sweep family.
- Modify [`run_data_collector.sh`](../../../run_data_collector.sh) only when changing the lab's convenient default sequence or Python path.
- Use [`src/data_collectors/record_realsense_reference.py`](src/data_collectors/record_realsense_reference.py) whenever a fresh depth reference is required.
- Treat [`src/data_collectors/trajectory_dictionary.py`](src/data_collectors/trajectory_dictionary.py) as an untracked exploratory file, not the module used by the collector. The live collector imports `trajectory.py`.
- Leave the older distributed collectors and `run_density_experiment.sh` alone unless deliberately reproducing their specific historical study.
