# Trajectory Files

This directory contains JSON trajectory files for the turtle robot.

## File Format

Each JSON file should have the following structure:

```json
{
  "name": "trajectory_name",
  "resolution": 0.01,
  "waypoints": [
    x1, y1, v1,
    x2, y2, v2,
    ...
  ]
}
```

- `name`: Descriptive name for the trajectory
- `resolution`: Distance resolution in meters (controls number of interpolated points)
- `waypoints`: Flat array of [x, y, velocity] triplets

## Usage

```bash
# From the highlevel directory
python3 trajectory_interpolator.py trajectories/circle.json
```

## Example Trajectories

- `circle.json` - Circular path trajectory

