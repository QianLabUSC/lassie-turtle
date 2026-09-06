"""Named right-leg sweep trajectories for terrain manipulation trials.

Each trajectory is a flattened list of waypoints:

    [right_adduction_rad, right_sweeping_rad, speed_rad_s, ...]

Labels use degree units:

    a{adduction displacement}_s{sweep displacement}_{front|back}

For example, ``a045_s090_front`` starts at the centered sweeping angle,
pre-positions to the back side, moves the leg down by adding 45 degrees to
right-adduction, sweeps forward through 90 degrees, lifts back up, and returns
to center.
"""

from __future__ import annotations

import math
from typing import Dict, List, Literal, Tuple


TRAJ_SPEED_RAD_S = 2.0

# Controller coordinates from the current fixed trajectory.
ADDUCTION_HOME_RAD = 0.5235987756
SWEEP_CENTER_RAD = -0.53

ADDUCTION_DISPLACEMENTS_DEG = (0, 45, 90)
SWEEP_DISPLACEMENTS_DEG = (30, 45, 90)
SWEEP_DIRECTIONS = ("front", "back")

SweepDirection = Literal["front", "back"]
Waypoint = Tuple[float, float]


def _flatten_waypoints(waypoints: List[Waypoint]) -> List[float]:
    trajectory: List[float] = []
    for adduction_rad, sweeping_rad in waypoints:
        trajectory.extend([adduction_rad, sweeping_rad, TRAJ_SPEED_RAD_S])
    return trajectory


def make_sweep_trajectory(
    adduction_displacement_deg: int,
    sweep_displacement_deg: int,
    direction: SweepDirection,
) -> List[float]:
    """Build one centered sweep trajectory in controller coordinates."""
    adduction_home = ADDUCTION_HOME_RAD
    adduction_down = adduction_home + math.radians(adduction_displacement_deg)

    half_sweep = math.radians(sweep_displacement_deg) / 2.0
    sweep_back = SWEEP_CENTER_RAD - half_sweep
    sweep_front = SWEEP_CENTER_RAD + half_sweep

    if direction == "front":
        sweep_start = sweep_back
        sweep_end = sweep_front
    elif direction == "back":
        sweep_start = sweep_front
        sweep_end = sweep_back
    else:
        raise ValueError(f"unknown sweep direction: {direction}")

    waypoints = [
        (adduction_home, SWEEP_CENTER_RAD),
        (adduction_home, sweep_start),
        (adduction_down, sweep_start),
        (adduction_down, SWEEP_CENTER_RAD),
        (adduction_down, sweep_end),
        (adduction_home, sweep_end),
        (adduction_home, SWEEP_CENTER_RAD),
    ]
    return _flatten_waypoints(waypoints)


TRAJECTORIES: Dict[str, List[float]] = {
    f"a{adduction_deg:03d}_s{sweep_deg:03d}_{direction}": make_sweep_trajectory(
        adduction_displacement_deg=adduction_deg,
        sweep_displacement_deg=sweep_deg,
        direction=direction,  # type: ignore[arg-type]
    )
    for adduction_deg in ADDUCTION_DISPLACEMENTS_DEG
    for sweep_deg in SWEEP_DISPLACEMENTS_DEG
    for direction in SWEEP_DIRECTIONS
}


TRAJECTORY_REPRESENTATIONS: Dict[str, Tuple[int, int, str]] = {
    name: (
        int(name[1:4]),
        int(name[6:9]),
        name.rsplit("_", maxsplit=1)[1],
    )
    for name in TRAJECTORIES
}

