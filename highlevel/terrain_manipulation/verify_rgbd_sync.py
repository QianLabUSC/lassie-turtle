#!/usr/bin/env python3
"""Verify alignment between rgbd_timestamps.npy and robot_state.csv."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path
from typing import List

import numpy as np

try:
    import matplotlib.pyplot as plt
except ModuleNotFoundError:
    plt = None


def _read_robot_times(csv_path: Path) -> List[float]:
    with csv_path.open("r", newline="") as fh:
        reader = csv.reader(fh)
        rows = list(reader)
    if len(rows) < 3:
        raise ValueError("robot_state.csv is missing data rows")
    data_rows = rows[3:]
    times = []
    for row in data_rows:
        if not row:
            continue
        try:
            times.append(float(row[0]))
        except ValueError:
            continue
    return times


def _describe_series(name: str, values: np.ndarray) -> None:
    if len(values) < 2:
        print(f"{name}: {len(values)} samples")
        return
    diffs = np.diff(values)
    print(
        f"{name}: {len(values)} samples, "
        f"dt mean {diffs.mean():.4f}s, "
        f"dt std {diffs.std():.4f}s, "
        f"fps ~ {1.0 / diffs.mean():.2f}"
    )


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("session_dir", type=Path, help="Session folder containing rgbd_timestamps.npy and robot_state.csv")
    args = ap.parse_args()

    session_dir = Path(str(args.session_dir).strip())
    if session_dir.is_file():
        session_dir = session_dir.parent
    ts_path = session_dir / "rgbd_timestamps.npy"
    csv_path = session_dir / "robot_state.csv"

    if not ts_path.exists():
        raise SystemExit(f"Missing {ts_path}")
    if not csv_path.exists():
        raise SystemExit(f"Missing {csv_path}")

    rgbd_ts = np.load(ts_path)
    robot_times = np.asarray(_read_robot_times(csv_path))

    print(f"rgbd_timestamps: {len(rgbd_ts)}")
    print(f"robot_state rows: {len(robot_times)}")

    _describe_series("rgbd", rgbd_ts)
    _describe_series("robot_state", robot_times)

    if len(rgbd_ts) == 0 or len(robot_times) == 0:
        return

    n = min(len(rgbd_ts), len(robot_times))
    diffs = robot_times[:n] - rgbd_ts[:n]
    print(
        "alignment (robot_time - rgbd_time): "
        f"mean {diffs.mean():.4f}s, "
        f"std {diffs.std():.4f}s, "
        f"min {diffs.min():.4f}s, "
        f"max {diffs.max():.4f}s"
    )

    if plt is not None:
        fig, ax = plt.subplots()
        ax.plot(diffs, linewidth=1.0)
        ax.axhline(0.0, color="black", linewidth=0.8)
        ax.set_title("Robot vs RGB-D Timestamp Offset")
        ax.set_xlabel("Sample index")
        ax.set_ylabel("robot_time - rgbd_time (s)")
        fig.tight_layout()
        out_path = session_dir / "sync_plot.png"
        fig.savefig(out_path, dpi=150)
        print(f"Saved plot to {out_path}")
    else:
        print("[WARN] matplotlib not installed; skipping plot.")

    if len(rgbd_ts) != len(robot_times):
        print("[WARN] Different sample counts; alignment stats use the shorter length.")


if __name__ == "__main__":
    main()
