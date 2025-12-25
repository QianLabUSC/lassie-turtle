#!/usr/bin/env python3
"""Validate RGB vs robot_state timing via motion/torque onset."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path
from typing import List, Tuple

import cv2
import numpy as np


def _read_robot_times_and_signal(csv_path: Path, column: str) -> Tuple[np.ndarray, np.ndarray]:
    with csv_path.open("r", newline="") as fh:
        reader = csv.reader(fh)
        rows = list(reader)
    if len(rows) < 4:
        raise ValueError("robot_state.csv is missing data rows")
    header = rows[2]
    data_rows = rows[3:]
    try:
        col_idx = header.index(column)
    except ValueError as exc:
        raise ValueError(f"{column} column not found in robot_state.csv") from exc

    times: List[float] = []
    values: List[float] = []
    for row in data_rows:
        if not row:
            continue
        try:
            times.append(float(row[0]))
            values.append(float(row[col_idx]))
        except ValueError:
            continue
    return np.asarray(times), np.asarray(values)


def _detect_first_spike(values: np.ndarray, z: float) -> int:
    if len(values) < 3:
        return 0
    dv = np.abs(np.diff(values))
    mean = float(np.mean(dv))
    std = float(np.std(dv))
    threshold = mean + z * std
    idxs = np.where(dv > threshold)[0]
    if idxs.size == 0:
        return 0
    return int(idxs[0] + 1)


def _detect_motion_onset(frames: np.ndarray, z: float) -> int:
    if len(frames) < 3:
        return 0
    gray = [cv2.cvtColor(f, cv2.COLOR_BGR2GRAY) for f in frames]
    diffs = []
    for i in range(1, len(gray)):
        diff = cv2.absdiff(gray[i], gray[i - 1])
        diffs.append(float(np.mean(diff)))
    diffs_arr = np.asarray(diffs)
    mean = float(np.mean(diffs_arr))
    std = float(np.std(diffs_arr))
    threshold = mean + z * std
    idxs = np.where(diffs_arr > threshold)[0]
    if idxs.size == 0:
        return 0
    return int(idxs[0] + 1)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("session_dir", type=Path, help="Session folder")
    ap.add_argument("--signal", default="rightadduction_curr", help="robot_state column to use")
    ap.add_argument("--z", type=float, default=5.0, help="Z-score multiplier for spike detection")
    args = ap.parse_args()

    session_dir = Path(str(args.session_dir).strip())
    if session_dir.is_file():
        session_dir = session_dir.parent

    rgb_path = session_dir / "rgb_raw.npy"
    ts_path = session_dir / "rgbd_timestamps.npy"
    csv_path = session_dir / "robot_state.csv"

    for path in (rgb_path, ts_path, csv_path):
        if not path.exists():
            raise SystemExit(f"Missing {path}")

    frames = np.load(rgb_path)
    frame_times = np.load(ts_path)
    robot_times, signal = _read_robot_times_and_signal(csv_path, args.signal)

    if len(frame_times) == 0 or len(robot_times) == 0:
        raise SystemExit("No timestamps to analyze.")

    motion_idx = _detect_motion_onset(frames, args.z)
    torque_idx = _detect_first_spike(signal, args.z)

    motion_time = float(frame_times[min(motion_idx, len(frame_times) - 1)])
    torque_time = float(robot_times[min(torque_idx, len(robot_times) - 1)])

    print(f"motion onset index: {motion_idx}, time {motion_time:.4f}s")
    print(f"signal onset index: {torque_idx}, time {torque_time:.4f}s")
    print(f"offset (motion - signal): {motion_time - torque_time:.4f}s")


if __name__ == "__main__":
    main()
