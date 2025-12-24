#!/usr/bin/env python3
"""Render RGB|Torque plot video from a session folder."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path
from typing import List, Tuple

import cv2
import numpy as np

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
except ModuleNotFoundError as exc:
    raise SystemExit("matplotlib is required for plotting.") from exc


def _read_robot_times_and_torque(csv_path: Path) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    with csv_path.open("r", newline="") as fh:
        reader = csv.reader(fh)
        rows = list(reader)
    if len(rows) < 4:
        raise ValueError("robot_state.csv is missing data rows")
    header = rows[2]
    data_rows = rows[3:]
    try:
        right_adduction_idx = header.index("rightadduction_curr")
        right_sweeping_idx = header.index("rightsweeping_curr")
    except ValueError as exc:
        raise ValueError("rightadduction_curr/rightsweeping_curr columns not found in robot_state.csv") from exc

    times: List[float] = []
    right_adduction: List[float] = []
    right_sweeping: List[float] = []
    for row in data_rows:
        if not row:
            continue
        try:
            times.append(float(row[0]))
            right_adduction.append(float(row[right_adduction_idx]))
            right_sweeping.append(float(row[right_sweeping_idx]))
        except ValueError:
            continue
    return np.asarray(times), np.asarray(right_adduction), np.asarray(right_sweeping)


def _closest_indices(robot_times: np.ndarray, frame_times: np.ndarray) -> np.ndarray:
    idxs = np.searchsorted(robot_times, frame_times, side="left")
    idxs = np.clip(idxs, 1, len(robot_times) - 1)
    left = idxs - 1
    right = idxs
    left_diff = np.abs(frame_times - robot_times[left])
    right_diff = np.abs(frame_times - robot_times[right])
    use_right = right_diff < left_diff
    return np.where(use_right, right, left)


def _plot_to_image(
    times: np.ndarray,
    right_adduction: np.ndarray,
    right_sweeping: np.ndarray,
    current_idx: int,
    width: int,
    height: int,
) -> np.ndarray:
    fig, ax = plt.subplots(figsize=(width / 100.0, height / 100.0), dpi=100)
    ax.plot(times[: current_idx + 1], right_adduction[: current_idx + 1], color="tab:red", linewidth=1.5, label="rightadduction_curr")
    ax.plot(times[: current_idx + 1], right_sweeping[: current_idx + 1], color="tab:blue", linewidth=1.5, label="rightsweeping_curr")
    ax.set_title("Right Motor Torque Setpoints")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("torque setpoint")
    ax.legend(loc="upper right")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.canvas.draw()
    img = np.frombuffer(fig.canvas.tostring_rgb(), dtype=np.uint8)
    img = img.reshape(fig.canvas.get_width_height()[::-1] + (3,))
    plt.close(fig)
    return img


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("session_dir", type=Path, help="Session folder")
    ap.add_argument("--output", type=Path, default=None, help="Output mp4 path (default: session_dir/rgbd_force.mp4)")
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

    rgb = np.load(rgb_path)
    frame_times = np.load(ts_path)
    robot_times, right_adduction, right_sweeping = _read_robot_times_and_torque(csv_path)

    if len(rgb) != len(frame_times):
        raise SystemExit("RGB and timestamps lengths do not match.")

    if len(robot_times) < 2:
        raise SystemExit("Not enough robot_state samples.")

    plot_h = rgb.shape[1]
    plot_w = rgb.shape[2]

    idxs = _closest_indices(robot_times, frame_times)

    output_path = args.output or (session_dir / "rgb_force.mp4")
    fps = max(1.0, (len(frame_times) - 1) / (frame_times[-1] - frame_times[0]))
    out_w = rgb.shape[2] + plot_w
    out_h = rgb.shape[1]

    writer = cv2.VideoWriter(
        str(output_path),
        cv2.VideoWriter_fourcc(*"mp4v"),
        fps,
        (out_w, out_h),
    )
    if not writer.isOpened():
        raise SystemExit("Failed to open output video writer.")

    for i in range(len(rgb)):
        color_img = rgb[i]
        plot_img = _plot_to_image(robot_times, right_adduction, right_sweeping, int(idxs[i]), plot_w, plot_h)
        plot_bgr = cv2.cvtColor(plot_img, cv2.COLOR_RGB2BGR)

        canvas = np.hstack([color_img, plot_bgr])
        writer.write(canvas)

    writer.release()
    print(f"Saved {output_path}")


if __name__ == "__main__":
    main()
