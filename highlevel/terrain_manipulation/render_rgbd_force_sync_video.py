#!/usr/bin/env python3
"""Render a sync-review video with two RGB streams, two depth streams, and motor forces.

Creates an MP4 that stacks:
  - RGB camera 0 and RGB camera 1 (top row)
  - Depth camera 0 and Depth camera 1 (bottom row)
  - Force plot (right column)
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
except ModuleNotFoundError as exc:
    raise SystemExit("matplotlib is required for plotting.") from exc

DATA_ROOT = Path(__file__).resolve().parent / "data"


def _load_payload(path: Path) -> Dict[str, object]:
    data = np.load(path, allow_pickle=True)
    if isinstance(data, np.ndarray) and data.shape == ():
        return data.item()
    if isinstance(data, dict):
        return data
    raise ValueError(f"Unexpected npy payload format in {path}")


def _pick_trial(path: Path, trial_index: Optional[int]) -> Path:
    if path.is_file():
        return path
    trials = sorted(path.glob("trial_*.npy"))
    if not trials:
        raise FileNotFoundError(f"No trial_*.npy files found in {path}")
    if trial_index is None:
        return trials[-1]
    if trial_index < 1 or trial_index > len(trials):
        raise ValueError(f"trial index {trial_index} out of range (1..{len(trials)})")
    return trials[trial_index - 1]


def _latest_session_dir(data_root: Path) -> Path:
    if not data_root.exists():
        raise FileNotFoundError(f"Data root not found: {data_root}")
    sessions = [path for path in data_root.iterdir() if path.is_dir() and path.name.startswith("session_")]
    if not sessions:
        raise FileNotFoundError(f"No session folders found under {data_root}")
    return max(sessions, key=lambda path: path.name)


def _closest_indices(sample_times: np.ndarray, query_times: np.ndarray) -> np.ndarray:
    if sample_times.size == 0:
        return np.zeros_like(query_times, dtype=int)
    idx = np.searchsorted(sample_times, query_times, side="left")
    idx = np.clip(idx, 0, sample_times.size - 1)
    prev_idx = np.clip(idx - 1, 0, sample_times.size - 1)
    next_idx = idx
    prev_diff = np.abs(query_times - sample_times[prev_idx])
    next_diff = np.abs(query_times - sample_times[next_idx])
    choose_prev = prev_diff <= next_diff
    return np.where(choose_prev, prev_idx, next_idx)


def _depth_values(depth: np.ndarray, max_frames: int = 50) -> np.ndarray:
    if depth.size == 0:
        return np.empty((0,), dtype=depth.dtype)
    frames = depth
    if depth.ndim == 3 and depth.shape[0] > max_frames:
        idx = np.linspace(0, depth.shape[0] - 1, max_frames, dtype=int)
        frames = depth[idx]
    values = frames.reshape(-1)
    return values[values > 0]


def _estimate_depth_range(depth: np.ndarray, max_frames: int = 50) -> Tuple[float, float]:
    values = _depth_values(depth, max_frames)
    if values.size == 0:
        return 0.0, 1.0
    low, high = np.percentile(values, [1.0, 99.0])
    if high <= low:
        high = low + 1.0
    return float(low), float(high)


def _estimate_shared_depth_range(
    depth_a: np.ndarray,
    depth_b: np.ndarray,
    max_frames: int = 50,
) -> Tuple[float, float]:
    values_a = _depth_values(depth_a, max_frames)
    values_b = _depth_values(depth_b, max_frames)
    if values_a.size == 0 and values_b.size == 0:
        return 0.0, 1.0
    if values_a.size == 0:
        values = values_b
    elif values_b.size == 0:
        values = values_a
    else:
        values = np.concatenate([values_a, values_b])
    low, high = np.percentile(values, [1.0, 99.0])
    if high <= low:
        high = low + 1.0
    return float(low), float(high)


def _colorize_depth(depth_raw: np.ndarray, depth_min: float, depth_max: float) -> np.ndarray:
    depth = depth_raw.astype(np.float32)
    depth = np.clip((depth - depth_min) / (depth_max - depth_min), 0.0, 1.0)
    depth_u8 = (depth * 255.0).astype(np.uint8)
    return cv2.applyColorMap(depth_u8, cv2.COLORMAP_JET)


def _plot_forces(
    times: np.ndarray,
    force_a: np.ndarray,
    force_b: np.ndarray,
    idx: int,
    width: int,
    height: int,
    label_a: str,
    label_b: str,
    time_diffs: Optional[np.ndarray] = None,
) -> np.ndarray:
    if time_diffs is None:
        fig, ax = plt.subplots(figsize=(width / 100.0, height / 100.0), dpi=100)
        ax.plot(times[: idx + 1], force_a[: idx + 1], color="tab:red", linewidth=1.5, label=label_a)
        ax.plot(times[: idx + 1], force_b[: idx + 1], color="tab:blue", linewidth=1.5, label=label_b)
        ax.set_title("Motor Forces")
        ax.set_xlabel("time (s)")
        ax.set_ylabel("force (scaled)")
        ax.legend(loc="upper right")
        ax.grid(True, alpha=0.3)
    else:
        fig, (ax_force, ax_diff) = plt.subplots(
            2,
            1,
            figsize=(width / 100.0, height / 100.0),
            dpi=100,
            sharex=True,
            gridspec_kw={"height_ratios": [2, 1]},
        )
        ax_force.plot(times[: idx + 1], force_a[: idx + 1], color="tab:red", linewidth=1.5, label=label_a)
        ax_force.plot(times[: idx + 1], force_b[: idx + 1], color="tab:blue", linewidth=1.5, label=label_b)
        ax_force.set_title("Motor Forces")
        ax_force.set_ylabel("force (scaled)")
        ax_force.legend(loc="upper right")
        ax_force.grid(True, alpha=0.3)

        diffs = np.abs(time_diffs[: idx + 1])
        ax_diff.plot(times[: idx + 1], diffs, color="tab:green", linewidth=1.2, label="|t_force - t_frame|")
        ax_diff.set_ylabel("Δt (s)")
        ax_diff.set_xlabel("time (s)")
        ax_diff.grid(True, alpha=0.3)
        ax_diff.legend(loc="upper right")
    fig.tight_layout()
    fig.canvas.draw()
    try:
        buf = fig.canvas.buffer_rgba()
        img = np.asarray(buf, dtype=np.uint8)[..., :3]
    except Exception:
        img = np.frombuffer(fig.canvas.tostring_rgb(), dtype=np.uint8)
        img = img.reshape(fig.canvas.get_width_height()[::-1] + (3,))
    plt.close(fig)
    return img


def main() -> None:
    ap = argparse.ArgumentParser(description="Render sync-review MP4 for RGB-D + forces.")
    ap.add_argument(
        "session_or_trial",
        nargs="?",
        default=None,
        type=Path,
        help="Session directory or trial .npy file (default: latest session).",
    )
    ap.add_argument("--trial", type=int, default=None, help="Trial index (1-based) when a session dir is provided")
    ap.add_argument("--output", type=Path, default=None, help="Output mp4 path")
    ap.add_argument("--force-a", default="rightadduction_curr", help="robot_state key for force A")
    ap.add_argument("--force-b", default="rightsweeping_curr", help="robot_state key for force B")
    ap.add_argument("--torque-scale", type=float, default=0.072, help="Scale forces by this factor")
    args = ap.parse_args()

    if args.session_or_trial is None:
        session_dir = _latest_session_dir(DATA_ROOT)
        trial_path = _pick_trial(session_dir, 1)
    else:
        trial_path = _pick_trial(Path(str(args.session_or_trial).strip()), args.trial)
    payload = _load_payload(trial_path)

    rgb0 = payload.get("rgb_0") if "rgb_0" in payload else payload.get("rgb")
    depth0 = payload.get("depth_0") if "depth_0" in payload else payload.get("depth")
    t0 = payload.get("camera_time_0") if "camera_time_0" in payload else payload.get("camera_time")
    rgb1 = payload.get("rgb_1") if "rgb_1" in payload else payload.get("rgb_2")
    depth1 = payload.get("depth_1") if "depth_1" in payload else payload.get("depth_2")
    t1 = payload.get("camera_time_1") if "camera_time_1" in payload else payload.get("timestamps_2")
    robot_state = payload.get("robot_state")

    if not isinstance(rgb0, np.ndarray) or not isinstance(depth0, np.ndarray) or not isinstance(t0, np.ndarray):
        raise SystemExit("Missing rgb_0/depth_0/camera_time_0 in payload.")
    if not isinstance(rgb1, np.ndarray) or not isinstance(depth1, np.ndarray) or not isinstance(t1, np.ndarray):
        raise SystemExit("Missing rgb_1/depth_1/camera_time_1 in payload.")
    if not isinstance(robot_state, dict):
        raise SystemExit("Missing robot_state in payload.")

    force_a = robot_state.get(args.force_a)
    force_b = robot_state.get(args.force_b)
    robot_time = robot_state.get("time")
    if not isinstance(force_a, np.ndarray) or not isinstance(force_b, np.ndarray) or not isinstance(robot_time, np.ndarray):
        raise SystemExit("robot_state missing force keys or time array.")

    force_a = force_a * float(args.torque_scale)
    force_b = force_b * float(args.torque_scale)

    if len(rgb0) != len(t0):
        raise SystemExit("rgb_0 and camera_time_0 length mismatch.")
    if len(rgb1) != len(t1):
        raise SystemExit("rgb_1 and camera_time_1 length mismatch.")

    idx1_for_t0 = _closest_indices(np.asarray(t1, dtype=float), np.asarray(t0, dtype=float))
    robot_idx_for_t0 = _closest_indices(np.asarray(robot_time, dtype=float), np.asarray(t0, dtype=float))

    depth_min, depth_max = _estimate_shared_depth_range(depth0, depth1)
    depth0_min, depth0_max = depth_min, depth_max
    depth1_min, depth1_max = depth_min, depth_max

    h, w = rgb0.shape[1], rgb0.shape[2]
    plot_w, plot_h = w, h * 2
    canvas_w, canvas_h = w * 3, h * 2

    output_path = args.output or (trial_path.parent / f"{trial_path.stem}_sync.mp4")
    if len(t0) > 1:
        fps = max(1.0, (len(t0) - 1) / float(t0[-1] - t0[0]))
    else:
        fps = 30.0

    writer = cv2.VideoWriter(
        str(output_path),
        cv2.VideoWriter_fourcc(*"mp4v"),
        fps,
        (canvas_w, canvas_h),
    )
    if not writer.isOpened():
        raise SystemExit("Failed to open output video writer.")

    for i in range(len(t0)):
        idx1 = int(idx1_for_t0[i])
        idx_robot = int(robot_idx_for_t0[i])

        rgb0_img = rgb0[i]
        rgb1_img = rgb1[idx1]
        depth0_img = _colorize_depth(depth0[i], depth0_min, depth0_max)
        depth1_img = _colorize_depth(depth1[idx1], depth1_min, depth1_max)

        top = np.hstack([rgb0_img, rgb1_img])
        bottom = np.hstack([depth0_img, depth1_img])
        grid = np.vstack([top, bottom])

        time_diffs = np.asarray(robot_time, dtype=float)[robot_idx_for_t0] - np.asarray(t0, dtype=float)
        plot_img = _plot_forces(
            robot_time,
            force_a,
            force_b,
            idx_robot,
            plot_w,
            plot_h,
            args.force_a,
            args.force_b,
            time_diffs=time_diffs,
        )
        plot_bgr = cv2.cvtColor(plot_img, cv2.COLOR_RGB2BGR)

        canvas = np.hstack([grid, plot_bgr])
        writer.write(canvas)

    writer.release()
    max_diff = float(np.max(np.abs(time_diffs))) if len(t0) else 0.0
    print(f"Saved {output_path}")
    print(f"Max |t_force - t_frame|: {max_diff:.6f} s")


if __name__ == "__main__":
    main()
