#!/usr/bin/env python3
"""Render a sync-review video with RGB-D, motor torque, and mocap signals.

Creates an MP4 that stacks:
  - RGB camera 0 and RGB camera 1 (top row)
  - Depth camera 0 and Depth camera 1 (bottom row)
  - Right panel plots for torque, timing offset,
    plus mocap for a selected half sphere (empty/lead/resin/steel/sand)
    
Usage e.g.:
/home/parnia/anaconda3/envs/Turtle_TM/bin/python highlevel/terrain_manipulation/src/utils/render_rgbd_force_mocap_sync_video.py --half-sphere resin --mode experiment --depth-min-m 0.25 --depth-max-m 1.00
"""
# NOTE:
# Steel experiments on March 14, 2026 and resin experiments on
# March 16, 2026 used an incorrect Motive pivot/COM definition.
# For those sessions, pass:
#   --com-offset-y-m -0.00375

from __future__ import annotations

import argparse
import json
import re
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

DATA_ROOT = Path(__file__).resolve().parents[2] / "data"
MOCAP_RB_EMPTY_ID = 2
MOCAP_RB_LEAD_ID = 3
MOCAP_RB_RESIN_ID = 5
MOCAP_RB_STEEL_ID = 6
MOCAP_RB_SAND_ID = 8
MOCAP_RB_IDS_BY_KIND = {
    "empty": MOCAP_RB_EMPTY_ID,
    "lead": MOCAP_RB_LEAD_ID,
    "resin": MOCAP_RB_RESIN_ID,
    "steel": MOCAP_RB_STEEL_ID,
    "sand": MOCAP_RB_SAND_ID,
}
MOCAP_RB_NAMES = {
    MOCAP_RB_EMPTY_ID: "Empty Half Sphere",
    MOCAP_RB_LEAD_ID: "Lead Half Sphere",
    MOCAP_RB_RESIN_ID: "Resin Half Sphere",
    MOCAP_RB_STEEL_ID: "Steel Half Sphere",
    MOCAP_RB_SAND_ID: "Sand Half Sphere",
}


def _load_payload(path: Path) -> Dict[str, object]:
    data = np.load(path, allow_pickle=True)
    if isinstance(data, np.ndarray) and data.shape == ():
        return data.item()
    if isinstance(data, dict):
        return data
    raise ValueError(f"Unexpected npy payload format in {path}")


def _scalar_float(value: object) -> Optional[float]:
    if value is None:
        return None
    if isinstance(value, (int, float, np.integer, np.floating)):
        return float(value)
    if isinstance(value, np.ndarray) and value.shape == ():
        try:
            return float(value.item())
        except Exception:
            return None
    return None


def _load_session_metadata(session_dir: Path) -> Dict[str, object]:
    meta_path = session_dir / "metadata.json"
    if not meta_path.is_file():
        return {}
    try:
        with open(meta_path, "r", encoding="utf-8") as fh:
            data = json.load(fh)
    except Exception:
        return {}
    return data if isinstance(data, dict) else {}


def _trial_number(path: Path) -> int:
    match = re.match(r"trial_(\d+)\.npy$", path.name)
    if match is None:
        return 10**9
    return int(match.group(1))


def _list_trials(path: Path) -> List[Path]:
    if path.is_file():
        return [path]
    trials = [trial for trial in path.glob("trial_*.npy") if trial.is_file()]
    return sorted(trials, key=lambda trial: (_trial_number(trial), trial.name))


def _pick_trial(path: Path, trial_index: Optional[int]) -> Path:
    trials = _list_trials(path)
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


def _list_mocap_rb_ids(trial_path: Path) -> List[int]:
    payload = _load_payload(trial_path)
    mocap = payload.get("mocap")
    if not isinstance(mocap, dict):
        return []
    ids: List[int] = []
    for key, state in mocap.items():
        if not isinstance(state, dict):
            continue
        try:
            ids.append(int(key))
        except Exception:
            continue
    return sorted(set(ids))


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


def _extract_mocap_series(
    rb_state: Optional[Dict[str, object]],
    signal_key: str,
    camera_times: np.ndarray,
) -> np.ndarray:
    fallback = np.full(camera_times.shape, np.nan, dtype=float)
    if not isinstance(rb_state, dict):
        return fallback

    signal = rb_state.get(signal_key)
    if not isinstance(signal, np.ndarray):
        return fallback

    signal_arr = np.asarray(signal, dtype=float)
    if signal_arr.size == camera_times.size:
        return signal_arr

    sample_times = rb_state.get("time")
    if not isinstance(sample_times, np.ndarray):
        return fallback
    sample_times_arr = np.asarray(sample_times, dtype=float)
    if sample_times_arr.size == 0 or sample_times_arr.size != signal_arr.size:
        return fallback

    idx = _closest_indices(sample_times_arr, camera_times)
    return signal_arr[idx]


def _extract_first_mocap_series(
    rb_state: Optional[Dict[str, object]],
    signal_keys: Tuple[str, ...],
    camera_times: np.ndarray,
) -> np.ndarray:
    if not isinstance(rb_state, dict):
        return np.full(camera_times.shape, np.nan, dtype=float)
    for key in signal_keys:
        if key in rb_state:
            return _extract_mocap_series(rb_state, key, camera_times)
    return np.full(camera_times.shape, np.nan, dtype=float)


def _rotate_vector_by_quaternion_batch(quat_wxyz: np.ndarray, vector_xyz: np.ndarray) -> np.ndarray:
    # Uses v' = q * v * q_conjugate; inputs are expected to be normalized.
    u = quat_wxyz[:, 1:4]
    s = quat_wxyz[:, :1]
    v = vector_xyz
    dot_uv = np.sum(u * v, axis=1, keepdims=True)
    dot_uu = np.sum(u * u, axis=1, keepdims=True)
    cross_uv = np.cross(u, v)
    return 2.0 * dot_uv * u + (s * s - dot_uu) * v + 2.0 * s * cross_uv


def _apply_body_com_offset(
    pos_x: np.ndarray,
    pos_y: np.ndarray,
    pos_z: np.ndarray,
    quat_w: np.ndarray,
    quat_x: np.ndarray,
    quat_y: np.ndarray,
    quat_z: np.ndarray,
    offset_body_xyz_m: Tuple[float, float, float],
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, bool]:
    offset = np.asarray(offset_body_xyz_m, dtype=float).reshape(3)
    if np.allclose(offset, 0.0):
        return pos_x, pos_y, pos_z, False

    pos = np.column_stack([pos_x, pos_y, pos_z]).astype(float, copy=True)
    quat = np.column_stack([quat_w, quat_x, quat_y, quat_z]).astype(float, copy=False)

    finite_mask = np.isfinite(pos).all(axis=1) & np.isfinite(quat).all(axis=1)
    if not np.any(finite_mask):
        return pos_x, pos_y, pos_z, False

    quat_valid = quat[finite_mask]
    norms = np.linalg.norm(quat_valid, axis=1, keepdims=True)
    norm_mask = (norms[:, 0] > 1e-12) & np.isfinite(norms[:, 0])
    if not np.any(norm_mask):
        return pos_x, pos_y, pos_z, False

    quat_valid = quat_valid[norm_mask] / norms[norm_mask]
    rotated_offset = _rotate_vector_by_quaternion_batch(
        quat_valid,
        np.broadcast_to(offset, (quat_valid.shape[0], 3)),
    )

    finite_indices = np.flatnonzero(finite_mask)
    target_indices = finite_indices[norm_mask]
    pos[target_indices] += rotated_offset

    return pos[:, 0], pos[:, 1], pos[:, 2], True


def _get_mocap_state(mocap: object, rb_id: int) -> Optional[Dict[str, object]]:
    if not isinstance(mocap, dict):
        return None
    state = mocap.get(str(rb_id))
    if state is None:
        state = mocap.get(rb_id)
    return state if isinstance(state, dict) else None


def _load_trial_arrays(
    trial_path: Path,
    force_a_key: str,
    force_b_key: str,
    torque_scale: float,
    mocap_rb_id: int,
    com_offset_y_m: float,
) -> Dict[str, np.ndarray]:
    payload = _load_payload(trial_path)

    rgb0 = payload.get("rgb_0") if "rgb_0" in payload else payload.get("rgb")
    depth0 = payload.get("depth_0") if "depth_0" in payload else payload.get("depth")
    t0 = payload.get("camera_time_0") if "camera_time_0" in payload else payload.get("camera_time")
    rgb1 = payload.get("rgb_1") if "rgb_1" in payload else payload.get("rgb_2")
    depth1 = payload.get("depth_1") if "depth_1" in payload else payload.get("depth_2")
    t1 = payload.get("camera_time_1") if "camera_time_1" in payload else payload.get("timestamps_2")
    robot_state = payload.get("robot_state")
    depth_scale0 = _scalar_float(payload.get("depth_scale_0"))
    if depth_scale0 is None:
        depth_scale0 = _scalar_float(payload.get("depth_scale"))
    depth_scale1 = _scalar_float(payload.get("depth_scale_1"))
    if depth_scale1 is None:
        depth_scale1 = _scalar_float(payload.get("depth_scale_2"))
    if depth_scale1 is None:
        depth_scale1 = depth_scale0
    if depth_scale0 is None or depth_scale1 is None:
        session_meta = _load_session_metadata(trial_path.parent)
        if depth_scale0 is None:
            depth_scale0 = _scalar_float(session_meta.get("depth_scale_0"))
            if depth_scale0 is None:
                depth_scale0 = _scalar_float(session_meta.get("depth_scale"))
        if depth_scale1 is None:
            depth_scale1 = _scalar_float(session_meta.get("depth_scale_1"))
            if depth_scale1 is None:
                depth_scale1 = _scalar_float(session_meta.get("depth_scale_2"))
        if depth_scale1 is None:
            depth_scale1 = depth_scale0

    if not isinstance(rgb0, np.ndarray) or not isinstance(depth0, np.ndarray) or not isinstance(t0, np.ndarray):
        raise ValueError("missing rgb_0/depth_0/camera_time_0")
    if not isinstance(rgb1, np.ndarray) or not isinstance(depth1, np.ndarray) or not isinstance(t1, np.ndarray):
        raise ValueError("missing rgb_1/depth_1/camera_time_1")
    if not isinstance(robot_state, dict):
        raise ValueError("missing robot_state")

    force_a = robot_state.get(force_a_key)
    force_b = robot_state.get(force_b_key)
    robot_time = robot_state.get("time")
    if not isinstance(force_a, np.ndarray) or not isinstance(force_b, np.ndarray) or not isinstance(robot_time, np.ndarray):
        raise ValueError("robot_state missing force keys or time array")

    if len(rgb0) != len(t0):
        raise ValueError("rgb_0 and camera_time_0 length mismatch")
    if len(rgb1) != len(t1):
        raise ValueError("rgb_1 and camera_time_1 length mismatch")
    if len(robot_time) != len(t0):
        raise ValueError("robot_time and camera_time_0 length mismatch")
    if len(force_a) != len(robot_time) or len(force_b) != len(robot_time):
        raise ValueError("force arrays and robot_time length mismatch")

    t0_arr = np.asarray(t0, dtype=float)
    mocap = payload.get("mocap")
    mocap_state = _get_mocap_state(mocap, mocap_rb_id)

    mocap_pos_x = _extract_first_mocap_series(
        mocap_state,
        ("rotated_zeroed_position_x", "rotated_position_x", "zeroed_position_x", "relative_position_x", "position_x"),
        t0_arr,
    )
    mocap_pos_y = _extract_first_mocap_series(
        mocap_state,
        ("rotated_zeroed_position_y", "rotated_position_y", "zeroed_position_y", "relative_position_y", "position_y"),
        t0_arr,
    )
    mocap_pos_z = _extract_first_mocap_series(
        mocap_state,
        ("rotated_zeroed_position_z", "rotated_position_z", "zeroed_position_z", "relative_position_z", "position_z"),
        t0_arr,
    )
    mocap_roll = _extract_first_mocap_series(mocap_state, ("rotated_roll_deg", "roll_deg"), t0_arr)
    mocap_pitch = _extract_first_mocap_series(mocap_state, ("rotated_pitch_deg", "pitch_deg"), t0_arr)
    mocap_yaw = _extract_first_mocap_series(mocap_state, ("rotated_yaw_deg", "yaw_deg"), t0_arr)
    mocap_qw = _extract_first_mocap_series(
        mocap_state,
        ("rotated_zeroed_orientation_w", "rotated_orientation_w", "zeroed_orientation_w", "orientation_w"),
        t0_arr,
    )
    mocap_qx = _extract_first_mocap_series(
        mocap_state,
        ("rotated_zeroed_orientation_x", "rotated_orientation_x", "zeroed_orientation_x", "orientation_x"),
        t0_arr,
    )
    mocap_qy = _extract_first_mocap_series(
        mocap_state,
        ("rotated_zeroed_orientation_y", "rotated_orientation_y", "zeroed_orientation_y", "orientation_y"),
        t0_arr,
    )
    mocap_qz = _extract_first_mocap_series(
        mocap_state,
        ("rotated_zeroed_orientation_z", "rotated_orientation_z", "zeroed_orientation_z", "orientation_z"),
        t0_arr,
    )
    mocap_pos_x, mocap_pos_y, mocap_pos_z, com_applied = _apply_body_com_offset(
        mocap_pos_x,
        mocap_pos_y,
        mocap_pos_z,
        mocap_qw,
        mocap_qx,
        mocap_qy,
        mocap_qz,
        (0.0, float(com_offset_y_m), 0.0),
    )
    if float(com_offset_y_m) != 0.0 and not com_applied:
        print(
            f"Warning: {trial_path.name}: unable to apply COM offset because valid mocap orientation samples were not found."
        )

    robot_time_arr = np.asarray(robot_time, dtype=float)
    time_diff = robot_time_arr - t0_arr

    return {
        "rgb0": rgb0,
        "depth0": depth0,
        "t0": t0_arr,
        "rgb1": rgb1,
        "depth1": depth1,
        "t1": np.asarray(t1, dtype=float),
        "force_a": np.asarray(force_a, dtype=float) * float(torque_scale),
        "force_b": np.asarray(force_b, dtype=float) * float(torque_scale),
        "robot_time": robot_time_arr,
        "time_diff": np.asarray(time_diff, dtype=float),
        "mocap_pos_x": mocap_pos_x,
        "mocap_pos_y": mocap_pos_y,
        "mocap_pos_z": mocap_pos_z,
        "mocap_roll": mocap_roll,
        "mocap_pitch": mocap_pitch,
        "mocap_yaw": mocap_yaw,
        "depth_scale0": np.asarray(np.nan if depth_scale0 is None else depth_scale0, dtype=float),
        "depth_scale1": np.asarray(np.nan if depth_scale1 is None else depth_scale1, dtype=float),
    }


def _depth_values(depth: np.ndarray, max_frames: int = 50) -> np.ndarray:
    if depth.size == 0:
        return np.empty((0,), dtype=depth.dtype)
    frames = depth
    if depth.ndim == 3 and depth.shape[0] > max_frames:
        idx = np.linspace(0, depth.shape[0] - 1, max_frames, dtype=int)
        frames = depth[idx]
    values = frames.reshape(-1)
    return values[values > 0]


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


def _estimate_depth_range_for_trials(trials: List[Dict[str, np.ndarray]]) -> Tuple[float, float]:
    if not trials:
        return 0.0, 1.0
    lows: List[float] = []
    highs: List[float] = []
    for trial in trials:
        low, high = _estimate_shared_depth_range(trial["depth0"], trial["depth1"])
        lows.append(low)
        highs.append(high)
    low = min(lows)
    high = max(highs)
    if high <= low:
        high = low + 1.0
    return low, high


def _estimate_fps_from_trials(trials: List[Dict[str, np.ndarray]]) -> float:
    deltas: List[np.ndarray] = []
    for trial in trials:
        t0 = trial["t0"]
        if len(t0) < 2:
            continue
        dt = np.diff(t0)
        dt = dt[dt > 0]
        if dt.size > 0:
            deltas.append(dt)
    if not deltas:
        return 30.0
    median_dt = float(np.median(np.concatenate(deltas)))
    if median_dt <= 0.0:
        return 30.0
    return float(np.clip(1.0 / median_dt, 1.0, 240.0))


def _median_positive_step(times: np.ndarray) -> float:
    if len(times) < 2:
        return 0.0
    dt = np.diff(times)
    dt = dt[dt > 0]
    if dt.size == 0:
        return 0.0
    return float(np.median(dt))


def _build_experiment_timeline(
    trials: List[Dict[str, np.ndarray]],
) -> Tuple[
    np.ndarray,
    np.ndarray,
    np.ndarray,
    np.ndarray,
    np.ndarray,
    np.ndarray,
    np.ndarray,
    np.ndarray,
    np.ndarray,
    np.ndarray,
    List[int],
]:
    all_times: List[np.ndarray] = []
    all_force_a: List[np.ndarray] = []
    all_force_b: List[np.ndarray] = []
    all_time_diffs: List[np.ndarray] = []
    all_mocap_pos_x: List[np.ndarray] = []
    all_mocap_pos_y: List[np.ndarray] = []
    all_mocap_pos_z: List[np.ndarray] = []
    all_mocap_roll: List[np.ndarray] = []
    all_mocap_pitch: List[np.ndarray] = []
    all_mocap_yaw: List[np.ndarray] = []
    trial_idx_offsets: List[int] = []

    running_time_offset = 0.0
    running_idx_offset = 0

    for trial in trials:
        frame_time_local = trial["t0"]
        robot_time_local = trial["robot_time"]
        force_a_local = trial["force_a"]
        force_b_local = trial["force_b"]
        time_diff_local = trial["time_diff"]

        if len(frame_time_local) == 0:
            trial_idx_offsets.append(running_idx_offset)
            continue

        frame_time_rel = frame_time_local - float(frame_time_local[0])
        robot_time_rel = robot_time_local - float(robot_time_local[0]) if len(robot_time_local) else frame_time_rel.copy()
        frame_time_global = frame_time_rel + running_time_offset

        trial_idx_offsets.append(running_idx_offset)
        running_idx_offset += len(frame_time_local)

        all_times.append(frame_time_global)
        all_force_a.append(force_a_local)
        all_force_b.append(force_b_local)
        all_time_diffs.append(time_diff_local)
        all_mocap_pos_x.append(trial["mocap_pos_x"])
        all_mocap_pos_y.append(trial["mocap_pos_y"])
        all_mocap_pos_z.append(trial["mocap_pos_z"])
        all_mocap_roll.append(trial["mocap_roll"])
        all_mocap_pitch.append(trial["mocap_pitch"])
        all_mocap_yaw.append(trial["mocap_yaw"])

        trial_span = max(float(frame_time_rel[-1]), float(robot_time_rel[-1])) if len(robot_time_rel) else float(frame_time_rel[-1])
        gap = max(_median_positive_step(frame_time_rel), _median_positive_step(robot_time_rel))
        running_time_offset += trial_span + gap

    if not all_times:
        empty = np.empty((0,), dtype=float)
        return (
            empty,
            empty,
            empty,
            empty,
            empty,
            empty,
            empty,
            empty,
            empty,
            empty,
            trial_idx_offsets,
        )

    return (
        np.concatenate(all_times),
        np.concatenate(all_force_a),
        np.concatenate(all_force_b),
        np.concatenate(all_time_diffs),
        np.concatenate(all_mocap_pos_x),
        np.concatenate(all_mocap_pos_y),
        np.concatenate(all_mocap_pos_z),
        np.concatenate(all_mocap_roll),
        np.concatenate(all_mocap_pitch),
        np.concatenate(all_mocap_yaw),
        trial_idx_offsets,
    )


def _colorize_depth(depth_raw: np.ndarray, depth_min: float, depth_max: float) -> np.ndarray:
    depth = depth_raw.astype(np.float32)
    depth = np.clip((depth - depth_min) / (depth_max - depth_min), 0.0, 1.0)
    depth_u8 = (depth * 255.0).astype(np.uint8)
    return cv2.applyColorMap(depth_u8, cv2.COLORMAP_JET)


def _colorize_depth_meters(
    depth_raw: np.ndarray,
    depth_scale_m_per_unit: float,
    depth_min_m: float,
    depth_max_m: float,
) -> np.ndarray:
    if not np.isfinite(depth_scale_m_per_unit) or depth_scale_m_per_unit <= 0.0:
        return _colorize_depth(depth_raw, depth_min_m, depth_max_m)
    raw_min = depth_min_m / depth_scale_m_per_unit
    raw_max = depth_max_m / depth_scale_m_per_unit
    return _colorize_depth(depth_raw, raw_min, raw_max)


def _torque_label_from_key(signal_key: str) -> str:
    if "curr" in signal_key:
        return signal_key.replace("curr", "torque")
    return signal_key


def _has_mocap_values(*arrays: np.ndarray) -> bool:
    for arr in arrays:
        if arr.size and not np.all(np.isnan(arr)):
            return True
    return False


def _plot_signals(
    times: np.ndarray,
    force_a: np.ndarray,
    force_b: np.ndarray,
    time_diffs: np.ndarray,
    mocap_pos_x: np.ndarray,
    mocap_pos_y: np.ndarray,
    mocap_pos_z: np.ndarray,
    mocap_roll: np.ndarray,
    mocap_pitch: np.ndarray,
    mocap_yaw: np.ndarray,
    idx: int,
    width: int,
    height: int,
    label_a: str,
    label_b: str,
    mocap_name: str,
) -> np.ndarray:
    fig, (ax_force, ax_diff, ax_mocap_pos, ax_mocap_rpy) = plt.subplots(
        4,
        1,
        figsize=(width / 100.0, height / 100.0),
        dpi=100,
        sharex=True,
        gridspec_kw={"height_ratios": [2, 1, 2, 2]},
    )

    if times.size == 0:
        for ax in (ax_force, ax_diff, ax_mocap_pos, ax_mocap_rpy):
            ax.text(0.5, 0.5, "no data", ha="center", va="center", transform=ax.transAxes)
            ax.set_xticks([])
            ax.set_yticks([])
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

    idx = int(np.clip(idx, 0, times.size - 1))
    end = idx + 1

    ax_force.plot(times[:end], force_a[:end], color="tab:red", linewidth=1.4, label=label_a)
    ax_force.plot(times[:end], force_b[:end], color="tab:blue", linewidth=1.4, label=label_b)
    ax_force.set_title("Motor Torque")
    ax_force.set_ylabel("torque")
    ax_force.grid(True, alpha=0.3)
    ax_force.legend(loc="upper right")

    ax_diff.plot(times[:end], np.abs(time_diffs[:end]), color="tab:green", linewidth=1.2, label="|t_torque - t_frame|")
    ax_diff.set_ylabel("delta t (s)")
    ax_diff.grid(True, alpha=0.3)
    ax_diff.legend(loc="upper right")

    ax_mocap_pos.set_title(f"Mocap Position ({mocap_name})")
    if not _has_mocap_values(mocap_pos_x[:end], mocap_pos_y[:end], mocap_pos_z[:end]):
        ax_mocap_pos.text(0.5, 0.5, "no mocap samples", ha="center", va="center", transform=ax_mocap_pos.transAxes)
    else:
        ax_mocap_pos.plot(times[:end], 100.0 * mocap_pos_x[:end], color="tab:purple", linewidth=1.2, label="x")
        ax_mocap_pos.plot(times[:end], 100.0 * mocap_pos_y[:end], color="tab:orange", linewidth=1.2, label="y'")
        ax_mocap_pos.plot(times[:end], 100.0 * mocap_pos_z[:end], color="tab:cyan", linewidth=1.2, label="z'")
        ax_mocap_pos.legend(loc="upper right")
    ax_mocap_pos.set_ylabel("position (cm)")
    ax_mocap_pos.grid(True, alpha=0.3)

    ax_mocap_rpy.set_title(f"Mocap Orientation ({mocap_name})")
    if not _has_mocap_values(mocap_roll[:end], mocap_pitch[:end], mocap_yaw[:end]):
        ax_mocap_rpy.text(0.5, 0.5, "no mocap samples", ha="center", va="center", transform=ax_mocap_rpy.transAxes)
    else:
        ax_mocap_rpy.plot(times[:end], mocap_roll[:end], color="tab:red", linewidth=1.2, label="roll")
        ax_mocap_rpy.plot(times[:end], mocap_pitch[:end], color="tab:blue", linewidth=1.2, label="pitch")
        ax_mocap_rpy.plot(times[:end], mocap_yaw[:end], color="tab:green", linewidth=1.2, label="yaw")
        ax_mocap_rpy.legend(loc="upper right")
    ax_mocap_rpy.set_ylabel("deg")
    ax_mocap_rpy.set_xlabel("time (s)")
    ax_mocap_rpy.grid(True, alpha=0.3)

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


def _render_trial_frames(
    writer: cv2.VideoWriter,
    trial: Dict[str, np.ndarray],
    depth_min: float,
    depth_max: float,
    plot_w: int,
    plot_h: int,
    force_a_label: str,
    force_b_label: str,
    trial_caption: Optional[str] = None,
    plot_times: Optional[np.ndarray] = None,
    plot_force_a: Optional[np.ndarray] = None,
    plot_force_b: Optional[np.ndarray] = None,
    plot_time_diffs: Optional[np.ndarray] = None,
    plot_mocap_pos_x: Optional[np.ndarray] = None,
    plot_mocap_pos_y: Optional[np.ndarray] = None,
    plot_mocap_pos_z: Optional[np.ndarray] = None,
    plot_mocap_roll: Optional[np.ndarray] = None,
    plot_mocap_pitch: Optional[np.ndarray] = None,
    plot_mocap_yaw: Optional[np.ndarray] = None,
    mocap_name: str = "Mocap Rigid Body",
    plot_idx_offset: int = 0,
    depth_min_m: Optional[float] = None,
    depth_max_m: Optional[float] = None,
) -> float:
    rgb0 = trial["rgb0"]
    depth0 = trial["depth0"]
    t0 = trial["t0"]
    rgb1 = trial["rgb1"]
    depth1 = trial["depth1"]
    t1 = trial["t1"]
    depth_scale0 = float(np.asarray(trial.get("depth_scale0", np.nan)).reshape(()))
    depth_scale1 = float(np.asarray(trial.get("depth_scale1", np.nan)).reshape(()))

    local_force_a = trial["force_a"]
    local_force_b = trial["force_b"]
    local_time_diffs = trial["time_diff"]
    local_mocap_pos_x = trial["mocap_pos_x"]
    local_mocap_pos_y = trial["mocap_pos_y"]
    local_mocap_pos_z = trial["mocap_pos_z"]
    local_mocap_roll = trial["mocap_roll"]
    local_mocap_pitch = trial["mocap_pitch"]
    local_mocap_yaw = trial["mocap_yaw"]

    idx1_for_t0 = _closest_indices(t1, t0)

    plot_times_final = t0 if plot_times is None else plot_times
    plot_force_a_final = local_force_a if plot_force_a is None else plot_force_a
    plot_force_b_final = local_force_b if plot_force_b is None else plot_force_b
    plot_time_diffs_final = local_time_diffs if plot_time_diffs is None else plot_time_diffs
    plot_mocap_pos_x_final = local_mocap_pos_x if plot_mocap_pos_x is None else plot_mocap_pos_x
    plot_mocap_pos_y_final = local_mocap_pos_y if plot_mocap_pos_y is None else plot_mocap_pos_y
    plot_mocap_pos_z_final = local_mocap_pos_z if plot_mocap_pos_z is None else plot_mocap_pos_z
    plot_mocap_roll_final = local_mocap_roll if plot_mocap_roll is None else plot_mocap_roll
    plot_mocap_pitch_final = local_mocap_pitch if plot_mocap_pitch is None else plot_mocap_pitch
    plot_mocap_yaw_final = local_mocap_yaw if plot_mocap_yaw is None else plot_mocap_yaw

    max_diff = float(np.nanmax(np.abs(local_time_diffs))) if local_time_diffs.size else 0.0

    for i in range(len(t0)):
        idx1 = int(idx1_for_t0[i])
        idx_plot = int(plot_idx_offset + i)
        if plot_times_final.size:
            idx_plot = min(idx_plot, plot_times_final.size - 1)

        rgb0_img = rgb0[i]
        rgb1_img = rgb1[idx1]
        if depth_min_m is not None and depth_max_m is not None:
            depth0_img = _colorize_depth_meters(depth0[i], depth_scale0, depth_min_m, depth_max_m)
            depth1_img = _colorize_depth_meters(depth1[idx1], depth_scale1, depth_min_m, depth_max_m)
        else:
            depth0_img = _colorize_depth(depth0[i], depth_min, depth_max)
            depth1_img = _colorize_depth(depth1[idx1], depth_min, depth_max)

        top = np.hstack([rgb0_img, rgb1_img])
        bottom = np.hstack([depth0_img, depth1_img])
        grid = np.vstack([top, bottom])

        plot_img = _plot_signals(
            plot_times_final,
            plot_force_a_final,
            plot_force_b_final,
            plot_time_diffs_final,
            plot_mocap_pos_x_final,
            plot_mocap_pos_y_final,
            plot_mocap_pos_z_final,
            plot_mocap_roll_final,
            plot_mocap_pitch_final,
            plot_mocap_yaw_final,
            idx_plot,
            plot_w,
            plot_h,
            force_a_label,
            force_b_label,
            mocap_name,
        )
        plot_bgr = cv2.cvtColor(plot_img, cv2.COLOR_RGB2BGR)

        canvas = np.hstack([grid, plot_bgr])
        if trial_caption:
            cv2.putText(canvas, trial_caption, (24, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(canvas, trial_caption, (24, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 1, cv2.LINE_AA)
        writer.write(canvas)

    return max_diff


def main() -> None:
    ap = argparse.ArgumentParser(description="Render sync-review MP4 for RGB-D + torque + mocap (selected half sphere).")
    ap.add_argument(
        "session_or_trial",
        nargs="?",
        default=None,
        type=Path,
        help="Session directory or trial .npy file (default: latest session).",
    )
    ap.add_argument(
        "--mode",
        choices=["trial", "experiment"],
        default="trial",
        help="Render one trial or concatenate all trials in a session folder.",
    )
    ap.add_argument("--trial", type=int, default=None, help="Trial index (1-based) when a session dir is provided")
    ap.add_argument("--output", type=Path, default=None, help="Output mp4 path")
    ap.add_argument(
        "--compare-output",
        action="store_true",
        help="Append '_compare' to the default output filename (ignored when --output is set).",
    )
    ap.add_argument("--force-a", default="rightadduction_curr", help="robot_state key for force A")
    ap.add_argument("--force-b", default="rightsweeping_curr", help="robot_state key for force B")
    ap.add_argument("--torque-scale", type=float, default=0.072, help="Scale forces by this factor")
    ap.add_argument(
        "--com-offset-y-m",
        type=float,
        default=0.0,
        help="Constant COM offset along body Y (meters), rotated into world frame per sample.",
    )
    ap.add_argument(
        "--depth-min-m",
        type=float,
        default=None,
        help="Fixed minimum depth in meters for depth colormap normalization.",
    )
    ap.add_argument(
        "--depth-max-m",
        type=float,
        default=None,
        help="Fixed maximum depth in meters for depth colormap normalization.",
    )
    ap.add_argument(
        "--half-sphere",
        choices=sorted(MOCAP_RB_IDS_BY_KIND.keys()),
        default="lead",
        help="Which half-sphere mocap body to plot (empty, lead, resin, steel, sand).",
    )
    args = ap.parse_args()
    if (args.depth_min_m is None) != (args.depth_max_m is None):
        raise SystemExit("Provide both --depth-min-m and --depth-max-m, or neither.")
    if (
        args.depth_min_m is not None
        and args.depth_max_m is not None
        and float(args.depth_max_m) <= float(args.depth_min_m)
    ):
        raise SystemExit("--depth-max-m must be greater than --depth-min-m.")

    selected_mocap_kind = args.half_sphere
    selected_mocap_rb_id = MOCAP_RB_IDS_BY_KIND[selected_mocap_kind]
    selected_mocap_name = MOCAP_RB_NAMES.get(selected_mocap_rb_id, f"Rigid Body {selected_mocap_rb_id}")

    source_path = _latest_session_dir(DATA_ROOT) if args.session_or_trial is None else Path(str(args.session_or_trial).strip())
    if args.mode == "trial":
        trial_paths = [_pick_trial(source_path, args.trial)]
    else:
        if args.trial is not None:
            raise SystemExit("--trial can only be used with --mode trial.")
        session_dir = source_path.parent if source_path.is_file() else source_path
        trial_paths = _list_trials(session_dir)
        if not trial_paths:
            raise SystemExit(f"No trial_*.npy files found in {session_dir}")

    available_mocap_ids = sorted({rb_id for path in trial_paths for rb_id in _list_mocap_rb_ids(path)})
    missing_named = [selected_mocap_rb_id] if selected_mocap_rb_id not in available_mocap_ids else []
    for rb_id in missing_named:
        rb_name = MOCAP_RB_NAMES.get(rb_id, f"Rigid Body {rb_id}")
        print(f"Warning: {rb_name} (ID {rb_id}) not found. Its traces will be empty.")

    trials: List[Dict[str, np.ndarray]] = []
    for trial_path in trial_paths:
        try:
            trials.append(
                _load_trial_arrays(
                    trial_path,
                    args.force_a,
                    args.force_b,
                    args.torque_scale,
                    selected_mocap_rb_id,
                    args.com_offset_y_m,
                )
            )
        except ValueError as exc:
            raise SystemExit(f"{trial_path.name}: {exc}") from exc

    if not trials:
        raise SystemExit("No trials to render.")

    first_rgb = trials[0]["rgb0"]
    if first_rgb.ndim != 4 or first_rgb.shape[-1] != 3:
        raise SystemExit(f"{trial_paths[0].name}: rgb_0 expected shape (N,H,W,3), got {first_rgb.shape}")
    h, w = int(first_rgb.shape[1]), int(first_rgb.shape[2])

    for trial_path, trial in zip(trial_paths, trials):
        rgb0 = trial["rgb0"]
        rgb1 = trial["rgb1"]
        depth0 = trial["depth0"]
        depth1 = trial["depth1"]
        if rgb0.ndim != 4 or rgb0.shape[-1] != 3:
            raise SystemExit(f"{trial_path.name}: rgb_0 expected shape (N,H,W,3), got {rgb0.shape}")
        if rgb1.ndim != 4 or rgb1.shape[-1] != 3:
            raise SystemExit(f"{trial_path.name}: rgb_1 expected shape (N,H,W,3), got {rgb1.shape}")
        if depth0.ndim != 3:
            raise SystemExit(f"{trial_path.name}: depth_0 expected shape (N,H,W), got {depth0.shape}")
        if depth1.ndim != 3:
            raise SystemExit(f"{trial_path.name}: depth_1 expected shape (N,H,W), got {depth1.shape}")
        if rgb0.shape[1:3] != (h, w) or rgb1.shape[1:3] != (h, w):
            raise SystemExit(f"{trial_path.name}: RGB resolution mismatch, expected ({h},{w})")
        if depth0.shape[1:3] != (h, w) or depth1.shape[1:3] != (h, w):
            raise SystemExit(f"{trial_path.name}: depth resolution mismatch, expected ({h},{w})")
        if args.depth_min_m is not None:
            depth_scale0 = float(np.asarray(trial.get("depth_scale0", np.nan)).reshape(()))
            depth_scale1 = float(np.asarray(trial.get("depth_scale1", np.nan)).reshape(()))
            if not np.isfinite(depth_scale0) or depth_scale0 <= 0.0:
                raise SystemExit(f"{trial_path.name}: missing/invalid depth_scale_0 for meter-based depth rendering.")
            if not np.isfinite(depth_scale1) or depth_scale1 <= 0.0:
                raise SystemExit(f"{trial_path.name}: missing/invalid depth_scale_1 for meter-based depth rendering.")

    plot_w, plot_h = w, h * 2
    canvas_w, canvas_h = w * 3, h * 2

    if args.output is not None:
        output_path = args.output
    else:
        if args.mode == "trial":
            output_stem = f"{trial_paths[0].stem}_sync_mocap"
        else:
            output_stem = f"{trial_paths[0].parent.name}_sync_mocap"
        if args.compare_output:
            output_stem = f"{output_stem}_compare"
        output_path = trial_paths[0].parent / f"{output_stem}.mp4"

    if args.depth_min_m is None:
        depth_min, depth_max = _estimate_depth_range_for_trials(trials)
    else:
        depth_min, depth_max = 0.0, 1.0
    fps = _estimate_fps_from_trials(trials)
    torque_label_a = _torque_label_from_key(args.force_a)
    torque_label_b = _torque_label_from_key(args.force_b)

    experiment_plot_times: Optional[np.ndarray] = None
    experiment_plot_force_a: Optional[np.ndarray] = None
    experiment_plot_force_b: Optional[np.ndarray] = None
    experiment_plot_time_diffs: Optional[np.ndarray] = None
    experiment_plot_mocap_pos_x: Optional[np.ndarray] = None
    experiment_plot_mocap_pos_y: Optional[np.ndarray] = None
    experiment_plot_mocap_pos_z: Optional[np.ndarray] = None
    experiment_plot_mocap_roll: Optional[np.ndarray] = None
    experiment_plot_mocap_pitch: Optional[np.ndarray] = None
    experiment_plot_mocap_yaw: Optional[np.ndarray] = None
    experiment_plot_idx_offsets: List[int] = []

    if args.mode == "experiment":
        (
            experiment_plot_times,
            experiment_plot_force_a,
            experiment_plot_force_b,
            experiment_plot_time_diffs,
            experiment_plot_mocap_pos_x,
            experiment_plot_mocap_pos_y,
            experiment_plot_mocap_pos_z,
            experiment_plot_mocap_roll,
            experiment_plot_mocap_pitch,
            experiment_plot_mocap_yaw,
            experiment_plot_idx_offsets,
        ) = _build_experiment_timeline(trials)

    writer = cv2.VideoWriter(
        str(output_path),
        cv2.VideoWriter_fourcc(*"mp4v"),
        fps,
        (canvas_w, canvas_h),
    )
    if not writer.isOpened():
        raise SystemExit("Failed to open output video writer.")

    max_diff = 0.0
    try:
        for trial_idx, trial in enumerate(trials, start=1):
            caption = None
            if len(trials) > 1:
                caption = f"Trial {trial_idx}/{len(trials)} ({trial_paths[trial_idx - 1].name})"
            plot_idx_offset = 0
            if args.mode == "experiment":
                plot_idx_offset = experiment_plot_idx_offsets[trial_idx - 1]
            trial_max_diff = _render_trial_frames(
                writer,
                trial,
                depth_min,
                depth_max,
                plot_w,
                plot_h,
                torque_label_a,
                torque_label_b,
                trial_caption=caption,
                plot_times=experiment_plot_times,
                plot_force_a=experiment_plot_force_a,
                plot_force_b=experiment_plot_force_b,
                plot_time_diffs=experiment_plot_time_diffs,
                plot_mocap_pos_x=experiment_plot_mocap_pos_x,
                plot_mocap_pos_y=experiment_plot_mocap_pos_y,
                plot_mocap_pos_z=experiment_plot_mocap_pos_z,
                plot_mocap_roll=experiment_plot_mocap_roll,
                plot_mocap_pitch=experiment_plot_mocap_pitch,
                plot_mocap_yaw=experiment_plot_mocap_yaw,
                mocap_name=selected_mocap_name,
                plot_idx_offset=plot_idx_offset,
                depth_min_m=args.depth_min_m,
                depth_max_m=args.depth_max_m,
            )
            max_diff = max(max_diff, trial_max_diff)
    finally:
        writer.release()

    print(f"Mode: {args.mode}")
    print(f"Trials rendered: {len(trials)}")
    print(
        "Mocap body: "
        f"{selected_mocap_name} (ID {selected_mocap_rb_id}, kind={selected_mocap_kind})"
    )
    print(f"Saved {output_path}")
    print(f"COM offset (body y): {args.com_offset_y_m:.6f} m")
    if args.depth_min_m is not None and args.depth_max_m is not None:
        print(f"Depth colormap range: [{args.depth_min_m:.3f}, {args.depth_max_m:.3f}] m (fixed)")
    else:
        print(f"Depth colormap range: [{depth_min:.1f}, {depth_max:.1f}] raw units (percentile)")
    print(f"Max |t_force - t_frame|: {max_diff:.6f} s")


if __name__ == "__main__":
    main()
