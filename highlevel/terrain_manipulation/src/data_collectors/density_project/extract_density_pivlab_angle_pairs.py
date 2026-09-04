#!/usr/bin/env python3
"""Extract PIVLab frame pairs from density collector trials by target flipper angle."""

from __future__ import annotations

import argparse
import ast
import csv
import math
from pathlib import Path
from typing import Dict, List, Mapping, Optional, Sequence, Tuple

import cv2
import numpy as np


DEFAULT_SESSION_DIR = (
    Path(__file__).resolve().parents[3]
    / "data"
    / "session_20260314_121349"
)
DEFAULT_THRESHOLD_SCRIPT = Path(__file__).with_name("capture_pivlab_frame_pair_threshold.py")
DEFAULT_OUTPUT_DIR_NAME = "pivlab_angle_matched_pairs"
DEFAULT_ALL_FRAMES_OUTPUT_DIR_NAME = "pivlab_all_trial_frames"
DEFAULT_CAMERA_INDEX = 0
DEFAULT_SECOND_FRAME_OFFSET = 1


class _SafeEval(ast.NodeVisitor):
    def __init__(self, names: Mapping[str, float]) -> None:
        self.names = names

    def visit_Expression(self, node: ast.Expression) -> float:
        return self.visit(node.body)

    def visit_Constant(self, node: ast.Constant) -> float:
        if isinstance(node.value, (int, float)):
            return float(node.value)
        raise ValueError(f"Unsupported constant in threshold expression: {node.value!r}")

    def visit_Name(self, node: ast.Name) -> float:
        if node.id in self.names:
            return float(self.names[node.id])
        raise ValueError(f"Unsupported name in threshold expression: {node.id}")

    def visit_UnaryOp(self, node: ast.UnaryOp) -> float:
        value = self.visit(node.operand)
        if isinstance(node.op, ast.UAdd):
            return value
        if isinstance(node.op, ast.USub):
            return -value
        raise ValueError("Unsupported unary operator in threshold expression.")

    def visit_BinOp(self, node: ast.BinOp) -> float:
        left = self.visit(node.left)
        right = self.visit(node.right)
        if isinstance(node.op, ast.Add):
            return left + right
        if isinstance(node.op, ast.Sub):
            return left - right
        if isinstance(node.op, ast.Mult):
            return left * right
        if isinstance(node.op, ast.Div):
            return left / right
        raise ValueError("Unsupported binary operator in threshold expression.")

    def visit_Call(self, node: ast.Call) -> float:
        if (
            isinstance(node.func, ast.Attribute)
            and isinstance(node.func.value, ast.Name)
            and node.func.value.id == "math"
            and node.func.attr == "radians"
            and len(node.args) == 1
            and not node.keywords
        ):
            return math.radians(self.visit(node.args[0]))
        raise ValueError("Unsupported function call in threshold expression.")


def _eval_float_expr(node: ast.AST, names: Mapping[str, float]) -> float:
    return _SafeEval(names).visit(ast.Expression(node))


def load_threshold_angles(threshold_script: Path) -> Tuple[float, float]:
    """Read the current default trigger angles without importing the hardware script."""
    tree = ast.parse(threshold_script.read_text(encoding="utf-8"), filename=str(threshold_script))
    names: Dict[str, float] = {}
    wanted = {
        "DEFAULT_SWEEPING_OFFSET_DEG",
        "DEFAULT_ADDUCTION_OFFSET_DEG",
        "DEFAULT_TRIGGER_RIGHT_ADDUCTION_RAD",
        "DEFAULT_TRIGGER_RIGHT_SWEEPING_RAD",
    }
    for node in tree.body:
        if not isinstance(node, ast.Assign) or len(node.targets) != 1:
            continue
        target = node.targets[0]
        if not isinstance(target, ast.Name) or target.id not in wanted:
            continue
        names[target.id] = _eval_float_expr(node.value, names)

    try:
        return (
            names["DEFAULT_TRIGGER_RIGHT_ADDUCTION_RAD"],
            names["DEFAULT_TRIGGER_RIGHT_SWEEPING_RAD"],
        )
    except KeyError as exc:
        raise RuntimeError(f"Could not find threshold angle in {threshold_script}") from exc


def _trial_sort_key(path: Path) -> Tuple[int, str]:
    stem = path.stem
    try:
        return int(stem.rsplit("_", 1)[1]), stem
    except (IndexError, ValueError):
        return 10**9, stem


def _load_trial(path: Path) -> Dict[str, object]:
    loaded = np.load(path, allow_pickle=True)
    try:
        payload = loaded.item()
    except ValueError as exc:
        raise RuntimeError(f"Expected {path} to contain one saved payload dict.") from exc
    if not isinstance(payload, dict):
        raise RuntimeError(f"Expected {path} to contain a dict payload.")
    return payload


def _nearest_indices(sample_times: np.ndarray, query_times: np.ndarray) -> np.ndarray:
    if sample_times.size == 0:
        raise RuntimeError("No robot_state_raw samples are available for alignment.")
    idx = np.searchsorted(sample_times, query_times, side="left")
    idx = np.clip(idx, 0, sample_times.size - 1)
    prev_idx = np.clip(idx - 1, 0, sample_times.size - 1)
    next_idx = idx
    prev_diff = np.abs(query_times - sample_times[prev_idx])
    next_diff = np.abs(query_times - sample_times[next_idx])
    return np.where(prev_diff <= next_diff, prev_idx, next_idx)


def _align_state_to_camera(
    robot_state_raw: Mapping[str, np.ndarray],
    camera_times: np.ndarray,
    method: str,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    state_times = np.asarray(robot_state_raw["time"], dtype=float)
    adduction_turns = np.asarray(robot_state_raw["rightadduction_pos"], dtype=float)
    sweeping_turns = np.asarray(robot_state_raw["rightsweeping_pos"], dtype=float)

    if method == "nearest":
        indices = _nearest_indices(state_times, camera_times)
        aligned_times = state_times[indices]
        aligned_adduction_turns = adduction_turns[indices]
        aligned_sweeping_turns = sweeping_turns[indices]
    elif method == "interp":
        if state_times.size == 0:
            raise RuntimeError("No robot_state_raw samples are available for interpolation.")
        aligned_times = camera_times
        aligned_adduction_turns = np.interp(camera_times, state_times, adduction_turns)
        aligned_sweeping_turns = np.interp(camera_times, state_times, sweeping_turns)
    else:
        raise RuntimeError(f"Unsupported alignment method: {method}")

    right_adduction_rad = -2.0 * math.pi * aligned_adduction_turns
    right_sweeping_rad = -2.0 * math.pi * aligned_sweeping_turns
    return aligned_times, right_adduction_rad, right_sweeping_rad


def _select_frame_index(
    right_adduction_rad: np.ndarray,
    right_sweeping_rad: np.ndarray,
    target_adduction_rad: float,
    target_sweeping_rad: float,
    second_frame_offset: int,
) -> Tuple[int, int, float, float, float]:
    errors = np.maximum(
        np.abs(right_adduction_rad - target_adduction_rad),
        np.abs(right_sweeping_rad - target_sweeping_rad),
    )
    first_idx = int(np.argmin(errors))
    second_idx = first_idx + int(second_frame_offset)
    if second_idx < 0 or second_idx >= right_adduction_rad.size:
        raise RuntimeError(
            f"Selected frame {first_idx} but second frame offset {second_frame_offset} "
            f"would produce out-of-range frame {second_idx}."
        )

    adduction_error = float(abs(right_adduction_rad[first_idx] - target_adduction_rad))
    sweeping_error = float(abs(right_sweeping_rad[first_idx] - target_sweeping_rad))
    max_error = float(errors[first_idx])
    return first_idx, second_idx, adduction_error, sweeping_error, max_error


def _to_grayscale(frame: np.ndarray) -> np.ndarray:
    return cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)


def _write_image(path: Path, image: np.ndarray) -> None:
    if not cv2.imwrite(str(path), image):
        raise RuntimeError(f"Failed to write image: {path}")


def _load_aligned_trial(
    trial_path: Path, camera_index: int, align_method: str
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    payload = _load_trial(trial_path)
    rgb_key = f"rgb_{camera_index}"
    camera_time_key = f"camera_time_{camera_index}"
    if rgb_key not in payload or camera_time_key not in payload:
        raise RuntimeError(f"{trial_path.name} does not contain {rgb_key}/{camera_time_key}.")
    if "robot_state_raw" not in payload:
        raise RuntimeError(f"{trial_path.name} does not contain robot_state_raw.")

    frames = np.asarray(payload[rgb_key])
    camera_times = np.asarray(payload[camera_time_key], dtype=float)
    robot_state_raw = payload["robot_state_raw"]
    if not isinstance(robot_state_raw, Mapping):
        raise RuntimeError(f"{trial_path.name} has invalid robot_state_raw data.")
    if frames.shape[0] != camera_times.size:
        raise RuntimeError(
            f"{trial_path.name} has {frames.shape[0]} frames but {camera_times.size} camera timestamps."
        )
    state_times, right_adduction_rad, right_sweeping_rad = _align_state_to_camera(
        robot_state_raw=robot_state_raw,
        camera_times=camera_times,
        method=align_method,
    )
    return frames, camera_times, state_times, right_adduction_rad, right_sweeping_rad


def _extract_trial_pair(
    trial_path: Path,
    output_dir: Path,
    camera_index: int,
    target_adduction_rad: float,
    target_sweeping_rad: float,
    second_frame_offset: int,
    frames: np.ndarray,
    camera_times: np.ndarray,
    state_times: np.ndarray,
    right_adduction_rad: np.ndarray,
    right_sweeping_rad: np.ndarray,
) -> Dict[str, object]:
    first_idx, second_idx, adduction_error, sweeping_error, max_error = _select_frame_index(
        right_adduction_rad=right_adduction_rad,
        right_sweeping_rad=right_sweeping_rad,
        target_adduction_rad=target_adduction_rad,
        target_sweeping_rad=target_sweeping_rad,
        second_frame_offset=second_frame_offset,
    )

    trial_label = trial_path.stem
    gray_a = output_dir / f"{trial_label}_cam{camera_index}_frame_{first_idx:04d}_A_gray.png"
    gray_b = output_dir / f"{trial_label}_cam{camera_index}_frame_{second_idx:04d}_B_gray.png"
    _write_image(gray_a, _to_grayscale(frames[first_idx]))
    _write_image(gray_b, _to_grayscale(frames[second_idx]))

    return {
        "trial": trial_label,
        "source_trial_path": str(trial_path),
        "camera_index": camera_index,
        "frame_a_index": first_idx,
        "frame_b_index": second_idx,
        "frame_a_time_s": float(camera_times[first_idx]),
        "frame_b_time_s": float(camera_times[second_idx]),
        "state_time_for_frame_a_s": float(state_times[first_idx]),
        "target_adduction_rad": float(target_adduction_rad),
        "target_sweeping_rad": float(target_sweeping_rad),
        "frame_a_adduction_rad": float(right_adduction_rad[first_idx]),
        "frame_a_sweeping_rad": float(right_sweeping_rad[first_idx]),
        "adduction_error_rad": adduction_error,
        "sweeping_error_rad": sweeping_error,
        "max_error_rad": max_error,
        "adduction_error_deg": math.degrees(adduction_error),
        "sweeping_error_deg": math.degrees(sweeping_error),
        "max_error_deg": math.degrees(max_error),
        "gray_image_a": str(gray_a),
        "gray_image_b": str(gray_b),
    }


def _extract_all_trial_frames(
    trial_path: Path,
    output_root: Path,
    camera_index: int,
    second_frame_offset: int,
    frames: np.ndarray,
    camera_times: np.ndarray,
    state_times: np.ndarray,
    right_adduction_rad: np.ndarray,
    right_sweeping_rad: np.ndarray,
) -> Dict[str, object]:
    trial_label = trial_path.stem
    trial_dir = output_root / trial_label
    frames_dir = trial_dir / "frames"
    pairs_dir = trial_dir / f"pivlab_pairs_stride_{abs(int(second_frame_offset))}"
    trial_dir.mkdir(parents=True, exist_ok=True)
    frames_dir.mkdir(parents=True, exist_ok=True)
    pairs_dir.mkdir(parents=True, exist_ok=True)

    rows: List[Mapping[str, object]] = []
    for frame_idx, frame in enumerate(frames):
        gray_path = frames_dir / f"{trial_label}_cam{camera_index}_frame_{frame_idx:04d}_gray.png"
        _write_image(gray_path, _to_grayscale(frame))
        rows.append(
            {
                "trial": trial_label,
                "source_trial_path": str(trial_path),
                "camera_index": camera_index,
                "frame_index": frame_idx,
                "frame_time_s": float(camera_times[frame_idx]),
                "state_time_s": float(state_times[frame_idx]),
                "right_adduction_rad": float(right_adduction_rad[frame_idx]),
                "right_sweeping_rad": float(right_sweeping_rad[frame_idx]),
                "right_adduction_deg": math.degrees(float(right_adduction_rad[frame_idx])),
                "right_sweeping_deg": math.degrees(float(right_sweeping_rad[frame_idx])),
                "gray_image": str(gray_path),
            }
        )

    summary_path = trial_dir / "frame_summary.csv"
    _write_summary(summary_path, rows)
    pair_rows: List[Mapping[str, object]] = []
    pair_idx = 0
    for first_idx in range(frames.shape[0]):
        second_idx = first_idx + int(second_frame_offset)
        if second_idx < 0 or second_idx >= frames.shape[0]:
            continue
        first_image = pairs_dir / f"pair_{pair_idx:04d}_A.png"
        second_image = pairs_dir / f"pair_{pair_idx:04d}_B.png"
        _write_image(first_image, _to_grayscale(frames[first_idx]))
        _write_image(second_image, _to_grayscale(frames[second_idx]))
        pair_rows.append(
            {
                "trial": trial_label,
                "source_trial_path": str(trial_path),
                "camera_index": camera_index,
                "frame_a_index": first_idx,
                "frame_b_index": second_idx,
                "frame_a_time_s": float(camera_times[first_idx]),
                "frame_b_time_s": float(camera_times[second_idx]),
                "dt_s": float(camera_times[second_idx] - camera_times[first_idx]),
                "frame_a_adduction_rad": float(right_adduction_rad[first_idx]),
                "frame_a_sweeping_rad": float(right_sweeping_rad[first_idx]),
                "frame_b_adduction_rad": float(right_adduction_rad[second_idx]),
                "frame_b_sweeping_rad": float(right_sweeping_rad[second_idx]),
                "frame_a_adduction_deg": math.degrees(float(right_adduction_rad[first_idx])),
                "frame_a_sweeping_deg": math.degrees(float(right_sweeping_rad[first_idx])),
                "frame_b_adduction_deg": math.degrees(float(right_adduction_rad[second_idx])),
                "frame_b_sweeping_deg": math.degrees(float(right_sweeping_rad[second_idx])),
                "gray_image_a": str(first_image),
                "gray_image_b": str(second_image),
            }
        )
        pair_idx += 1
    pair_summary_path = trial_dir / "consecutive_pair_summary.csv"
    _write_summary(pair_summary_path, pair_rows)
    return {
        "trial": trial_label,
        "source_trial_path": str(trial_path),
        "camera_index": camera_index,
        "frame_count": int(frames.shape[0]),
        "pair_count": len(pair_rows),
        "output_dir": str(trial_dir),
        "pivlab_pairs_dir": str(pairs_dir),
        "summary_csv": str(summary_path),
        "pair_summary_csv": str(pair_summary_path),
    }


def _write_summary(path: Path, rows: Sequence[Mapping[str, object]]) -> None:
    if not rows:
        return
    fieldnames = list(rows[0].keys())
    with open(path, "w", encoding="utf-8", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(
        description=(
            "Extract A/B PIVLab image pairs from density collector .npy trials by matching "
            "the right flipper angle to the current capture_pivlab threshold."
        )
    )
    ap.add_argument("--session-dir", type=Path, default=DEFAULT_SESSION_DIR)
    ap.add_argument("--threshold-script", type=Path, default=DEFAULT_THRESHOLD_SCRIPT)
    ap.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help="Destination folder. Defaults to SESSION_DIR/pivlab_angle_matched_pairs.",
    )
    ap.add_argument(
        "--all-frames-output-dir",
        type=Path,
        default=None,
        help="Destination folder for all trial frames. Defaults to SESSION_DIR/pivlab_all_trial_frames.",
    )
    ap.add_argument("--camera-index", type=int, default=DEFAULT_CAMERA_INDEX, choices=(0, 1))
    ap.add_argument(
        "--second-frame-offset",
        type=int,
        default=DEFAULT_SECOND_FRAME_OFFSET,
        help="Frame offset from A to B. Default 1 means B is the next frame.",
    )
    ap.add_argument(
        "--align-method",
        choices=("interp", "nearest"),
        default="interp",
        help="How to estimate robot angles at camera frame times.",
    )
    ap.add_argument(
        "--target-adduction-rad",
        type=float,
        default=None,
        help="Override target right adduction angle in radians.",
    )
    ap.add_argument(
        "--target-sweeping-rad",
        type=float,
        default=None,
        help="Override target right sweeping angle in radians.",
    )
    return ap.parse_args()


def main() -> int:
    args = parse_args()
    session_dir = args.session_dir
    if not session_dir.is_dir():
        raise SystemExit(f"Session directory does not exist: {session_dir}")

    target_adduction_rad, target_sweeping_rad = load_threshold_angles(args.threshold_script)
    if args.target_adduction_rad is not None:
        target_adduction_rad = float(args.target_adduction_rad)
    if args.target_sweeping_rad is not None:
        target_sweeping_rad = float(args.target_sweeping_rad)

    output_dir = args.output_dir if args.output_dir is not None else session_dir / DEFAULT_OUTPUT_DIR_NAME
    output_dir.mkdir(parents=True, exist_ok=True)
    all_frames_output_dir = (
        args.all_frames_output_dir
        if args.all_frames_output_dir is not None
        else session_dir / DEFAULT_ALL_FRAMES_OUTPUT_DIR_NAME
    )
    all_frames_output_dir.mkdir(parents=True, exist_ok=True)

    trial_paths = sorted(session_dir.glob("trial_*.npy"), key=_trial_sort_key)
    if not trial_paths:
        raise SystemExit(f"No trial_*.npy files found in {session_dir}")

    print(
        "Matching target angle: "
        f"adduction={target_adduction_rad:.9f} rad ({math.degrees(target_adduction_rad):.2f} deg), "
        f"sweeping={target_sweeping_rad:.9f} rad ({math.degrees(target_sweeping_rad):.2f} deg)"
    )
    print(
        f"Using camera {args.camera_index}, align_method={args.align_method}, "
        f"matched_output={output_dir}, all_frames_output={all_frames_output_dir}"
    )

    rows: List[Mapping[str, object]] = []
    all_frame_rows: List[Mapping[str, object]] = []
    for trial_path in trial_paths:
        frames, camera_times, state_times, right_adduction_rad, right_sweeping_rad = _load_aligned_trial(
            trial_path=trial_path,
            camera_index=int(args.camera_index),
            align_method=str(args.align_method),
        )
        row = _extract_trial_pair(
            trial_path=trial_path,
            output_dir=output_dir,
            camera_index=int(args.camera_index),
            target_adduction_rad=target_adduction_rad,
            target_sweeping_rad=target_sweeping_rad,
            second_frame_offset=int(args.second_frame_offset),
            frames=frames,
            camera_times=camera_times,
            state_times=state_times,
            right_adduction_rad=right_adduction_rad,
            right_sweeping_rad=right_sweeping_rad,
        )
        all_frame_row = _extract_all_trial_frames(
            trial_path=trial_path,
            output_root=all_frames_output_dir,
            camera_index=int(args.camera_index),
            second_frame_offset=int(args.second_frame_offset),
            frames=frames,
            camera_times=camera_times,
            state_times=state_times,
            right_adduction_rad=right_adduction_rad,
            right_sweeping_rad=right_sweeping_rad,
        )
        rows.append(row)
        all_frame_rows.append(all_frame_row)
        print(
            f"{row['trial']}: A=frame {row['frame_a_index']}, B=frame {row['frame_b_index']}, "
            f"max_error={float(row['max_error_rad']):.6f} rad "
            f"({float(row['max_error_deg']):.3f} deg), "
            f"all_frames={all_frame_row['frame_count']}"
        )

    summary_path = output_dir / "matched_frame_summary.csv"
    _write_summary(summary_path, rows)
    print(f"Saved summary to {summary_path}")
    all_frames_summary_path = all_frames_output_dir / "all_frames_summary.csv"
    _write_summary(all_frames_summary_path, all_frame_rows)
    print(f"Saved all-frame trial summary to {all_frames_summary_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
