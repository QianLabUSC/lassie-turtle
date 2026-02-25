#!/usr/bin/env python3
"""Post-process density experiment trials into a per-trial CSV summary.

Computes motion metrics from a selected mocap rigid body and right-side motor
torque metrics from robot_state currents. Force columns are emitted as NaN
placeholders until a torque->force mapping (e.g. Jacobian-based) is available.
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import re
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np


DATA_ROOT = Path(__file__).resolve().parent / "data"
DEFAULT_TORQUE_SCALE = 0.072
MOCAP_SPEED_SMOOTH_WINDOW_S = 0.02  # 20 ms moving-average smoothing before dp/dt
MOCAP_SPEED_DERIV_WINDOW_S = 0.02  # 20 ms forward time-span derivative for robust speed
DEFAULT_GSHEET_ID = "1B-EuHAwquNCQMVgdqTAPPxMhi-4xVDhihVAg2atVA38"
DEFAULT_GSHEET_WORKSHEET = "Run_Log_14x33_init"

MOCAP_RB_IDS_BY_KIND = {
    "empty": 2,
    "lead": 3,
    "resin": 5,
}

MOCAP_RB_NAMES = {
    2: "Empty Half Sphere",
    3: "Lead Half Sphere",
    5: "Resin Half Sphere",
}

RIGHT_MOTOR_CURRENT_KEYS = (
    "rightadduction_curr",
    "rightsweeping_curr",
)

CSV_COLUMNS: Sequence[str] = (
    "Session",
    "Trial",
    "Mocap_RB_ID",
    "Max_Abs_Delta_x_cm",
    "Max_Abs_Delta_y_cm",
    "Max_Abs_Delta_z_cm",
    "Max_Planar_Displacement_cm",
    "Max_Abs_Speed_x_cm_s",
    "Max_Abs_Speed_y_cm_s",
    "Max_Abs_Speed_z_cm_s",
    "P95_Abs_Speed_x_cm_s",
    "P95_Abs_Speed_y_cm_s",
    "P95_Abs_Speed_z_cm_s",
    "Max_Torque_rightadduction_Nm",
    "Mean_Torque_rightadduction_Nm",
    "Max_Torque_rightsweeping_Nm",
    "Mean_Torque_rightsweeping_Nm",
    "Max_Force_rightadduction_N",
    "Mean_Force_rightadduction_N",
    "Max_Force_rightsweeping_N",
    "Mean_Force_rightsweeping_N",
    "Compaction",
)


def _load_payload(path: Path) -> Dict[str, object]:
    data = np.load(path, allow_pickle=True)
    if isinstance(data, np.ndarray) and data.shape == ():
        payload = data.item()
        if isinstance(payload, dict):
            return payload
    if isinstance(data, dict):
        return data
    raise ValueError(f"Unexpected npy payload format in {path}")


def _trial_number(path: Path) -> Optional[int]:
    match = re.match(r"trial_(\d+)\.npy$", path.name)
    if match is None:
        return None
    return int(match.group(1))


def _list_trials(path: Path) -> List[Path]:
    if path.is_file():
        return [path]
    trials = [trial for trial in path.glob("trial_*.npy") if trial.is_file()]
    return sorted(trials, key=lambda p: (_trial_number(p) or 10**9, p.name))


def _latest_session_dir(data_root: Path) -> Path:
    if not data_root.exists():
        raise FileNotFoundError(f"Data root not found: {data_root}")
    sessions = [p for p in data_root.iterdir() if p.is_dir() and p.name.startswith("session_")]
    if not sessions:
        raise FileNotFoundError(f"No session folders found under {data_root}")
    return max(sessions, key=lambda p: p.name)


def _as_float_array(value: object) -> Optional[np.ndarray]:
    if not isinstance(value, np.ndarray):
        return None
    try:
        return np.asarray(value, dtype=float)
    except Exception:
        return None


def _get_mocap_state(payload: Dict[str, object], rb_id: int) -> Optional[Dict[str, object]]:
    # Prefer raw mocap for speed peaks; fall back to aligned mocap if needed.
    for container_key in ("mocap_raw", "mocap"):
        mocap = payload.get(container_key)
        if not isinstance(mocap, dict):
            continue
        state = mocap.get(str(rb_id), mocap.get(rb_id))
        if isinstance(state, dict):
            return state
    return None


def _available_mocap_ids(payload: Dict[str, object]) -> List[int]:
    ids: set[int] = set()
    for container_key in ("mocap_raw", "mocap"):
        mocap = payload.get(container_key)
        if not isinstance(mocap, dict):
            continue
        for key in mocap.keys():
            try:
                ids.add(int(key))
            except Exception:
                continue
    return sorted(ids)


def _select_mocap_rb_id(
    payload: Dict[str, object],
    requested_rb_id: Optional[int],
    trial_path: Path,
) -> int:
    ids = _available_mocap_ids(payload)
    if requested_rb_id is not None:
        if requested_rb_id not in ids:
            raise ValueError(
                f"{trial_path.name}: requested mocap RB ID {requested_rb_id} not found; available IDs: {ids}"
            )
        return requested_rb_id
    if len(ids) == 1:
        return ids[0]
    if not ids:
        raise ValueError(f"{trial_path.name}: no mocap rigid bodies found in payload")
    names = [f"{rb_id} ({MOCAP_RB_NAMES.get(rb_id, 'unknown')})" for rb_id in ids]
    raise ValueError(
        f"{trial_path.name}: multiple mocap rigid bodies found ({', '.join(names)}); "
        "specify --mocap-rb-id or --mocap-kind"
    )


def _finite_first_last(values: np.ndarray) -> Tuple[float, float]:
    if values.size == 0:
        return math.nan, math.nan
    finite = np.isfinite(values)
    if not np.any(finite):
        return math.nan, math.nan
    idx = np.flatnonzero(finite)
    return float(values[idx[0]]), float(values[idx[-1]])


def _abs_speed_samples_cm_s(position_m: np.ndarray, time_s: np.ndarray) -> np.ndarray:
    """Return |speed| samples (cm/s) using a forward time-window derivative.

    Speed is computed over an approximately fixed time span (default 20 ms):
      |p(t+Δ) - p(t)| / Δ
    after smoothing the position signal with a moving average.
    """
    if position_m.size < 2 or time_s.size < 2 or position_m.size != time_s.size:
        return np.empty((0,), dtype=float)

    mask = np.isfinite(position_m) & np.isfinite(time_s)
    pos = position_m[mask]
    t = time_s[mask]
    if pos.size < 2:
        return np.empty((0,), dtype=float)

    # Require strictly increasing time for derivative calculations.
    keep = np.ones(pos.shape, dtype=bool)
    keep[1:] = np.diff(t) > 0.0
    pos = pos[keep]
    t = t[keep]
    if pos.size < 2:
        return np.empty((0,), dtype=float)

    dt_pos = np.diff(t)
    if dt_pos.size == 0:
        return np.empty((0,), dtype=float)
    median_dt = float(np.median(dt_pos))
    if not math.isfinite(median_dt) or median_dt <= 0.0:
        return np.empty((0,), dtype=float)

    smooth_window_samples = max(1, int(round(MOCAP_SPEED_SMOOTH_WINDOW_S / median_dt)))
    pos_smooth = _moving_average_1d(pos, smooth_window_samples)

    target_time = t + float(MOCAP_SPEED_DERIV_WINDOW_S)
    future_idx = np.searchsorted(t, target_time, side="left")
    base_idx = np.arange(t.size, dtype=int)
    valid = future_idx < t.size
    if not np.any(valid):
        return np.empty((0,), dtype=float)

    base = base_idx[valid]
    fut = future_idx[valid]
    dt = t[fut] - t[base]
    dp = pos_smooth[fut] - pos_smooth[base]
    valid_dt = dt > 0.0
    if not np.any(valid_dt):
        return np.empty((0,), dtype=float)

    return np.abs(dp[valid_dt] / dt[valid_dt]) * 100.0


def _max_abs_speed_cm_s(position_m: np.ndarray, time_s: np.ndarray) -> float:
    speeds = _abs_speed_samples_cm_s(position_m, time_s)
    if speeds.size == 0:
        return math.nan
    return float(np.nanmax(speeds))


def _p95_abs_speed_cm_s(position_m: np.ndarray, time_s: np.ndarray) -> float:
    speeds = _abs_speed_samples_cm_s(position_m, time_s)
    if speeds.size == 0:
        return math.nan
    return float(np.nanpercentile(speeds, 95.0))


def _max_abs_and_mean_abs(values: np.ndarray) -> Tuple[float, float]:
    if values.size == 0:
        return math.nan, math.nan
    vals = np.asarray(values, dtype=float)
    vals = vals[np.isfinite(vals)]
    if vals.size == 0:
        return math.nan, math.nan
    abs_vals = np.abs(vals)
    return float(np.max(abs_vals)), float(np.mean(abs_vals))


def _max_abs_sum_abs_count(values: np.ndarray) -> Tuple[float, float, int]:
    if values.size == 0:
        return math.nan, 0.0, 0
    vals = np.asarray(values, dtype=float)
    vals = vals[np.isfinite(vals)]
    if vals.size == 0:
        return math.nan, 0.0, 0
    abs_vals = np.abs(vals)
    return float(np.max(abs_vals)), float(np.sum(abs_vals)), int(abs_vals.size)


def _moving_average_1d(values: np.ndarray, window_samples: int) -> np.ndarray:
    if values.size == 0 or window_samples <= 1:
        return values

    window_samples = int(window_samples)
    if window_samples % 2 == 0:
        window_samples += 1
    window_samples = min(window_samples, int(values.size))
    if window_samples <= 1:
        return values
    if window_samples % 2 == 0:
        window_samples -= 1
    if window_samples <= 1:
        return values

    pad = window_samples // 2
    kernel = np.ones(window_samples, dtype=float) / float(window_samples)
    padded = np.pad(values, (pad, pad), mode="edge")
    return np.convolve(padded, kernel, mode="valid")


def _robot_state_for_metrics(payload: Dict[str, object]) -> Dict[str, object]:
    robot_state_raw = payload.get("robot_state_raw")
    if isinstance(robot_state_raw, dict):
        return robot_state_raw
    robot_state = payload.get("robot_state")
    if isinstance(robot_state, dict):
        return robot_state
    raise ValueError("missing robot_state/robot_state_raw")


def _compute_motion_metrics(mocap_state: Dict[str, object]) -> Dict[str, float]:
    t = _as_float_array(mocap_state.get("time"))
    x = _as_float_array(mocap_state.get("position_x"))
    y = _as_float_array(mocap_state.get("position_y"))
    z = _as_float_array(mocap_state.get("position_z"))
    if t is None or x is None or y is None or z is None:
        raise ValueError("mocap state missing time/position_x/position_y/position_z arrays")

    if not (len(t) == len(x) == len(y) == len(z)):
        raise ValueError("mocap position arrays length mismatch")

    x0, x1 = _finite_first_last(x)
    y0, y1 = _finite_first_last(y)
    z0, z1 = _finite_first_last(z)

    delta_x_cm = (x1 - x0) * 100.0 if np.isfinite([x0, x1]).all() else math.nan
    delta_y_cm = (y1 - y0) * 100.0 if np.isfinite([y0, y1]).all() else math.nan
    delta_z_cm = (z1 - z0) * 100.0 if np.isfinite([z0, z1]).all() else math.nan

    if math.isfinite(delta_x_cm) and math.isfinite(delta_z_cm):
        planar_disp_cm = float(math.hypot(delta_x_cm, delta_z_cm))
    else:
        planar_disp_cm = math.nan

    speed_x = _abs_speed_samples_cm_s(x, t)
    speed_y = _abs_speed_samples_cm_s(y, t)
    speed_z = _abs_speed_samples_cm_s(z, t)
    max_x = float(np.nanmax(speed_x)) if speed_x.size else math.nan
    max_y = float(np.nanmax(speed_y)) if speed_y.size else math.nan
    max_z = float(np.nanmax(speed_z)) if speed_z.size else math.nan
    p95_x = float(np.nanpercentile(speed_x, 95.0)) if speed_x.size else math.nan
    p95_y = float(np.nanpercentile(speed_y, 95.0)) if speed_y.size else math.nan
    p95_z = float(np.nanpercentile(speed_z, 95.0)) if speed_z.size else math.nan

    return {
        "Max_Abs_Delta_x_cm": abs(float(delta_x_cm)) if math.isfinite(delta_x_cm) else math.nan,
        "Max_Abs_Delta_y_cm": abs(float(delta_y_cm)) if math.isfinite(delta_y_cm) else math.nan,
        "Max_Abs_Delta_z_cm": abs(float(delta_z_cm)) if math.isfinite(delta_z_cm) else math.nan,
        "Max_Planar_Displacement_cm": planar_disp_cm,
        "Max_Abs_Speed_x_cm_s": max_x,
        "Max_Abs_Speed_y_cm_s": max_y,
        "Max_Abs_Speed_z_cm_s": max_z,
        "P95_Abs_Speed_x_cm_s": p95_x,
        "P95_Abs_Speed_y_cm_s": p95_y,
        "P95_Abs_Speed_z_cm_s": p95_z,
    }


def _compute_right_torque_metrics(robot_state: Dict[str, object], torque_scale: float) -> Dict[str, float]:
    metrics: Dict[str, float] = {}
    for motor_key in RIGHT_MOTOR_CURRENT_KEYS:
        current = _as_float_array(robot_state.get(motor_key))
        if current is None:
            raise ValueError(f"robot_state missing '{motor_key}' array")
        torque = current * float(torque_scale)
        motor_name = motor_key.replace("_curr", "")
        max_tau, mean_tau = _max_abs_and_mean_abs(torque)
        metrics[f"Max_Torque_{motor_name}_Nm"] = max_tau
        metrics[f"Mean_Torque_{motor_name}_Nm"] = mean_tau
    return metrics


def _compute_right_torque_aggregate_components(
    robot_state: Dict[str, object],
    torque_scale: float,
) -> Dict[str, Tuple[float, float, int]]:
    components: Dict[str, Tuple[float, float, int]] = {}
    for motor_key in RIGHT_MOTOR_CURRENT_KEYS:
        current = _as_float_array(robot_state.get(motor_key))
        if current is None:
            raise ValueError(f"robot_state missing '{motor_key}' array")
        torque = current * float(torque_scale)
        motor_name = motor_key.replace("_curr", "")
        components[motor_name] = _max_abs_sum_abs_count(torque)
    return components


def _force_placeholder_metrics() -> Dict[str, float]:
    return {
        "Max_Force_rightadduction_N": math.nan,
        "Mean_Force_rightadduction_N": math.nan,
        "Max_Force_rightsweeping_N": math.nan,
        "Mean_Force_rightsweeping_N": math.nan,
    }


def _compaction_placeholder_metrics() -> Dict[str, float]:
    return {
        "Compaction": math.nan,
    }


def _format_value(value: object) -> object:
    if isinstance(value, (float, np.floating)):
        if not math.isfinite(float(value)):
            return "nan"
        return f"{float(value):.6f}"
    return value


def _format_value_for_sheet(value: object) -> str:
    formatted = _format_value(value)
    return "" if formatted is None else str(formatted)


def _row_for_trial(
    trial_path: Path,
    requested_rb_id: Optional[int],
    torque_scale: float,
) -> Dict[str, object]:
    payload = _load_payload(trial_path)
    rb_id = _select_mocap_rb_id(payload, requested_rb_id=requested_rb_id, trial_path=trial_path)

    mocap_state = _get_mocap_state(payload, rb_id)
    if mocap_state is None:
        raise ValueError(f"{trial_path.name}: mocap state for RB ID {rb_id} not found")
    robot_state = _robot_state_for_metrics(payload)

    row: Dict[str, object] = {
        "Session": trial_path.parent.name,
        "Trial": _trial_number(trial_path) if _trial_number(trial_path) is not None else trial_path.name,
        "Mocap_RB_ID": rb_id,
    }
    row.update(_compute_motion_metrics(mocap_state))
    row.update(_compute_right_torque_metrics(robot_state, torque_scale=torque_scale))
    row.update(_force_placeholder_metrics())
    row.update(_compaction_placeholder_metrics())
    return row


def _nanmean(values: Iterable[float]) -> float:
    arr = np.asarray(list(values), dtype=float)
    if arr.size == 0:
        return math.nan
    finite = np.isfinite(arr)
    if not np.any(finite):
        return math.nan
    return float(np.mean(arr[finite]))


def _row_for_experiment(
    trial_paths: Sequence[Path],
    requested_rb_id: Optional[int],
    torque_scale: float,
) -> Dict[str, object]:
    if not trial_paths:
        raise ValueError("no trial paths provided")

    session_name = trial_paths[0].parent.name
    selected_rb_id_for_experiment: Optional[int] = None

    trial_motion_rows: List[Dict[str, float]] = []
    speed_samples_by_axis: Dict[str, List[np.ndarray]] = {"x": [], "y": [], "z": []}

    # Aggregate torque stats across all samples of all trials (experiment-level).
    max_abs_tau: Dict[str, float] = {"rightadduction": math.nan, "rightsweeping": math.nan}
    sum_abs_tau: Dict[str, float] = {"rightadduction": 0.0, "rightsweeping": 0.0}
    count_tau: Dict[str, int] = {"rightadduction": 0, "rightsweeping": 0}

    for trial_path in trial_paths:
        payload = _load_payload(trial_path)
        rb_id = _select_mocap_rb_id(payload, requested_rb_id=requested_rb_id, trial_path=trial_path)
        if selected_rb_id_for_experiment is None:
            selected_rb_id_for_experiment = rb_id
        elif selected_rb_id_for_experiment != rb_id:
            raise ValueError(
                f"Inconsistent selected mocap RB IDs across experiment: "
                f"{selected_rb_id_for_experiment} vs {rb_id} in {trial_path.name}"
            )

        mocap_state = _get_mocap_state(payload, rb_id)
        if mocap_state is None:
            raise ValueError(f"{trial_path.name}: mocap state for RB ID {rb_id} not found")
        robot_state = _robot_state_for_metrics(payload)

        trial_motion_rows.append(_compute_motion_metrics(mocap_state))
        t_arr = _as_float_array(mocap_state.get("time"))
        x_arr = _as_float_array(mocap_state.get("position_x"))
        y_arr = _as_float_array(mocap_state.get("position_y"))
        z_arr = _as_float_array(mocap_state.get("position_z"))
        if t_arr is not None and x_arr is not None and y_arr is not None and z_arr is not None:
            speed_samples_by_axis["x"].append(_abs_speed_samples_cm_s(x_arr, t_arr))
            speed_samples_by_axis["y"].append(_abs_speed_samples_cm_s(y_arr, t_arr))
            speed_samples_by_axis["z"].append(_abs_speed_samples_cm_s(z_arr, t_arr))

        components = _compute_right_torque_aggregate_components(robot_state, torque_scale=torque_scale)
        for motor_name, (trial_max_abs, trial_sum_abs, trial_count) in components.items():
            prev_max = max_abs_tau[motor_name]
            if math.isfinite(trial_max_abs):
                max_abs_tau[motor_name] = (
                    trial_max_abs if not math.isfinite(prev_max) else max(prev_max, trial_max_abs)
                )
            sum_abs_tau[motor_name] += float(trial_sum_abs)
            count_tau[motor_name] += int(trial_count)

    if selected_rb_id_for_experiment is None:
        raise ValueError("failed to determine mocap rigid body ID for experiment")

    row: Dict[str, object] = {
        "Session": session_name,
        "Trial": f"ALL_{len(trial_paths)}",
        "Mocap_RB_ID": selected_rb_id_for_experiment,
        # In experiment mode, displacement metrics are maxima across trials.
        "Max_Abs_Delta_x_cm": _nanmean([]),
        "Max_Abs_Delta_y_cm": _nanmean([]),
        "Max_Abs_Delta_z_cm": _nanmean([]),
        "Max_Planar_Displacement_cm": _nanmean([]),
        # Peak speed metrics are maxima across all trials (experiment-level peak).
        "Max_Abs_Speed_x_cm_s": _nanmean([]),
        "Max_Abs_Speed_y_cm_s": _nanmean([]),
        "Max_Abs_Speed_z_cm_s": _nanmean([]),
        "P95_Abs_Speed_x_cm_s": _nanmean([]),
        "P95_Abs_Speed_y_cm_s": _nanmean([]),
        "P95_Abs_Speed_z_cm_s": _nanmean([]),
    }
    for disp_key in (
        "Max_Abs_Delta_x_cm",
        "Max_Abs_Delta_y_cm",
        "Max_Abs_Delta_z_cm",
        "Max_Planar_Displacement_cm",
    ):
        vals = np.asarray([m.get(disp_key, math.nan) for m in trial_motion_rows], dtype=float)
        finite_vals = vals[np.isfinite(vals)]
        row[disp_key] = float(np.max(finite_vals)) if finite_vals.size else math.nan

    for axis_suffix in ("x", "y", "z"):
        chunks = [arr for arr in speed_samples_by_axis[axis_suffix] if isinstance(arr, np.ndarray) and arr.size > 0]
        if chunks:
            all_speeds = np.concatenate(chunks)
            finite = all_speeds[np.isfinite(all_speeds)]
        else:
            finite = np.empty((0,), dtype=float)
        row[f"Max_Abs_Speed_{axis_suffix}_cm_s"] = float(np.max(finite)) if finite.size else math.nan
        row[f"P95_Abs_Speed_{axis_suffix}_cm_s"] = float(np.percentile(finite, 95.0)) if finite.size else math.nan

    for motor_name in ("rightadduction", "rightsweeping"):
        row[f"Max_Torque_{motor_name}_Nm"] = max_abs_tau[motor_name]
        total_count = count_tau[motor_name]
        row[f"Mean_Torque_{motor_name}_Nm"] = (
            float(sum_abs_tau[motor_name] / total_count) if total_count > 0 else math.nan
        )

    row.update(_force_placeholder_metrics())
    row.update(_compaction_placeholder_metrics())
    return row


def _determine_output_path(source_path: Path, output_arg: Optional[Path]) -> Path:
    if output_arg is not None:
        return output_arg
    if source_path.is_file():
        return source_path.with_name(f"{source_path.stem}_summary.csv")
    return source_path / "density_experiment_summary.csv"


def _import_gspread_or_raise():
    try:
        import gspread  # type: ignore
    except ModuleNotFoundError as exc:
        raise RuntimeError(
            "gspread is not installed. Install with: pip install gspread google-auth"
        ) from exc
    try:
        from google.oauth2.service_account import Credentials  # type: ignore
    except ModuleNotFoundError as exc:
        raise RuntimeError(
            "google-auth is not installed. Install with: pip install google-auth"
        ) from exc
    return gspread, Credentials


def _sheet_headers(worksheet) -> List[str]:
    headers = worksheet.row_values(1)
    return [str(h).strip() for h in headers]


def _ensure_columns_exist(worksheet, required_columns: Sequence[str]) -> List[str]:
    headers = _sheet_headers(worksheet)
    if not headers:
        # Initialize a new/empty tab with the columns we need.
        worksheet.update("A1", [list(required_columns)])
        return list(required_columns)

    missing = [col for col in required_columns if col not in headers]
    if not missing:
        return headers

    new_headers = headers + missing
    worksheet.update("A1", [new_headers])
    return new_headers


def _find_unique_row_index_by_key(
    worksheet,
    headers: Sequence[str],
    key_column: str,
    key_value: str,
) -> Optional[int]:
    if key_column not in headers:
        raise ValueError(f"Worksheet is missing key column '{key_column}'")
    key_col_idx_1based = headers.index(key_column) + 1

    col_values = worksheet.col_values(key_col_idx_1based)
    matches: List[int] = []
    for row_idx_1based, cell_value in enumerate(col_values, start=1):
        if row_idx_1based == 1:
            continue  # header
        if str(cell_value).strip() == key_value:
            matches.append(row_idx_1based)

    if len(matches) > 1:
        raise ValueError(
            f"Duplicate rows found for {key_column}='{key_value}' in worksheet (rows: {matches})"
        )
    return matches[0] if matches else None


def _row_to_sheet_cells(row: Dict[str, object], headers: Sequence[str]) -> List[str]:
    return [_format_value_for_sheet(row.get(col, "")) for col in headers]


def _upsert_rows_to_google_sheet(
    rows: Sequence[Dict[str, object]],
    sheet_id: str,
    worksheet_name: str,
    key_column: str,
    creds_env_var: str,
    dry_run: bool = False,
) -> None:
    if not rows:
        return

    creds_path = os.environ.get(creds_env_var)
    if not creds_path:
        raise RuntimeError(
            f"Credentials env var '{creds_env_var}' is not set. "
            "Set it to your service account JSON path."
        )

    gspread, Credentials = _import_gspread_or_raise()
    scopes = [
        "https://www.googleapis.com/auth/spreadsheets",
        "https://www.googleapis.com/auth/drive",
    ]
    creds = Credentials.from_service_account_file(creds_path, scopes=scopes)
    client = gspread.authorize(creds)
    spreadsheet = client.open_by_key(sheet_id)
    worksheet = spreadsheet.worksheet(worksheet_name)

    # Ensure the tab contains the key column + all auto metric columns, but do
    # not disturb existing manual columns.
    required_columns = [key_column] + [col for col in CSV_COLUMNS if col != key_column]
    headers = _ensure_columns_exist(worksheet, required_columns)

    # Guard against duplicate keys in the generated payload before touching the sheet.
    seen_keys: set[str] = set()
    for row in rows:
        key_value = _format_value_for_sheet(row.get(key_column, "")).strip()
        if not key_value:
            raise ValueError(f"Generated row is missing key column '{key_column}'")
        if key_value in seen_keys:
            raise ValueError(f"Duplicate generated rows for {key_column}='{key_value}'")
        seen_keys.add(key_value)

    for row in rows:
        key_value = _format_value_for_sheet(row.get(key_column, "")).strip()
        row_index = _find_unique_row_index_by_key(worksheet, headers, key_column=key_column, key_value=key_value)

        if row_index is None:
            # Append only one logical record; unspecified manual columns remain blank.
            new_row = _row_to_sheet_cells(row, headers)
            if dry_run:
                print(f"[dry-run] append row for {key_column}='{key_value}' to '{worksheet_name}'")
            else:
                worksheet.append_row(new_row, value_input_option="USER_ENTERED")
            continue

        # Update only the auto-generated columns present in the sheet.
        if dry_run:
            print(f"[dry-run] update row {row_index} for {key_column}='{key_value}' in '{worksheet_name}'")
            continue

        for col_name in CSV_COLUMNS:
            if col_name not in headers:
                continue
            col_index = headers.index(col_name) + 1
            cell_value = _format_value_for_sheet(row.get(col_name, ""))
            worksheet.update_cell(row_index, col_index, cell_value)


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(description="Post-process density experiment trial(s) into a CSV summary.")
    ap.add_argument(
        "session_or_trial",
        nargs="?",
        default=None,
        type=Path,
        help="Session directory or trial .npy file (default: latest session).",
    )
    ap.add_argument(
        "--summary-level",
        choices=["experiment", "trial"],
        default="experiment",
        help="Output one row for the whole experiment (all trials) or one row per trial.",
    )
    rb_group = ap.add_mutually_exclusive_group()
    rb_group.add_argument("--mocap-rb-id", type=int, default=None, help="Rigid body ID to process (e.g. 2, 3, 5)")
    rb_group.add_argument(
        "--mocap-kind",
        choices=sorted(MOCAP_RB_IDS_BY_KIND.keys()),
        default=None,
        help="Convenience alias for the mocap rigid body (empty, lead, resin)",
    )
    ap.add_argument(
        "--torque-scale",
        type=float,
        default=DEFAULT_TORQUE_SCALE,
        help="Current-to-torque scale factor (Nm per current unit).",
    )
    ap.add_argument("--output", type=Path, default=None, help="Output CSV path")
    ap.add_argument(
        "--push-google-sheet",
        action="store_true",
        help="Upsert generated rows into a Google Sheet tab keyed by Session.",
    )
    ap.add_argument(
        "--gsheet-id",
        default=DEFAULT_GSHEET_ID,
        help="Google Sheet file ID (defaults to DEFAULT_GSHEET_ID in this script)",
    )
    ap.add_argument(
        "--gsheet-worksheet",
        default=DEFAULT_GSHEET_WORKSHEET,
        help="Worksheet/tab name to upsert into (defaults to DEFAULT_GSHEET_WORKSHEET in this script)",
    )
    ap.add_argument(
        "--gsheet-key-column",
        default="Session",
        help="Column name used to match existing rows (default: Session)",
    )
    ap.add_argument(
        "--gsheet-creds-env",
        default="GOOGLE_APPLICATION_CREDENTIALS",
        help="Environment variable containing path to service-account JSON credentials.",
    )
    ap.add_argument(
        "--dry-run-google",
        action="store_true",
        help="Validate and print intended Google Sheets actions without writing.",
    )
    return ap.parse_args()


def main() -> int:
    args = parse_args()

    source_path = _latest_session_dir(DATA_ROOT) if args.session_or_trial is None else Path(str(args.session_or_trial).strip())
    trial_paths = _list_trials(source_path)
    if not trial_paths:
        print(f"No trial_*.npy files found in {source_path}")
        return 1

    requested_rb_id = args.mocap_rb_id
    if requested_rb_id is None and args.mocap_kind is not None:
        requested_rb_id = MOCAP_RB_IDS_BY_KIND[args.mocap_kind]

    output_path = _determine_output_path(source_path, args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    rows: List[Dict[str, object]] = []
    errors = 0
    if args.summary_level == "experiment":
        try:
            row = _row_for_experiment(
                trial_paths,
                requested_rb_id=requested_rb_id,
                torque_scale=float(args.torque_scale),
            )
        except Exception as exc:
            print(f"EXPERIMENT: ERROR - {exc}")
            return 1
        rows.append(row)
        print(
            f"experiment: processed {len(trial_paths)} trial(s) "
            f"(RB {row['Mocap_RB_ID']}, max planar x-z disp {row['Max_Planar_Displacement_cm']:.3f} cm)"
        )
    else:
        for trial_path in trial_paths:
            try:
                row = _row_for_trial(trial_path, requested_rb_id=requested_rb_id, torque_scale=float(args.torque_scale))
            except Exception as exc:
                print(f"{trial_path.name}: ERROR - {exc}")
                errors += 1
                continue
            rows.append(row)
            print(
                f"{trial_path.name}: processed "
                f"(RB {row['Mocap_RB_ID']}, "
                f"planar x-z disp {row['Max_Planar_Displacement_cm']:.3f} cm)"
            )

    if not rows:
        print("No rows generated.")
        return 1

    with open(output_path, "w", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=list(CSV_COLUMNS), extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow({col: _format_value(row.get(col, "")) for col in CSV_COLUMNS})

    print(f"Wrote {len(rows)} row(s) to {output_path}")

    if args.push_google_sheet:
        if not args.gsheet_id or not args.gsheet_worksheet:
            print("Google Sheets push requested but --gsheet-id and --gsheet-worksheet are required.")
            return 1
        try:
            _upsert_rows_to_google_sheet(
                rows=rows,
                sheet_id=str(args.gsheet_id).strip(),
                worksheet_name=str(args.gsheet_worksheet).strip(),
                key_column=str(args.gsheet_key_column).strip(),
                creds_env_var=str(args.gsheet_creds_env).strip(),
                dry_run=bool(args.dry_run_google),
            )
        except Exception as exc:
            print(f"Google Sheets upsert failed: {exc}")
            return 1
        if args.dry_run_google:
            print("Google Sheets dry-run completed.")
        else:
            print(
                f"Upserted {len(rows)} row(s) into Google Sheet {args.gsheet_id} / tab '{args.gsheet_worksheet}' "
                f"using key '{args.gsheet_key_column}'."
            )

    return 1 if errors else 0


if __name__ == "__main__":
    raise SystemExit(main())


# Change row updates to a single batch row write instead of per-cell writes:
# one worksheet.update(...) call for the whole row range (e.g. A12:Z12)