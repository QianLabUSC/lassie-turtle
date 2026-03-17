#!/usr/bin/env python3
"""Plot per-trial mocap displacement across sessions with mean/std bands.

For each input session, this script loads trial_*.npy files, computes per-trial
net displacement (last finite sample minus first finite sample) for x, y', z'
from rotated mocap position arrays, then plots:
  - mean displacement across sessions
  - +/- 1 std band across sessions

Example:
    python3 highlevel/terrain_manipulation/plot_trial_displacement_band_across_sessions.py \
      highlevel/terrain_manipulation/data/session_20260301_101010 \
      highlevel/terrain_manipulation/data/session_20260302_101010 \
      highlevel/terrain_manipulation/data/session_20260303_101010 \
      --mocap-rb-id 6
"""

from __future__ import annotations

import argparse
import math
import re
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import matplotlib.pyplot as plt
import numpy as np


DATA_ROOT = Path(__file__).resolve().parent / "data"
# Default sessions used when no positional session args are provided.

# empty
# DEFAULT_SESSION_NAMES: Sequence[str] = (
#     "session_20260313_111621",
#     "session_20260313_112741",
#     "session_20260313_125836",
#     "session_20260313_121725",
#     "session_20260313_123622",
# )

# lead
# DEFAULT_SESSION_NAMES: Sequence[str] = (
#     "session_20260314_114731",
#     "session_20260314_115216",
#     "session_20260314_115730",
#     "session_20260314_120728",
#     "session_20260314_121349",
#     "session_20260314_121907"
# )

# # steel
# DEFAULT_SESSION_NAMES: Sequence[str] = (
#     "session_20260314_122949",
#     "session_20260314_123416",
#     "session_20260314_123744",
#     "session_20260314_124126",
#     "session_20260314_124658",
#     "session_20260314_125646"
# )

# resin
DEFAULT_SESSION_NAMES: Sequence[str] = (
"session_20260316_132543",
"session_20260316_142942",
"session_20260316_144047",
"session_20260316_145733",
"session_20260316_150341",
"session_20260316_152118",
"session_20260316_152712"
)


MOCAP_RB_IDS_BY_KIND: Dict[str, int] = {
    "empty": 2,
    "lead": 3,
    "resin": 5,
    "steel": 6,
    "sand": 8,
}
# Used when neither --mocap-rb-id nor --mocap-kind is passed.
DEFAULT_MOCAP_KIND = "steel"


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


def _list_trials(session_dir: Path) -> List[Path]:
    trials = [p for p in session_dir.glob("trial_*.npy") if p.is_file()]
    return sorted(trials, key=lambda p: (_trial_number(p) or 10**9, p.name))


def _as_float_array(value: object) -> Optional[np.ndarray]:
    if not isinstance(value, np.ndarray):
        return None
    try:
        return np.asarray(value, dtype=float)
    except (TypeError, ValueError):
        return None


def _extract_first_mocap_series(mocap_state: Dict[str, object], keys: Sequence[str]) -> Optional[np.ndarray]:
    for key in keys:
        arr = _as_float_array(mocap_state.get(key))
        if arr is not None:
            return arr
    return None


def _get_mocap_state(payload: Dict[str, object], rb_id: int) -> Optional[Dict[str, object]]:
    for container_key in ("mocap_raw", "mocap"):
        mocap = payload.get(container_key)
        if not isinstance(mocap, dict):
            continue
        state = mocap.get(str(rb_id), mocap.get(rb_id))
        if isinstance(state, dict):
            return state
    return None


def _available_mocap_ids(payload: Dict[str, object]) -> List[int]:
    ids = set()
    for container_key in ("mocap_raw", "mocap"):
        mocap = payload.get(container_key)
        if not isinstance(mocap, dict):
            continue
        for key in mocap.keys():
            try:
                ids.add(int(key))
            except (TypeError, ValueError):
                continue
    return sorted(ids)


def _select_mocap_rb_id(payload: Dict[str, object], requested_rb_id: Optional[int], trial_path: Path) -> int:
    ids = _available_mocap_ids(payload)
    if requested_rb_id is not None:
        if requested_rb_id not in ids:
            raise ValueError(
                f"{trial_path.name}: requested RB ID {requested_rb_id} not found; available IDs: {ids}"
            )
        return requested_rb_id
    if len(ids) == 1:
        return ids[0]
    raise ValueError(
        f"{trial_path.name}: multiple mocap RB IDs present ({ids}); "
        "provide --mocap-rb-id or --mocap-kind"
    )


def _finite_first_last(values: np.ndarray) -> Tuple[float, float]:
    finite_idx = np.flatnonzero(np.isfinite(values))
    if finite_idx.size == 0:
        return (math.nan, math.nan)
    return float(values[finite_idx[0]]), float(values[finite_idx[-1]])


def _trial_delta_xyz_cm(payload: Dict[str, object], rb_id: int) -> Dict[str, float]:
    mocap_state = _get_mocap_state(payload, rb_id)
    if mocap_state is None:
        raise ValueError(f"mocap state for RB ID {rb_id} not found")

    x = _extract_first_mocap_series(
        mocap_state,
        ("rotated_position_x", "position_x", "rotated_zeroed_position_x", "zeroed_position_x"),
    )
    y = _extract_first_mocap_series(
        mocap_state,
        ("rotated_position_y", "position_y", "rotated_zeroed_position_y", "zeroed_position_y"),
    )
    z = _extract_first_mocap_series(
        mocap_state,
        ("rotated_position_z", "position_z", "rotated_zeroed_position_z", "zeroed_position_z"),
    )
    if x is None or y is None or z is None:
        raise ValueError("missing mocap position arrays (expected rotated_position_* or fallback position_*)")

    x0, x1 = _finite_first_last(x)
    y0, y1 = _finite_first_last(y)
    z0, z1 = _finite_first_last(z)
    return {
        "x": (x1 - x0) * 100.0 if np.isfinite([x0, x1]).all() else math.nan,
        "y": (y1 - y0) * 100.0 if np.isfinite([y0, y1]).all() else math.nan,
        "z": (z1 - z0) * 100.0 if np.isfinite([z0, z1]).all() else math.nan,
    }


def _session_trial_deltas_cm(session_dir: Path, requested_rb_id: Optional[int]) -> Dict[int, Dict[str, float]]:
    trials = _list_trials(session_dir)
    if not trials:
        raise ValueError(f"no trial_*.npy files found in {session_dir}")

    out: Dict[int, Dict[str, float]] = {}
    selected_rb_id: Optional[int] = None
    for trial_path in trials:
        trial_num = _trial_number(trial_path)
        if trial_num is None:
            continue
        payload = _load_payload(trial_path)
        rb_id = _select_mocap_rb_id(payload, requested_rb_id=requested_rb_id, trial_path=trial_path)
        if selected_rb_id is None:
            selected_rb_id = rb_id
        elif selected_rb_id != rb_id:
            raise ValueError(
                f"{session_dir.name}: inconsistent selected RB IDs ({selected_rb_id} vs {rb_id})"
            )
        out[trial_num] = _trial_delta_xyz_cm(payload, rb_id)
    return out


def _nanmean_std(values: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    mean = np.nanmean(values, axis=0)
    std = np.nanstd(values, axis=0)
    mean[~np.isfinite(mean)] = np.nan
    std[~np.isfinite(std)] = np.nan
    return mean, std


def _build_arg_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(
        description=(
            "Plot per-trial net displacement (x, y', z') across multiple sessions "
            "with mean and std bands."
        )
    )
    ap.add_argument(
        "sessions",
        nargs="*",
        type=Path,
        help=(
            "Session directories (e.g., data/session_YYYYMMDD_HHMMSS). "
            "If omitted, script uses DEFAULT_SESSION_NAMES."
        ),
    )
    ap.add_argument(
        "--mocap-rb-id",
        type=int,
        default=None,
        help="Mocap rigid-body ID to use (recommended when multiple RBs are present).",
    )
    ap.add_argument(
        "--mocap-kind",
        choices=sorted(MOCAP_RB_IDS_BY_KIND.keys()),
        default=None,
        help="Shortcut for --mocap-rb-id using known mapping.",
    )
    ap.add_argument(
        "--title",
        type=str,
        default=None,
        help="Custom figure title.",
    )
    ap.add_argument(
        "--output",
        type=Path,
        default=None,
        help="Output PNG path (default: ./trial_displacement_band_across_sessions.png).",
    )
    return ap


def main() -> int:
    args = _build_arg_parser().parse_args()
    requested_rb_id = args.mocap_rb_id
    if args.mocap_kind is not None:
        kind_id = MOCAP_RB_IDS_BY_KIND[args.mocap_kind]
        if requested_rb_id is not None and requested_rb_id != kind_id:
            raise SystemExit(
                f"--mocap-rb-id ({requested_rb_id}) conflicts with --mocap-kind {args.mocap_kind} ({kind_id})"
            )
        requested_rb_id = kind_id

    if args.sessions:
        session_dirs = [Path(str(p).strip()) for p in args.sessions]
    else:
        session_dirs = [DATA_ROOT / name for name in DEFAULT_SESSION_NAMES]
        print("No session args provided; using DEFAULT_SESSION_NAMES from script.")

    if requested_rb_id is None:
        requested_rb_id = int(MOCAP_RB_IDS_BY_KIND[DEFAULT_MOCAP_KIND])
        print(f"No mocap selection provided; using DEFAULT_MOCAP_KIND='{DEFAULT_MOCAP_KIND}' (RB {requested_rb_id}).")

    for session_dir in session_dirs:
        if not session_dir.is_dir():
            raise SystemExit(f"Not a session directory: {session_dir}")

    per_session: List[Dict[int, Dict[str, float]]] = []
    for session_dir in session_dirs:
        trial_deltas = _session_trial_deltas_cm(session_dir, requested_rb_id=requested_rb_id)
        per_session.append(trial_deltas)
        print(f"{session_dir.name}: loaded {len(trial_deltas)} trial delta(s)")

    trial_numbers = sorted({t for sess in per_session for t in sess.keys()})
    if not trial_numbers:
        raise SystemExit("No trial deltas found across provided sessions.")

    n_sessions = len(per_session)
    n_trials = len(trial_numbers)
    x_mat = np.full((n_sessions, n_trials), np.nan, dtype=float)
    y_mat = np.full((n_sessions, n_trials), np.nan, dtype=float)
    z_mat = np.full((n_sessions, n_trials), np.nan, dtype=float)
    trial_idx_map = {trial_num: idx for idx, trial_num in enumerate(trial_numbers)}

    for sidx, sess in enumerate(per_session):
        for trial_num, deltas in sess.items():
            tidx = trial_idx_map[trial_num]
            x_mat[sidx, tidx] = float(deltas.get("x", math.nan))
            y_mat[sidx, tidx] = float(deltas.get("y", math.nan))
            z_mat[sidx, tidx] = float(deltas.get("z", math.nan))

    x_mean, x_std = _nanmean_std(x_mat)
    y_mean, y_std = _nanmean_std(y_mat)
    z_mean, z_std = _nanmean_std(z_mat)

    fig, axes = plt.subplots(3, 1, figsize=(10, 10), sharex=True)
    dims: Sequence[Tuple[str, np.ndarray, np.ndarray, str, str]] = (
        ("x", x_mean, x_std, "tab:purple", "x"),
        ("y'", y_mean, y_std, "tab:orange", "y'"),
        ("z'", z_mean, z_std, "tab:cyan", "z'"),
    )

    x_axis = np.asarray(trial_numbers, dtype=int)
    for ax, (dim_label, mean_vals, std_vals, color, legend_label) in zip(axes, dims):
        lower = mean_vals - std_vals
        upper = mean_vals + std_vals
        ax.fill_between(x_axis, lower, upper, color=color, alpha=0.25, linewidth=0.0, label=f"{legend_label} ±1 std")
        ax.plot(x_axis, mean_vals, color=color, linewidth=2.0, marker="o", label=f"{legend_label} mean")
        ax.axhline(0.0, color="black", linewidth=1.0, alpha=0.35)
        ax.set_ylabel(f"{dim_label} delta (cm)")
        ax.grid(True, alpha=0.3)
        ax.legend(loc="best")

    axes[-1].set_xlabel("trial number")
    title = args.title or f"Per-trial net displacement across {n_sessions} session(s)"
    fig.suptitle(title)
    fig.tight_layout()

    output_path = args.output or (Path.cwd() / "trial_displacement_band_across_sessions.png")
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=180)
    plt.close(fig)
    print(f"Wrote plot: {output_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
