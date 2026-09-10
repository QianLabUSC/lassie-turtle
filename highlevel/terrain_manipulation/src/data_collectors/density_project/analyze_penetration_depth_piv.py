#!/usr/bin/env python3
"""Compare PIV grain velocities across recorded penetration/height conditions.

Run without arguments for the three September 3, 2026 sessions. Requires NumPy
and Matplotlib; no ROS, camera hardware, or pandas is needed. See --help.

TODO(next experiments): Rectify original image pairs to the grain-surface plane,
recalibrate the rectified images, rerun PIVlab, and repeat this analysis.
TODO(next experiments): Resolve the inconsistent calibration between conditions;
use a verified spatial reference and frame interval for each acquisition setup.
The existing m/s exports are analyzed unchanged, without applying calibration twice.

One file must represent one trial at the chosen motor-angle target. Spatial
vectors are summarized within trials before equally weighting trial summaries.
The three default conditions have one session each; these descriptive summaries
cannot separate penetration effects from session, trial-order, or bed-history effects.
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
from datetime import datetime, timezone
import hashlib
import math
from pathlib import Path
import re
import shlex
import sys
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np


DATA_ROOT = Path(__file__).resolve().parents[3] / "data"
DEFAULT_SESSIONS = (
    (0.0, DATA_ROOT / "session_20260903_174737_height_0cm"),
    (2.0, DATA_ROOT / "session_20260903_174023_height_2cm"),
    (4.0, DATA_ROOT / "session_20260903_173420_height_4cm"),
)
DEFAULT_PIV_SUBDIR = Path("pivlab_angle_matched_pairs/PIVlab_out_larger_roi")
DEFAULT_OUTPUT_DIR = DATA_ROOT / "penetration_depth_piv_analysis"
COLUMNS = ("x [m]", "y [m]", "u [m/s]", "v [m/s]", "Vector type [-]")
STAT_NAMES = ("mean", "median", "std", "min", "max", "q25", "q75", "p95")
METRICS = tuple(f"{component}_{stat}_m_s" for component in ("u", "v", "speed")
                for stat in STAT_NAMES) + ("abs_u_mean_m_s", "abs_v_mean_m_s")
POLICIES = ((1,), (1, 3), (1, 2, 3))
VECTOR_SOURCE = "https://github.com/Shrediquette/PIVlab/blob/main/%2Bvalidate/filtervectors.m"
COLORS = ("#2469A0", "#C46B24", "#777C35", "#B75F87", "#555555")
MARKERS = ("o", "s", "^", "D", "v")
LINESTYLES = ("-", "--", "-.", ":", "-")


@dataclass
class Trial:
    condition_cm: float
    path: Path
    trial_number: int
    image_a: str
    image_b: str
    camera: int
    frame_a: int
    frame_b: int
    xy_factor: float
    uv_factor: float
    data: np.ndarray
    metadata: Dict[str, str]
    warnings: List[str]
    session_dir: Optional[Path] = None


def basename(value: str) -> str:
    return value.replace("\\", "/").rsplit("/", 1)[-1]


def read_piv(path: Path, condition_cm: float) -> Trial:
    """Read calibrated ASCII data and verify A/B trial identity, units, and flags."""
    lines = path.read_text(encoding="utf-8-sig").splitlines()
    column_index = next((i for i, line in enumerate(lines)
                         if line.strip().startswith("x [")), None)
    if column_index is None:
        raise ValueError(f"{path}: missing PIVlab column header")
    columns = [value.strip() for value in next(csv.reader([lines[column_index]]))]
    if len(columns) != len(COLUMNS) or set(columns) != set(COLUMNS):
        raise ValueError(f"{path}: expected calibrated columns {COLUMNS}; got {columns}")
    header = "\n".join(lines[:column_index])
    pair = re.search(r"filenames:\s*A:\s*(.*?)\s*&\s*B:\s*(.*?),\s*conversion factor", header)
    if not pair:
        raise ValueError(f"{path}: cannot read A/B source filenames")
    image_a, image_b = (basename(value.strip()) for value in pair.groups())
    image_pattern = r"trial_(\d+)_cam(\d+)_frame_(\d+)_([AB])_gray\.png"
    a, b = re.fullmatch(image_pattern, image_a), re.fullmatch(image_pattern, image_b)
    if (not a or not b or a.group(4) != "A" or b.group(4) != "B"
            or a.groups()[:2] != b.groups()[:2] or int(b.group(3)) <= int(a.group(3))):
        raise ValueError(f"{path}: A/B images must be increasing frames from the same trial/camera")
    factors = []
    for name in ("xy", "uv"):
        match = re.search(rf"conversion factor {name}\s*\([^)]*\):\s*([^,\s]+)", header)
        factor = float(match.group(1)) if match else math.nan
        if not math.isfinite(factor) or factor <= 0:
            raise ValueError(f"{path}: missing or invalid positive {name} calibration factor")
        factors.append(factor)
    try:
        data = np.loadtxt(lines[column_index + 1:], delimiter=",", ndmin=2)
    except ValueError as exc:
        raise ValueError(f"{path}: malformed numeric data: {exc}") from exc
    if data.size == 0 or data.shape[1] != len(COLUMNS):
        raise ValueError(f"{path}: expected nonempty five-column numeric data")
    data = data[:, [columns.index(name) for name in COLUMNS]]
    if not np.isfinite(data[:, :2]).all():
        raise ValueError(f"{path}: nonfinite grid coordinates")
    if np.unique(data[:, :2], axis=0).shape[0] != len(data):
        raise ValueError(f"{path}: duplicate vector grid coordinates")
    if not np.isin(data[:, 4], (0, 1, 2, 3)).all():
        raise ValueError(f"{path}: unsupported vector types {np.unique(data[:, 4]).tolist()}")
    return Trial(condition_cm, path, int(a.group(1)), image_a, image_b,
                 int(a.group(2)), int(a.group(3)), int(b.group(3)),
                 factors[0], factors[1], data, {}, [])


def attach_metadata(trials: Sequence[Trial], summary_path: Path) -> None:
    if not summary_path.is_file():
        for trial in trials:
            trial.warnings.append("No matched_frame_summary.csv; angle/timestamp checks unavailable")
        return
    with summary_path.open(newline="", encoding="utf-8-sig") as stream:
        rows = list(csv.DictReader(stream))
    by_pair = {}
    for row in rows:
        key = (basename(row.get("gray_image_a", "")), basename(row.get("gray_image_b", "")))
        if not all(key) or key in by_pair:
            raise ValueError(f"{summary_path}: missing or duplicate A/B image pair")
        by_pair[key] = row
    for trial in trials:
        row = by_pair.get((trial.image_a, trial.image_b))
        if row is None:
            if any(r.get("trial") == f"trial_{trial.trial_number}" for r in rows):
                raise ValueError(f"{trial.path}: metadata lists different images for this trial")
            trial.warnings.append("No metadata for this image pair; angle/timestamp checks unavailable")
            continue
        expected = {"trial": f"trial_{trial.trial_number}", "camera_index": str(trial.camera),
                    "frame_a_index": str(trial.frame_a), "frame_b_index": str(trial.frame_b)}
        if any(row.get(key) != value for key, value in expected.items()):
            raise ValueError(f"{trial.path}: matched-frame metadata disagrees with the A/B header")
        trial.metadata = row


def selected_vectors(trial: Trial, types: Tuple[int, ...],
                     roi_px: Optional[Sequence[float]]) -> Tuple[np.ndarray, np.ndarray]:
    finite = np.isfinite(trial.data[:, 2:4]).all(axis=1)
    in_roi = np.ones(len(trial.data), dtype=bool)
    if roi_px is not None:
        x, y = (trial.data[:, :2] / trial.xy_factor).T
        xmin, xmax, ymin, ymax = roi_px
        # Header calibration is rounded; tolerate up to 0.05 px at ROI boundaries.
        in_roi = ((x >= xmin - 0.05) & (x <= xmax + 0.05)
                  & (y >= ymin - 0.05) & (y <= ymax + 0.05))
    keep = finite & in_roi & np.isin(trial.data[:, 4], types)
    return trial.data[keep], in_roi


def velocity_statistics(uv: np.ndarray) -> Dict[str, float]:
    """Spatial descriptive statistics; std uses ddof=0, preserving signed u/v."""
    if len(uv) == 0:
        return {key: math.nan for key in METRICS}
    u, v = uv.T
    result = {}
    for name, values in (("u", u), ("v", v), ("speed", np.hypot(u, v))):
        q25, median, q75, p95 = np.percentile(values, [25, 50, 75, 95])
        stats = (np.mean(values), median, np.std(values), np.min(values),
                 np.max(values), q25, q75, p95)
        result.update({f"{name}_{stat}_m_s": float(value)
                       for stat, value in zip(STAT_NAMES, stats)})
    result["abs_u_mean_m_s"] = float(np.mean(np.abs(u)))
    result["abs_v_mean_m_s"] = float(np.mean(np.abs(v)))
    return result


def summarize_trial(trial: Trial, types: Tuple[int, ...],
                    roi_px: Optional[Sequence[float]]) -> dict:
    data, in_roi = selected_vectors(trial, types, roi_px)
    row = {"condition_cm": trial.condition_cm,
           "session": str(trial.session_dir) if trial.session_dir is not None else "",
           "trial": trial.trial_number, "source_file": str(trial.path),
           "vector_types": "+".join(map(str, types)), "n_vectors_total": len(trial.data),
           "n_vectors_in_roi": int(in_roi.sum()), "n_vectors_used": len(data)}
    row.update(velocity_statistics(data[:, 2:4]))
    return row


def summarize_conditions(rows: Sequence[dict]) -> List[dict]:
    """Each available trial has equal weight, regardless of retained vector count."""
    result = []
    groups = sorted({(row["condition_cm"], row["vector_types"]) for row in rows})
    for condition, types in groups:
        group = [row for row in rows if (row["condition_cm"], row["vector_types"]) == (condition, types)]
        for metric in METRICS:
            values = np.array([row[metric] for row in group], dtype=float)
            values = values[np.isfinite(values)]
            result.append({
                "condition_cm": condition, "vector_types": types, "metric": metric,
                "units": "m/s", "n_trials_total": len(group), "n_trials_with_data": len(values),
                "mean": float(np.mean(values)) if len(values) else math.nan,
                "between_trial_sd": float(np.std(values, ddof=1)) if len(values) > 1 else math.nan,
                "median": float(np.median(values)) if len(values) else math.nan,
                "min": float(np.min(values)) if len(values) else math.nan,
                "max": float(np.max(values)) if len(values) else math.nan,
            })
    return result


def metadata_number(trial: Trial, key: str) -> float:
    if not trial.metadata:
        return math.nan
    try:
        value = float(trial.metadata[key])
    except (KeyError, ValueError) as exc:
        raise ValueError(f"{trial.path}: missing or invalid metadata field {key}") from exc
    if not math.isfinite(value):
        raise ValueError(f"{trial.path}: nonfinite metadata field {key}")
    return value


def quality_row(trial: Trial, types: Tuple[int, ...], roi_px: Optional[Sequence[float]],
                angle_warning_deg: float, timing_warning_percent: float) -> dict:
    used, in_roi = selected_vectors(trial, types, roi_px)
    row = {"condition_cm": trial.condition_cm, "trial": trial.trial_number,
           "source_file": str(trial.path), "source_sha256": hashlib.sha256(trial.path.read_bytes()).hexdigest(),
           "image_a": trial.image_a, "image_b": trial.image_b, "camera_index": trial.camera,
           "frame_a_index": trial.frame_a, "frame_b_index": trial.frame_b,
           "xy_m_per_px": trial.xy_factor, "uv_m_s_per_px_frame": trial.uv_factor,
           "implied_pair_interval_s": trial.xy_factor / trial.uv_factor,
           "n_vectors_total": len(trial.data), "n_vectors_in_roi": int(in_roi.sum()),
           "n_vectors_used": len(used),
           "used_fraction_in_roi": len(used) / int(in_roi.sum()) if in_roi.any() else math.nan,
           "n_nonfinite_uv": int((~np.isfinite(trial.data[:, 2:4]).all(axis=1)).sum())}
    for flag in (0, 1, 2, 3):
        row[f"n_type_{flag}"] = int((trial.data[:, 4] == flag).sum())
        row[f"n_type_{flag}_in_roi"] = int(((trial.data[:, 4] == flag) & in_roi).sum())
    for axis, index in (("x", 0), ("y", 1)):
        row[f"{axis}_min_m"] = float(trial.data[:, index].min())
        row[f"{axis}_max_m"] = float(trial.data[:, index].max())
        row[f"{axis}_min_px_approx"] = row[f"{axis}_min_m"] / trial.xy_factor
        row[f"{axis}_max_px_approx"] = row[f"{axis}_max_m"] / trial.xy_factor
    for component in ("adduction", "sweeping"):
        target = metadata_number(trial, f"target_{component}_rad")
        actual = metadata_number(trial, f"frame_a_{component}_rad")
        row[f"target_{component}_deg"] = math.degrees(target)
        row[f"frame_a_{component}_deg"] = math.degrees(actual)
        row[f"{component}_error_deg"] = math.degrees(abs(actual - target))
    row["max_angle_error_deg"] = max(row["adduction_error_deg"], row["sweeping_error_deg"])
    dt = metadata_number(trial, "frame_b_time_s") - metadata_number(trial, "frame_a_time_s")
    if math.isfinite(dt) and dt <= 0:
        raise ValueError(f"{trial.path}: nonpositive frame interval in metadata")
    row["recorded_pair_interval_s"] = dt
    row["timing_difference_percent"] = 100 * (dt / row["implied_pair_interval_s"] - 1)
    if row["max_angle_error_deg"] > angle_warning_deg:
        trial.warnings.append(f"Angle error {row['max_angle_error_deg']:.2f} deg exceeds {angle_warning_deg:g} deg")
    if abs(row["timing_difference_percent"]) > timing_warning_percent:
        trial.warnings.append(f"Recorded pair interval differs from calibration-implied interval by {row['timing_difference_percent']:+.1f}%")
    if not len(used):
        trial.warnings.append("No usable vectors under the primary vector/ROI filter")
    if row["n_nonfinite_uv"]:
        trial.warnings.append(f"Excluded {row['n_nonfinite_uv']} nonfinite u/v rows before vector-type/ROI selection")
    row["warnings"] = "; ".join(trial.warnings)
    return row


def comparison_checks(trials: Sequence[Trial], quality: Sequence[dict]) -> List[str]:
    warnings = []
    factors = sorted({(trial.xy_factor, trial.uv_factor) for trial in trials})
    if len(factors) > 1:
        warnings.append("Calibration factors differ across exports. Existing m/s values were retained; resolve calibration in the next experiment round.")
    if len({trial.camera for trial in trials}) > 1:
        warnings.append("Multiple camera indices are present; image-axis velocities may not be comparable.")
    # Integer rounding is only for a grid-comparability diagnostic. Never changes velocities.
    grids = set()
    for trial in trials:
        pixels = np.rint(trial.data[:, :2] / trial.xy_factor).astype(np.int64)
        grids.add(tuple(sorted(map(tuple, pixels.tolist()))))
    if len(grids) > 1:
        warnings.append("Exported pixel grids differ. Review ROIs/alignment before interpreting condition differences.")
    targets = {(round(row["target_adduction_deg"], 5), round(row["target_sweeping_deg"], 5))
               for row in quality if math.isfinite(row["target_adduction_deg"])}
    if len(targets) > 1:
        warnings.append("Motor-angle targets differ across files; these are not all observations of the same target pose.")
    warnings.append("Equal pixel grids do not establish a common physical ROI. Review grain/flipper/background masks; the script does not segment images.")
    warnings.append("One session per condition: depth is confounded with session and acquisition order; trial repeats may share bed history. No significance tests are performed.")
    return warnings


def write_csv(path: Path, rows: Sequence[dict]) -> None:
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        for row in rows:
            writer.writerow({key: "" if isinstance(value, (float, np.floating))
                             and not math.isfinite(value) else value for key, value in row.items()})


def make_plots(trials: Sequence[Trial], rows: Sequence[dict], types: Tuple[int, ...],
               roi_px: Optional[Sequence[float]], output: Path) -> None:
    # Chart contract: static PNG/PDF; 21 trial summaries, 7 ordered trials/condition.
    # Comparison = faceted dots + mean/SD; distributions = per-trial boxplots;
    # progression = observed trial-order markers joined within each session only.
    # Three categories use blue/orange/olive with distinct markers/line styles.
    # No inferred trend fit, pooled-vector inferential intervals, or dropped outliers.
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 10,
                         "axes.spines.top": False, "axes.spines.right": False,
                         "axes.labelcolor": "#272727", "text.color": "#272727",
                         "axes.edgecolor": "#777777", "savefig.facecolor": "white",
                         "pdf.fonttype": 42})
    conditions = sorted({trial.condition_cm for trial in trials})
    policy = "+".join(map(str, types))
    roi_label = "exported ROI" if roi_px is None else "selected pixel ROI"
    subtitle = f"Vector types {policy}; {roi_label}; calibrated camera-view velocity (m/s)"
    footnote = "Unrectified images; calibration differs in the original sessions. Condition labels are recorded heights."

    def save(fig, name: str, note: str) -> None:
        fig.text(0.5, 0.035, note, ha="center", fontsize=9)
        fig.text(0.5, 0.009, footnote, ha="center", fontsize=8, color="#555555")
        for extension in ("png", "pdf"):
            fig.savefig(output / f"{name}.{extension}", dpi=200, bbox_inches="tight")
        plt.close(fig)

    fig, axes = plt.subplots(1, 3, figsize=(12, 4.9), sharey=True)
    fig.suptitle("PIV motion strength by condition\n" + subtitle, fontsize=13)
    for ax, metric, title in zip(axes, ("speed_mean_m_s", "abs_u_mean_m_s", "abs_v_mean_m_s"),
                                 ("Mean speed", "Mean |u|", "Mean |v|")):
        labels = []
        for i, condition in enumerate(conditions):
            group = [row for row in rows if row["condition_cm"] == condition]
            available = [row for row in group if math.isfinite(row[metric])]
            values = np.array([row[metric] for row in available])
            offsets = np.linspace(-0.16, 0.16, len(values)) if len(values) > 1 else [0]
            ax.scatter(i + np.array(offsets), values, color=COLORS[i % len(COLORS)],
                       marker=MARKERS[i % len(MARKERS)], s=36, alpha=0.85, zorder=3)
            mean = float(np.mean(values))
            sd = float(np.std(values, ddof=1)) if len(values) > 1 else 0
            ax.errorbar(i + 0.27, mean, yerr=sd, fmt="_", color="#272727",
                        markersize=12, capsize=4, linewidth=1.5, zorder=4)
            labels.append(f"{condition:g} cm\nn={len(values)}/{len(group)}")
        ax.set(title=title, xlabel="Recorded condition (cm)", xticks=range(len(conditions)),
               xticklabels=labels, xlim=(-0.5, len(conditions) - 0.4))
        ax.grid(axis="y", color="#E7E7E7", linewidth=0.7)
    axes[0].set_ylabel("Velocity (m/s)")
    # Include zero while retaining any below-zero mean-SD interval in unusual data.
    if axes[0].get_ylim()[0] > 0:
        axes[0].set_ylim(bottom=0)
    fig.tight_layout(rect=(0, 0.13, 1, 0.87))
    save(fig, "condition_comparison", "Dots: one spatial mean per trial. Black marks: equal-weight trial mean ± between-trial SD (not a confidence interval).")

    fig, axes = plt.subplots(3, len(conditions), figsize=(max(8, 4 * len(conditions)), 9),
                             sharey="row", squeeze=False)
    fig.suptitle("Within-trial velocity distributions\n" + subtitle, fontsize=13)
    for col, condition in enumerate(conditions):
        group = [trial for trial in trials if trial.condition_cm == condition]
        fields = [selected_vectors(trial, types, roi_px)[0][:, 2:4] for trial in group]
        counts = [len(field) for field in fields]
        for row_index, name in enumerate(("u", "v", "speed")):
            ax = axes[row_index, col]
            values = [(field[:, row_index] if row_index < 2 else np.hypot(field[:, 0], field[:, 1]))
                      if len(field) else np.array([math.nan]) for field in fields]
            artists = ax.boxplot(values, positions=range(1, len(group) + 1), widths=0.55,
                                 patch_artist=True, whis=1.5, showfliers=True,
                                 medianprops={"color": "#272727", "linewidth": 1.4},
                                 flierprops={"marker": ".", "markersize": 2, "alpha": 0.4,
                                             "markeredgecolor": "#555555"})
            for patch in artists["boxes"]:
                patch.set(facecolor=COLORS[col % len(COLORS)], alpha=0.45, edgecolor="#272727")
            ax.set_xticks(range(1, len(group) + 1))
            ax.set_xticklabels([str(trial.trial_number) for trial in group])
            ax.axhline(0, color="#777777", linewidth=0.7, zorder=0)
            ax.grid(axis="y", color="#E7E7E7", linewidth=0.7)
            if row_index == 0:
                ax.set_title(f"{condition:g} cm | {len(group)} trials\n{min(counts)}–{max(counts)} vectors/trial")
            if col == 0:
                ax.set_ylabel(f"{name} (m/s)")
            if row_index == 2:
                ax.set_xlabel("Trial number")
    axes[2, 0].set_ylim(bottom=0)
    fig.tight_layout(rect=(0, 0.085, 1, 0.92))
    save(fig, "trial_distributions", "Boxes: spatial Q25–Q75 and median. Whiskers: 1.5×IQR; all points beyond whiskers shown. Spatial vectors are not independent repeats.")

    fig, ax = plt.subplots(figsize=(10, 5.5))
    fig.suptitle("Mean grain speed by trial order\n" + subtitle, fontsize=13)
    for i, condition in enumerate(conditions):
        group = [row for row in rows if row["condition_cm"] == condition]
        ax.plot([row["trial"] for row in group], [row["speed_mean_m_s"] for row in group],
                marker=MARKERS[i % len(MARKERS)], linestyle=LINESTYLES[i % len(LINESTYLES)],
                color=COLORS[i % len(COLORS)], label=f"{condition:g} cm", linewidth=1.5)
    ax.set(xlabel="Trial number within session", ylabel="Mean speed (m/s)", ylim=(0, None),
           xticks=sorted({row["trial"] for row in rows}))
    ax.grid(axis="y", color="#E7E7E7", linewidth=0.7)
    ax.legend(title="Recorded condition", frameon=False)
    fig.tight_layout(rect=(0, 0.13, 1, 0.90))
    save(fig, "trial_order", "One point per trial; lines connect observed repeats within each session. Matching trial numbers across conditions do not establish pairing.")


def write_notes(output: Path, trials: Sequence[Trial],
                quality: Sequence[dict], warnings: Sequence[str], args: argparse.Namespace,
                command: str) -> None:
    lines = [
        "# Penetration-condition PIV analysis", "",
        f"Generated: {datetime.now(timezone.utc).isoformat()}", "",
        "## TODO for the next experiments", "",
        "- [ ] Rectify original A/B images to the grain-surface plane, recalibrate the rectified images, rerun PIVlab, and repeat this analysis.",
        "- [ ] Resolve inconsistent calibration across conditions. Verify the spatial reference and frame interval for each acquisition setup; use consistent calibration procedures and record the factors.",
        "", "## Method and interpretation", "",
        "The exported u/v values are used unchanged in m/s. Their calibration factors are recorded, never applied a second time. Speed is sqrt(u²+v²) at each vector, then summarized spatially within each trial.", "",
        "Trial CSVs contain signed u/v and speed mean, median, population spatial SD (ddof=0), min/max, Q25/Q75, and P95; also mean |u| and mean |v|. Percentiles use NumPy's default linear interpolation. Min/max are descriptive, with no automatic outlier trimming.", "",
        "Condition CSVs summarize each trial metric with equal trial weight: mean, between-trial sample SD (ddof=1), median, min/max, and trial counts. These are not pooled-vector summaries. For example, the condition mean of speed_p95_m_s is the mean of trial P95s, not a pooled P95. Missing/nonfinite results are blank in CSVs; missing trials are counted and excluded from numerical aggregates.", "",
        f"Primary vector types: {'+'.join(map(str, args.vector_types))}. Type 0 (masked) and nonfinite u/v are always excluded. Type 1 is the original accepted field; type 2 marks rejected vectors that may have been filled by interpolation; type 3 denotes accepted second-peak substitutions in current PIVlab. Sensitivity tables compare 1, 1+3, and 1+2+3. Confirm these semantics against the version used for export. Including replacements changes both values and spatial coverage.",
        f"Vector-type source: {VECTOR_SOURCE}", "",
        f"Pixel ROI restriction: {args.roi_px if args.roi_px is not None else 'none; use full exported ROI'}. Optional --roi-px bounds are inclusive and use x/y divided by the rounded header xy factor (0.05 px boundary tolerance). This is an image-grid selection, not a physical top-down region. No automatic flipper/background segmentation is performed.", "",
        "Coordinates and components remain in the unrectified camera view. Condition values preserve the supplied height/penetration labels; they do not imply that 0 cm is a no-motion control or establish an absolute penetration measurement.", "",
        "Angles/timestamps are joined by both A/B source filenames, then trial/camera/frame identities are verified. Errors are recomputed from actual and target angles. Timestamp differences are diagnostics: collector timestamps may differ from camera exposure times. No timing correction is applied automatically.", "",
        f"Warning thresholds: angle error > {args.angle_warning_deg:g} deg; absolute recorded-versus-implied pair-interval difference > {args.timing_warning_percent:g}%. They flag observations without discarding them.", "",
        "The three figures show trial means with between-trial SD, full within-trial boxplots, and observed trial order. The seven points per default condition are the available repeats; connecting them is descriptive, without a fitted trend. There are no p-values or confidence intervals. Spatial PIV locations are correlated and are not independent experimental repeats.", "",
        "## Calibration and metadata checks", "",
        "| Condition (cm) | Trials | xy (m/px) | uv ((m/s)/(px/frame)) | Max-angle error range (deg) | Recorded pair interval range (ms) |",
        "|---|---:|---|---|---|---|",
    ]
    for condition in sorted({trial.condition_cm for trial in trials}):
        group = [row for row in quality if row["condition_cm"] == condition]
        def extent(key: str, scale: float = 1) -> str:
            values = [row[key] * scale for row in group if math.isfinite(row[key])]
            return f"{min(values):.3f}–{max(values):.3f}" if values else "unavailable"
        xy = ", ".join(f"{value:.8g}" for value in sorted({row["xy_m_per_px"] for row in group}))
        uv = ", ".join(f"{value:.8g}" for value in sorted({row["uv_m_s_per_px_frame"] for row in group}))
        lines.append(f"| {condition:g} | {len(group)} | {xy} | {uv} | {extent('max_angle_error_deg')} | {extent('recorded_pair_interval_s', 1000)} |")
    lines.extend(["", "## Review flags", ""])
    lines.extend(f"- {warning}" for warning in warnings)
    lines.extend(f"- {row['condition_cm']:g} cm, trial {row['trial']}: {row['warnings']}"
                 for row in quality if row["warnings"])
    lines.extend(["", "## Outputs", "",
                  "- trial_summary.csv: primary vector filter, one row per trial; all velocity statistics in m/s.",
                  "- condition_summary.csv: primary filter, one row per condition and trial metric.",
                  "- vector_sensitivity_trials.csv / vector_sensitivity_conditions.csv: same summaries for alternative vector inclusion policies.",
                  "- quality_checks.csv: original file hashes, source image names, calibration, angles, timestamps, grid bounds, and vector counts. Type/nonfinite counts cover the full export; *_in_roi counts cover the optional ROI.",
                  "- condition_comparison.png/.pdf, trial_distributions.png/.pdf, trial_order.png/.pdf.",
                  "", "## Reproduce", "", "```bash", command, "```", "",
                  "Requires Python 3, NumPy, and Matplotlib. --session CONDITION_CM SESSION_DIR may be repeated to replace the default sessions. --piv-subdir selects a different PIV export folder relative to each session. One session per condition and one PIV pair per trial are required; duplicated trial exports are rejected.",
                  "", "Sources:", ""])
    lines.extend(f"- {path}" for path in sorted({str(trial.path.parent) for trial in trials}))
    lines.append("")
    (output / "README.md").write_text("\n".join(lines), encoding="utf-8")


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--session", nargs=2, action="append", metavar=("CONDITION_CM", "SESSION_DIR"),
                        help="Repeat for each condition; replaces the three built-in sessions.")
    parser.add_argument("--piv-subdir", type=Path, default=DEFAULT_PIV_SUBDIR,
                        help="PIV export folder relative to each session.")
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR,
                        help=f"Destination for CSVs, figures, and notes (default: {DEFAULT_OUTPUT_DIR}).")
    parser.add_argument("--vector-types", type=int, nargs="+", choices=(1, 2, 3), default=[1],
                        help="Primary vector inclusion policy (default: 1); sensitivity CSVs are always produced.")
    parser.add_argument("--roi-px", nargs=4, type=float, metavar=("XMIN", "XMAX", "YMIN", "YMAX"),
                        help="Optional common rectangle in original image-grid pixels, with inclusive bounds.")
    parser.add_argument("--angle-warning-deg", type=float, default=2.0)
    parser.add_argument("--timing-warning-percent", type=float, default=10.0)
    args = parser.parse_args(argv)
    args.vector_types = tuple(sorted(set(args.vector_types)))
    try:
        args.sessions = sorted((float(label), Path(path).expanduser().resolve())
                               for label, path in args.session) if args.session else list(DEFAULT_SESSIONS)
    except ValueError:
        parser.error("Each --session condition must be a number in cm")
    if any(not math.isfinite(label) for label, _ in args.sessions):
        parser.error("Conditions must be finite numbers")
    if len({label for label, _ in args.sessions}) != len(args.sessions):
        parser.error("Provide one session per unique condition")
    if len({path for _, path in args.sessions}) != len(args.sessions):
        parser.error("The same session cannot represent multiple conditions")
    if args.piv_subdir.is_absolute() or ".." in args.piv_subdir.parts:
        parser.error("--piv-subdir must be a path within each session")
    if args.roi_px is not None:
        xmin, xmax, ymin, ymax = args.roi_px
        if not all(map(math.isfinite, args.roi_px)) or xmin >= xmax or ymin >= ymax:
            parser.error("--roi-px requires finite bounds with XMIN < XMAX and YMIN < YMAX")
    for key in ("angle_warning_deg", "timing_warning_percent"):
        if not math.isfinite(getattr(args, key)) or getattr(args, key) < 0:
            parser.error(f"--{key.replace('_', '-')} must be finite and nonnegative")
    return args


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    trials = []
    for condition, session in args.sessions:
        paths = sorted((session / args.piv_subdir).glob("PIVlab_*.txt"))
        if not paths:
            raise ValueError(f"No PIVlab_*.txt files found in {session / args.piv_subdir}")
        group = [read_piv(path, condition) for path in paths]
        for trial in group:
            trial.session_dir = session
        if len({trial.trial_number for trial in group}) != len(group):
            raise ValueError(f"{session}: multiple exports per trial; select one angle/pair per trial")
        attach_metadata(group, session / args.piv_subdir.parent / "matched_frame_summary.csv")
        trials.extend(sorted(group, key=lambda trial: trial.trial_number))
    primary = [summarize_trial(trial, args.vector_types, args.roi_px) for trial in trials]
    for condition, _ in args.sessions:
        if not any(row["n_vectors_used"] for row in primary if row["condition_cm"] == condition):
            raise ValueError(f"{condition:g} cm: no usable vectors after primary vector/ROI filtering")
    quality = [quality_row(trial, args.vector_types, args.roi_px, args.angle_warning_deg,
                           args.timing_warning_percent) for trial in trials]
    warnings = comparison_checks(trials, quality)
    policies = list(dict.fromkeys((args.vector_types,) + POLICIES))
    sensitivity = [summarize_trial(trial, policy, args.roi_px) for policy in policies for trial in trials]
    output = args.output_dir.expanduser().resolve()
    output.mkdir(parents=True, exist_ok=True)
    write_csv(output / "trial_summary.csv", primary)
    write_csv(output / "condition_summary.csv", summarize_conditions(primary))
    write_csv(output / "vector_sensitivity_trials.csv", sensitivity)
    write_csv(output / "vector_sensitivity_conditions.csv", summarize_conditions(sensitivity))
    write_csv(output / "quality_checks.csv", quality)
    make_plots(trials, primary, args.vector_types, args.roi_px, output)
    command = shlex.join([sys.executable, str(Path(__file__).resolve()), *(sys.argv[1:] if argv is None else argv)])
    write_notes(output, trials, quality, warnings, args, command)
    print(f"Analyzed {len(trials)} trials across {len(args.sessions)} conditions; vector types {args.vector_types}.")
    for row in summarize_conditions(primary):
        if row["metric"] == "speed_mean_m_s":
            print(f"{row['condition_cm']:g} cm: mean speed {row['mean']:.6f} m/s; "
                  f"between-trial SD {row['between_trial_sd']:.6f}; "
                  f"{row['n_trials_with_data']}/{row['n_trials_total']} trials")
    for warning in warnings:
        print(f"REVIEW: {warning}")
    print(f"Trial-specific review flags: {sum(bool(row['warnings']) for row in quality)}; see quality_checks.csv.")
    print(f"Saved CSVs, PNG/PDF figures, and rectification/calibration TODOs to {output}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (ValueError, OSError) as exc:
        raise SystemExit(f"Error: {exc}") from exc
