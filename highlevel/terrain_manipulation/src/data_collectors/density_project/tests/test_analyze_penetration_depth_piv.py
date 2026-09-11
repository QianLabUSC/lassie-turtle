"""Numerical and provenance checks for the standalone PIV comparison script."""

import csv
import importlib.util
from pathlib import Path
import sys

import numpy as np
import pytest


SCRIPT = Path(__file__).resolve().parents[1] / "analyze_penetration_depth_piv.py"
SPEC = importlib.util.spec_from_file_location("penetration_piv", SCRIPT)
piv = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = piv
SPEC.loader.exec_module(piv)


def export(tmp_path, rows=None, image_b="trial_1_cam0_frame_0054_B_gray.png"):
    path = tmp_path / "PIVlab_9999.txt"
    rows = rows if rows is not None else [
        "0,0,1000,0,0",  # Finite masked vector must never enter the statistics.
        "1,0,3,4,1",
        "2,0,30,40,2",  # Filled rejection would strongly change the mean.
        "3,0,0,6,3",
        "4,0,nan,1,1",  # Accepted flag alone is insufficient.
    ]
    path.write_text("\ufeffPIVlab, ASCII chart output\r\n"
                    "FRAME: 9999, filenames: A: trial_1_cam0_frame_0053_A_gray.png & B: "
                    + image_b + ", conversion factor xy (px -> m): 0.5, "
                    "conversion factor uv (px/frame -> m/s): 15\r\n"
                    "x [m],y [m],u [m/s],v [m/s],Vector type [-]\r\n"
                    + "\r\n".join(rows) + "\r\n", encoding="utf-8")
    return path


def test_opposing_motion_keeps_speed_and_absolute_components():
    stats = piv.velocity_statistics(np.array([[3., 4.], [-3., -4.]]))
    assert stats["u_mean_m_s"] == stats["v_mean_m_s"] == 0
    assert stats["speed_mean_m_s"] == stats["speed_median_m_s"] == 5
    assert stats["abs_u_mean_m_s"] == stats["u_std_m_s"] == 3
    assert stats["abs_v_mean_m_s"] == 4
    assert stats["u_min_m_s"] == -3
    assert stats["speed_std_m_s"] == 0


def test_extreme_speed_is_retained_but_p95_is_separate():
    stats = piv.velocity_statistics(np.array([[0., 0.]] * 19 + [[100., 0.]]))
    assert stats["speed_max_m_s"] == 100
    assert stats["speed_mean_m_s"] == 5
    assert stats["speed_median_m_s"] == 0
    assert stats["speed_p95_m_s"] == pytest.approx(5)


def test_component_p95_uses_magnitudes_before_percentiles():
    stats = piv.velocity_statistics(np.array([[-100., 0.], [-1., 1.], [-1., 2.]]))
    # The fast negative component must remain in the upper magnitude tail.
    assert stats["u_p95_m_s"] == -1
    assert stats["abs_u_p95_m_s"] == pytest.approx(90.1)
    assert stats["abs_v_p95_m_s"] == pytest.approx(1.9)


def test_filter_sensitivity_and_no_double_calibration(tmp_path):
    trial = piv.read_piv(export(tmp_path), 2)
    assert trial.trial_number == 1  # Comes from source image, not export sequence.
    strict = piv.summarize_trial(trial, (1,), None)
    measured = piv.summarize_trial(trial, (1, 3), None)
    filled = piv.summarize_trial(trial, (1, 2, 3), None)
    assert strict["n_vectors_used"] == 1
    assert strict["speed_mean_m_s"] == 5  # Not multiplied by uv factor 15.
    assert measured["speed_mean_m_s"] == 5.5
    assert filled["n_vectors_used"] == 3
    assert filled["speed_mean_m_s"] == pytest.approx(61 / 3)
    assert trial.data[0, 2] == 1000  # Raw input remains intact.


def test_roi_uses_pixel_grid_and_empty_trial_is_missing(tmp_path):
    trial = piv.read_piv(export(tmp_path), 2)
    selected = piv.summarize_trial(trial, (1, 2, 3), [4, 6, -1, 1])
    assert selected["n_vectors_used"] == 2
    assert selected["speed_mean_m_s"] == 28
    empty = piv.summarize_trial(trial, (1,), [4, 6, -1, 1])
    assert empty["n_vectors_used"] == 0
    assert np.isnan(empty["speed_mean_m_s"])


def test_conditions_weight_trials_equally_and_report_missingness():
    rows = []
    for trial_number, velocities in enumerate(([[1., 0.]] * 100, [[9., 0.]], []), 1):
        rows.append({"condition_cm": 0., "vector_types": "1", "trial": trial_number,
                     **piv.velocity_statistics(np.array(velocities))})
    summary = next(row for row in piv.summarize_conditions(rows) if row["metric"] == "speed_mean_m_s")
    assert summary["mean"] == 5  # Pooling vectors would give ~1.079 instead.
    assert summary["between_trial_sd"] == pytest.approx(np.sqrt(32))
    assert summary["n_trials_total"] == 3
    assert summary["n_trials_with_data"] == 2


@pytest.mark.parametrize("change, message", [
    (lambda text: text.replace("u [m/s]", "u [px/frame]"), "calibrated columns"),
    (lambda text: text.replace("B: trial_1", "B: trial_2"), "same trial/camera"),
    (lambda text: text.replace("3,0,0,6,3", "3,0,0,6,9"), "unsupported vector types"),
    (lambda text: text.replace("3,0,0,6,3", "2,0,0,6,3"), "duplicate vector grid"),
])
def test_invalid_inputs_fail_with_context(tmp_path, change, message):
    path = export(tmp_path)
    path.write_text(change(path.read_text()))
    with pytest.raises(ValueError, match=message):
        piv.read_piv(path, 0)


def test_metadata_cannot_join_different_pair_from_same_trial(tmp_path):
    trial = piv.read_piv(export(tmp_path), 0)
    summary = tmp_path / "matched_frame_summary.csv"
    summary.write_text("trial,gray_image_a,gray_image_b\n"
                       "trial_1,trial_1_cam0_frame_0052_A_gray.png,trial_1_cam0_frame_0053_B_gray.png\n")
    with pytest.raises(ValueError, match="different images"):
        piv.attach_metadata([trial], summary)


def test_quality_checks_expose_angle_and_timing_mismatch(tmp_path):
    trial = piv.read_piv(export(tmp_path), 0)
    trial.metadata = {
        "target_adduction_rad": "0", "target_sweeping_rad": "0",
        "frame_a_adduction_rad": str(np.radians(3)), "frame_a_sweeping_rad": "0",
        "frame_a_time_s": "1", "frame_b_time_s": "1.04",
    }
    quality = piv.quality_row(trial, (1,), None, 2, 10)
    assert quality["max_angle_error_deg"] == pytest.approx(3)
    assert quality["timing_difference_percent"] == pytest.approx(20)
    assert quality["n_nonfinite_uv"] == 1
    assert quality["n_type_0"] == 1
    assert quality["n_vectors_used"] == 1
    assert "Angle error" in quality["warnings"]
    assert "interval" in quality["warnings"]


def test_csv_does_not_turn_missing_values_into_zero(tmp_path):
    path = tmp_path / "stats.csv"
    piv.write_csv(path, [{"speed": np.nan, "u": 0.0}])
    with path.open(newline="") as stream:
        row = next(csv.DictReader(stream))
    assert row == {"speed": "", "u": "0.0"}
