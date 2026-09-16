#!/usr/bin/env python3
"""Extract absolute obstacle positions from trusted density-experiment .npy files.

Only NumPy is required; --review needs OpenCV, and STL volume needs SciPy. See --help and the
generated README.md and extraction_notes.md. Recorded finite positions are retained
even when mocap quality is suspect; separate QC columns let the user filter later.
Only unavailable numeric values or unidentified events remain blank.
The script never writes to recordings. NPY files contain pickles: use trusted
experiment files only. A small optional signal cache avoids reloading RGB-D.
"""
from __future__ import annotations

import argparse
from collections import Counter
import csv
from dataclasses import asdict, dataclass
import hashlib
import json
import math
from pathlib import Path
import re
import shlex
import struct
import sys

import numpy as np

DEFAULT_GROUPS = {
    "empty": ("20260313_111621", "20260313_112741", "20260313_125836", "20260313_121725"),
    "lead": ("20260314_114731", "20260314_115216", "20260314_115730", "20260314_120728", "20260314_121349", "20260314_121907"),
    "steel": ("20260319_134621", "20260319_135104", "20260319_140118", "20260319_140705", "20260319_141232", "20260319_141641"),
    "resin": ("20260319_142653", "20260319_145057", "20260319_145447", "20260319_145955", "20260319_152329"),
}
RB_IDS = {"empty": 2, "lead": 3, "steel": 6, "resin": 5}
OBJECT_IDS = {"empty": "A", "lead": "D", "steel": "C", "resin": "B"}
MASSES_G = {"empty": 12.0, "lead": 215.0, "steel": 150.0, "resin": 43.0}
MASS_SOURCE = "density_experiment_log.xlsx; Lookup_Tables!A1:C5"
PIV_ADDUCTION_RAD = 0.785 + math.radians(-5.0)
PIV_SWEEPING_RAD = -1.315 + math.radians(45.0)
DEFAULT_OUTPUT = Path(__file__).resolve().parents[2] / "data" / "density_obstacle_positions"


def inspect_stl_envelope(path: Path, mm_per_unit: float) -> dict:
    """Audit a closed hemispherical shell with a straight rim; measure its envelope.

    The signed mesh volume is printing material, not the external volume of a
    hollow obstacle. A convex hull is accepted only when the source's outer
    hemisphere/rim surfaces agree with it. No mesh repair or implicit scaling.
    """
    from scipy.sparse import coo_matrix
    from scipy.sparse.csgraph import connected_components
    from scipy.spatial import ConvexHull

    if not math.isfinite(mm_per_unit) or mm_per_unit <= 0:
        raise ValueError("STL scale must be positive finite mm per coordinate unit")
    data = path.read_bytes()
    count = struct.unpack_from("<I", data, 80)[0] if len(data) >= 84 else 0
    if count and len(data) == 84 + 50 * count:
        dtype = np.dtype([("normal", "<f4", (3,)), ("vertices", "<f4", (3, 3)), ("attribute", "<u2")])
        triangles = np.frombuffer(data, dtype=dtype, offset=84)["vertices"].astype(float)
        file_format = "binary STL"
    else:
        try:
            matches = re.findall(r"(?im)^\s*vertex\s+(\S+)\s+(\S+)\s+(\S+)\s*$", data.decode("ascii"))
            triangles = np.asarray(matches, dtype=float).reshape(-1, 3, 3)
        except (UnicodeDecodeError, ValueError) as exc:
            raise ValueError("Invalid binary or ASCII STL") from exc
        file_format = "ASCII STL"
    triangles *= mm_per_unit
    if not len(triangles) or not np.isfinite(triangles).all():
        raise ValueError("STL has no finite triangles")
    vertices, indices = np.unique(triangles.reshape(-1, 3), axis=0, return_inverse=True)
    faces = indices.reshape(-1, 3)
    cross = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
    areas = np.linalg.norm(cross, axis=1) / 2
    if np.any(areas <= 1e-12):
        raise ValueError("STL contains degenerate triangles")
    edges = np.concatenate([faces[:, [0, 1]], faces[:, [1, 2]], faces[:, [2, 0]]])
    unique_edges, edge_indices, edge_counts = np.unique(np.sort(edges, axis=1), axis=0,
                                                       return_inverse=True, return_counts=True)
    if np.any(edge_counts != 2):
        raise ValueError("STL is not closed: each edge must belong to two faces")
    balance = np.bincount(edge_indices, weights=np.where(edges[:, 0] < edges[:, 1], 1, -1))
    if np.any(balance != 0):
        raise ValueError("STL face winding is inconsistent")
    graph = coo_matrix((np.ones(len(edges)), (edges[:, 0], edges[:, 1])),
                       shape=(len(vertices), len(vertices))).tocsr()
    components = connected_components(graph, directed=False, return_labels=False)
    if components != 1:
        raise ValueError("Multiple STL components require an explicit assembly-volume definition")
    low, high = vertices.min(axis=0), vertices.max(axis=0)
    extents = high - low
    axis = int(np.argmin(extents))
    transverse = [i for i in range(3) if i != axis]
    radius = float(np.mean(extents[transverse]) / 2)
    tolerance = max(radius * 1e-5, 1e-6)
    if abs(extents[transverse[0]] - extents[transverse[1]]) > tolerance:
        raise ValueError("STL does not have the expected circular dome base")
    center = (low + high) / 2
    center[axis] = low[axis] + radius
    collar = float(high[axis] - center[axis])
    if collar < -tolerance:
        raise ValueError("STL is not a hemisphere with a nonnegative straight rim")
    delta = triangles - center
    on_sphere = (np.abs(np.linalg.norm(delta, axis=2) - radius) < tolerance)
    on_sphere &= triangles[:, :, axis] <= center[axis] + tolerance
    on_cylinder = np.abs(np.linalg.norm(delta[:, :, transverse], axis=2) - radius) < tolerance
    on_cylinder &= triangles[:, :, axis] >= center[axis] - tolerance
    outer = on_sphere.all(axis=1) | on_cylinder.all(axis=1)
    # A base disk's vertices may all lie on the rim circle; it is not side area.
    outer &= ~np.all(np.abs(triangles[:, :, axis] - high[axis]) < tolerance, axis=1)
    if not outer.any():
        raise ValueError("Cannot identify the source STL's outer hemisphere/rim surfaces")
    hull = ConvexHull(vertices)
    hull_triangles = vertices[hull.simplices]
    hull_areas = np.linalg.norm(np.cross(hull_triangles[:, 1] - hull_triangles[:, 0],
                                         hull_triangles[:, 2] - hull_triangles[:, 0]), axis=1) / 2
    base = np.all(np.abs(hull_triangles[:, :, axis] - high[axis]) < tolerance, axis=1)
    side_area_residual = abs(float(areas[outer].sum() / hull_areas[~base].sum()) - 1)
    # An origin on the rim plane makes its closing disk contribute zero volume.
    rim_origin = center.copy()
    rim_origin[axis] = high[axis]
    relative = triangles[outer] - rim_origin
    surface_volume = abs(float(np.einsum("ij,ij->i", relative[:, 0],
                               np.cross(relative[:, 1], relative[:, 2])).sum() / 6))
    volume_residual = abs(surface_volume / hull.volume - 1)
    if side_area_residual > 1e-4 or volume_residual > 1e-4:
        raise ValueError("Convex hull does not reproduce the STL exterior; external volume requires review")
    relative = triangles - vertices.mean(axis=0)
    material_volume = abs(float(np.einsum("ij,ij->i", relative[:, 0],
                               np.cross(relative[:, 1], relative[:, 2])).sum() / 6))
    sphere_points = vertices[vertices[:, axis] < center[axis] - tolerance]
    inner_radius = float(np.min(np.linalg.norm(sphere_points - center, axis=1)))
    return {
        "source_stl": str(path.resolve()), "source_stl_sha256": hashlib.sha256(data).hexdigest(),
        "format": file_format, "mm_per_stl_unit": mm_per_unit,
        "unit_basis": "explicit scale; diameter checked against user-supplied 50.8 mm",
        "triangles": len(triangles), "vertices": len(vertices), "unique_edges": len(unique_edges),
        "closed_edges": True, "consistent_winding": True, "connected_components": int(components),
        "bounds_mm": [low.tolist(), high.tolist()], "axis_extents_mm": extents.tolist(),
        "height_axis": "xyz"[axis], "external_height_mm": float(extents[axis]),
        "external_base_radius_mm": radius, "straight_rim_height_mm": collar,
        "nominal_shell_thickness_mm": radius - inner_radius,
        "external_volume_cm3": float(hull.volume / 1000),
        "print_material_volume_cm3": material_volume / 1000,
        "envelope_volume_from_source_outer_facets_cm3": surface_volume / 1000,
        "outer_surface_volume_relative_difference": volume_residual,
        "outer_side_area_relative_difference": side_area_residual,
        "external_volume_method": "convex outer envelope closed across rim; checked against source hemisphere/rim facets",
        "interpretation": "CAD-based effective volume including the interior; excludes protruding tracking markers; no physical print measurement",
    }


@dataclass
class Config:
    max_sync_s: float = 0.05
    angle_tolerance_deg: float = 5.0
    onset_velocity_rad_s: float = 0.2
    final_hold_tolerance_deg: float = 2.0
    stale_pose_s: float = 0.25
    jump_threshold_mm: float = 5.0


def parse_trials(value: str) -> list[int]:
    """CSV Trial is a stroke subset; Trial_Number is never used here."""
    value = str(value or "").strip().upper()
    if value in ("", "ALL", "ALL_7", "NAN"):
        return list(range(1, 8))
    value = re.sub(r"TRIAL[_ ]*", "", value).strip("[]() ")
    result = set()
    for token in re.split(r"[,;\s]+", value):
        match = re.fullmatch(r"(\d+)(?:[-:](\d+))?", token)
        if not match:
            raise ValueError(f"Unrecognized Trial subset {value!r}")
        lo, hi = int(match[1]), int(match[2] or match[1])
        if lo < 1 or hi < lo:
            raise ValueError(f"Invalid Trial subset {value!r}")
        result.update(range(lo, hi + 1))
    return sorted(result)


def rotate_positions(position: np.ndarray, incline_deg: float) -> np.ndarray:
    """World metres -> absolute slope mm; y into bed, z downslope, no translation."""
    x, y, z = np.asarray(position, dtype=float).T
    c, s = math.cos(math.radians(incline_deg)), math.sin(math.radians(incline_deg))
    return 1000.0 * np.column_stack((x, -(c * y + s * z), -s * y + c * z))


def nearest_index(times: np.ndarray, query: float) -> int:
    i = int(np.searchsorted(times, query))
    candidates = [j for j in (i - 1, i) if 0 <= j < len(times)]
    return min(candidates, key=lambda j: (abs(float(times[j]) - query), j))


def number(value, default=math.nan) -> float:
    try:
        result = float(value)
        return result if math.isfinite(result) else default
    except (TypeError, ValueError):
        return default


def valid_time(times: np.ndarray) -> bool:
    return len(times) >= 2 and np.isfinite(times).all() and np.all(np.diff(times) > 0)


def blank_events() -> dict:
    row = {"mocap_qc_flag": "unavailable", "mocap_qc_notes": ""}
    for event in ("start", "reference", "end"):
        for key in ("yprime_mm", "zprime_mm", "time_s", "mocap_sample_index",
                    "motor_sample_index", "motor_time_s", "motor_offset_s",
                    "camera0_frame_index", "camera0_time_s", "camera0_offset_s"):
            row[f"{event}_{key}"] = None
        row[f"{event}_status"] = "unavailable"
    return row


def extract_trial(payload: dict, rb_id: int, incline_deg: float,
                  target_adduction_rad: float | None, target_sweeping_rad: float | None,
                  config: Config | None = None) -> dict:
    """Extract using raw common-clock streams. Indices always refer to original arrays."""
    cfg = config or Config()
    row = blank_events()
    notes = []
    mocap_qc = []
    meta = payload.get("metadata", {})
    row.update(position_units="mm", angle_units="rad", timestamp_units="s",
               timestamp_origin="trial run_start host wall clock", mocap_rb_id=rb_id,
               coordinate_origin="original Motive world origin; rotation only",
               position_definition="recorded obstacle rigid-body pivot; no COM correction",
               trial_start_time_recorded=meta.get("start_time", ""),
               trial_stop_time_recorded=meta.get("stop_time", ""),
               incline_deg=incline_deg,
               reference_target_adduction_rad=target_adduction_rad,
               reference_target_sweeping_rad=target_sweeping_rad,
               pre_action_submerged_fraction=None, pre_action_submerged_volume_mm3=None,
               submergence_status="unavailable: no verified per-trial surface/volume result")
    row["source_position_series"] = f"mocap_raw[{rb_id}].position_x/y/z"

    def finish():
        if mocap_qc:
            row["mocap_qc_flag"] = "suspect"
            row["mocap_qc_notes"] = "; ".join(dict.fromkeys(mocap_qc))
        exported = all(row.get(f"{e}_yprime_mm") is not None for e in ("start", "reference", "end"))
        row["extraction_status"] = ("complete_with_qc" if notes or mocap_qc else "complete") if exported else "partial"
        row["qc_notes"] = "; ".join(dict.fromkeys(notes))
        return row

    mocap = payload.get("mocap_raw", {})
    state = mocap.get(str(rb_id), mocap.get(rb_id))
    motor = payload.get("robot_state_raw", {})
    if not isinstance(state, dict):
        notes.append("requested_rigid_body_missing")
        return finish()
    try:
        mt = np.asarray(state["time"], dtype=float)
        xyz = np.column_stack([state[f"position_{a}"] for a in "xyz"]).astype(float)
        rt = np.asarray(motor["time"], dtype=float)
        ad = -2 * np.pi * np.asarray(motor["rightadduction_pos"], dtype=float)
        sw = -2 * np.pi * np.asarray(motor["rightsweeping_pos"], dtype=float)
    except (KeyError, ValueError, TypeError) as exc:
        notes.append(f"missing_or_invalid_raw_series:{exc}")
        return finish()
    if not valid_time(mt) or not valid_time(rt) or xyz.shape != (len(mt), 3) or len(ad) != len(rt) or len(sw) != len(rt):
        notes.append("invalid_series_shape_or_nonmonotonic_timestamps")
        return finish()
    if not np.isfinite(ad).all() or not np.isfinite(sw).all():
        notes.append("nonfinite_motor_angles; phase and synchronization cannot be verified")
        return finish()
    pos = rotate_positions(xyz, incline_deg)
    # Availability controls extraction; heuristic quality checks only add flags.
    available = np.isfinite(xyz).all(axis=1) & (mt >= 0)
    zero_xyz = np.linalg.norm(xyz, axis=1) <= 1e-9
    valid = available & ~zero_xyz
    quaternion_invalid = np.zeros(len(mt), dtype=bool)
    isolated_spike = np.zeros(len(mt), dtype=bool)
    row["mocap_qc_flag"] = "not_flagged"
    quat_keys = [f"orientation_{a}" for a in "xyzw"]
    if all(k in state for k in quat_keys):
        q = np.column_stack([state[k] for k in quat_keys])
        quaternion_invalid = ~np.isfinite(q).all(axis=1) | (np.linalg.norm(q, axis=1) <= 1e-9)
        valid &= ~quaternion_invalid
    else:
        notes.append("quaternion_validation_unavailable")
    # Identify a single-frame excursion for QC, but preserve the recorded value.
    step = np.linalg.norm(np.diff(pos, axis=0), axis=1)
    if len(pos) > 2:
        spike = ((step[:-1] > cfg.jump_threshold_mm) & (step[1:] > cfg.jump_threshold_mm)
                 & (np.linalg.norm(pos[2:] - pos[:-2], axis=1) < cfg.jump_threshold_mm / 2))
        valid[1:-1] &= ~spike
        isolated_spike[1:-1] = spike
        row["isolated_tracking_spikes"] = int(spike.sum())
    # A packet stream repeating exactly the same pose can conceal tracking loss.
    changed = np.r_[True, np.any(np.diff(xyz, axis=0) != 0, axis=1)]
    run_start = np.maximum.accumulate(np.where(changed, np.arange(len(mt)), 0))
    stale = (mt - mt[run_start]) > cfg.stale_pose_s
    valid &= ~stale
    row.update(mocap_samples=len(mt), motor_samples=len(rt),
               mocap_invalid_samples=int((~valid).sum()),
               mocap_max_packet_gap_s=float(np.max(np.diff(mt))),
               motor_max_packet_gap_s=float(np.max(np.diff(rt))),
               mocap_raw_last_time_s=float(mt[-1]), motor_raw_last_time_s=float(rt[-1]),
               tracking_validity_available=False)
    notes.append("Motive tracking-valid flag/exposure timestamps not recorded; host receive-time synchronization")
    if not valid.all():
        notes.append(f"invalid_or_stale_mocap_samples={int((~valid).sum())}/{len(mt)}")
    if np.any(step > cfg.jump_threshold_mm):
        notes.append(f"position_steps_over_{cfg.jump_threshold_mm:g}mm={int((step > cfg.jump_threshold_mm).sum())}; inspect possible rolling/tracking loss")
    if row["mocap_max_packet_gap_s"] > cfg.max_sync_s or row["motor_max_packet_gap_s"] > cfg.max_sync_s:
        notes.append("recording_contains_timestamp_gap_over_sync_limit")
    if all(f"rotated_position_{a}" in state for a in "yz"):
        saved = np.column_stack((-np.asarray(state["rotated_position_y"]), state["rotated_position_z"])) * 1000
        residual = np.max(np.abs(pos[valid, 1:] - saved[valid])) if valid.any() else math.nan
        row["saved_rotation_max_residual_mm"] = float(residual)
        if residual > 1e-5:
            notes.append("stored_rotation_disagrees_with_log_incline; recomputed from world coordinates")
    cam = np.asarray(payload.get("camera_time_0", []), dtype=float)
    camera_ok = valid_time(cam)
    duration = number(meta.get("duration_sec"))
    complete = number(meta.get("traj_complete_time_sec"))
    dwell = number(meta.get("dwell_time_sec"))
    record_stop = max([x for x in (duration, cam[-1] if camera_ok else math.nan) if math.isfinite(x)], default=math.nan)
    row.update(recording_stop_time_s=record_stop, trajectory_complete_time_s=complete,
               dwell_time_s=dwell, post_dwell_threshold_time_s=complete + dwell,
               raw_mocap_tail_beyond_recording_s=float(mt[-1] - record_stop))
    if mt[-1] > record_stop + 0.01:
        notes.append("raw snapshot continues after RGB-D stop, before save and next trial command")
    # Validate saved nearest-neighbor alignment without using its repeated samples as observations.
    aligned = payload.get("mocap", {}).get(str(rb_id), payload.get("mocap", {}).get(rb_id, {}))
    if camera_ok and isinstance(aligned, dict) and len(aligned.get("time", [])) == len(cam):
        expected = np.array([mt[nearest_index(mt, t)] for t in cam])
        row["saved_alignment_max_time_residual_s"] = float(np.max(np.abs(np.asarray(aligned["time"]) - expected)))
        row["camera_mocap_alignment_max_abs_s"] = float(np.max(np.abs(expected - cam)))
        if row["saved_alignment_max_time_residual_s"] > 1e-9:
            notes.append("saved_mocap_alignment_differs_from_raw_nearest")

    def event(event_name: str, index: int):
        t = float(mt[index])
        ri = nearest_index(rt, t)
        event_qc = []
        sync_gap = abs(float(rt[ri]) - t)
        if sync_gap > cfg.max_sync_s:
            event_qc.append(f"motor_sync_gap_{sync_gap:.6f}s (limit={cfg.max_sync_s:g}s)")
        if stale[index]:
            age = float(mt[index] - mt[run_start[index]])
            event_qc.append(f"unchanged_xyz_for_{age:.3f}s (threshold={cfg.stale_pose_s:g}s; possible held tracking)")
        if zero_xyz[index]:
            event_qc.append("all_zero_xyz")
        if quaternion_invalid[index]:
            event_qc.append("invalid_quaternion")
        if isolated_spike[index]:
            event_qc.append("isolated_position_spike")
        neighborhood = (mt[1:] >= t - 0.05) & (mt[1:] <= t + 0.05)
        if np.any(step[neighborhood] > cfg.jump_threshold_mm):
            event_qc.append(f"position_jump_over_{cfg.jump_threshold_mm:g}mm_within_50ms")
        mocap_qc.extend(f"{event_name}: {issue}" for issue in event_qc)
        row[f"{event_name}_mocap_qc"] = "; ".join(event_qc)
        row.update({f"{event_name}_yprime_mm": float(pos[index, 1]),
                    f"{event_name}_zprime_mm": float(pos[index, 2]),
                    f"{event_name}_time_s": t,
                    f"{event_name}_mocap_sample_index": int(index),
                    f"{event_name}_motor_sample_index": ri,
                    f"{event_name}_motor_time_s": float(rt[ri]),
                    f"{event_name}_motor_offset_s": float(rt[ri] - t),
                    f"{event_name}_status": "ok_with_sync_warning" if sync_gap > cfg.max_sync_s else "ok"})
        if camera_ok:
            ci = nearest_index(cam, t)
            row.update({f"{event_name}_camera0_frame_index": ci,
                        f"{event_name}_camera0_time_s": float(cam[ci]),
                        f"{event_name}_camera0_offset_s": float(cam[ci] - t)})
        return ri

    waypoints = np.asarray(payload.get("trajectory_points", []), dtype=float)
    if waypoints.size < 12 or waypoints.size % 3:
        notes.append("missing_or_invalid_trajectory_waypoints")
        return finish()
    wp = waypoints.reshape(-1, 3)
    # Locate the positive sweeping run at maximum insertion. Merge adjacent segments.
    delta = np.diff(wp[:, :2], axis=0)
    segments = np.flatnonzero((delta[:, 1] > 0) & (np.abs(delta[:, 0]) < 1e-5)
                             & np.isclose(wp[:-1, 0], np.max(wp[:, 0]), atol=1e-5))
    if not len(segments) or np.any(np.diff(segments) != 1):
        notes.append("intended_working_sweep_not_unique_in_saved_trajectory")
        return finish()
    low_ad, high_ad = float(np.min(wp[:, 0])), float(np.max(wp[:, 0]))
    low_sw, high_sw = float(wp[segments[0], 1]), float(wp[segments[-1] + 1, 1])
    phase_limit = complete if math.isfinite(complete) else min(rt[-1], record_stop)
    grid = np.arange(max(0, rt[0]), min(rt[-1], phase_limit), 0.005)
    ga, gs = np.interp(grid, rt, ad), np.interp(grid, rt, sw)
    # Central 50ms difference suppresses encoder noise; confirm 6/7 points rising.
    velocity = (np.interp(grid + 0.025, rt, sw) - np.interp(grid - 0.025, rt, sw)) / 0.05
    rising = velocity > cfg.onset_velocity_rad_s
    sustained = np.array([np.count_nonzero(rising[i:i + 7]) >= 6 for i in range(len(grid))])
    eligible = ((ga > low_ad + 0.5 * (high_ad - low_ad))
                & (gs < low_sw + 0.3 * (high_sw - low_sw)) & sustained)
    starts = np.flatnonzero(eligible)
    if not len(starts):
        notes.append("working_sweep_onset_not_identified")
    else:
        onset = float(grid[starts[0]])
        # End at first retraction below 80% insertion after reaching the latter half.
        stop_candidates = np.flatnonzero((grid > onset)
            & (gs > low_sw + 0.6 * (high_sw - low_sw))
            & (ga < low_ad + 0.8 * (high_ad - low_ad)))
        sweep_end = float(grid[stop_candidates[0]]) if len(stop_candidates) else math.nan
        row.update(sweep_onset_time_s=onset, sweep_end_time_s=sweep_end,
                   sweep_onset_motor_sample_index=nearest_index(rt, onset),
                   sweep_onset_resolution_s=0.005,
                   sweep_onset_velocity_threshold_rad_s=cfg.onset_velocity_rad_s)
        before = np.flatnonzero(available & (mt < onset) & (mt >= onset - cfg.max_sync_s))
        if len(before):
            event("start", int(before[-1]))
            row["start_time_before_sweep_s"] = onset - mt[before[-1]]
        else:
            row["start_status"] = "no_valid_tracking_immediately_before_sweep"
            mocap_qc.append("start: no finite recorded position within 50ms before sweep")
            notes.append(row["start_status"])
        if target_adduction_rad is None or target_sweeping_rad is None:
            row["reference_status"] = "reference_configuration_pending"
        elif not math.isfinite(sweep_end):
            row["reference_status"] = "working_sweep_end_unidentified"
            notes.append(row["reference_status"])
        else:
            # Select angles before testing position validity: never move the reference
            # to a different configuration simply because tracking was lost there.
            candidates = np.flatnonzero((mt >= onset) & (mt <= sweep_end)
                                       & (mt >= rt[0]) & (mt <= rt[-1]))
            if len(candidates):
                motor_indices = np.array([nearest_index(rt, float(mt[i])) for i in candidates])
                synchronized = np.abs(rt[motor_indices] - mt[candidates]) <= cfg.max_sync_s
                candidates, motor_indices = candidates[synchronized], motor_indices[synchronized]
            if not len(candidates):
                row["reference_status"] = "no_valid_synchronized_sweep_tracking"
                notes.append(row["reference_status"])
            else:
                errors = np.maximum(np.abs(ad[motor_indices] - target_adduction_rad),
                                    np.abs(sw[motor_indices] - target_sweeping_rad))
                best = int(np.argmin(errors))
                mi, ri = int(candidates[best]), int(motor_indices[best])
                err_ad, err_sw = ad[ri] - target_adduction_rad, sw[ri] - target_sweeping_rad
                row.update(reference_actual_adduction_rad=float(ad[ri]),
                           reference_actual_sweeping_rad=float(sw[ri]),
                           reference_adduction_error_rad=float(err_ad), reference_sweeping_error_rad=float(err_sw),
                           reference_max_abs_error_deg=math.degrees(float(errors[best])),
                           reference_candidate_time_s=float(mt[mi]),
                           reference_candidate_yprime_mm=float(pos[mi, 1]) if available[mi] else None,
                           reference_candidate_zprime_mm=float(pos[mi, 2]) if available[mi] else None,
                           reference_candidate_mocap_sample_index=mi,
                           reference_candidate_motor_sample_index=ri,
                           reference_candidate_motor_time_s=float(rt[ri]),
                           reference_candidate_motor_offset_s=float(rt[ri] - mt[mi]),
                           reference_match_rule="minimize max absolute joint-angle error during positive working sweep")
                if camera_ok:
                    ci = nearest_index(cam, float(mt[mi]))
                    row.update(reference_candidate_camera0_frame_index=ci,
                               reference_candidate_camera0_time_s=float(cam[ci]),
                               reference_candidate_camera0_offset_s=float(cam[ci] - mt[mi]))
                angle_limit = math.radians(cfg.angle_tolerance_deg) + 1e-12
                acceptable = errors <= angle_limit
                accepted_times = mt[candidates[acceptable]]
                episodes = 0 if not len(accepted_times) else 1 + int(np.sum(np.diff(accepted_times) > cfg.max_sync_s))
                row.update(reference_matching_episodes=episodes, reference_samples_within_tolerance=int(acceptable.sum()))
                if not available[mi]:
                    row["reference_status"] = "no_valid_tracking_at_best_angle_match"
                    mocap_qc.append("reference: recorded position nonfinite at best angle match")
                    notes.append(row["reference_status"])
                elif errors[best] > angle_limit:
                    row["reference_status"] = "joint_angle_target_not_reached_within_tolerance"
                    notes.append(row["reference_status"])
                elif episodes > 1:
                    row["reference_status"] = "ambiguous_multiple_angle_matching_episodes"
                    notes.append(row["reference_status"])
                else:
                    event("reference", mi)

    threshold = complete + dwell
    if not math.isfinite(threshold) or dwell < 0 or complete < 0:
        row["end_status"] = "completion_or_dwell_metadata_missing"
    elif not math.isfinite(record_stop) or record_stop < threshold:
        row["end_status"] = "recording_stopped_before_dwell_completed"
    else:
        after = np.flatnonzero(available & (mt >= threshold))
        if not len(after):
            row["end_status"] = "no_valid_post_dwell_tracking"
            mocap_qc.append("end: no finite recorded position after full dwell")
        else:
            last = int(after[-1])
            if mt[-1] - mt[last] > cfg.max_sync_s:
                row["end_status"] = "tracking_lost_before_recording_endpoint"
                mocap_qc.append("end: final finite recorded position is over 50ms before recording endpoint")
            else:
                if last != len(mt) - 1:
                    mocap_qc.append("end: final position nonfinite; retained finite sample within 50ms")
                tail = (rt >= mt[last] - 0.5) & (rt <= mt[last] + cfg.max_sync_s)
                hold_warning = not tail.any()
                if tail.any():
                    hold_error = np.max(np.maximum(np.abs(ad[tail] - wp[-1, 0]), np.abs(sw[tail] - wp[-1, 1])))
                    row["end_motor_hold_max_error_deg"] = math.degrees(float(hold_error))
                    hold_warning = hold_error > math.radians(cfg.final_hold_tolerance_deg)
                event("end", last)
                if hold_warning:
                    row["end_status"] = "ok_with_motor_hold_warning"
                    mocap_qc.append("end: returned_motor_hold_not_verified; recorded post-dwell position retained")
                tail_obs = available & (mt >= mt[last] - 0.5) & (mt <= mt[last])
                spans = np.ptp(pos[tail_obs, 1:], axis=0)
                row.update(end_last_half_second_yprime_span_mm=float(spans[0]),
                           end_last_half_second_zprime_span_mm=float(spans[1]),
                           end_time_after_dwell_s=float(mt[last] - threshold),
                           end_selection_rule="last finite same-trial raw mocap sample after full dwell; mocap/sync/hold concerns flagged without dropping finite values")
                if np.max(spans) > 1.0:
                    notes.append("obstacle_varies_over_1mm_in_final_half_second")
    if row["end_status"] != "ok":
        notes.append(row["end_status"])
    return finish()


def write_csv(path: Path, rows: list[dict]):
    fields = list(dict.fromkeys(key for row in rows for key in row))
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow({k: ("" if v is None or isinstance(v, (float, np.floating)) and not np.isfinite(v) else v)
                             for k, v in row.items()})


def summary_row(row: dict) -> dict:
    """Keep plotting values plus the event provenance requested by the user."""
    fields = ["session_id", "obstacle_material", "obstacle_density", "density_units", "trial"]
    for event in ("start", "reference", "end"):
        fields.extend(f"{event}_{field}" for field in
                      ("yprime_mm", "zprime_mm", "time_s", "mocap_sample_index"))
    summary = {key: row.get(key) for key in fields}
    for joint in ("adduction", "sweeping"):
        for kind in ("target", "actual"):
            angle = number(row.get(f"reference_{kind}_{joint}_rad"))
            summary[f"reference_{kind}_{joint}_deg"] = math.degrees(angle) if math.isfinite(angle) else None
    for key in ("reference_max_abs_error_deg", "start_status", "reference_status", "end_status",
                "extraction_status", "source_recording"):
        summary[key] = row.get(key)
    summary["mocap_qc_flag"] = row.get("mocap_qc_flag", "unavailable")
    summary["mocap_qc_notes"] = row.get("mocap_qc_notes", "")
    # Common acquisition caveats and raw diagnostics belong in the audit/notes.
    # Keep only row-specific limitations affecting how a position should be used.
    original = row.get("qc_notes", "")
    notes = []
    if "height mismatch" in row.get("density_geometry_status", ""):
        notes.append("STL-based density; CAD/reported height mismatch")
    if "numeric_obstacle_density_unavailable" in original:
        notes.append("Density unavailable")
    for marker, label in (
        ("invalid_or_stale_mocap_samples", "Potentially invalid/unchanged mocap samples in trial; recorded values retained"),
        ("position_steps_over_", "Large tracking jumps in trial"),
        ("recording_contains_timestamp_gap_over_sync_limit", "Timestamp gaps in trial"),
        ("obstacle_varies_over_1mm_in_final_half_second", "Position varies >1 mm near endpoint"),
        ("stored_rotation_disagrees_with_log_incline", "Saved rotation disagrees; recomputed from world positions"),
        ("saved_mocap_alignment_differs_from_raw_nearest", "Saved synchronization disagrees with raw timestamps"),
        ("log_object_id_disagrees_with_requested_material", "Material disagrees with source log"),
        ("session_missing_from_input_log", "Session missing from source log"),
    ):
        if marker in original:
            notes.append(label)
    if row.get("extraction_status") in ("error", "missing_trial") or any(
            row.get(f"{event}_status") == "unavailable" for event in ("start", "reference", "end")):
        notes.append(original)
    if row.get("source_log_notes"):
        notes.append(f"Session note: {row['source_log_notes']}")
    summary["qc_notes"] = "; ".join(dict.fromkeys(n for n in notes if n))
    return summary


def load_payload(path: Path, cache_dir: Path | None, need_rgb=False) -> dict:
    stat = path.stat()
    key = hashlib.sha256(f"{path.resolve()}:{stat.st_size}:{stat.st_mtime_ns}".encode()).hexdigest()[:16]
    cache = cache_dir / f"{path.parent.name}_{path.stem}_{key}.npy" if cache_dir else None
    if cache and cache.is_file() and not need_rgb:
        return np.load(cache, allow_pickle=True).item()
    payload = np.load(path, allow_pickle=True).item()
    if not isinstance(payload, dict):
        raise ValueError("trial payload is not a dictionary")
    if cache and not cache.is_file():
        cache.parent.mkdir(parents=True, exist_ok=True)
        slim = {k: v for k, v in payload.items() if not k.startswith(("rgb_", "depth_"))}
        np.save(cache, slim, allow_pickle=True)
    return payload


def save_review(payload: dict, row: dict, out: Path):
    import cv2
    panels = []
    for camera in (0, 1):
        frames = payload.get(f"rgb_{camera}")
        times = np.asarray(payload.get(f"camera_time_{camera}", []))
        if frames is None or not len(times):
            continue
        strip = []
        for event in ("start", "reference", "end"):
            time = row.get(f"{event}_time_s")
            label_event = event
            if time is None and event == "reference":
                time = row.get("reference_candidate_time_s")
                label_event = "reference candidate"
            if time is None:
                time = row.get("sweep_onset_time_s") if event == "start" else float(times[-1])
                label_event = f"{event} missing; context"
            if time is None:
                panel = np.zeros((300, 424, 3), dtype=np.uint8)
                cv2.putText(panel, f"{event}: unavailable", (8, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            else:
                idx = nearest_index(times, time)
                panel = np.zeros((300, 424, 3), dtype=np.uint8)
                panel[60:] = cv2.resize(frames[idx], (424, 240))
                for n, label in enumerate((f"cam{camera} {label_event} frame {idx}",
                                          f"frame {times[idx]:.4f}s; query {time:.4f}s")):
                    cv2.putText(panel, label, (8, 20 + n * 23), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
            strip.append(panel)
        panels.append(np.concatenate(strip, axis=1))
    if panels:
        out.mkdir(parents=True, exist_ok=True)
        path = out / f"{row['session_id']}_trial_{row['trial']}.jpg"
        if not cv2.imwrite(str(path), np.concatenate(panels, axis=0)):
            raise OSError(f"Could not write {path}")
        row["review_contact_sheet"] = str(path.resolve())


def write_notes(output: Path, rows: list[dict], manifest: dict):
    lines = ["# Density-experiment obstacle positions", "",
             f"{manifest['rows']} rows from {manifest['sessions']} selected sessions. No requested trial was silently dropped.", "",
             "## Files and reuse", "",
             "`trial_position_summary.csv` is the compact plotting file: session/material/density/trial, the three signed positions in mm, timestamps in seconds from trial start, one original mocap sample index per position, target and actual reference angles/error in degrees, event statuses, source recording, mocap_qc_flag/mocap_qc_notes, and concise general QC notes. Blank numeric fields mean unavailable. Indices are zero-based (181 means the 182nd raw mocap sample within that trial). The original recordings are not needed for plotting or calculating displacements.", "",
             "`trial_position_audit.csv` contains the full extraction diagnostics, motor/video indices, candidates, calibration details, and source provenance. Candidate duplicates and unavailable submergence columns are omitted from the main CSV. `extraction_manifest.json` records the command, thresholds, input hashes, reference target, and coverage. `recording_checks/` holds optional RGB context sheets. `sample_verification.csv` records independent coordinate/angle recalculations. The verifier joins the compact file to the audit by session/trial when needed.", "",
             "From the repository root, the extraction command used was:", "", "```bash",
             shlex.join(["python3"] + manifest["command"]), "```", "",
             "Only NumPy is required for extraction. OpenCV is required for `--review`; SciPy is required for `--density-mode stl-envelope`. `--cache-dir` is optional and contains small signal-only copies keyed by original source path, byte size, and modification time. The recordings are read-only. NPY inputs use pickles and must be trusted.", "",
             "## Coordinate convention", "",
             "Source: absolute `mocap_raw[rigid_body_id].position_x/y/z` in metres, never `zeroed_*`. For incline θ=23°, export y′ = −1000(cosθ y + sinθ z) and z′ = 1000(−sinθ y + cosθ z), in mm. The recorded rotation is checked against this calculation. Existing plots define rotated y as out of sand and z as downslope; y is negated here to make positive y′ point into the bed. No translation, per-trial reset, absolute-value operation, or COM offset is applied. Negative absolute y′ values are expected: the retained Motive origin is not the sand surface. These are recorded rigid-body pivot positions, not a newly estimated mass centre.", "",
             "Each file is one recorded stroke/trial. The CSV `Trial` column specifies a subset; blank/ALL_7 means 1–7. `Trial_Number` is the session repeat number. Rigid body IDs are empty=2, lead=3, steel=6, resin=5. The unrelated session metadata `slope=0` does not override `mocap_incline_deg=23` or log `Incline_deg=23`.", "",
             "## Events", "",
             "**Start:** last finite recorded mocap sample strictly before the measured working-sweep onset, at most 50 ms old. The positive sweep follows insertion, after the initial negative positioning sweep. Identify its stored positive constant-adduction waypoint segments; on a 5 ms motor grid, require the centred 50 ms sweeping velocity to exceed 0.2 rad/s at at least 6 of the next 7 grid points, while adduction exceeds half its stored range and sweeping is in the first 30% of the working range. This is a measured onset criterion with smoothing/threshold uncertainty, not a recorded phase trigger. The sweep window ends when adduction retracts below 80% after the latter part of the sweep.", "",
             f"**Reference target:** adduction {manifest['reference_adduction_rad']} rad; sweeping {manifest['reference_sweeping_rad']} rad. Source: `{manifest.get('reference_summary_path', 'command-line selection')}`. Motor feedback converts turns to radians with −2π. Match both angles jointly by minimizing their maximum absolute error within the working sweep, with nearest raw motor samples at mocap timestamps. The configuration is selected before checking obstacle tracking, so tracking loss cannot shift the reference to another pose.", "",
             f"`reference_*` position fields accept both joint errors ≤{manifest['config']['angle_tolerance_deg']:g}° and one contiguous matching episode. The user approved up to 5°; the default acceptance limit is now 5°. Target angles, actual angles, and the maximum absolute joint error are retained in degrees in the main CSV. The target remains populated even if recording/tracking is missing. This is an angle-matched observed position within the tolerance, not an assertion that the target was reached exactly. Finite positions with suspect tracking are retained with QC flags. An absent/nonfinite position or multiple separated matching episodes leaves the position blank. Closest-candidate diagnostics remain in the audit file only.", "",
             "**End:** final finite same-trial raw mocap sample after `traj_complete_time_sec + dwell_time_sec`, with the full dwell confirmed within the RGB-D/metadata recording interval. Motor synchronization >50 ms and returned motor configuration errors >2° over the terminal 0.5 s are flagged; finite post-dwell positions remain exported. A nonfinite last position may fall back ≤50 ms to a finite sample still after the dwell. No such numeric sample leaves the endpoint blank. Exactly repeated values, invalid quaternions, zero positions and tracking jumps are retained with QC warnings. Obstacle terminal variation >1 mm is flagged. Raw snapshots happen after RGB-D finalization but before saving and before publishing the next trial command, so their final recorded samples belong to the same post-dwell state. The audit records the raw extension and the separate nearest RGB frame/time; an RGB frame before a raw endpoint is context, not an image of that exact instant. No next-trial start is reused.", "",
             "The controller executes initial positioning → insertion → positive sweep → retraction → return, then publishes completion. No separate `feel` command/event exists in the inspected controller; the requested endpoint is interpreted as post-dwell following this complete sequence, not sweep-end.", "",
             "## Quality and limitations", "",
             "Motor, mocap, and camera data use a shared per-trial host clock. The saved nearest-camera alignment is checked against raw timestamps, but host receipt time is not hardware exposure time. Camera 0/1 share collector poll timestamps rather than independently measured exposure times. Recorded ISO start/stop timestamps are preserved as written, including their timezone offsets; no retrospective timezone correction is made.", "",
             "Finite recorded positions are exported even if all-zero, accompanied by an invalid quaternion, part of an isolated >5 mm excursion, or exactly unchanged for >0.25 s. These conditions now produce mocap_qc_flag/mocap_qc_notes instead of removing values. The 0.25 s cutoff is a heuristic, not confirmed tracking loss. Nonfinite positions cannot yield numeric slope coordinates and remain unavailable. Timestamp gaps, larger steps, and source log rolling notes remain explicit QC information. The recordings do not preserve a Motive tracking-valid flag; these checks cannot certify all subtle tracking errors. Cross-session world-origin/calibration stability cannot be established from these files alone.", "",
             "No verified pre-action submerged fraction/volume was found. Nominal 25.5 mm initial embed depth and bed height are retained as metadata; they are not converted into evolving per-trial submergence.", "",
             "## Density", "",
             "Recorded masses are empty 12 g, resin 43 g, steel 150 g, lead 215 g from `/home/parnia/Downloads/density_experiment_log.xlsx`, `Lookup_Tables!A1:C5`. The user supplied a base radius of 25.4 mm and height of 31.75 mm, replacing the older notebook's 30 mm hemisphere description. Effective obstacle density means recorded mass divided by total external volume; initial embedded depth is not used in this denominator. Reported dimensions and STL dimensions are retained separately when they differ.", "",
             f"Density mode: **{manifest['density_mode']}**; external volume: **{manifest['external_volume_cm3']} cm³**. Supported choices are a checked STL outer envelope, spherical cap V=πh(3r²+h²)/6, half ellipsoid V=2πr²h/3, or a supplied external volume. No profile is assumed from dimensions alone.", "",
             "## Coverage", "", "| Material | Rows | Exported starts | Exported references | Exported ends |", "|---|---:|---:|---:|---:|"]
    for kind in DEFAULT_GROUPS:
        group = [r for r in rows if r["obstacle_material"] == kind]
        totals = [sum(r.get(f"{event}_yprime_mm") is not None for r in group)
                  for event in ("start", "reference", "end")]
        lines.append(f"| {kind} | {len(group)} | {totals[0]} | {totals[1]} | {totals[2]} |")
    lines += ["", "Event status counts:", "", "```json", json.dumps(manifest["event_status_counts"], indent=2), "```", "",
              "Trials with missing start or end:", "", "| Session | Trial | Start | End |", "|---|---:|---|---|"]
    for r in rows:
        if r.get("start_yprime_mm") is None or r.get("end_yprime_mm") is None:
            lines.append(f"| {r['session_id']} | {r['trial']} | {r.get('start_status')} | {r.get('end_status')} |")
    geometry = manifest.get("stl_geometry")
    if geometry:
        lines += ["", "## STL density evidence", "",
                  f"Source: `{geometry['source_stl']}`; SHA-256 `{geometry['source_stl_sha256']}`. Scale is explicitly {geometry['mm_per_stl_unit']} mm per STL coordinate unit; the diameter agrees with the reported 50.8 mm. The mesh has {geometry['triangles']} nondegenerate faces, closed two-face edges, consistent winding, and one connected component. Full audit: `obstacle_geometry.json`.", "",
                  f"The CAD exterior height is **{geometry['external_height_mm']:.5f} mm**, including a {geometry['straight_rim_height_mm']:.5f} mm straight rim; shell thickness is approximately {geometry['nominal_shell_thickness_mm']:.5f} mm. Its outer envelope is **{geometry['external_volume_cm3']:.6f} cm³**, while printing material occupies only **{geometry['print_material_volume_cm3']:.6f} cm³**. Density uses the outer envelope, including the interior volume. Source outer-facet integration and the convex envelope agree within {geometry['outer_surface_volume_relative_difference'] * 100:.6f}%; this checks that the hull follows the dome exterior rather than introducing unsupported outer concavities.", "",
                  "The STL already includes shell thickness. Its 27.94 mm outside height does not resolve the previously reported 31.75 mm height. Densities are therefore explicitly **CAD-based effective densities**, with `density_geometry_status`/`qc_notes` preserving the unresolved physical-height discrepancy. No stretch, added base, or volume correction was invented. The protruding tracking markers are not represented by this dome STL.", "",
                  "| Obstacle | Recorded mass (g) | CAD-based density (g/cm³) |", "|---|---:|---:|"]
        for kind, mass in MASSES_G.items():
            lines.append(f"| {kind} | {mass:g} | {mass / geometry['external_volume_cm3']:.6f} |")
    lines += ["", "All finite recorded checkpoint values are retained; mocap QC does not remove them. Inspect `mocap_qc_flag`, `mocap_qc_notes`, event statuses and `qc_notes` when optionally filtering data for plotting. Source-log notes are incorporated into the compact QC notes and preserved verbatim in the audit. Within-trial displacements are end minus start, reference minus start, or end minus reference; do not bridge missing trials or reset the origin.", ""]
    (output / "extraction_notes.md").write_text("\n".join(lines))


def write_readme(output: Path, rows: list[dict], manifest: dict):
    """Write a portable data dictionary beside the CSV; no source files needed."""
    flagged = sum(r.get("mocap_qc_flag") == "suspect" for r in rows)
    counts = {event: sum(r.get(f"{event}_yprime_mm") is not None for r in rows)
              for event in ("start", "reference", "end")}
    lines = [
        "# trial_position_summary.csv — obstacle positions", "",
        "Copy **trial_position_summary.csv** and this **README.md** together to your SSD/laptop. "
        "They are sufficient for position/displacement plotting; recordings and the audit CSV are not required.", "",
        f"This export contains **{len(rows)} rows from {manifest['sessions']} sessions**, one row per session and trial. "
        f"Exported checkpoints: start {counts['start']}, reference {counts['reference']}, end {counts['end']}. "
        f"**{flagged} rows have suspect mocap at one or more checkpoints; their recorded values are included.**", "",
        "## Retention policy", "",
        "This version keeps finite positions exactly as recorded at the selected checkpoints, after the documented "
        "coordinate rotation. Repeated/frozen-looking positions, zero positions, invalid orientations and position jumps "
        "are flagged rather than deleted. No interpolation, estimated replacement, or next-trial substitution is used. "
        "A zero displacement can therefore mean identical stored start/end positions; it does not establish that the "
        "physical obstacle was stationary.", "",
        "Blank numeric cells mean the recording/event/numeric position was unavailable or the event requirements "
        "could not be established. They are not zero. Reference angles must still match within 5° during the working "
        "sweep, and an endpoint must still follow the complete dwell. No recorded value is removed merely because "
        "the heuristic suspects poor mocap quality.", "",
        "## Coordinates and checkpoints", "",
        "All positions are **millimetres** in the slope frame after the **23° inclination rotation**. "
        "Positive **y′ points into the bed**, positive **z′ points downslope**. The original Motive world origin is "
        "preserved: these are signed absolute positions, not trial-zeroed displacement or depth relative to the sand surface. "
        "The point is the recorded rigid-body pivot, without a centre-of-mass offset.", "",
        "For raw world coordinates y,z in metres and θ=23°: `y′ = -1000*(cos(θ)*y + sin(θ)*z)`; "
        "`z′ = 1000*(-sin(θ)*y + cos(θ)*z)`. Do not rotate these CSV columns again.", "",
        "- **start:** last finite recorded sample immediately before the measured working sweep, within 50 ms. "
        "This follows positioning/insertion; it is not necessarily the first sample in the recording.",
        "- **reference:** sample minimizing the largest of the two joint-angle errors within the working sweep; "
        "both errors must be ≤5° and the matching episode unambiguous. This is not the temporal midpoint.",
        "- **end:** last finite same-trial raw sample after trajectory completion plus the full dwell (3 s in these recordings). "
        "Motor hold and timing are checked; concerns are flagged without deleting the recorded post-dwell position. Raw mocap may continue after the final video frame. "
        "An unchanged/frozen-looking last position is included and flagged.", "",
        "## Column dictionary", "",
        "In the following rows, `{event}` means each of `start`, `reference`, `end`, and `{joint}` means "
        "each of `adduction`, `sweeping`. The CSV contains 30 columns.", "",
        "| Column(s) | Meaning |", "|---|---|",
        "| `session_id` | Recording session identifier; identifies a repeat experiment. |",
        "| `obstacle_material` | empty, resin, steel, or lead. |",
        "| `obstacle_density`, `density_units` | CAD-based effective obstacle density; units are g/cm^3. |",
        "| `trial` | Stroke/trial number within the session, 1–7 for this export. |",
        "| `{event}_yprime_mm`, `{event}_zprime_mm` | Signed absolute obstacle coordinates at that checkpoint, in mm. |",
        "| `{event}_time_s` | Recorded host timestamp in seconds relative to this trial's recording start; not wall-clock time or time since sweeping starts. |",
        "| `{event}_mocap_sample_index` | Zero-based index in this trial's original raw mocap array. Index 181 is its 182nd sample; this is not a video frame index. |",
        "| `reference_target_{joint}_deg` | Requested joint angle, in degrees. The same target is repeated on every row. |",
        "| `reference_actual_{joint}_deg` | Recorded motor feedback nearest the matched reference timestamp, in degrees. |",
        "| `reference_max_abs_error_deg` | Larger absolute difference between actual and target over the two joints. |",
        "| `start_status`, `reference_status`, `end_status` | `ok` means the event was identified and a numeric position exported. It does not certify tracking quality; consult the separate mocap columns. `ok_with_sync_warning` and `ok_with_motor_hold_warning` retain the position but warn about motor alignment/hold. Other values explain an unavailable checkpoint. |",
        "| `extraction_status` | `complete_with_qc` / `complete`: all three events exported; `partial`: at least one unavailable; `missing_trial` / `error`: source unavailable or extraction failed. General acquisition/geometry notes can produce complete_with_qc without suspect mocap. |",
        "| `source_recording` | Original recording path for provenance only; it need not exist on your laptop to plot this CSV. |",
        "| `mocap_qc_flag` | `suspect`: one or more selected checkpoints have a mocap/timing concern; `not_flagged`: no listed checkpoint concern detected; `unavailable`: mocap QC could not be assessed. |",
        "| `mocap_qc_notes` | Specific checkpoint and concern, e.g. `end: unchanged_xyz_for_4.064s ...`. Empty when no checkpoint concern was detected. |",
        "| `qc_notes` | General trial/density/source notes, including anomalies elsewhere in the trial. These are not an automatic exclusion rule. |", "",
        "## Interpreting mocap flags", "",
        "- `unchanged_xyz_for_...s`: all three stored coordinates repeat exactly for longer than 0.25 s at the checkpoint. "
        "This is a conservative heuristic for possibly held tracking; the threshold is not independently validated.",
        "- `position_jump_over_5mm_within_50ms`: a consecutive position step exceeds 5 mm within ±50 ms of the checkpoint. "
        "It could represent real motion or tracking error.",
        "- `isolated_position_spike`: a >5 mm excursion immediately returns close to the preceding position.",
        "- `all_zero_xyz` / `invalid_quaternion`: zero raw position or nonfinite or zero-norm orientation at the selected sample. "
        "The finite recorded position remains included.",
        "- `motor_sync_gap_...s` / `returned_motor_hold_not_verified`: the position is retained, but nearby motor feedback is too far away in time or does not verify the returned hold. The post-dwell timestamp still comes from the recorded completion/dwell metadata.",
        "- Messages about nonfinite/missing positions explain genuinely unavailable numeric values or a short endpoint fallback.", "",
        "The recordings do not contain a Motive tracking-valid flag. **Suspect does not mean confirmed tracking loss, "
        "and not_flagged does not prove accuracy.** The event name in each note lets you distinguish a reference-only "
        "concern from concerns at the start/end used for net displacement. Filtering on the row flag is conservative.", "",
        "## Calculating displacement and choosing rows", "",
        "Within each row: `delta_y_mm = end_yprime_mm - start_yprime_mm`; "
        "`delta_z_mm = end_zprime_mm - start_zprime_mm`. Use `reference - start` for the earlier portion if desired.", "",
        "Use all numeric pairs to include everything recorded. Optionally retain only rows with "
        "`mocap_qc_flag == 'not_flagged'` to remove suspect checkpoints. Do not filter on `extraction_status == 'complete'` "
        "alone because ordinary source/geometry notes are also QC notes.", "",
        "```python", "import csv", "",
        "with open('trial_position_summary.csv', newline='') as f:",
        "    rows = list(csv.DictReader(f))",
        "# Optional filter: rows = [r for r in rows if r['mocap_qc_flag'] == 'not_flagged']",
        "for r in rows:",
        "    if all(r[k] != '' for k in ('start_yprime_mm', 'end_yprime_mm', 'start_zprime_mm', 'end_zprime_mm')):",
        "        dy_mm = float(r['end_yprime_mm']) - float(r['start_yprime_mm'])",
        "        dz_mm = float(r['end_zprime_mm']) - float(r['start_zprime_mm'])", "```", "",
        "For comparison with the older net-displacement bars: sum the seven within-trial displacements for each "
        "session, then average session totals within material. Convert y′ to the old upward-positive centimetres "
        "by multiplying the sum by **−0.1**; convert z′ to downslope-positive centimetres with **+0.1**. "
        "A per-trial mean is a different quantity. Require all seven numeric pairs for a complete session total; "
        "never fill missing values with zero. The new checks include suspect values rather than filtering them.", "",
        "## Angles, density, and remaining limitations", "",
        f"Reference targets: adduction **{math.degrees(manifest['reference_adduction_rad']):.9f}°**, "
        f"sweeping **{math.degrees(manifest['reference_sweeping_rad']):.9f}°**." if manifest['reference_adduction_rad'] is not None else "Reference targets are not configured.",
        "The target pair comes from the user-selected PIV matched-frame summary. Actual angles and errors are preserved; "
        "a reference within 5° is not an assertion that the exact target was reached.", "",
        "Effective density = recorded mass / external obstacle volume, including the hollow interior. "
        "Masses: empty 12 g, resin 43 g, steel 150 g, lead 215 g. "
        f"External volume for this export: **{manifest['external_volume_cm3']} cm³**; mode: `{manifest['density_mode']}`.",
        "For the supplied STL, the outer diameter is 50.8 mm and the outer height is 27.94 mm. "
        "The reported physical height was 31.75 mm, so the physical-volume discrepancy remains unresolved. "
        "Density values are CAD-based; no assumed height correction was applied. No reliable per-trial submerged "
        "fraction or submerged volume was available.", "",
        "Motor/mocap alignment uses host receipt timestamps, not hardware exposure times. "
        "Numerical verification checks indexed samples and transformations, not independent physical calibration.", "",
        "## Optional files", "",
        "`trial_position_audit.csv`, `extraction_manifest.json`, `extraction_notes.md`, and `sample_verification.csv` "
        "provide detailed provenance and verification; they are not required for plotting the compact CSV. "
        "`csv_validation_plots/` contains PNG/PDF checks and their derived numeric tables. "
        "Earlier notes/contact sheets in `recording_checks/` describe the previous filtered export; the present CSV "
        "and this README define the current retention policy.", "",
        "Extractor: `highlevel/terrain_manipulation/src/utils/extract_density_obstacle_positions.py`. "
        "Verification and CSV-only plotting tools are under `highlevel/terrain_manipulation/src/utils/tests/`.", "",
    ]
    (output / "README.md").write_text("\n".join(lines))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--log-csv", type=Path, default=Path("/home/parnia/Downloads/density_experiment_log - Run_Log_14x33_init.csv"))
    parser.add_argument("--data-root", type=Path, default=Path("/media/parnia/Extreme SSD/density experiments backup"))
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--session", action="append", help="Repeat to override the 21-session selection; each must occur in the log")
    parser.add_argument("--trials", help="Explicit override of CSV Trial subset, e.g. 1-7")
    parser.add_argument("--reference", choices=("pending", "piv", "march"), default="pending",
                        help="piv: .6977335/-.5296018 rad; march: .785/-.5296018 rad; pending leaves reference blank")
    parser.add_argument("--reference-adduction-rad", type=float)
    parser.add_argument("--reference-sweeping-rad", type=float)
    parser.add_argument("--reference-summary-csv", type=Path,
                        help="Read the unique target_adduction_rad/target_sweeping_rad pair from a PIV summary")
    parser.add_argument("--density-mode", choices=("unavailable", "spherical-cap", "half-ellipsoid", "volume", "stl-envelope"), default="unavailable")
    parser.add_argument("--base-radius-mm", type=float, default=25.4)
    parser.add_argument("--dome-height-mm", type=float, default=31.75)
    parser.add_argument("--external-volume-cm3", type=float, help="Known CAD/displaced external volume; requires --density-mode volume")
    parser.add_argument("--obstacle-stl", type=Path, help="Hemisphere/rim shell STL for --density-mode stl-envelope")
    parser.add_argument("--stl-mm-per-unit", type=float, help="Explicit STL length scale; STL itself stores no units")
    parser.add_argument("--cache-dir", type=Path, help="Optional small cache of raw signals, without RGB-D")
    parser.add_argument("--review", action="store_true", help="Save camera contact sheets for trials 1 and 7 of the first session per material, plus noted resin trials 5 and 7")
    parser.add_argument("--max-sync-s", type=float, default=0.05)
    parser.add_argument("--angle-tolerance-deg", type=float, default=5.0,
                        help="Maximum absolute error allowed in each reference joint (default: 5 degrees)")
    parser.add_argument("--onset-velocity-rad-s", type=float, default=0.2)
    args = parser.parse_args()
    cfg = Config(max_sync_s=args.max_sync_s, angle_tolerance_deg=args.angle_tolerance_deg,
                 onset_velocity_rad_s=args.onset_velocity_rad_s)
    if any(not math.isfinite(v) or v <= 0 for v in asdict(cfg).values()):
        parser.error("QC thresholds must be finite and positive")
    if any(not math.isfinite(v) or v <= 0 for v in (args.base_radius_mm, args.dome_height_mm)):
        parser.error("Dome dimensions must be finite and positive")
    volume_cm3 = None
    geometry = None
    geometry_status = ""
    radius, height = args.base_radius_mm / 10, args.dome_height_mm / 10
    if args.density_mode == "spherical-cap":
        volume_cm3 = math.pi * height * (3 * radius**2 + height**2) / 6
    elif args.density_mode == "half-ellipsoid":
        volume_cm3 = 2 * math.pi * radius**2 * height / 3
    elif args.density_mode == "volume":
        volume_cm3 = args.external_volume_cm3
        if volume_cm3 is None or not math.isfinite(volume_cm3) or volume_cm3 <= 0:
            parser.error("Provide a positive finite --external-volume-cm3")
    elif args.density_mode == "stl-envelope":
        if args.obstacle_stl is None or args.stl_mm_per_unit is None:
            parser.error("STL density requires --obstacle-stl and --stl-mm-per-unit")
        geometry = inspect_stl_envelope(args.obstacle_stl, args.stl_mm_per_unit)
        if abs(geometry["external_base_radius_mm"] - args.base_radius_mm) > 0.01:
            parser.error("Scaled STL radius disagrees with --base-radius-mm; verify units/dimensions")
        volume_cm3 = geometry["external_volume_cm3"]
        geometry_status = "CAD-based; physical print dimensions not independently measured"
        if abs(geometry["external_height_mm"] - args.dome_height_mm) > 0.01:
            geometry_status += f"; height mismatch: STL {geometry['external_height_mm']:.5f} mm vs reported {args.dome_height_mm:g} mm"
    if args.external_volume_cm3 is not None and args.density_mode != "volume":
        parser.error("--external-volume-cm3 requires --density-mode volume")
    if args.density_mode != "stl-envelope" and (args.obstacle_stl is not None or args.stl_mm_per_unit is not None):
        parser.error("STL options require --density-mode stl-envelope")
    if (args.reference_adduction_rad is None) != (args.reference_sweeping_rad is None):
        parser.error("Provide both custom reference angles")
    target_ad = args.reference_adduction_rad
    target_sw = args.reference_sweeping_rad
    if target_ad is None and args.reference != "pending":
        target_ad = PIV_ADDUCTION_RAD if args.reference == "piv" else 0.785
        target_sw = PIV_SWEEPING_RAD
    if args.reference_summary_csv:
        if args.reference_adduction_rad is not None or args.reference != "pending":
            parser.error("Use --reference-summary-csv alone to select reference angles")
        with args.reference_summary_csv.open(newline="", encoding="utf-8-sig") as f:
            targets = {(float(r["target_adduction_rad"]), float(r["target_sweeping_rad"]))
                       for r in csv.DictReader(f)}
        if len(targets) != 1:
            parser.error("Reference summary must contain exactly one common target pair")
        target_ad, target_sw = next(iter(targets))
    if target_ad is not None and not all(math.isfinite(v) for v in (target_ad, target_sw)):
        parser.error("Reference angles must be finite")
    with args.log_csv.open(newline="", encoding="utf-8-sig") as f:
        log = list(csv.DictReader(f))
    selected = [(kind, f"session_{s}") for kind, group in DEFAULT_GROUPS.items() for s in group]
    if args.session:
        selected = []
        for session in args.session:
            match = [r for r in log if r.get("Session", "").strip() == session]
            if len(match) != 1 or match[0].get("Object_ID") not in OBJECT_IDS.values():
                parser.error(f"{session}: needs one log row with known Object_ID")
            kind = next(k for k, v in OBJECT_IDS.items() if v == match[0]["Object_ID"])
            selected.append((kind, session))
    if len({s for _, s in selected}) != len(selected):
        parser.error("Duplicate selected sessions")
    args.output_dir.mkdir(parents=True, exist_ok=True)
    rows = []
    for kind, session in selected:
        matching = [(i + 2, r) for i, r in enumerate(log) if r.get("Session", "").strip() == session]
        if len(matching) > 1:
            parser.error(f"Duplicate session in input CSV: {session}")
        line, logrow = matching[0] if matching else (None, {})
        trials = parse_trials(args.trials if args.trials is not None else logrow.get("Trial", ""))
        session_dir = args.data_root / session
        metadata_path = session_dir / "metadata.json"
        session_meta = json.loads(metadata_path.read_text()) if metadata_path.is_file() else {}
        for trial in trials:
            path = session_dir / f"trial_{trial}.npy"
            rb_id = int(number(logrow.get("Mocap_RB_ID"), RB_IDS[kind]))
            row = {"session_id": session, "obstacle_material": kind,
                   "obstacle_density": None, "density_units": None,
                   "density_definition": "not recorded", "trial": trial,
                   "object_id": logrow.get("Object_ID", OBJECT_IDS[kind]),
                   "session_repeat_number": logrow.get("Trial_Number", ""),
                   "obstacle_mass_g": MASSES_G[kind], "mass_source": MASS_SOURCE,
                   "obstacle_base_radius_mm": args.base_radius_mm,
                   "obstacle_dome_height_mm": args.dome_height_mm,
                   "position_units": "mm", "angle_units": "rad", "timestamp_units": "s",
                   "source_recording": str(path.resolve()),
                   "source_session_metadata": str(metadata_path.resolve()),
                   "source_log_csv": str(args.log_csv.resolve()), "source_log_row": line,
                   "reference_configuration_source": str(args.reference_summary_csv.resolve()) if args.reference_summary_csv else ("custom angles" if args.reference_adduction_rad is not None else args.reference),
                   "reference_target_adduction_rad": target_ad,
                   "reference_target_sweeping_rad": target_sw,
                   "source_log_trial_subset": logrow.get("Trial", ""),
                   "source_log_notes": " | ".join(str(logrow.get(k, "")) for k in ("Notes_Placement", "General_Notes") if logrow.get(k)),
                   "nominal_embed_depth_mm": number(logrow.get("Embed_Depth_mm")),
                   "bed_height_mm": number(logrow.get("Bed_Height_mm"))}
            if volume_cm3 is not None:
                row.update(obstacle_density=MASSES_G[kind] / volume_cm3, density_units="g/cm^3",
                           density_definition=f"effective mass / external volume; profile={args.density_mode}",
                           external_volume_cm3=volume_cm3)
            if geometry:
                row.update(density_definition="CAD-based effective density: recorded mass / STL outer envelope including interior",
                           density_geometry_status=geometry_status,
                           density_volume_source=geometry["source_stl"], density_volume_source_sha256=geometry["source_stl_sha256"],
                           stl_mm_per_unit=geometry["mm_per_stl_unit"],
                           stl_outer_height_mm=geometry["external_height_mm"],
                           stl_outer_base_radius_mm=geometry["external_base_radius_mm"],
                           stl_print_material_volume_cm3=geometry["print_material_volume_cm3"])
            first = session == f"session_{DEFAULT_GROUPS[kind][0]}"
            review = args.review and ((first and trial in (1, 7)) or (session in ("session_20260319_145057", "session_20260319_145955") and trial in (5, 7)))
            if not path.is_file():
                row.update(blank_events(), extraction_status="missing_trial", qc_notes="requested_recording_missing")
            else:
                try:
                    stat = path.stat()
                    row.update(source_recording_bytes=stat.st_size, source_recording_mtime_ns=stat.st_mtime_ns)
                    payload = load_payload(path, args.cache_dir, need_rgb=review)
                    meta = payload.get("metadata", {})
                    incline = number(logrow.get("Incline_deg"), number(meta.get("mocap_incline_deg"), number(session_meta.get("mocap_incline_deg"))))
                    if not math.isfinite(incline):
                        raise ValueError("Missing slope calibration in log and metadata")
                    row.update(extract_trial(payload, rb_id, incline, target_ad, target_sw, cfg))
                    if matching and row["object_id"] != OBJECT_IDS[kind]:
                        row["qc_notes"] += "; log_object_id_disagrees_with_requested_material"
                    if not matching:
                        row["qc_notes"] += "; session_missing_from_input_log"
                    if review:
                        save_review(payload, row, args.output_dir / "recording_checks")
                    del payload
                except (OSError, ValueError, KeyError, TypeError) as exc:
                    row.update(blank_events(), extraction_status="error", qc_notes=f"{type(exc).__name__}: {exc}")
            if args.density_mode == "unavailable":
                row["qc_notes"] += "; numeric_obstacle_density_unavailable"
            if geometry:
                row["qc_notes"] += "; " + geometry_status
            rows.append(row)
            print(f"{session} trial {trial}: {row['extraction_status']} ({row['start_status']}, {row['reference_status']}, {row['end_status']})", flush=True)
        write_csv(args.output_dir / "trial_position_audit.csv", rows)
        write_csv(args.output_dir / "trial_position_summary.csv", [summary_row(row) for row in rows])
    counts = {e: dict(Counter(r[f"{e}_status"] for r in rows)) for e in ("start", "reference", "end")}
    manifest = {"command": sys.argv, "config": asdict(cfg), "reference_adduction_rad": target_ad,
                "reference_sweeping_rad": target_sw, "density_mode": args.density_mode,
                "external_volume_cm3": volume_cm3, "base_radius_mm": args.base_radius_mm,
                "dome_height_mm": args.dome_height_mm,
                "sessions": len(selected), "rows": len(rows), "event_status_counts": counts,
                "summary_columns": list(summary_row(rows[0])) if rows else [],
                "audit_file": "trial_position_audit.csv",
                "log_sha256": hashlib.sha256(args.log_csv.read_bytes()).hexdigest(),
                "script_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
    manifest["mocap_policy"] = "retain finite recorded checkpoint positions; heuristic QC flags do not remove values"
    manifest["mocap_qc_counts"] = dict(Counter(r.get("mocap_qc_flag", "unavailable") for r in rows))
    if args.reference_summary_csv:
        manifest["reference_summary_path"] = str(args.reference_summary_csv.resolve())
        manifest["reference_summary_sha256"] = hashlib.sha256(args.reference_summary_csv.read_bytes()).hexdigest()
    if geometry:
        manifest["stl_geometry"] = geometry
        manifest["density_geometry_status"] = geometry_status
        (args.output_dir / "obstacle_geometry.json").write_text(json.dumps(geometry, indent=2) + "\n")
    (args.output_dir / "extraction_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    write_notes(args.output_dir, rows, manifest)
    write_readme(args.output_dir, rows, manifest)
    print(json.dumps(counts, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
