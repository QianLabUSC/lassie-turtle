#!/usr/bin/env python3
"""
Record a timestamped RealSense sand reference for a new camera setup.

Run this on the lab PC after the camera pose is set, the bed is flattened, and
the robot/tool is removed or raised out of the sand view. The script saves a
reference package under output/references/reference_<time>/ without deleting
older references.
"""

import argparse
import json
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import cv2
import numpy as np
import pyrealsense2 as rs


@dataclass
class StreamCfg:
    w: int
    h: int
    fps: int


@dataclass
class Intrinsics:
    width: int
    height: int
    fx: float
    fy: float
    ppx: float
    ppy: float


DEFAULT_DEPTH = StreamCfg(848, 480, 30)
DEFAULT_RS_CONFIG = Path(__file__).with_name("rs_config_gui.json")
DEFAULT_OUTPUT_ROOT = Path(__file__).with_name("output") / "references"


def _load_gui_config(config_path: Path) -> Dict[str, Any]:
    with open(config_path, "r", encoding="utf-8") as fh:
        return json.load(fh)


def _stream_cfg_from_gui_config(gui_config: Dict[str, Any], fallback: StreamCfg) -> StreamCfg:
    viewer = gui_config.get("viewer", {})
    return StreamCfg(
        int(viewer.get("stream-width", fallback.w)),
        int(viewer.get("stream-height", fallback.h)),
        int(viewer.get("stream-fps", fallback.fps)),
    )


def _apply_gui_config(device: rs.device, config_path: Path) -> None:
    if not config_path.exists():
        raise FileNotFoundError(f"RealSense GUI config not found: {config_path}")

    raw_config = config_path.read_text(encoding="utf-8")
    try:
        advanced = rs.rs400_advanced_mode(device)
    except Exception as exc:
        print(f"[WARN] Device does not expose D400 advanced mode; GUI config not loaded: {exc}")
        return

    try:
        if not advanced.is_enabled():
            print("[WARN] D400 advanced mode is disabled; GUI JSON was not loaded.")
            print("[WARN] Open Intel RealSense Viewer once, enable Advanced Mode, then rerun this script.")
            return
        advanced.load_json(raw_config)
        print(f"Loaded RealSense GUI config: {config_path}")
    except Exception as exc:
        print(f"[WARN] Failed to load RealSense GUI config {config_path}: {exc}")


def _try_set(opt_owner, option, value) -> bool:
    try:
        opt_owner.set_option(option, value)
        return True
    except Exception as exc:
        print(f"[WARN] Could not set {option} to {value}: {exc}")
        return False


def _get_depth_intrinsics(profile: rs.pipeline_profile) -> Intrinsics:
    stream_profile = profile.get_stream(rs.stream.depth).as_video_stream_profile()
    intr = stream_profile.get_intrinsics()
    return Intrinsics(
        width=intr.width,
        height=intr.height,
        fx=intr.fx,
        fy=intr.fy,
        ppx=intr.ppx,
        ppy=intr.ppy,
    )


def _get_color_intrinsics(profile: rs.pipeline_profile) -> Intrinsics:
    stream_profile = profile.get_stream(rs.stream.color).as_video_stream_profile()
    intr = stream_profile.get_intrinsics()
    return Intrinsics(
        width=intr.width,
        height=intr.height,
        fx=intr.fx,
        fy=intr.fy,
        ppx=intr.ppx,
        ppy=intr.ppy,
    )


def _pixel_rays(intr: Intrinsics) -> Tuple[np.ndarray, np.ndarray]:
    v, u = np.indices((intr.height, intr.width), dtype=np.float32)
    x_coeff = (u - intr.ppx) / intr.fx
    y_coeff = (v - intr.ppy) / intr.fy
    return x_coeff, y_coeff


def _points_from_depth(depth_raw: np.ndarray, depth_scale: float, x_coeff: np.ndarray, y_coeff: np.ndarray):
    z = depth_raw.astype(np.float32) * depth_scale
    x = x_coeff * z
    y = y_coeff * z
    return x, y, z


def _fit_mask(
    depth_raw: np.ndarray,
    *,
    top_crop: float,
    bottom_crop: float,
    side_crop: float,
    min_depth_m: float,
    max_depth_m: float,
    depth_scale: float,
) -> np.ndarray:
    h, w = depth_raw.shape
    depth_m = depth_raw.astype(np.float32) * depth_scale
    mask = (depth_raw > 0) & (depth_m >= min_depth_m) & (depth_m <= max_depth_m)
    mask[: int(h * top_crop), :] = False
    mask[int(h * bottom_crop) :, :] = False
    mask[:, : int(w * side_crop)] = False
    mask[:, int(w * (1.0 - side_crop)) :] = False
    return mask


def _fit_plane_ransac(
    x: np.ndarray,
    y: np.ndarray,
    z: np.ndarray,
    mask: np.ndarray,
    *,
    max_points: int,
    iterations: int,
    threshold_m: float,
    seed: int = 7,
) -> np.ndarray:
    points = np.column_stack([x[mask], y[mask], z[mask]])
    if len(points) < 1000:
        raise RuntimeError(f"Not enough valid sand points to fit plane: {len(points)}")

    rng = np.random.default_rng(seed)
    if len(points) > max_points:
        points = points[rng.choice(len(points), size=max_points, replace=False)]

    best_inliers: Optional[np.ndarray] = None
    best_count = -1
    for _ in range(iterations):
        sample = points[rng.choice(len(points), size=3, replace=False)]
        normal = np.cross(sample[1] - sample[0], sample[2] - sample[0])
        norm = np.linalg.norm(normal)
        if norm < 1e-9:
            continue
        normal = normal / norm
        d = -float(normal @ sample[0])
        residuals = np.abs(points @ normal + d)
        inliers = residuals < threshold_m
        count = int(inliers.sum())
        if count > best_count:
            best_count = count
            best_inliers = inliers

    if best_inliers is None or best_count < 1000:
        raise RuntimeError("RANSAC could not find a stable sand plane.")

    inlier_points = points[best_inliers]
    centroid = inlier_points.mean(axis=0)
    _, _, vh = np.linalg.svd(inlier_points - centroid, full_matrices=False)
    normal = vh[-1]
    normal = normal / np.linalg.norm(normal)
    if normal[2] < 0:
        normal = -normal
    d = -float(normal @ centroid)
    return np.array([normal[0], normal[1], normal[2], d], dtype=np.float64)


def _signed_plane_distance_mm(x: np.ndarray, y: np.ndarray, z: np.ndarray, plane: np.ndarray) -> np.ndarray:
    a, b, c, d = plane
    dist_m = (a * x + b * y + c * z + d) / np.linalg.norm(plane[:3])
    return dist_m * 1000.0


def _raw_depth_to_bgr(depth_raw: np.ndarray) -> np.ndarray:
    valid = depth_raw > 0
    if np.any(valid):
        low = np.percentile(depth_raw[valid], 2)
        high = np.percentile(depth_raw[valid], 98)
    else:
        low, high = 0.0, 1.0
    if high <= low:
        high = low + 1.0
    depth_u8 = np.clip((depth_raw.astype(np.float32) - low) * 255.0 / (high - low), 0, 255).astype(np.uint8)
    depth_u8[~valid] = 0
    return cv2.applyColorMap(depth_u8, cv2.COLORMAP_JET)


def _height_to_bgr(height_mm: np.ndarray, valid_mask: np.ndarray, range_mm: float) -> np.ndarray:
    norm = np.clip((height_mm + range_mm) / (2.0 * range_mm), 0.0, 1.0)
    bgr = np.zeros((*height_mm.shape, 3), dtype=np.uint8)

    low = norm < 0.5
    high = ~low
    t_low = norm[low] / 0.5
    bgr[low, 0] = 255
    bgr[low, 1] = np.clip(255 * t_low, 0, 255).astype(np.uint8)
    bgr[low, 2] = np.clip(255 * t_low, 0, 255).astype(np.uint8)

    t_high = (norm[high] - 0.5) / 0.5
    bgr[high, 0] = np.clip(255 * (1.0 - t_high), 0, 255).astype(np.uint8)
    bgr[high, 1] = np.clip(255 * (1.0 - t_high), 0, 255).astype(np.uint8)
    bgr[high, 2] = 255

    bgr[~valid_mask] = (0, 0, 0)
    return bgr


def _draw_label(image: np.ndarray, text: str) -> None:
    cv2.putText(image, text, (16, 36), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 4, cv2.LINE_AA)
    cv2.putText(image, text, (16, 36), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2, cv2.LINE_AA)


def _reference_name(now: datetime) -> str:
    return "reference_" + now.strftime("%Y%m%d_%H%M%S")


def _safe_name(value: str) -> str:
    return "".join(ch if ch.isalnum() or ch in ("-", "_") else "_" for ch in value)


def _get_realsense_serials() -> List[str]:
    ctx = rs.context()
    devices = ctx.query_devices()
    serials: List[str] = []
    for dev in devices:
        try:
            serials.append(dev.get_info(rs.camera_info.serial_number))
        except Exception:
            continue
    return sorted(serials)


def _record_one_camera(
    *,
    serial: str,
    camera_index: int,
    camera_count: int,
    session_dir: Path,
    reference_name: str,
    created_at: datetime,
    args: argparse.Namespace,
    gui_config: Optional[Dict[str, Any]],
    stream_cfg: StreamCfg,
) -> Dict[str, Any]:
    camera_name = _safe_name(serial)
    output_dir = session_dir / camera_name
    output_dir.mkdir(parents=True)

    pipeline = rs.pipeline()
    cfg = rs.config()
    cfg.enable_device(serial)
    cfg.enable_stream(rs.stream.depth, stream_cfg.w, stream_cfg.h, rs.format.z16, stream_cfg.fps)
    cfg.enable_stream(rs.stream.color, stream_cfg.w, stream_cfg.h, rs.format.bgr8, stream_cfg.fps)

    profile = pipeline.start(cfg)
    intr = _get_color_intrinsics(profile)
    x_coeff, y_coeff = _pixel_rays(intr)

    dev = profile.get_device()
    if gui_config is not None:
        _apply_gui_config(dev, args.rs_config)
    actual_serial = dev.get_info(rs.camera_info.serial_number)
    depth_sensor = dev.first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()
    spatial = rs.spatial_filter()
    temporal = rs.temporal_filter()
    hole = rs.hole_filling_filter()
    align = rs.align(rs.stream.color)

    print()
    print(f"[{camera_index + 1}/{camera_count}] Recording reference for camera {actual_serial}")
    print("Assumes robot/tool is removed or raised and sand is flat.")
    print(f"Output: {output_dir}")
    print(f"Depth stream: {stream_cfg.w}x{stream_cfg.h}@{stream_cfg.fps}, aligned to color")
    print(f"Depth scale: {depth_scale}")
    print(f"Discarding {args.warmup_frames} warmup frames, then recording {args.frames} frames...")

    frames = []
    win = f"Record RealSense Reference {actual_serial}"
    if args.preview:
        cv2.namedWindow(win, cv2.WINDOW_NORMAL)

    try:
        for i in range(args.warmup_frames + args.frames):
            rs_frames = pipeline.wait_for_frames()
            rs_frames = align.process(rs_frames)
            depth = rs_frames.get_depth_frame()
            if not depth:
                continue

            if args.post:
                depth = spatial.process(depth)
                depth = temporal.process(depth)
                depth = hole.process(depth)

            depth_raw = np.asanyarray(depth.get_data()).copy()
            if i >= args.warmup_frames:
                frames.append(depth_raw)

            if args.preview:
                preview = _raw_depth_to_bgr(depth_raw)
                phase = "warmup" if i < args.warmup_frames else "recording"
                count = 0 if i < args.warmup_frames else len(frames)
                _draw_label(preview, f"{phase} {count}/{args.frames}")
                cv2.imshow(win, preview)
                if cv2.waitKey(1) & 0xFF in (27, ord("q")):
                    raise KeyboardInterrupt
    finally:
        pipeline.stop()
        if args.preview:
            cv2.destroyAllWindows()

    if len(frames) != args.frames:
        raise SystemExit(f"Recorded {len(frames)} frames, expected {args.frames}. Reference was not completed.")

    frame_stack = np.stack(frames, axis=0)
    reference_depth = np.median(frame_stack, axis=0).astype(np.float32)
    x_ref, y_ref, z_ref = _points_from_depth(reference_depth, depth_scale, x_coeff, y_coeff)
    fit_mask = _fit_mask(
        reference_depth,
        top_crop=args.top_crop,
        bottom_crop=args.bottom_crop,
        side_crop=args.side_crop,
        min_depth_m=args.fit_min_depth,
        max_depth_m=args.fit_max_depth,
        depth_scale=depth_scale,
    )
    plane = _fit_plane_ransac(
        x_ref,
        y_ref,
        z_ref,
        fit_mask,
        max_points=args.ransac_max_points,
        iterations=args.ransac_iters,
        threshold_m=args.ransac_threshold_mm * 0.001,
    )
    reference_height_mm = _signed_plane_distance_mm(x_ref, y_ref, z_ref, plane)
    valid = reference_depth > 0
    reference_height_mm[~valid] = np.nan

    np.save(output_dir / "reference_depth_median_raw.npy", reference_depth)
    np.save(output_dir / "reference_height_plane_corrected_mm.npy", reference_height_mm)
    np.save(output_dir / "sand_plane_abcd.npy", plane)
    np.save(output_dir / "fit_mask.npy", fit_mask)
    if args.save_raw_frames:
        np.savez_compressed(output_dir / "reference_depth_frames_raw.npz", depth=frame_stack)

    raw_preview = _raw_depth_to_bgr(reference_depth.astype(np.uint16))
    corrected_preview = _height_to_bgr(reference_height_mm, valid, args.height_range_mm)
    _draw_label(raw_preview, "Median raw depth")
    _draw_label(corrected_preview, f"Plane-corrected height +/-{args.height_range_mm:.0f} mm")
    cv2.imwrite(str(output_dir / "reference_raw_depth_preview.png"), raw_preview)
    cv2.imwrite(str(output_dir / "reference_plane_corrected_preview.png"), corrected_preview)

    metadata = {
        "reference_name": reference_name,
        "created_at_local": created_at.isoformat(timespec="seconds"),
        "camera_index": camera_index,
        "camera_count": camera_count,
        "device_serial": actual_serial,
        "depth_scale_m_per_unit": depth_scale,
        "stream": asdict(stream_cfg),
        "intrinsics": asdict(intr),
        "frames_recorded": len(frames),
        "warmup_frames": args.warmup_frames,
        "postprocess_enabled": args.post,
        "rs_config": None if args.no_rs_config else str(args.rs_config),
        "fit_crop": {
            "top_crop": args.top_crop,
            "bottom_crop": args.bottom_crop,
            "side_crop": args.side_crop,
            "fit_min_depth_m": args.fit_min_depth,
            "fit_max_depth_m": args.fit_max_depth,
        },
        "ransac": {
            "iterations": args.ransac_iters,
            "threshold_mm": args.ransac_threshold_mm,
            "max_points": args.ransac_max_points,
            "fit_points": int(fit_mask.sum()),
        },
        "plane_abcd": plane.tolist(),
        "files": {
            "median_depth_raw": "reference_depth_median_raw.npy",
            "corrected_height_mm": "reference_height_plane_corrected_mm.npy",
            "sand_plane_abcd": "sand_plane_abcd.npy",
            "fit_mask": "fit_mask.npy",
            "raw_frames": "reference_depth_frames_raw.npz" if args.save_raw_frames else None,
            "raw_preview": "reference_raw_depth_preview.png",
            "corrected_preview": "reference_plane_corrected_preview.png",
        },
    }
    (output_dir / "metadata.json").write_text(json.dumps(metadata, indent=2), encoding="utf-8")
    latest_payload = {"latest_reference": str(output_dir), **metadata}
    latest_serial_path = args.output_root / f"latest_reference_{_safe_name(actual_serial)}.json"
    latest_serial_path.write_text(json.dumps(latest_payload, indent=2), encoding="utf-8")

    print(f"Saved reference package: {output_dir}")
    print(f"Plane [a, b, c, d]: {plane}")
    print(f"Fit points: {int(fit_mask.sum())}")
    print(f"Latest reference pointer for this device: {latest_serial_path}")
    return {
        "camera_name": camera_name,
        "device_serial": actual_serial,
        "reference_dir": str(output_dir),
        "metadata": "metadata.json",
        "latest_reference": str(latest_serial_path),
    }


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--serial",
        action="append",
        default=None,
        help="RealSense serial to record. Repeat for multiple cameras. Default: discover and record all connected cameras.",
    )
    ap.add_argument("--expected-cameras", type=int, default=2, help="Expected camera count when --serial is omitted.")
    ap.add_argument(
        "--rs-config",
        type=Path,
        default=DEFAULT_RS_CONFIG,
        help="Intel RealSense Viewer JSON config to load onto each device.",
    )
    ap.add_argument("--no-rs-config", action="store_true", help="Do not load the Intel RealSense Viewer JSON config.")
    ap.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT_ROOT)
    ap.add_argument("--name", type=str, default=None, help="Reference session folder name. Default: reference_YYYYMMDD_HHMMSS.")
    ap.add_argument("--frames", type=int, default=120, help="Depth frames to record for each camera reference.")
    ap.add_argument("--warmup-frames", type=int, default=30, help="Initial frames to discard before recording.")
    ap.add_argument("--preview", action="store_true", help="Show live raw-depth preview while recording.")
    ap.add_argument("--post", action="store_true", help="Enable spatial/temporal/hole-fill depth filters before recording.")
    ap.add_argument("--save-raw-frames", action="store_true", default=True, help="Save compressed raw reference frame stack.")
    ap.add_argument("--no-save-raw-frames", dest="save_raw_frames", action="store_false")
    ap.add_argument("--height-range-mm", type=float, default=80.0)
    ap.add_argument("--top-crop", type=float, default=0.30)
    ap.add_argument("--bottom-crop", type=float, default=0.98)
    ap.add_argument("--side-crop", type=float, default=0.06)
    ap.add_argument("--fit-min-depth", type=float, default=0.15)
    ap.add_argument("--fit-max-depth", type=float, default=3.0)
    ap.add_argument("--ransac-iters", type=int, default=250)
    ap.add_argument("--ransac-threshold-mm", type=float, default=8.0)
    ap.add_argument("--ransac-max-points", type=int, default=40000)
    args = ap.parse_args()

    now = datetime.now()
    reference_name = args.name or _reference_name(now)
    session_dir = args.output_root / reference_name
    if session_dir.exists():
        raise SystemExit(f"Reference session already exists, refusing to overwrite: {session_dir}")
    session_dir.mkdir(parents=True)

    gui_config = None if args.no_rs_config else _load_gui_config(args.rs_config)
    stream_cfg = _stream_cfg_from_gui_config(gui_config, DEFAULT_DEPTH) if gui_config is not None else DEFAULT_DEPTH

    serials = sorted(args.serial) if args.serial else _get_realsense_serials()
    if not serials:
        raise SystemExit("No RealSense devices detected.")
    if args.serial is None and len(serials) != args.expected_cameras:
        raise SystemExit(
            f"Expected {args.expected_cameras} RealSense devices, found {len(serials)}: {serials}. "
            "Use --expected-cameras or --serial to override."
        )

    print("Reference session assumes all camera poses are final and the sand is flat.")
    print(f"Session output: {session_dir}")
    print(f"Camera serials: {serials}")

    cameras = []
    for idx, serial in enumerate(serials):
        cameras.append(
            _record_one_camera(
                serial=serial,
                camera_index=idx,
                camera_count=len(serials),
                session_dir=session_dir,
                reference_name=reference_name,
                created_at=now,
                args=args,
                gui_config=gui_config,
                stream_cfg=stream_cfg,
            )
        )

    session_metadata = {
        "reference_name": reference_name,
        "created_at_local": now.isoformat(timespec="seconds"),
        "session_dir": str(session_dir),
        "camera_count": len(cameras),
        "camera_serials": serials,
        "stream": asdict(stream_cfg),
        "rs_config": None if args.no_rs_config else str(args.rs_config),
        "cameras": cameras,
    }
    (session_dir / "session_metadata.json").write_text(json.dumps(session_metadata, indent=2), encoding="utf-8")
    latest_session_path = args.output_root / "latest_reference_session.json"
    latest_session_path.write_text(json.dumps(session_metadata, indent=2), encoding="utf-8")

    print()
    print(f"Saved reference session: {session_dir}")
    print(f"Latest reference session pointer: {latest_session_path}")


if __name__ == "__main__":
    main()
