"""Intel RealSense D435i RGB + depth preview with tuned depth settings.

Run `python3 realsense_d435i_tuned.py` to open a side-by-side preview window.

Differences vs the baseline script:
    * Enables emitter and pushes laser power to max.
    * Uses RealSense visual presets (default: high density).
    * Applies decimation + spatial + temporal + hole-filling filters.
    * Normalizes/clamps depth before colorizing for better contrast (default 0–1.5 m).
    * Uses a perceptual colormap (default: TURBO; CLI-selectable).
    * Retains recording (`--record`) and RGBD video/NumPy export (`--video`) features.

Requires `pyrealsense2`, `opencv-python`, `numpy`.
"""

from __future__ import annotations

import argparse
import signal
import sys
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Tuple

try:
    import pyrealsense2 as rs
except ModuleNotFoundError as exc:  # pragma: no cover - import guard
    raise SystemExit(
        "pyrealsense2 is not installed.\n"
        "Install the Intel RealSense SDK Python bindings before running this script."
    ) from exc

import cv2
import numpy as np

from camera_configs import STREAM_WIDTH, STREAM_HEIGHT, STREAM_FPS

WINDOW_NAME = "Intel RealSense D435i (tuned)"
CROSSHAIR_SIZE = 12
RECORDINGS_DIR = Path(__file__).resolve().parent / "rgbd_recordings"

# Perceptual colormaps to choose from.
COLORMAPS: Dict[str, int] = {
    "turbo": cv2.COLORMAP_TURBO,
    "inferno": cv2.COLORMAP_INFERNO,
    "plasma": cv2.COLORMAP_PLASMA,
    "magma": cv2.COLORMAP_MAGMA,
    "viridis": cv2.COLORMAP_VIRIDIS,
    "jet": cv2.COLORMAP_JET,
}

# Visual presets of interest.
VISUAL_PRESETS = {
    "default": None,
    "high_accuracy": rs.rs400_visual_preset.high_accuracy,
    "high_density": rs.rs400_visual_preset.high_density,
    "medium_density": rs.rs400_visual_preset.medium_density,
}


@dataclass
class VideoRecorder:
    """Lightweight recorder storing RGB/Depth MP4s and raw NumPy tensors."""

    color_writer: cv2.VideoWriter
    depth_writer: cv2.VideoWriter
    color_raw_frames: List[np.ndarray]
    depth_raw_frames: List[np.ndarray]
    color_raw_path: Path
    depth_raw_path: Path

    @classmethod
    def create(cls, width: int, height: int) -> "VideoRecorder":
        RECORDINGS_DIR.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

        color_path = RECORDINGS_DIR / f"{timestamp}_rgb.mp4"
        depth_vis_path = RECORDINGS_DIR / f"{timestamp}_depth_colormap.mp4"
        color_raw_path = RECORDINGS_DIR / f"{timestamp}_rgb.npy"
        depth_raw_path = RECORDINGS_DIR / f"{timestamp}_depth.npy"

        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        color_writer = cv2.VideoWriter(
            str(color_path), fourcc, STREAM_FPS, (width, height)
        )
        depth_writer = cv2.VideoWriter(
            str(depth_vis_path), fourcc, STREAM_FPS, (width, height)
        )
        if not color_writer.isOpened() or not depth_writer.isOpened():
            color_writer.release()
            depth_writer.release()
            raise SystemExit("Unable to open video writers for RGB-D recording.")

        print(f"Recording RGB video to {color_path}")
        print(f"Recording depth colormap video to {depth_vis_path}")
        print(f"Saving raw RGB tensor to {color_raw_path}")
        print(f"Saving raw depth tensor to {depth_raw_path}")

        return cls(
            color_writer=color_writer,
            depth_writer=depth_writer,
            color_raw_frames=[],
            depth_raw_frames=[],
            color_raw_path=color_raw_path,
            depth_raw_path=depth_raw_path,
        )

    def write(self, color_image: np.ndarray, depth_colormap: np.ndarray, depth_raw: np.ndarray) -> None:
        self.color_writer.write(np.ascontiguousarray(color_image))
        self.depth_writer.write(np.ascontiguousarray(depth_colormap))
        self.color_raw_frames.append(color_image.copy())
        self.depth_raw_frames.append(depth_raw.copy())

    def close(self) -> None:
        self.color_writer.release()
        self.depth_writer.release()
        if self.color_raw_frames:
            rgb_stack = np.stack(self.color_raw_frames)
            np.save(self.color_raw_path, rgb_stack)
            print(f"Saved {rgb_stack.shape[0]} raw RGB frames to {self.color_raw_path}")
        if self.depth_raw_frames:
            depth_stack = np.stack(self.depth_raw_frames)
            np.save(self.depth_raw_path, depth_stack)
            print(f"Saved raw depth stack to {self.depth_raw_path}")
        self.color_raw_frames.clear()
        self.depth_raw_frames.clear()


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--record",
        type=Path,
        metavar="PATH",
        help="Write a librealsense .bag recording to PATH (overwrites existing file).",
    )
    parser.add_argument(
        "--save-dir",
        type=Path,
        default=Path("captures"),
        help="Directory used when saving RGB/depth PNG frames via the 's' key.",
    )
    parser.add_argument(
        "--video",
        action="store_true",
        help="Record RGB-D video/NumPy outputs under terrain_manipulation/rgbd_recordings.",
    )
    parser.add_argument(
        "--colormap",
        choices=sorted(COLORMAPS.keys()),
        default="turbo",
        help="Colormap for depth visualization.",
    )
    parser.add_argument(
        "--max-depth",
        type=float,
        default=1.5,
        help="Depth range upper bound (meters) used for clamping/normalization.",
    )
    parser.add_argument(
        "--min-depth",
        type=float,
        default=0.0,
        help="Depth range lower bound (meters) used for clamping/normalization.",
    )
    parser.add_argument(
        "--preset",
        choices=sorted(VISUAL_PRESETS.keys()),
        default="high_density",
        help="RealSense visual preset applied to the depth sensor.",
    )
    parser.add_argument(
        "--decimate",
        type=int,
        default=1,
        help="Decimation magnitude (>=1). Values >1 downsample depth to reduce noise.",
    )
    parser.add_argument(
        "--auto-range",
        dest="auto_range",
        action="store_true",
        help="Automatically determine depth min/max per frame using percentiles.",
    )
    parser.add_argument(
        "--no-auto-range",
        dest="auto_range",
        action="store_false",
        help="Disable automatic depth range detection; rely solely on --min-depth/--max-depth.",
    )
    parser.set_defaults(auto_range=True)
    parser.add_argument(
        "--display-smooth",
        choices=("none", "median", "gaussian", "bilateral"),
        default="median",
        help="Extra smoothing applied only to the depth visualization (not raw depth).",
    )
    parser.add_argument(
        "--smooth-kernel",
        type=int,
        default=5,
        help="Kernel size for display smoothing (odd integer; ignored when --display-smooth=none).",
    )
    return parser.parse_args()


def _configure_pipeline(record_to: Optional[Path]) -> rs.pipeline:
    """Create and start a RealSense pipeline streaming color and depth frames."""
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, STREAM_WIDTH, STREAM_HEIGHT, rs.format.z16, STREAM_FPS)
    config.enable_stream(rs.stream.color, STREAM_WIDTH, STREAM_HEIGHT, rs.format.bgr8, STREAM_FPS)
    if record_to:
        record_path = record_to.expanduser().resolve()
        record_path.parent.mkdir(parents=True, exist_ok=True)
        if record_path.exists():
            record_path.unlink()
        config.enable_record_to_file(str(record_path))
        print(f"Recording RealSense stream to {record_path}")
    pipeline.start(config)
    return pipeline


def _apply_depth_settings(pipeline: rs.pipeline, preset: str) -> float:
    """Enable emitter/laser, apply visual preset, and return depth scale."""
    profile = pipeline.get_active_profile()
    depth_sensor = profile.get_device().first_depth_sensor()
    if depth_sensor.supports(rs.option.laser_power):
        laser_range = depth_sensor.get_option_range(rs.option.laser_power)
        depth_sensor.set_option(rs.option.laser_power, laser_range.max)
        print(f"Set laser_power to max: {laser_range.max}")
    if depth_sensor.supports(rs.option.emitter_enabled):
        depth_sensor.set_option(rs.option.emitter_enabled, 1)
        print("Enabled emitter")
    preset_value = VISUAL_PRESETS.get(preset)
    if preset_value is not None and depth_sensor.supports(rs.option.visual_preset):
        depth_sensor.set_option(rs.option.visual_preset, preset_value)
        print(f"Applied visual preset: {preset}")
    depth_scale = depth_sensor.get_depth_scale()
    print(f"Depth scale: {depth_scale} meters per unit")
    return depth_scale


def _register_signal_handlers(stop_cb):
    """Stop gracefully when the user hits Ctrl+C."""
    signal.signal(signal.SIGINT, lambda *_: stop_cb())
    signal.signal(signal.SIGTERM, lambda *_: stop_cb())


def _colorize_depth(
    depth_image: np.ndarray,
    depth_scale: float,
    min_depth_m: float,
    max_depth_m: float,
    colormap: int,
    auto_range: bool,
    display_smooth: str,
    smooth_kernel: int,
) -> Tuple[np.ndarray, Tuple[float, float]]:
    """Normalize and colorize depth image with clamping to the specified range."""
    min_units = max(min_depth_m / depth_scale, 0.0)
    max_units = max(max_depth_m / depth_scale, min_units + 1e-6)

    if auto_range:
        valid_mask = depth_image > 0
        if np.any(valid_mask):
            valid = depth_image[valid_mask].astype(np.float32)
            low = np.percentile(valid, 5)
            high = np.percentile(valid, 95)
            if high - low < 1e-6:
                high = low + 1e-3
            min_units = max(min_units, low)
            max_units = min(max_units, high)

    depth_clipped = np.clip(depth_image, min_units, max_units)
    depth_norm = cv2.normalize(depth_clipped, None, 0, 255, cv2.NORM_MINMAX)
    depth_u8 = depth_norm.astype(np.uint8)
    if display_smooth != "none":
        kernel = smooth_kernel if smooth_kernel % 2 == 1 else smooth_kernel + 1
        kernel = max(kernel, 3)
        if display_smooth == "median":
            depth_u8 = cv2.medianBlur(depth_u8, kernel)
        elif display_smooth == "gaussian":
            depth_u8 = cv2.GaussianBlur(depth_u8, (kernel, kernel), sigmaX=0)
        elif display_smooth == "bilateral":
            depth_u8 = cv2.bilateralFilter(depth_u8, d=kernel, sigmaColor=50, sigmaSpace=5)
    colormap_img = cv2.applyColorMap(depth_u8, colormap)
    return colormap_img, (min_units * depth_scale, max_units * depth_scale)


def _draw_crosshair(image: np.ndarray, center: Tuple[int, int]) -> None:
    cv2.drawMarker(
        image,
        center,
        (255, 255, 255),
        markerType=cv2.MARKER_CROSS,
        markerSize=CROSSHAIR_SIZE,
        thickness=1,
    )


def _format_distance(depth_frame: rs.depth_frame, point: Tuple[int, int]) -> str:
    distance = depth_frame.get_distance(*point)
    return f"Center depth: {distance:.2f} m" if distance > 0 else "Center depth: ---"


def _save_frame_pair(save_dir: Path, color_image: np.ndarray, depth_image: np.ndarray) -> None:
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    save_dir.mkdir(parents=True, exist_ok=True)
    color_path = save_dir / f"color_{timestamp}.png"
    depth_path = save_dir / f"depth_{timestamp}.png"
    cv2.imwrite(str(color_path), color_image)
    cv2.imwrite(str(depth_path), depth_image)
    print(f"Saved frame pair to {color_path} and {depth_path}")


def main() -> None:
    args = _parse_args()
    pipeline: Optional[rs.pipeline] = None
    recorder: Optional[VideoRecorder] = None
    stop_requested = False

    def request_stop():
        nonlocal stop_requested
        stop_requested = True

    align_to_color = rs.align(rs.stream.color)
    decimation = rs.decimation_filter()
    if args.decimate > 1:
        decimation.set_option(rs.option.filter_magnitude, float(args.decimate))
    depth_to_disparity = rs.disparity_transform(True)
    disparity_to_depth = rs.disparity_transform(False)
    spatial = rs.spatial_filter()
    temporal = rs.temporal_filter()
    hole_filling = rs.hole_filling_filter()

    try:
        pipeline = _configure_pipeline(args.record)
        depth_scale = _apply_depth_settings(pipeline, args.preset)
    except RuntimeError as exc:
        raise SystemExit(f"Failed to start RealSense pipeline: {exc}") from exc

    _register_signal_handlers(request_stop)
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW_NAME, STREAM_WIDTH * 2, STREAM_HEIGHT)

    try:
        while not stop_requested:
            frames = pipeline.wait_for_frames(timeout_ms=5000)
            frames = align_to_color.process(frames)
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            if not color_frame or not depth_frame:
                continue

            # Filtering pipeline for smoother, denser depth.
            depth_frame = decimation.process(depth_frame)
            depth_frame = depth_to_disparity.process(depth_frame)
            depth_frame = spatial.process(depth_frame)
            depth_frame = temporal.process(depth_frame)
            depth_frame = disparity_to_depth.process(depth_frame)
            depth_frame = hole_filling.process(depth_frame)
            depth_frame = depth_frame.as_depth_frame()

            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())

            color_h, color_w, _ = color_image.shape
            depth_h, depth_w = depth_image.shape
            color_sample = (color_w // 2, color_h // 2)
            depth_sample = (depth_w // 2, depth_h // 2)

            depth_colormap, depth_range = _colorize_depth(
                depth_image,
                depth_scale,
                args.min_depth,
                args.max_depth,
                COLORMAPS[args.colormap],
                args.auto_range,
                args.display_smooth,
                args.smooth_kernel,
            )
            _draw_crosshair(color_image, color_sample)
            _draw_crosshair(depth_colormap, depth_sample)

            # Resize depth display to match color for side-by-side view/recording.
            if depth_colormap.shape[:2] != color_image.shape[:2]:
                depth_display = cv2.resize(depth_colormap, (color_w, color_h), interpolation=cv2.INTER_NEAREST)
            else:
                depth_display = depth_colormap

            overlay_text = _format_distance(depth_frame, depth_sample)
            cv2.putText(
                color_image,
                overlay_text,
                (10, color_h - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2,
            )
            cv2.putText(
                depth_display,
                f"Depth ({args.colormap.upper()})",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),
                2,
            )
            cv2.putText(
                depth_display,
                f"{depth_range[0]:.2f}-{depth_range[1]:.2f} m",
                (10, color_h - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2,
            )

            if args.video and recorder is None:
                recorder = VideoRecorder.create(color_w, color_h)

            if recorder is not None:
                recorder.write(color_image, depth_display, depth_image)

            combined = np.hstack((color_image, depth_display))
            cv2.imshow(WINDOW_NAME, combined)

            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q")):
                break
            if key in (ord("s"), ord("S")):
                _save_frame_pair(args.save_dir, color_image, depth_image)
    except RuntimeError as exc:
        print(f"RealSense stream error: {exc}", file=sys.stderr)
    finally:
        if recorder is not None:
            recorder.close()
        if pipeline is not None:
            pipeline.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
