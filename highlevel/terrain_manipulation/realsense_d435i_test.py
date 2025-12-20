"""Intel RealSense D435i RGB + depth preview, recorder, and snapshot helper.

Run `python3 realsense_d435i_test.py` to open a side-by-side preview window.

Optional features:
    * `--record output.bag` stores a librealsense recording (color + depth).
    * `--video` records RGB/Depth MP4s plus raw RGB/Depth NumPy stacks under
      `highlevel/terrain_manipulation/rgbd_recordings`.
    * Press `s` while previewing to save a PNG pair of RGB and depth frames.

Requires the Intel RealSense SDK Python bindings (`pyrealsense2`) and
`opencv-python`. Install them with:
    pip install pyrealsense2 opencv-python numpy
"""

from __future__ import annotations

import argparse
import signal
import sys
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import List, Optional, Tuple

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

WINDOW_NAME = "Intel RealSense D435i"
DEPTH_COLORMAP_LABEL = "Depth (JET)"
CROSSHAIR_SIZE = 12
RECORDINGS_DIR = Path(__file__).resolve().parent / "rgbd_recordings"


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


def _register_signal_handlers(stop_cb):
    """Stop gracefully when the user hits Ctrl+C."""
    signal.signal(signal.SIGINT, lambda *_: stop_cb())
    signal.signal(signal.SIGTERM, lambda *_: stop_cb())


def _colorize_depth(depth_image: np.ndarray) -> np.ndarray:
    """Convert a raw depth frame to a displayable colormap."""
    depth_scaled = cv2.convertScaleAbs(depth_image, alpha=0.03)
    return cv2.applyColorMap(depth_scaled, cv2.COLORMAP_JET)


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

    try:
        pipeline = _configure_pipeline(args.record)
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

            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())

            height, width, _ = color_image.shape
            sample_point = (width // 2, height // 2)

            depth_colormap = _colorize_depth(depth_image)
            _draw_crosshair(color_image, sample_point)
            _draw_crosshair(depth_colormap, sample_point)

            overlay_text = _format_distance(depth_frame, sample_point)
            cv2.putText(
                color_image,
                overlay_text,
                (10, height - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2,
            )
            cv2.putText(
                depth_colormap,
                DEPTH_COLORMAP_LABEL,
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),
                2,
            )

            if args.video and recorder is None:
                recorder = VideoRecorder.create(width, height)

            if recorder is not None:
                recorder.write(color_image, depth_colormap, depth_image)

            combined = np.hstack((color_image, depth_colormap))
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
