#!/usr/bin/env python3
"""Capture a timed grayscale RealSense image sequence for PIVLab.

The script starts one RealSense color stream, waits for a manual trigger, then
saves only grayscale PNG images at a fixed dt for a requested duration.
"""

from __future__ import annotations

import argparse
import math
import time
from datetime import datetime
from pathlib import Path
from typing import List, Optional

import cv2
import numpy as np

try:
    import pyrealsense2 as rs
except ModuleNotFoundError as exc:  # pragma: no cover - hardware dependency
    raise SystemExit(
        "pyrealsense2 is not installed.\n"
        "Install the Intel RealSense SDK Python bindings before running this script."
    ) from exc


OUTPUT_ROOT = Path(__file__).resolve().parents[2] / "data" / "pivlab_sequences"
COLOR_WIDTH = 1920
COLOR_HEIGHT = 1080
STREAM_FPS = 30
WARMUP_FRAMES = 30
DEFAULT_DT_S = 1.0 / STREAM_FPS
DEFAULT_DURATION_S = 1.0
DEFAULT_PREVIEW = True

TRAJ_SPEED_RAD_S = 2.0
SWEEPING_START_OFFSET_DEG = 0
SWEEPING_START_OFFSET_RAD = math.radians(SWEEPING_START_OFFSET_DEG)
FIXED_TRAJECTORY = [
    0.0,
    -0.53 + SWEEPING_START_OFFSET_RAD,
    TRAJ_SPEED_RAD_S,
    0.0,
    -1.315 + SWEEPING_START_OFFSET_RAD,
    TRAJ_SPEED_RAD_S,
    0.785,
    -1.315 + SWEEPING_START_OFFSET_RAD,
    TRAJ_SPEED_RAD_S,
    0.785,
    -0.53 + SWEEPING_START_OFFSET_RAD,
    TRAJ_SPEED_RAD_S,
    0.785,
    0.1 + SWEEPING_START_OFFSET_RAD,
    TRAJ_SPEED_RAD_S,
    0.0,
    0.1 + SWEEPING_START_OFFSET_RAD,
    TRAJ_SPEED_RAD_S,
    0.0,
    -0.53 + SWEEPING_START_OFFSET_RAD,
    TRAJ_SPEED_RAD_S,
]


def _timestamp_label() -> str:
    return datetime.now().strftime("%Y%m%d_%H%M%S_%f")


class RealSenseCapture:
    def __init__(
        self,
        serial: Optional[str],
        color_width: int,
        color_height: int,
        fps: int,
    ) -> None:
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        if serial:
            self.config.enable_device(serial)
        self.config.enable_stream(rs.stream.color, color_width, color_height, rs.format.bgr8, fps)

    def start(self) -> None:
        self.pipeline.start(self.config)

    def stop(self) -> None:
        self.pipeline.stop()

    def poll(self) -> np.ndarray:
        frames = self.pipeline.wait_for_frames(timeout_ms=5000)
        color = frames.get_color_frame()
        if not color:
            raise RuntimeError("Incomplete color frame received from RealSense pipeline.")
        return np.asanyarray(color.get_data()).copy()

    def poll_gray(self) -> np.ndarray:
        return cv2.cvtColor(self.poll(), cv2.COLOR_BGR2GRAY)


def _get_realsense_serials() -> List[str]:
    ctx = rs.context()
    serials: List[str] = []
    for dev in ctx.query_devices():
        try:
            serials.append(dev.get_info(rs.camera_info.serial_number))
        except Exception:
            continue
    return serials


def _pick_serial(serial: Optional[str], camera_index: int) -> Optional[str]:
    if serial:
        return serial
    serials = _get_realsense_serials()
    if not serials:
        raise SystemExit("No RealSense devices detected.")
    if camera_index < 0 or camera_index >= len(serials):
        raise SystemExit(f"--camera-index {camera_index} out of range; found {len(serials)} device(s): {serials}")
    return serials[camera_index]


def _publish_fixed_trajectory() -> None:
    try:
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import Float64MultiArray
    except ModuleNotFoundError as exc:
        raise SystemExit("ROS 2 Python packages are required to publish the flipper trajectory.") from exc

    rclpy.init(args=None)
    node = Node("pivlab_sequence_trigger")
    publisher = node.create_publisher(Float64MultiArray, "/trajectory_points", 10)
    msg = Float64MultiArray()
    msg.data = list(FIXED_TRAJECTORY)
    end_time = time.time() + 0.5
    while time.time() < end_time:
        publisher.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.05)
    node.destroy_node()
    rclpy.shutdown()


def _publish_stop_command() -> None:
    try:
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import Float64MultiArray
    except ModuleNotFoundError as exc:
        raise RuntimeError("ROS 2 Python packages are required to publish the stop command.") from exc

    rclpy.init(args=None)
    node = Node("pivlab_sequence_stop")
    publisher = node.create_publisher(Float64MultiArray, "/Gui_information", 10)
    msg = Float64MultiArray()
    msg.data = [0.0, 0.0]
    end_time = time.time() + 0.5
    while time.time() < end_time:
        publisher.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.05)
    node.destroy_node()
    rclpy.shutdown()


def _capture_grayscale_sequence(
    capture: RealSenseCapture,
    output_dir: Path,
    dt_s: float,
    duration_s: float,
    prefix: str,
) -> List[float]:
    output_dir.mkdir(parents=True, exist_ok=False)
    frame_times: List[float] = []
    start_time = time.time()
    frame_idx = 0

    while True:
        target_time = start_time + frame_idx * dt_s
        now = time.time()
        if target_time - now > 0.0:
            time.sleep(target_time - now)

        elapsed = time.time() - start_time
        if elapsed > duration_s and frame_idx > 0:
            break

        gray = capture.poll_gray()
        frame_time = time.time() - start_time
        frame_times.append(frame_time)
        path = output_dir / f"{prefix}_{frame_idx:04d}.png"
        if not cv2.imwrite(str(path), gray):
            raise RuntimeError(f"Failed to write image: {path}")
        frame_idx += 1

    return frame_times


def _run_capture(
    capture: RealSenseCapture,
    output_root: Path,
    dt_s: float,
    duration_s: float,
    prefix: str,
    publish_trajectory: bool,
) -> Path:
    if publish_trajectory:
        _publish_fixed_trajectory()
        print("Published fixed trajectory on /trajectory_points.")

    output_dir = output_root / f"sequence_{_timestamp_label()}"
    frame_times = _capture_grayscale_sequence(
        capture=capture,
        output_dir=output_dir,
        dt_s=dt_s,
        duration_s=duration_s,
        prefix=prefix,
    )
    print(f"Saved {len(frame_times)} grayscale frame(s) to {output_dir}")
    if len(frame_times) > 1:
        intervals = np.diff(np.asarray(frame_times, dtype=float))
        print(f"Mean frame spacing: {float(np.mean(intervals)):.6f} s")
    return output_dir


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(description="Capture grayscale RealSense frames for PIVLab.")
    ap.add_argument("--output-root", type=Path, default=OUTPUT_ROOT)
    ap.add_argument("--serial", type=str, default=None, help="RealSense serial number. Defaults to --camera-index.")
    ap.add_argument("--camera-index", type=int, default=0, help="Device index used when --serial is omitted.")
    ap.add_argument("--color-width", type=int, default=COLOR_WIDTH)
    ap.add_argument("--color-height", type=int, default=COLOR_HEIGHT)
    ap.add_argument("--fps", type=int, default=STREAM_FPS)
    ap.add_argument("--warmup-frames", type=int, default=WARMUP_FRAMES)
    ap.add_argument(
        "--dt",
        type=float,
        default=DEFAULT_DT_S,
        help="Target seconds between saved grayscale frames.",
    )
    ap.add_argument(
        "--duration",
        type=float,
        default=DEFAULT_DURATION_S,
        help="Seconds to keep saving grayscale frames after the trigger.",
    )
    ap.add_argument("--prefix", type=str, default="pivlab", help="Filename prefix for saved grayscale PNGs.")
    ap.add_argument(
        "--preview",
        dest="preview",
        action="store_true",
        default=DEFAULT_PREVIEW,
        help="Show a live OpenCV preview; press t for trajectory and g/space to save.",
    )
    ap.add_argument(
        "--no-preview",
        dest="preview",
        action="store_false",
        help="Run from the terminal without an OpenCV preview.",
    )
    return ap.parse_args()


def main() -> int:
    args = parse_args()
    if args.dt <= 0.0:
        raise SystemExit("--dt must be positive.")
    if args.duration <= 0.0:
        raise SystemExit("--duration must be positive.")

    serial = _pick_serial(args.serial, args.camera_index)
    capture = RealSenseCapture(
        serial=serial,
        color_width=args.color_width,
        color_height=args.color_height,
        fps=args.fps,
    )

    capture.start()
    try:
        print(f"Using RealSense serial: {serial}")
        print(f"Saving grayscale frames at dt={args.dt:.6f} s for duration={args.duration:.3f} s")
        print(f"Warming up {args.warmup_frames} frame(s)...")
        for _ in range(max(0, args.warmup_frames)):
            capture.poll()

        if args.preview:
            win = "PIVLab sequence capture"
            cv2.namedWindow(win, cv2.WINDOW_NORMAL)
            trajectory_started = False
            print("Preview active. Press t to start trajectory, g/space to save; q/Esc quits.")
            while True:
                frame = capture.poll()
                cv2.imshow(win, frame)
                key = cv2.waitKey(1) & 0xFF
                if key in (27, ord("q")):
                    return 0
                if key == ord("t"):
                    _publish_fixed_trajectory()
                    trajectory_started = True
                    print("Published fixed trajectory on /trajectory_points. Press g/space when ready to save.")
                if key in (ord("g"), ord(" ")):
                    _run_capture(
                        capture=capture,
                        output_root=args.output_root,
                        dt_s=float(args.dt),
                        duration_s=float(args.duration),
                        prefix=str(args.prefix),
                        publish_trajectory=not trajectory_started,
                    )
                    trajectory_started = True
                    print("Preview active. Press g/space to capture again; q/Esc quits.")
        else:
            input("Press Enter to start the flipper trajectory and begin saving...")
            _run_capture(
                capture=capture,
                output_root=args.output_root,
                dt_s=float(args.dt),
                duration_s=float(args.duration),
                prefix=str(args.prefix),
                publish_trajectory=True,
            )
            return 0
    finally:
        try:
            _publish_stop_command()
            print("Published stop command on /Gui_information.")
        except Exception as exc:
            print(f"[WARN] Failed to publish stop command on /Gui_information: {exc}")
        capture.stop()
        if args.preview:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    raise SystemExit(main())
