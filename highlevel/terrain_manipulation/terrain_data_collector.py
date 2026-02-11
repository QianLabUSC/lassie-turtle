#!/usr/bin/env python3
"""Terrain manipulation data collection pipeline with RGB-D capture.

This script reuses the turtle control interfaces defined in
`LASSIE_GUI/lassie_gui.py` to orchestrate a full data-capture session
without modifying existing GUI code. It:

1. Starts the turtle ROS2 control node (as used by the GUI).
2. Waits for the operator to begin a trial.
3. Publishes the same GUI command message to trigger a predefined
   movement profile on the robot.
4. Streams RGB-D frames from an Intel RealSense D435i camera while the
   robot moves, recording synchronized RGB video, depth visualization
   video, and raw depth tensors.
5. Collects the robot state telemetry exposed by
   `ros2_interface_turtle.ControlNode_Turtle` and saves it alongside the
   RGB-D media under `highlevel/terrain_manipulation/data/<session>/`.

The operator stops the run manually (press Enter or CTRL+C), after which
all recordings and telemetry are flushed to disk.
"""

from __future__ import annotations

import argparse
import json
import math
import signal
import sys
import threading
import time
from dataclasses import dataclass, asdict
from datetime import datetime
from pathlib import Path
from typing import List, Optional, Tuple
import os

import cv2
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Float64MultiArray

try:
    import pyrealsense2 as rs
except ModuleNotFoundError as exc:  # pragma: no cover - defensive guard
    raise SystemExit(
        "pyrealsense2 is not installed.\n"
        "Install the Intel RealSense SDK Python bindings before running this program."
    ) from exc

# Ensure we can import the existing turtle interface as used by lassie_gui.py
PROJECT_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = PROJECT_ROOT.parent
for path in (PROJECT_ROOT, REPO_ROOT):
    if str(path) not in sys.path:
        sys.path.append(str(path))

try:  # Prefer the full GUI interface when its dependencies are available.
    from LASSIE_GUI.ros2_interface_turtle import ControlNode_Turtle  # type: ignore  # noqa: E402
except Exception:  # pragma: no cover - fallback for headless environments
    from highlevel.terrain_manipulation.headless_ros2_interface_turtle import (  # noqa: E402
        ControlNode_Turtle,
    )

STREAM_WIDTH = 640
STREAM_HEIGHT = 480
STREAM_FPS = 30
SESSION_ROOT = Path(__file__).resolve().parent / "data"
PREVIEW_WINDOW = "Terrain Manipulation RGB-D"
DEFAULT_TIMEZONE = os.environ.get("TERRAIN_TIMEZONE", "America/Los_Angeles")


@dataclass
class MovementProfile:
    """Parameters mirroring the turtle GUI sliders."""

    drag_traj: float = 5.0
    lateral_angle_deg: float = 45.0
    drag_speed_mm_s: float = 100.0
    wiggle_time_tenths: float = 2.0
    servo_time_ms: float = 110.0
    extraction_angle_deg: float = 0.0
    wiggle_frequency_hz: float = 10.0
    insertion_depth_mm: float = 10.0
    wiggle_amplitude_cm: float = 1.0

    def to_gui_payload(self, start_flag: float) -> Float64MultiArray:
        msg = Float64MultiArray()
        msg.data = [
            float(start_flag),
            float(self.drag_traj),
            math.radians(self.lateral_angle_deg),
            self.drag_speed_mm_s / 1000.0,
            self.wiggle_time_tenths / 10.0,
            self.servo_time_ms / 1000.0,
            self.extraction_angle_deg,
            self.wiggle_frequency_hz,
            self.insertion_depth_mm / 1000.0,
            self.wiggle_amplitude_cm / 100.0,
        ]
        return msg

    def to_metadata(self) -> dict:
        return asdict(self)


@dataclass
class RGBDRecorder:
    color_writer: cv2.VideoWriter
    depth_writer: cv2.VideoWriter
    color_raw_frames: List[np.ndarray]
    depth_raw_frames: List[np.ndarray]
    color_raw_path: Path
    depth_raw_path: Path

    @classmethod
    def create(cls, session_dir: Path) -> "RGBDRecorder":
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        color_path = session_dir / "rgb.mp4"
        depth_vis_path = session_dir / "depth_colormap.mp4"
        color_raw_path = session_dir / "rgb_raw.npy"
        depth_raw_path = session_dir / "depth_raw.npy"

        color_writer = cv2.VideoWriter(str(color_path), fourcc, STREAM_FPS, (STREAM_WIDTH, STREAM_HEIGHT))
        depth_writer = cv2.VideoWriter(str(depth_vis_path), fourcc, STREAM_FPS, (STREAM_WIDTH, STREAM_HEIGHT))

        if not color_writer.isOpened() or not depth_writer.isOpened():
            color_writer.release()
            depth_writer.release()
            raise SystemExit("Failed to open video writers for RGB-D recording.")

        print(f"Recording RGB video to {color_path}")
        print(f"Recording depth visualization video to {depth_vis_path}")
        print(f"Accumulating raw RGB frames in {color_raw_path}")
        print(f"Accumulating raw depth frames in {depth_raw_path}")

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
            color_stack = np.stack(self.color_raw_frames)
            np.save(self.color_raw_path, color_stack)
            print(f"Saved {color_stack.shape[0]} raw RGB frames to {self.color_raw_path}")
        if self.depth_raw_frames:
            depth_stack = np.stack(self.depth_raw_frames)
            np.save(self.depth_raw_path, depth_stack)
            print(f"Saved {depth_stack.shape[0]} raw depth frames to {self.depth_raw_path}")
        self.color_raw_frames.clear()
        self.depth_raw_frames.clear()


class RealSenseSession:
    def __init__(self) -> None:
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.depth, STREAM_WIDTH, STREAM_HEIGHT, rs.format.z16, STREAM_FPS)
        self.config.enable_stream(rs.stream.color, STREAM_WIDTH, STREAM_HEIGHT, rs.format.bgr8, STREAM_FPS)
        self.align = rs.align(rs.stream.color)

    def start(self) -> None:
        self.pipeline.start(self.config)

    def stop(self) -> None:
        self.pipeline.stop()

    @staticmethod
    def _colorize(depth_image: np.ndarray) -> np.ndarray:
        depth_scaled = cv2.convertScaleAbs(depth_image, alpha=0.03)
        return cv2.applyColorMap(depth_scaled, cv2.COLORMAP_JET)

    def poll(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray, float]:
        frames = self.pipeline.wait_for_frames(timeout_ms=5000)
        frames = self.align.process(frames)
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()
        if not color_frame or not depth_frame:
            raise RuntimeError("Incomplete RGB-D frame received from RealSense pipeline.")

        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())
        depth_colormap = self._colorize(depth_image)

        height, width, _ = color_image.shape
        center = (width // 2, height // 2)
        center_depth = depth_frame.get_distance(*center)
        return color_image, depth_image, depth_colormap, center_depth


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--preview",
        dest="preview",
        action="store_true",
        help="Display a live RGB/depth preview during capture (default).",
    )
    parser.add_argument(
        "--no-preview",
        dest="preview",
        action="store_false",
        help="Disable live RGB/depth preview (useful over SSH/headless).",
    )
    parser.add_argument("--drag-traj", type=float, default=5.0, help="Trajectory ID (matches GUI dropdown).")
    parser.add_argument("--lateral-angle", type=float, default=45.0, help="Initial lateral angle in degrees.")
    parser.add_argument("--drag-speed", type=float, default=100.0, help="Drag speed in mm/s.")
    parser.add_argument("--wiggle-time", type=float, default=2.0, help="Wiggle time units (0.1 s increments).")
    parser.add_argument("--servo-time", type=float, default=110.0, help="Servo time in milliseconds.")
    parser.add_argument("--extraction-angle", type=float, default=0.0, help="Extraction angle in degrees.")
    parser.add_argument("--wiggle-frequency", type=float, default=10.0, help="Wiggle frequency in Hz.")
    parser.add_argument("--insertion-depth", type=float, default=10.0, help="Insertion depth in millimetres.")
    parser.add_argument("--wiggle-amplitude", type=float, default=1.0, help="Wiggle amplitude in centimetres.")
    parser.add_argument("--metadata-note", type=str, help="Optional operator note stored with the session metadata.")
    parser.add_argument(
        "--waypoint-pattern",
        type=str,
        default="circle",
        choices=["none", "line", "diag", "triangle", "circle"],
        help="Publish a waypoint set to /trajectory_points before sending the start flag.",
    )
    parser.add_argument(
        "--waypoint-rate",
        type=float,
        default=10.0,
        help="Waypoint publish rate (Hz) while the run is active.",
    )
    parser.set_defaults(preview=True)
    return parser.parse_args()


def build_profile(args: argparse.Namespace) -> MovementProfile:
    return MovementProfile(
        drag_traj=args.drag_traj,
        lateral_angle_deg=args.lateral_angle,
        drag_speed_mm_s=args.drag_speed,
        wiggle_time_tenths=args.wiggle_time,
        servo_time_ms=args.servo_time,
        extraction_angle_deg=args.extraction_angle,
        wiggle_frequency_hz=args.wiggle_frequency,
        insertion_depth_mm=args.insertion_depth,
        wiggle_amplitude_cm=args.wiggle_amplitude,
    )


def build_waypoints(pattern: str) -> List[float]:
    """Return a flattened waypoint list [x, y, vel, ...] suitable for /trajectory_points."""
    pattern = pattern.lower()
    if pattern == "line":
        return [0.0, 0.16, 0.01, 0.0, 0.21, 0.01]
    if pattern == "diag":
        return [-0.4, 0.0, 0.02, 0.4, 0.0, 0.02]
    if pattern == "triangle":
        return [-0.4, 0.0, 0.5, 0.4, 0.0, 0.5, 0.0, -0.6, 0.5]
    if pattern == "circle":
        return [
            0.35,
            0.00,
            0.5,
            0.25,
            0.25,
            0.5,
            0.00,
            0.35,
            0.5,
            -0.25,
            0.25,
            0.5,
            -0.35,
            0.00,
            0.5,
            -0.25,
            -0.25,
            0.5,
            0.00,
            -0.35,
            0.5,
            0.25,
            -0.25,
            0.5,
            0.35,
            0.00,
            0.5,
        ]
    return []


def _resolve_now(timezone_name: Optional[str]) -> datetime:
    """Return an aware datetime using the requested timezone, falling back to local time."""
    if timezone_name:
        try:
            from zoneinfo import ZoneInfo  # Python 3.9+
        except ModuleNotFoundError:  # pragma: no cover - Python 3.8 fallback
            try:
                from backports.zoneinfo import ZoneInfo  # type: ignore
            except ModuleNotFoundError:
                ZoneInfo = None  # type: ignore
        if ZoneInfo is not None:
            try:
                return datetime.now(ZoneInfo(timezone_name))
            except Exception:
                pass  # fall through to local time
    now = datetime.now()
    if now.tzinfo is None:
        return now.astimezone()
    return now


def ensure_session_dir(run_time: datetime) -> Path:
    SESSION_ROOT.mkdir(parents=True, exist_ok=True)
    timestamp = run_time.strftime("%Y%m%d_%H%M%S")
    session_dir = SESSION_ROOT / f"session_{timestamp}"
    session_dir.mkdir(parents=True, exist_ok=False)
    return session_dir


def save_metadata(session_dir: Path, profile: MovementProfile, metadata_note: Optional[str], start_time: datetime, stop_time: datetime) -> None:
    payload = {
        "profile": profile.to_metadata(),
        "start_time": start_time.isoformat(),
        "stop_time": stop_time.isoformat(),
        "duration_sec": (stop_time - start_time).total_seconds(),
    }
    if metadata_note:
        payload["operator_note"] = metadata_note

    with open(session_dir / "metadata.json", "w", encoding="utf-8") as fh:
        json.dump(payload, fh, indent=2)


def save_robot_data(session_dir: Path, node: ControlNode_Turtle, profile: MovementProfile, real_time_plot: bool = False) -> None:
    csv_path = session_dir / "robot_state.csv"
    headers = [
        "time",
        "turtle_state",
        "leftadduction_pos",
        "leftsweeping_pos",
        "rightadduction_pos",
        "rightsweeping_pos",
        "leftadduction_curr",
        "leftsweeping_curr",
        "rightadduction_curr",
        "rightsweeping_curr",
        "OptitrackPosition_x",
        "OptitrackPosition_y",
        "OptitrackPosition_z",
        "OptitrackOrientation_x",
        "OptitrackOrientation_y",
        "OptitrackOrientation_z",
        "OptitrackOrientation_w",
        "LeftFlipperPosition_x",
        "LeftFlipperPosition_y",
        "LeftFlipperPosition_z",
        "LeftFlipperOrientation_x",
        "LeftFlipperOrientation_y",
        "LeftFlipperOrientation_z",
        "LeftFlipperOrientation_w",
        "RightFlipperPosition_x",
        "RightFlipperPosition_y",
        "RightFlipperPosition_z",
        "RightFlipperOrientation_x",
        "RightFlipperOrientation_y",
        "RightFlipperOrientation_z",
        "RightFlipperOrientation_w",
    ]

    with open(csv_path, "w", encoding="utf-8") as fh:
        fh.write("scenario,real_time_plot,lateral_angle_rad,drag_speed_m_s,wiggle_time_s,servo_time_s,extraction_angle_deg,wiggle_frequency_hz,insertion_depth_m,wiggle_amplitude_m\n")
        fh.write(
            f"turtle,{int(real_time_plot)},{math.radians(profile.lateral_angle_deg):.6f},{profile.drag_speed_mm_s / 1000.0:.6f},"
            f"{profile.wiggle_time_tenths / 10.0:.6f},{profile.servo_time_ms / 1000.0:.6f},{profile.extraction_angle_deg:.6f},"
            f"{profile.wiggle_frequency_hz:.6f},{profile.insertion_depth_mm / 1000.0:.6f},{profile.wiggle_amplitude_cm / 100.0:.6f}\n"
        )
        fh.write(",".join(headers) + "\n")
        rows = zip(
            node.time_list,
            node.turtle_state_list,
            node.leftadduction_pos_array,
            node.leftsweeping_pos_array,
            node.rightadduction_pos_array,
            node.rightsweeping_pos_array,
            node.leftadduction_curr_array,
            node.leftsweeping_curr_array,
            node.rightadduction_curr_array,
            node.rightsweeping_curr_array,
            node.OptitrackPosition_x_list,
            node.OptitrackPosition_y_list,
            node.OptitrackPosition_z_list,
            node.OptitrackOrientation_x_list,
            node.OptitrackOrientation_y_list,
            node.OptitrackOrientation_z_list,
            node.OptitrackOrientation_w_list,
            node.LeftFlipperPosition_x_list,
            node.LeftFlipperPosition_y_list,
            node.LeftFlipperPosition_z_list,
            node.LeftFlipperOrientation_x_list,
            node.LeftFlipperOrientation_y_list,
            node.LeftFlipperOrientation_z_list,
            node.LeftFlipperOrientation_w_list,
            node.RightFlipperPosition_x_list,
            node.RightFlipperPosition_y_list,
            node.RightFlipperPosition_z_list,
            node.RightFlipperOrientation_x_list,
            node.RightFlipperOrientation_y_list,
            node.RightFlipperOrientation_z_list,
            node.RightFlipperOrientation_w_list,
        )
        for row in rows:
            fh.write(",".join(f"{value}" for value in row) + "\n")

    print(f"Robot telemetry saved to {csv_path}")


def bind_signal(sig: int, handler) -> None:
    try:
        signal.signal(sig, handler)
    except ValueError:
        # Signals are not available on all platforms (e.g., Windows), so ignore failure.
        pass


def main() -> None:
    args = parse_args()
    profile = build_profile(args)
    session_dir = ensure_session_dir(_resolve_now(DEFAULT_TIMEZONE))
    print(f"Session directory: {session_dir}")

    rclpy.init()
    node = ControlNode_Turtle()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    executor_stop = threading.Event()
    data_loop_running = threading.Event()
    waypoint_loop_running = threading.Event()

    def executor_thread():
        while not executor_stop.is_set():
            executor.spin_once(timeout_sec=0.1)

    spin_thread = threading.Thread(target=executor_thread, daemon=True)
    spin_thread.start()

    force_loop_stop = threading.Event()

    def force_data_thread():
        while not force_loop_stop.is_set():
            node.update_force_data(data_loop_running.is_set())
            time.sleep(0.01)

    force_thread = threading.Thread(target=force_data_thread, daemon=True)
    force_thread.start()

    waypoints = build_waypoints(args.waypoint_pattern)
    waypoint_publisher = None
    waypoint_thread_stop = threading.Event()
    waypoint_thread_handle: Optional[threading.Thread] = None
    if waypoints:
        waypoint_publisher = node.create_publisher(Float64MultiArray, "/trajectory_points", 10)

    def waypoint_thread():
        if waypoint_publisher is None:
            return
        msg = Float64MultiArray()
        msg.data = waypoints
        while not waypoint_thread_stop.is_set():
            if waypoint_loop_running.is_set():
                waypoint_publisher.publish(msg)
                time.sleep(max(0.01, 1.0 / max(args.waypoint_rate, 0.1)))
            else:
                time.sleep(0.05)

    if waypoint_publisher is not None:
        waypoint_thread_handle = threading.Thread(target=waypoint_thread, daemon=True)
        waypoint_thread_handle.start()

    stop_requested = threading.Event()

    def request_stop(*_args):
        stop_requested.set()

    bind_signal(signal.SIGINT, request_stop)
    bind_signal(signal.SIGTERM, request_stop)

    input("Press Enter to start the terrain manipulation run...")

    node.calibrate()
    realsense = RealSenseSession()
    realsense.start()
    recorder = RGBDRecorder.create(session_dir)
    if waypoint_publisher is not None:
        initial_waypoints = Float64MultiArray()
        initial_waypoints.data = waypoints
        waypoint_publisher.publish(initial_waypoints)
        waypoint_loop_running.set()
        print(f"Published {len(waypoints) // 3} waypoints to /trajectory_points.")

    start_time = _resolve_now(DEFAULT_TIMEZONE)
    data_loop_running.set()
    node.publish_gui_information(profile.to_gui_payload(start_flag=1))
    print("Robot command issued. Recording RGB-D and telemetry...\nPress Enter again or use CTRL+C to stop.")

    def stop_listener():
        input()
        stop_requested.set()

    threading.Thread(target=stop_listener, daemon=True).start()

    center_depth_display = 0.0
    try:
        while not stop_requested.is_set():
            color_image, depth_raw, depth_colormap, center_depth_display = realsense.poll()
            recorder.write(color_image, depth_colormap, depth_raw)

            if args.preview:
                preview = np.hstack((color_image, depth_colormap))
                overlay_text = f"Center depth: {center_depth_display:.2f} m"
                cv2.putText(preview, overlay_text, (10, STREAM_HEIGHT - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.imshow(PREVIEW_WINDOW, preview)
                if cv2.waitKey(1) & 0xFF in (27, ord('q')):
                    stop_requested.set()
    except RuntimeError as exc:
        print(f"RealSense stream error: {exc}")
        stop_requested.set()

    stop_time = _resolve_now(DEFAULT_TIMEZONE)

    # Issue stop command to the robot and tear down resources
    data_loop_running.clear()
    waypoint_loop_running.clear()
    node.publish_gui_information(profile.to_gui_payload(start_flag=0))

    force_loop_stop.set()
    waypoint_thread_stop.set()
    recorder.close()
    realsense.stop()
    if args.preview:
        cv2.destroyWindow(PREVIEW_WINDOW)

    executor_stop.set()
    spin_thread.join(timeout=1.0)
    force_thread.join(timeout=1.0)
    if waypoint_thread_handle is not None:
        waypoint_thread_handle.join(timeout=1.0)

    save_metadata(session_dir, profile, args.metadata_note, start_time, stop_time)
    save_robot_data(session_dir, node, profile, real_time_plot=False)

    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()

    print("Session complete.")
    print(f"Center-point depth at stop: {center_depth_display:.2f} m")
    print(f"Data stored under: {session_dir}")


if __name__ == "__main__":
    main()
