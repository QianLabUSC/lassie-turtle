#!/usr/bin/env python3
"""Distributed RGB-D data collection for terrain manipulation.

Runs on the PC with a single RealSense camera attached, while the RPI executes
robot motion. This node publishes trajectory/start/stop commands via ROS 2 and
records robot telemetry streamed back from the RPI.

Output: one npy file per trial containing RGB-D frames, timestamps, telemetry,
and metadata.
"""

from __future__ import annotations

import signal
import sys
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Tuple
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

SESSION_ROOT = Path(__file__).resolve().parent / "data"
DEFAULT_TIMEZONE = os.environ.get("TERRAIN_TIMEZONE", "Etc/GMT+8")

STREAM_WIDTH = 848
STREAM_HEIGHT = 480
STREAM_FPS = 30
DEPTH_MIN_M = None
DEPTH_MAX_M = None
DEPTH_SCHEME = "jet"
DEPTH_HIST_EQ = True
DEPTH_POSTPROCESS = False
TRIAL_COUNT = 1
TRIAL_DURATION_SEC = 7.0

FIXED_TRAJECTORY = [
    0.0,
    -0.53,
    0.7,
    0.0,
    -1.315,
    0.7,
    0.785,
    -1.315,
    0.7,
    0.785,
    -0.53,
    0.7,
    0.785,
    0.255,
    0.7,
    0.0,
    0.255,
    0.7,
    0.0,
    -0.53,
    0.7,
]


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


def build_metadata(start_time: datetime, stop_time: datetime) -> Dict[str, object]:
    return {
        "trajectory_points": FIXED_TRAJECTORY,
        "start_time": start_time.isoformat(),
        "stop_time": stop_time.isoformat(),
        "duration_sec": (stop_time - start_time).total_seconds(),
    }


def build_robot_state(node: ControlNode_Turtle) -> Dict[str, np.ndarray]:
    return {
        "time": np.asarray(node.time_list),
        "turtle_state": np.asarray(node.turtle_state_list),
        "leftadduction_pos": np.asarray(node.leftadduction_pos_array),
        "leftsweeping_pos": np.asarray(node.leftsweeping_pos_array),
        "rightadduction_pos": np.asarray(node.rightadduction_pos_array),
        "rightsweeping_pos": np.asarray(node.rightsweeping_pos_array),
        "leftadduction_curr": np.asarray(node.leftadduction_curr_array),
        "leftsweeping_curr": np.asarray(node.leftsweeping_curr_array),
        "rightadduction_curr": np.asarray(node.rightadduction_curr_array),
        "rightsweeping_curr": np.asarray(node.rightsweeping_curr_array),
        "OptitrackPosition_x": np.asarray(node.OptitrackPosition_x_list),
        "OptitrackPosition_y": np.asarray(node.OptitrackPosition_y_list),
        "OptitrackPosition_z": np.asarray(node.OptitrackPosition_z_list),
        "OptitrackOrientation_x": np.asarray(node.OptitrackOrientation_x_list),
        "OptitrackOrientation_y": np.asarray(node.OptitrackOrientation_y_list),
        "OptitrackOrientation_z": np.asarray(node.OptitrackOrientation_z_list),
        "OptitrackOrientation_w": np.asarray(node.OptitrackOrientation_w_list),
        "LeftFlipperPosition_x": np.asarray(node.LeftFlipperPosition_x_list),
        "LeftFlipperPosition_y": np.asarray(node.LeftFlipperPosition_y_list),
        "LeftFlipperPosition_z": np.asarray(node.LeftFlipperPosition_z_list),
        "LeftFlipperOrientation_x": np.asarray(node.LeftFlipperOrientation_x_list),
        "LeftFlipperOrientation_y": np.asarray(node.LeftFlipperOrientation_y_list),
        "LeftFlipperOrientation_z": np.asarray(node.LeftFlipperOrientation_z_list),
        "LeftFlipperOrientation_w": np.asarray(node.LeftFlipperOrientation_w_list),
        "RightFlipperPosition_x": np.asarray(node.RightFlipperPosition_x_list),
        "RightFlipperPosition_y": np.asarray(node.RightFlipperPosition_y_list),
        "RightFlipperPosition_z": np.asarray(node.RightFlipperPosition_z_list),
        "RightFlipperOrientation_x": np.asarray(node.RightFlipperOrientation_x_list),
        "RightFlipperOrientation_y": np.asarray(node.RightFlipperOrientation_y_list),
        "RightFlipperOrientation_z": np.asarray(node.RightFlipperOrientation_z_list),
        "RightFlipperOrientation_w": np.asarray(node.RightFlipperOrientation_w_list),
    }


def _try_set(opt_owner, option, value) -> None:
    try:
        opt_owner.set_option(option, value)
    except Exception as exc:
        print(f"[WARN] Could not set {option} to {value}: {exc}")


def _make_colorizer() -> rs.colorizer:
    scheme_map = {
        "jet": 0,
        "classic": 1,
        "white_to_black": 2,
        "black_to_white": 3,
        "bio": 4,
        "cold": 5,
        "warm": 6,
        "quantized": 7,
        "pattern": 8,
        "turbo": 9,
    }
    cz = rs.colorizer()
    scheme = scheme_map.get(DEPTH_SCHEME, 0)
    _try_set(cz, rs.option.color_scheme, float(scheme))
    if DEPTH_MIN_M is not None:
        _try_set(cz, rs.option.min_distance, float(DEPTH_MIN_M))
    if DEPTH_MAX_M is not None:
        _try_set(cz, rs.option.max_distance, float(DEPTH_MAX_M))
    _try_set(cz, rs.option.histogram_equalization_enabled, 1.0 if DEPTH_HIST_EQ else 0.0)
    return cz


class RGBDRecorder:
    def __init__(self) -> None:
        self.color_raw_frames: List[np.ndarray] = []
        self.depth_raw_frames: List[np.ndarray] = []
        self.timestamps: List[float] = []

    def write(self, color_image: np.ndarray, depth_raw: np.ndarray, timestamp: float) -> None:
        self.color_raw_frames.append(color_image.copy())
        self.depth_raw_frames.append(depth_raw.copy())
        self.timestamps.append(float(timestamp))

    def finalize(self) -> Dict[str, np.ndarray]:
        rgb = np.stack(self.color_raw_frames) if self.color_raw_frames else np.empty((0,))
        depth = np.stack(self.depth_raw_frames) if self.depth_raw_frames else np.empty((0,))
        timestamps = np.asarray(self.timestamps, dtype=float)
        self.color_raw_frames.clear()
        self.depth_raw_frames.clear()
        self.timestamps.clear()
        return {
            "rgb": rgb,
            "depth": depth,
            "timestamps": timestamps,
        }


class RealSenseSession:
    def __init__(self, serial: Optional[str] = None) -> None:
        self.serial = serial
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        if serial:
            self.config.enable_device(serial)
        self.config.enable_stream(rs.stream.depth, STREAM_WIDTH, STREAM_HEIGHT, rs.format.z16, STREAM_FPS)
        self.config.enable_stream(rs.stream.color, STREAM_WIDTH, STREAM_HEIGHT, rs.format.bgr8, STREAM_FPS)
        self.colorizer = _make_colorizer()
        self.spatial = rs.spatial_filter()
        self.temporal = rs.temporal_filter()
        self.hole = rs.hole_filling_filter()
        self.align = rs.align(rs.stream.color)

    def start(self) -> None:
        self.pipeline.start(self.config)

    def stop(self) -> None:
        self.pipeline.stop()

    def poll(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        frames = self.pipeline.wait_for_frames(timeout_ms=5000)
        frames = self.align.process(frames)
        depth = frames.get_depth_frame()
        color = frames.get_color_frame()
        if not depth or not color:
            raise RuntimeError("Incomplete RGB-D frame received from RealSense pipeline.")

        if DEPTH_POSTPROCESS:
            depth = self.spatial.process(depth)
            depth = self.temporal.process(depth)
            depth = self.hole.process(depth)

        depth_color = self.colorizer.colorize(depth)
        depth_img = np.asanyarray(depth_color.get_data())
        depth_bgr = cv2.cvtColor(depth_img, cv2.COLOR_RGB2BGR)
        color_img = np.asanyarray(color.get_data())
        depth_raw = np.asanyarray(depth.get_data())
        return color_img, depth_raw, depth_bgr


def _get_realsense_serials() -> List[str]:
    ctx = rs.context()
    devices = ctx.query_devices()
    serials: List[str] = []
    for dev in devices:
        try:
            serials.append(dev.get_info(rs.camera_info.serial_number))
        except Exception:
            continue
    return serials


def bind_signal(sig: int, handler) -> None:
    try:
        signal.signal(sig, handler)
    except ValueError:
        # Signals are not available on all platforms (e.g., Windows), so ignore failure.
        pass


def _build_gui_message(start_flag: float) -> Float64MultiArray:
    msg = Float64MultiArray()
    msg.data = [float(start_flag), 0.0]
    return msg


def _select_realsense_serial() -> Optional[str]:
    serials = _get_realsense_serials()
    if not serials:
        raise SystemExit("No RealSense devices detected. Connect a camera to this PC.")
    serials = sorted(serials)
    if len(serials) > 1:
        print(f"[WARN] Multiple RealSense devices detected, using {serials[0]}.")
    return serials[0]


def main() -> None:
    session_dir = ensure_session_dir(_resolve_now(DEFAULT_TIMEZONE))
    print(f"Session directory: {session_dir}")

    rclpy.init()
    node = ControlNode_Turtle()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    trajectory_publisher = node.create_publisher(Float64MultiArray, "/trajectory_points", 10)

    stop_requested = threading.Event()

    def request_stop(*_args):
        stop_requested.set()

    bind_signal(signal.SIGINT, request_stop)
    bind_signal(signal.SIGTERM, request_stop)

    input("Press Enter to start the fixed trajectory run...")

    node.calibrate()

    serial = _select_realsense_serial()
    realsense = RealSenseSession(serial)
    realsense.start()

    trajectory_msg = Float64MultiArray()
    trajectory_msg.data = list(FIXED_TRAJECTORY)

    print(
        "Robot command issued. Recording RGB-D + telemetry...\n"
        f"Each trial runs for {TRIAL_DURATION_SEC:.1f} seconds (CTRL+C to abort)."
    )

    for trial_idx in range(TRIAL_COUNT):
        if stop_requested.is_set():
            break
        recorder = RGBDRecorder()
        run_start = time.time()
        start_time = _resolve_now(DEFAULT_TIMEZONE)
        trajectory_publisher.publish(trajectory_msg)
        node.publish_gui_information(_build_gui_message(start_flag=1.0))
        print(f"Starting trial {trial_idx + 1}/{TRIAL_COUNT}...")

        try:
            while not stop_requested.is_set():
                executor.spin_once(timeout_sec=0.0)
                color_img, depth_raw, _depth_bgr = realsense.poll()
                frame_time = time.time() - run_start
                node.update_force_data(True)
                recorder.write(color_img, depth_raw, frame_time)
                if frame_time >= TRIAL_DURATION_SEC:
                    break
        except RuntimeError as exc:
            print(f"RealSense stream error: {exc}")
            stop_requested.set()

        stop_time = _resolve_now(DEFAULT_TIMEZONE)
        node.publish_gui_information(_build_gui_message(start_flag=0.0))

        rgbd_payload = recorder.finalize()
        robot_state = build_robot_state(node)
        metadata = build_metadata(start_time, stop_time)
        payload = {
            "rgb": rgbd_payload["rgb"],
            "depth": rgbd_payload["depth"],
            "timestamps": rgbd_payload["timestamps"],
            "robot_state": robot_state,
            "metadata": metadata,
        }

        trial_path = session_dir / f"trial_{trial_idx + 1}.npy"
        np.save(trial_path, payload, allow_pickle=True)
        print(f"Saved trial data to {trial_path}")

    realsense.stop()

    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()

    print("Session complete.")
    print(f"Data stored under: {session_dir}")


if __name__ == "__main__":
    main()
