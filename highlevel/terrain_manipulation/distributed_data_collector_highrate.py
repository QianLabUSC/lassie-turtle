#!/usr/bin/env python3
"""High-rate robot_state logging with camera-time alignment.

Captures every /robot_state message (plus OptiTrack body + flipper poses)
at the native publish rate, then aligns those samples to RGB-D frame 
timestamps (from camera 0) after each trial. Output includes:
  - robot_state_raw: full-rate samples
  - robot_state: nearest-neighbor aligned to camera_time_0
  - per-trial metadata and session-level metadata.json
"""

from __future__ import annotations

import argparse
import signal
import threading
import time
from dataclasses import dataclass
from typing import Dict, List, Optional

import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from distributed_data_collector import (  # type: ignore
    DEFAULT_TIMEZONE,
    SAVE_RGB_MP4,
    STREAM_FPS,
    STREAM_HEIGHT,
    STREAM_WIDTH,
    TRIAL_COUNT,
    TRIAL_DURATION_SEC,
    TRAJECTORY_POINTS,
    RGBDRecorder,
    RealSenseSession,
    bind_signal,
    _build_gui_message,
    _get_realsense_serials,
    _open_rgb_writer,
    _resolve_now,
    _write_rgb_frame,
    build_metadata,
    ensure_session_dir,
    save_session_metadata,
)


@dataclass
class RobotStateSample:
    time_s: float
    turtle_state: float
    leftadduction_pos: float
    leftsweeping_pos: float
    rightadduction_pos: float
    rightsweeping_pos: float
    leftadduction_curr: float
    leftsweeping_curr: float
    rightadduction_curr: float
    rightsweeping_curr: float
    optitrack_position_x: float
    optitrack_position_y: float
    optitrack_position_z: float
    optitrack_orientation_x: float
    optitrack_orientation_y: float
    optitrack_orientation_z: float
    optitrack_orientation_w: float
    left_flipper_position_x: float
    left_flipper_position_y: float
    left_flipper_position_z: float
    left_flipper_orientation_x: float
    left_flipper_orientation_y: float
    left_flipper_orientation_z: float
    left_flipper_orientation_w: float
    right_flipper_position_x: float
    right_flipper_position_y: float
    right_flipper_position_z: float
    right_flipper_orientation_x: float
    right_flipper_orientation_y: float
    right_flipper_orientation_z: float
    right_flipper_orientation_w: float


class ControlNodeHighRate(Node):
    def __init__(self) -> None:
        super().__init__("control_node_highrate")
        self.publisher_ = self.create_publisher(Float64MultiArray, "/Gui_information", 10)
        self.create_subscription(Float64MultiArray, "/robot_state", self._robot_state_cb, 10)
        self.create_subscription(Pose, "/optitrack_body", self._optitrack_cb, 10)
        self.create_subscription(Pose, "/optitrack_left_flipper", self._left_flipper_cb, 10)
        self.create_subscription(Pose, "/optitrack_right_flipper", self._right_flipper_cb, 10)
        self.run_start = time.time()
        self._lock = threading.Lock()
        self._samples: List[RobotStateSample] = []
        self._optitrack_pose: Optional[Pose] = None
        self._left_flipper_pose: Optional[Pose] = None
        self._right_flipper_pose: Optional[Pose] = None

    def publish_gui_information(self, msg: Float64MultiArray) -> None:
        self.publisher_.publish(msg)

    def reset(self, run_start: float) -> None:
        with self._lock:
            self.run_start = run_start
            self._samples.clear()
            self._optitrack_pose = None
            self._left_flipper_pose = None
            self._right_flipper_pose = None

    def _optitrack_cb(self, msg: Pose) -> None:
        with self._lock:
            self._optitrack_pose = msg

    def _left_flipper_cb(self, msg: Pose) -> None:
        with self._lock:
            self._left_flipper_pose = msg

    def _right_flipper_cb(self, msg: Pose) -> None:
        with self._lock:
            self._right_flipper_pose = msg

    def _robot_state_cb(self, msg: Float64MultiArray) -> None:
        now_s = time.time() - self.run_start
        data = msg.data
        if len(data) < 9:
            return
        with self._lock:
            optitrack_pose = self._optitrack_pose
            left_flipper_pose = self._left_flipper_pose
            right_flipper_pose = self._right_flipper_pose

        sample = RobotStateSample(
            time_s=now_s,
            turtle_state=data[0],
            leftadduction_pos=data[1],
            leftsweeping_pos=data[2],
            rightadduction_pos=data[3],
            rightsweeping_pos=data[4],
            leftadduction_curr=data[5],
            leftsweeping_curr=data[6],
            rightadduction_curr=data[7],
            rightsweeping_curr=data[8],
            optitrack_position_x=0.0 if optitrack_pose is None else optitrack_pose.position.x,
            optitrack_position_y=0.0 if optitrack_pose is None else optitrack_pose.position.y,
            optitrack_position_z=0.0 if optitrack_pose is None else optitrack_pose.position.z,
            optitrack_orientation_x=0.0 if optitrack_pose is None else optitrack_pose.orientation.x,
            optitrack_orientation_y=0.0 if optitrack_pose is None else optitrack_pose.orientation.y,
            optitrack_orientation_z=0.0 if optitrack_pose is None else optitrack_pose.orientation.z,
            optitrack_orientation_w=1.0 if optitrack_pose is None else optitrack_pose.orientation.w,
            left_flipper_position_x=0.0 if left_flipper_pose is None else left_flipper_pose.position.x,
            left_flipper_position_y=0.0 if left_flipper_pose is None else left_flipper_pose.position.y,
            left_flipper_position_z=0.0 if left_flipper_pose is None else left_flipper_pose.position.z,
            left_flipper_orientation_x=0.0 if left_flipper_pose is None else left_flipper_pose.orientation.x,
            left_flipper_orientation_y=0.0 if left_flipper_pose is None else left_flipper_pose.orientation.y,
            left_flipper_orientation_z=0.0 if left_flipper_pose is None else left_flipper_pose.orientation.z,
            left_flipper_orientation_w=1.0 if left_flipper_pose is None else left_flipper_pose.orientation.w,
            right_flipper_position_x=0.0 if right_flipper_pose is None else right_flipper_pose.position.x,
            right_flipper_position_y=0.0 if right_flipper_pose is None else right_flipper_pose.position.y,
            right_flipper_position_z=0.0 if right_flipper_pose is None else right_flipper_pose.position.z,
            right_flipper_orientation_x=0.0 if right_flipper_pose is None else right_flipper_pose.orientation.x,
            right_flipper_orientation_y=0.0 if right_flipper_pose is None else right_flipper_pose.orientation.y,
            right_flipper_orientation_z=0.0 if right_flipper_pose is None else right_flipper_pose.orientation.z,
            right_flipper_orientation_w=1.0 if right_flipper_pose is None else right_flipper_pose.orientation.w,
        )
        with self._lock:
            self._samples.append(sample)

    def snapshot(self) -> List[RobotStateSample]:
        with self._lock:
            return list(self._samples)


def _build_robot_state(samples: List[RobotStateSample]) -> Dict[str, np.ndarray]:
    return {
        "time": np.asarray([s.time_s for s in samples], dtype=float),
        "turtle_state": np.asarray([s.turtle_state for s in samples], dtype=float),
        "leftadduction_pos": np.asarray([s.leftadduction_pos for s in samples], dtype=float),
        "leftsweeping_pos": np.asarray([s.leftsweeping_pos for s in samples], dtype=float),
        "rightadduction_pos": np.asarray([s.rightadduction_pos for s in samples], dtype=float),
        "rightsweeping_pos": np.asarray([s.rightsweeping_pos for s in samples], dtype=float),
        "leftadduction_curr": np.asarray([s.leftadduction_curr for s in samples], dtype=float),
        "leftsweeping_curr": np.asarray([s.leftsweeping_curr for s in samples], dtype=float),
        "rightadduction_curr": np.asarray([s.rightadduction_curr for s in samples], dtype=float),
        "rightsweeping_curr": np.asarray([s.rightsweeping_curr for s in samples], dtype=float),
        "OptitrackPosition_x": np.asarray([s.optitrack_position_x for s in samples], dtype=float),
        "OptitrackPosition_y": np.asarray([s.optitrack_position_y for s in samples], dtype=float),
        "OptitrackPosition_z": np.asarray([s.optitrack_position_z for s in samples], dtype=float),
        "OptitrackOrientation_x": np.asarray([s.optitrack_orientation_x for s in samples], dtype=float),
        "OptitrackOrientation_y": np.asarray([s.optitrack_orientation_y for s in samples], dtype=float),
        "OptitrackOrientation_z": np.asarray([s.optitrack_orientation_z for s in samples], dtype=float),
        "OptitrackOrientation_w": np.asarray([s.optitrack_orientation_w for s in samples], dtype=float),
        "LeftFlipperPosition_x": np.asarray([s.left_flipper_position_x for s in samples], dtype=float),
        "LeftFlipperPosition_y": np.asarray([s.left_flipper_position_y for s in samples], dtype=float),
        "LeftFlipperPosition_z": np.asarray([s.left_flipper_position_z for s in samples], dtype=float),
        "LeftFlipperOrientation_x": np.asarray([s.left_flipper_orientation_x for s in samples], dtype=float),
        "LeftFlipperOrientation_y": np.asarray([s.left_flipper_orientation_y for s in samples], dtype=float),
        "LeftFlipperOrientation_z": np.asarray([s.left_flipper_orientation_z for s in samples], dtype=float),
        "LeftFlipperOrientation_w": np.asarray([s.left_flipper_orientation_w for s in samples], dtype=float),
        "RightFlipperPosition_x": np.asarray([s.right_flipper_position_x for s in samples], dtype=float),
        "RightFlipperPosition_y": np.asarray([s.right_flipper_position_y for s in samples], dtype=float),
        "RightFlipperPosition_z": np.asarray([s.right_flipper_position_z for s in samples], dtype=float),
        "RightFlipperOrientation_x": np.asarray([s.right_flipper_orientation_x for s in samples], dtype=float),
        "RightFlipperOrientation_y": np.asarray([s.right_flipper_orientation_y for s in samples], dtype=float),
        "RightFlipperOrientation_z": np.asarray([s.right_flipper_orientation_z for s in samples], dtype=float),
        "RightFlipperOrientation_w": np.asarray([s.right_flipper_orientation_w for s in samples], dtype=float),
    }


def _nearest_indices(sample_times: np.ndarray, query_times: np.ndarray) -> np.ndarray:
    if sample_times.size == 0:
        return np.zeros_like(query_times, dtype=int)
    idx = np.searchsorted(sample_times, query_times, side="left")
    idx = np.clip(idx, 0, sample_times.size - 1)
    prev_idx = np.clip(idx - 1, 0, sample_times.size - 1)
    next_idx = idx
    prev_diff = np.abs(query_times - sample_times[prev_idx])
    next_diff = np.abs(query_times - sample_times[next_idx])
    choose_prev = prev_diff <= next_diff
    return np.where(choose_prev, prev_idx, next_idx)


def _align_robot_state(robot_state: Dict[str, np.ndarray], camera_times: np.ndarray) -> Dict[str, np.ndarray]:
    sample_times = robot_state["time"]
    indices = _nearest_indices(sample_times, camera_times)
    aligned: Dict[str, np.ndarray] = {"time": sample_times[indices]}
    for key, values in robot_state.items():
        if key == "time":
            continue
        aligned[key] = values[indices]
    return aligned


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(description="High-rate robot_state logging with camera alignment.")
    ap.add_argument("--trials", type=int, default=TRIAL_COUNT, help="Number of trials to record.")
    ap.add_argument(
        "--save-rgb-mp4",
        action="store_true",
        default=SAVE_RGB_MP4,
        help="Save RGB MP4 videos for both cameras.",
    )
    return ap.parse_args()


def main() -> int:
    args = parse_args()
    session_dir = ensure_session_dir(_resolve_now(DEFAULT_TIMEZONE))
    print(f"Session directory: {session_dir}")

    rclpy.init()
    node = ControlNodeHighRate()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    stop_requested = threading.Event()

    def executor_thread() -> None:
        while not stop_requested.is_set():
            executor.spin_once(timeout_sec=0.1)

    spin_thread = threading.Thread(target=executor_thread, daemon=True)
    spin_thread.start()

    serials = _get_realsense_serials()
    if len(serials) < 2:
        raise SystemExit("Need two RealSense devices connected for high-rate collection.")
    realsense_primary = RealSenseSession(serials[0])
    realsense_secondary = RealSenseSession(serials[1])
    realsense_primary.start()
    realsense_secondary.start()
    depth_scale = (
        realsense_primary.pipeline.get_active_profile()
        .get_device()
        .first_depth_sensor()
        .get_depth_scale()
    )

    trajectory_msg = Float64MultiArray()
    trajectory_msg.data = list(TRAJECTORY_POINTS)
    trajectory_publisher = node.create_publisher(Float64MultiArray, "/trajectory_points", 10)

    print(
        "Robot command issued. Recording RGB-D + high-rate telemetry...\n"
        f"Each trial runs for {TRIAL_DURATION_SEC:.1f} seconds (CTRL+C to abort)."
    )

    session_start_time = _resolve_now(DEFAULT_TIMEZONE)
    trials_completed = 0

    def request_stop(*_args) -> None:
        stop_requested.set()

    bind_signal(signal.SIGINT, request_stop)
    bind_signal(signal.SIGTERM, request_stop)

    input("Press Enter to start the fixed trajectory run...")

    for trial_idx in range(args.trials):
        if stop_requested.is_set():
            break
        recorder = RGBDRecorder()
        recorder_2 = RGBDRecorder()
        rgb_size = (STREAM_WIDTH, STREAM_HEIGHT)
        rgb_writer_0: Optional[object] = None
        rgb_writer_1: Optional[object] = None
        if args.save_rgb_mp4:
            rgb_path_0 = session_dir / f"trial_{trial_idx + 1}_rgb_0.mp4"
            rgb_path_1 = session_dir / f"trial_{trial_idx + 1}_rgb_1.mp4"
            rgb_writer_0 = _open_rgb_writer(rgb_path_0, STREAM_FPS, rgb_size)
            rgb_writer_1 = _open_rgb_writer(rgb_path_1, STREAM_FPS, rgb_size)

        run_start = time.time()
        node.reset(run_start)
        start_time = _resolve_now(DEFAULT_TIMEZONE)
        trajectory_publisher.publish(trajectory_msg)
        node.publish_gui_information(_build_gui_message(start_flag=1.0))
        print(f"Starting trial {trial_idx + 1}/{TRIAL_COUNT}...")

        try:
            while not stop_requested.is_set():
                color_img, depth_raw, _depth_bgr = realsense_primary.poll()
                color_img_2, depth_raw_2, _depth_bgr_2 = realsense_secondary.poll()
                frame_time = time.time() - run_start
                recorder.write(color_img, depth_raw, frame_time)
                recorder_2.write(color_img_2, depth_raw_2, frame_time)
                _write_rgb_frame(rgb_writer_0, color_img, rgb_size)
                _write_rgb_frame(rgb_writer_1, color_img_2, rgb_size)
                if frame_time >= TRIAL_DURATION_SEC:
                    break
        except RuntimeError as exc:
            print(f"RealSense stream error: {exc}")
            stop_requested.set()
        finally:
            if rgb_writer_0 is not None:
                rgb_writer_0.release()
            if rgb_writer_1 is not None:
                rgb_writer_1.release()

        stop_time = _resolve_now(DEFAULT_TIMEZONE)
        node.publish_gui_information(_build_gui_message(start_flag=0.0))

        rgbd_payload = recorder.finalize()
        rgbd_payload_2 = recorder_2.finalize()
        robot_state_raw = _build_robot_state(node.snapshot())
        robot_state_aligned = _align_robot_state(robot_state_raw, rgbd_payload["timestamps"])
        metadata = build_metadata(start_time, stop_time)
        payload = {
            "rgb_0": rgbd_payload["rgb"],
            "depth_0": rgbd_payload["depth"],
            "camera_time_0": rgbd_payload["timestamps"],
            "rgb_1": rgbd_payload_2["rgb"],
            "depth_1": rgbd_payload_2["depth"],
            "camera_time_1": rgbd_payload_2["timestamps"],
            "trajectory_points": np.asarray(TRAJECTORY_POINTS, dtype=float),
            "robot_state_raw": robot_state_raw,
            "robot_state": robot_state_aligned,
            "metadata": metadata,
        }

        trial_path = session_dir / f"trial_{trial_idx + 1}.npy"
        np.save(trial_path, payload, allow_pickle=True)
        print(f"Saved trial data to {trial_path}")
        trials_completed += 1

    realsense_primary.stop()
    realsense_secondary.stop()

    stop_requested.set()
    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()

    session_stop_time = _resolve_now(DEFAULT_TIMEZONE)
    duration_sec = (session_stop_time - session_start_time).total_seconds()
    save_session_metadata(
        session_dir,
        session_start_time,
        session_stop_time,
        args.trials,
        trials_completed,
        TRIAL_DURATION_SEC,
        depth_scale,
    )
    print(f"Completed {trials_completed} trial(s) in {duration_sec:.1f} seconds.")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
