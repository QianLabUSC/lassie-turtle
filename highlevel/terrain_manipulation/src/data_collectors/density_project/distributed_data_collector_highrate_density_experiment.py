#!/usr/bin/env python3
"""High-rate robot_state + UDP mocap logging with camera-time alignment.

Captures every /robot_state message (plus OptiTrack body + flipper poses)
at the native publish rate, then aligns those samples to RGB-D frame 
timestamps (from camera 0) after each trial. Output includes:
  - robot_state_raw: full-rate samples
  - robot_state: nearest-neighbor aligned to camera_time_0
  - mocap_raw: full-rate UDP mocap samples grouped by rigid body ID
  - mocap: nearest-neighbor aligned to camera_time_0
  - per-trial metadata and session-level metadata.json
"""

from __future__ import annotations

import argparse
import json
import math
import os
import pickle
import signal
import socket
import threading
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Pose
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, Float64MultiArray

try:
    import pyrealsense2 as rs
except ModuleNotFoundError as exc:  # pragma: no cover - defensive guard
    raise SystemExit(
        "pyrealsense2 is not installed.\n"
        "Install the Intel RealSense SDK Python bindings before running this program."
    ) from exc

SESSION_ROOT = Path(__file__).resolve().parents[2] / "data"
DEFAULT_TIMEZONE = os.environ.get("TERRAIN_TIMEZONE", "Etc/GMT+8")

STREAM_WIDTH = 848
STREAM_HEIGHT = 480
STREAM_FPS = 60
DEPTH_MIN_M = None
DEPTH_MAX_M = None
DEPTH_SCHEME = "jet"
DEPTH_HIST_EQ = False
DEPTH_POSTPROCESS = False
TRIAL_COUNT = 2
HEIGHT_CM = -1
# Dwell duration after /trajectory_complete before ending the trial record.
DWELL_TIME_S = 3.0
SAVE_RGB_MP4 = False
MOCAP_ENABLED = True
MOCAP_UDP_IP = "0.0.0.0"
MOCAP_UDP_PORT = 8000
MOCAP_PACKET_SIZE_BYTES = 4096
MOCAP_RB_NAMES = {
    2: "Empty Half Sphere",
    3: "Lead Half Sphere",
    6: "Steel Half Sphere",
    5: "Resin Half Sphere",
    8: "Sand Half Sphere",
}
# Orientation reference mode for roll/pitch/yaw:
#   "trial"   -> re-zero each trial (recommended if object is re-placed each trial)
#   "session" -> keep one reference for the whole run
MOCAP_REFERENCE_MODE = "session"
MOCAP_INCLINE_DEG = 0.0

TRAJ_SPEED_RAD_S = 2.0
SWEEPING_START_OFFSET_DEG = 45.0
ADDUCTION_START_OFFSET_DEG = -5.0

SWEEPING_START_OFFSET_RAD = math.radians(SWEEPING_START_OFFSET_DEG)
ADDUCTION_START_OFFSET_RAD = math.radians(ADDUCTION_START_OFFSET_DEG)

FIXED_TRAJECTORY = [
    0.0 + ADDUCTION_START_OFFSET_RAD,
    -0.53 + SWEEPING_START_OFFSET_RAD,
    TRAJ_SPEED_RAD_S,
    0.0 + ADDUCTION_START_OFFSET_RAD,
    -1.315 + SWEEPING_START_OFFSET_RAD,
    TRAJ_SPEED_RAD_S,
    0.785 + ADDUCTION_START_OFFSET_RAD,
    -1.315 + SWEEPING_START_OFFSET_RAD,
    TRAJ_SPEED_RAD_S,
    0.785 + ADDUCTION_START_OFFSET_RAD,
    -0.53 + SWEEPING_START_OFFSET_RAD,
    TRAJ_SPEED_RAD_S,
    0.785 + ADDUCTION_START_OFFSET_RAD,
    0.1 + SWEEPING_START_OFFSET_RAD,
    TRAJ_SPEED_RAD_S,
    0.0 + ADDUCTION_START_OFFSET_RAD,
    0.1 + SWEEPING_START_OFFSET_RAD,
    TRAJ_SPEED_RAD_S,
    0.0 + ADDUCTION_START_OFFSET_RAD,
    -0.53 + SWEEPING_START_OFFSET_RAD,
    TRAJ_SPEED_RAD_S,
]

TRAJECTORY_POINTS = list(FIXED_TRAJECTORY)


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


def _format_height_cm_label(height_cm: float) -> str:
    label = f"{float(height_cm):g}"
    return label.replace("-", "minus").replace(".", "p")


def ensure_session_dir(run_time: datetime, height_cm: float) -> Path:
    SESSION_ROOT.mkdir(parents=True, exist_ok=True)
    timestamp = run_time.strftime("%Y%m%d_%H%M%S")
    height_label = _format_height_cm_label(height_cm)
    session_dir = SESSION_ROOT / f"session_{timestamp}_height_{height_label}cm"
    session_dir.mkdir(parents=True, exist_ok=False)
    return session_dir


def save_session_metadata(
    session_dir: Path,
    start_time: datetime,
    stop_time: datetime,
    trials_planned: int,
    trials_completed: int,
    trial_duration_sec: float,
    dwell_time_sec: float,
    depth_scale_0: float,
    depth_scale_1: float,
    mocap_incline_deg: float,
    height_cm: float,
) -> None:
    payload = {
        "start_time": start_time.isoformat(),
        "stop_time": stop_time.isoformat(),
        "duration_sec": (stop_time - start_time).total_seconds(),
        "trials_planned": int(trials_planned),
        "trials_completed": int(trials_completed),
        "trial_duration_sec": float(trial_duration_sec),
        "dwell_time_sec": float(dwell_time_sec),
        "slope": 0,
        "initial_compaction": -1,
        "height_cm": float(height_cm),
        "image_resolution": [int(STREAM_WIDTH), int(STREAM_HEIGHT)],
        "fps": int(STREAM_FPS),
        "histogram_equalization": bool(DEPTH_HIST_EQ),
        "depth_scale_0": float(depth_scale_0),
        "depth_scale_1": float(depth_scale_1),
        "mocap_enabled": bool(MOCAP_ENABLED),
        "mocap_udp_ip": str(MOCAP_UDP_IP),
        "mocap_udp_port": int(MOCAP_UDP_PORT),
        "mocap_reference_mode": str(MOCAP_REFERENCE_MODE),
        "mocap_incline_deg": float(mocap_incline_deg),
        "mocap_rigid_body_names": {str(key): value for key, value in sorted(MOCAP_RB_NAMES.items())},
    }
    with open(session_dir / "metadata.json", "w", encoding="utf-8") as fh:
        json.dump(payload, fh, indent=2)


def build_metadata(
    start_time: datetime,
    stop_time: datetime,
    dwell_time_sec: float,
    mocap_incline_deg: float,
    height_cm: float,
    traj_complete_time_sec: Optional[float] = None,
    mocap_summary: Optional[Dict[str, object]] = None,
) -> Dict[str, object]:
    metadata = {
        "start_time": start_time.isoformat(),
        "stop_time": stop_time.isoformat(),
        "duration_sec": (stop_time - start_time).total_seconds(),
        "dwell_time_sec": float(dwell_time_sec),
        "height_cm": float(height_cm),
        "mocap_enabled": bool(MOCAP_ENABLED),
        "mocap_reference_mode": str(MOCAP_REFERENCE_MODE),
        "mocap_incline_deg": float(mocap_incline_deg),
    }
    if traj_complete_time_sec is not None:
        metadata["traj_complete_time_sec"] = float(traj_complete_time_sec)
    if mocap_summary is not None:
        metadata["mocap_summary"] = mocap_summary
    return metadata


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


def _try_set(opt_owner, option, value) -> None:
    try:
        opt_owner.set_option(option, value)
    except Exception:
        pass


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


def _open_rgb_writer(path: Path, fps: int, size: Tuple[int, int]) -> cv2.VideoWriter:
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    writer = cv2.VideoWriter(str(path), fourcc, float(fps), size)
    if not writer.isOpened():
        raise RuntimeError(f"Failed to open video writer for {path}")
    return writer


def _write_rgb_frame(writer: Optional[cv2.VideoWriter], frame: np.ndarray, size: Tuple[int, int]) -> None:
    if writer is None:
        return
    if frame.shape[1] != size[0] or frame.shape[0] != size[1]:
        frame = cv2.resize(frame, size, interpolation=cv2.INTER_AREA)
    writer.write(frame)


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


@dataclass
class MocapSample:
    time_s: float
    rigid_body_id: int
    position_x: float
    position_y: float
    position_z: float
    zeroed_position_x: float
    zeroed_position_y: float
    zeroed_position_z: float
    rotated_position_x: float
    rotated_position_y: float
    rotated_position_z: float
    rotated_zeroed_position_x: float
    rotated_zeroed_position_y: float
    rotated_zeroed_position_z: float
    orientation_x: float
    orientation_y: float
    orientation_z: float
    orientation_w: float
    zeroed_orientation_x: float
    zeroed_orientation_y: float
    zeroed_orientation_z: float
    zeroed_orientation_w: float
    rotated_orientation_x: float
    rotated_orientation_y: float
    rotated_orientation_z: float
    rotated_orientation_w: float
    rotated_zeroed_orientation_x: float
    rotated_zeroed_orientation_y: float
    rotated_zeroed_orientation_z: float
    rotated_zeroed_orientation_w: float
    roll_deg: float
    pitch_deg: float
    yaw_deg: float
    rotated_roll_deg: float
    rotated_pitch_deg: float
    rotated_yaw_deg: float


class MocapUDPReceiver:
    def __init__(self, udp_ip: str, udp_port: int, incline_deg: float = MOCAP_INCLINE_DEG) -> None:
        self.udp_ip = udp_ip
        self.udp_port = int(udp_port)
        self.incline_deg = float(incline_deg)
        # Clockwise y-z plane rotation equals a -x right-handed rotation.
        self._frame_rot = R.from_euler("x", -self.incline_deg, degrees=True)
        self._lock = threading.Lock()
        self._run_start = time.time()
        self._samples_by_rigid_body: Dict[int, List[MocapSample]] = {}
        self._initial_rot: Dict[int, R] = {}
        self._initial_pos: Dict[int, np.ndarray] = {}
        self._packets_received = 0
        self._decode_errors = 0
        self._stop_requested = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((self.udp_ip, self.udp_port))
        self._sock.settimeout(0.2)

    def start(self) -> None:
        if self._thread is not None:
            return
        self._thread = threading.Thread(target=self._recv_loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop_requested.set()
        try:
            self._sock.close()
        except OSError:
            pass
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None

    def reset(self, run_start: float, reset_reference: bool = True) -> None:
        with self._lock:
            self._run_start = run_start
            self._samples_by_rigid_body.clear()
            if reset_reference:
                self._initial_rot.clear()
                self._initial_pos.clear()
            self._packets_received = 0
            self._decode_errors = 0

    def snapshot(self) -> Dict[int, List[MocapSample]]:
        with self._lock:
            return {rigid_body_id: list(samples) for rigid_body_id, samples in self._samples_by_rigid_body.items()}

    def summary(self) -> Dict[str, object]:
        with self._lock:
            return {
                "packets_received": int(self._packets_received),
                "decode_errors": int(self._decode_errors),
                "rigid_body_ids": [int(rigid_body_id) for rigid_body_id in sorted(self._samples_by_rigid_body.keys())],
                "samples_per_rigid_body": {
                    str(rigid_body_id): int(len(samples))
                    for rigid_body_id, samples in sorted(self._samples_by_rigid_body.items())
                },
                "rigid_body_names": {str(key): value for key, value in sorted(MOCAP_RB_NAMES.items())},
                "udp_ip": self.udp_ip,
                "udp_port": self.udp_port,
                "incline_deg": float(self.incline_deg),
            }

    def _recv_loop(self) -> None:
        while not self._stop_requested.is_set():
            try:
                packet, _addr = self._sock.recvfrom(MOCAP_PACKET_SIZE_BYTES)
            except socket.timeout:
                continue
            except OSError:
                if self._stop_requested.is_set():
                    break
                continue
            self._handle_packet(packet)

    def _record_decode_error(self) -> None:
        with self._lock:
            self._decode_errors += 1

    def _handle_packet(self, packet: bytes) -> None:
        with self._lock:
            self._packets_received += 1

        try:
            rb = pickle.loads(packet)
        except Exception:
            self._record_decode_error()
            return

        if not hasattr(rb, "__len__") or len(rb) < 8:
            self._record_decode_error()
            return

        try:
            rigid_body_id = int(rb[0])
            quat = np.asarray(rb[1:5], dtype=float)
            pos = np.asarray(rb[5:8], dtype=float)
        except Exception:
            self._record_decode_error()
            return

        if quat.shape != (4,) or pos.shape != (3,):
            self._record_decode_error()
            return
        if not np.all(np.isfinite(quat)) or not np.all(np.isfinite(pos)):
            self._record_decode_error()
            return

        quat_norm = float(np.linalg.norm(quat))
        if quat_norm <= 1e-9:
            self._record_decode_error()
            return
        quat = quat / quat_norm

        try:
            current_rot = R.from_quat(quat)
        except Exception:
            self._record_decode_error()
            return

        with self._lock:
            if rigid_body_id not in self._initial_rot:
                self._initial_rot[rigid_body_id] = current_rot
                relative_rot = R.identity()
            else:
                # World-relative zeroed rotation: initial pose is treated as the
                # session reference frame, and deltas are expressed in fixed axes.
                relative_rot = current_rot * self._initial_rot[rigid_body_id].inv()
            if rigid_body_id not in self._initial_pos:
                self._initial_pos[rigid_body_id] = pos.copy()
            zeroed_pos = pos - self._initial_pos[rigid_body_id]
            zeroed_quat = relative_rot.as_quat()
            # Report RPY in fixed axes of the zeroed trial frame (extrinsic xyz).
            roll_deg, pitch_deg, yaw_deg = relative_rot.as_euler("xyz", degrees=True)
            rotated_pos = self._frame_rot.apply(pos)
            rotated_zeroed_pos = self._frame_rot.apply(zeroed_pos)
            # Express orientation in the rotated frame via basis change:
            # R' = F * R * F^{-1}
            rotated_rot = self._frame_rot * current_rot * self._frame_rot.inv()
            rotated_zeroed_rot = self._frame_rot * relative_rot * self._frame_rot.inv()
            rotated_quat = rotated_rot.as_quat()
            rotated_zeroed_quat = rotated_zeroed_rot.as_quat()
            rotated_roll_deg, rotated_pitch_deg, rotated_yaw_deg = rotated_zeroed_rot.as_euler("xyz", degrees=True)

            sample = MocapSample(
                time_s=time.time() - self._run_start,
                rigid_body_id=rigid_body_id,
                position_x=float(pos[0]),
                position_y=float(pos[1]),
                position_z=float(pos[2]),
                zeroed_position_x=float(zeroed_pos[0]),
                zeroed_position_y=float(zeroed_pos[1]),
                zeroed_position_z=float(zeroed_pos[2]),
                rotated_position_x=float(rotated_pos[0]),
                rotated_position_y=float(rotated_pos[1]),
                rotated_position_z=float(rotated_pos[2]),
                rotated_zeroed_position_x=float(rotated_zeroed_pos[0]),
                rotated_zeroed_position_y=float(rotated_zeroed_pos[1]),
                rotated_zeroed_position_z=float(rotated_zeroed_pos[2]),
                orientation_x=float(quat[0]),
                orientation_y=float(quat[1]),
                orientation_z=float(quat[2]),
                orientation_w=float(quat[3]),
                zeroed_orientation_x=float(zeroed_quat[0]),
                zeroed_orientation_y=float(zeroed_quat[1]),
                zeroed_orientation_z=float(zeroed_quat[2]),
                zeroed_orientation_w=float(zeroed_quat[3]),
                rotated_orientation_x=float(rotated_quat[0]),
                rotated_orientation_y=float(rotated_quat[1]),
                rotated_orientation_z=float(rotated_quat[2]),
                rotated_orientation_w=float(rotated_quat[3]),
                rotated_zeroed_orientation_x=float(rotated_zeroed_quat[0]),
                rotated_zeroed_orientation_y=float(rotated_zeroed_quat[1]),
                rotated_zeroed_orientation_z=float(rotated_zeroed_quat[2]),
                rotated_zeroed_orientation_w=float(rotated_zeroed_quat[3]),
                roll_deg=float(roll_deg),
                pitch_deg=float(pitch_deg),
                yaw_deg=float(yaw_deg),
                rotated_roll_deg=float(rotated_roll_deg),
                rotated_pitch_deg=float(rotated_pitch_deg),
                rotated_yaw_deg=float(rotated_yaw_deg),
            )
            self._samples_by_rigid_body.setdefault(rigid_body_id, []).append(sample)


class ControlNodeHighRate(Node):
    def __init__(self) -> None:
        super().__init__("control_node_highrate")
        self.publisher_ = self.create_publisher(Float64MultiArray, "/Gui_information", 10)
        self.create_subscription(Float64MultiArray, "/robot_state", self._robot_state_cb, 10)
        self.create_subscription(Bool, "/trajectory_complete", self._traj_complete_cb, 10)
        self.create_subscription(Pose, "/optitrack_body", self._optitrack_cb, 10)
        self.create_subscription(Pose, "/optitrack_left_flipper", self._left_flipper_cb, 10)
        self.create_subscription(Pose, "/optitrack_right_flipper", self._right_flipper_cb, 10)
        self.run_start = time.time()
        self._lock = threading.Lock()
        self._samples: List[RobotStateSample] = []
        self._traj_complete = False
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
            self._traj_complete = False

    def _traj_complete_cb(self, msg: Bool) -> None:
        with self._lock:
            self._traj_complete = bool(msg.data)

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

    def is_traj_complete(self) -> bool:
        with self._lock:
            return self._traj_complete


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


def _build_mocap_state(samples_by_rigid_body: Dict[int, List[MocapSample]]) -> Dict[str, Dict[str, np.ndarray]]:
    mocap_state: Dict[str, Dict[str, np.ndarray]] = {}
    for rigid_body_id, samples in sorted(samples_by_rigid_body.items()):
        key = str(rigid_body_id)
        mocap_state[key] = {
            "time": np.asarray([s.time_s for s in samples], dtype=float),
            "rigid_body_id": np.asarray([s.rigid_body_id for s in samples], dtype=float),
            "position_x": np.asarray([s.position_x for s in samples], dtype=float),
            "position_y": np.asarray([s.position_y for s in samples], dtype=float),
            "position_z": np.asarray([s.position_z for s in samples], dtype=float),
            "zeroed_position_x": np.asarray([s.zeroed_position_x for s in samples], dtype=float),
            "zeroed_position_y": np.asarray([s.zeroed_position_y for s in samples], dtype=float),
            "zeroed_position_z": np.asarray([s.zeroed_position_z for s in samples], dtype=float),
            "rotated_position_x": np.asarray([s.rotated_position_x for s in samples], dtype=float),
            "rotated_position_y": np.asarray([s.rotated_position_y for s in samples], dtype=float),
            "rotated_position_z": np.asarray([s.rotated_position_z for s in samples], dtype=float),
            "rotated_zeroed_position_x": np.asarray([s.rotated_zeroed_position_x for s in samples], dtype=float),
            "rotated_zeroed_position_y": np.asarray([s.rotated_zeroed_position_y for s in samples], dtype=float),
            "rotated_zeroed_position_z": np.asarray([s.rotated_zeroed_position_z for s in samples], dtype=float),
            "orientation_x": np.asarray([s.orientation_x for s in samples], dtype=float),
            "orientation_y": np.asarray([s.orientation_y for s in samples], dtype=float),
            "orientation_z": np.asarray([s.orientation_z for s in samples], dtype=float),
            "orientation_w": np.asarray([s.orientation_w for s in samples], dtype=float),
            # Zeroed orientation (initial pose treated as identity/world-aligned).
            "zeroed_orientation_x": np.asarray([s.zeroed_orientation_x for s in samples], dtype=float),
            "zeroed_orientation_y": np.asarray([s.zeroed_orientation_y for s in samples], dtype=float),
            "zeroed_orientation_z": np.asarray([s.zeroed_orientation_z for s in samples], dtype=float),
            "zeroed_orientation_w": np.asarray([s.zeroed_orientation_w for s in samples], dtype=float),
            "rotated_orientation_x": np.asarray([s.rotated_orientation_x for s in samples], dtype=float),
            "rotated_orientation_y": np.asarray([s.rotated_orientation_y for s in samples], dtype=float),
            "rotated_orientation_z": np.asarray([s.rotated_orientation_z for s in samples], dtype=float),
            "rotated_orientation_w": np.asarray([s.rotated_orientation_w for s in samples], dtype=float),
            "rotated_zeroed_orientation_x": np.asarray([s.rotated_zeroed_orientation_x for s in samples], dtype=float),
            "rotated_zeroed_orientation_y": np.asarray([s.rotated_zeroed_orientation_y for s in samples], dtype=float),
            "rotated_zeroed_orientation_z": np.asarray([s.rotated_zeroed_orientation_z for s in samples], dtype=float),
            "rotated_zeroed_orientation_w": np.asarray([s.rotated_zeroed_orientation_w for s in samples], dtype=float),
            "roll_deg": np.asarray([s.roll_deg for s in samples], dtype=float),
            "pitch_deg": np.asarray([s.pitch_deg for s in samples], dtype=float),
            "yaw_deg": np.asarray([s.yaw_deg for s in samples], dtype=float),
            "rotated_roll_deg": np.asarray([s.rotated_roll_deg for s in samples], dtype=float),
            "rotated_pitch_deg": np.asarray([s.rotated_pitch_deg for s in samples], dtype=float),
            "rotated_yaw_deg": np.asarray([s.rotated_yaw_deg for s in samples], dtype=float),
        }
    return mocap_state


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


def _align_mocap_state(
    mocap_state: Dict[str, Dict[str, np.ndarray]],
    camera_times: np.ndarray,
) -> Dict[str, Dict[str, np.ndarray]]:
    aligned_mocap: Dict[str, Dict[str, np.ndarray]] = {}
    for rigid_body_id, rigid_body_state in mocap_state.items():
        sample_times = rigid_body_state.get("time")
        if not isinstance(sample_times, np.ndarray) or sample_times.size == 0:
            continue
        indices = _nearest_indices(sample_times, camera_times)
        aligned_state: Dict[str, np.ndarray] = {"time": sample_times[indices]}
        for key, values in rigid_body_state.items():
            if key == "time":
                continue
            aligned_state[key] = values[indices]
        aligned_mocap[rigid_body_id] = aligned_state
    return aligned_mocap


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(description="High-rate robot_state logging with camera alignment.")
    ap.add_argument("--trials", type=int, default=TRIAL_COUNT, help="Number of trials to record.")
    ap.add_argument(
        "--height-cm",
        type=float,
        default=HEIGHT_CM,
        help="Physically set experiment height in cm; used for run naming and metadata only.",
    )
    ap.add_argument(
        "--incline-deg",
        type=float,
        default=MOCAP_INCLINE_DEG,
        help="Clockwise y-z plane rotation angle (deg) used to save rotated mocap position/orientation.",
    )
    ap.add_argument(
        "--save-rgb-mp4",
        action="store_true",
        default=SAVE_RGB_MP4,
        help="Save RGB MP4 videos for both cameras.",
    )
    return ap.parse_args()


def main() -> int:
    args = parse_args()
    session_dir = ensure_session_dir(_resolve_now(DEFAULT_TIMEZONE), height_cm=float(args.height_cm))
    print(f"Session directory: {session_dir}")
    print(f"Experiment height: {float(args.height_cm):g} cm")

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
    depth_scale_0 = (
        realsense_primary.pipeline.get_active_profile()
        .get_device()
        .first_depth_sensor()
        .get_depth_scale()
    )
    depth_scale_1 = (
        realsense_secondary.pipeline.get_active_profile()
        .get_device()
        .first_depth_sensor()
        .get_depth_scale()
    )

    trajectory_msg = Float64MultiArray()
    trajectory_msg.data = list(TRAJECTORY_POINTS)
    trajectory_publisher = node.create_publisher(Float64MultiArray, "/trajectory_points", 10)
    mocap_receiver: Optional[MocapUDPReceiver] = None
    if MOCAP_ENABLED:
        try:
            mocap_receiver = MocapUDPReceiver(MOCAP_UDP_IP, MOCAP_UDP_PORT, incline_deg=float(args.incline_deg))
            mocap_receiver.start()
            print(
                f"Mocap UDP listener active on {MOCAP_UDP_IP}:{MOCAP_UDP_PORT} "
                f"(incline={float(args.incline_deg):.3f} deg)"
            )
        except OSError as exc:
            raise SystemExit(
                f"Failed to bind mocap UDP listener on {MOCAP_UDP_IP}:{MOCAP_UDP_PORT}: {exc}"
            ) from exc

    print(
        "Robot command issued. Recording RGB-D + high-rate telemetry...\n"
        "Each trial runs until /trajectory_complete is received (CTRL+C to abort)."
    )

    session_start_time = _resolve_now(DEFAULT_TIMEZONE)
    trials_completed = 0
    trial_durations: List[float] = []

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
        if mocap_receiver is not None:
            reset_reference = (MOCAP_REFERENCE_MODE == "trial") or (trial_idx == 0)
            mocap_receiver.reset(run_start, reset_reference=reset_reference)
        start_time = _resolve_now(DEFAULT_TIMEZONE)
        print(f"Starting trial {trial_idx + 1}/{args.trials}...")
        trajectory_publisher.publish(trajectory_msg)
        traj_complete_time_s: Optional[float] = None

        try:
            while not stop_requested.is_set():
                color_img, depth_raw, _depth_bgr = realsense_primary.poll()
                color_img_2, depth_raw_2, _depth_bgr_2 = realsense_secondary.poll()
                frame_time = time.time() - run_start
                recorder.write(color_img, depth_raw, frame_time)
                recorder_2.write(color_img_2, depth_raw_2, frame_time)
                _write_rgb_frame(rgb_writer_0, color_img, rgb_size)
                _write_rgb_frame(rgb_writer_1, color_img_2, rgb_size)
                if node.is_traj_complete():
                    if traj_complete_time_s is None:
                        traj_complete_time_s = frame_time
                        if DWELL_TIME_S > 0.0:
                            print(
                                "Trajectory complete received; "
                                f"recording dwell for {DWELL_TIME_S:.2f} second(s)..."
                            )
                    if frame_time - traj_complete_time_s >= DWELL_TIME_S:
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
        trial_duration_sec = (stop_time - start_time).total_seconds()

        rgbd_payload = recorder.finalize()
        rgbd_payload_2 = recorder_2.finalize()
        robot_state_raw = _build_robot_state(node.snapshot())
        robot_state_aligned = _align_robot_state(robot_state_raw, rgbd_payload["timestamps"])
        if mocap_receiver is not None:
            mocap_raw = _build_mocap_state(mocap_receiver.snapshot())
            mocap_aligned = _align_mocap_state(mocap_raw, rgbd_payload["timestamps"])
            mocap_summary = mocap_receiver.summary()
        else:
            mocap_raw = {}
            mocap_aligned = {}
            mocap_summary = {
                "packets_received": 0,
                "decode_errors": 0,
                "rigid_body_ids": [],
                "samples_per_rigid_body": {},
                "rigid_body_names": {},
                "udp_ip": "",
                "udp_port": 0,
            }
        metadata = build_metadata(
            start_time,
            stop_time,
            DWELL_TIME_S,
            mocap_incline_deg=float(args.incline_deg),
            height_cm=float(args.height_cm),
            traj_complete_time_sec=traj_complete_time_s,
            mocap_summary=mocap_summary,
        )
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
            "mocap_raw": mocap_raw,
            "mocap": mocap_aligned,
            "metadata": metadata,
        }

        trial_path = session_dir / f"trial_{trial_idx + 1}.npy"
        np.save(trial_path, payload, allow_pickle=True)
        print(f"Saved trial data to {trial_path}")
        trials_completed += 1
        trial_durations.append(trial_duration_sec)

    # Send one final stop command when exiting so motor control is released
    node.publish_gui_information(_build_gui_message(start_flag=0.0))
    
    realsense_primary.stop()
    realsense_secondary.stop()
    if mocap_receiver is not None:
        mocap_receiver.stop()

    stop_requested.set()
    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()

    session_stop_time = _resolve_now(DEFAULT_TIMEZONE)
    duration_sec = (session_stop_time - session_start_time).total_seconds()
    avg_trial_duration = float(sum(trial_durations) / len(trial_durations)) if trial_durations else 0.0
    save_session_metadata(
        session_dir,
        session_start_time,
        session_stop_time,
        args.trials,
        trials_completed,
        avg_trial_duration,
        DWELL_TIME_S,
        depth_scale_0,
        depth_scale_1,
        mocap_incline_deg=float(args.incline_deg),
        height_cm=float(args.height_cm),
    )
    print(f"Completed {trials_completed} trial(s) in {duration_sec:.1f} seconds.")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())


# there is a small jump between trials to save stuff and to send the trajectory command again. if we need uninterrupted recording, i can chnage the whole thing so we save only one file.
# also make sure copy mocap scripts from pc
## I need to think about RPY. I think the orientatins are coupled. 
# i think ppl changed the location of cameras. need to readjust. having issue with mocap
# to expediate data collection, maybe I should make a plotting script so I dont have to wait for the full render to check the accuracy of mocap etc.
# why is the delay increasing?


# the plots for sand vs resin doesn't make sense. recalculate resin with correct COM or redo (same with steel) and observe.
# also check sand videos for validation.


# check rendered vbideos for resin and steel to see what changed vs previous time.
