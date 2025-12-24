#!/usr/bin/env python3
"""Fixed-trajectory force data collection pipeline for terrain manipulation.

This script reuses the turtle control interfaces defined in
`LASSIE_GUI/lassie_gui.py` to orchestrate a force-feedback capture
session without modifying existing GUI code. It:

1. Starts the turtle ROS2 control node (as used by the GUI).
2. Waits for the operator to press Enter to begin.
3. Publishes a fixed waypoint trajectory on `/trajectory_points`.
4. Sends the GUI start flag on `/Gui_information`.
5. Logs force/telemetry data until the trajectory completes or the
   operator stops the run (Enter or CTRL+C).

The operator can stop early via Enter/CTRL+C, and the run will also stop
automatically once the turtle returns to the idle state.
"""

from __future__ import annotations

import json
import shutil
import signal
import subprocess
import sys
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Optional
import os

import rclpy
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Float64MultiArray

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
REMOTE_HOST = os.environ.get("TERRAIN_REMOTE_HOST", "qianlab@192.168.10.16")
REMOTE_DATA_ROOT = os.environ.get("TERRAIN_REMOTE_DATA_ROOT", "/home/qianlab/Turtle_workspace/lassie-turtle/highlevel/terrain_manipulation/data")
KEEP_LOCAL = os.environ.get("TERRAIN_KEEP_LOCAL", "0").lower() in ("1", "true", "yes")

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


def save_metadata(session_dir: Path, start_time: datetime, stop_time: datetime) -> None:
    payload = {
        "trajectory_points": FIXED_TRAJECTORY,
        "start_time": start_time.isoformat(),
        "stop_time": stop_time.isoformat(),
        "duration_sec": (stop_time - start_time).total_seconds(),
    }
    with open(session_dir / "metadata.json", "w", encoding="utf-8") as fh:
        json.dump(payload, fh, indent=2)


def save_robot_data(session_dir: Path, node: ControlNode_Turtle) -> None:
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
        fh.write(
            "scenario,real_time_plot,lateral_angle_rad,drag_speed_m_s,wiggle_time_s,servo_time_s,"
            "extraction_angle_deg,wiggle_frequency_hz,insertion_depth_m,wiggle_amplitude_m\n"
        )
        fh.write("turtle_fixed_traj,0,0,0,0,0,0,0,0,0\n")
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


def transfer_session(session_dir: Path) -> Optional[str]:
    if not REMOTE_HOST or not REMOTE_DATA_ROOT:
        print("Remote transfer disabled: missing host or data root.")
        return None
    remote_root = REMOTE_DATA_ROOT.rstrip("/")
    remote_session = f"{remote_root}/{session_dir.name}"
    try:
        subprocess.run(["ssh", REMOTE_HOST, "mkdir", "-p", remote_session], check=True)
        subprocess.run(["rsync", "-a", f"{session_dir}/", f"{REMOTE_HOST}:{remote_session}/"], check=True)
    except (subprocess.CalledProcessError, FileNotFoundError) as exc:
        print(f"Failed to transfer session to {REMOTE_HOST}:{remote_session}: {exc}")
        return None
    print(f"Session transferred to {REMOTE_HOST}:{remote_session}")
    return remote_session


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


def main() -> None:
    session_dir = ensure_session_dir(_resolve_now(DEFAULT_TIMEZONE))
    print(f"Session directory: {session_dir}")

    rclpy.init()
    node = ControlNode_Turtle()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    executor_stop = threading.Event()
    data_loop_running = threading.Event()

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

    trajectory_publisher = node.create_publisher(Float64MultiArray, "/trajectory_points", 10)

    stop_requested = threading.Event()

    def request_stop(*_args):
        stop_requested.set()

    bind_signal(signal.SIGINT, request_stop)
    bind_signal(signal.SIGTERM, request_stop)

    input("Press Enter to start the fixed trajectory run...")

    node.calibrate()

    trajectory_msg = Float64MultiArray()
    trajectory_msg.data = list(FIXED_TRAJECTORY)
    trajectory_publisher.publish(trajectory_msg)
    print(f"Published {len(FIXED_TRAJECTORY) // 3} waypoints to /trajectory_points.")

    start_time = _resolve_now(DEFAULT_TIMEZONE)
    data_loop_running.set()
    node.publish_gui_information(_build_gui_message(start_flag=1.0))
    print("Robot command issued. Recording force/telemetry...\nPress Enter again or use CTRL+C to stop.")

    def stop_listener():
        input()
        stop_requested.set()

    threading.Thread(target=stop_listener, daemon=True).start()

    has_moved = False
    while not stop_requested.is_set():
        if node.turtle_state != 0.0:
            has_moved = True
        elif has_moved and node.turtle_state == 0.0:
            stop_requested.set()
            break
        time.sleep(0.02)

    stop_time = _resolve_now(DEFAULT_TIMEZONE)

    data_loop_running.clear()
    node.publish_gui_information(_build_gui_message(start_flag=0.0))

    force_loop_stop.set()
    executor_stop.set()
    spin_thread.join(timeout=1.0)
    force_thread.join(timeout=1.0)

    save_metadata(session_dir, start_time, stop_time)
    save_robot_data(session_dir, node)

    remote_session = transfer_session(session_dir)
    if remote_session and not KEEP_LOCAL:
        try:
            shutil.rmtree(session_dir)
            print(f"Removed local session data at {session_dir}")
        except OSError as exc:
            print(f"Failed to remove local session data {session_dir}: {exc}")

    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()

    print("Session complete.")
    if remote_session and not KEEP_LOCAL:
        print(f"Data stored under: {REMOTE_HOST}:{remote_session}")
    elif remote_session:
        print(f"Data stored under: {session_dir} and {REMOTE_HOST}:{remote_session}")
    else:
        print(f"Data stored under: {session_dir}")


if __name__ == "__main__":
    main()
