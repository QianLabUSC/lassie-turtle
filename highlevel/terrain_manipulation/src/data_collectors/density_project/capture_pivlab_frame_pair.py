#!/usr/bin/env python3
"""Capture a timed grayscale RealSense image sequence for PIVLab.

The script starts one RealSense color stream, waits for a manual trigger, then
publishes the fixed right-flipper trajectory and saves grayscale PNG images.
Optional prepositioning can move the flipper to the first waypoint before the
real trajectory starts.
"""

from __future__ import annotations

import argparse
import math
import time
from datetime import datetime
from pathlib import Path
from typing import List, Optional, Tuple

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

DEFAULT_SWEEPING_OFFSET_DEG = 100.0
DEFAULT_TRAJECTORY_SPEED_RAD_S = 2.0
DEFAULT_TRAJECTORY_PUBLISH_S = 0.5
DEFAULT_TRAJECTORY_TIMEOUT_S = 20.0
DEFAULT_POSITION_PRINT_INTERVAL_S = 0.25
DEFAULT_WAIT_TRAJECTORY_COMPLETE_AFTER_CAPTURE = True

DEFAULT_PREPOSITION = False
DEFAULT_PREPOSITION_SPEED_RAD_S = 2.0
DEFAULT_PREPOSITION_TOLERANCE_RAD = math.radians(2.0)
DEFAULT_PREPOSITION_TIMEOUT_S = 8.0

RIGHT_ADDUCTION_STATE_INDEX = 3
RIGHT_SWEEPING_STATE_INDEX = 4

BASE_TRAJECTORY_WAYPOINTS = [
    (0.0, -0.53),
    (0.0, -1.315),
    (0.785, -1.315),
    (0.785, -0.53),
    (0.785, 0.1),
    (0.0, 0.1),
    (0.0, -0.53),
]


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


def _timestamp_label() -> str:
    return datetime.now().strftime("%Y%m%d_%H%M%S_%f")


def _build_fixed_trajectory(sweeping_offset_deg: float, speed_rad_s: float) -> List[float]:
    sweeping_offset_rad = math.radians(float(sweeping_offset_deg))
    trajectory: List[float] = []
    for adduction_rad, sweeping_rad in BASE_TRAJECTORY_WAYPOINTS:
        trajectory.extend(
            [
                float(adduction_rad),
                float(sweeping_rad + sweeping_offset_rad),
                float(speed_rad_s),
            ]
        )
    return trajectory


def _trajectory_start(trajectory: List[float]) -> Tuple[float, float]:
    if len(trajectory) < 3:
        raise RuntimeError("Trajectory must contain at least one waypoint.")
    return (float(trajectory[0]), float(trajectory[1]))


def _trajectory_end(trajectory: List[float]) -> Tuple[float, float]:
    if len(trajectory) < 3:
        raise RuntimeError("Trajectory must contain at least one waypoint.")
    return (float(trajectory[-3]), float(trajectory[-2]))


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


def _right_flipper_state_to_radians(robot_state: List[float]) -> Tuple[float, float]:
    if len(robot_state) <= RIGHT_SWEEPING_STATE_INDEX:
        raise RuntimeError(
            "Incomplete /robot_state received; expected right adduction/sweeping position fields."
        )
    right_adduction_turns = float(robot_state[RIGHT_ADDUCTION_STATE_INDEX])
    right_sweeping_turns = float(robot_state[RIGHT_SWEEPING_STATE_INDEX])
    return (
        -2.0 * math.pi * right_adduction_turns,
        -2.0 * math.pi * right_sweeping_turns,
    )


def _right_flipper_error_rad(robot_state: List[float], target: Tuple[float, float]) -> float:
    current = _right_flipper_state_to_radians(robot_state)
    return max(abs(current[0] - target[0]), abs(current[1] - target[1]))


def _publish_trajectory(points: List[float], node_name: str) -> None:
    try:
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import Float64MultiArray
    except ModuleNotFoundError as exc:
        raise SystemExit("ROS 2 Python packages are required to publish the flipper trajectory.") from exc

    rclpy.init(args=None)
    node = Node(node_name)
    publisher = node.create_publisher(Float64MultiArray, "/trajectory_points", 10)
    msg = Float64MultiArray()
    msg.data = list(points)
    try:
        end_time = time.time() + DEFAULT_TRAJECTORY_PUBLISH_S
        while time.time() < end_time:
            publisher.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def _publish_fixed_trajectory(trajectory: List[float]) -> None:
    _publish_trajectory(list(trajectory), "pivlab_sequence_trigger")


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
    try:
        end_time = time.time() + DEFAULT_TRAJECTORY_PUBLISH_S
        while time.time() < end_time:
            publisher.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def _preposition_to_trajectory_start(
    trajectory: List[float],
    speed_rad_s: float,
    tolerance_rad: float,
    timeout_s: float,
) -> None:
    if speed_rad_s <= 0.0:
        raise RuntimeError("Preposition speed must be positive.")
    if tolerance_rad <= 0.0:
        raise RuntimeError("Preposition tolerance must be positive.")
    if timeout_s <= 0.0:
        raise RuntimeError("Preposition timeout must be positive.")

    try:
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import Float64MultiArray
    except ModuleNotFoundError as exc:
        raise SystemExit("ROS 2 Python packages are required to pre-position the flipper.") from exc

    class RobotStateWatcher(Node):
        def __init__(self) -> None:
            super().__init__("pivlab_preposition_monitor")
            self.latest_state: Optional[List[float]] = None
            self.create_subscription(Float64MultiArray, "/robot_state", self._robot_state_cb, 10)

        def _robot_state_cb(self, msg: Float64MultiArray) -> None:
            self.latest_state = list(msg.data)

    target = _trajectory_start(trajectory)

    rclpy.init(args=None)
    node = RobotStateWatcher()
    publisher = node.create_publisher(Float64MultiArray, "/trajectory_points", 10)
    try:
        state_deadline = time.time() + timeout_s
        while node.latest_state is None and time.time() < state_deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        if node.latest_state is None:
            raise RuntimeError(f"Timed out waiting for /robot_state after {timeout_s:.2f} s.")

        start = _right_flipper_state_to_radians(node.latest_state)
        start_error = _right_flipper_error_rad(node.latest_state, target)
        print(
            "Right flipper pre-position check: "
            f"current adduction={start[0]:.4f} rad, sweeping={start[1]:.4f} rad; "
            f"target adduction={target[0]:.4f} rad, sweeping={target[1]:.4f} rad; "
            f"error={start_error:.4f} rad."
        )
        if start_error <= tolerance_rad:
            print(
                "Right flipper already at trajectory start "
                f"(error={start_error:.4f} rad <= {tolerance_rad:.4f} rad)."
            )
            return

        preposition_msg = Float64MultiArray()
        preposition_msg.data = [
            start[0],
            start[1],
            float(speed_rad_s),
            target[0],
            target[1],
            float(speed_rad_s),
        ]

        print(
            "Pre-positioning right flipper to first waypoint: "
            f"adduction={target[0]:.4f} rad, sweeping={target[1]:.4f} rad "
            f"(current={start[0]:.4f}, {start[1]:.4f} rad)."
        )
        publish_end = time.time() + DEFAULT_TRAJECTORY_PUBLISH_S
        while time.time() < publish_end:
            publisher.publish(preposition_msg)
            rclpy.spin_once(node, timeout_sec=0.05)

        wait_deadline = time.time() + timeout_s
        last_error = start_error
        while time.time() < wait_deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if node.latest_state is None:
                continue
            last_error = _right_flipper_error_rad(node.latest_state, target)
            if last_error <= tolerance_rad:
                print(
                    "Pre-position complete "
                    f"(error={last_error:.4f} rad <= {tolerance_rad:.4f} rad)."
                )
                return

        raise RuntimeError(
            "Timed out waiting for right flipper to reach trajectory start "
            f"(last error={last_error:.4f} rad, tolerance={tolerance_rad:.4f} rad)."
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()


def _wait_for_trajectory_complete(
    trajectory: List[float],
    timeout_s: float,
    position_print_interval_s: float,
    allow_final_position_completion: bool,
    final_position_tolerance_rad: float,
) -> None:
    if timeout_s <= 0.0:
        raise RuntimeError("Trajectory timeout must be positive.")
    if position_print_interval_s < 0.0:
        raise RuntimeError("Position print interval cannot be negative.")

    try:
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import Bool, Float64MultiArray
    except ModuleNotFoundError as exc:
        raise SystemExit("ROS 2 Python packages are required to wait for trajectory completion.") from exc

    class TrajectoryCompleteWatcher(Node):
        def __init__(self) -> None:
            super().__init__("pivlab_trajectory_complete_monitor")
            self.latest_complete: Optional[bool] = None
            self.latest_state: Optional[List[float]] = None
            self.create_subscription(Bool, "/trajectory_complete", self._trajectory_complete_cb, 10)
            self.create_subscription(Float64MultiArray, "/robot_state", self._robot_state_cb, 10)

        def _trajectory_complete_cb(self, msg: Bool) -> None:
            self.latest_complete = bool(msg.data)

        def _robot_state_cb(self, msg: Float64MultiArray) -> None:
            self.latest_state = list(msg.data)

    final_target = _trajectory_end(trajectory)
    rclpy.init(args=None)
    node = TrajectoryCompleteWatcher()
    try:
        print(f"Waiting for /trajectory_complete=true for up to {timeout_s:.2f} s.")
        deadline = time.time() + timeout_s
        start_time = time.time()
        next_position_print = start_time
        while time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            now = time.time()
            if (
                position_print_interval_s > 0.0
                and node.latest_state is not None
                and now >= next_position_print
            ):
                _, right_sweeping_rad = _right_flipper_state_to_radians(node.latest_state)
                print(f"t={now - start_time:.2f}s rightsweeping={right_sweeping_rad:.4f} rad")
                next_position_print = now + position_print_interval_s
            if node.latest_complete is True:
                print("Trajectory complete.")
                return
            if allow_final_position_completion and node.latest_state is not None:
                final_error = _right_flipper_error_rad(node.latest_state, final_target)
                if final_error <= final_position_tolerance_rad:
                    print(
                        "Trajectory complete by final waypoint check "
                        f"(error={final_error:.4f} rad <= {final_position_tolerance_rad:.4f} rad)."
                    )
                    return
        raise RuntimeError(f"Timed out waiting for /trajectory_complete after {timeout_s:.2f} s.")
    finally:
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
    trajectory: List[float],
    output_root: Path,
    dt_s: float,
    duration_s: float,
    prefix: str,
    publish_trajectory: bool,
    preposition: bool,
    preposition_speed_rad_s: float,
    preposition_tolerance_rad: float,
    preposition_timeout_s: float,
    wait_trajectory_complete_after_capture: bool,
    trajectory_timeout_s: float,
    position_print_interval_s: float,
) -> Path:
    if publish_trajectory:
        if preposition:
            _preposition_to_trajectory_start(
                trajectory=trajectory,
                speed_rad_s=preposition_speed_rad_s,
                tolerance_rad=preposition_tolerance_rad,
                timeout_s=preposition_timeout_s,
            )
        _publish_fixed_trajectory(trajectory)
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

    if wait_trajectory_complete_after_capture:
        _wait_for_trajectory_complete(
            trajectory=trajectory,
            timeout_s=trajectory_timeout_s,
            position_print_interval_s=position_print_interval_s,
            allow_final_position_completion=True,
            final_position_tolerance_rad=preposition_tolerance_rad,
        )
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
    ap.add_argument("--dt", type=float, default=DEFAULT_DT_S, help="Target seconds between saved frames.")
    ap.add_argument("--duration", type=float, default=DEFAULT_DURATION_S, help="Seconds to save frames.")
    ap.add_argument("--prefix", type=str, default="pivlab", help="Filename prefix for saved grayscale PNGs.")
    ap.add_argument(
        "--sweeping-offset-deg",
        type=float,
        default=DEFAULT_SWEEPING_OFFSET_DEG,
        help="Degrees added to every right sweeping waypoint.",
    )
    ap.add_argument(
        "--trajectory-speed",
        type=float,
        default=DEFAULT_TRAJECTORY_SPEED_RAD_S,
        help="Waypoint interpolation speed in rad/s.",
    )
    ap.add_argument(
        "--preposition",
        dest="preposition",
        action="store_true",
        default=DEFAULT_PREPOSITION,
        help="Move the right flipper to the first trajectory waypoint before running the real trajectory.",
    )
    ap.add_argument("--no-preposition", dest="preposition", action="store_false")
    ap.add_argument("--preposition-speed", type=float, default=DEFAULT_PREPOSITION_SPEED_RAD_S)
    ap.add_argument("--preposition-tolerance", type=float, default=DEFAULT_PREPOSITION_TOLERANCE_RAD)
    ap.add_argument("--preposition-timeout", type=float, default=DEFAULT_PREPOSITION_TIMEOUT_S)
    ap.add_argument(
        "--home-only",
        action="store_true",
        help="Only move the right flipper to the first trajectory waypoint, then exit.",
    )
    ap.add_argument(
        "--trajectory-only",
        action="store_true",
        help="Run the fixed trajectory, wait for completion, then exit without camera capture.",
    )
    ap.add_argument("--trajectory-timeout", type=float, default=DEFAULT_TRAJECTORY_TIMEOUT_S)
    ap.add_argument("--position-print-interval", type=float, default=DEFAULT_POSITION_PRINT_INTERVAL_S)
    ap.add_argument(
        "--wait-trajectory-complete-after-capture",
        dest="wait_trajectory_complete_after_capture",
        action="store_true",
        default=DEFAULT_WAIT_TRAJECTORY_COMPLETE_AFTER_CAPTURE,
    )
    ap.add_argument(
        "--no-wait-trajectory-complete-after-capture",
        dest="wait_trajectory_complete_after_capture",
        action="store_false",
    )
    ap.add_argument(
        "--preview",
        dest="preview",
        action="store_true",
        default=DEFAULT_PREVIEW,
        help="Show preview; press t to start trajectory and g/space to save.",
    )
    ap.add_argument("--no-preview", dest="preview", action="store_false")
    return ap.parse_args()


def _validate_args(args: argparse.Namespace) -> None:
    if args.dt <= 0.0:
        raise SystemExit("--dt must be positive.")
    if args.duration <= 0.0:
        raise SystemExit("--duration must be positive.")
    if args.preposition_speed <= 0.0:
        raise SystemExit("--preposition-speed must be positive.")
    if args.preposition_tolerance <= 0.0:
        raise SystemExit("--preposition-tolerance must be positive.")
    if args.preposition_timeout <= 0.0:
        raise SystemExit("--preposition-timeout must be positive.")
    if args.trajectory_timeout <= 0.0:
        raise SystemExit("--trajectory-timeout must be positive.")
    if args.position_print_interval < 0.0:
        raise SystemExit("--position-print-interval cannot be negative.")
    if args.home_only and args.trajectory_only:
        raise SystemExit("--home-only and --trajectory-only cannot be used together.")
    if not math.isfinite(float(args.sweeping_offset_deg)):
        raise SystemExit("--sweeping-offset-deg must be finite.")
    if args.trajectory_speed <= 0.0 or not math.isfinite(float(args.trajectory_speed)):
        raise SystemExit("--trajectory-speed must be positive and finite.")


def _print_trajectory_summary(trajectory: List[float], sweeping_offset_deg: float, speed_rad_s: float) -> None:
    start_adduction_rad, start_sweeping_rad = _trajectory_start(trajectory)
    print(
        "Using fixed trajectory with sweeping offset "
        f"{sweeping_offset_deg:.2f} deg; "
        f"speed={speed_rad_s:.3f} rad/s; "
        f"first waypoint adduction={start_adduction_rad:.4f} rad, "
        f"sweeping={start_sweeping_rad:.4f} rad."
    )


def _stop_safely() -> None:
    try:
        _publish_stop_command()
        print("Published stop command on /Gui_information.")
    except Exception as exc:
        print(f"[WARN] Failed to publish stop command on /Gui_information: {exc}")


def main() -> int:
    args = parse_args()
    _validate_args(args)

    trajectory = _build_fixed_trajectory(
        sweeping_offset_deg=float(args.sweeping_offset_deg),
        speed_rad_s=float(args.trajectory_speed),
    )
    _print_trajectory_summary(
        trajectory=trajectory,
        sweeping_offset_deg=float(args.sweeping_offset_deg),
        speed_rad_s=float(args.trajectory_speed),
    )

    if args.home_only:
        try:
            _preposition_to_trajectory_start(
                trajectory=trajectory,
                speed_rad_s=float(args.preposition_speed),
                tolerance_rad=float(args.preposition_tolerance),
                timeout_s=float(args.preposition_timeout),
            )
            print("Home-only pre-position finished.")
            return 0
        finally:
            _stop_safely()

    if args.trajectory_only:
        try:
            if args.preposition:
                _preposition_to_trajectory_start(
                    trajectory=trajectory,
                    speed_rad_s=float(args.preposition_speed),
                    tolerance_rad=float(args.preposition_tolerance),
                    timeout_s=float(args.preposition_timeout),
                )
            _publish_fixed_trajectory(trajectory)
            print("Published fixed trajectory on /trajectory_points.")
            _wait_for_trajectory_complete(
                trajectory=trajectory,
                timeout_s=float(args.trajectory_timeout),
                position_print_interval_s=float(args.position_print_interval),
                allow_final_position_completion=False,
                final_position_tolerance_rad=float(args.preposition_tolerance),
            )
            print("Trajectory-only test finished.")
            return 0
        finally:
            _stop_safely()

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
                    if args.preposition:
                        _preposition_to_trajectory_start(
                            trajectory=trajectory,
                            speed_rad_s=float(args.preposition_speed),
                            tolerance_rad=float(args.preposition_tolerance),
                            timeout_s=float(args.preposition_timeout),
                        )
                    _publish_fixed_trajectory(trajectory)
                    trajectory_started = True
                    print("Published fixed trajectory on /trajectory_points. Press g/space when ready to save.")
                if key in (ord("g"), ord(" ")):
                    _run_capture(
                        capture=capture,
                        trajectory=trajectory,
                        output_root=args.output_root,
                        dt_s=float(args.dt),
                        duration_s=float(args.duration),
                        prefix=str(args.prefix),
                        publish_trajectory=not trajectory_started,
                        preposition=bool(args.preposition),
                        preposition_speed_rad_s=float(args.preposition_speed),
                        preposition_tolerance_rad=float(args.preposition_tolerance),
                        preposition_timeout_s=float(args.preposition_timeout),
                        wait_trajectory_complete_after_capture=bool(
                            args.wait_trajectory_complete_after_capture
                        ),
                        trajectory_timeout_s=float(args.trajectory_timeout),
                        position_print_interval_s=float(args.position_print_interval),
                    )
                    trajectory_started = True
                    print("Preview active. Press g/space to capture again; q/Esc quits.")
        else:
            input("Press Enter to start the flipper trajectory and begin saving...")
            _run_capture(
                capture=capture,
                trajectory=trajectory,
                output_root=args.output_root,
                dt_s=float(args.dt),
                duration_s=float(args.duration),
                prefix=str(args.prefix),
                publish_trajectory=True,
                preposition=bool(args.preposition),
                preposition_speed_rad_s=float(args.preposition_speed),
                preposition_tolerance_rad=float(args.preposition_tolerance),
                preposition_timeout_s=float(args.preposition_timeout),
                wait_trajectory_complete_after_capture=bool(args.wait_trajectory_complete_after_capture),
                trajectory_timeout_s=float(args.trajectory_timeout),
                position_print_interval_s=float(args.position_print_interval),
            )
            return 0
    finally:
        _stop_safely()
        capture.stop()
        if args.preview:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    raise SystemExit(main())
