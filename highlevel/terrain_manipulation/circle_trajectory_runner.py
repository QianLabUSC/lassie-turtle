#!/usr/bin/env python3
"""Publish a single circular waypoint trajectory and start/stop the controller, with live state logs."""

import math
import threading
import time
from typing import List, Optional

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


CIRCLE_WAYPOINTS = [
    0.35, 0.00, 0.5,
    0.25, 0.25, 0.5,
    0.00, 0.35, 0.5,
    -0.25, 0.25, 0.5,
    -0.35, 0.00, 0.5,
    -0.25, -0.25, 0.5,
    0.00, -0.35, 0.5,
    0.25, -0.25, 0.5,
    0.35, 0.00, 0.5,
]


class CircleRunner(Node):
    def __init__(self) -> None:
        super().__init__("circle_runner")
        self.waypoint_pub = self.create_publisher(Float64MultiArray, "/trajectory_points", 10)
        self.gui_pub = self.create_publisher(Float64MultiArray, "/Gui_information", 10)
        self.robot_state_sub = self.create_subscription(
            Float64MultiArray, "/robot_state", self._robot_state_cb, 10
        )
        self.stop_event = threading.Event()
        self.logging_enabled = False
        self.last_state: Optional[List[float]] = None
        self.create_timer(0.5, self._log_state)

    def publish_waypoints(self) -> None:
        msg = Float64MultiArray()
        msg.data = CIRCLE_WAYPOINTS
        self.waypoint_pub.publish(msg)
        self.get_logger().info("Published circle waypoints.")

    def start_motion(self) -> None:
        msg = Float64MultiArray()
        msg.data = [1.0, 0.0]
        self.gui_pub.publish(msg)
        self.get_logger().info("Start command sent.")
        self.logging_enabled = True

    def stop_motion(self) -> None:
        msg = Float64MultiArray()
        msg.data = [0.0, 0.0]
        self.gui_pub.publish(msg)
        self.get_logger().info("Stop command sent.")
        self.logging_enabled = False

    def _compute_duration(self) -> float:
        pts = [(CIRCLE_WAYPOINTS[i], CIRCLE_WAYPOINTS[i + 1], CIRCLE_WAYPOINTS[i + 2])
               for i in range(0, len(CIRCLE_WAYPOINTS), 3)]
        total = 0.0
        for i in range(1, len(pts)):
            x0, y0, _ = pts[i - 1]
            x1, y1, v = pts[i]
            dist = math.hypot(x1 - x0, y1 - y0)
            total += dist / max(v, 1e-6)
        return total

    def wait_or_stop(self, duration: float) -> None:
        def _wait_for_enter() -> None:
            input("Press Enter again to stop early...\n")
            self.stop_event.set()

        threading.Thread(target=_wait_for_enter, daemon=True).start()
        end_time = time.time() + duration
        while time.time() < end_time:
            if self.stop_event.is_set():
                break
            time.sleep(0.05)
        self.stop_motion()

    def _robot_state_cb(self, msg: Float64MultiArray) -> None:
        self.last_state = list(msg.data)

    def _log_state(self) -> None:
        if not self.logging_enabled:
            return
        if not self.last_state or len(self.last_state) < 5:
            return
        gait = self.last_state[0]
        la, ls, ra, rs = self.last_state[1:5]
        self.get_logger().info(
            f"gait={gait:.1f} pos turns [LA={la:.3f}, LS={ls:.3f}, RA={ra:.3f}, RS={rs:.3f}]"
        )


def main() -> None:
    rclpy.init()
    node = CircleRunner()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        input("Press Enter to publish circle trajectory and start motion...\n")
        node.publish_waypoints()
        node.start_motion()
        # Run one loop or until user stops early.
        node.wait_or_stop(node._compute_duration() + 1.0)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted, stopping.")
        node.stop_motion()
    finally:
        executor.shutdown()
        spin_thread.join(timeout=1.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
