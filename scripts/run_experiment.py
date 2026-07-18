#!/usr/bin/env python3
"""Publish one goal and record ground truth, attitude and motor RPM to CSV."""

import argparse
import csv
import math
from pathlib import Path
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


CSV_FIELDS = [
    "t", "x", "y", "z", "goal_x", "goal_y", "goal_z",
    "roll", "pitch", "yaw", "rpm1", "rpm2", "rpm3", "rpm4",
]


def quaternion_to_euler(x, y, z, w):
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return roll, pitch, math.atan2(siny_cosp, cosy_cosp)


class ExperimentRunner(Node):
    def __init__(self, goal):
        super().__init__("experiment_runner")
        self.goal = tuple(goal)
        self.rows = []
        self.rpm = [0.0] * 4
        self.start = time.monotonic()
        self.goal_publish_count = 0

        self.goal_pub = self.create_publisher(PoseStamped, "/drone/goal", 10)
        self.create_subscription(
            Odometry, "/drone/ground_truth/odom", self.odom_callback, 10
        )
        self.create_subscription(
            Float32MultiArray, "/drone/motor_rpm", self.rpm_callback, 10
        )
        self.goal_timer = self.create_timer(0.25, self.publish_goal)

    def rpm_callback(self, message):
        values = list(message.data[:4])
        self.rpm = values + [0.0] * (4 - len(values))

    def publish_goal(self):
        # Avoid losing the one-shot goal while DDS discovery is still in progress.
        if self.goal_pub.get_subscription_count() == 0:
            return
        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "map"
        (
            message.pose.position.x,
            message.pose.position.y,
            message.pose.position.z,
        ) = self.goal
        message.pose.orientation.w = 1.0
        self.goal_pub.publish(message)
        self.goal_publish_count += 1
        if self.goal_publish_count >= 5:
            self.goal_timer.cancel()

    def odom_callback(self, message):
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        roll, pitch, yaw = quaternion_to_euler(
            orientation.x, orientation.y, orientation.z, orientation.w
        )
        self.rows.append(
            {
                "t": time.monotonic() - self.start,
                "x": position.x,
                "y": position.y,
                "z": position.z,
                "goal_x": self.goal[0],
                "goal_y": self.goal[1],
                "goal_z": self.goal[2],
                "roll": roll,
                "pitch": pitch,
                "yaw": yaw,
                "rpm1": self.rpm[0],
                "rpm2": self.rpm[1],
                "rpm3": self.rpm[2],
                "rpm4": self.rpm[3],
            }
        )


def save_csv(output, rows):
    path = Path(output).expanduser()
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=CSV_FIELDS)
        writer.writeheader()
        writer.writerows(rows)
    return path


def main():
    parser = argparse.ArgumentParser(
        description="Record a single-goal experiment from an already running simulation."
    )
    parser.add_argument("--goal", nargs=3, type=float, default=[0.0, 0.0, 1.5])
    parser.add_argument("--duration", type=float, default=25.0)
    parser.add_argument("--output", default="experiment.csv")
    args = parser.parse_args()
    if args.duration <= 0.0:
        parser.error("--duration must be positive")

    rclpy.init()
    node = ExperimentRunner(args.goal)
    deadline = time.monotonic() + args.duration
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().warning("recording interrupted; saving collected samples")

    path = save_csv(args.output, node.rows)
    row_count = len(node.rows)
    publish_count = node.goal_publish_count
    node.destroy_node()
    rclpy.shutdown()

    if row_count == 0:
        print(
            "ERROR: no /drone/ground_truth/odom samples were received. "
            "Start a launch file and source install/setup.bash first.",
            file=sys.stderr,
        )
        return 2
    if publish_count == 0:
        print(
            "ERROR: /drone/goal had no subscriber; the planner is probably not running.",
            file=sys.stderr,
        )
        return 3

    print(f"recorded {row_count} samples to {path}")
    print(f"published goal {publish_count} times after DDS discovery")
    return 0


if __name__ == "__main__":
    sys.exit(main())
