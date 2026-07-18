#!/usr/bin/env python3
"""Fly and record a four-corner mission, advancing after a stable arrival."""

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


FIELDS = [
    "t", "x", "y", "z", "goal_x", "goal_y", "goal_z",
    "rpm1", "rpm2", "rpm3", "rpm4", "waypoint_index",
]


class WaypointMission(Node):
    def __init__(self, waypoints, dwell):
        super().__init__("waypoint_mission")
        self.waypoints = waypoints
        self.dwell = dwell
        self.index = 0
        self.entered = None
        self.done = False
        self.start = time.monotonic()
        self.rows = []
        self.rpm = [0.0] * 4
        self.goal_publish_count = 0

        self.goal_pub = self.create_publisher(PoseStamped, "/drone/goal", 10)
        self.create_subscription(
            Odometry, "/drone/ground_truth/odom", self.odom_callback, 10
        )
        self.create_subscription(
            Float32MultiArray, "/drone/motor_rpm", self.rpm_callback, 10
        )
        self.create_timer(0.2, self.publish_goal)

    def rpm_callback(self, message):
        values = list(message.data[:4])
        self.rpm = values + [0.0] * (4 - len(values))

    def publish_goal(self):
        if self.done or self.goal_pub.get_subscription_count() == 0:
            return
        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "map"
        goal = self.waypoints[self.index]
        (
            message.pose.position.x,
            message.pose.position.y,
            message.pose.position.z,
        ) = goal
        message.pose.orientation.w = 1.0
        self.goal_pub.publish(message)
        self.goal_publish_count += 1

    def odom_callback(self, message):
        if self.done:
            return
        position = message.pose.pose.position
        goal = self.waypoints[self.index]
        self.rows.append(
            {
                "t": time.monotonic() - self.start,
                "x": position.x,
                "y": position.y,
                "z": position.z,
                "goal_x": goal[0],
                "goal_y": goal[1],
                "goal_z": goal[2],
                "rpm1": self.rpm[0],
                "rpm2": self.rpm[1],
                "rpm3": self.rpm[2],
                "rpm4": self.rpm[3],
                "waypoint_index": self.index + 1,
            }
        )

        distance = math.dist((position.x, position.y, position.z), goal)
        if distance < 0.3:
            if self.entered is None:
                self.entered = time.monotonic()
            elif time.monotonic() - self.entered >= self.dwell:
                self.get_logger().info(
                    f"waypoint {self.index + 1}/{len(self.waypoints)} reached"
                )
                self.index += 1
                self.entered = None
                if self.index == len(self.waypoints):
                    self.done = True
        else:
            self.entered = None


def save_csv(output, rows):
    path = Path(output).expanduser()
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=FIELDS)
        writer.writeheader()
        writer.writerows(rows)
    return path


def main():
    parser = argparse.ArgumentParser(
        description="Fly and record takeoff plus a four-corner waypoint mission."
    )
    parser.add_argument("--dwell", type=float, default=1.0)
    parser.add_argument("--timeout", type=float, default=90.0)
    parser.add_argument("--output", default="waypoint.csv")
    args = parser.parse_args()
    if args.dwell < 0.0 or args.timeout <= 0.0:
        parser.error("--dwell must be non-negative and --timeout must be positive")

    waypoints = [
        (0.0, 0.0, 1.5),
        (2.0, 0.0, 1.5),
        (2.0, 2.0, 2.0),
        (0.0, 2.0, 1.5),
        (0.0, 0.0, 1.5),
    ]
    rclpy.init()
    node = WaypointMission(waypoints, args.dwell)
    deadline = time.monotonic() + args.timeout
    try:
        while rclpy.ok() and not node.done and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().warning("mission interrupted; saving collected samples")

    completed = node.done
    completed_count = node.index
    publish_count = node.goal_publish_count
    path = save_csv(args.output, node.rows)
    row_count = len(node.rows)
    node.destroy_node()
    rclpy.shutdown()

    print(f"recorded {row_count} samples to {path}")
    if row_count == 0:
        print(
            "ERROR: no /drone/ground_truth/odom samples were received. "
            "Start open.launch.py and source install/setup.bash first.",
            file=sys.stderr,
        )
        return 2
    if publish_count == 0:
        print("ERROR: /drone/goal had no subscriber", file=sys.stderr)
        return 3
    if not completed:
        print(
            f"ERROR: mission timeout after reaching {completed_count}/{len(waypoints)} waypoints",
            file=sys.stderr,
        )
        return 4
    print(f"mission complete: reached {completed_count}/{len(waypoints)} waypoints")
    return 0


if __name__ == "__main__":
    sys.exit(main())
