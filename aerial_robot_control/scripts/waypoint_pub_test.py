#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2026, DRAGON Laboratory, The University of Tokyo
import time
import numpy as np

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.duration import Duration
from rclpy.node import Node
from tf_transformations import quaternion_from_euler

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path


def create_pose(node: Node, x: float, y: float, z: float, yaw_deg: float, time_offset_sec: int) -> PoseStamped:
    pose = PoseStamped()
    now = node.get_clock().now()
    pose.header.stamp = (now + Duration(seconds=int(time_offset_sec))).to_msg()
    pose.header.frame_id = "world"

    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = float(z)

    yaw_rad = float(yaw_deg) * np.pi / 180.0
    q = quaternion_from_euler(0.0, 0.0, yaw_rad)
    pose.pose.orientation.x = float(q[0])
    pose.pose.orientation.y = float(q[1])
    pose.pose.orientation.z = float(q[2])
    pose.pose.orientation.w = float(q[3])

    return pose


class PathPublisher(Node):
    def __init__(self) -> None:
        super().__init__("path_publisher")
        self.pub = self.create_publisher(Path, "/quadrotor/target_path", 10)

    def publish_path(self) -> None:
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = "world"

        wp1 = create_pose(self, 0.5, 0.5, 1.0, 0.0, 2)
        wp2 = create_pose(self, 1.5, -0.5, 2.0, 0.0, 5)
        wp3 = create_pose(self, 5.0, 0.0, 1.0, 0.0, 10)

        path.poses = [wp1, wp2, wp3]

        self.pub.publish(path)
        self.get_logger().info("Published path with 3 waypoints.")


def main() -> None:
    rclpy.init()
    node = PathPublisher()
    try:
        time.sleep(1.0)  # Allow discovery
        node.publish_path()
    except ExternalShutdownException:
        pass


if __name__ == "__main__":
    main()
