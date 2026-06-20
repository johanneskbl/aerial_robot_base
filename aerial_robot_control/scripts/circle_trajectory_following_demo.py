#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2026, DRAGON Laboratory, The University of Tokyo
import math
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from aerial_robot_msgs.msg import FlightNav, PoseControlPid


class CircTrajFollow(Node):
    def __init__(self) -> None:
        super().__init__("circle_trajectory_follow")

        self.declare_parameter("nav_rate", 20.0)  # Hz
        self.declare_parameter("period", 40.0)
        self.declare_parameter("radius", 1.0)
        self.declare_parameter("init_theta", 0.0)
        self.declare_parameter("yaw", True)
        self.declare_parameter("loop", False)

        nav_rate_value = self.get_parameter("nav_rate").value
        period_value = self.get_parameter("period").value
        radius_value = self.get_parameter("radius").value
        init_theta_value = self.get_parameter("init_theta").value
        self.yaw = bool(self.get_parameter("yaw").value)
        self.loop = bool(self.get_parameter("loop").value)

        if nav_rate_value is None or period_value is None or radius_value is None or init_theta_value is None:
            raise ValueError("trajectory parameters must not be None")

        self.nav_rate = float(nav_rate_value)
        self.period = float(period_value)
        self.radius = float(radius_value)
        self.init_theta = float(init_theta_value)
        if self.nav_rate <= 0.0:
            raise ValueError("nav_rate must be > 0")

        self.nav_dt = 1.0 / self.nav_rate
        self.omega = 2.0 * math.pi / self.period
        self.velocity = self.omega * self.radius

        self.nav_pub = self.create_publisher(FlightNav, "uav/nav", QoSProfile())
        self.control_sub = self.create_subscription(PoseControlPid, "debug/pose/pid", self.control_cb, QoSProfile())

        self.center_pos_x: Optional[float] = None
        self.center_pos_y: Optional[float] = None
        self.initial_target_yaw: Optional[float] = None

        self.cnt = 0
        self.steps_per_loop = int(self.period / self.nav_dt)

        self.flight_nav = FlightNav()
        self.flight_nav.target = FlightNav.COG
        self.flight_nav.pos_xy_nav_mode = FlightNav.POS_VEL_MODE
        if self.yaw:
            self.flight_nav.yaw_nav_mode = FlightNav.POS_VEL_MODE

        self._last_wait_log = 0.0
        self._finished = False

        self.timer = self.create_timer(self.nav_dt, self.timer_cb)
        time.sleep(0.5)

    def control_cb(self, msg) -> None:
        self.initial_target_yaw = msg.yaw.target_p

        self.center_pos_x = msg.x.target_p - math.cos(self.init_theta) * self.radius
        self.center_pos_y = msg.y.target_p - math.sin(self.init_theta) * self.radius

        self.get_logger().info(f"the center position is [{self.center_pos_x:.6f}, {self.center_pos_y:.6f}]")

        if self.control_sub is not None:
            self.destroy_subscription(self.control_sub)
            self.control_sub = None

    def stop(self) -> None:
        self.get_logger().info("stop following")
        self.flight_nav.target_vel_x = 0.0
        self.flight_nav.target_vel_y = 0.0
        self.flight_nav.target_omega_z = 0.0
        self.nav_pub.publish(self.flight_nav)

    def timer_cb(self) -> None:
        if self._finished:
            return

        if self.center_pos_x is None or self.center_pos_y is None:
            self.get_logger().info("Not yet received the controller message...", throttle_duration_sec=1.0)
            time.sleep(self.nav_dt)
            return

        theta = self.init_theta + self.cnt * self.nav_dt * self.omega
        target_pos_x = self.center_pos_x + math.cos(theta) * self.radius
        target_pos_y = self.center_pos_y + math.sin(theta) * self.radius
        self.flight_nav.target_pos_x = float(target_pos_x)
        self.flight_nav.target_pos_y = float(target_pos_y)
        self.flight_nav.target_vel_x = float(-math.sin(theta) * self.velocity)
        self.flight_nav.target_vel_y = float(math.cos(theta) * self.velocity)

        if self.yaw and self.initial_target_yaw is not None:
            self.flight_nav.target_yaw = float(self.initial_target_yaw + self.cnt * self.nav_dt * self.omega)
            self.flight_nav.target_omega_z = float(self.omega)
        else:
            self.flight_nav.target_omega_z = 0.0

        self.nav_pub.publish(self.flight_nav)
        self.cnt += 1

        if self.cnt >= self.steps_per_loop:
            if self.loop:
                self.cnt = 0
            else:
                time.sleep(0.1)
                self.flight_nav.target_vel_x = 0
                self.flight_nav.target_vel_y = 0
                self.flight_nav.target_omega_z = 0
                self.nav_pub.publish(self.flight_nav)

                self.stop()
                self._finished = True
                self.timer.cancel()
                rclpy.shutdown()


if __name__ == "__main__":
    rclpy.init()
    node = CircTrajFollow()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
