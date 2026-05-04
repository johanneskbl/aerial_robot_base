#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2026, DRAGON Laboratory, The University of Tokyo

import sys
import select
import termios
import tty
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from aerial_robot_msgs.msg import FlightNav

msg_text = """
Instruction:

---------------------------

r:  arming motor (please do before takeoff)
t:  takeoff
l:  land
f:  force landing
h:  halt (force stop motor)

     q           w           e           [
(turn left)  (forward)  (turn right)  (move up)

     a           s           d           ]
(move left)  (backward) (move right) (move down)


Please don't have caps lock on.
CTRL+c to quit
---------------------------
"""


def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def printMsg(msg_str, msg_len=50):
    print(msg_str.ljust(msg_len) + "\r", end="")


class KeyboardCommandNode(Node):
    def __init__(self):
        super().__init__("keyboard_command")
        self.declare_parameter("robot_ns", "")
        self.robot_ns = self.get_parameter("robot_ns").value
        if not self.robot_ns:
            self.robot_ns = ""

        ns = self.robot_ns + "/teleop_command"
        self.land_pub = self.create_publisher(Empty, ns + "/land", 1)
        self.halt_pub = self.create_publisher(Empty, ns + "/halt", 1)
        self.start_pub = self.create_publisher(Empty, ns + "/start", 1)
        self.takeoff_pub = self.create_publisher(Empty, ns + "/takeoff", 1)
        self.force_landing_pub = self.create_publisher(Empty, ns + "/force_landing", 1)
        self.nav_pub = self.create_publisher(FlightNav, self.robot_ns + "/uav/nav", 1)
        self.motion_start_pub = self.create_publisher(Empty, "task_start", 1)

        self.declare_parameter("xy_vel", 0.2)
        self.declare_parameter("z_vel", 0.2)
        self.declare_parameter("yaw_vel", 0.2)
        self.xy_vel = self.get_parameter("xy_vel").value
        self.z_vel = self.get_parameter("z_vel").value
        self.yaw_vel = self.get_parameter("yaw_vel").value

    def run(self):
        self.get_logger().info("Keyboard Command Node started")
        print(msg_text)
        while rclpy.ok():
            nav_msg = FlightNav()
            nav_msg.control_frame = FlightNav.WORLD_FRAME
            nav_msg.target = FlightNav.COG

            key = getKey()
            output = ""

            if key == "l":
                self.land_pub.publish(Empty())
                output = "sent land command"
            elif key == "r":
                self.start_pub.publish(Empty())
                output = "sent motor-arming command"
            elif key == "h":
                self.halt_pub.publish(Empty())
                output = "sent motor-disarming (halt) command"
            elif key == "f":
                self.force_landing_pub.publish(Empty())
                output = "sent force landing command"
            elif key == "t":
                self.takeoff_pub.publish(Empty())
                output = "sent takeoff command"
            elif key == "x":
                self.motion_start_pub.publish(Empty())
                output = "sent task-start command"
            elif key == "w":
                nav_msg.pos_xy_nav_mode = FlightNav.VEL_MODE
                nav_msg.target_vel_x = self.xy_vel
                self.nav_pub.publish(nav_msg)
                output = "sent +x vel command"
            elif key == "s":
                nav_msg.pos_xy_nav_mode = FlightNav.VEL_MODE
                nav_msg.target_vel_x = -self.xy_vel
                self.nav_pub.publish(nav_msg)
                output = "sent -x vel command"
            elif key == "a":
                nav_msg.pos_xy_nav_mode = FlightNav.VEL_MODE
                nav_msg.target_vel_y = self.xy_vel
                self.nav_pub.publish(nav_msg)
                output = "sent +y vel command"
            elif key == "d":
                nav_msg.pos_xy_nav_mode = FlightNav.VEL_MODE
                nav_msg.target_vel_y = -self.xy_vel
                self.nav_pub.publish(nav_msg)
                output = "sent -y vel command"
            elif key == "q":
                nav_msg.yaw_nav_mode = FlightNav.VEL_MODE
                nav_msg.target_omega_z = self.yaw_vel
                self.nav_pub.publish(nav_msg)
                output = "sent +yaw vel command"
            elif key == "e":
                nav_msg.yaw_nav_mode = FlightNav.VEL_MODE
                nav_msg.target_omega_z = -self.yaw_vel
                self.nav_pub.publish(nav_msg)
                output = "sent -yaw vel command"
            elif key == "[":
                nav_msg.pos_z_nav_mode = FlightNav.VEL_MODE
                nav_msg.target_vel_z = self.z_vel
                self.nav_pub.publish(nav_msg)
                output = "sent +z vel command"
            elif key == "]":
                nav_msg.pos_z_nav_mode = FlightNav.VEL_MODE
                nav_msg.target_vel_z = -self.z_vel
                self.nav_pub.publish(nav_msg)
                output = "sent -z vel command"
            elif key == "\x03":  # Ctrl+C
                break

            printMsg(output)
            time.sleep(0.001)


def main():
    global settings
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = KeyboardCommandNode()
    try:
        node.run()
    except Exception as e:
        print(repr(e))
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
