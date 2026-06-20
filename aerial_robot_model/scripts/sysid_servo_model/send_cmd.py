#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2026, DRAGON Laboratory, The University of Tokyo
"""
created by Jinjie LI, 2024/02/15
"""

import argparse
import threading

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from spinal.msg import ServoControlCmd


def rad_2_kondo_pos(angle):
    kondo_pos = (11500 - 3500) * (-angle - (-2.36)) / (2.36 - (-2.36)) + 3500
    return int(kondo_pos)


def kondo_pos_2_rad(pos):
    angle = -((2.36 - (-2.36)) * (pos - 3500) / (11500 - 3500) + (-2.36))
    return float(angle)


# Use a ROS 2 timer to send the command.
class ServoControlCmdSender(Node):
    def __init__(self, init_set_angle: float):
        super().__init__("send_cmd")
        self.pub = self.create_publisher(ServoControlCmd, "/kondo_servo/states_cmd", 10)
        self.cmd = ServoControlCmd()
        self.cmd.index = [1, 2, 3, 4]
        self.cmd.angles = [
            rad_2_kondo_pos(init_set_angle),
            rad_2_kondo_pos(init_set_angle),
            rad_2_kondo_pos(init_set_angle),
            rad_2_kondo_pos(init_set_angle),
        ]
        self.timer = self.create_timer(0.01, self.send_cmd)  # 100 Hz

    def set_angle(self, angle):
        self.cmd.angles = [
            rad_2_kondo_pos(angle),
            rad_2_kondo_pos(angle),
            rad_2_kondo_pos(angle),
            rad_2_kondo_pos(angle),
        ]

    def send_cmd(self):
        self.pub.publish(self.cmd)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Set the angle of the servo", add_help=True)
    parser.add_argument("--init_set_angle", type=float, help="The angle to set the servo to", default=0.0)

    rclpy.init()
    sender = ServoControlCmdSender(parser.parse_args().init_set_angle)
    executor = SingleThreadedExecutor()
    executor.add_node(sender)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        while rclpy.ok():
            try:
                sender.set_angle(float(input("Enter the angle (rad): ")))
            except EOFError:
                break
            except ValueError:
                print("Please enter a valid number. Input Ctrl+D to exit.")
    except KeyboardInterrupt:
        print("KeyboardInterrupt")
    finally:
        executor.shutdown()
        sender.destroy_node()
        rclpy.shutdown()
