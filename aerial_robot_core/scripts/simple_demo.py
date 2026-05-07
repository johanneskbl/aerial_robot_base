#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2026, DRAGON Laboratory, The University of Tokyo

import rclpy
import smach
import smach_ros

from aerial_robot_core.robot_interface import RobotInterface
from aerial_robot_core.state_machine import Start, Arm, Takeoff, WayPoint, CircleTrajectory, Land


class SimpleDemo:
    def __init__(self):
        self.robot = RobotInterface(robot_ns="")

        self.sm_top = smach.StateMachine(outcomes=["succeeded", "preempted"])
        self.sm_top.userdata.flags = {}
        self.sm_top.userdata.extra = {}

        with self.sm_top:
            smach.StateMachine.add(
                "Start",
                Start(self.robot),
                transitions={"succeeded": "Arm", "preempted": "preempted"},
                remapping={"flags": "flags", "extra": "extra"},
            )
            smach.StateMachine.add(
                "Arm",
                Arm(self.robot),
                transitions={"succeeded": "Takeoff", "preempted": "preempted"},
                remapping={"flags": "flags", "extra": "extra"},
            )
            smach.StateMachine.add(
                "Takeoff",
                Takeoff(self.robot),
                transitions={"succeeded": "WayPoint", "preempted": "preempted"},
                remapping={"flags": "flags", "extra": "extra"},
            )
            smach.StateMachine.add(
                "WayPoint",
                WayPoint(self.robot, waypoints=[[0, 0, 1.5], [1, 0, 1]], hold_time=2.0),
                transitions={"succeeded": "Circle", "preempted": "preempted"},
                remapping={"flags": "flags", "extra": "extra"},
            )
            smach.StateMachine.add(
                "Circle",
                CircleTrajectory(self.robot, period=20.0),
                transitions={"succeeded": "Land", "preempted": "preempted"},
                remapping={"flags": "flags", "extra": "extra"},
            )
            smach.StateMachine.add(
                "Land",
                Land(self.robot),
                transitions={"succeeded": "succeeded", "preempted": "preempted"},
                remapping={"flags": "flags", "extra": "extra"},
            )

            self.sis = smach_ros.IntrospectionServer("task_smach_server", self.sm_top, "/SM_ROOT")

        self.sis.start()
        outcome = self.sm_top.execute()
        self.sis.stop()
        self.robot.get_logger().info("State machine finished with outcome: " + outcome)


def main(args=None):
    rclpy.init(args=args)
    demo = SimpleDemo()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
