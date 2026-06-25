/*
 * Software License Agreement (BSD-3 License)
 *
 * Copyright (c) 2026, DRAGON Laboratory, The University of Tokyo
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   1. Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above
 *      copyright notice, this list of conditions and the following
 *      disclaimer in the documentation and/or other materials provided
 *      with the distribution.
 *   3. Neither the name of the DRAGON Laboratory nor the names of its
 *      contributors may be used to endorse or promote products derived
 *      from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include "aerial_robot_navigation/util/joy_parser.hpp"

const sensor_msgs::msg::Joy joyParse(const sensor_msgs::msg::Joy &joy_msg)
{
  sensor_msgs::msg::Joy joy_cmd;

  int a_size = joy_msg.axes.size();
  int b_size = joy_msg.buttons.size();

  if (a_size == PS3_AXIS_SIZE && b_size == PS3_BUTTON_SIZE)
  {
    joy_cmd = joy_msg;
  }

  if (a_size == PS4_AXIS_SIZE && b_size == PS4_BUTTON_SIZE)
  {
    joy_cmd.header = joy_msg.header;
    joy_cmd.axes.resize(JOY_AXIS_SIZE, 0);
    joy_cmd.buttons.resize(JOY_BUTTON_SIZE, 0);
    joy_cmd.buttons[JOY_BUTTON_START] = joy_msg.buttons[PS4_BUTTON_OPTIONS];
    joy_cmd.buttons[JOY_BUTTON_STOP] = joy_msg.buttons[PS4_BUTTON_SHARE];
    joy_cmd.buttons[JOY_BUTTON_STICK_LEFT] = joy_msg.buttons[PS4_BUTTON_STICK_LEFT];
    joy_cmd.buttons[JOY_BUTTON_STICK_RIGHT] = joy_msg.buttons[PS4_BUTTON_STICK_RIGHT];
    if (joy_msg.axes[PS4_AXIS_BUTTON_CROSS_UP_DOWN] == 1) joy_cmd.buttons[JOY_BUTTON_CROSS_UP] = 1;
    if (joy_msg.axes[PS4_AXIS_BUTTON_CROSS_UP_DOWN] == -1) joy_cmd.buttons[JOY_BUTTON_CROSS_DOWN] = 1;
    if (joy_msg.axes[PS4_AXIS_BUTTON_CROSS_LEFT_RIGHT] == 1) joy_cmd.buttons[JOY_BUTTON_CROSS_LEFT] = 1;
    if (joy_msg.axes[PS4_AXIS_BUTTON_CROSS_LEFT_RIGHT] == -1) joy_cmd.buttons[JOY_BUTTON_CROSS_RIGHT] = 1;
    joy_cmd.buttons[JOY_BUTTON_REAR_LEFT_1] = joy_msg.buttons[PS4_BUTTON_REAR_LEFT_1];
    joy_cmd.buttons[JOY_BUTTON_REAR_RIGHT_1] = joy_msg.buttons[PS4_BUTTON_REAR_RIGHT_1];
    joy_cmd.buttons[JOY_BUTTON_REAR_LEFT_2] = joy_msg.buttons[PS4_BUTTON_REAR_LEFT_2];
    joy_cmd.buttons[JOY_BUTTON_REAR_RIGHT_2] = joy_msg.buttons[PS4_BUTTON_REAR_RIGHT_2];
    joy_cmd.buttons[JOY_BUTTON_ACTION_TRIANGLE] = joy_msg.buttons[PS4_BUTTON_ACTION_TRIANGLE];
    joy_cmd.buttons[JOY_BUTTON_ACTION_CIRCLE] = joy_msg.buttons[PS4_BUTTON_ACTION_CIRCLE];
    joy_cmd.buttons[JOY_BUTTON_ACTION_CROSS] = joy_msg.buttons[PS4_BUTTON_ACTION_CROSS];
    joy_cmd.buttons[JOY_BUTTON_ACTION_SQUARE] = joy_msg.buttons[PS4_BUTTON_ACTION_SQUARE];
    joy_cmd.buttons[JOY_BUTTON_PAIRING] = joy_msg.buttons[PS4_BUTTON_PAIRING];
    joy_cmd.axes[JOY_AXIS_STICK_LEFT_LEFTWARDS] = joy_msg.axes[PS4_AXIS_STICK_LEFT_LEFTWARDS];
    joy_cmd.axes[JOY_AXIS_STICK_LEFT_UPWARDS] = joy_msg.axes[PS4_AXIS_STICK_LEFT_UPWARDS];
    joy_cmd.axes[JOY_AXIS_STICK_RIGHT_LEFTWARDS] = joy_msg.axes[PS4_AXIS_STICK_RIGHT_LEFTWARDS];
    joy_cmd.axes[JOY_AXIS_STICK_RIGHT_UPWARDS] = joy_msg.axes[PS4_AXIS_STICK_RIGHT_UPWARDS];
    joy_cmd.axes[JOY_AXIS_ACCELEROMETER_LEFT] = joy_msg.axes[PS4_AXIS_ACCELEROMETER_LEFT];
    joy_cmd.axes[JOY_AXIS_ACCELEROMETER_FORWARD] = joy_msg.axes[PS4_AXIS_ACCELEROMETER_FORWARD];
    joy_cmd.axes[JOY_AXIS_ACCELEROMETER_UP] = joy_msg.axes[PS4_AXIS_ACCELEROMETER_UP];
    joy_cmd.axes[JOY_AXIS_GYRO_YAW] = joy_msg.axes[PS4_AXIS_GYRO_YAW];
  }

  if (a_size == BLT_AXIS_SIZE && b_size == BLT_BUTTON_SIZE)
  {
    joy_cmd.header = joy_msg.header;
    joy_cmd.axes.resize(JOY_AXIS_SIZE, 0);
    joy_cmd.buttons.resize(JOY_BUTTON_SIZE, 0);
    joy_cmd.buttons[JOY_BUTTON_START] = joy_msg.buttons[BLT_BUTTON_OPTIONS];
    joy_cmd.buttons[JOY_BUTTON_STOP] = joy_msg.buttons[BLT_BUTTON_SHARE];
    joy_cmd.buttons[JOY_BUTTON_STICK_LEFT] = joy_msg.buttons[BLT_BUTTON_STICK_LEFT];
    joy_cmd.buttons[JOY_BUTTON_STICK_RIGHT] = joy_msg.buttons[BLT_BUTTON_STICK_RIGHT];
    if (joy_msg.axes[BLT_AXIS_BUTTON_CROSS_UP_DOWN] == 1) joy_cmd.buttons[JOY_BUTTON_CROSS_UP] = 1;
    if (joy_msg.axes[BLT_AXIS_BUTTON_CROSS_UP_DOWN] == -1) joy_cmd.buttons[JOY_BUTTON_CROSS_DOWN] = 1;
    if (joy_msg.axes[BLT_AXIS_BUTTON_CROSS_LEFT_RIGHT] == 1) joy_cmd.buttons[JOY_BUTTON_CROSS_LEFT] = 1;
    if (joy_msg.axes[BLT_AXIS_BUTTON_CROSS_LEFT_RIGHT] == -1) joy_cmd.buttons[JOY_BUTTON_CROSS_RIGHT] = 1;
    joy_cmd.buttons[JOY_BUTTON_REAR_LEFT_1] = joy_msg.buttons[BLT_BUTTON_REAR_LEFT_1];
    joy_cmd.buttons[JOY_BUTTON_REAR_RIGHT_1] = joy_msg.buttons[BLT_BUTTON_REAR_RIGHT_1];
    joy_cmd.buttons[JOY_BUTTON_REAR_LEFT_2] = joy_msg.buttons[BLT_BUTTON_REAR_LEFT_2];
    joy_cmd.buttons[JOY_BUTTON_REAR_RIGHT_2] = joy_msg.buttons[BLT_BUTTON_REAR_RIGHT_2];
    joy_cmd.buttons[JOY_BUTTON_ACTION_TRIANGLE] = joy_msg.buttons[BLT_BUTTON_ACTION_TRIANGLE];
    joy_cmd.buttons[JOY_BUTTON_ACTION_CIRCLE] = joy_msg.buttons[BLT_BUTTON_ACTION_CIRCLE];
    joy_cmd.buttons[JOY_BUTTON_ACTION_CROSS] = joy_msg.buttons[BLT_BUTTON_ACTION_CROSS];
    joy_cmd.buttons[JOY_BUTTON_ACTION_SQUARE] = joy_msg.buttons[BLT_BUTTON_ACTION_SQUARE];
    joy_cmd.buttons[JOY_BUTTON_PAIRING] = joy_msg.buttons[BLT_BUTTON_PAIRING];
    joy_cmd.axes[JOY_AXIS_STICK_LEFT_LEFTWARDS] = joy_msg.axes[BLT_AXIS_STICK_LEFT_LEFTWARDS];
    joy_cmd.axes[JOY_AXIS_STICK_LEFT_UPWARDS] = joy_msg.axes[BLT_AXIS_STICK_LEFT_UPWARDS];
    joy_cmd.axes[JOY_AXIS_STICK_RIGHT_LEFTWARDS] = joy_msg.axes[BLT_AXIS_STICK_RIGHT_LEFTWARDS];
    joy_cmd.axes[JOY_AXIS_STICK_RIGHT_UPWARDS] = joy_msg.axes[BLT_AXIS_STICK_RIGHT_UPWARDS];
  }

  if (a_size == BLT_AXIS_SIZE && b_size == BLT_BUTTON_SIZE)
  {
    joy_cmd.header = joy_msg.header;
    joy_cmd.axes.resize(JOY_AXIS_SIZE, 0);
    joy_cmd.buttons.resize(JOY_BUTTON_SIZE, 0);
    joy_cmd.buttons[JOY_BUTTON_START] = joy_msg.buttons[BLT_BUTTON_OPTIONS];
    joy_cmd.buttons[JOY_BUTTON_STOP] = joy_msg.buttons[BLT_BUTTON_SHARE];
    joy_cmd.buttons[JOY_BUTTON_STICK_LEFT] = joy_msg.buttons[BLT_BUTTON_STICK_LEFT];
    joy_cmd.buttons[JOY_BUTTON_STICK_RIGHT] = joy_msg.buttons[BLT_BUTTON_STICK_RIGHT];
    if (joy_msg.axes[BLT_AXIS_BUTTON_CROSS_UP_DOWN] == 1) joy_cmd.buttons[JOY_BUTTON_CROSS_UP] = 1;
    if (joy_msg.axes[BLT_AXIS_BUTTON_CROSS_UP_DOWN] == -1) joy_cmd.buttons[JOY_BUTTON_CROSS_DOWN] = 1;
    if (joy_msg.axes[BLT_AXIS_BUTTON_CROSS_LEFT_RIGHT] == 1) joy_cmd.buttons[JOY_BUTTON_CROSS_LEFT] = 1;
    if (joy_msg.axes[BLT_AXIS_BUTTON_CROSS_LEFT_RIGHT] == -1) joy_cmd.buttons[JOY_BUTTON_CROSS_RIGHT] = 1;
    joy_cmd.buttons[JOY_BUTTON_REAR_LEFT_1] = joy_msg.buttons[BLT_BUTTON_REAR_LEFT_1];
    joy_cmd.buttons[JOY_BUTTON_REAR_RIGHT_1] = joy_msg.buttons[BLT_BUTTON_REAR_RIGHT_1];
    joy_cmd.buttons[JOY_BUTTON_REAR_LEFT_2] = joy_msg.buttons[BLT_BUTTON_REAR_LEFT_2];
    joy_cmd.buttons[JOY_BUTTON_REAR_RIGHT_2] = joy_msg.buttons[BLT_BUTTON_REAR_RIGHT_2];
    joy_cmd.buttons[JOY_BUTTON_ACTION_TRIANGLE] = joy_msg.buttons[BLT_BUTTON_ACTION_TRIANGLE];
    joy_cmd.buttons[JOY_BUTTON_ACTION_CIRCLE] = joy_msg.buttons[BLT_BUTTON_ACTION_CIRCLE];
    joy_cmd.buttons[JOY_BUTTON_ACTION_CROSS] = joy_msg.buttons[BLT_BUTTON_ACTION_CROSS];
    joy_cmd.buttons[JOY_BUTTON_ACTION_SQUARE] = joy_msg.buttons[BLT_BUTTON_ACTION_SQUARE];
    joy_cmd.buttons[JOY_BUTTON_PAIRING] = joy_msg.buttons[BLT_BUTTON_PAIRING];
    joy_cmd.axes[JOY_AXIS_STICK_LEFT_LEFTWARDS] = joy_msg.axes[BLT_AXIS_STICK_LEFT_LEFTWARDS];
    joy_cmd.axes[JOY_AXIS_STICK_LEFT_UPWARDS] = joy_msg.axes[BLT_AXIS_STICK_LEFT_UPWARDS];
    joy_cmd.axes[JOY_AXIS_STICK_RIGHT_LEFTWARDS] = joy_msg.axes[BLT_AXIS_STICK_RIGHT_LEFTWARDS];
    joy_cmd.axes[JOY_AXIS_STICK_RIGHT_UPWARDS] = joy_msg.axes[BLT_AXIS_STICK_RIGHT_UPWARDS];
  }

  if (a_size == ROG1_AXIS_SIZE && b_size == ROG1_BUTTON_SIZE)
  {
    joy_cmd.header = joy_msg.header;
    joy_cmd.axes.resize(JOY_AXIS_SIZE, 0);
    joy_cmd.buttons.resize(JOY_BUTTON_SIZE, 0);
    joy_cmd.buttons[JOY_BUTTON_START] = joy_msg.buttons[ROG1_BUTTON_TRI_LINE];
    joy_cmd.buttons[JOY_BUTTON_STOP] = joy_msg.buttons[ROG1_BUTTON_DUO_RECT];
    joy_cmd.buttons[JOY_BUTTON_STICK_LEFT] = joy_msg.buttons[ROG1_BUTTON_STICK_LEFT];
    joy_cmd.buttons[JOY_BUTTON_STICK_RIGHT] = joy_msg.buttons[ROG1_BUTTON_STICK_RIGHT];
    if (joy_msg.axes[ROG1_AXIS_BUTTON_CROSS_UP_DOWN] == 1) joy_cmd.buttons[JOY_BUTTON_CROSS_UP] = 1;
    if (joy_msg.axes[ROG1_AXIS_BUTTON_CROSS_UP_DOWN] == -1) joy_cmd.buttons[JOY_BUTTON_CROSS_DOWN] = 1;
    if (joy_msg.axes[ROG1_AXIS_BUTTON_CROSS_LEFT_RIGHT] == 1) joy_cmd.buttons[JOY_BUTTON_CROSS_LEFT] = 1;
    if (joy_msg.axes[ROG1_AXIS_BUTTON_CROSS_LEFT_RIGHT] == -1) joy_cmd.buttons[JOY_BUTTON_CROSS_RIGHT] = 1;
    joy_cmd.buttons[JOY_BUTTON_REAR_LEFT_1] = joy_msg.buttons[ROG1_BUTTON_REAR_LEFT_1];
    joy_cmd.buttons[JOY_BUTTON_REAR_RIGHT_1] = joy_msg.buttons[ROG1_BUTTON_REAR_RIGHT_1];
    joy_cmd.buttons[JOY_BUTTON_REAR_LEFT_2] = (joy_msg.axes[ROG1_AXIS_BUTTON_REAR_LEFT_2] == -1) ? 1 : 0;
    joy_cmd.buttons[JOY_BUTTON_REAR_RIGHT_2] = (joy_msg.axes[ROG1_AXIS_BUTTON_REAR_RIGHT_2] == -1) ? 1 : 0;
    joy_cmd.buttons[JOY_BUTTON_ACTION_CROSS] = joy_msg.buttons[ROG1_BUTTON_ACTION_A];
    joy_cmd.buttons[JOY_BUTTON_ACTION_CIRCLE] = joy_msg.buttons[ROG1_BUTTON_ACTION_B];
    joy_cmd.buttons[JOY_BUTTON_ACTION_SQUARE] = joy_msg.buttons[ROG1_BUTTON_ACTION_X];
    joy_cmd.buttons[JOY_BUTTON_ACTION_TRIANGLE] = joy_msg.buttons[ROG1_BUTTON_ACTION_Y];
    joy_cmd.axes[JOY_AXIS_STICK_LEFT_LEFTWARDS] = joy_msg.axes[ROG1_AXIS_STICK_LEFT_LEFTWARDS];
    joy_cmd.axes[JOY_AXIS_STICK_LEFT_UPWARDS] = joy_msg.axes[ROG1_AXIS_STICK_LEFT_UPWARDS];
    joy_cmd.axes[JOY_AXIS_STICK_RIGHT_LEFTWARDS] = joy_msg.axes[ROG1_AXIS_STICK_RIGHT_LEFTWARDS];
    joy_cmd.axes[JOY_AXIS_STICK_RIGHT_UPWARDS] = joy_msg.axes[ROG1_AXIS_STICK_RIGHT_UPWARDS];
  }

  return joy_cmd;
}
