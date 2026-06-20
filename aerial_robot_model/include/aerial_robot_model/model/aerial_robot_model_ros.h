// -*- mode: c++ -*-
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
#pragma once

#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include <memory>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <string>

#include "aerial_robot_model/model/aerial_robot_model.h"
#include "aerial_robot_model/srv/add_extra_module.hpp"
#include "spinal/msg/desire_coord.hpp"

namespace aerial_robot_model
{

class RobotModelRos
{
public:
  RobotModelRos(rclcpp::Node::SharedPtr node);
  ~RobotModelRos() = default;

  /// Return the lastreceived joint state
  sensor_msgs::msg::JointState getJointState() const { return joint_state_; }

  /// Access the underlying RobotModel instance
  std::shared_ptr<aerial_robot_model::RobotModel> getRobotModel() const { return robot_model_; }

private:
  // ROS node
  rclcpp::Node::SharedPtr node_;
  // Callback for incoming JointState messages
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

  // Callback for the AddExtraModule service
  void addExtraModuleCallback(const std::shared_ptr<rmw_request_id_t> request_header,
                              const std::shared_ptr<aerial_robot_model::srv::AddExtraModule::Request> req,
                              std::shared_ptr<aerial_robot_model::srv::AddExtraModule::Response> res);
  void desireCoordinateCallback(const spinal::msg::DesireCoord::ConstSharedPtr &msg);

  // ROS2 interfaces
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<spinal::msg::DesireCoord>::SharedPtr desire_coordinate_sub_;
  rclcpp::Service<aerial_robot_model::srv::AddExtraModule>::SharedPtr add_extra_module_srv_;

  // TF broadcasters
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  tf2_ros::StaticTransformBroadcaster static_broadcaster_;

  // Stored state
  sensor_msgs::msg::JointState joint_state_;
  std::string tf_prefix_;

  // Pluginlib loader for your RobotModel plugins
  pluginlib::ClassLoader<aerial_robot_model::RobotModel> robot_model_loader_;
  std::shared_ptr<aerial_robot_model::RobotModel> robot_model_;
};
}
