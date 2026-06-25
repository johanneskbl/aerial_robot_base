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

/* Standard library */
#include <iostream>
#include <time.h>
#include <cmath>

/* ROS 2 */
#include <rclcpp/rclcpp.hpp>
#include <tf2_kdl/tf2_kdl.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/set_bool.hpp>

/* Kalman filter library */
#include "kalman_filter/kf_base_plugin.h"
#include "kalman_filter/lpf_filter.h"

/* Aerial robot packages */
#include "aerial_robot_model/model/aerial_robot_model.h"
#include "aerial_robot_estimation/state_estimation.h"
#include "aerial_robot_msgs/msg/states.hpp"


namespace Status
{
enum
{
  INACTIVE = 0,
  INIT = 1,
  ACTIVE = 2,
  INVALID = 3,
  RESET = 4
};
}

namespace sensor_plugin
{
class SensorBase
{
public:
  SensorBase();
  virtual ~SensorBase() {}

  virtual void initialize(rclcpp::Node::SharedPtr node, std::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                          std::shared_ptr<aerial_robot_estimation::StateEstimator> estimator, std::string sensor_name,
                          int index);


  inline const std::string &getSensorName() const { return sensor_name_; }

  const rclcpp::Time getTimeStamp() { return time_stamp_; }

  const int getStatus();

  void setStatus(const int status);

  virtual bool reset() { return true; }

  virtual void changeStatus(bool flag);

  void healthCheck();

protected:
  /* Node handle */
  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_;

  /* Publisher */
  rclcpp::Publisher<aerial_robot_msgs::msg::States>::SharedPtr state_pub_;

  /* Service */
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_status_service_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_service_;

  std::shared_ptr<aerial_robot_model::RobotModel> robot_model_;
  std::shared_ptr<aerial_robot_estimation::StateEstimator> estimator_;

  /* Reconfigurable variable */
  int estimate_mode_;

  bool param_verbose_;
  bool debug_verbose_;

  std::string sensor_name_;
  int sensor_index_;

  bool get_sensor_rel_pose_;
  bool sensor_pose_vary_flag_;

  std::string sensor_frame_;

  rclcpp::Time time_stamp_, prev_time_stamp_;
  bool time_sync_;
  double delay_;
  double curr_timestamp_;
  double prev_timestamp_;
  double sensor_hz_;                     // Frequency of the sensor in Hz
  std::vector<int> estimate_indices_;    // The fuser_egomotion index
  std::vector<int> experiment_indices_;  // The fuser_experiment indices

  /* The transformation between sensor frame and baselink frame */
  KDL::Frame sensor_rel_pose_;

  /* Status */
  int sensor_status_;
  int prev_status_;
  std::mutex status_mutex_;
  std::mutex health_check_mutex_;

  /* Health check */
  double reset_stamp_;
  double reset_duration_;
  std::vector<bool> health_;
  std::vector<double> health_stamp_;
  double health_timeout_;
  int unhealth_level_;

  inline const bool isModeActivate(uint8_t mode) const { return (estimate_mode_ & (1 << mode)); }

  virtual void estimateProcess() {}
  virtual void activateFuser() {}
  virtual void fuse() {}
  virtual void preProcessState() {}
  virtual void setState() {}

  virtual void publish() {}
  virtual void rosParamInit() {}

  bool resetCb(const std::shared_ptr<rmw_request_id_t> req_id, const std::shared_ptr<std_srvs::srv::Empty::Request> req,
               std::shared_ptr<std_srvs::srv::Empty::Response> res);

  bool setStatusCb(const std::shared_ptr<rmw_request_id_t> req_id,
                   const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                   std::shared_ptr<std_srvs::srv::SetBool::Response> res);

  void setHealthChanNum(const uint8_t &chan_num);

  void updateHealthStamp(uint8_t chan = 0);

  inline const KDL::Frame &getBase2SensorTF() const { return sensor_rel_pose_; }

  bool updateBase2SensorTF();

  template <class T> void getParam(std::string param_name, T &param, T default_value, bool verbose = false)
  {
    std::string full_param_name = "estimation.fusion.sensor_plugin." + sensor_name_ + "." + param_name;
    node_->get_parameter_or<T>(full_param_name, param, default_value);

    if (param_verbose_ || verbose)
    {
      rclcpp::Parameter ros_param(full_param_name, param);
      RCLCPP_INFO(node_->get_logger(), "[%s] %s: %s", node_->get_namespace(), param_name.c_str(),
                  ros_param.value_to_string().c_str());
    }
  }
};
}
