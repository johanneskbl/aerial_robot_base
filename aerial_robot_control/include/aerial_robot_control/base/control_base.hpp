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

/* Aerial robot packages */
#include "aerial_robot_model/model/aerial_robot_model.h"
#include "aerial_robot_estimation/state_estimation.h"
#include "aerial_robot_navigation/flight_navigation.hpp"
#include "spinal/msg/pwm_info.hpp"
#include "spinal/msg/uav_info.hpp"


namespace aerial_robot_control
{

class ControlBase
{
public:
  ControlBase() : control_timestamp_(-1), activate_timestamp_(0) {}

  virtual ~ControlBase() {}

  virtual void initialize(rclcpp::Node::SharedPtr node, std::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                          std::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                          std::shared_ptr<aerial_robot_navigation::NavigationBase> navigator, double ctrl_loop_dt);

  virtual void activate();
  virtual bool update();

  virtual void reset() { control_timestamp_ = -1; }

protected:
  /* Node handle */
  rclcpp::Node::SharedPtr node_;

  /* Publishers */
  rclcpp::Publisher<spinal::msg::PwmInfo>::SharedPtr motor_info_pub_;
  rclcpp::Publisher<spinal::msg::UavInfo>::SharedPtr uav_info_pub_;

  /* API handles */
  std::shared_ptr<aerial_robot_model::RobotModel> robot_model_;
  std::shared_ptr<aerial_robot_estimation::StateEstimator> estimator_;
  std::shared_ptr<aerial_robot_navigation::NavigationBase> navigator_;
  int estimate_mode_;

  /* Timestamps */
  double activate_timestamp_;
  double control_timestamp_;
  double ctrl_loop_dt_;

  /* Physical configuration */
  int uav_model_;
  int motor_num_;
  double m_f_rate_;
  int pwm_conversion_mode_;
  double min_pwm_, max_pwm_;
  double min_thrust_;
  double force_landing_thrust_;  // PWM
  int vel_ref_num;
  std::vector<spinal::msg::MotorInfo> motor_info_;

  bool param_verbose_;

  template <class T> void getParam(std::string param_name, T &param, T default_value, bool verbose = false)
  {
    node_->get_parameter_or(param_name, param, default_value);

    if (param_verbose_ || verbose)
    {
      rclcpp::Parameter ros_param(param_name, param);
      RCLCPP_INFO(node_->get_logger(), "[%s] %s: %s", node_->get_namespace(), param_name.c_str(),
                  ros_param.value_to_string().c_str());
    }
  }
};
}
