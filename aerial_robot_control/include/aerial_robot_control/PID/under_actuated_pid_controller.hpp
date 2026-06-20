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
#include "aerial_robot_control/PID/pose_pid_controller_base.hpp"
#include "spinal/msg/four_axis_command.hpp"
#include "spinal/msg/roll_pitch_yaw_terms.hpp"
#include "spinal/msg/torque_allocation_matrix_inv.hpp"


namespace aerial_robot_control
{
class UnderActuatedPIDController : public PosePIDControllerBase
{
public:
  UnderActuatedPIDController();
  virtual ~UnderActuatedPIDController() = default;

  virtual void initialize(rclcpp::Node::SharedPtr node, std::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                          std::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                          std::shared_ptr<aerial_robot_navigation::NavigationBase> navigator,
                          double ctrl_loop_dt) override;

  virtual void reset() override;
  // For update() use implementation in parent class

protected:
  rclcpp::Publisher<spinal::msg::FourAxisCommand>::SharedPtr flight_cmd_pub_;                              // for spinal
  rclcpp::Publisher<spinal::msg::RollPitchYawTerms>::SharedPtr rpy_gain_pub_;                              // for spinal
  rclcpp::Publisher<spinal::msg::TorqueAllocationMatrixInv>::SharedPtr torque_allocation_matrix_inv_pub_;  // for spinal
  double torque_allocation_matrix_inv_pub_stamp_;

  Eigen::MatrixXd q_mat_inv_;

  double target_roll_, target_pitch_;  // Under-actuated
  double candidate_yaw_term_;
  std::vector<float> target_base_thrust_;

  double torque_allocation_matrix_inv_pub_interval_;

  double z_limit_;

  bool hovering_approximate_;

  void setAttitudeGains();
  virtual void rosParamInit();
  virtual void controlCore() override;
  virtual void sendCmd() override;
  virtual void sendFourAxisCommand();
  virtual void sendTorqueAllocationMatrixInv();
};
}
