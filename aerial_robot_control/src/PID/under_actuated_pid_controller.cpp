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
#include "aerial_robot_control/PID/under_actuated_pid_controller.hpp"

namespace aerial_robot_control
{

UnderActuatedPIDController::UnderActuatedPIDController()
  : PosePIDControllerBase(), torque_allocation_matrix_inv_pub_stamp_(0)
{
}

void UnderActuatedPIDController::initialize(rclcpp::Node::SharedPtr node,
                                            std::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                            std::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                            std::shared_ptr<aerial_robot_navigation::NavigationBase> navigator,
                                            double ctrl_loop_dt)
{
  PosePIDControllerBase::initialize(node, robot_model, estimator, navigator, ctrl_loop_dt);

  rosParamInit();

  q_mat_inv_.resize(motor_num_, 4);
  target_base_thrust_.resize(motor_num_);

  // Only fill the z-axis since the robot is under-actuated
  pid_msg_.z.total.resize(motor_num_);
  z_limit_ = pid_controllers_.at(Z).getLimitSum();
  pid_controllers_.at(Z).setLimitSum(1e6);  // Do not clamp the sum of PID terms for z axis

  rpy_gain_pub_ = node->create_publisher<spinal::msg::RollPitchYawTerms>("rpy/gain", 1);
  flight_cmd_pub_ = node->create_publisher<spinal::msg::FourAxisCommand>("four_axes/command", 1);
  torque_allocation_matrix_inv_pub_ = node->create_publisher<spinal::msg::TorqueAllocationMatrixInv>(
      "torque_allocation_matrix_inv", 1);
}

void UnderActuatedPIDController::rosParamInit()
{
  getParam<bool>("controller.hovering_approximate", hovering_approximate_, false);
  getParam<double>("controller.torque_allocation_matrix_inv_pub_interval", torque_allocation_matrix_inv_pub_interval_,
                   0.05);
}

void UnderActuatedPIDController::controlCore()
{
  PosePIDControllerBase::controlCore();

  // Wrench allocation matrix
  // TODO: q_mat_inv_ is a static computation, so can be moved to initialization (?) -> adapt this to all controllers
  q_mat_inv_ = getQInv();

  tf2::Vector3 target_acc_w(pid_controllers_.at(X).result(), pid_controllers_.at(Y).result(),
                            pid_controllers_.at(Z).result());

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, rpy_.z());
  tf2::Matrix3x3 rotation_matrix(q);
  tf2::Vector3 target_acc_dash = rotation_matrix.inverse() * target_acc_w;

  Eigen::VectorXd target_thrust_z_term;
  if (hovering_approximate_)
  {
    target_pitch_ = target_acc_dash.x() / aerial_robot_estimation::G;
    target_roll_ = -target_acc_dash.y() / aerial_robot_estimation::G;
    target_thrust_z_term = q_mat_inv_.col(0) * target_acc_w.z();
  }
  else
  {
    target_pitch_ = atan2(target_acc_dash.x(), target_acc_dash.z());
    target_roll_ = atan2(-target_acc_dash.y(),
                         sqrt(target_acc_dash.x() * target_acc_dash.x() + target_acc_dash.z() * target_acc_dash.z()));
    target_thrust_z_term = q_mat_inv_.col(0) * target_acc_w.length();
  }

  // Constraint z (also I-term)
  int index;
  double max_term = target_thrust_z_term.cwiseAbs().maxCoeff(&index);
  double residual = max_term - z_limit_;
  if (residual > 0)
  {
    RCLCPP_DEBUG(node_->get_logger(),
                 "[PID] Position z control exceeded the limit in rotor %d, (max) %f vs (actual) %f ", index, max_term,
                 pid_controllers_.at(Z).getLimitSum());
    pid_controllers_.at(Z).setErrI(pid_controllers_.at(Z).getPrevErrI());
    target_thrust_z_term *= (1 - residual / max_term);
  }

  // Assemble thrust command
  for (int i = 0; i < motor_num_; i++)
  {
    target_base_thrust_.at(i) = target_thrust_z_term(i);
    pid_msg_.z.total.at(i) = target_thrust_z_term(i);
  }

  // Special process for yaw because of the limited bandwidth between PC and spinal
  double max_yaw_scale = 0;  // To reconstruct yaw control term in spinal
  for (unsigned int i = 0; i < motor_num_; i++)
  {
    if (q_mat_inv_(i, YAW - 2) > max_yaw_scale) max_yaw_scale = q_mat_inv_(i, YAW - 2);
  }

  candidate_yaw_term_ = pid_controllers_.at(YAW).result() * max_yaw_scale;
}

void UnderActuatedPIDController::sendCmd()
{
  PosePIDControllerBase::sendCmd();

  sendFourAxisCommand();

  sendTorqueAllocationMatrixInv();
}

void UnderActuatedPIDController::sendFourAxisCommand()
{
  spinal::msg::FourAxisCommand flight_command_data;
  flight_command_data.angles[0] = target_roll_;
  flight_command_data.angles[1] = target_pitch_;
  flight_command_data.angles[2] = candidate_yaw_term_;
  flight_command_data.base_thrust = target_base_thrust_;
  flight_cmd_pub_->publish(flight_command_data);
}

void UnderActuatedPIDController::sendTorqueAllocationMatrixInv()
{
  if (node_->now().seconds() - torque_allocation_matrix_inv_pub_stamp_ > torque_allocation_matrix_inv_pub_interval_)
  {
    torque_allocation_matrix_inv_pub_stamp_ = node_->now().seconds();

    spinal::msg::TorqueAllocationMatrixInv torque_allocation_matrix_inv_msg;
    torque_allocation_matrix_inv_msg.rows.resize(motor_num_);
    Eigen::MatrixXd torque_allocation_matrix_inv = q_mat_inv_.rightCols(3);
    if (torque_allocation_matrix_inv.cwiseAbs().maxCoeff() > INT16_MAX * 0.001f)
    {
      RCLCPP_ERROR(node_->get_logger(), "[PID] Torque Allocation Matrix overflow");
    }
    for (unsigned int i = 0; i < motor_num_; i++)
    {
      torque_allocation_matrix_inv_msg.rows.at(i).x = torque_allocation_matrix_inv(i, 0) * 1000;
      torque_allocation_matrix_inv_msg.rows.at(i).y = torque_allocation_matrix_inv(i, 1) * 1000;
      torque_allocation_matrix_inv_msg.rows.at(i).z = torque_allocation_matrix_inv(i, 2) * 1000;
    }
    torque_allocation_matrix_inv_pub_->publish(torque_allocation_matrix_inv_msg);
  }
}

void UnderActuatedPIDController::reset()
{
  PosePIDControllerBase::reset();

  setAttitudeGains();
}

void UnderActuatedPIDController::setAttitudeGains()
{
  spinal::msg::RollPitchYawTerms rpy_gain_msg;  // for rosserial
  /* Send to flight controller via rosserial scaling by 1000 */
  rpy_gain_msg.motors.resize(1);
  rpy_gain_msg.motors.at(0).roll_p = pid_controllers_.at(ROLL).getPGain() * 1000;
  rpy_gain_msg.motors.at(0).roll_i = pid_controllers_.at(ROLL).getIGain() * 1000;
  rpy_gain_msg.motors.at(0).roll_d = pid_controllers_.at(ROLL).getDGain() * 1000;
  rpy_gain_msg.motors.at(0).pitch_p = pid_controllers_.at(PITCH).getPGain() * 1000;
  rpy_gain_msg.motors.at(0).pitch_i = pid_controllers_.at(PITCH).getIGain() * 1000;
  rpy_gain_msg.motors.at(0).pitch_d = pid_controllers_.at(PITCH).getDGain() * 1000;
  rpy_gain_msg.motors.at(0).yaw_d = pid_controllers_.at(YAW).getDGain() * 1000;
  rpy_gain_pub_->publish(rpy_gain_msg);
}

}

/* Plugin registration */
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::UnderActuatedPIDController, aerial_robot_control::ControlBase);
