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
#include "aerial_robot_control/LQI/under_actuated_tilted_lqi_controller.hpp"

namespace aerial_robot_control
{

// TODO: Doesn't underactuated + tilted = fully actuated? If so change naming & descriptions everywhere

void UnderActuatedTiltedLQIController::initialize(rclcpp::Node::SharedPtr node,
                                                  std::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                                  std::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                                  std::shared_ptr<aerial_robot_navigation::NavigationBase> navigator,
                                                  double ctrl_loop_dt)
{
  UnderActuatedLQIController::initialize(node, robot_model, estimator, navigator, ctrl_loop_dt);

  desired_baselink_rot_pub_ = node_->create_publisher<spinal::msg::DesireCoord>("desire_coordinate", 1);

  pid_msg_.z.p_term.resize(1);
  pid_msg_.z.i_term.resize(1);
  pid_msg_.z.d_term.resize(1);
  z_limit_ = pid_controllers_.at(Z).getLimitSum();
  pid_controllers_.at(Z).setLimitSum(1e6);  // Do not clamp the sum of PID terms for z-axis
}

void UnderActuatedTiltedLQIController::controlCore()
{
  PosePIDControllerBase::controlCore();

  target_acc_w_ = tf2::Vector3(pid_controllers_.at(X).result(), pid_controllers_.at(Y).result(),
                               pid_controllers_.at(Z).result());

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, rpy_.z());
  tf2::Matrix3x3 rotation_matrix(q);
  tf2::Vector3 target_acc_dash = rotation_matrix.inverse() * target_acc_w_;

  target_pitch_ = atan2(target_acc_dash.x(), target_acc_dash.z());
  target_roll_ = atan2(-target_acc_dash.y(),
                       sqrt(target_acc_dash.x() * target_acc_dash.x() + target_acc_dash.z() * target_acc_dash.z()));

  if (navigator_->getForceLandingFlag())
  {
    target_pitch_ = 0;
    target_roll_ = 0;
  }

  allocateZTerm();

  allocateYawTerm();
}

void UnderActuatedTiltedLQIController::allocateZTerm()
{
  Eigen::VectorXd f = robot_model_->getStaticThrust();
  Eigen::VectorXd g = robot_model_->getGravity();
  Eigen::VectorXd allocate_scales = f / g.norm();
  Eigen::VectorXd target_thrust_z_term = allocate_scales * target_acc_w_.length();

  // Constraint z (also I-term)
  int index;
  double max_term = target_thrust_z_term.cwiseAbs().maxCoeff(&index);
  double residual = max_term - z_limit_;

  if (residual > 0)
  {
    pid_controllers_.at(Z).setErrI(pid_controllers_.at(Z).getPrevErrI());
    target_thrust_z_term *= (1 - residual / max_term);
  }

  for (int i = 0; i < motor_num_; i++)
  {
    target_base_thrust_.at(i) = target_thrust_z_term(i);
    pid_msg_.z.total.at(i) = target_thrust_z_term(i);
  }
}

bool UnderActuatedTiltedLQIController::optimalGain()
{
  /* Calculate the P_orig pseudo inverse */
  Eigen::MatrixXd P = robot_model_->calcWrenchMatrixOnCoG();
  Eigen::MatrixXd inertia = robot_model_->getInertia<Eigen::Matrix3d>();
  Eigen::MatrixXd P_dash = inertia.inverse() * P.bottomRows(3);  // Roll, pitch, yaw

  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(9, 9);
  Eigen::MatrixXd B = Eigen::MatrixXd::Zero(9, motor_num_);
  Eigen::MatrixXd C = Eigen::MatrixXd::Zero(3, 9);
  for (int i = 0; i < 3; i++)
  {
    A(2 * i, 2 * i + 1) = 1;
    B.row(2 * i + 1) = P_dash.row(i);
    C(i, 2 * i) = 1;
  }
  A.block(6, 0, 3, 9) = -C;

  RCLCPP_DEBUG_STREAM(node_->get_logger(), "[LQI] Gain generator: B: \n" << B);

  Eigen::VectorXd q_diagonals(9);
  q_diagonals << lqi_roll_pitch_weight_(0), lqi_roll_pitch_weight_(2), lqi_roll_pitch_weight_(0),
      lqi_roll_pitch_weight_(2), lqi_yaw_weight_(0), lqi_yaw_weight_(2), lqi_roll_pitch_weight_(1),
      lqi_roll_pitch_weight_(1), lqi_yaw_weight_(1);
  Eigen::MatrixXd Q = q_diagonals.asDiagonal();

  Eigen::MatrixXd P_trans = P.topRows(3) / robot_model_->getMass();
  Eigen::MatrixXd R_trans = P_trans.transpose() * P_trans;
  Eigen::MatrixXd R_input = Eigen::MatrixXd::Identity(motor_num_, motor_num_);
  Eigen::MatrixXd R = R_trans * trans_constraint_weight_ + R_input * att_control_weight_;

  double t = node_->get_clock()->now().seconds();
  bool use_kleinman_method = true;
  if (K_.cols() == 0 || K_.rows() == 0) use_kleinman_method = false;
  if (!control_utils::care(A, B, R, Q, K_, use_kleinman_method))
  {
    RCLCPP_ERROR(node_->get_logger(),
                 "[LQI] Gain generator: Error in solver of continuous-time algebraic Riccati equation (CARE)");
    return false;
  }

  RCLCPP_DEBUG_STREAM(node_->get_logger(),
                      "[LQI] Gain generator: CARE: " << node_->get_clock()->now().seconds() - t << " sec");
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "[LQI] Gain generator: K \n" << K_);

  for (int i = 0; i < motor_num_; ++i)
  {
    roll_gains_.at(i) = Eigen::Vector3d(-K_(i, 0), K_(i, 6), -K_(i, 1));
    pitch_gains_.at(i) = Eigen::Vector3d(-K_(i, 2), K_(i, 7), -K_(i, 3));
    yaw_gains_.at(i) = Eigen::Vector3d(-K_(i, 4), K_(i, 8), -K_(i, 5));
  }

  return true;
}

void UnderActuatedTiltedLQIController::sendGain()
{
  UnderActuatedLQIController::sendGain();

  double roll, pitch, yaw;
  robot_model_->getCogDesireOrientation<KDL::Rotation>().GetRPY(roll, pitch, yaw);

  spinal::msg::DesireCoord coord_msg;
  coord_msg.roll = roll;
  coord_msg.pitch = pitch;
  // TODO: Why not also publish the desired yaw?
  desired_baselink_rot_pub_->publish(coord_msg);
}

void UnderActuatedTiltedLQIController::rosParamInit()
{
  UnderActuatedLQIController::rosParamInit();

  std::string lqi_ns = "controller.lqi.";
  getParam<double>(lqi_ns + "trans_constraint_weight", trans_constraint_weight_, 1.0);
  getParam<double>(lqi_ns + "att_control_weight", att_control_weight_, 1.0);
}

}

/* Plugin registration */
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::UnderActuatedTiltedLQIController, aerial_robot_control::ControlBase);
