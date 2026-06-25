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
#include "aerial_robot_estimation/sensor/imu_for_wrench_est.h"


namespace sensor_plugin
{
void Imu4WrenchEst::initialize(rclcpp::Node::SharedPtr node,
                               std::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                               std::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                               std::string sensor_name, int index)
{
  Imu::initialize(node, robot_model, estimator, std::string("sensor_plugin/imu"), index);

  // FIR differentiator for omega dot: 5-point differentiator
  std::vector<double> diffB = { -1, 8, 0, -8, 1 };
  double gain = 1.0 / 12.0;
  for (auto &f : omega_diff_)
  {
    f.setCoeffs(diffB, gain);
    f.reset();
  }

  // Debug
  std::string topic_name = sensor_name + std::to_string(index) + "/acc_lin_ang_baselink";
  pub_acc_ = node_->create_publisher<geometry_msgs::msg::AccelStamped>(topic_name, rclcpp::SystemDefaultsQoS());
  RCLCPP_INFO(logger_, "[IMU] Type: Imu4WrenchEst");
}

bool Imu4WrenchEst::reset()
{
  for (auto &f : omega_diff_) f.reset();
  return true;
}

// override to get filtered gyro data
void Imu4WrenchEst::imuCallback(const spinal_msgs::msg::Imu::SharedPtr msg)
{
  // Check the validity first
  for (int i = 0; i < 3; i++)
  {
    if (std::isnan(msg->acc[i]) ||
        // std::isnan(msg->angles[i]) ||
        std::isnan(msg->gyro[i]) || std::isnan(msg->mag[i]))
    {
      RCLCPP_ERROR_THROTTLE(logger_, *node_->get_clock(), 1.0, "[IMU] Sensor publishes NaN value!");
      return;
    }
  }

  geometry_msgs::msg::Quaternion q;
  q.x = msg->quaternion[0];
  q.y = msg->quaternion[1];
  q.z = msg->quaternion[2];
  q.w = msg->quaternion[3];

  if (isnan(q.x) || isnan(q.y) || isnan(q.z) || isnan(q.w))
  {
    RCLCPP_ERROR_THROTTLE(logger_, *node_->get_clock(), 1.0, "[IMU] Plugin receives NaN value in Quaternion!");
    return;
  }

  acc_b_ = KDL::Vector(msg->acc[0], msg->acc[1], msg->acc[2]);
  // g_b_ = KDL::Vector(msg->angles[0], msg->angles[1], msg->angles[2]);
  omega_ = KDL::Vector(msg->gyro[0], msg->gyro[1], msg->gyro[2]);
  mag_ = KDL::Vector(msg->mag[0], msg->mag[1], msg->mag[2]);
  raw_rot_ = aerial_robot_model::msgToKdl(q);

  // Compute omega dot via FIR differentiator
  // Note: at 200 Hz the 5-point filter introduces delay of 2 * 5ms = 10 ms.
  KDL::Vector omega_b_dot(omega_diff_[0].filter(omega_.x()), omega_diff_[1].filter(omega_.y()),
                          omega_diff_[2].filter(omega_.z()));

  // Publish linear and angular acceleration for debugging
  geometry_msgs::msg::AccelStamped acc_msg;
  acc_msg.header.stamp = msg->stamp;
  acc_msg.accel.linear.x = acc_b_.x();
  acc_msg.accel.linear.y = acc_b_.y();
  acc_msg.accel.linear.z = acc_b_.z();
  acc_msg.accel.angular.x = omega_b_dot.x();
  acc_msg.accel.angular.y = omega_b_dot.y();
  acc_msg.accel.angular.z = omega_b_dot.z();
  pub_acc_->publish(acc_msg);

  // Coordinate transform
  KDL::Frame cog2baselink_tf = robot_model_->getCog2Baselink<KDL::Frame>();
  int estimate_mode = estimator_->getEstimateMode();
  setOmegaCogInCog(cog2baselink_tf.M * omega_);

  const KDL::Vector baselink_vel = estimator_->getBaseVel(estimate_mode);
  const KDL::Rotation baselink_rot = estimator_->getBaseOrientation(estimate_mode);
  const KDL::Vector baselink2cog_pos = cog2baselink_tf.Inverse().p;
  setVelCogInW(baselink_vel + baselink_rot * (omega_ * baselink2cog_pos));  // In KDL, "*" between two vectors is the
                                                                            // cross product

  // TODO: this is a simple version of the acceleration estimation. Need to improve.
  setAccCogInCog(cog2baselink_tf.M * acc_b_);
  setOmegaDotCogInCog(cog2baselink_tf.M * omega_b_dot);

  // Main process
  time_stamp_ = msg->stamp;
  estimateProcess();
  prev_time_stamp_ = time_stamp_;
  updateHealthStamp();
}

}

/* Plugin registration */
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(sensor_plugin::Imu4WrenchEst, sensor_plugin::SensorBase);
