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
#include <cmath>

/* ROS 2 */
#include <geodesy/utm.h>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geographic_msgs/msg/geo_point.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nav_msgs/msg/odometry.hpp>

/* Kalman filter library */
#include "kalman_filter/kf_pos_vel_acc_plugin.h"

/* Aerial robot packages */
#include "aerial_robot_estimation/sensor/base_plugin.h"
#include "aerial_robot_estimation/sensor/odom.h"
#include "spinal/msg/gps.hpp"
#include "spinal/msg/gps_full.hpp"

/* TODO:
   1. gps redundant proccess to improce the accuracy of position estimation
   2. better way point control which is from sensor fusion but not only gps
   3. interaction between single gps and rtk gps
*/

namespace sensor_plugin
{

class Gps : public sensor_plugin::SensorBase
{
public:
  void initialize(rclcpp::Node::SharedPtr node, std::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                  std::shared_ptr<aerial_robot_estimation::StateEstimator> estimator, std::string sensor_name,
                  int index) override;

  ~Gps() {}
  Gps();

  static KDL::Vector wgs84ToNedLocalFrame(geographic_msgs::msg::GeoPoint base_pos,
                                          geographic_msgs::msg::GeoPoint target_pos);
  static geographic_msgs::msg::GeoPoint NedLocalFrameToWgs84(KDL::Vector diff_pos,
                                                             geographic_msgs::msg::GeoPoint base_pos);

  const bool isRtk() const { return is_rtk_gps_; }
  const geographic_msgs::msg::GeoPoint getBasePoint() const { return base_wgs84_point_; }
  const geographic_msgs::msg::GeoPoint getCurrentPoint() const { return curr_wgs84_point_; }

private:
  /* ROS */
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub_;
  rclcpp::Subscription<spinal::msg::Gps>::SharedPtr gps_sub_;
  rclcpp::Subscription<spinal::msg::GpsFull>::SharedPtr gps_full_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_ros_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr rtk_gps_sub_;

  /* ROS param */
  double pos_noise_sigma_, vel_noise_sigma_;
  int min_est_sat_num_;

  bool ned_flag_;
  bool only_use_vel_;
  bool only_use_pos_;
  bool is_rtk_gps_; /* Self is RTK GPS */
  bool rtk_offset_; /* Set the takeoff point as zero point for RTK mode */

  aerial_robot_msgs::msg::States gps_state_;

  geographic_msgs::msg::GeoPoint base_wgs84_point_, curr_wgs84_point_;

  KDL::Vector raw_pos_, prev_raw_pos_;
  KDL::Vector raw_vel_;
  KDL::Vector pos_offset_;

  void gpsCallback(const spinal::msg::Gps::SharedPtr gps_msg);
  void gpsFullCallback(const spinal::msg::GpsFull::SharedPtr gps_full_msg);
  void gpsRosCallback(const sensor_msgs::msg::NavSatFix::SharedPtr gps_msg);
  void rtkGpsCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr gps_msg);
  void estimateProcess();
  void rosParamInit();
  void activate();

  /* Utc time */
  /* https://github.com/KumarRobotics/ublox/blob/master/ublox_gps/include/ublox_gps/mkgmtime.h */
  time_t mkgmtime(struct tm *const tmp);
  int tmcomp(const struct tm *const atmp, const struct tm *const btmp);
};
}
