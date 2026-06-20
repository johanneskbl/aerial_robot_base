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

/* ROS 2 */
#include <pluginlib/class_loader.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <sensor_msgs/msg/range.hpp>

/* Kalman filter library */
#include "kalman_filter/kf_pos_vel_acc_plugin.h"

/* Aerial robot packages */
#include "aerial_robot_estimation/sensor/base_plugin.h"
#include "spinal/msg/barometer.hpp"

namespace sensor_plugin
{
class AltitudeSensor : public sensor_plugin::SensorBase
{
public:
  void initialize(rclcpp::Node::SharedPtr node, std::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                  std::shared_ptr<aerial_robot_estimation::StateEstimator> estimator, std::string sensor_name,
                  int index) override;

  ~AltitudeSensor() override = default;
  AltitudeSensor();

  int getRangeSensorSanity() const { return range_sensor_sanity_; }
  int getStateOnTerrain() const { return state_on_terrain_; }

  /* The height estimation related function */
  static constexpr uint8_t ONLY_BARO_MODE = 0;     // We estimate the height only based the baro, but the bias of baro
                                                   // is constexprant(keep the last eistamted value)
  static constexpr uint8_t WITH_BARO_MODE = 1;     // We estimate the height using range sensor etc. without the baro,
                                                   // but we are estimate the bias of baro
  static constexpr uint8_t WITHOUT_BARO_MODE = 2;  // We estimate the height using range sensor etc. with the baro,
                                                   // also estimating the bias of baro

  /* The state of the height sensor value in terms of estimation  */
  static constexpr uint8_t NORMAL = 0;
  static constexpr uint8_t ABNORMAL = 1;
  static constexpr uint8_t MAX_EXCEED = 2;

  /* The criteria to express the sanity of the range sensor */
  /* E.g. the sonar which is attached at the bottom of the uav can not measure the distance at the moment before takeoff
   * and after landing. so the sensr(value) is insane. Also, we have to consider that the sensor may become insane at
   * the moment of landing or to low. */
  static constexpr int TOTAL_INSANE = -1;       // The state before takeoff and after landing
  static constexpr int POTENTIALLY_INSANE = 0;  // The inflight state which can be insane potentially
  static constexpr int TOTAL_SANE = 1;          // The totally sane state.

private:
  /* ROS */
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr alt_mode_sub_;
  /* Range sensor */
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr range_sensor_sub_;
  /* Barometer */
  rclcpp::Subscription<spinal::msg::Barometer>::SharedPtr barometer_sub_;
  rclcpp::Publisher<spinal::msg::Barometer>::SharedPtr barometer_pub_;

  /* ROS param */
  /* Range sensor */
  KDL::Vector range_origin_; /* The origin of range based on cog of UAV */
  bool no_height_offset_;
  double range_noise_sigma_;
  int calibrate_cnt_;
  /* Barometer */
  std::string barometer_sub_name_;
  double baro_noise_sigma_, baro_bias_noise_sigma_;
  /* The iir filter for the barometer for the first filtering stage */
  double sample_freq_, cutoff_freq_, high_cutoff_freq_;

  /* Base variables */
  /* Range sensor */
  double raw_range_sensor_value_;
  double raw_range_pos_z_, prev_raw_range_pos_z_, raw_range_vel_z_;
  double min_range_, max_range_;
  double ascending_check_range_; /* Merge range around the min range */
  float range_sensor_offset_;    // The offset of the sensor value due to the attachment(hardware);
  float range_sensor_hz_;
  int range_sensor_sanity_; /* for the (initial) sanity of the senser in terms of the attachment hardware */

  /* Barometer */
  /* The kalman filter for the baro bias estimation */
  std::shared_ptr<pluginlib::ClassLoader<kf_plugin::KalmanFilter>> kf_loader_ptr_;
  std::shared_ptr<kf_plugin::KalmanFilter> baro_bias_kf_;
  IirFilter baro_lpf_filter_, baro_lpf_high_filter_;
  bool inflight_state_;  // The flag for the inflight state
  double raw_baro_pos_z_, baro_pos_z_, prev_raw_baro_pos_z_, prev_baro_pos_z_;
  double raw_baro_vel_z_, baro_vel_z_;
  double high_filtered_baro_pos_z_, prev_high_filtered_baro_pos_z_, high_filtered_baro_vel_z_;
  double baro_temp_;  // The temperature of the chip

  /* for vo validity check */
  double max_flight_height_;

  /* for terrain check */
  int alt_estimate_mode_;
  bool terrain_check_with_baro_;  // The flag to enable or disable the terrain check
  double outlier_threshold_;      // The check threshold between the estimated value and sensor value: e.g. 10 times
  double inlier_threshold_;       // The check threshold between the estimated value and sensor value: e.g. 5 times
  double check_duration1_;  // The duration to check the sensor value in terms of the big noise, short time (e.g. 0.1s)
  double check_duration2_;  // The duration to check the sensor value in terms of the new flat terrain, long time (e.g.
                            // 1s)
  uint8_t state_on_terrain_;  // The sensor state(normal or abnormal)
  double t_ab_;               // The time when the state becomes abnormal.
  double t_ab_incre_;         // The increment time from  t_ab_.
  double first_outlier_val_;  // The first sensor value during the check_duration2_;
  float height_offset_; /* General offset between esimated height and range sensor value, maybe change beacause of the
                           terrain */

  aerial_robot_msgs::msg::States alt_state_;

  void rangeCallback(const sensor_msgs::msg::Range::SharedPtr range_msg);
  void rangeEstimateProcess();
  bool terrainProcess(double current_secs);
  void baroCallback(const spinal::msg::Barometer::SharedPtr baro_msg);
  void baroEstimateProcess(rclcpp::Time stamp);
  void altEstimateModeCallback(const std_msgs::msg::UInt8::SharedPtr mode_msg);
  void rosParamInit() override;
  void changeStatus(bool flag) override;
};
}