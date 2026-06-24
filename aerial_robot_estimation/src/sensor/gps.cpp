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
#include "aerial_robot_estimation/sensor/gps.h"


namespace
{
/* TODO:
 * https://github.com/tu-darmstadt-ros-pkg/hector_gazebo/blob/kinetic-devel/hector_gazebo_plugins/src/gazebo_ros_gps.cpp
 */
/*
 Provides meters-per-degree latitude at a given latitude

  Args:
  lat: latitude
  Returns: meters-per-degree value
*/

double mDegLat(double lat)
{
  double lat_rad = lat * M_PI / 180.0;

  return 111132.09 - 566.05 * cos(2.0 * lat_rad) + 1.20 * cos(4.0 * lat_rad) - 0.002 * cos(6.0 * lat_rad);
}

/*
 Provides meters-per-degree longitude at a given latitude
  Args:
    lat: latitude in decimal degrees
  Returns: meters per degree longitude
*/

double mDegLon(double lat)
{
  double lat_rad = lat * M_PI / 180.0;
  return 111415.13 * cos(lat_rad) - 94.55 * cos(3.0 * lat_rad) - 0.12 * cos(5.0 * lat_rad);
}

}

namespace sensor_plugin
{
Gps::Gps()
  : sensor_plugin::SensorBase(), raw_pos_(0, 0, 0), prev_raw_pos_(0, 0, 0), raw_vel_(0, 0, 0), pos_offset_(0, 0, 0)
{
  gps_state_.states.resize(2);
  gps_state_.states[0].id = "x";
  gps_state_.states[0].state.resize(1);
  gps_state_.states[1].id = "y";
  gps_state_.states[1].state.resize(1);
}

void Gps::initialize(rclcpp::Node::SharedPtr node, std::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                     std::shared_ptr<aerial_robot_estimation::StateEstimator> estimator, std::string sensor_name,
                     int index)
{
  SensorBase::initialize(node, robot_model, estimator, sensor_name, index);
  rosParamInit();

  /* ROS subscriber for gps */
  std::string topic_name;
  getParam<std::string>("gps_sub_name", topic_name, std::string("gps"));
  gps_sub_ = node_->create_subscription<spinal_msgs::msg::Gps>(
      topic_name, rclcpp::SystemDefaultsQoS(), std::bind(&Gps::gpsCallback, this, std::placeholders::_1));

  getParam<std::string>("gps_full_sub_name", topic_name, std::string("gps_full"));
  // gps_full_sub_ = node_->create_subscription<spinal_msgs::msg::GpsFull>(
  //   topic_name, rclcpp::SystemDefaultsQoS(), std::bind(&Gps::gpsFullCallback, this, std::placeholders::_1));

  getParam<std::string>("gps_ros_sub_name", topic_name, std::string("ros_fix"));
  gps_ros_sub_ = node_->create_subscription<sensor_msgs::msg::NavSatFix>(
      topic_name, rclcpp::SystemDefaultsQoS(), std::bind(&Gps::gpsRosCallback, this, std::placeholders::_1));

  if (is_rtk_gps_)
  {
    getParam<std::string>("rtk_pos_sub_name", topic_name, std::string("rtk_pos"));
    rtk_gps_sub_ = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        topic_name, rclcpp::SystemDefaultsQoS(), std::bind(&Gps::rtkGpsCallback, this, std::placeholders::_1));

    aerial_robot_msgs::msg::State z_state;
    z_state.id = "z";
    z_state.state.resize(1);
    gps_state_.states.push_back(z_state);
  }
  else
  {
    /* ROS publisher of sensor_msgs::NavSatFix */
    gps_pub_ = node_->create_publisher<sensor_msgs::msg::NavSatFix>("ros_converted", rclcpp::SystemDefaultsQoS());
  }
}

void Gps::rosParamInit()
{
  getParam<bool>("ned_flag", ned_flag_, true);
  getParam<bool>("only_use_vel", only_use_vel_, true);
  getParam<int>("min_est_sat_num", min_est_sat_num_, 4);
  getParam<double>("pos_noise_sigma", pos_noise_sigma_, 1.0);
  getParam<double>("vel_noise_sigma", vel_noise_sigma_, 0.1);
  getParam<bool>("rtk", is_rtk_gps_, false);
  getParam<bool>("rtk_offset", rtk_offset_, false);

  if (is_rtk_gps_)
    only_use_pos_ = true;
  else
    only_use_pos_ = false;
}

void Gps::gpsCallback(const spinal_msgs::msg::Gps::SharedPtr gps_msg)
{
  if (!updateBase2SensorTF()) return;

  /* Check other gps modules */
  bool has_rtk_gps = false;
  for (const auto &handler : estimator_->getGpsHandlers())
  {
    if (handler.get() == this) continue;

    if (std::dynamic_pointer_cast<sensor_plugin::Gps>(handler)->isRtk())
    {
      has_rtk_gps = true;
      only_use_vel_ = true;  // if has RTK GPS, we do not use the position information of normal GPS for fusion
      RCLCPP_DEBUG(logger_, "[GPS] Has another RTK GPS");
      break;
    }
  }

  /* Temporal update */
  double curr_timestamp = rclcpp::Time(gps_msg->stamp).seconds() + delay_;

  /* Assignment lat/lon, velocity */
  curr_wgs84_point_ = geodesy::toMsg(gps_msg->location[0], gps_msg->location[1]);
  if (!has_rtk_gps || is_rtk_gps_) estimator_->setCurrGpsPoint(curr_wgs84_point_);

  raw_vel_ = KDL::Vector(gps_msg->velocity[0], -gps_msg->velocity[1], 0);  // NED frame -> XYZ frame

  /* To get the correction rotation and omega of baselink with the consideration of time delay */
  bool imu_initialized = false;
  for (const auto &handler : estimator_->getImuHandlers())
  {
    if (handler->getStatus() == Status::ACTIVE)
    {
      imu_initialized = true;
      break;
    }
  }
  if (!imu_initialized)
  {
    RCLCPP_WARN_THROTTLE(logger_, *node_->get_clock(), 1.0, "[GPS] The IMU is not initialized, wait...");
    return;
  }

  /* Fusion process */
  /* Quit if the satellite number is too low */
  if (gps_msg->sat_num >= min_est_sat_num_)
  {
    if (getStatus() == Status::INVALID) setStatus(prev_status_);
  }
  if (gps_msg->sat_num < min_est_sat_num_)
  {
    if (getStatus() == Status::ACTIVE)
      RCLCPP_WARN_THROTTLE(logger_, *node_->get_clock(), 1.0, "[GPS] Number of satellites is not enough: %d",
                           gps_msg->sat_num);
    setStatus(Status::INVALID);
  }

  if (getStatus() == Status::INACTIVE)
  {
    setStatus(Status::INIT);

    if (!is_rtk_gps_) activate();

    /* Set base position */
    base_wgs84_point_ = curr_wgs84_point_;
    RCLCPP_WARN(logger_, "[GPS] Base lat/lon: [%f, %f] deg for %s GPS", base_wgs84_point_.latitude,
                base_wgs84_point_.longitude, is_rtk_gps_ ? "RTK" : "normal");
    return;
  }

  if (is_rtk_gps_) return;  // Do not do esimation

  /* Get the position and velocity  w.r.t. the local frame (the origin is the initial takeoff place) */
  KDL::Rotation r = KDL::Rotation::Identity();
  KDL::Vector omega(0, 0, 0);
  int mode = aerial_robot_estimation::EGOMOTION_ESTIMATE;
  if (!estimator_->findBaseRotOmega(curr_timestamp, mode, r, omega) && estimator_->getFlyingFlag())
    RCLCPP_WARN_STREAM(logger_, "[GPS] Omega is not updated from findBaseRotOmega");

  raw_vel_ += r * (-(omega * sensor_rel_pose_.p));  // Offset from gps to baselink

  KDL::Rotation convert_frame = KDL::Rotation::RPY(M_PI, 0, 0);  // NED -> XYZ
  raw_pos_ = convert_frame * Gps::wgs84ToNedLocalFrame(base_wgs84_point_, curr_wgs84_point_) - r * sensor_rel_pose_.p;

  /* Update timestamp for estimation */
  curr_timestamp_ = curr_timestamp;

  estimateProcess();

  /* Update the timestamp */
  gps_state_.header.stamp = gps_msg->stamp;
  gps_state_.states[0].state[0].pos = raw_pos_[0];
  gps_state_.states[0].state[0].vel = raw_vel_[0];
  gps_state_.states[1].state[0].pos = raw_pos_[1];
  gps_state_.states[1].state[0].vel = raw_vel_[1];
  state_pub_->publish(gps_state_);

  /* Update */
  prev_raw_pos_ = raw_pos_;
  updateHealthStamp();
}

void Gps::gpsFullCallback(const spinal_msgs::msg::GpsFull::SharedPtr gps_full_msg)
{
  /* Time */
  struct tm time = { 0 };
  time.tm_year = gps_full_msg->year - 1900;
  time.tm_mon = gps_full_msg->month - 1;
  time.tm_mday = gps_full_msg->day;
  time.tm_hour = gps_full_msg->hour;
  time.tm_min = gps_full_msg->min;
  time.tm_sec = gps_full_msg->sec;
  auto time_sec = mkgmtime(&time);

  double fix_ros_time = static_cast<double>(time_sec);
  if (gps_full_msg->nano < 0)
  {
    fix_ros_time -= 1.0;
    fix_ros_time += static_cast<double>(gps_full_msg->nano + 1e9) * 1e-9;
  }
  else
  {
    fix_ros_time += static_cast<double>(gps_full_msg->nano) * 1e-9;
  }

  RCLCPP_DEBUG(logger_, "[GPS] UTC time %f; spinal time: %f", fix_ros_time,
               rclcpp::Time(gps_full_msg->stamp).seconds() + delay_);

  auto gps_msg = std::make_shared<spinal_msgs::msg::Gps>();
  gps_msg->stamp = gps_full_msg->stamp;
  gps_msg->location[0] = gps_full_msg->location[0];
  gps_msg->location[1] = gps_full_msg->location[1];
  gps_msg->velocity[0] = gps_full_msg->velocity[0];
  gps_msg->velocity[1] = gps_full_msg->velocity[1];
  gps_msg->sat_num = gps_full_msg->sat_num;

  gpsCallback(gps_msg);
}

void Gps::gpsRosCallback(const sensor_msgs::msg::NavSatFix::SharedPtr gps_msg)
{
  /* TODO: add velocity */
  auto spinal_gps_msg = std::make_shared<spinal_msgs::msg::Gps>();
  spinal_gps_msg->stamp = gps_msg->header.stamp;
  spinal_gps_msg->location[0] = gps_msg->latitude;
  spinal_gps_msg->location[1] = gps_msg->longitude;

  if (gps_msg->status.status >= sensor_msgs::msg::NavSatStatus::STATUS_FIX)
    spinal_gps_msg->sat_num = min_est_sat_num_;  // Temporarily
  gpsCallback(spinal_gps_msg);
}


void Gps::rtkGpsCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr rtk_gps_msg)
{
  auto cov = rtk_gps_msg->pose.covariance;
  if (cov[0] == 0 || cov[7] == 0 || cov[14] == 0)
  {
    // TODO: implement dynamic recovery if RTK GPS is disconnected in air
    RCLCPP_WARN_THROTTLE(
        logger_, *node_->get_clock(), 1.0,
        "[GPS] RTK GPS is not convergent, not recommend to fly. If this is the first bringup, please wait "
        "for few minute for covergence");
    return;
  }


  /* Temporal update */
  curr_timestamp_ = rclcpp::Time(rtk_gps_msg->header.stamp).seconds() + delay_;

  /* Pos accuracy */
  pos_noise_sigma_ = cov[0];

  /* Get the position */
  KDL::Rotation r = KDL::Rotation::Identity();
  KDL::Vector omega(0, 0, 0);
  int mode = aerial_robot_estimation::EGOMOTION_ESTIMATE;
  if (!estimator_->findBaseRotOmega(curr_timestamp_, mode, r, omega) && estimator_->getFlyingFlag())
    RCLCPP_WARN_STREAM(logger_, "[GPS] Omega is not updated from findBaseRotOmega");

  KDL::Vector raw_pos(rtk_gps_msg->pose.pose.position.x, rtk_gps_msg->pose.pose.position.y,
                      rtk_gps_msg->pose.pose.position.z);

  raw_pos_ = raw_pos - r * sensor_rel_pose_.p;

  if (getStatus() == Status::INIT) pos_offset_ = raw_pos_;

  if (rtk_offset_) raw_pos_ -= pos_offset_;
  // TODO: add offset for z axis if we use altitude from RTK GPS

  if (getStatus() == Status::INIT)
  {
    activate();
    setStatus(Status::ACTIVE);
  }

  estimateProcess();

  /* Update the timestamp */
  gps_state_.header.stamp = rtk_gps_msg->header.stamp;
  gps_state_.states[0].state[0].pos = raw_pos_[0];
  gps_state_.states[1].state[0].pos = raw_pos_[1];
  gps_state_.states[2].state[0].pos = raw_pos_[2];
  state_pub_->publish(gps_state_);

  /* Update */
  updateHealthStamp();
}

void Gps::estimateProcess()
{
  if (getStatus() == Status::INVALID) return;

  /* Collaboration wit VO */
  if (estimator_->getVoHandlers().size() > 0 && !only_use_vel_)
  {
    for (const auto &handler : estimator_->getVoHandlers())
    {
      if (handler->getStatus() == Status::ACTIVE && std::dynamic_pointer_cast<sensor_plugin::Odometry>(handler) &&
          std::dynamic_pointer_cast<sensor_plugin::Odometry>(handler)->odomPosMode())
      {
        RCLCPP_WARN(logger_, "[GPS] VO pose odom mode, so only use vel");
        only_use_vel_ = true;
        break;
      }
    }
  }

  /* Fuser for 0: egomotion, 1: experiment */
  for (int mode = 0; mode < 2; mode++)
  {
    for (auto &fuser : estimator_->getFuserList(mode))
    {
      std::string plugin_name = fuser.first;
      std::shared_ptr<kf_plugin::KalmanFilter> kf = fuser.second;

      int id = kf->getId();
      if ((id & (1 << State::X)) || (id & (1 << State::Y)))
      {
        if (plugin_name == "kalman_filter/kf_pos_vel_acc")
        {
          /* Correction */
          Eigen::VectorXd measure_sigma(1);
          measure_sigma << vel_noise_sigma_;

          int index = id >> (State::X + 1);

          if (only_use_pos_)
          {
            Eigen::VectorXd meas(1);
            meas << raw_pos_[index];
            std::vector<double> params = { kf_plugin::POS };
            Eigen::VectorXd measure_sigma(1);
            measure_sigma << pos_noise_sigma_;
            kf->correction(meas, measure_sigma, time_sync_ ? curr_timestamp_ : -1, params);
          }
          else if (only_use_vel_)
          {
            Eigen::VectorXd meas(1);
            meas << raw_vel_[index];
            std::vector<double> params = { kf_plugin::VEL };
            Eigen::VectorXd measure_sigma(1);
            measure_sigma << vel_noise_sigma_;
            kf->correction(meas, measure_sigma, time_sync_ ? curr_timestamp_ : -1, params);
          }
          else
          {
            Eigen::VectorXd measure_sigma(2);
            measure_sigma << pos_noise_sigma_, vel_noise_sigma_;
            Eigen::VectorXd meas(2);
            meas << raw_pos_[index], raw_vel_[index];
            std::vector<double> params = { kf_plugin::POS_VEL };
            kf->correction(meas, measure_sigma, time_sync_ ? curr_timestamp_ : -1, params);
          }
        }
      }
    }
  }
}

void Gps::activate()
{
  if (!estimator_->getBasePosStateStatus(State::X, aerial_robot_estimation::EGOMOTION_ESTIMATE) ||
      !estimator_->getBasePosStateStatus(State::Y, aerial_robot_estimation::EGOMOTION_ESTIMATE))
  {
    RCLCPP_WARN(logger_, "[GPS] Start GPS Kalman filter");

    /* Fuser for 0: egomotion, 1: experiment */
    for (int mode = 0; mode < 2; mode++)
    {
      for (auto &fuser : estimator_->getFuserList(mode))
      {
        std::shared_ptr<kf_plugin::KalmanFilter> kf = fuser.second;
        int id = kf->getId();

        std::string plugin_name = fuser.first;
        if ((id & (1 << State::X)) || (id & (1 << State::Y)))
        {
          if (plugin_name == "kalman_filter/kf_pos_vel_acc")
          {
            kf->setInitState(raw_pos_[id >> (State::X + 1)], 0);
            kf->setMeasureFlag();
          }
        }
      }
    }
  }

  /* Set the status */
  estimator_->setBasePosStateStatus(State::X, aerial_robot_estimation::EGOMOTION_ESTIMATE, true);
  estimator_->setBasePosStateStatus(State::Y, aerial_robot_estimation::EGOMOTION_ESTIMATE, true);
  setStatus(Status::ACTIVE);
}


/*
 AlvinXY: Lat/Long to X/Y (NED)
  Converts Lat/Lon (WGS84) to Alvin XYs using a Mercator projection.
*/
KDL::Vector Gps::wgs84ToNedLocalFrame(geographic_msgs::msg::GeoPoint base_point,
                                      geographic_msgs::msg::GeoPoint target_point)
{
  return KDL::Vector((target_point.latitude - base_point.latitude) * mDegLat(base_point.latitude),
                     (target_point.longitude - base_point.longitude) * mDegLon(base_point.latitude), 0);
}


/*
  X/Y (NED) to Lat/Lon
  Converts Alvin XYs to Lat/Lon (WGS84) using a Mercator projection.
*/
geographic_msgs::msg::GeoPoint Gps::NedLocalFrameToWgs84(KDL::Vector diff_pos,
                                                         geographic_msgs::msg::GeoPoint base_point)
{
  geographic_msgs::msg::GeoPoint target_point;

  target_point.longitude = diff_pos.y() / mDegLon(base_point.latitude) + base_point.longitude;
  target_point.latitude = diff_pos.x() / mDegLat(base_point.latitude) + base_point.latitude;
  return target_point;
}


time_t Gps::mkgmtime(struct tm *const tmp)
{
  int dir;
  int bits;
  int saved_seconds;
  time_t t;
  struct tm yourtm, *mytm;

  yourtm = *tmp;
  saved_seconds = yourtm.tm_sec;
  yourtm.tm_sec = 0;
  /*
  ** Calculate the number of magnitude bits in a time_t
  ** (this works regardless of whether time_t is
  ** signed or unsigned, though lint complains if unsigned).
  */
  for (bits = 0, t = 1; t > 0; ++bits, t <<= 1)
    ;
  /*
  ** If time_t is signed, then 0 is the median value,
  ** if time_t is unsigned, then 1 << bits is median.
  */
  t = (t < 0) ? 0 : ((time_t)1 << bits);

  /* Some gmtime() implementations are broken and will return
   * NULL for time_ts larger than 40 bits even on 64-bit platforms
   * so we'll just cap it at 40 bits */
  if (bits > 40) bits = 40;

  for (;;)
  {
    mytm = gmtime(&t);

    if (!mytm) return -1;

    dir = tmcomp(mytm, &yourtm);
    if (dir != 0)
    {
      if (bits-- < 0) return -1;
      if (bits < 0)
        --t;
      else if (dir > 0)
        t -= (time_t)1 << bits;
      else
        t += (time_t)1 << bits;
      continue;
    }
    break;
  }
  t += saved_seconds;
  return t;
}

int Gps::tmcomp(const struct tm *const atmp, const struct tm *const btmp)
{
  int result;

  if ((result = (atmp->tm_year - btmp->tm_year)) == 0 && (result = (atmp->tm_mon - btmp->tm_mon)) == 0 &&
      (result = (atmp->tm_mday - btmp->tm_mday)) == 0 && (result = (atmp->tm_hour - btmp->tm_hour)) == 0 &&
      (result = (atmp->tm_min - btmp->tm_min)) == 0)
    result = atmp->tm_sec - btmp->tm_sec;
  return result;
}
}

/* Plugin registration */
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(sensor_plugin::Gps, sensor_plugin::SensorBase);
