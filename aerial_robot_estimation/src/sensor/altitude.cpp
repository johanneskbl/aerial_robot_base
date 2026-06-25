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
#include "aerial_robot_estimation/sensor/altitude.h"


namespace
{
int calibrate_cnt = 0;
double range_previous_secs = 0.0;
}

namespace sensor_plugin
{
AltitudeSensor::AltitudeSensor()
  : sensor_plugin::SensorBase(),
    /* Range sensor */
    raw_range_sensor_value_(0),
    raw_range_pos_z_(0),
    prev_raw_range_pos_z_(0),
    raw_range_vel_z_(0),
    min_range_(0),
    max_range_(0),
    range_sensor_sanity_(TOTAL_SANE),
    range_sensor_offset_(0),
    range_sensor_hz_(0),
    /* Barometer */
    raw_baro_pos_z_(0),
    baro_pos_z_(0),
    prev_raw_baro_pos_z_(0),
    baro_vel_z_(0),
    raw_baro_vel_z_(0),
    baro_temp_(0),
    high_filtered_baro_pos_z_(0),
    prev_high_filtered_baro_pos_z_(0),
    high_filtered_baro_vel_z_(0),
    /* Terrain state */
    alt_estimate_mode_(ONLY_BARO_MODE),
    state_on_terrain_(NORMAL),
    inflight_state_(false),
    first_outlier_val_(0),
    t_ab_(0),
    t_ab_incre_(0),
    height_offset_(0)
{
  alt_state_.states.resize(2);
  alt_state_.states[0].id = "range_sensor";
  alt_state_.states[0].state.resize(2);
  alt_state_.states[1].id = "baro";
  alt_state_.states[1].state.resize(3);

  /* Set health chan num */
  setHealthChanNum(2);
}

void AltitudeSensor::initialize(rclcpp::Node::SharedPtr node,
                                std::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                std::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                std::string sensor_name, int index)
{
  SensorBase::initialize(node, robot_model, estimator, sensor_name, index);
  rosParamInit();

  kf_loader_ptr_ = std::make_shared<pluginlib::ClassLoader<kf_plugin::KalmanFilter>>("kalman_filter",
                                                                                     "kf_plugin::KalmanFilter");
  baro_bias_kf_ = kf_loader_ptr_->createSharedInstance("aerial_robot_base/kf_baro_bias");
  baro_bias_kf_->initialize(std::string(""), 0);

  baro_lpf_filter_ = IirFilter(sample_freq_, cutoff_freq_);
  baro_lpf_high_filter_ = IirFilter(sample_freq_, high_cutoff_freq_);

  /* Terrain check */
  if (!terrain_check_with_baro_) state_on_terrain_ = NORMAL;

  /* Range sensor */
  std::string topic_name;
  getParam<std::string>("range_sensor_sub_name", topic_name, std::string("distance"));
  range_sensor_sub_ = node_->create_subscription<sensor_msgs::msg::Range>(
      topic_name, rclcpp::SystemDefaultsQoS(), std::bind(&AltitudeSensor::rangeCallback, this, std::placeholders::_1));

  alt_mode_sub_ = node_->create_subscription<std_msgs::msg::UInt8>(
      "estimate_alt_mode", rclcpp::SystemDefaultsQoS(),
      std::bind(&AltitudeSensor::altEstimateModeCallback, this, std::placeholders::_1));


  /* Barometer */
  // barometer_sub_ = node_->create_subscription<spinal_msgs::msg::Barometer>(
  //     barometer_sub_name_, rclcpp::SystemDefaultsQoS(), std::bind(&AltitudeSensor::baroCallback, this,
  //                                                                 std::placeholders::_1));
}

void AltitudeSensor::rangeCallback(const sensor_msgs::msg::Range::SharedPtr range_msg)
{
  double current_secs = rclcpp::Time(range_msg->header.stamp).seconds();

  if (getStatus() != Status::ACTIVE && estimator_->getForceAttControlFlag())
  {
    sensor_status_ = Status::ACTIVE;
    RCLCPP_WARN(logger_, "[altitude] Force activate range sensor in force att contorl mode");
  }

  if (!updateBase2SensorTF()) return;

    /* Consider the orientation of the uav */
#if 0
  float roll = (estimator_->getState(State::ROLL_BASE, aerial_robot_estimation::EGOMOTION_ESTIMATE))[0];
  float pitch = (estimator_->getState(State::PITCH_BASE, aerial_robot_estimation::EGOMOTION_ESTIMATE))[0];
  /* Add the offset from the base_link to the sensor */
  KDL::Matrix3x3 tilt_r; tilt_r.setRPY(roll, pitch, 0);
  double raw_range_sensor_value = cos(roll) * cos(pitch) * range_msg->range - (tilt_r * sensor_rel_pose_.p).z();
#endif

  raw_range_sensor_value_ = -(estimator_->getBaseOrientation(aerial_robot_estimation::EGOMOTION_ESTIMATE) *
                              (sensor_rel_pose_ * KDL::Vector(0, 0, range_msg->range)))
                                 .z();

  /* Calibrate phase */
  if (calibrate_cnt > 0)
  {
    if (calibrate_cnt == calibrate_cnt_)
    {
      range_previous_secs = current_secs;
      setStatus(Status::INIT);
    }

    calibrate_cnt--;
    sensor_hz_ += (current_secs - range_previous_secs);
    range_sensor_offset_ -= raw_range_sensor_value_;

    /* Initialize */
    if (calibrate_cnt == 0)
    {
      /* Set the range of the sensor value */
      max_range_ = range_msg->max_range;
      min_range_ = range_msg->min_range;

      /* Check the sanity of the range sensor value */
      if (range_msg->max_range <= range_msg->min_range)
      {
        calibrate_cnt = 1;
        RCLCPP_ERROR(logger_, "[altitude] Range sensor: the min/max range is not correct");
        return;
      }

      if (max_flight_height_ < 0) max_flight_height_ = max_range_;

      sensor_hz_ /= (float)(calibrate_cnt_ - 1);
      range_sensor_offset_ /= (float)calibrate_cnt_;
      /* The initial height offset is equal with the hardware initial height offset */
      height_offset_ = range_sensor_offset_;

      /* Check the sanity of the first height (height_offset) */
      if (-height_offset_ < min_range_ || -height_offset_ > max_range_)
      {
        /* Sonar sensor should be here */
        RCLCPP_WARN(logger_, "[altitude] The range sensor is attached close to the ground");

        range_sensor_sanity_ = TOTAL_INSANE;

        /* The offset (init) should be 0 */
        height_offset_ = 0;
        range_sensor_offset_ = 0;

        /* Set the undescending mode because we may only use imu for z(alt) estimation */
        estimator_->setUnDescendMode(true);
      }

      /* Set the height offset to be zero, if the sensor is too closed to the ground */
      if (no_height_offset_)
      {
        height_offset_ = 0;
        range_sensor_offset_ = 0;
      }

      /* Fuser for 0: egomotion, 1: experiment */
      for (int mode = 0; mode < 2; mode++)
      {
        if (!isModeActivate(mode)) continue;

        for (auto &fuser : estimator_->getFuserList(mode))
        {
          std::shared_ptr<kf_plugin::KalmanFilter> kf = fuser.second;
          int id = kf->getId();

          if (id & (1 << State::Z))
          {
            kf->setMeasureFlag();
            kf->setInitState(raw_range_sensor_value_ + height_offset_, 0);
          }
        }
      }

      /* Change the alt estimate mode */
      alt_estimate_mode_ = WITHOUT_BARO_MODE;

      /* Set the status for Z (altitude) */
      estimator_->setBasePosStateStatus(State::Z, aerial_robot_estimation::EGOMOTION_ESTIMATE, true);

      setStatus(Status::ACTIVE);  // Active

      RCLCPP_WARN(
          logger_, "[altitude] %s: the range sensor offset: %f, initial sanity: %s, the hz is %f, estimate mode is %d",
          (range_msg->radiation_type == sensor_msgs::msg::Range::ULTRASOUND) ? std::string("sonar sensor").c_str() :
                                                                               std::string("infrared sensor").c_str(),
          range_sensor_offset_, range_sensor_sanity_ ? std::string("true").c_str() : std::string("false").c_str(),
          1.0 / sensor_hz_, alt_estimate_mode_);

      range_previous_secs = current_secs;
      prev_raw_range_pos_z_ = raw_range_pos_z_;
      return;
    }
  }

  /* Update */
  raw_range_pos_z_ = raw_range_sensor_value_ + height_offset_;
  raw_range_vel_z_ = (raw_range_pos_z_ - prev_raw_range_pos_z_) / (current_secs - range_previous_secs);

  /* Only for sonar in takeoff and landing phase
     before takeoff: total_sane -> total_insane
     takeoff: total_insane -> potentially_insane
     landing: potentially_insane -> total_insane */
  switch (range_sensor_sanity_)
  {
    case TOTAL_INSANE:
      if (!estimator_->getSensorFusionFlag())
      {
        /* This is for the repeat mode */
        if (!estimator_->getSensorFusionFlag()) calibrate_cnt = 0;

        for (int mode = 0; mode < 2; mode++)
        {
          if (!isModeActivate(mode)) continue;

          for (auto &fuser : estimator_->getFuserList(mode))
          {
            std::shared_ptr<kf_plugin::KalmanFilter> kf = fuser.second;
            int id = kf->getId();
            if (id & (1 << State::Z))
            {
              kf->setMeasureFlag(false);
              kf->resetState();
            }
          }
        }
        return;
      }

      /* First ascending phase */
      if (raw_range_pos_z_ < min_range_ + ascending_check_range_ && raw_range_pos_z_ > min_range_ &&
          prev_raw_range_pos_z_ < min_range_ && prev_raw_range_pos_z_ > min_range_ - ascending_check_range_)
      {
        RCLCPP_WARN(logger_,
                    "[altitude] Insanity %s: confirm ascending to sanity height, start sf correction process, "
                    "previous height: %f",
                    (range_msg->radiation_type == sensor_msgs::msg::Range::ULTRASOUND) ?
                        std::string("sonar sensor").c_str() :
                        std::string("infrared sensor").c_str(),
                    prev_raw_range_pos_z_);

        /* Release the non-descending mode, use the range sensor for z(alt) estimation */
        estimator_->setUnDescendMode(false);

        for (int mode = 0; mode < 2; mode++)
        {
          if (!isModeActivate(mode)) continue;

          for (auto &fuser : estimator_->getFuserList(mode))
          {
            RCLCPP_INFO(logger_, "[altitude] Debug sonar: init test:");
            std::shared_ptr<kf_plugin::KalmanFilter> kf = fuser.second;
            int id = kf->getId();
            if (id & (1 << State::Z))
            {
              kf->setInitState(raw_range_pos_z_, 0);
              kf->setMeasureFlag();
            }
          }
        }

        range_sensor_sanity_ = POTENTIALLY_INSANE;
      }
      return;
    case POTENTIALLY_INSANE:
      if (prev_raw_range_pos_z_ < min_range_ + ascending_check_range_ && prev_raw_range_pos_z_ > min_range_ &&
          raw_range_pos_z_ < min_range_ && raw_range_pos_z_ > min_range_ - ascending_check_range_)
      {
        RCLCPP_WARN(logger_,
                    "[altitude] Potentially insane %s: confirm descending to insane zone, stop sf correction process, "
                    "previous height: %f",
                    (range_msg->radiation_type == sensor_msgs::msg::Range::ULTRASOUND) ?
                        std::string("sonar sensor").c_str() :
                        std::string("infrared sensor").c_str(),
                    prev_raw_range_pos_z_);

        range_sensor_sanity_ = TOTAL_INSANE;
        return;
      }
      break;
    default:
      break;
  }

  /* Terrain check and height estimate */
  alt_state_.header.stamp = range_msg->header.stamp;
  if (terrainProcess(current_secs)) rangeEstimateProcess();

  /* Publish phase */
  alt_state_.states[0].state[0].pos = raw_range_pos_z_;
  alt_state_.states[0].state[0].vel = raw_range_vel_z_;

  state_pub_->publish(alt_state_);
  updateHealthStamp(1);  // Channel: 1

  /* Update */
  range_previous_secs = current_secs;
  prev_raw_range_pos_z_ = raw_range_pos_z_;
}

void AltitudeSensor::rangeEstimateProcess()
{
  if (getStatus() == Status::INVALID) return;

  Matrix<double, 1, 1> temp = MatrixXd::Zero(1, 1);

  for (int mode = 0; mode < 2; mode++)
  {
    if (!isModeActivate(mode)) continue;

    for (auto &fuser : estimator_->getFuserList(mode))
    {
      std::string plugin_name = fuser.first;
      std::shared_ptr<kf_plugin::KalmanFilter> kf = fuser.second;
      int id = kf->getId();
      if (id & (1 << State::Z))
      {
        if (plugin_name == "kalman_filter/kf_pos_vel_acc")
        {
          /* Correction */
          Eigen::VectorXd measure_sigma(1);
          measure_sigma << range_noise_sigma_;
          Eigen::VectorXd meas(1);
          meas << raw_range_pos_z_;
          std::vector<double> params = { kf_plugin::POS };

          kf->correction(meas, measure_sigma, time_sync_ ? (rclcpp::Time(alt_state_.header.stamp).seconds()) : -1,
                         params);
        }
      }
    }
  }
}

bool AltitudeSensor::terrainProcess(double current_secs)
{
  if (getStatus() == Status::INVALID) return false;

  std::shared_ptr<kf_plugin::KalmanFilter> kf = nullptr;
  if (!isModeActivate(aerial_robot_estimation::EGOMOTION_ESTIMATE))
  {
    RCLCPP_ERROR(logger_, "[altitude] Range sensor is not used in EGOMOTION_ESTIMATE mode");
    return false;
  }

  for (auto &fuser : estimator_->getFuserList(aerial_robot_estimation::EGOMOTION_ESTIMATE))
  {
    if (fuser.second->getId() & (1 << State::Z))
    {
      if (!kf)
        kf = fuser.second;
      else
      {
        RCLCPP_ERROR(logger_, "[altitude] More than one kalman filter estimating z axis is detected");
        return false;
      }
    }
  }

  if (terrain_check_with_baro_)
  {
    switch (state_on_terrain_)
    {
      case NORMAL:
        /* Flow chat 1 */
        /* Check the height of the range sensor when height exceeds limitation */
        if (raw_range_sensor_value_ < min_range_ || raw_range_sensor_value_ > max_range_)
        {
          state_on_terrain_ = MAX_EXCEED;
          alt_estimate_mode_ = ONLY_BARO_MODE;
          RCLCPP_WARN(logger_, "[altitude] Exceed the range of the sensor, change to only baro estimate mode");
          return false;
        }

        /* Flow chat 2 */
        /* Check the difference between the estimated value and raw range value */
        if (fabs((kf->getEstimateState())(0) - raw_range_pos_z_) > outlier_threshold_)
        {
          t_ab_ = current_secs;
          t_ab_incre_ = current_secs;
          first_outlier_val_ = raw_range_sensor_value_;
          state_on_terrain_ = ABNORMAL;
          RCLCPP_WARN(logger_,
                      "[altitude] Range sensor: we find the outlier value in NORMAL mode, switch to ABNORMAL mode, the "
                      "sensor value is "
                      "%f, the estimator value is %f, the first outlier value is %f",
                      raw_range_pos_z_, (kf->getEstimateState())(0), first_outlier_val_);
          return false;
        }

        return true;
      case ABNORMAL:

        /* Update the t_ab_incre_ for the second level oulier check */
        if (fabs(raw_range_sensor_value_ - first_outlier_val_) > outlier_threshold_)
        {
          RCLCPP_WARN(logger_,
                      "[altitude] Range sensor: update the t_ab_incre and first_outlier_val_, since the sensor value "
                      "is vibrated, "
                      "sensor value:%f, first outlier value: %f",
                      raw_range_sensor_value_, first_outlier_val_);
          t_ab_incre_ = current_secs;
          first_outlier_val_ = raw_range_sensor_value_;
        }

        if (current_secs - t_ab_ < check_duration1_)
        {
          /* The first level to check the outlier: recover to the last normal mode */
          /* We don't have to update the height_offset */
          if (fabs((kf->getEstimateState())(0) - raw_range_pos_z_) < inlier_threshold_)
          {
            state_on_terrain_ = NORMAL;
            return true;
          }
        }
        else
        {
          /* The second level to check the outlier: find new terrain */
          if (current_secs - t_ab_incre_ > check_duration2_)
          {
            state_on_terrain_ = NORMAL;
            height_offset_ = (kf->getEstimateState())(0) - raw_range_sensor_value_;
            /* Also update the landing height */
            estimator_->setLandingHeight(height_offset_ - range_sensor_offset_);
            RCLCPP_WARN(logger_, "[altitude] We find the new terrain, the new height_offset is %f", height_offset_);
            return true;
          }
        }
        return false;
      case MAX_EXCEED:
        /* The sensor value is below the max value ath the MAX_EXCEED state */
        /*We first turn back to ABNORMAL mode to verify the validity of the value */
        if (raw_range_sensor_value_ < max_range_ && raw_range_sensor_value_ > min_range_) state_on_terrain_ = ABNORMAL;
        return false;
      default:
        return false;
    }
  }
  else
  {
    if (estimator_->getForceAttControlFlag()) return true;

    /*
 Heuristic check method:
      check the validity of visual odometry by range sensor, with the assumption that there is no terrain change
    */
    if (estimator_->getVoHandlers().size() > 0 && raw_range_pos_z_ > max_flight_height_)
    {
      bool odom_active = false;
      for (const auto &handler : estimator_->getVoHandlers())
      {
        if (handler->getStatus() == Status::ACTIVE) odom_active = true;
      }

      if (odom_active)
      {
        /* TODO: find the invalid odom sensor, and only reset the invalid one */
        RCLCPP_WARN(logger_,
                    "[altitude] Reset all odom sensor, because the value of range sensor exceeds the max flight "
                    "height: %f, prev raw "
                    "range pos z: %f, kf pos z: %f",
                    raw_range_sensor_value_, prev_raw_range_pos_z_, (kf->getEstimateState())(0));
        for (const auto &handler : estimator_->getVoHandlers()) handler->reset();
        return true;
      }
    }
    return true;
  }
  return false;
}

void AltitudeSensor::baroCallback(const spinal_msgs::msg::Barometer::SharedPtr baro_msg)
{
  static double baro_previous_secs;
  double current_secs = rclcpp::Time(baro_msg->stamp).seconds();

  raw_baro_pos_z_ = baro_msg->altitude;
  baro_temp_ = baro_msg->temperature;

  /* First Filtering: IIR filter */
  /* Position */
  baro_pos_z_ = baro_lpf_filter_.filterFunction(raw_baro_pos_z_);
  high_filtered_baro_pos_z_ = baro_lpf_high_filter_.filterFunction(raw_baro_pos_z_);
  /* Velocity */
  raw_baro_vel_z_ = (raw_baro_pos_z_ - prev_raw_baro_pos_z_) / (current_secs - baro_previous_secs);
  baro_vel_z_ = (baro_pos_z_ - prev_baro_pos_z_) / (current_secs - baro_previous_secs);
  high_filtered_baro_vel_z_ = (high_filtered_baro_pos_z_ - prev_high_filtered_baro_pos_z_) /
                              (current_secs - baro_previous_secs);

  /* The true initial phase for baro based estimattion for inflight state */
  /* Since the value of pressure will decrease during the rising of the propeller rotation speed */
  if (((alt_estimate_mode_ == ONLY_BARO_MODE && high_filtered_baro_vel_z_ > 0.1) ||
       alt_estimate_mode_ == WITHOUT_BARO_MODE) &&
      estimator_->getFlyingFlag() && !inflight_state_)
  {  // The inflight state should be with the velocity of 0.1(up)
    inflight_state_ = true;
    RCLCPP_WARN(logger_, "[altitude] Barometer: start the inflight barometer height estimation");

    /* The initialization of the baro bias kf filter */
    Eigen::VectorXd input_sigma(1);
    input_sigma << baro_bias_noise_sigma_;
    baro_bias_kf_->setPredictionNoiseCovariance(input_sigma);
    baro_bias_kf_->setInputFlag();
    baro_bias_kf_->setMeasureFlag();
    baro_bias_kf_->setInitState(-baro_pos_z_, 0);
  }
  /* Reset */
  if (estimator_->getLandedFlag()) inflight_state_ = false;
  baroEstimateProcess(rclcpp::Time(baro_msg->stamp));

  /* Publish */
  alt_state_.header.stamp = baro_msg->stamp;
  alt_state_.states[1].state[0].pos = raw_baro_pos_z_;
  alt_state_.states[1].state[0].vel = raw_baro_vel_z_;
  alt_state_.states[1].state[1].pos = baro_pos_z_;
  alt_state_.states[1].state[1].vel = baro_vel_z_;
  alt_state_.states[1].state[2].pos = high_filtered_baro_pos_z_;
  alt_state_.states[1].state[2].vel = high_filtered_baro_vel_z_;

  state_pub_->publish(alt_state_);

  /* Update */
  baro_previous_secs = current_secs;
  prev_raw_baro_pos_z_ = raw_baro_pos_z_;
  prev_baro_pos_z_ = baro_pos_z_;
  prev_high_filtered_baro_pos_z_ = high_filtered_baro_pos_z_;
  updateHealthStamp(0);  // Channel: 0
}

void AltitudeSensor::baroEstimateProcess(rclcpp::Time stamp)
{
  if (getStatus() == Status::INVALID) return;

  if (!inflight_state_) return;

  switch (alt_estimate_mode_)
  {
    case ONLY_BARO_MODE:
      for (int mode = 0; mode < 2; mode++)
      {
        if (!isModeActivate(mode)) continue;

        for (auto &fuser : estimator_->getFuserList(mode))
        {
          std::string plugin_name = fuser.first;
          std::shared_ptr<kf_plugin::KalmanFilter> kf = fuser.second;
          int id = kf->getId();
          if (id & (1 << State::Z))
          {
            if (!kf->getFilteringFlag())
            {
              // RCLCPP_FATAL(logger_, "[altitude] Barometer: can not estiamte the height by baro(ONLY_BARO_MODE),
              // because the filtering flag is not activated");
              return;
            }
            /* We should set the sigma every time, since we may have several different sensors to correct the kalman
             * filter(e.g. odom + opti, laser + baro) */

            if (plugin_name == "kalman_filter/kf_pos_vel_acc")
            {
              /* Correction */
              Eigen::VectorXd measure_sigma(1);
              measure_sigma << baro_noise_sigma_;
              Eigen::VectorXd meas(1);
              meas << baro_pos_z_ + (baro_bias_kf_->getEstimateState())(0);
              std::vector<double> params = { kf_plugin::POS };
              kf->correction(meas, measure_sigma, -1, params);
            }

            /* Set the state */
            // Eigen::VectorXd state = kf->getEstimateState();
            // estimator_->setState(State::Z_BASE, mode, 0, state(0));
            // estimator_->setState(State::Z_BASE, mode, 1, state(1));
            // alt_state_.states[0].state[1].pos = state(0);
            // alt_state_.states[0].state[1].vel = state(1);
            alt_state_.states[0].state[1].acc = (baro_bias_kf_->getEstimateState())(0);
          }
        }
      }
      break;
    case WITHOUT_BARO_MODE: {
      baro_bias_kf_->prediction(Eigen::VectorXd::Zero(1), stamp.seconds());
      Eigen::VectorXd meas(1);
      meas << estimator_->getBasePos(aerial_robot_estimation::EGOMOTION_ESTIMATE)[2] - baro_pos_z_;
      baro_bias_kf_->correction(meas, Eigen::VectorXd::Zero(1), -1);
    }
    break;
    case WITH_BARO_MODE:
      // TODO: this is another part, maybe we have to use another package:
      // http://wiki.ros.org/ethzasl_sensor_fusion
      break;
    default:
      break;
  }
}

/* Force to change the estimate mode */
void AltitudeSensor::altEstimateModeCallback(const std_msgs::msg::UInt8::SharedPtr mode_msg)
{
  alt_estimate_mode_ = mode_msg->data;
  RCLCPP_INFO(logger_, "[altitude] Change the height estimate mode: %d", alt_estimate_mode_);
}

void AltitudeSensor::rosParamInit()
{
  /* Range sensor */
  getParam<double>("range_noise_sigma", range_noise_sigma_, 0.005);
  getParam<int>("calibrate_cnt", calibrate_cnt_, 100);
  getParam<bool>("no_height_offset", no_height_offset_, true);

  calibrate_cnt = calibrate_cnt_;

  /* First ascending process: check range */
  getParam<double>("ascending_check_range", ascending_check_range_, 0.1);  // [m]

  /* for odom validity check */
  getParam<double>("max_flight_height", max_flight_height_, -1);  // [m]

  /* for terrain and outlier check */
  getParam<double>("outlier_threshold", outlier_threshold_, 1.0);  // [m]
  getParam<double>("inlier_threshold", inlier_threshold_, 0.5);    // [m]
  getParam<bool>("terrain_check_with_baro", terrain_check_with_baro_, false);
  getParam<double>("check_du1", check_duration1_, 0.1);  // [sec]
  getParam<double>("check_du2", check_duration2_, 1.0);  // [sec]

  /* Barometer */
  getParam<std::string>("barometer_sub_name", barometer_sub_name_, std::string("/baro"));
  getParam<double>("baro_noise_sigma", baro_noise_sigma_, 0.05);
  getParam<double>("baro_bias_noise_sigma", baro_bias_noise_sigma_, 0.001);
  getParam<double>("sample_freq", sample_freq_, 100.0);
  getParam<double>("cutoff_freq", cutoff_freq_, 10.0);
  getParam<double>("high_cutoff_freq", high_cutoff_freq_, 1.0);
}

void AltitudeSensor::changeStatus(bool flag)
{
  if (flag)
  {
    sensor_status_ = Status::ACTIVE;
    RCLCPP_INFO_STREAM(logger_, "[altitude] " << node_->get_namespace() << ", set to active");
  }
  else
  {
    sensor_status_ = Status::INVALID;
    RCLCPP_INFO_STREAM(logger_, "[altitude] " << node_->get_namespace() << ", set to invalid");
  }
}
}

/* Plugin registration */
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(sensor_plugin::AltitudeSensor, sensor_plugin::SensorBase);
