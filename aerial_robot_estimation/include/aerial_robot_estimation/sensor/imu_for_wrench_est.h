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
#include <algorithm>
#include <array>
#include <vector>
#include <mutex>
#include <stdexcept>

/* ROS 2 */
#include <geometry_msgs/msg/accel_stamped.hpp>

/* Aerial robot packages */
#include "aerial_robot_estimation/sensor/imu.h"

namespace digital_filter
{

// --------------------------------------------------------------------------
//  FIRFilter: directform FIR, compiletime length (uses std::vector)
// --------------------------------------------------------------------------

template <std::size_t N> class FIRFilter
{
  static_assert(N > 0, "FIRFilter length must be > 0");

public:
  FIRFilter() : coeffs_(N, 0.0), hist_(N, 0.0) {}

  // Load coefficients and optional gain
  void setCoeffs(const std::vector<double> &b, double gain = 1.0)
  {
    if (b.size() != N) throw std::invalid_argument("FIR taps != N");
    for (std::size_t i = 0; i < N; ++i) coeffs_[i] = b[i] * gain;
  }

  void reset()
  {
    primed_ = false;  // Reset the primed flag
  }

  [[nodiscard]] constexpr std::size_t order() const noexcept { return N - 1; }

  // Process one sample
  double filter(double x_n)
  {
    if (!primed_)
    {
      resetWMeas(x_n);  // If not primed, reset with the first sample
    }

    hist_[idx_] = x_n;  // Overwrite oldest sample
    double acc = 0.0;
    std::size_t tap = idx_;
    for (std::size_t k = 0; k < N; ++k)
    {
      acc += coeffs_[k] * hist_[tap];
      tap = (tap == 0) ? N - 1 : tap - 1;  // Circular buffer walk
    }
    idx_ = (idx_ + 1) % N;
    return acc;
  }

private:
  bool primed_{ false };        // True if filter is primed
  std::vector<double> coeffs_;  // b_{0} ... b_{N1}
  std::vector<double> hist_;    // Circular buffer of past inputs
  std::size_t idx_{};           // Write index

  // Prime the internal delay line so that the first output equals `y0`
  void resetWMeas(double y0)
  {
    std::fill(hist_.begin(), hist_.end(), y0);
    idx_ = 0;
    primed_ = true;  // Primed means the filter is ready to process samples
  }
};
}

namespace sensor_plugin
{

class Imu4WrenchEst : public sensor_plugin::Imu
{
public:
  void initialize(rclcpp::Node::SharedPtr node, std::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                  std::shared_ptr<aerial_robot_estimation::StateEstimator> estimator, std::string sensor_name,
                  int index) override;

  bool reset() override;

  void setOmegaCogInCog(const KDL::Vector &omega_cog_in_cog)
  {
    std::lock_guard<std::mutex> lock(omega_mutex_);
    omega_cog_in_cog_ = omega_cog_in_cog;
  }

  void setVelCogInW(const KDL::Vector &vel_cog_in_w)
  {
    std::lock_guard<std::mutex> lock(vel_mutex_);
    vel_cog_in_w_ = vel_cog_in_w;
  }

  void setOmegaDotCogInCog(const KDL::Vector &omega_dot_cog_in_cog)
  {
    std::lock_guard<std::mutex> lock(omega_mutex_);
    omega_dot_cog_in_cog_ = omega_dot_cog_in_cog;
  }

  void setAccCogInCog(const KDL::Vector &acc_cog_in_cog)
  {
    std::lock_guard<std::mutex> lock(vel_mutex_);
    acc_cog_in_cog_ = acc_cog_in_cog;
  }

  KDL::Vector getOmegaCogInCog()
  {
    std::lock_guard<std::mutex> lock(omega_mutex_);
    return omega_cog_in_cog_;
  }

  KDL::Vector getVelCogInW()
  {
    std::lock_guard<std::mutex> lock(vel_mutex_);
    return vel_cog_in_w_;
  }

  KDL::Vector getOmegaDotCogInCog()
  {
    std::lock_guard<std::mutex> lock(omega_mutex_);
    return omega_dot_cog_in_cog_;
  }

  KDL::Vector getAccCogInCog()
  {
    std::lock_guard<std::mutex> lock(vel_mutex_);
    return acc_cog_in_cog_;
  }

protected:
  void imuCallback(const spinal_msgs::msg::Imu::SharedPtr msg) override;

  // Semaphores
  std::mutex omega_mutex_;
  std::mutex vel_mutex_;

  // Data
  KDL::Vector vel_cog_in_w_;          // CoG point, world frame
  KDL::Vector acc_cog_in_cog_;        // CoG point, CoG frame, aligned with IMU raw data
  KDL::Vector omega_cog_in_cog_;      // CoG point, CoG frame
  KDL::Vector omega_dot_cog_in_cog_;  // CoG point, CoG frame (numerical derivative)

  // FIR filter to smooth numerical derivative of omega
  std::array<digital_filter::FIRFilter<5>, 3> omega_diff_;

  // Publisher
  rclcpp::Publisher<geometry_msgs::msg::AccelStamped>::SharedPtr pub_acc_;
};

}