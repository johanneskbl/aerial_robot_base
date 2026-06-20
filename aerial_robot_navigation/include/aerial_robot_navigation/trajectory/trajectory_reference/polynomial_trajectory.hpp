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
#include "aerial_robot_navigation/trajectory/reference_base.hpp"
#include "aerial_robot_navigation/trajectory/trajectory_reference/polynomial.hpp"
#include "aerial_robot_navigation/trajectory/types/quadrotor.hpp"
#include "aerial_robot_navigation/trajectory/math/gravity.hpp"


namespace agi
{

template <class PolyType = Polynomial<>> class PolynomialTrajectory : public ReferenceBase
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  PolynomialTrajectory(const QuadState &start_state, const QuadState &end_state,
                       const Vector<> &weights = Vector<4>(0, 0, 0, 1), const int order = 11, const int continuity = -1,
                       const std::string &name = "Polynomial Trajectory");
  PolynomialTrajectory(const std::vector<QuadState> &states, const Vector<> &weights = Vector<4>(0, 0, 0, 1),
                       const int order = 11, const int continuity = -1,
                       const std::string &name = "Polynomial Trajectory");
  virtual ~PolynomialTrajectory() = default;

  using ReferenceBase::getSetpoint;
  Setpoint getSetpoint(const QuadState &state, const Scalar time) override;
  QuadState getState(const Scalar time) const;
  virtual Setpoint getStartSetpoint() override final;
  virtual Setpoint getEndSetpoint() override final;

  bool addStateConstraint(const QuadState &state, int ord = -1);
  bool solved() const;
  bool valid() const override;

  // Utilities
  Vector<> evalTranslation(const Scalar time, const int order = 0) const;
  Scalar findTimeMaxAcc(const Scalar precision = 1e-3) const;
  Scalar findTimeMaxOmega(const Scalar precision = 1e-3) const;
  Scalar findTimeMaxAcc(const Scalar t_start, const Scalar t_end, const Scalar precision = 1e-3) const;
  Scalar findTimeMaxOmega(const Scalar t_start, const Scalar t_end, const Scalar precision = 1e-3) const;
  void scale(const Scalar start_time = NAN, const Scalar duration = NAN);
  Scalar scaleToLimits(const Quadrotor &quad, const int iterations = 10, const Scalar tolerance = 1e-4);
  Scalar scaleToLimits(const Scalar acc_limit, const int iterations = 10, const Scalar tolerance = 1e-4);

  void setForwardHeading(const bool forward);

protected:
  template <typename EvalFunc, typename CompFunc> Scalar findTime(const Scalar dt, const Scalar dt_min,
                                                                  const Scalar t_start, const Scalar t_end,
                                                                  EvalFunc eval, CompFunc comp) const;

  QuadState end_state_;
  PolyType x_;
  PolyType y_;
  PolyType z_;
  PolyType yaw_;
  QuadState prev_constraint_;

  std::vector<QuadState> states_;

  mutable Quaternion q_pitch_roll_last_{ 1, 0, 0, 0 };
  mutable Scalar yaw_last_{ 0.0 };
  bool forward_heading_{ false };
};

// Child classes
class MinSnapTrajectory : public PolynomialTrajectory<Polynomial<>>
{
public:
  MinSnapTrajectory(const QuadState &start_state, const QuadState &end_state, const int order = 11,
                    const int continuity = 3, const std::string &name = "Minimum Snap Trajectory")
    : PolynomialTrajectory(start_state, end_state, Vector<4>(0, 0, 0, 1), order, continuity, name)
  {
  }
  MinSnapTrajectory(const std::vector<QuadState> &states, const int order = 11, const int continuity = 3,
                    const std::string &name = "Minimum Snap Trajectory")
    : PolynomialTrajectory(states, Vector<4>(0, 0, 0, 1), order, continuity, name)
  {
  }
};

class MinJerkTrajectory : public PolynomialTrajectory<Polynomial<>>
{
public:
  MinJerkTrajectory(const QuadState &start_state, const QuadState &end_state, const int order = 11,
                    const int continuity = 2, const std::string &name = "Minimum Jerk Trajectory")
    : PolynomialTrajectory(start_state, end_state, Vector<3>(0, 0, 1), order, continuity, name)
  {
  }
  MinJerkTrajectory(const std::vector<QuadState> &states, const int order = 11, const int continuity = 2,
                    const std::string &name = "Minimum Jerk Trajectory")
    : PolynomialTrajectory(states, Vector<3>(0, 0, 1), order, continuity, name)
  {
  }
};

// Closed-Form Minimum-Jerk Specialization
template <> PolynomialTrajectory<ClosedFormMinJerkAxis>::PolynomialTrajectory(const QuadState &start_state,
                                                                              const QuadState &end_state,
                                                                              const Vector<> &weights, const int order,
                                                                              const int continuity,
                                                                              const std::string &name);

template <> PolynomialTrajectory<ClosedFormMinJerkAxis>::PolynomialTrajectory(const std::vector<QuadState> &states,
                                                                              const Vector<> &weights, const int order,
                                                                              const int continuity,
                                                                              const std::string &name) = delete;

class ClosedFormMinJerkTrajectory : public PolynomialTrajectory<ClosedFormMinJerkAxis>
{
public:
  ClosedFormMinJerkTrajectory(const QuadState &start_state, const QuadState &end_state)
    : PolynomialTrajectory(start_state, end_state, Vector<3>(0, 0, 1), 5, 3, "Closed-Form Mininum Jerk Trajectory")
  {
  }
};

}
