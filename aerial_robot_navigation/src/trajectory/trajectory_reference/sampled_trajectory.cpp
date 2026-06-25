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
#include "aerial_robot_navigation/trajectory/trajectory_reference/sampled_trajectory.hpp"


namespace agi
{

SampledTrajectory::SampledTrajectory(const SetpointVector &setpoints)
  : ReferenceBase(setpoints.front().state, (setpoints.back().state.t - setpoints.front().state.t), "Sampled Trajectory")
{
  setpoints_ = setpoints;
}

Setpoint SampledTrajectory::getSetpoint(const QuadState &state, const Scalar t)
{
  // Find first point with timestamp larger than query:
  SetpointVector::const_iterator upper_setpoint = std::lower_bound(setpoints_.begin(), setpoints_.end(), t,
                                                                   [](const Setpoint &setpoint, const Scalar t) -> bool
                                                                   { return setpoint.state.t <= t; });

  // If query time is beyond trajectory, return last available state instead
  if (upper_setpoint == setpoints_.end()) return setpoints_.back();

  // If query time is earlier than reference, return first point
  if (upper_setpoint == setpoints_.begin()) return setpoints_.front();

  SetpointVector::const_iterator lower_setpoint = std::prev(upper_setpoint);

  // Interpolate between closest points in SetpointVector
  // setpoint = (1 - a) * lower_setpoint + a * upper_setpoint
  const double x = (t - lower_setpoint->state.t) / (upper_setpoint->state.t - lower_setpoint->state.t);
  return interpolateSetpoints(*lower_setpoint, *upper_setpoint, x);
}

Setpoint SampledTrajectory::getStartSetpoint() { return setpoints_.front(); }

Setpoint SampledTrajectory::getEndSetpoint() { return setpoints_.back(); }

/// setpoint = (1 - a) * setpoint_1 + a * setpoint_2
Setpoint SampledTrajectory::interpolateSetpoints(const Setpoint &setpoint_1, const Setpoint &setpoint_2,
                                                 const Scalar x) const
{
  if (!setpoint_1.state.valid() || !setpoint_2.state.valid() || !setpoint_1.input.valid() || !setpoint_2.input.valid())
  {
    return Setpoint{};
  }

  const Scalar x_comp = 1.0 - x;

  // State
  Setpoint setpoint;
  setpoint.state.t = x_comp * setpoint_1.state.t + x * setpoint_2.state.t;
  setpoint.state.x = x_comp * setpoint_1.state.x + x * setpoint_2.state.x;
  setpoint.state.q(setpoint_1.state.q().slerp(x, setpoint_2.state.q()));

  // Inputs
  setpoint.input.t = x_comp * setpoint_1.input.t + x * setpoint_2.input.t;
  if (setpoint_1.input.isRatesThrust())
  {
    setpoint.input.collective_thrust = x_comp * setpoint_1.input.collective_thrust +
                                       x * setpoint_2.input.collective_thrust;
    setpoint.input.omega = x_comp * setpoint_1.input.omega + x * setpoint_2.input.omega;
    setpoint.input.thrusts.setConstant(NAN);
  }
  else
  {
    setpoint.input.thrusts = x_comp * setpoint_1.input.thrusts + x * setpoint_2.input.thrusts;
    setpoint.input.collective_thrust = NAN;
    setpoint.input.omega.setConstant(NAN);
  }

  return setpoint;
}
}
