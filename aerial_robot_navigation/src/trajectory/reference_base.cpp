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
#include "aerial_robot_navigation/trajectory/reference_base.hpp"


namespace agi
{

ReferenceBase::ReferenceBase(const std::string &name) : name_(name) {}

ReferenceBase::ReferenceBase(const QuadState &state, const Scalar duration, const std::string &name)
  : start_state_(state), duration_(duration), name_(name)
{
}

ReferenceBase::~ReferenceBase() {}

Setpoint ReferenceBase::getSetpoint(const Scalar t, const Scalar heading)
{
  QuadState query_state;
  query_state.t = t;
  if (std::isfinite(heading)) query_state.q(heading);
  return getSetpoint(query_state, t);
}

Setpoint ReferenceBase::getSetpoint(const QuadState &state) { return getSetpoint(state, state.t); }

Setpoint ReferenceBase::getStartSetpoint()
{
  Setpoint setpoint;
  setpoint.state = start_state_;
  setpoint.input = Command(start_state_.t, G, Vector<3>::Zero());
  return setpoint;
}

Setpoint ReferenceBase::getEndSetpoint()
{
  Setpoint setpoint;
  setpoint.state = start_state_;
  setpoint.state.t = start_state_.t + duration_;
  setpoint.input = Command(start_state_.t + duration_, G, Vector<3>::Zero());
  return setpoint;
}

bool ReferenceBase::truncate(const Scalar &t)
{
  if (t <= start_state_.t) return false;
  duration_ = t - start_state_.t;
  return true;
}

bool ReferenceBase::isTimeInRange(const Scalar time) const { return time >= getStartTime() && time <= getEndTime(); }

std::string ReferenceBase::time_in_HH_MM_SS_MMM(const Scalar time) const
{
  if (!std::isfinite(time)) return "inf";

  const unsigned int ms = fmod(time, 1000.0);

  // Convert to std::time_t in order to convert to std::tm (broken time)
  std::time_t timer = (std::time_t)(time);

  // Convert to broken time
  std::tm bt = *std::localtime(&timer);

  std::ostringstream oss;

  oss << std::put_time(&bt, "%H:%M:%S");  // HH:MM:SS
  oss << '.' << std::setfill('0') << std::setw(3) << ms;

  return oss.str();
}

std::ostream &operator<<(std::ostream &os, const ReferenceBase &ref)
{
  os << std::setw(30) << std::left << ref.name() << "  | " << ref.time_in_HH_MM_SS_MMM(ref.getStartTime()) << " --> "
     << ref.getDuration() << "s --> " << ref.time_in_HH_MM_SS_MMM(ref.getEndTime()) << " |" << std::endl;
  return os;
}
}
