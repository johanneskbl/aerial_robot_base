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
#include <string>
#include <vector>
#include <iomanip>

/* Aerial robot packages */
#include "aerial_robot_navigation/trajectory/base/module.hpp"
#include "aerial_robot_navigation/trajectory/math/types.hpp"
#include "aerial_robot_navigation/trajectory/types/setpoint.hpp"
#include "aerial_robot_navigation/trajectory/utils/logger.hpp"
#include "aerial_robot_navigation/trajectory/math/gravity.hpp"

namespace agi
{

class ReferenceBase
{
public:
  ReferenceBase(const std::string &name = "ReferenceBase");
  ReferenceBase(const QuadState &state, const Scalar duration, const std::string &name = "ReferenceBase");
  virtual ~ReferenceBase();

  virtual Setpoint getSetpoint(const QuadState &state, const Scalar t) = 0;
  virtual Setpoint getSetpoint(const QuadState &state) final;
  virtual Setpoint getSetpoint(const Scalar t, const Scalar heading = NAN) final;
  virtual Setpoint getStartSetpoint();
  virtual Setpoint getEndSetpoint();

  inline Scalar getStartTime() const { return start_state_.t; }
  inline Scalar getEndTime() const { return start_state_.t + duration_; }
  inline Scalar getDuration() const { return duration_; }
  bool isTimeInRange(const Scalar time) const;
  std::string time_in_HH_MM_SS_MMM(const Scalar time) const;

  virtual bool valid() const { return start_state_.valid(); }
  const std::string &name() const { return name_; }
  virtual bool isHover() const { return false; }
  virtual bool isVelocityRefernce() const { return false; }
  virtual bool isAbsolute() const { return true; }

  bool truncate(const Scalar &t);
  friend std::ostream &operator<<(std::ostream &os, const ReferenceBase &ref);

protected:
  QuadState start_state_;
  Scalar duration_;
  const std::string name_;
};

using ReferenceVector = std::vector<std::shared_ptr<ReferenceBase>>;

}
