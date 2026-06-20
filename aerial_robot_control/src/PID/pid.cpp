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
#include "aerial_robot_control/PID/pid.hpp"

namespace aerial_robot_control
{

PID::PID(const std::string name, const double p_gain, const double i_gain, const double d_gain, const double limit_sum,
         const double limit_p, const double limit_i, const double limit_d, const double limit_err_p,
         const double limit_err_i, const double limit_err_d)
  : name_(name), result_(0), err_p_(0), err_i_(0), err_i_prev_(0), err_d_(0), p_term_(0), i_term_(0), d_term_(0)
{
  setGains(p_gain, i_gain, d_gain);
  setLimits(limit_sum, limit_p, limit_i, limit_d, limit_err_p, limit_err_i, limit_err_d);
}

void PID::setGains(const double p_gain, const double i_gain, const double d_gain)
{
  setPGain(p_gain);
  setIGain(i_gain);
  setDGain(d_gain);
}

void PID::setLimits(const double limit_sum, const double limit_p, const double limit_i, const double limit_d,
                    const double limit_err_p, const double limit_err_i, const double limit_err_d)
{
  setLimitSum(limit_sum);
  setLimitP(limit_p);
  setLimitI(limit_i);
  setLimitD(limit_d);
  setLimitErrP(limit_err_p);
  setLimitErrI(limit_err_i);
  setLimitErrD(limit_err_d);
}

void PID::update(const double err_p, const double err_d, const double feedforward_term, const double dt)
{
  err_p_ = std::clamp(err_p, -limit_err_p_, limit_err_p_);
  err_i_prev_ = err_i_;
  err_i_ = std::clamp(err_i_ + err_p_ * dt, -limit_err_i_, limit_err_i_);
  err_d_ = std::clamp(err_d, -limit_err_d_, limit_err_d_);

  p_term_ = std::clamp(err_p_ * p_gain_, -limit_p_, limit_p_);
  i_term_ = std::clamp(err_i_ * i_gain_, -limit_i_, limit_i_);
  d_term_ = std::clamp(err_d_ * d_gain_, -limit_d_, limit_d_);

  result_ = std::clamp(p_term_ + i_term_ + d_term_ + feedforward_term, -limit_sum_, limit_sum_);
}

void PID::reset()
{
  err_i_ = 0;
  err_i_prev_ = 0;
  result_ = 0;
}

}
