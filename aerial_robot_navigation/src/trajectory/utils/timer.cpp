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
#include "aerial_robot_navigation/trajectory/utils/timer.hpp"


namespace agi
{

Timer::Timer(const std::string name, const std::string module)
  : name_(name),
    module_(module),
    timing_mean_(0.0),
    timing_last_(0.0),
    timing_S_(0.0),
    timing_min_(std::numeric_limits<Scalar>::max()),
    timing_max_(0.0),
    n_samples_(0)
{
}

Timer::Timer(const Timer &other)
  : name_(other.name_),
    module_(other.module_),
    t_start_(other.t_start_),
    timing_mean_(other.timing_mean_),
    timing_last_(other.timing_last_),
    timing_S_(other.timing_S_),
    timing_min_(other.timing_min_),
    timing_max_(other.timing_max_),
    n_samples_(other.n_samples_)
{
}

void Timer::tic() { t_start_ = std::chrono::high_resolution_clock::now(); }

Scalar Timer::toc()
{
  // Calculate timing.
  const TimePoint t_end = std::chrono::high_resolution_clock::now();
  timing_last_ = 1e-9 * std::chrono::duration_cast<std::chrono::nanoseconds>(t_end - t_start_).count();
  n_samples_++;

  // Set timing, filter if already initialized.
  if (timing_mean_ <= 0.0)
  {
    timing_mean_ = timing_last_;
  }
  else
  {
    const Scalar timing_mean_prev = timing_mean_;
    timing_mean_ = timing_mean_prev + (timing_last_ - timing_mean_prev) / n_samples_;
    timing_S_ = timing_S_ + (timing_last_ - timing_mean_prev) * (timing_last_ - timing_mean_);
  }
  timing_min_ = (timing_last_ < timing_min_) ? timing_last_ : timing_min_;
  timing_max_ = (timing_last_ > timing_max_) ? timing_last_ : timing_max_;

  t_start_ = t_end;

  return timing_last_;
}

Scalar Timer::operator()() const { return timing_mean_; }

Scalar Timer::mean() const { return timing_mean_; }

Scalar Timer::last() const { return timing_last_; }

Scalar Timer::min() const { return timing_min_; }

Scalar Timer::max() const { return timing_max_; }

Scalar Timer::std() const { return std::sqrt(timing_S_ / n_samples_); }

int Timer::count() const { return n_samples_; }

Scalar Timer::startTime() const
{
  return 1e-9 * std::chrono::duration_cast<std::chrono::nanoseconds>(t_start_.time_since_epoch()).count();
}

void Timer::reset()
{
  n_samples_ = 0u;
  t_start_ = TimePoint();
  timing_mean_ = 0.0;
  timing_last_ = 0.0;
  timing_S_ = 0.0;
  timing_min_ = std::numeric_limits<Scalar>::max();
  timing_max_ = 0.0;
}

void Timer::print() const { std::cout << *this; }

std::ostream &operator<<(std::ostream &os, const Timer &timer)
{
  if (!timer.module_.empty()) os << "[" << timer.module_ << "] ";

  if (timer.n_samples_ < 1)
  {
    os << "Timing " << timer.name_ << " has no call yet." << std::endl;
    return os;
  }

  const std::streamsize prec = os.precision();
  os.precision(3);

  os << "Timing " << timer.name_ << " in " << timer.n_samples_ << " calls" << std::endl;

  if (!timer.module_.empty()) os << "[" << timer.module_ << "] ";
  os << "mean|std:  " << 1000 * timer.timing_mean_ << " | " << 1000 * timer.timing_S_ << " ms    "
     << "[min|max:  " << 1000 * timer.timing_min_ << " | " << 1000 * timer.timing_max_ << " ms]" << std::endl;

  os.precision(prec);
  return os;
}

ScopedTimer::ScopedTimer(const std::string name, const std::string module) : Timer(name, module) { this->tic(); }

ScopedTimer::~ScopedTimer()
{
  this->toc();
  this->print();
}

ScopedTicToc::ScopedTicToc(Timer &timer) : timer_(timer) { timer_.tic(); }

ScopedTicToc::~ScopedTicToc() { timer_.toc(); }

}
