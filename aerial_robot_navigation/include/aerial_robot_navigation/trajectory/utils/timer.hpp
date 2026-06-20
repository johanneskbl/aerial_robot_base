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
#include <chrono>

/* Aerial robot packages */
#include "aerial_robot_navigation/trajectory/math/types.hpp"

namespace agi
{

/*
 * Timer class to perform runtime analytics.
 *
 * This timer class provides a simple solution to time code.
 * Simply construct a timer and call it's `tic()` and `toc()` functions to time
 * code. It is intended to be used to time multiple calls of a function and not
 * only reports the `last()` timing, but also statistics such as the `mean()`,
 * `min()`, `max()` time, the `count()` of calls to the timer , and even
 * standard deviation `std()`.
 *
 * The constructor can take a name for the timer (like "update") and a name for
 * the module (like "Filter").
 * After construction it can be `reset()` if needed.
 *
 * A simple way to get the timing and stats is `std::cout << timer;` which can
 * output to arbitrary streams, overloading the stream operator,
 * or `print()` which always prints to console.
 *
 */

using TimeFunction = std::function<Scalar()>;
static constexpr auto ChronoTime = []() -> agi::Scalar
{
  const std::chrono::high_resolution_clock::time_point now = std::chrono::high_resolution_clock::now();
  return 1e-9 * std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
};

class Timer
{
public:
  Timer(const std::string name = "", const std::string module = "");
  Timer(const Timer &other);
  ~Timer() {}

  /// Start the timer.
  void tic();

  /// Stops timer, calculates timing, also tics again.
  Scalar toc();

  /// Reset saved timings and calls;
  void reset();

  // Accessors
  Scalar operator()() const;
  Scalar mean() const;
  Scalar last() const;
  Scalar min() const;
  Scalar max() const;
  Scalar std() const;
  int count() const;
  Scalar startTime() const;

  /// Custom stream operator for outputs.
  friend std::ostream &operator<<(std::ostream &os, const Timer &timer);

  /// Print timing information to console.
  void print() const;

private:
  std::string name_, module_;
  using TimePoint = std::chrono::high_resolution_clock::time_point;
  TimePoint t_start_;

  // Initialize timing to impossible values.
  Scalar timing_mean_;
  Scalar timing_last_;
  Scalar timing_S_;
  Scalar timing_min_;
  Scalar timing_max_;

  int n_samples_;
};

/*
 * Simple class to time scopes.
 *
 * This effectively instantiates a timer and calls `tic()` in its constructor
 * and `toc()` and ` print()` in its destructor.
 */
class ScopedTimer : public Timer
{
public:
  ScopedTimer(const std::string name = "", const std::string module = "");
  ~ScopedTimer();
};

/*
 *  * Helper Timer class to instantiate a static Timer that prints in
 * descructor.
 *   *
 *    * Debugging slow code? Simply create this as a static object somewhere and
 *     * tic-toc it. Once the program ends, the destructor of StaticTimer will
 * print
 *      * its stats.
 *       */
class StaticTimer : public Timer
{
public:
  using Timer::Timer;

  ~StaticTimer() { this->print(); }
};

/*
 * Simple class to tic and toc a timer within a scope.
 *
 * This takes a timer as argument, tics in the constructor, and tocs on
 * destruction.
 */
class ScopedTicToc
{
public:
  ScopedTicToc(Timer &timer);
  ~ScopedTicToc();

private:
  Timer &timer_;
};

}
