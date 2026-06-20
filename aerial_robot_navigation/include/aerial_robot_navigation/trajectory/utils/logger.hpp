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
#include <cstdarg>
#include <cstdio>
#include <fstream>
#include <string>
#include <unordered_map>

/* Aerial robot packages */
#include "aerial_robot_navigation/trajectory/math/types.hpp"

namespace agi
{

#ifndef DEBUG_LOG
namespace
{
struct NoPrint
{
  template <typename T> constexpr NoPrint operator<<(const T &) const noexcept { return NoPrint(); }
  constexpr NoPrint operator<<(std::ostream &(*)(std::ostream &)) const  //
      noexcept
  {
    return NoPrint();
  }
};
}
#endif

struct PublishLogContainer
{
  Vector<> data;
  bool advertise;
};


class Logger
{
public:
  Logger(const std::string &name, const bool color = true);
  ~Logger();

  inline std::streamsize precision(const std::streamsize n);
  inline void scientific(const bool on = true);

  void info(const char *msg, ...) const;
  void warn(const char *msg, ...) const;
  void error(const char *msg, ...) const;
  void fatal(const char *msg, ...) const;

#ifdef DEBUG_LOG
  void debug(const char *msg, ...) const;
  std::ostream &debug() const;
  void debug(const std::function<void(void)> &&lambda) const;
#else
  inline constexpr void debug(const char *, ...) const noexcept {}
  inline constexpr NoPrint debug() const { return NoPrint(); }
  inline constexpr void debug(const std::function<void(void)> &&) const  //
      noexcept
  {
  }
#endif

  inline void addPublishingVariable(const std::string &var_name, const Vector<> &vec) const
  {
    publishing_variables_[name_ + var_name].advertise = false;  // Not only advertising
    publishing_variables_[name_ + var_name].data = vec;
  }

  inline void addPublishingVariable(const std::string &var_name, Scalar val) const
  {
    addPublishingVariable(var_name, Vector<1>(val));
  }

  inline void advertisePublishingVariable(const std::string &var_name) const
  {
    publishing_variables_[name_ + var_name].advertise = true;  // Only advertising
  }

  inline void erasePublishingVariable(const std::string var_name) { publishing_variables_.erase(name_ + var_name); }

  template <typename T> std::ostream &operator<<(const T &printable) const;

  inline const std::string &name() const { return formatted_name_; }

  static constexpr int MAX_CHARS = 256;

  static void for_each_instance(
      std::function<void(const std::string &name, const PublishLogContainer &container)> function)
  {
    for (auto &pub_var : publishing_variables_)
    {
      function(pub_var.first, pub_var.second);
    }
  }

private:
  static constexpr int DEFAULT_PRECISION = 3;
  static constexpr int NAME_PADDING = 15;
  static constexpr char RESET[] = "\033[0m";
  static constexpr char RED[] = "\033[31m";
  static constexpr char YELLOW[] = "\033[33m";
  static constexpr char INFO[] = "Info:    ";
  static constexpr char WARN[] = "Warning: ";
  static constexpr char ERROR[] = "Error:   ";
  static constexpr char FATAL[] = "Fatal:   ";
  static constexpr char DEBUGPREFIX[] = "Debug:   ";

  const std::string name_;
  std::string formatted_name_;
  const bool colored_;
  static std::unordered_map<std::string, PublishLogContainer> publishing_variables_;  // Map from variable name to a
                                                                                      // struct containing info about
                                                                                      // logging data
};

template <typename T> std::ostream &Logger::operator<<(const T &printable) const
{
  return std::cout << formatted_name_ << printable;
}

}
