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
#include "aerial_robot_navigation/trajectory/utils/logger.hpp"


namespace agi
{

std::unordered_map<std::string, PublishLogContainer> Logger::publishing_variables_;

Logger::Logger(const std::string &name, const bool color) : name_("Logger/" + name + "/"), colored_(color)
{
  std::cout.precision(DEFAULT_PRECISION);

  // Format name
  formatted_name_ = "[" + name + "]";
  if (formatted_name_.size() < NAME_PADDING)
    formatted_name_ = formatted_name_ + std::string(NAME_PADDING - formatted_name_.size(), ' ');
  else
    formatted_name_ = formatted_name_ + " ";
}

Logger::~Logger() {}


inline std::streamsize Logger::precision(const std::streamsize n) { return std::cout.precision(n); }

inline void Logger::scientific(const bool on)
{
  if (on)
    std::cout << std::scientific;
  else
    std::cout << std::fixed;
}

void Logger::info(const char *msg, ...) const
{
  std::va_list args;
  va_start(args, msg);
  char buf[MAX_CHARS];
  const int n = std::vsnprintf(buf, MAX_CHARS, msg, args);
  va_end(args);
  if (n < 0 || n >= MAX_CHARS) std::cout << formatted_name_ << "=== Logging error ===" << std::endl;
  if (colored_)
    std::cout << formatted_name_ << buf << std::endl;
  else
    std::cout << formatted_name_ << INFO << buf << std::endl;
}

void Logger::warn(const char *msg, ...) const
{
  std::va_list args;
  va_start(args, msg);
  char buf[MAX_CHARS];
  const int n = std::vsnprintf(buf, MAX_CHARS, msg, args);
  va_end(args);
  if (n < 0 || n >= MAX_CHARS) std::cout << formatted_name_ << "=== Logging error ===" << std::endl;
  if (colored_)
    std::cout << YELLOW << formatted_name_ << buf << RESET << std::endl;
  else
    std::cout << formatted_name_ << WARN << buf << std::endl;
}

void Logger::error(const char *msg, ...) const
{
  std::va_list args;
  va_start(args, msg);
  char buf[MAX_CHARS];
  const int n = std::vsnprintf(buf, MAX_CHARS, msg, args);
  va_end(args);
  if (n < 0 || n >= MAX_CHARS) std::cout << formatted_name_ << "=== Logging error ===" << std::endl;
  if (colored_)
    std::cout << RED << formatted_name_ << buf << RESET << std::endl;
  else
    std::cout << formatted_name_ << ERROR << buf << std::endl;
}

void Logger::fatal(const char *msg, ...) const
{
  std::va_list args;
  va_start(args, msg);
  char buf[MAX_CHARS];
  const int n = std::vsnprintf(buf, MAX_CHARS, msg, args);
  va_end(args);
  if (n < 0 || n >= MAX_CHARS) std::cout << formatted_name_ << "=== Logging error ===" << std::endl;
  if (colored_)
    std::cout << RED << formatted_name_ << buf << RESET << std::endl;
  else
    std::cout << formatted_name_ << FATAL << buf << std::endl;
  throw std::runtime_error(formatted_name_ + buf);
}

#ifdef DEBUG_LOG
void Logger::debug(const char *msg, ...) const
{
  std::va_list args;
  va_start(args, msg);
  char buf[MAX_CHARS];
  const int n = std::vsnprintf(buf, MAX_CHARS, msg, args);
  va_end(args);
  if (n < 0 || n >= MAX_CHARS) std::cout << formatted_name_ << "=== Logging error ===" << std::endl;
  if (colored_)
    std::cout << formatted_name_ << buf << std::endl;
  else
    std::cout << formatted_name_ << DEBUGPREFIX << buf << std::endl;
}

std::ostream &Logger::debug() const { return std::cout << formatted_name_; }

void Logger::debug(const std::function<void(void)> &&lambda) const { lambda(); }

#endif

}
