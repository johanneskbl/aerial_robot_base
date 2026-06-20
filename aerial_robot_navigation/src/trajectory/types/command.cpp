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
#include "aerial_robot_navigation/trajectory/types/command.hpp"


namespace agi
{

Command::Command() {}

Command::Command(const Scalar t, const Scalar thrust, const Vector<3> &omega)
  : t(t), collective_thrust(thrust), omega(omega)
{
}

Command::Command(const Scalar t, const Vector<4> &thrusts) : t(t), thrusts(thrusts) {}

Command::Command(const Scalar t) : t(t), collective_thrust(0.0), omega(Vector<3>::Zero()), thrusts(Vector<4>::Zero()) {}

bool Command::valid() const
{
  return std::isfinite(t) && ((std::isfinite(collective_thrust) && omega.allFinite()) || thrusts.allFinite());
}

bool Command::isSingleRotorThrusts() const { return std::isfinite(t) && thrusts.allFinite(); }

bool Command::isRatesThrust() const
{
  return std::isfinite(t) && std::isfinite(collective_thrust) && omega.allFinite();
}

std::ostream &operator<<(std::ostream &os, const Command &command)
{
  os.precision(3);
  os << std::scientific;
  os << "Command at " << command.t << "s:\n"
     << "collective_thrust =  [" << command.collective_thrust << "]\n"
     << "omega =  [" << command.omega.transpose() << "]\n"
     << "thrusts =  [" << command.thrusts.transpose() << "]" << std::endl;
  os.precision();
  os.unsetf(std::ios::scientific);
  return os;
}

bool Command::operator==(const Command &rhs) const
{
  if (t != rhs.t)
  {
    return false;
  }
  if (isRatesThrust())
  {
    return (collective_thrust == rhs.collective_thrust && omega.isApprox(rhs.omega, 1e-3));
  }
  else
  {
    return (thrusts.isApprox(rhs.thrusts, 1e-3));
  }
}
}
