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
#include <functional>

/* Aerial robot packages */
#include "aerial_robot_navigation/trajectory/base/parameter_base.hpp"
#include "aerial_robot_navigation/trajectory/math/math.hpp"
#include "aerial_robot_navigation/trajectory/math/types.hpp"
#include "aerial_robot_navigation/trajectory/types/quad_state.hpp"

namespace agi
{

struct Quadrotor : public ParameterBase
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  enum class RotorConfig
  {
    plus,
    cross
  };

  Quadrotor(const Scalar m, const Scalar l);
  Quadrotor();

  bool dynamics(const QuadState &state, QuadState *const derivative) const;

  bool dynamics(const Ref<const Vector<QuadState::SIZE>> state, Ref<Vector<QuadState::SIZE>> derivative) const;

  bool jacobian(const Ref<const Vector<QuadState::SIZE>> state,
                Ref<Matrix<QuadState::SIZE, QuadState::SIZE>> jac) const;

  bool jacobian(const Ref<const Vector<QuadState::SIZE>> state, SparseMatrix &jac) const;

  DynamicsFunction getDynamicsFunction() const;

  using ParameterBase::load;
  bool load(const Yaml &node) override;
  bool valid() const override;

  // Helpers to apply limits.
  Vector<4> clampThrust(const Vector<4> thrusts) const;
  Scalar clampThrust(const Scalar thrust) const;
  Scalar clampCollectiveThrust(const Scalar thrust) const;
  Vector<4> clampMotorOmega(const Vector<4> &omega) const;
  Vector<3> clampBodyrates(const Vector<3> &omega) const;

  inline Scalar collective_thrust_min() const { return 4.0 * thrust_min_ / m_; }
  inline Scalar collective_thrust_max() const { return 4.0 * thrust_max_ / m_; }

  // Helpers for conversion
  Vector<4> motorOmegaToThrust(const Vector<4> &omega) const;
  Vector<4> motorOmegaToTorque(const Vector<4> &omega) const;
  Vector<4> motorThrustToOmega(const Vector<4> &thrusts) const;

  // Getter Functions for member variables
  Matrix<4, 4> getAllocationMatrix() const;

  friend std::ostream &operator<<(std::ostream &os, const Quadrotor &quad);

  // Quadrotor physics
  Scalar m_;
  Matrix<3, 4> t_BM_;
  Matrix<3, 3> J_;
  Matrix<3, 3> J_inv_;

  // Motor
  Scalar motor_omega_min_;
  Scalar motor_omega_max_;
  Scalar motor_tau_inv_;

  // Propellers
  Vector<3> thrust_map_;
  Vector<3> torque_map_;
  Scalar kappa_;
  Scalar thrust_min_;
  Scalar thrust_max_;
  RotorConfig rotors_config_;

  // Quadrotor limits
  Vector<3> omega_max_;

  // Simple cubic aerodynamic model f= c_1 * v +_ c_3 * v^3.
  Vector<3> aero_coeff_1_;
  Vector<3> aero_coeff_3_;

  // Forward-flight induced thrust variation fz -= c_h * (vx^2 + vy^2)
  Scalar aero_coeff_h_;
};

}
