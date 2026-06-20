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

/* Aerial robot packages */
#include "aerial_robot_navigation/trajectory/math/types.hpp"

namespace agi
{

template <typename Solver = Eigen::HouseholderQR<Matrix<>>> class Polynomial
{
public:
  Polynomial();
  Polynomial(const int order, const Vector<> &weights = Vector<4>(0, 0, 0, 1), const int continuity = -1);
  Polynomial(const Polynomial &rhs) = default;
  ~Polynomial() = default;

  bool scale(const Scalar start_time = NAN, const Scalar duration = NAN);
  int addConstraint(const Scalar time, const Vector<> &constraint);
  bool solve();
  bool eval(const Scalar time, Ref<Vector<>> state) const;
  Scalar operator()(const Scalar time, const int order = 0) const;

  inline int order() const noexcept { return order_; }
  inline int size() const noexcept { return order_ + 1; }
  bool solved() const { return c_.allFinite(); }

  Matrix<> H() const { return H_; }
  Matrix<> A() const { return A_; }
  Vector<> b() const { return b_; }
  Matrix<> exponents() const { return exponents_.transpose(); }
  Matrix<> alpha() const { return alpha_.transpose(); }
  Vector<> coeffs() const { return c_; }

  Scalar tauFromTime(const Scalar t) const;
  Vector<> tauVector(const Scalar tau, const int order = 0) const;
  Vector<> tauVectorFromTime(const Scalar time, const int order = 0) const;
  Matrix<> polyMatrix(const Scalar tau, const int order) const;
  Matrix<> polyMatrixFromTime(const Scalar time, const int order) const;

  void reset();

private:
  Matrix<> createH(const Vector<> &weights) const;
  bool solve(const Matrix<> &H, const Matrix<> &A, const Vector<> &b);

  const int order_;
  const int continuity_;
  Vector<> c_;
  const Array<> exponents_;
  const Array<> alpha_;

  Vector<> weights_;  // Weights on derivatives starting from velocity,
  Matrix<> H_ = Matrix<>::Constant(1, order_, NAN);
  Matrix<> A_ = Matrix<>::Constant(1, order_, NAN);
  Vector<> b_ = Vector<>::Constant(1, NAN);

  Scalar t_scale_{ 1.0 };
  Scalar t_offset_{ 0.0 };
};

// Closed-Form Minimum Jerk Specialization
template <> Polynomial<void>::Polynomial(const int order, const Vector<> &weights, const int continuity) = delete;

template <> bool Polynomial<void>::solve(const Matrix<> &H, const Matrix<> &A, const Vector<> &b) = delete;

class ClosedFormMinJerkAxis : public Polynomial<void>
{
public:
  using Polynomial::Polynomial;
};

}
