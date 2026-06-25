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
#include <memory>
#include <iostream>
#include <cmath>
#include <Eigen/Eigen>
#include <limits>

namespace agi
{

using Scalar = double;
static constexpr Scalar INF = std::numeric_limits<Scalar>::infinity();

static constexpr int Dynamic = Eigen::Dynamic;

// Using shorthand for `Matrix<rows, cols>` with scalar type.
template <int rows = Dynamic, int cols = rows> using Matrix = Eigen::Matrix<Scalar, rows, cols>;

// Using shorthand for `Vector<rows>` with scalar type.
template <int rows = Dynamic> using Vector = Matrix<rows, 1>;

// Using shorthand for `Array<rows, cols>` with scalar type.
template <int rows = Dynamic, int cols = rows> using Array = Eigen::Array<Scalar, rows, cols>;

template <int rows = Dynamic> using ArrayVector = Array<rows, 1>;

using SparseMatrix = Eigen::SparseMatrix<Scalar>;

using SparseTriplet = Eigen::Triplet<Scalar>;

using Quaternion = Eigen::Quaternion<Scalar>;

template <class Derived> using Ref = Eigen::Ref<Derived>;

template <class Derived> using ConstRef = const Eigen::Ref<const Derived>;

template <class Derived> using Map = Eigen::Map<Derived>;

using DynamicsFunction = std::function<bool(const Ref<const Vector<>>, Ref<Vector<>>)>;
}
