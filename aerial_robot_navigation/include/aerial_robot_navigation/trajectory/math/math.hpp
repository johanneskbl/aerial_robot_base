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

Matrix<3, 3> skew(const Vector<3> &v);

Matrix<4, 4> Q_left(const Quaternion &q);

Matrix<4, 4> Q_right(const Quaternion &q);

Matrix<4, 3> qFromQeJacobian(const Quaternion &q);

Matrix<4, 4> qConjugateJacobian();

Matrix<3, 3> qeRotJacobian(const Quaternion &q, const Matrix<3, 1> &t);

Matrix<3, 3> qeInvRotJacobian(const Quaternion &q, const Matrix<3, 1> &t);

void matrixToTripletList(const SparseMatrix &matrix, std::vector<SparseTriplet> *const list, const int row_offset = 0,
                         const int col_offset = 0);

void matrixToTripletList(const Matrix<> &matrix, std::vector<SparseTriplet> *const list, const int row_offset = 0,
                         const int col_offset = 0);

void insert(const SparseMatrix &from, SparseMatrix *const into, const int row_offset = 0, const int col_offset = 0);

void insert(const Matrix<> &from, SparseMatrix *const into, const int row_offset = 0, const int col_offset = 0);

void insert(const Matrix<> &from, Matrix<> *const into, const int row_offset = 0, const int col_offset = 0);

Vector<> clip(const Vector<> &v, const Vector<> &bound);

inline constexpr Scalar toRad(const Scalar angle_deg) { return angle_deg * M_PI / 180.0; }
inline constexpr Scalar toDeg(const Scalar angle_deg) { return angle_deg / M_PI * 180.0; }

}
