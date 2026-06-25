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
#include "aerial_robot_navigation/trajectory/math/math.hpp"

namespace agi
{

Matrix<3, 3> skew(const Vector<3> &v)
{
  return (Matrix<3, 3>() << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(), v.x(), 0).finished();
}

Matrix<4, 4> Q_left(const Quaternion &q)
{
  return (Matrix<4, 4>() << q.w(), -q.x(), -q.y(), -q.z(), q.x(), q.w(), -q.z(), q.y(), q.y(), q.z(), q.w(), -q.x(),
          q.z(), -q.y(), q.x(), q.w())
      .finished();
}

Matrix<4, 4> Q_right(const Quaternion &q)
{
  return (Matrix<4, 4>() << q.w(), -q.x(), -q.y(), -q.z(), q.x(), q.w(), q.z(), -q.y(), q.y(), -q.z(), q.w(), q.x(),
          q.z(), q.y(), -q.x(), q.w())
      .finished();
}

Matrix<4, 3> qFromQeJacobian(const Quaternion &q)
{
  return (Matrix<4, 3>() << -1.0 / q.w() * q.vec().transpose(), Matrix<3, 3>::Identity()).finished();
}

Matrix<4, 4> qConjugateJacobian() { return Matrix<4, 1>(1, -1, -1, -1).asDiagonal(); }

Matrix<3, 3> qeRotJacobian(const Quaternion &q, const Matrix<3, 1> &t)
{
  return 2.0 * (Matrix<3, 3>()
                    << (q.y() + q.z() * q.x() / q.w()) * t.y() + (q.z() - q.y() * q.x() / q.w()) * t.z(),  // Entry 0,0
                -2.0 * q.y() * t.x() + (q.x() + q.z() * q.y() / q.w()) * t.y() +
                    (q.w() - q.y() * q.y() / q.w()) * t.z(),  // Entry 0,1
                -2.0 * q.z() * t.x() + (-q.w() + q.z() * q.z() / q.w()) * t.y() +
                    (q.x() - q.y() * q.z() / q.w()) * t.z(),  // Entry 0,2

                (q.y() - q.z() * q.x() / q.w()) * t.x() + (-2.0 * q.x()) * t.y() +
                    (-q.w() + q.x() * q.x() / q.w()) * t.z(),                                       // Entry 1,0
                (q.x() - q.z() * q.y() / q.w()) * t.x() + (q.z() + q.x() * q.y() / q.w()) * t.z(),  // Entry 1,1
                (q.w() - q.z() * q.z() / q.w()) * t.x() + (-2.0 * q.z()) * t.y() +
                    (q.y() + q.x() * q.z() / q.w()) * t.z(),  // Entry 1,2

                (q.z() + q.y() * q.x() / q.w()) * t.x() + (q.w() - q.x() * q.x() / q.w()) * t.y() +
                    (-2.0 * q.x()) * t.z(),  // Entry 2,0
                (-q.w() + q.y() * q.y() / q.w()) * t.x() + (q.z() - q.x() * q.y() / q.w()) * t.y() +
                    (-2.0 * q.y()) * t.z(),                                                        // Entry 2,1
                (q.x() + q.y() * q.z() / q.w()) * t.x() + (q.y() - q.x() * q.z() / q.w()) * t.y()  // Entry 2,2
                )
                   .finished();
}

Matrix<3, 3> qeInvRotJacobian(const Quaternion &q, const Matrix<3, 1> &t)
{
  return 2.0 * (Matrix<3, 3>()
                    << (q.y() - q.z() * q.x() / q.w()) * t.y() + (q.z() + q.y() * q.x() / q.w()) * t.z(),  // Entry 0,0
                -2.0 * q.y() * t.x() + (q.x() - q.z() * q.y() / q.w()) * t.y() -
                    (q.w() - q.y() * q.y() / q.w()) * t.z(),  // Entry 0,1
                -2.0 * q.z() * t.x() + (q.w() - q.z() * q.z() / q.w()) * t.y() +
                    (q.x() + q.y() * q.z() / q.w()) * t.z(),  // Entry 0,2

                (q.y() + q.z() * q.x() / q.w()) * t.x() - 2.0 * q.x() * t.y() +
                    (q.w() - q.x() * q.x() / q.w()) * t.z(),                                        // Entry 1,0
                (q.x() + q.z() * q.y() / q.w()) * t.x() + (q.z() - q.x() * q.y() / q.w()) * t.z(),  // Entry 1,1
                -(q.w() - q.z() * q.z() / q.w()) * t.x() - 2.0 * q.z() * t.y() +
                    (q.y() - q.x() * q.z() / q.w()) * t.z(),  // Entry 1,2

                (q.z() - q.y() * q.x() / q.w()) * t.x() - (q.w() - q.x() * q.x() / q.w()) * t.y() -
                    2.0 * q.x() * t.z(),  // Entry 2,0
                (q.w() - q.y() * q.y() / q.w()) * t.x() + (q.z() + q.x() * q.y() / q.w()) * t.y() -
                    2.0 * q.y() * t.z(),                                                           // Entry 2,1
                (q.x() - q.y() * q.z() / q.w()) * t.x() + (q.y() + q.x() * q.z() / q.w()) * t.y()  // Entry 2,2
                )
                   .finished();
}

void matrixToTripletList(const SparseMatrix &matrix, std::vector<SparseTriplet> *const list, const int row_offset,
                         const int col_offset)
{
  list->reserve((size_t)matrix.nonZeros() + list->size());

  for (int i = 0; i < matrix.outerSize(); i++)
  {
    for (typename SparseMatrix::InnerIterator it(matrix, i); it; ++it)
    {
      list->emplace_back(it.row() + row_offset, it.col() + col_offset, it.value());
    }
  }
}

void matrixToTripletList(const Matrix<Dynamic, Dynamic> &matrix, std::vector<SparseTriplet> *const list,
                         const int row_offset, const int col_offset)
{
  const SparseMatrix sparse = matrix.sparseView();
  matrixToTripletList(sparse, list, row_offset, col_offset);
}

void insert(const SparseMatrix &from, SparseMatrix *const into, const int row_offset, const int col_offset)
{
  if (into == nullptr) return;  // TODO: Handle exception
  std::vector<SparseTriplet> v;
  matrixToTripletList(*into, &v);
  matrixToTripletList(from, &v, row_offset, col_offset);

  into->setFromTriplets(v.begin(), v.end(), [](const Scalar &older, const Scalar &newer) { return newer; });
}

void insert(const Matrix<> &from, SparseMatrix *const into, const int row_offset, const int col_offset)
{
  if (into == nullptr) return;  // TODO: Handle exception
  const SparseMatrix from_sparse = from.sparseView();
  insert(from_sparse, into, row_offset, col_offset);
}

inline void insert(const Matrix<> &from, Matrix<> *const into, const int row_offset, const int col_offset)
{
  into->block(row_offset, col_offset, from.rows(), from.cols()) = from;
}

Vector<> clip(const Vector<> &v, const Vector<> &bound) { return v.cwiseMin(bound).cwiseMax(-bound); }
}
