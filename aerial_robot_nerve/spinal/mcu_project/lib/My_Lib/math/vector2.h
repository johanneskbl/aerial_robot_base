// -*- mode: c++ -*-
/*
 This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

// Copyright 2010 Michael Smith, all rights reserved.

// Derived closely from:
/****************************************
 * 2D Vector Classes
 * By Bill Perone (billperone@yahoo.com)
 * Original: 9-16-2002
 * Revised: 19-11-2003
 *          18-12-2003
 *          06-06-2004
 *
 *  2003, This code is provided "as is" and you can use it freely as long as
 * credit is given to Bill Perone in the application it is used in
 ****************************************/

#ifndef VECTOR2_H
#define VECTOR2_H

#include <float.h>
#include <math.h>

namespace ap
{
template <typename T> struct Vector2
{
  T x, y;

  // Trivial ctor
  Vector2<T>() { x = y = 0; }

  // Setting ctor
  Vector2<T>(const T x0, const T y0) : x(x0), y(y0) {}

  // Function call operator
  void operator()(const T x0, const T y0)
  {
    x = x0;
    y = y0;
  }

  // Test for equality
  bool operator==(const Vector2<T> &v) const;

  // Test for inequality
  bool operator!=(const Vector2<T> &v) const;

  // Negation
  Vector2<T> operator-(void) const;

  // Addition
  Vector2<T> operator+(const Vector2<T> &v) const;

  // Subtraction
  Vector2<T> operator-(const Vector2<T> &v) const;

  // Uniform scaling
  Vector2<T> operator*(const T num) const;

  // Uniform scaling
  Vector2<T> operator/(const T num) const;

  // Addition
  Vector2<T> &operator+=(const Vector2<T> &v);

  // Subtraction
  Vector2<T> &operator-=(const Vector2<T> &v);

  // Uniform scaling
  Vector2<T> &operator*=(const T num);

  // Uniform scaling
  Vector2<T> &operator/=(const T num);

  // Dot product
  T operator*(const Vector2<T> &v) const;

  // Cross product
  T operator%(const Vector2<T> &v) const;

  // Computes the angle between this vector and another vector
  float angle(const Vector2<T> &v2) const;

  // Computes the angle in radians between the origin and this vector
  T angle(void) const;

  // Check if any elements are NAN
  bool is_nan(void) const;

  // Check if any elements are infinity
  bool is_inf(void) const;

  // Check if all elements are zero
  bool is_zero(void) const { return (fabsf(x) < FLT_EPSILON) && (fabsf(y) < FLT_EPSILON); }

  // Zero the vector
  void zero() { x = y = 0; }

  // Gets the length of this vector squared
  T length_squared() const { return (T)(*this * *this); }

  // Gets the length of this vector
  float length(void) const;

  // Normalizes this vector
  void normalize() { *this /= length(); }

  // Returns the normalized vector
  Vector2<T> normalized() const { return *this / length(); }

  // Reflects this vector about n
  void reflect(const Vector2<T> &n)
  {
    Vector2<T> orig(*this);
    project(n);
    *this = *this * 2 - orig;
  }

  // Projects this vector onto v
  void project(const Vector2<T> &v) { *this = v * (*this * v) / (v * v); }

  // Returns this vector projected onto v
  Vector2<T> projected(const Vector2<T> &v) { return v * (*this * v) / (v * v); }
};

typedef Vector2<int16_t> Vector2i;
typedef Vector2<uint16_t> Vector2ui;
typedef Vector2<int32_t> Vector2l;
typedef Vector2<uint32_t> Vector2ul;
typedef Vector2<float> Vector2f;
}

#endif  // VECTOR2_H