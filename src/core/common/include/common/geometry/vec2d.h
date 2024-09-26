/**
 * *********************************************************
 *
 * @file: vector2d.h
 * @brief: geometry: 2D vector
 * @author: Yang Haodong
 * @date: 2024-9-6
 * @version: 1.0
 *
 * Copyright (c) 2024, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */

/**
 * @file
 * @brief Defines the Vec2d class.
 */

#pragma once

#include <cmath>
#include <string>

namespace rmp
{
namespace common
{
namespace geometry
{
constexpr double kMathEpsilon = 1e-10;

/**
 * @class Vec2d
 *
 * @brief Implements a class of 2-dimensional vectors.
 */
class Vec2d
{
public:
  //! Constructor which takes x- and y-coordinates.
  constexpr Vec2d(const double x, const double y) noexcept : x_(x), y_(y)
  {
  }

  //! Constructor returning the zero vector.
  constexpr Vec2d() noexcept : Vec2d(0, 0)
  {
  }

  //! Creates a unit-vector with a given angle to the positive x semi-axis
  static Vec2d createUnitVec2d(const double angle);

  //! Getter for x component
  double x() const
  {
    return x_;
  }

  //! Getter for y component
  double y() const
  {
    return y_;
  }

  //! Setter for x component
  void setX(const double x)
  {
    x_ = x;
  }

  //! Setter for y component
  void setY(const double y)
  {
    y_ = y;
  }

  //! Gets the length of the vector
  double length() const;

  //! Gets the squared length of the vector
  double lengthSquare() const;

  //! Gets the angle between the vector and the positive x semi-axis
  double angle() const;

  //! Returns the unit vector that is co-linear with this vector
  void normalize();

  //! Returns the distance to the given vector
  double distanceTo(const Vec2d& other) const;

  //! Returns the squared distance to the given vector
  double distanceSquareTo(const Vec2d& other) const;

  //! Returns the "cross" product between these two Vec2d (non-standard).
  double crossProd(const Vec2d& other) const;

  //! Returns the inner product between these two Vec2d.
  double innerProd(const Vec2d& other) const;

  //! rotate the vector by angle.
  Vec2d rotate(const double angle) const;

  //! rotate the vector itself by angle.
  void selfRotate(const double angle);

  //! Sums two Vec2d
  Vec2d operator+(const Vec2d& other) const;

  //! Subtracts two Vec2d
  Vec2d operator-(const Vec2d& other) const;

  //! Multiplies Vec2d by a scalar
  Vec2d operator*(const double ratio) const;

  //! Divides Vec2d by a scalar
  Vec2d operator/(const double ratio) const;

  //! Sums another Vec2d to the current one
  Vec2d& operator+=(const Vec2d& other);

  //! Subtracts another Vec2d to the current one
  Vec2d& operator-=(const Vec2d& other);

  //! Multiplies this Vec2d by a scalar
  Vec2d& operator*=(const double ratio);

  //! Divides this Vec2d by a scalar
  Vec2d& operator/=(const double ratio);

  //! Compares two Vec2d
  bool operator==(const Vec2d& other) const;

protected:
  double x_ = 0.0;
  double y_ = 0.0;
};

//! Multiplies the given Vec2d by a given scalar
Vec2d operator*(const double ratio, const Vec2d& vec);

}  // namespace structure
}  // namespace common
}  // namespace apollo
