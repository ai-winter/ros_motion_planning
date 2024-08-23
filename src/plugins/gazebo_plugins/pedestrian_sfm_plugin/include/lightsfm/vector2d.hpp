/***********************************************************************/
/**                                                                    */
/** vector2d.hpp                                                       */
/**                                                                    */
/** Copyright (c) 2016, Service Robotics Lab.                          */
/**                     http://robotics.upo.es                         */
/**                                                                    */
/** All rights reserved.                                               */
/**                                                                    */
/** Authors:                                                           */
/** Ignacio Perez-Hurtado (maintainer)                                 */
/** Jesus Capitan                                                      */
/** Fernando Caballero                                                 */
/** Luis Merino                                                        */
/**                                                                    */
/** This software may be modified and distributed under the terms      */
/** of the BSD license. See the LICENSE file for details.              */
/**                                                                    */
/** http://www.opensource.org/licenses/BSD-3-Clause                    */
/**                                                                    */
/***********************************************************************/

#ifndef _VECTOR2D_HPP_
#define _VECTOR2D_HPP_

#include <iostream>
#include <cmath>
// #include <geometry_msgs/Point.h>
#include <boost/functional/hash.hpp>

#include "angle.hpp"

namespace utils
{
class Vector2d
{
public:
  Vector2d() : x(0), y(0)
  {
  }
  Vector2d(double x, double y) : x(x), y(y)
  {
  }
  virtual ~Vector2d()
  {
  }
  double operator()(int index) const
  {
    return index == 0 ? x : y;
  }
  double operator[](int index) const
  {
    return index == 0 ? x : y;
  }
  bool operator==(const Vector2d& other) const
  {
    return x == other.x && y == other.y;
  }
  bool operator<(const Vector2d& other) const
  {
    return x < other.x || (x == other.x && y < other.y);
  }
  double getX() const
  {
    return x;
  }
  double getY() const
  {
    return y;
  }

  /*geometry_msgs::Point toPoint() const
  {
    geometry_msgs::Point p;
    p.x = x;
    p.y = y;
    p.z = 0;
    return p;
  }*/

  Vector2d& set(double x, double y)
  {
    Vector2d::x = x;
    Vector2d::y = y;
    return *this;
  }
  Vector2d& setX(double x)
  {
    Vector2d::x = x;
    return *this;
  }

  Vector2d& setY(double y)
  {
    Vector2d::y = y;
    return *this;
  }

  Vector2d& incX(double inc_x)
  {
    x += inc_x;
    return *this;
  }

  Vector2d& incY(double inc_y)
  {
    y += inc_y;
    return *this;
  }

  Vector2d& inc(double inc_x, double inc_y)
  {
    x += inc_x;
    y += inc_y;
    return *this;
  }

  const Angle angle() const
  {
    return Angle::fromRadian(std::atan2(y, x));
  }

  Angle angleTo(const Vector2d& other) const
  {
    return other.angle() - angle();
  }

  double squaredNorm() const
  {
    return x * x + y * y;
  }

  double norm() const
  {
    return std::sqrt(squaredNorm());
  }

  double dot(const Vector2d& other) const
  {
    return x * other.x + y * other.y;
  }

  Vector2d& normalize()
  {
    double n = norm();
    if (n > 0)
    {
      x /= n;
      y /= n;
    }
    return *this;
  }

  Vector2d normalized() const
  {
    Vector2d v(*this);
    v.normalize();
    return v;
  }

  Vector2d& operator*=(double scalar)
  {
    x *= scalar;
    y *= scalar;
    return *this;
  }
  Vector2d operator*(double scalar) const
  {
    return Vector2d(x * scalar, y * scalar);
  }

  Vector2d& operator/=(double scalar)
  {
    x /= scalar;
    y /= scalar;
    return *this;
  }
  Vector2d operator/(double scalar) const
  {
    return Vector2d(x / scalar, y / scalar);
  }

  Vector2d leftNormalVector() const
  {
    return Vector2d(-y, x);
  }

  Vector2d rightNormalVector() const
  {
    return Vector2d(y, -x);
  }

  Vector2d& operator+=(const Vector2d& other)
  {
    set(x + other.x, y + other.y);
    return *this;
  }
  Vector2d operator+(const Vector2d& other) const
  {
    return Vector2d(x + other.x, y + other.y);
  }
  Vector2d& operator-=(const Vector2d& other)
  {
    set(x - other.x, y - other.y);
    return *this;
  }
  Vector2d operator-(const Vector2d& other) const
  {
    return Vector2d(x - other.x, y - other.y);
  }
  Vector2d operator-() const
  {
    return Vector2d(-x, -y);
  }

  static const Vector2d& Zero()
  {
    static Vector2d zero;
    return zero;
  }

private:
  double x;
  double y;
};
}  // namespace utils

inline utils::Vector2d operator*(double scalar, const utils::Vector2d& v)
{
  utils::Vector2d w(v);
  w *= scalar;
  return w;
}

namespace std
{
inline ostream& operator<<(ostream& stream, const utils::Vector2d& v)
{
  stream << "(" << v.getX() << "," << v.getY() << ")";
  return stream;
}

template <>
struct hash<utils::Vector2d>
{
  size_t operator()(const utils::Vector2d& v) const
  {
    using boost::hash_combine;
    using boost::hash_value;
    std::size_t seed = 0;
    hash_combine(seed, hash_value(v[0]));
    hash_combine(seed, hash_value(v[1]));
    return seed;
  }
};
}  // namespace std

#endif
