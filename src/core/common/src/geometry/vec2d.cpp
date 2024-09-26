/**
 * *********************************************************
 *
 * @file: vector2d.cpp
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

#include <cmath>
#include "common/geometry/vec2d.h"
#include "common/util/log.h"

namespace rmp
{
namespace common
{
namespace geometry
{
Vec2d Vec2d::createUnitVec2d(const double angle)
{
  return Vec2d(std::cos(angle), std::sin(angle));
}

double Vec2d::length() const
{
  return std::hypot(x_, y_);
}

double Vec2d::lengthSquare() const
{
  return x_ * x_ + y_ * y_;
}

double Vec2d::angle() const
{
  return std::atan2(y_, x_);
}

void Vec2d::normalize()
{
  const double l = length();
  if (l > kMathEpsilon)
  {
    x_ /= l;
    y_ /= l;
  }
}

double Vec2d::distanceTo(const Vec2d& other) const
{
  return std::hypot(x_ - other.x_, y_ - other.y_);
}

double Vec2d::distanceSquareTo(const Vec2d& other) const
{
  const double dx = x_ - other.x_;
  const double dy = y_ - other.y_;
  return dx * dx + dy * dy;
}

double Vec2d::crossProd(const Vec2d& other) const
{
  return x_ * other.y() - y_ * other.x();
}

double Vec2d::innerProd(const Vec2d& other) const
{
  return x_ * other.x() + y_ * other.y();
}

Vec2d Vec2d::rotate(const double angle) const
{
  return Vec2d(x_ * cos(angle) - y_ * sin(angle), x_ * sin(angle) + y_ * cos(angle));
}

void Vec2d::selfRotate(const double angle)
{
  double tmp_x = x_;
  x_ = x_ * cos(angle) - y_ * sin(angle);
  y_ = tmp_x * sin(angle) + y_ * cos(angle);
}

Vec2d Vec2d::operator+(const Vec2d& other) const
{
  return Vec2d(x_ + other.x(), y_ + other.y());
}

Vec2d Vec2d::operator-(const Vec2d& other) const
{
  return Vec2d(x_ - other.x(), y_ - other.y());
}

Vec2d Vec2d::operator*(const double ratio) const
{
  return Vec2d(x_ * ratio, y_ * ratio);
}

Vec2d Vec2d::operator/(const double ratio) const
{
  CHECK_GT(std::abs(ratio), kMathEpsilon);
  return Vec2d(x_ / ratio, y_ / ratio);
}

Vec2d& Vec2d::operator+=(const Vec2d& other)
{
  x_ += other.x();
  y_ += other.y();
  return *this;
}

Vec2d& Vec2d::operator-=(const Vec2d& other)
{
  x_ -= other.x();
  y_ -= other.y();
  return *this;
}

Vec2d& Vec2d::operator*=(const double ratio)
{
  x_ *= ratio;
  y_ *= ratio;
  return *this;
}

Vec2d& Vec2d::operator/=(const double ratio)
{
  CHECK_GT(std::abs(ratio), kMathEpsilon);
  x_ /= ratio;
  y_ /= ratio;
  return *this;
}

bool Vec2d::operator==(const Vec2d& other) const
{
  return (std::abs(x_ - other.x()) < kMathEpsilon && std::abs(y_ - other.y()) < kMathEpsilon);
}

Vec2d operator*(const double ratio, const Vec2d& vec)
{
  return vec * ratio;
}

}  // namespace geometry
}  // namespace common
}  // namespace rmp
