/***********************************************************
 *
 * @file: point.h
 * @breif: point template class
 * @author: Yang Haodong
 * @update: 2024-9-20
 * @version: 1.0
 *
 * Copyright (c) 2023, Yang Haodong
 * All rights reserved.
 * --------------------------------------------------------
 *
 **********************************************************/
#ifndef RMP_COMMON_GEOMETRY_POINT_H_
#define RMP_COMMON_GEOMETRY_POINT_H_

#include <cmath>
#include <vector>

#include "common/util/log.h"

namespace rmp
{
namespace common
{
namespace geometry
{
template <typename T>
class Point2_
{
public:
  Point2_(T x = 0, T y = 0) : x_(x), y_(y){};
  ~Point2_() = default;

  T x() const
  {
    return x_;
  };

  T y() const
  {
    return y_;
  };

  void setX(T x)
  {
    x_ = x;
  }

  void setY(T y)
  {
    y_ = y;
  }

  T operator[](size_t i) const
  {
    return i == 0 ? x_ : y_;
  }

  bool operator==(const Point2_& other)
  {
    return (x_ == static_cast<T>(other.x())) && (y_ == static_cast<T>(other.y()));
  }

  bool operator!=(const Point2_& other)
  {
    return !operator==(other);
  }

public:
  static const int dim = 2;

private:
  T x_, y_;
};

template <typename T>
class Point3_
{
public:
  Point3_(T x = 0, T y = 0, T theta = 0) : x_(x), y_(y), theta_(theta){};
  ~Point3_() = default;

  T x() const
  {
    return x_;
  };

  T y() const
  {
    return y_;
  };

  T theta() const
  {
    return theta_;
  };

  void setX(T x)
  {
    x_ = x;
  }

  void setY(T y)
  {
    y_ = y;
  }

  void setTheta(T theta)
  {
    theta_ = theta;
  }

  T operator[](size_t i) const
  {
    return i == 0 ? x_ : (i == 1 ? y_ : theta_);
  }

  bool operator==(const Point3_& other)
  {
    return (x_ == static_cast<T>(other.x())) &&
           (y_ == static_cast<T>(other.y()) && (theta_ == static_cast<T>(other.theta())));
  }

  bool operator!=(const Point3_& other)
  {
    return !operator==(other);
  }

public:
  static const int dim = 3;

private:
  T x_, y_, theta_;
};

using Point2i = Point2_<int>;
using Point2f = Point2_<float>;
using Point2d = Point2_<double>;

using Points2i = std::vector<Point2i>;
using Points2f = std::vector<Point2f>;
using Points2d = std::vector<Point2d>;

using Point3i = Point3_<int>;
using Point3f = Point3_<float>;
using Point3d = Point3_<double>;

using Points3i = std::vector<Point3i>;
using Points3f = std::vector<Point3f>;
using Points3d = std::vector<Point3d>;

}  // namespace geometry
}  // namespace common
}  // namespace rmp

#endif